# Perception Requirements

Owners: Perception & Sensing (Alex, Amartya). Updated Nov 14 2026 for the 5" build. The mission is to fly from point A to point B while avoiding still and moving obstacles. The drone is a 5" on 6S (team requirements: under 950 g, 4:1 thrust-to-weight, 7+ min hover); the layout comes from `cad/skeleton.py` on `p3/interface-agreement`.

## Assumptions

1. Indoor and outdoor, max altitude 3 m. GPS-denied by design; GPS is optional for outdoor flights.
2. Max transit speed v_max = 1.5 m/s (1.0 m/s nominal). Dodges are allowed to go faster (P-13).
3. Obstacles can be still or moving: balls, people and other drones, from any direction.
4. Sensors: OAK-D Lite forward under the nose, STL-27L 2D lidar on a rear mast, MTF-01P flow and range under the frame. The side cameras are a later upgrade.
5. Reference camera for the depth math: 640 px wide, 70° HFOV, 7.5 cm stereo baseline, so f ≈ 457 px.
6. Braking a = 2 m/s², reaction time t_r = 0.3 s, safety margin d_m = 0.5 m.

## Requirements

| ID | Requirement | Value | Derived from | Verify in sim |
|----|-------------|-------|--------------|---------------|
| P-1 | Obstacle detection range | ≥ 5.0 m for still obstacles; moving obstacles per P-13 | d_stop = v·t_r + v²/2a + d_m = 1.51 m, ×2 for planning | Wall world: log first-detection distance |
| P-2 | Depth accuracy | σ_z ≤ 0.10 m at 3 m | σ_z ≈ z²/(f·B) · σ_d ≈ 0.066 m | Compare depth to ground truth |
| P-3 | Minimum depth range | ≤ 0.4 m | Z_min = f·B/d_max ≈ 0.36 m | Box approached until it drops out |
| P-4 | Smallest obstacle | 7 cm at the ranges in P-13 (no wires, glass, mirrors) | About 7 px across for stereo, 3 lidar readings | `dodge` world, `ball_7cm` |
| P-5 | Coverage | a. 3D depth ahead: OAK-D Lite 73° × 58°, tilted up 8°<br>b. 360° lidar plane 73 mm above the props, clear of the battery<br>c. Known blind spots: everything outside the forward view and off the lidar plane, so above and below the drone at the sides and rear<br>d. Drone faces its direction of travel, so the OAK-D Lite covers the path<br>e. Vertical moves only through mapped-free space<br>f. Take off at A and land at B, both pre-checked spots | Team requirement: 270°+ horizontal sensing plus forward depth. Side cameras would add 3D coverage at the sides and rear later | a, b. FOV solids in CAD: no props in either view<br>c. Check on the built drone |
| P-6 | Rate and latency | Depth ≥ 30 Hz, lidar 10 Hz. Threat confirmed within 0.10 s (OAK-D), 0.20 s (lidar); decision within 0.05 s after that | P1's dodge model: total dodge time = sense + 0.05 s + start-up + move | `ros2 topic hz`, timestamps |
| P-7 | Height and hold | a. Rangefinder 0.1–4 m at ≥ 20 Hz<br>b. Optical flow ≥ 20 Hz<br>c. Hover ≥ 0.3 m<br>d. Textured floor | a. h_max + 1 m = 3 + 1 = 4 m<br>c. Flow footprint w = 2h·tan 35° ≈ 0.42 m at 0.3 m | a. Rangefinder vs truth<br>b. 60 s hover drift |
| P-9 | Lighting | ≥ 200 lux, no direct sun, exposure ≤ 6 ms; the lidar also works in the dark | Motion blur b = f·v·t_exp/Z ≤ 3 px → t_exp ≈ 6.6 ms | Lighting variation tests |
| P-10 | Sensor budget | ≤ 120 g total; ≤ 5 W typical; regulator sized for 10 W or more (P1's proposed wording) | OAK-D Lite 61 g, STL-27L 45 g, MTF-01P 8 g: 114 g before mounts. Whole drone about 835 g, 919 g with 10% margin, under the 950 g limit | Weigh the built drone |
| P-11 | Fault flags | a. Depth older than 0.5 s → hold on flow, land after 2 s (proposed by P1, to agree with P3)<br>b. Flow/rangefinder invalid → land<br>c. The OAK-D Lite fails or goes dark → dodge on the lidar alone, at the lidar-plane limits in P-13<br>d. SLAM loses tracking, such as after a hard dodge → hold on flow and rangefinder until it recovers | Safety: v·0.5 s + d_stop = 2.26 m < 3 m | Inject dropout in sim |
| P-12 | Flying room | Walls: posters, printed patterns or coloured tape at camera height; busy and high-contrast, not repeating. Floor: textured, with a mat or tape if plain. Lighting per P-9. | SLAM and stereo need real features; optical flow needs floor texture | Check the room before flying |
| P-13 | Moving obstacles | Dodge objects approaching at up to the speeds below, one at a time, from a hover. Dodge vertically by default (up first); sideways when there's no headroom or the object comes steeply. Clearance = drone half-size + object width | About half of P1's theoretical limits | `dodge` world: send each ball from each direction |

P-8 (AprilTag) was removed. The other IDs keep their numbers.

### P-13 limits

Approach speed for a dodge from a hover. Flying forward at v toward an oncoming object adds v to its closing speed.

| Object | Front (OAK-D Lite + lidar) | Sides and behind (lidar plane only) | Above or below the lidar plane, outside the forward view |
|---|---|---|---|
| 7 cm | 3 m/s | 3 m/s | Not covered (known limit) |
| 20 cm | 5 m/s | 5 m/s | Not covered (known limit) |

These come from P1's dodge analysis (front and phase 1 lidar-only columns), which modelled the 7"; the smaller 5" needs less sideways travel to clear, so they're on the safe side until P1 reruns the numbers. Adding the side cameras later covers the sides and rear above and below the lidar plane. Raise these once real tests show more headroom.

## What must be detected, per mission phase

| Mission phase | What the drone must know | What must be detected |
|---------------|--------------------------|-----------------------|
| Takeoff at A | Height above ground, no obstacle overhead | Ground distance |
| Transit to B | Obstacles on the path | Still obstacles ahead |
| Any phase | Whether anything is coming at the drone | Moving obstacles from any covered direction, with position, velocity and size |
| Landing at B | Height above ground, landing spot clear | Ground distance |

## Perception pipeline

1. **Capture:** grab depth frames, camera images, lidar scans and rangefinder readings, each with an accurate timestamp.
2. **Clean up:** undistort using the camera calibration, filter noise, crop out the drone's own legs, props and mast.
3. **Detect:** depth points that are too close (OAK-D Lite) and lidar clusters.
4. **Track:** subtract the drone's own motion, then follow each object over several frames to get its velocity.
5. **Estimate:** each obstacle's position, velocity and size. Where a camera and the lidar both see an object, use the lidar's distance.
6. **Transform:** convert sensor-frame results into the world frame so the planner can use them.
