# Perception Requirements

Owners: Perception & Sensing (Alex, Amartya). Updated Nov 10 2026. The mission is to fly from point A to point B while avoiding still and moving obstacles. Sensors and layout follow P1's dodge analysis (Nov 7 2026) and hardware decision log (Nov 6 2026).

## Assumptions

1. Indoor and outdoor, max altitude 3 m. GPS-denied by design; GPS is optional for outdoor flights.
2. Max transit speed v_max = 1.5 m/s (1.0 m/s nominal). Dodges are allowed to go faster (P-13).
3. Obstacles can be still or moving: balls, people and other drones, from any direction.
4. Sensors: OAK-D Lite forward, two IMX219-200 side cameras, STL-27L 2D lidar, MTF-01P flow and range, all placed as in P1's layout.
5. Reference camera for the depth math: 640 px wide, 70° HFOV, 7.5 cm stereo baseline, so f ≈ 457 px.
6. Braking a = 2 m/s², reaction time t_r = 0.3 s, safety margin d_m = 0.5 m.

## Requirements

| ID | Requirement | Value | Derived from | Verify in sim |
|----|-------------|-------|--------------|---------------|
| P-1 | Obstacle detection range | ≥ 5.0 m for still obstacles; moving obstacles per P-13 | d_stop = v·t_r + v²/2a + d_m = 1.51 m, ×2 for planning | Wall world: log first-detection distance |
| P-2 | Depth accuracy | σ_z ≤ 0.10 m at 3 m | σ_z ≈ z²/(f·B) · σ_d ≈ 0.066 m | Compare depth to ground truth |
| P-3 | Minimum depth range | ≤ 0.4 m | Z_min = f·B/d_max ≈ 0.36 m | Box approached until it drops out |
| P-4 | Smallest obstacle | 7 cm at the ranges in P-13 (no wires, glass, mirrors) | About 7 px across for stereo, 10 px for side-camera looming, 3 lidar readings | `dodge` world, `ball_7cm` |
| P-5 | Coverage | a. 3D camera view over about 69% of all directions, with no gap around the horizon<br>b. 360° lidar plane at the top of the drone<br>c. Known blind spots: below the drone (about 28%) and a small cap straight up (about 3%)<br>d. Drone faces its direction of travel, so the OAK-D Lite covers the path<br>e. Vertical moves only through mapped-free space<br>f. Take off at A and land at B, both pre-checked spots | P1's coverage model: OAK-D Lite 73° × 58° tilted up 8°; side cameras about 160° × 120° each, 15° rearward, tilted up 25° | a, b. FOV solids in CAD<br>c. Check props and the lidar body stay out of each view |
| P-6 | Rate and latency | Cameras ≥ 30 Hz, lidar 10 Hz. Threat confirmed within 0.10 s (OAK-D), 0.15 s (side cameras), 0.20 s (lidar); decision within 0.05 s after that | P1's dodge model: total dodge time = sense + 0.05 s + start-up + move | `ros2 topic hz`, timestamps |
| P-7 | Height and hold | a. Rangefinder 0.1–4 m at ≥ 20 Hz<br>b. Optical flow ≥ 20 Hz<br>c. Hover ≥ 0.3 m<br>d. Textured floor | a. h_max + 1 m = 3 + 1 = 4 m<br>c. Flow footprint w = 2h·tan 35° ≈ 0.42 m at 0.3 m | a. Rangefinder vs truth<br>b. 60 s hover drift |
| P-9 | Lighting | ≥ 200 lux, no direct sun, exposure ≤ 6 ms; the lidar also works in the dark | Motion blur b = f·v·t_exp/Z ≤ 3 px → t_exp ≈ 6.6 ms. The side cameras are rolling shutter, so fast objects skew | Lighting variation tests |
| P-10 | Sensor budget | ≤ 120 g total; ≤ 5 W typical; regulator sized for 10 W or more (P1's proposed wording) | OAK-D Lite 61 g, STL-27L 45 g, MTF-01P about 5 g: about 111 g before mounts and the side cameras | Weigh the built drone |
| P-11 | Fault flags | a. Depth older than 0.5 s → hold on flow, land after 2 s (proposed by P1, to agree with P3)<br>b. Flow/rangefinder invalid → land<br>c. A camera fails or goes dark → dodge on the lidar alone, at the phase 1 limits in P-13<br>d. SLAM loses tracking, such as after a hard dodge → hold on flow and rangefinder until it recovers | Safety: v·0.5 s + d_stop = 2.26 m < 3 m | Inject dropout in sim |
| P-12 | Flying room | Walls: posters, printed patterns or coloured tape at camera height; busy and high-contrast, not repeating. Floor: textured, with a mat or tape if plain. Lighting per P-9. | SLAM and stereo need real features; optical flow needs floor texture | Check the room before flying |
| P-13 | Moving obstacles | Dodge objects approaching at up to the speeds below, one at a time, from a hover. Dodge vertically by default (up first); sideways when there's no headroom or the object comes steeply. Clearance = drone half-size + object width | About half of P1's theoretical limits | `dodge` world: send each ball from each direction |

P-8 (AprilTag) was removed. The other IDs keep their numbers.

### P-13 limits

Approach speed for a dodge from a hover. Flying forward at v toward an oncoming object adds v to its closing speed.

| Object | Front | Sides, diagonals and behind | Phase 1 (lidar only, drone height) | Steeply above or below |
|---|---|---|---|---|
| 7 cm | 3 m/s | 2 m/s | 3 m/s | Not covered (known limit) |
| 20 cm | 5 m/s | 5 m/s | 5 m/s | Not covered (known limit) |

Raise these once real tests show more headroom.

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
3. **Detect:** depth points that are too close (OAK-D Lite), objects that grow between frames (side cameras) and lidar clusters.
4. **Track:** subtract the drone's own motion, then follow each object over several frames to get its velocity.
5. **Estimate:** each obstacle's position, velocity and size. Where a camera and the lidar both see an object, use the lidar's distance.
6. **Transform:** convert sensor-frame results into the world frame so the planner can use them.
