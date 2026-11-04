# Perception Requirements

Owners: Perception & Sensing (Alex, Amartya). Day 1 deliverable (P3): mission turned into perception requirements.

## Assumptions

1. Indoor and outdoor, max altitude 3 m, GPS-denied.
2. Max transit speed v_max = 1.5 m/s (1.0 m/s nominal).
3. Reference camera for the math: 640 px wide, 70° HFOV, 7.5 cm stereo baseline, so f ≈ 457 px.
4. Braking a = 2 m/s², reaction time t_r = 0.3 s, safety margin d_m = 0.5 m.

## Requirements

| ID | Requirement | Value | Derived from | Verify in sim |
|----|-------------|-------|--------------|---------------|
| P-1 | Obstacle detection range | ≥ 5.0 m | d_stop = v·t_r + v²/2a + d_m = 1.51 m, ×2 for planning | Wall world: log first-detection distance |
| P-2 | Depth accuracy | σ_z ≤ 0.10 m at 3 m | σ_z ≈ z²/(f·B) · σ_d ≈ 0.066 m | Compare depth to ground truth |
| P-3 | Minimum depth range | ≤ 0.4 m | Z_min = f·B/d_max ≈ 0.36 m | Box approached until it drops out |
| P-4 | Smallest obstacle | 5 cm wide at 3 m (no wires, glass, mirrors) | n = f·w/Z ≈ 7.6 px | Pole world |
| P-5 | Coverage | a. Forward depth only, HFOV ≥ 70°, VFOV ≥ 50°<br>b. Yaw to face direction of travel<br>c. Flight-path angle ≤ ±15°<br>d. Vertical moves only through mapped-free space<br>e. Takeoff/land at pre-checked spots | Path angle ≤ VFOV/2 − 10° margin = 25° − 10° = 15°. Forward-only replaces 270° coverage (mass/cost) | a. FOV cones in CAD<br>b. Beam/overhang worlds |
| P-6 | Rate and latency | Depth ≥ 15 Hz, frame-to-command ≤ 150 ms | t_r ≈ 1/15 + 0.15 + control ≈ 0.3 s | `ros2 topic hz`, timestamps |
| P-7 | Height and hold | a. Rangefinder 0.1–4 m at ≥ 20 Hz<br>b. Optical flow ≥ 20 Hz<br>c. Hover ≥ 0.3 m<br>d. Textured floor | a. h_max + 1 m = 3 + 1 = 4 m<br>c. Flow footprint w = 2h·tan 35° ≈ 0.42 m at 0.3 m | a. Rangefinder vs truth<br>b. 60 s hover drift |
| P-9 | Lighting | ≥ 200 lux, no direct sun, exposure ≤ 6 ms; global shutter + active stereo preferred | Motion blur b = f·v·t_exp/Z ≤ 3 px → t_exp ≈ 6.6 ms | Lighting variation tests |
| P-10 | Sensor budget | ≤ 120 g, ≤ 5 W (peak <= 5.5 W), regulator sized with at least 2x headroom (excluding companion computer) | 950 g AUW | Mass budget |
| P-11 | Fault flags | a. Depth older than 0.5 s → hover<br>b. Flow/rangefinder invalid → land | Safety: v·0.5 s + d_stop = 2.26 m < 3 m | Inject dropout in sim |
| P-12 | Flying room | Walls: posters, printed patterns or coloured tape at camera height; busy and high-contrast, not repeating. Floor: textured, with a mat or tape if plain. Lighting per P-9. | SLAM and stereo need real features; optical flow needs floor texture | Check the room before flying |

**NOTES**
- P-8 (AprilTag) was removed. The other IDs keep their numbers
- Check the regulator: P1’s pick is a 5 A BEC for the Pi. If the camera is powered through the Pi, that BEC also has to cover the Pi’s own draw
- Measure it: when the camera arrives, check the real draw with a USB power meter
- If SLAM tracking lost then hold on flow and rangefinder, then land


## What must be detected, per mission phase

| Mission phase | What the drone must know | What must be detected |
|---------------|--------------------------|-----------------------|
| Takeoff | Height above ground, no obstacle overhead | Ground distance |
| Transit | Obstacles ahead | Obstacles |
| Landing | Height above ground | Ground distance |

## Perception pipeline

1. **Capture:** grab images, depth frames, or rangefinder readings, each with an accurate timestamp.
2. **Clean up:** undistort using the camera calibration, filter noise, crop out the drone's own legs and props.
3. **Detect:** find depth points that are too close.
4. **Estimate:** compute each obstacle's distance and direction from the camera.
5. **Transform:** convert camera-frame results into the world frame so the planner can use them.
