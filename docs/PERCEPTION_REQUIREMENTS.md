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
| P-8 | AprilTag | tag36h11, 18 cm black-border edge, search altitude 2.5 m, landing error ≤ 10 cm, tag pose error ≤ 5 cm at 1 m | Z_max ≈ f·s/20 px ≈ 4.1 m ≥ 3.75 m | Tag at increasing heights/angles (day 8 table) |
| P-9 | Lighting | ≥ 200 lux, no direct sun, exposure ≤ 6 ms; global shutter + active stereo preferred | Motion blur b = f·v·t_exp/Z ≤ 3 px → t_exp ≈ 6.6 ms | Lighting variation tests |
| P-10 | Sensor budget | ≤ 120 g, ≤ 5 W (excluding companion computer) | 950 g AUW | Mass budget |
| P-11 | Fault flags | a. Depth older than 0.5 s → hover<br>b. Flow/rangefinder invalid → land<br>c. Tag lost > 3 s → re-search | Safety: v·0.5 s + d_stop = 2.26 m < 3 m | Inject dropout in sim |

## What must be detected, per mission phase

| Mission phase | What the drone must know | What must be detected |
|---------------|--------------------------|-----------------------|
| Takeoff | Height above ground, no obstacle overhead | Ground distance |
| Search | Target visibility | Marker or pad, from search height |
| Transit | Obstacles ahead | Obstacles |
| Approach | Exactly where the target is relative to the drone | Marker pose, close range |
| Landing | Height, and whether still centered | Marker + ground |

## Perception pipeline

1. **Capture:** grab images, depth frames, or rangefinder readings, each with an accurate timestamp.
2. **Clean up:** undistort using the camera calibration, filter noise, crop out the drone's own legs and props.
3. **Detect:** find the AprilTag in the image, or depth points that are too close.
4. **Estimate:** compute position and size; the tag's pose relative to the camera is (R_ct, t_ct).
5. **Transform:** convert camera-frame results into the world frame so the planner can use them.
