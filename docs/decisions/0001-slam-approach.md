# 0001: SLAM approach

Date: 2026-11-04 · Owner: P2 · Status: proposed (freeze at Gate 1)

## Decision
RTAB-Map, RGB-D visual odometry with loop closure, using the OAK-D Lite's on-camera depth aligned to its left mono camera. The LD19 2D lidar scan is added as a second input.

Backup: Isaac ROS cuVSLAM on the Orin Nano Super's GPU, only if RTAB-Map's CPU load is too high on Days 6–8. cuVSLAM cannot run on team laptops or in CI (no NVIDIA GPU), so it is tested on the Jetson only.

## Sensors it serves
- **OAK-D Lite, fixed focus** (P3): stereo camera, depth computed on the camera. Stereo pair 640 × 480, global shutter. Colour camera (rolling shutter) not used for SLAM. BMI270 IMU, rate TBD (need ≥ 200 Hz). Mounted forward-facing. No IR projector.
- **LD19** (P3): 360° 2D lidar, 10 Hz, 0.02–12 m, on the companion's USB.
- **MTF-01P** (P1/P3): flow + range, goes to PX4 EKF2 over MAVLink, not into SLAM.

## Why
- Fits the sensor types: RTAB-Map takes RGB-D and lidar scans together.
- ROS 2 Jazzy packages install with apt.
- Runs without an NVIDIA GPU, so the same code runs on laptops, in CI and on the drone.
- On-camera depth means RTAB-Map does no stereo matching, which keeps CPU load down.
- Loop closure corrects drift when the drone revisits an area.

## Inputs and outputs
- In (topic names TBD, P3 to confirm from the Gazebo model):
  - Left mono image: `<topic>`
  - Depth aligned to left mono: `<topic>`
  - camera_info: `<topic>`
  - IMU (≥ 200 Hz): `<topic>`
  - LD19 scan: `<topic>`
  - All stamped with sim time (`use_sim_time:=true`).
- Out: `nav_msgs/Odometry` in frame `odom`; converted to PX4 `VehicleOdometry` on Day 8 using P3's frame rules.

## Compute needs
See 0002. SLAM estimate: 1.5–2.5 CPU cores, 1–2 GB memory. Measure on Day 6.

## Open questions
- **Phase 1 sensor:** P1's phase 1 build has the LD19 + flow but not the OAK-D Lite. Does phase 1 navigate on lidar-only SLAM? If yes, Day 6 starts with the lidar input. Owner: P1 + P3.
- **IMU in sim:** confirm the Gazebo camera publishes an IMU topic at ≥ 200 Hz. Owner: P3.

## Risks
- **Low-texture surfaces** (plain walls, floors) cause visual tracking loss. An IR projector would fix depth but not tracking. Mitigation: textured flying room (P-12), LD19 still sees blank walls, MTF-01P holds height and slows drift. OAK-D Pro is the named upgrade if depth proves a problem.
- **Moving obstacles** (a requirement) can corrupt the map and the pose estimate, because RTAB-Map assumes a static world. Mitigation: filter tracked moving objects out of SLAM inputs (P2, Days 6–8).
- **Lidar slice tilts** when the drone leans (10–20° seen in sim gusts), so the scan can hit floor or ceiling. Mitigation: filter the scan using attitude.
- **3D blind spots:** 3D view is forward only. Obstacles behind or beside are seen at drone height only. Mitigation: drone faces its direction of travel.
- **Rolling shutter** on the colour camera. Mitigation: colour camera not used for SLAM.
