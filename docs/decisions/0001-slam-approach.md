# 0001: SLAM approach

Date: 2026-11-04 · Updated: 2026-11-15 · Owner: P2 · Status: proposed (freeze at Gate 1)

## Decision
RTAB-Map in both phases, with the input changing as sensors are added:

- **Phase 1 (lidar + flow, no cameras):** RTAB-Map ICP odometry and mapping on the Waveshare D500 2D scan, with EKF2's flow-based odometry as the initial guess. Fallback if 2D ICP in RTAB-Map is hard to tune on the D500's sparser scan: `slam_toolbox`.
- **Phase 2 (cameras fitted):** add RGB-D input from the OAK-D Lite (on-camera depth aligned to its left mono camera) alongside the D500 scan.
Starting settings (from P3's compute benchmark, PR #8):
- SLAM images at **320 × 240**, made small on the OAK-D itself, not on the Jetson.
- Odometry at **10–15 Hz**, map update at **1 Hz**.
- Memory bounded with `Rtabmap/TimeThr` or `Rtabmap/MemoryThr`, so long flights don't slow it down.
- No full-size point clouds subscribed in flight, and no raw images recorded in flight.
Backup: Isaac ROS cuVSLAM on the Jetson's GPU (0002), if RTAB-Map's odometry is too slow or loses track too often during dodges on Days 6–8. cuVSLAM cannot run on team laptops or in CI (no NVIDIA GPU), so it is tested on the Jetson only.

## Sensors it serves
- **Waveshare D500** (P3, replaces the STL-27L to save ~$174): LDROBOT STL-19P inside. 360° 2D lidar, 10 Hz, 5,000 points/s (≤ 0.72° apart, ~500 points per scan), 0.03–12 m, 1.45 W. UART at 230400 through its CP2102 USB adapter board (micro-USB; a 20 cm USB-A cable comes in the kit) to a Jetson USB port. Mounted at the top of the centre mast, scan plane +138 mm above the props. Primary SLAM input in phase 1.
- **OAK-D Lite, fixed focus** (P3): stereo camera, depth computed on the camera. Stereo pair global shutter; SLAM gets 320 × 240. Colour camera (rolling shutter) not used for SLAM. BMI270 IMU, rate TBD (need ≥ 200 Hz). Under the nose, tilted up 8°, soft-mounted. No IR projector. SLAM input in phase 2.
- **2 × IMX219-200 side cameras** (P3): ~160° × 120°, on the mast, 15° rearward, tilted up 25°. Used for moving-obstacle detection, **not** as SLAM input.
- **MTF-01P** (P1/P3): flow + range, goes to PX4 EKF2 over MAVLink, not into SLAM directly; EKF2's output seeds the ICP odometry, and its flow stays fused for velocity.
## Why
- Same package in both phases: only the inputs change, so Day 6 work carries straight into phase 2.
- RTAB-Map takes 2D lidar scans and RGB-D together, and its odometry package supports ICP on lidar.
- ROS 2 Jazzy packages install with apt; already added to the dev container.
- Runs without an NVIDIA GPU, so the same code runs on laptops, in CI and on the drone.
- The lidar sees blank walls that defeat the cameras. The D500's scan is LD19-class (~500 points), enough for 2D ICP in a room with clear walls, though sparser than the STL-27L's ~2,160.
- On-camera depth means RTAB-Map does no stereo matching in phase 2.
- Loop closure corrects drift when the drone revisits an area. It can lag a second or two without affecting flight safety.
- Obstacle detection is separate from SLAM: dodges use depth and lidar directly, so a slow SLAM frame never delays a dodge.
## Inputs and outputs
- In (sim topic names from P3's benchmark; the real drivers' names will differ, check with `ros2 topic list`):
  - D500 scan (`sensor_msgs/LaserScan`): `/scan`
  - EKF2 odometry (initial guess): `<topic>` from the uXRCE-DDS bridge
  - Phase 2: left mono image `/oakd/left/image`, depth aligned to left mono `/depth_camera/image`, camera_info `/depth_camera/camera_info`, IMU (≥ 200 Hz) `<topic>`
  - All stamped with sim time (`use_sim_time:=true`).
- Out: `nav_msgs/Odometry` in frame `odom`; converted to PX4 `VehicleOdometry` on Day 8 using P3's frame rules.
## Sim
- Phase 1 starts on airframe 22053 (`gz_seveninch_lidar_flow`): lidar + flow, no GPS. Its lidar model must match the D500: 10 Hz, ~500 points per scan (0.72°), 12 m range.
- Phase 2 needs the OAK-D (and side cameras) added to the Gazebo model.
- Sim motor spacing to be updated from 295 mm to 320 mm (TBS Source One V5) by P1. P3's benchmark uses a 0.17 m drone radius (the 5" model); confirm the sim runs the 7" model (0.202 m axis, 0.249 m diagonal).
## Compute needs
See 0002. Published Pi 5 measurements (same per-core speed as the Jetson): RTAB-Map odometry takes ~38 ms a frame at 320 × 240 (~0.6 core at 15 Hz) and ~75 ms at 640 × 480 (at most ~13 Hz on either board). Map update at 1 Hz: 0.1–0.4 core. Measure on the Jetson on Day 6 with P3's `perception_load.py` and `sysmon.py`.

## Open questions
- **Phase 1 sensor:** proposed above (lidar + flow); P1 and P3 to confirm.
- **IMU in sim:** confirm the Gazebo camera publishes an IMU topic at ≥ 200 Hz. Owner: P3.
- **D500 ROS 2 driver:** confirm LDROBOT's driver for the STL-19P builds and runs on Jazzy. Owner: P3.
- **EKF2 with 15 Hz SLAM:** PX4's external-vision guide asks for a higher rate. Check in the sim that EKF2 stays stable with 15 Hz vision plus fused flow. Owner: P2.
## Risks
- **Sparser lidar scan:** ICP has ~500 points per scan to match instead of ~2,160, so phase 1 may drift more in plain or symmetric rooms. Mitigation: textured test room with clear walls; `slam_toolbox` fallback; the STL-27L ($308, ~$174 more) is the named upgrade.
- **Shorter lidar detection:** a 7 cm object is seen at ~1.9 m in the scan plane (~8 m with the STL-27L), which lowers phase 1's small-object speed limit to ~2 m/s (P-13).
- **Lidar slice tilts** when the drone leans (10–20° seen in sim gusts), so the scan can hit floor or ceiling. Mitigation: filter the scan using attitude; matters from day one in phase 1.
- **Phase 1 sees only one plane:** obstacles above or below the scan plane are invisible until the cameras are fitted. Mitigation: phase 1 tests in a room with clear vertical surfaces; P-13 phase 1 limits apply only at drone height.
- **Low-texture surfaces** (plain walls, floors) cause visual tracking loss in phase 2. An IR projector would fix depth but not tracking. Mitigation: textured flying room (P-12), the lidar still sees blank walls, MTF-01P holds height and slows drift. OAK-D Pro is the named upgrade if depth proves a problem.
- **Lower SLAM resolution** (320 × 240) gives fewer features and a less accurate pose than full size. Mitigation: measure drift in the sim; cuVSLAM backup on the GPU.
- **Moving obstacles** (a requirement, P-13) can corrupt the map and the pose estimate, because RTAB-Map assumes a static world. Mitigation: filter tracked moving objects out of SLAM inputs (P2, Days 6–8).
- **Hard dodges** (up to ~27° lean, fast climbs) will often make SLAM lose track briefly. Mitigation: hold on flow/EKF2 (P-11d) and let RTAB-Map relocalize.
- **Rolling shutter** on the OAK-D colour camera and the IMX219s. Mitigation: neither is used as SLAM input.
