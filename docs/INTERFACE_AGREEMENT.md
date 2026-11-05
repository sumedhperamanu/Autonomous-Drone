# Interface Agreement

Draft, Nov 5 2026. Needs P2's topics and final CAD positions. Sensors match `docs/DECISION_LOG.md`.

## Frames

```
map -> odom -> base_link -> oakd_link -> oakd_left_optical
                         -> flow_link
                         -> lidar_sensor_link
```

| System | World | Body |
|---|---|---|
| ROS 2 | ENU (East, North, Up) | FLU (Forward, Left, Up) |
| PX4 | NED (North, East, Down) | FRD (Forward, Right, Down) |

- ENU to NED: (x, y, z) -> (y, x, -z)
- FLU to FRD: (x, y, z) -> (x, -y, -z)
- Camera optical frame: X right, Y down, Z forward.
- Convert only in the node that talks to PX4. Everything else in ROS 2 uses ENU and FLU.

## Topics

Sim rates were measured on Nov 5 2026, in sim time from message timestamps.

| Topic | Type | Rate | Owner |
|---|---|---|---|
| `/depth_camera/image` | `sensor_msgs/Image` (32FC1, 640 x 480) | 30 Hz | P3 |
| `/depth_camera/points` | `sensor_msgs/PointCloud2` | 30 Hz | P3 |
| `/depth_camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | P3 |
| `/oakd/left/image` | `sensor_msgs/Image` (mono8, 640 x 480) | 30 Hz | P3 |
| `/oakd/left/camera_info` | `sensor_msgs/CameraInfo` (fx 432.5 px) | 30 Hz | P3 |
| `/oakd/imu` | `sensor_msgs/Imu` | 250 Hz | P3 |
| `/range/scan` (sim checks only) | `sensor_msgs/LaserScan` | 50 Hz | P3 |
| `/clock` (sim only) | `rosgraph_msgs/Clock` | | P3 |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | About 80 Hz real time, in sim | P2 |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | To confirm | P2 |
| RTAB-Map outputs | To confirm | To confirm | P2 |

QoS: subscribe to the camera topics with reliable. Best effort dropped depth from 30 Hz to about 4.5 Hz. Subscribe to `/fmu/out` topics with best effort.

On the real drone, the MTF-01 connects to PX4 over UART, so flow and range don't go through ROS 2.

## Static transforms

From `cad/skeleton.py`. These are proposed until the team picks the mount and checks the estimates in `cad/README.md`.

| Parent -> child | x y z (m) | roll pitch yaw (rad) |
|---|---|---|
| `base_link` -> `oakd_link` | 0.0700 0.0000 -0.0500 | 0 0 0 |
| `oakd_link` -> `oakd_left_optical` | 0.0000 0.0375 0.0000 | -1.5708 0 -1.5708 |
| `base_link` -> `flow_link` | 0.0200 0.0000 -0.0255 | 0 0 0 |
| `base_link` -> `lidar_sensor_link` | 0.0200 0.0000 -0.0255 | 0 1.5708 0 |

## Known differences to fix

- The sim's camera and IMU messages use the frame ID `oakd_link`, not `oakd_left_optical`. P2 and P3 need to fix this before RTAB-Map runs.
- The sim's x500 mounts the camera at (0.12, 0.03, 0.002), not the position above. Its mono and depth views also share one point, while the real left lens sits 37.5 mm to the side.
- The sim's default world has a plain floor and sky, so visual SLAM needs a textured world that follows P-12.
- The sim's depth camera still works on blank walls. The real OAK-D Lite doesn't.
