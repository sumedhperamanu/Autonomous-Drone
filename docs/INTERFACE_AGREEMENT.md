# Interface Agreement

Draft, Nov 10 2026. Sensors match `docs/DECISION_LOG.md`. Needs P2's topics and final CAD positions.

## Connections

From P1's hardware decision log.

| Device | Connects to | Link |
|---|---|---|
| OAK-D Lite | Jetson USB 3 | ROS 2 driver on the Jetson |
| 2 x IMX219-200 | Jetson CSI ports (22-pin to 15-pin cables) | Both CSI ports used, so any further camera must be USB |
| STL-27L | Jetson USB, through a UART adapter | ROS 2 driver on the Jetson |
| Flight controller TELEM1 | Jetson 40-pin header UART | ROS 2 over uXRCE-DDS, 921600 baud |
| Flight controller USB | Jetson | MAVLink forwarded to QGC |
| MTF-01P | Flight controller TELEM2 | MAVLink, 115200 baud |
| ELRS receiver | Flight controller RC | |
| GPS (optional) | Flight controller GPS1 | |
| SiK radio | Flight controller URT6 / TELEM3 | Keep one spare port for the optional upward rangefinder |

## Frames

```
map -> odom -> base_link -> oakd_link -> oakd_left_optical
                         -> side_left_link
                         -> side_right_link
                         -> lidar_link
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

## Sensor topics

Sim rates were measured on Nov 9 2026, in sim time from message timestamps, on `p3/perception-main`.

| Topic | Type | Rate | Owner |
|---|---|---|---|
| `/depth_camera/image` | `sensor_msgs/Image` (32FC1, 640 x 480) | 30 Hz | P3 |
| `/depth_camera/points` | `sensor_msgs/PointCloud2` | 30 Hz | P3 |
| `/depth_camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | P3 |
| `/oakd/left/image` | `sensor_msgs/Image` (mono8, 640 x 480) | 30 Hz | P3 |
| `/oakd/left/camera_info` | `sensor_msgs/CameraInfo` (fx 432.5 px) | 30 Hz | P3 |
| `/oakd/imu` | `sensor_msgs/Imu` | 250 Hz | P3 |
| `/side_left/image`, `/side_right/image` | `sensor_msgs/Image` (rgb8, 820 x 616 in sim, 1640 x 1232 real) | 30 Hz | P3 |
| `/scan` | `sensor_msgs/LaserScan` (2160 points, 0.167° apart) | 10 Hz | P3 |
| `/range/scan` (sim checks only) | `sensor_msgs/LaserScan` | 50 Hz | P3 |
| `/clock` (sim only) | `rosgraph_msgs/Clock` | | P3 |

QoS: subscribe to the camera topics with reliable. Best effort dropped depth from 30 Hz to about 4.5 Hz. Subscribe to `/fmu/out` topics with best effort.

On the real drone, the MTF-01P connects to PX4 over UART, so flow and range don't go through ROS 2. In the sim, PX4 also reads the lidar directly as `obstacle_distance` (72 sectors of 5°); on the real drone the lidar connects to the Jetson.

## Perception outputs (proposed)

| Topic | Type | Rate | Owner |
|---|---|---|---|
| `/perception/obstacles` | Tracked obstacles: position, velocity, size and covariance, in `odom`. Message type to agree; nothing standard carries velocity | ≥ 10 Hz | P3 |

## PX4 and SLAM topics

| Topic | Type | Rate | Owner |
|---|---|---|---|
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | About 80 Hz real time, in sim | P2 |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | To confirm | P2 |
| RTAB-Map outputs | To confirm | To confirm | P2 |

## Static transforms

From P1's layout, through `cad/skeleton.py`. `base_link` sits 36 mm below the prop plane, as in the sim.

| Parent -> child | x y z (m) | roll pitch yaw (rad) |
|---|---|---|
| `base_link` -> `oakd_link` | 0.0750 0.0000 -0.0140 | 0 -0.1396 0 |
| `oakd_link` -> `oakd_left_optical` | 0.0000 0.0375 0.0000 | -1.5708 0 -1.5708 |
| `base_link` -> `side_left_link` | 0.0000 0.0160 0.1360 | 0 -0.4363 1.8326 |
| `base_link` -> `side_right_link` | 0.0000 -0.0160 0.1360 | 0 -0.4363 -1.8326 |
| `base_link` -> `lidar_link` | 0.0000 0.0000 0.1740 | 0 0 0 |
| `base_link` -> `flow_link` | -0.0500 -0.0420 0.0140 | 0 0 0 |
| `base_link` -> `lidar_sensor_link` | -0.0500 -0.0420 0.0140 | 0 1.5708 0 |

Negative pitch tilts a camera up. The flow sensor uses P1's height here; `cad/` finds that inside the rear-right arm and places it 19 mm lower, at z -0.0050.

## Known differences to fix

- The sim's camera and IMU messages use the frame ID `oakd_link`, not `oakd_left_optical`, and the side cameras use Gazebo's own names. P2 and P3 need to fix this before RTAB-Map runs.
- The sim's mono and depth views share one point, while the real left lens sits 37.5 mm to the side.
- In the sim, the lidar's link is called `link` and its sensor `lidar_2d_v2`, because PX4 only reads that name. In ROS 2 it is `/scan`.
- The sim's default world has a plain floor and sky, so visual SLAM needs a textured world that follows P-12.
- The sim's depth camera still works on blank walls. The real OAK-D Lite doesn't.
