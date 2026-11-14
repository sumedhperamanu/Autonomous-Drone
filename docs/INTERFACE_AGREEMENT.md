# Interface Agreement

Draft, Nov 14 2026, for the 5" build. Sensors match `docs/DECISION_LOG.md`. Needs P2's topics and the measured positions from the built drone.

## Connections

| Device | Connects to | Link |
|---|---|---|
| OAK-D Lite | Raspberry Pi 5 USB 3 | ROS 2 driver on the Pi. Set `usb_max_current_enable=1` |
| STL-27L | Raspberry Pi 5 USB, through a UART adapter | ROS 2 driver on the Pi |
| Flight controller TELEM1 | Raspberry Pi 5 GPIO UART | ROS 2 over uXRCE-DDS, 921600 baud |
| Flight controller USB | Raspberry Pi 5 | MAVLink forwarded to QGC over Wi-Fi; there's no SiK radio |
| MTF-01P | Flight controller TELEM2 | MAVLink, 115200 baud |
| ELRS receiver | Flight controller RC | |
| GPS (optional) | Flight controller GPS1 | |
| Spare port | Flight controller | Keep one for the optional upward rangefinder |

## Frames

```
map -> odom -> base_link -> oakd_link -> oakd_left_optical
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

Sim rates were measured on Nov 14 2026, in sim time from message timestamps, on `p3/perception-main`.

| Topic | Type | Rate | Owner |
|---|---|---|---|
| `/depth_camera/image` | `sensor_msgs/Image` (32FC1, 640 x 480) | 30 Hz | P3 |
| `/depth_camera/points` | `sensor_msgs/PointCloud2` | 30 Hz | P3 |
| `/depth_camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | P3 |
| `/oakd/left/image` | `sensor_msgs/Image` (mono8, 640 x 480) | 30 Hz | P3 |
| `/oakd/left/camera_info` | `sensor_msgs/CameraInfo` (fx 432.5 px) | 30 Hz | P3 |
| `/oakd/imu` | `sensor_msgs/Imu` | 250 Hz | P3 |
| `/scan` | `sensor_msgs/LaserScan` (2160 points, 0.167° apart) | 10 Hz | P3 |
| `/range/scan` (sim checks only) | `sensor_msgs/LaserScan` | 50 Hz | P3 |
| `/clock` (sim only) | `rosgraph_msgs/Clock` | | P3 |

QoS: subscribe to the camera topics with reliable. Best effort dropped depth from 30 Hz to about 4.5 Hz. Subscribe to `/fmu/out` topics with best effort.

On the real drone, the MTF-01P connects to PX4 over UART, so flow and range don't go through ROS 2. In the sim, PX4 also reads the lidar directly as `obstacle_distance` (72 sectors of 5°); on the real drone the lidar connects to the Pi.

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

From `cad/skeleton.py`, and the same in the sim model `sim/perception/models/fiveinch_perception`. `base_link` sits 26.9 mm below the prop plane, as in P1's `fiveinch` model.

| Parent -> child | x y z (m) | roll pitch yaw (rad) |
|---|---|---|
| `base_link` -> `oakd_link` | 0.0700 0.0000 -0.0351 | 0 -0.1396 0 |
| `oakd_link` -> `oakd_left_optical` | 0.0000 0.0375 0.0000 | -1.5708 0 -1.5708 |
| `base_link` -> `lidar_link` | -0.0620 0.0000 0.0999 | 0 0 0 |
| `base_link` -> `flow_link` | -0.0630 0.0000 -0.0146 | 0 0 0 |
| `base_link` -> `lidar_sensor_link` | -0.0630 0.0000 -0.0146 | 0 1.5708 0 |

Negative pitch tilts a camera up. `oakd_link` is the centre of the OAK-D Lite's front face, `lidar_link` is the scan plane and `flow_link` is the MTF-01P's lens face.

## Known differences to fix

- The sim's camera and IMU messages use the frame ID `oakd_link`, not `oakd_left_optical`. P2 and P3 need to fix this before RTAB-Map runs.
- The sim's mono and depth views share one point, while the real left lens sits 37.5 mm to the side.
- In the sim, the lidar's link is called `link` and its sensor `lidar_2d_v2`, because PX4 only reads that name. In ROS 2 it is `/scan`.
- The sim's default world has a plain floor and sky, so visual SLAM needs a textured world that follows P-12.
- The sim's depth camera still works on blank walls. The real OAK-D Lite doesn't.
