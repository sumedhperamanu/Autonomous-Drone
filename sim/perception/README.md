# Perception sensors in sim

PX4's x500 with the sensors picked in `docs/PERCEPTION_SENSORS.md`, for checking `docs/PERCEPTION_REQUIREMENTS.md`.

| Sensor | Real part | Resolution | HFOV | Rate | Range | ROS 2 topic |
| --- | --- | --- | --- | --- | --- | --- |
| Forward depth | OAK-D Lite | 640 x 480 | 73 deg | 30 Hz | 0.2 to 12 m | `/depth_camera/image`, `/depth_camera/points` |
| Downward flow | MTF-01 | 100 x 100 | 42 deg | 50 Hz | | read by PX4 |
| Downward range | MTF-01 | | | 50 Hz | 0.02 to 8 m | `/range/scan` |

## Run it

```bash
bash sim/perception/install.sh
cd ~/PX4-Autopilot && make px4_sitl gz_x500_perception
```

In a second terminal:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/Autonomous-Drone/sim/perception/bridge.yaml
```

In a third:

```bash
ros2 topic hz /depth_camera/image
```

Subscribe to the depth topics with reliable QoS. With best effort, ROS 2 drops most of these large frames (we got about 4.5 Hz, not 30).

On WSL, run `export GALLIUM_DRIVER=d3d12` before `make` so Gazebo uses the GPU. Lower real time speed is fine: PX4 waits for Gazebo, and rates in sim time stay correct.

## Results (Nov 3, default world, WSL)

| Check | Result |
| --- | --- |
| Depth image | 640 x 480, 30.3 Hz sim time, fx 432.5 px (matches 73 deg) |
| Depth camera view | No legs or props in the frame |
| Range | 49.6 Hz sim time, 0.177 m on the ground |
| Flow | Quality 255 |
| Takeoff with no GPS | Holds 2.50 m in PX4's estimate, 2.35 m on the rangefinder |
| Sim speed | About half of real time |

Not tested yet: P-1 to P-4 (needs walls and obstacles), frame-to-command latency in P-6, and P-11 fault handling.

## What differs from the real sensors

- The mounts are placeholders until we have CAD. The depth camera sits where PX4's `x500_depth` puts it.
- Depth noise is a constant 0.06 m (the datasheet's 2% at 3 m). Real stereo error grows with distance squared.
- The depth camera doesn't fail on plain walls the way a passive stereo camera does.
- The flow and range sensors run at 50 Hz, not 100. That still passes P-7.
- The x500 is heavier than our drone. This model checks the sensors, not the flight performance.
