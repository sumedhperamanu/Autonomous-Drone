# Perception sensors in sim

P1's 5" drone (`sim/fiveinch`) with the perception sensors, placed as in `cad/skeleton.py`, for checking `docs/PERCEPTION_REQUIREMENTS.md`: flying from A to B while avoiding still and moving obstacles.

| Sensor | Real part | Sim resolution | Field of view | Rate | Range | ROS 2 topic |
| --- | --- | --- | --- | --- | --- | --- |
| Forward depth | OAK-D Lite | 640 x 480 | 73°, tilted up 8° | 30 Hz | 0.2 to 12 m | `/depth_camera/image`, `/depth_camera/points` |
| Forward mono | OAK-D Lite | 640 x 480 | 73°, tilted up 8° | 30 Hz | | `/oakd/left/image` |
| Camera IMU | OAK-D Lite | | | 250 Hz | | `/oakd/imu` |
| 360° lidar | STL-27L | 2160 points | 360°, 73 mm above the props on a rear mast | 10 Hz | 0.03 to 25 m | `/scan`; PX4 also reads it as `obstacle_distance` |
| Downward flow | MTF-01P | 100 x 100 | 42° | 50 Hz | | read by PX4 |
| Downward range | MTF-01P | | | 50 Hz | 0.02 to 12 m | `/range/scan` |

## Run it

`install.sh` installs P1's drones first, then this model, so run it after pulling.

```bash
bash sim/perception/install.sh
cd ~/PX4-Autopilot && export GALLIUM_DRIVER=d3d12 && make px4_sitl gz_fiveinch_perception
```

In a second terminal, start the bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/Autonomous-Drone/sim/perception/bridge.yaml
```

Subscribe to the camera topics with reliable QoS. With best effort, ROS 2 drops most of these large frames.

## Moving-obstacle world

`worlds/dodge.sdf` has a 7 cm ball 6 m ahead, a 20 cm ball 6 m to the left and a person-sized block (`walker`, 0.5 m wide and 1.7 m tall) 8 m ahead and 3 m to the right.

```bash
cd ~/PX4-Autopilot && PX4_GZ_WORLD=dodge make px4_sitl gz_fiveinch_perception
```

The bridge file's lidar and range paths name the world, so make a copy for this one:

```bash
sed "s#/world/default/#/world/dodge/#" ~/Autonomous-Drone/sim/perception/bridge.yaml > /tmp/bridge_dodge.yaml
```

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/tmp/bridge_dodge.yaml
```

Send an object at the drone. It keeps that speed until you send a new one, so send 0 to stop it:

```bash
gz topic -t /model/ball_7cm/cmd_vel -m gz.msgs.Twist -p 'linear: {x: -2.0}'
```

```bash
gz topic -t /model/ball_7cm/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0}'
```

Use `ball_20cm` with a negative `y` to send it from the left, and `walker` with a negative `x` to walk it in.

## Results (Nov 14, WSL)

| Check | Result |
| --- | --- |
| OAK-D Lite depth, mono | 30.3 Hz sim time, 640 x 480 |
| OAK-D Lite IMU | 247.8 Hz sim time |
| Lidar | 10.0 Hz sim time, 2160 points, 0.167° apart |
| Range | 50.0 Hz sim time |
| Flow | Quality 255 |
| Takeoff with no GPS | Holds 2.50 m in PX4's estimate, 2.43 m on the rangefinder |
| `dodge` world | Lidar sees the walker at 8.3 m, and keeps it as it walks past and behind the drone |
| Sim speed | Real time with nothing subscribed; about 40% of real time with the bridge running |

Not tested yet: P-1 to P-4 and P-13 detection ranges, frame-to-command latency in P-6, and P-11 fault handling.

If PX4 doesn't reach `Ready for takeoff` on the first start after `install.sh` rebuilds it, start it again: the second start has worked every time.

## What differs from the real sensors

- Positions come from the CAD layout, and nobody has measured the built drone yet. The sensor links weigh 1 g each; P1's `fiveinch_base` mass (0.819 kg) should become the weighed drone, 0.835 kg from the CAD.
- Depth noise is a constant 0.06 m (the datasheet's 2% at 3 m). Real stereo error grows with distance squared.
- The depth camera doesn't fail on plain walls the way a passive stereo camera does.
- The flow and range sensors run at 50 Hz, not 100. That still passes P-7.
- The sim drone sits on P1's model's legs, so the flow sensor is 87 mm off the ground there, against 84 mm in the CAD.
