# Perception sensors in sim

P1's 7" drone on 4S (`sim/seveninch`, airframe 22054) with the perception sensors of the Jetson build, placed as in `cad/skeleton.py` on `p3/interface-agreement`, for checking `docs/PERCEPTION_REQUIREMENTS.md`: flying from A to B while avoiding still and moving obstacles.

| Sensor | Real part | Sim resolution | Field of view | Rate | Range | ROS 2 topic |
| --- | --- | --- | --- | --- | --- | --- |
| Forward depth | OAK-D Lite | 640 x 480 | 73°, tilted up 8° | 30 Hz | 0.2 to 12 m | `/depth_camera/image`, `/depth_camera/points` |
| Forward mono | OAK-D Lite | 640 x 480 | 73°, tilted up 8° | 30 Hz | | `/oakd/left/image` |
| Camera IMU | OAK-D Lite | | | 250 Hz | | `/oakd/imu` |
| Side cameras | 2 x IMX219-200 | 820 x 616 (real 1640 x 1232) | 160° fisheye, on the mast 22 mm either side, 15° rearward, tilted up 25° | 30 Hz | | `/side_left/image`, `/side_right/image` |
| 360° lidar | Waveshare D500 (STL-19P) | 500 points, 0.72° apart | 360°, 138 mm above the props | 10 Hz | 0.03 to 12 m | `/scan`; PX4 also reads it as `obstacle_distance` |
| Downward flow | MTF-01P | 100 x 100 | 42° | 50 Hz | | read by PX4 |
| Downward range | MTF-01P | | | 50 Hz | 0.02 to 12 m | `/range/scan` |

## Run it

`install.sh` installs P1's drones first, then this model, so run it after pulling.

```bash
bash sim/perception/install.sh
cd ~/PX4-Autopilot && export GALLIUM_DRIVER=d3d12 && make px4_sitl gz_seveninch_perception
```

In a second terminal, start the bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/Autonomous-Drone/sim/perception/bridge.yaml
```

Subscribe to the camera topics with reliable QoS. With best effort, ROS 2 drops most of these large frames.

## Moving-obstacle world

`worlds/dodge.sdf` has a 7 cm ball 6 m ahead, a 20 cm ball 6 m to the left and a person-sized block (`walker`, 0.5 m wide and 1.7 m tall) 8 m ahead and 3 m to the right.

```bash
cd ~/PX4-Autopilot && PX4_GZ_WORLD=dodge make px4_sitl gz_seveninch_perception
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

## Results (Nov 16, WSL)

| Check | Result |
| --- | --- |
| OAK-D Lite depth, mono | 30.3 Hz sim time, 640 x 480 |
| OAK-D Lite IMU | 250 Hz sim time |
| Side cameras | 30.3 Hz sim time, 820 x 616 |
| Lidar | 10.0 Hz sim time, 500 points, 0.721° apart |
| Range | 50.0 Hz sim time |
| Flow | Quality 255 |
| PX4 ready | 14 s after starting (the first start after `install.sh` rebuilt PX4 didn't get there; the second did) |
| Takeoff with no GPS | Holds 2.50 m in PX4's estimate, 2.39 m on the rangefinder |
| Sim speed | About 3% of real time with the bridge and both side cameras running, on a laptop with a CAD build running alongside. Rates in sim time stay correct; give it time |

Not tested yet: P-1 to P-4 and P-13 detection ranges, frame-to-command latency in P-6, and P-11 fault handling. The Jetson's compute is checked with `bench/pi5`.

If PX4 doesn't reach `Ready for takeoff` on the first start after `install.sh` rebuilds it, start it again.

## What differs from the real sensors

- The side cameras run at half resolution to keep the sim usable. Set 1640 x 1232 in `model.sdf` when testing detection range, since P1's ranges assume it.
- Positions come from the CAD layout and nobody has measured the built drone yet. The sensor links weigh 1 g each; set P1's body mass to the weighed drone (the CAD estimates 1,476 g with a 567 g battery). P1 also still has to move the sim's motors from 295 to 320 mm apart.
- The MTF-01P sits under the rear-right arm, lens 40.5 mm below the props; P1's 22 mm below the props is inside the arm.
- Depth noise is a constant 0.06 m (the datasheet's 2% at 3 m). Real stereo error grows with distance squared.
- The depth camera doesn't fail on plain walls the way a passive stereo camera does.
- The side cameras don't skew fast objects, though the real ones are rolling shutter.
- The flow and range sensors run at 50 Hz, not 100. That still passes P-7.
