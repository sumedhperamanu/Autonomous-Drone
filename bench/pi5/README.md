# Companion computer compute check

Can the drone's companion computer run its perception? First run Nov 15 2026 for a Raspberry Pi 5; updated Nov 16 2026 for the 7" build's Jetson Orin Nano Super (P2's decision 0002), with the two side cameras added.

**Short answer for the Jetson: the CPU handles depth, lidar, SLAM and the rest with cores to spare, but side-camera detection at full resolution only fits if it runs on the GPU.** On the CPU, our motion and looming detection takes about 213 ms a frame at the real 1,640 px width, which is about 13 cores for both cameras. P2's budget gives the side cameras 0.3–0.5 cores, which assumes the GPU. That GPU version isn't written or measured yet, so it's the main thing to check on Day 6.

We haven't run any of this on a real board yet. The numbers come from running the drone's perception work on a laptop against the sim, scaled by 3.4 (the Jetson's Cortex-A78AE cores and the Pi 5's Cortex-A76 cores score almost the same, 778 and 774 in Geekbench 6 single-core, against about 2,600 for the laptop), plus published measurements of RTAB-Map on a Pi 5.

## Files

| File | What it does |
| --- | --- |
| `perception_load.py` | Runs and times the per-frame work: depth to obstacles to a dodge decision, lidar to obstacles to a dodge decision, side-camera motion and looming detection, and a stand-in for the SLAM front end. Reports each sensor's detection range at the settings used. It reads live ROS 2 topics (sim or real drivers), or loops `sim_clip.npz` with no sensors at all |
| `sysmon.py` | Records CPU use per process group (SLAM, OAK-D driver, lidar driver, XRCE agent, perception, other), the CPU temperature and, on a Pi, its throttling and under-voltage flags. On the Jetson, also run `tegrastats` for the GPU and power |
| `record_clip.py` | Records a new clip from the sim or the real sensors (edit its topic names for the real drivers) |
| `sim_clip.npz` | From the 7" sim's `dodge` world with the walker and the 20 cm ball moving: 40 frames of depth (16-bit mm, like the real OAK-D) and mono, 30 frames from each side camera (820 x 616), and 13 D500 scans |

## Jetson Orin Nano Super, 7" build (Nov 16)

Measured on the laptop against the 7" sim's `dodge` world with the walker and the 20 cm ball moving, then on the recorded clip. Pinned to one core, times per frame projected to one Jetson core. The drone's half-width is 0.202 m.

| Pipeline | Setting | Time per frame, mean / p95 | Jetson cores at full rate | 7 cm object seen at | 20 cm |
| --- | --- | --- | --- | --- | --- |
| Depth to dodge decision | 640 x 480, 30 Hz | 24–45 / 32–57 ms | 0.7–1.4 | 5.3 m | 12 m |
| Lidar to dodge decision | D500, 0.72°, 10 Hz | 1–2 / 1.5–3.6 ms | 0.02 | 1.8 m | 5.3 m |
| Side cameras, CPU | 1,640 px wide (real), 2 x 30 Hz | 213 / 227 ms | about 13 | 3.5 m | 10 m |
| | 820 px wide (sim), 2 x 30 Hz | 52–61 / 58–76 ms | 3.1–3.6 | 1.8 m | 5 m |
| | 410 px wide, 2 x 30 Hz | 11 / 15 ms | 0.6 | 0.9 m | 2.5 m |
| SLAM front-end stand-in | 320 x 240, 30 Hz | 3–4 / 4–5 ms | 0.1 | | |

The ranges follow P1's dodge-analysis rules: about 7 pixels across for stereo, 10 for side-camera looming and 3 lidar readings. The spread in each row comes from running the same settings live (with Gazebo on the same laptop) and on the clip.

What this means:

- **Side cameras need the GPU.** At the real resolution, the CPU version is far too slow. Running motion detection on the GPU (OpenCV CUDA or NVIDIA VPI) is the plan in P2's decision 0002; until it's measured, P2's 0.3–0.5 cores for the side cameras is an assumption. If the GPU version falls short, cutting the side cameras to about 410 px wide fits on the CPU, but then a 7 cm ball from the side shows at only about 0.9 m, which lowers the sides' P-13 limit well below 2 m/s.
- **Depth at 640 x 480 is close to its 33 ms budget on one core.** It passes on the clip and misses with Gazebo running alongside. Split it across two threads, move it to C++ or the GPU, or use 320 x 240 for close obstacles. 640 x 480 is what gives the 5.3 m range for a 7 cm ball ahead.
- **Everything else fits.** Lidar, the SLAM stand-in and the estimates for the drivers and SLAM come to about 2–2.5 of the Jetson's 6 cores, in line with P2's table.

## Earlier check: Raspberry Pi 5 (Nov 15)

This is the run that showed the Pi 5 fits only a reduced setup, which is why P2's decision 0002 picked the Jetson.

**Pi 5: very likely yes, at 320 x 240 and with the settings below, before the side cameras. It can't run SLAM on full 640 x 480 images at 30 Hz.**

## Estimated Pi 5 budget

Four cores in total. "Measured" rows ran on a Ryzen AI 7 445 and are scaled by 3.4, the ratio of the two chips' Geekbench 6 single-core scores (about 2,600 against the Pi 5's 774).

| Job | Setting | Pi 5 cores | Where the number comes from |
| --- | --- | --- | --- |
| Obstacle perception: depth + lidar + dodge decision | Depth 320 x 240 at 30 Hz, lidar at 10 Hz | 0.35–0.41 | Measured, live against the sim |
| Receiving depth and mono images in Python | 640 x 480 32-bit depth at 30 Hz | 0.45–0.65 | Measured, live against the sim |
| | 320 x 240 16-bit depth, made on the camera | About 0.1–0.15 | Scaled from the row above by image size (estimate) |
| SLAM odometry (RTAB-Map, F2M, GFTT/ORB) | 320 x 240 at 15 Hz | About 0.6 (38 ms a frame) | Published Pi 5 measurement |
| SLAM map update | Once a second | 0.1–0.4 (91–357 ms an update) | Published Pi 5 measurement, worst case with unbounded memory |
| OAK-D driver | Depth + mono | 0.1–0.3 | Estimate; one report shows 3% of a core while streaming |
| Lidar driver, XRCE agent | | 0.1–0.2 | Estimate |
| Planner, mission, offboard control | | 0.1–0.2 | Estimate |
| Linux, ROS 2, logging | No raw image recording in flight | About 0.2 | Estimate |
| **Total** | | **About 1.8–2.9 of 4** | Leaves 1.1–2.2 cores spare |

If the Pi turns out 5 times slower than the laptop instead of 3.4, the measured rows grow by half and the total reaches about 3.3 cores. That still runs, with little spare.

## Measured on the laptop, projected to a Pi 5

Live against the `dodge` world, 45 s each, pinned to one core (`taskset`). Times are per frame, projected to the Pi 5.

| Setting | Depth to decision, mean / p95 | Lidar to decision | SLAM front-end stand-in | Python receive cost per frame |
| --- | --- | --- | --- | --- |
| Depth 320 x 240, front end 640 x 480 | 11.6 / 15.0 ms, pass | 1.2 / 1.9 ms, pass | 37.0 / 55.6 ms, **fail** | 22 ms |
| Depth 640 x 480, front end 640 x 480 | 24.7 / 33.1 ms, just passes | 0.7 / 1.7 ms, pass | 46.5 / 57.9 ms, **fail** | 15 ms |
| Depth 320 x 240, front end 320 x 240 | 13.6 / 17.8 ms, pass | 0.5 / 0.7 ms, pass | 11.1 / 12.9 ms, pass | 19 ms |

Pass marks: depth and the SLAM front end finish within one 30 Hz frame (33.3 ms at p95), and the lidar decision within P-6's 50 ms. Replaying `sim_clip.npz` gives the same picture.

The SLAM stand-in only detects, matches and solves a pose. Real RTAB-Map does more, so use the published numbers for SLAM. The sim's images also have far fewer features than a real room, which makes the stand-in look cheaper than it would be in real use.

## What the published Pi 5 measurements say

From [RGB-D SLAM on the Edge: Benchmarking and Profiling RTAB-Map and ORB-SLAM3 on the Raspberry Pi 5](https://arxiv.org/html/2611.16472) (arXiv preprint, Nov 12 2026, not peer-reviewed). It used a Pi 5 with the Active Cooler, Ubuntu 24.04 and RTAB-Map 0.23 from ROS 2 Jazzy, on TUM RGB-D datasets rather than a live camera.

- RTAB-Map odometry: about 75 ms a frame at 640 x 480 and 38 ms at 320 x 240. So at most about 13 Hz at full size and about 26 Hz at 320 x 240.
- RTAB-Map's map update ran on every frame there, which is a worst case. The default is once a second.
- RTAB-Map kept "little more than one core busy" (26–39% of all four cores).
- ORB-SLAM3 tracking was faster: 45 / 29 / 13 ms a frame at 640 / 320 / 160 wide.
- No thermal throttling with the Active Cooler; peak 58–59 °C.
- Under-voltage flags appeared on a marginal supply. Our BEC must hold 5 V under load.

## Settings this budget depends on

1. Feed SLAM 320 x 240 images, and run its odometry at 10–15 Hz with the map update at 1 Hz. Bound its memory (`Rtabmap/TimeThr` or `Rtabmap/MemoryThr`) so long flights don't slow it down.
2. Make the depth image smaller on the OAK-D itself (its on-camera decimation or the 400P mode). Don't send full-size depth to the Pi.
3. Don't subscribe to full-size point clouds (`/depth_camera/points`) in flight, and don't record raw images to a bag in flight.
4. Keep obstacle detection separate from SLAM, as the requirements already say: dodges use depth and lidar directly, so a slow SLAM frame never delays a dodge. If SLAM loses tracking, P-11d holds on flow.
5. Fit the Active Cooler and give the Pi a steady 5 V, 5 A. Set `usb_max_current_enable=1` for the OAK-D.
6. PX4's external-vision guide asks for a higher rate than 15 Hz. Keep the MTF-01P's flow fused for velocity, and check EKF2 with 15 Hz SLAM in the sim.

Latency for P-6: from a depth frame arriving to a dodge decision takes about 20–40 ms on the Pi 5 (receiving plus processing). For the lidar it's a few milliseconds. Both are inside P-6's 50 ms.

## On the real board

On the Jetson, use the same commands with `--decimate 1` (the full-settings case in P2's 0002) and the side cameras on. Run `tegrastats` alongside `sysmon.py`.

### Raspberry Pi 5

Ubuntu 24.04 and ROS 2 Jazzy on the Pi 5, with the Active Cooler fitted and a proper 5 V, 5 A supply.

```bash
sudo apt install python3-numpy python3-scipy python3-opencv
```

**1. Bare Pi, no sensors.** Run these two at the same time, in two terminals, from this folder:

```bash
python3 perception_load.py --replay sim_clip.npz --duration 120 --decimate 2 --vo-half --report replay.json
```

```bash
python3 sysmon.py --duration 110 --report sysmon_replay.json
```

Then the hard case, full-size depth:

```bash
python3 perception_load.py --replay sim_clip.npz --duration 60 --decimate 1 --vo-half --report replay_640.json
```

**2. With the OAK-D Lite and STL-27L connected.** Start the camera driver, the lidar driver, `MicroXRCEAgent` and RTAB-Map at 320 x 240. Then run the benchmark on the real topics. Check the real names with `ros2 topic list`; the ones below are an example:

```bash
python3 perception_load.py --duration 120 --depth /oak/stereo/image_raw --info /oak/stereo/camera_info --mono /oak/left/image_raw --scan /scan --decimate 1 --vo-half --report live.json
```

```bash
python3 sysmon.py --duration 110 --report sysmon_live.json
```

```bash
ros2 topic hz /odom
```

Use `--decimate 1` here if the camera already sends 320 x 240 depth.

**Pass if:**

| Check | Pass mark |
| --- | --- |
| `perception_load.py` result | `"pass": true` (depth and SLAM front end p95 ≤ 33.3 ms, lidar ≤ 50 ms, and replay keeps up at 30 Hz) |
| `sysmon.py`, everything running | At least 0.75 of a core spare on average |
| CPU temperature | Under 85 °C, after 10 minutes running |
| Throttling | `throttled_flags` is `0x0`: no throttling and no under-voltage |
| SLAM odometry | `/odom` at 10 Hz or more |

If any of these fail, first make the images smaller, then drop SLAM odometry to 10 Hz. If they still fail, the next options are ORB-SLAM3 instead of RTAB-Map, moving the perception node to C++, or a Jetson Orin Nano.

## How the laptop numbers were made

Rebuild `sim_clip.npz` from the `dodge` world with `record_clip.py`, or record your own clip from the real camera (edit its topic names) to make the replay more realistic. The live runs used the sim from `sim/perception` with the bridge, the walker moving at 0.6 m/s and a ball at 0.8 m/s, with the benchmark pinned to one core. The Jetson runs used:

```bash
taskset -c 11 python3 perception_load.py --duration 45 --scale 3.4 --decimate 1 --vo-half
```
