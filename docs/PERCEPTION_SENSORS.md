# Perception Sensors

Updated Nov 16 2026. The mission is to fly from point A to point B while avoiding still and moving obstacles. The drone is P1's 7" on 4S with a Jetson Orin Nano Super (P2's decision 0002). The suite and its layout come from P1's dodge analysis (Nov 7 2026) and hardware decision log (Nov 6 2026), with the Waveshare D500 in place of the STL-27L to save about \$174.

| Job | Pick | Connects to | Reasoning |
| :---- | :---- | :---- | :---- |
| Forward depth, path obstacles, SLAM (P-1 to P-5) | OAK-D Lite, fixed focus | Jetson USB 3 | a. passes every hard number b. 61 g c. depth computed on the camera |
| Side and rear threats (P-13) | 2 x IMX219-200 | Jetson's two CSI ports (22-pin to 15-pin cables) | a. about 160° x 120° each b. with the OAK-D Lite, a 3D view over 69% of directions and no gap behind c. same cost as the 160° lenses, so no third camera needed |
| 360° lidar plane (P-13, phase 1; phase 1 SLAM) | Waveshare D500 (LDROBOT STL-19P) | Jetson USB, through its CP2102 adapter board (UART at 230,400) | a. 10 Hz, 0.03–12 m, 0.72° between readings, so a 7 cm ball shows at about 1.9 m b. works in the dark c. 45 g, 1.45 W d. about \$174 less than the STL-27L |
| Downward flow \+ height (P-7) | MicoAir MTF-01P | Flight controller TELEM2, MAVLink at 115,200 baud | a. flow \+ 12 m rangefinder b. holds the drone when SLAM loses tracking |
| Optional: navigation outdoors | GPS | Flight controller GPS1 | GPS-denied by design, so not needed indoors |
| Optional: objects falling straight down | Upward rangefinder | A spare flight-controller port | Covers the small blind cap above the drone |

**Notes:**

- OAK-D Lite has no IR projector. Blank surfaces break both depth and SLAM tracking, and a projector would only fix depth. See P-12. Upgrade if depth proves a problem in testing: OAK-D Pro, fixed focus.
- The OAK-D Lite's colour camera isn't used.

**Forward camera**

| Check  | Requirement | OAK-D Lite |
| :---- | :---- | :---- |
| Price | budget target \~\$1,200-1,500 CAD for the drone | \$376 CAD (Luxonis page, Nov 4); P1 lists about \$210-240 CAD, to confirm |
| Mass | P-10 | 61 g |
| Power | P-10 | 3.5 W expected; 5 W only if AI or colour encoding runs |
| HFOV x VFOV | P-5: 70o x 50o | 73o x 58o, tilted up 8o |
| Range | P-1: \>= 5 m | ideal 0.8-12 m |
| Accuracy at 3 m | P-2 \<= 0.10 m | \< 2% \~ 0.06 m |
| Min depth | P-3 \<= 0.4 m | 0.2 m |
| Shutter | P-9: global preferred  | Global (stereo pair); rolling (colour) |
| Active stereo | P-9: preferred | No |
| Depth computed | on camera saves the companion computer | On camera |
| IMU | useful for SLAM | BMI270, up to 250 Hz |
| Baseline | used in P-2/P-3 math | 7.5 cm |

**Notes:**

- Avoid Kickstarter-era OAK-D Lite units as they have no IMU; get the fixed-focus variant.
- SLAM input: left mono image + depth aligned to left mono.

**Side cameras**

| Check | Requirement | IMX219-200 (x 2) |
| :---- | :---- | :---- |
| Price | | To check |
| Mass | P-10 | To weigh |
| Field of view | P-5 | 200° diagonal, about 160° x 120° (estimate; measure it) |
| Mounting | P-5 | On the mast, 100 mm above the props and 22 mm either side of it, 15° rearward, tilted up 25°, on adjustable mounts. P1 gave 16 mm, which leaves no room for the 25 mm board beside the mast once it's tilted |
| Resolution and rate | P-6 | 1640 x 1232 at 30 fps |
| Shutter | P-9 | Rolling, so fast objects skew |
| Detection | P-13 | Looming (growth between frames) on the Jetson's GPU: a 7 cm ball at about 3.5 m at 1640 wide, 1.8 m at 820, 0.9 m at 410 |
| Compute | P-6 | Must run on the GPU at full width: on the CPU it takes about 213 ms a frame per camera on a Jetson core (`bench/pi5`). At 410 wide it fits on the CPU (0.6 cores) but the range drops to 0.9 m |

**Notes:**

- In CAD, each side camera sees about 27% of a rear prop disc in the lower part of its view and the edge of the lidar at the top. Tilting the cameras up about 34° instead of 25° clears the props (`cad/skeleton.py`), at the cost of less view below the horizon. Check both on the real mount with the props fitted; P-5 counts only what the cameras can actually see.
- The OAK-D Lite at P1's position sees about 3% of each front prop disc along the top edge of its view; mask it in software. Keeping the props fully out of view would need the camera about 98 mm below the props, too close to the ground.

**Lidar**

| Check | Requirement | Waveshare D500 (STL-19P) |
| :---- | :---- | :---- |
| Price | | About \$134 CAD (\$174 less than the STL-27L at \$308, per P2's decision 0001) |
| Mass | P-10 | 45 g |
| Size | | 54 x 46.29 x 35 mm |
| Power | P-10 | 1.45 W |
| Coverage | P-5 | 360°, one flat plane 138 mm above the props, nothing above it |
| Angular step | P-4 | 0.72° (5,000 points/s, about 500 a turn at 10 Hz) |
| Range | P-13 | 0.03–12 m; a 7 cm ball at about 1.9 m, a 20 cm ball at about 5.3 m |
| Rate | P-6 | 10 Hz |

**Notes:**

- Confirm LDROBOT's ROS 2 driver for the STL-19P runs on Jazzy.
- The coarser 0.72° step lowers the lidar-only small-object limit in P-13 (7 cm: 1 m/s instead of 3 m/s). Inside the cameras' views it doesn't matter, because the cameras detect and the lidar only measures distance. The STL-27L is the upgrade if lidar-only flight needs more.
- P2's phase 1 SLAM uses this scan for ICP odometry; about 500 points a scan is sparser than the STL-27L's 2,160.

**Downward optical flow \+ rangefinder**

| Check  | Requirement | MicoAir MTF-01P |
| :---- | :---- | :---- |
| Price | | To check (the MTF-01 was about \$46 CAD) |
| Mass | P-10 | About 8 g (to weigh) |
| Flow min light  | P-9: fly at \>= 200 lux | \> 60 lux |
| Flow min height | P-7: hover \>= 0.3 m | 8 cm |
| Rangefinder range | P-7: 0.1 \- 4 m | 0.02 \- 12 m |
| Rate | P-7: \>= 20 Hz | 100 Hz |
| Connection to PX4 | \-  | TELEM2, MAVLink, 115,200 baud |

**Notes:**

- MTF-01P setup: newer AUTO units detect the protocol themselves; on older units, set mav\_px4 in MicoAssistant. Either way, set the flight controller's serial port and enable flow and the rangefinder in PX4.
- Mounted under the rear-right arm, 50 mm aft and 42 mm right, facing down: 90 mm off the ground when landed, and its view clears the battery.

**Mass and power budget**

| Item  | Mass (g)  | Power (W) | Power peak (W) | Cost (CAD) |
| :---- | :---- | :---- | :---- | :---- |
| OAK-D Lite | 61 | 3.5 | 5 | 376 |
| 2 x IMX219-200 | To weigh | To measure | To measure | To check |
| D500 | 45 | 1.45 | 1.45 | ~134 |
| MicoAir MTF-01P | \~ 8 | 0.5 | 0.5 | ~57 |
| Mounts \+ cables | \~ 15 | \- | \- | \- |
| Total so far | \~ 129 | \~ 5.5 | \~ 7 | \- |
| **P-10 limit** | **120** | **5 typical** | **Regulator 10 W+** | **\-** |

**NOTES**
- The OAK-D Lite price was checked Nov 4, 2026, at about 1.40 CAD per USD. Excludes tax, shipping and duties.
- The total is already over 120 g and 5 W typical before the side cameras are counted, so P-10 still needs the team to agree.
