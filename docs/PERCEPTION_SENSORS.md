# Perception Sensors

Updated Nov 9 2026 for moving obstacles and person tracking. The suite and its layout come from P1's dodge analysis (Nov 7 2026).

| Job | Pick | Reasoning |
| :---- | :---- | :---- |
| Forward depth, obstacles and people (P-1 to P-5, P-14) | OAK-D Lite, fixed focus | a. passes every hard number b. 61 g c. \$376 CAD d. depth and the person detector both run on the camera |
| Side and rear threats (P-13) | 2 x IMX219-200 | a. about 160° x 120° each b. with the OAK-D Lite, a 3D view over 69% of directions and no gap behind c. same cost as the 160° lenses, so no third camera needed |
| 360° lidar plane (P-13, phase 1) | STL-27L | a. 0.167° between readings, so a 7 cm ball shows at about 8 m (1.7 m with the LD19) b. 10 Hz, 25 m c. works in the dark d. same footprint as the LD19 |
| Downward flow \+ height (P-7) | MicoAir MTF-01P | a. flow \+ 12 m rangefinder b. MAVLink to PX4 c. holds the drone when SLAM loses tracking |

**Notes:**

- OAK-D Lite has no IR projector. Blank surfaces break both depth and SLAM tracking, and a projector would only fix depth. See P-12. Upgrade if depth proves a problem in testing: OAK-D Pro, fixed focus.
- The drone also carries GPS (P1's layout) for outdoor flights. It isn't used indoors.

**Forward camera**

| Check  | Requirement | OAK-D Lite |
| :---- | :---- | :---- |
| Price | assuming \~\$700 total | \$376 CAD |
| Mass | P-10 | 61 g |
| Power | P-10 | 3.5 W expected; up to 5 W with the person detector running |
| HFOV x VFOV | P-5: 70o x 50o | 73o x 58o, tilted up 8o |
| Range | P-1: \>= 5 m | ideal 0.8-12 m |
| Accuracy at 3 m | P-2 \<= 0.10 m | \< 2% \~ 0.06 m |
| Min depth | P-3 \<= 0.4 m | 0.2 m |
| Shutter | P-9: global preferred  | Global (stereo pair); rolling (colour) |
| Active stereo | P-9: preferred | No |
| Depth computed | on camera saves the companion computer | On camera |
| Person detection | P-14 | On the camera's AI chip, with depth for 3D position |
| IMU | useful for SLAM | BMI270, up to 250 Hz |
| Baseline | used in P-2/P-3 math | 7.5 cm |

**Notes:**

- Avoid Kickstarter-era OAK-D Lite units as they have no IMU; get the fixed-focus variant.
- SLAM input: left mono image + depth aligned to left mono. The colour camera is used for person detection, not SLAM.

**Side cameras**

| Check | Requirement | IMX219-200 (x 2) |
| :---- | :---- | :---- |
| Price | | To check |
| Mass | P-10 | To weigh |
| Field of view | P-5 | 200° diagonal, about 160° x 120° (estimate; measure it) |
| Mounting | P-5 | On the mast, 15° rearward, tilted up 25° |
| Resolution and rate | P-6 | 1640 x 1232 at 30 fps |
| Shutter | P-9 | Rolling, so fast objects skew |
| Detection | P-13 | Looming (growth between frames): a 7 cm ball at about 3.5 m, 2.8 m at 1280 wide |

**Notes:**

- In the sim, the props show at the bottom edge of the side cameras' view, and the lidar body at the top edge. Check both on the real mount; P-5 counts only what the cameras can actually see.

**Lidar**

| Check | Requirement | STL-27L |
| :---- | :---- | :---- |
| Price | | To check |
| Mass | P-10 | About the LD19's 47 g (to weigh) |
| Coverage | P-5 | 360°, one flat plane 138 mm above the props |
| Angular step | P-4 | 0.167° (21,600 points/s at 10 Hz) |
| Range | P-13 | 25 m max; a 7 cm ball at about 8 m |
| Rate | P-6 | 10 Hz |

**Downward optical flow \+ rangefinder**

| Check  | Requirement | MicoAir MTF-01P |
| :---- | :---- | :---- |
| Price | | To check (the MTF-01 was about \$46 CAD) |
| Mass | P-10 | About 5 g (to weigh) |
| Flow min light  | P-9: fly at \>= 200 lux | \> 60 lux |
| Flow min height | P-7: hover \>= 0.3 m | 8 cm |
| Rangefinder range | P-7: 0.1 \- 4 m | 0.02 \- 12 m |
| Rate | P-7: \>= 20 Hz | 100 Hz |
| Connection to PX4 | \-  | UART, MAVLink |

**Notes:**

- MTF-01P setup: newer AUTO units detect the protocol themselves; on older units, set mav\_px4 in MicoAssistant. Either way, set the flight controller's serial port and enable flow and the rangefinder in PX4.

**Mass and power budget**

| Item  | Mass (g)  | Power (W) | Power peak (W) | Cost (CAD) |
| :---- | :---- | :---- | :---- | :---- |
| OAK-D Lite | 61 | 3.5 | 5 | 376 |
| 2 x IMX219-200 | To weigh | To measure | To measure | To check |
| STL-27L | \~ 47 | To measure | To measure | To check |
| MicoAir MTF-01P | \~ 5 | 0.5 | 0.5 | To check |
| Mounts \+ cables | \~ 15 | \- | \- | \- |
| Total so far | \~ 128 | \- | \- | \- |
| **P-10 limit** | **To re-set** | **To re-set** | **To re-set** | **\-** |

**NOTES**
- The OAK-D Lite price was checked Nov 4, 2026, at about 1.40 CAD per USD. Excludes tax, shipping and duties.
- The budget is already over the old 120 g limit before the side cameras are weighed, so P-10 has to be re-set with the team.
