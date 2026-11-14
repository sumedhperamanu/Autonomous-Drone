# Perception Sensors

Updated Nov 14 2026 for the 5" build. The mission is to fly from point A to point B while avoiding still and moving obstacles. Positions come from `cad/skeleton.py` on `p3/interface-agreement`; the full parts list with prices is in the team's 5" parts list PDF.

| Job | Pick | Connects to | Reasoning |
| :---- | :---- | :---- | :---- |
| Forward depth, path obstacles, SLAM (P-1 to P-5) | OAK-D Lite, fixed focus | Raspberry Pi 5 USB 3 | a. passes every hard number b. 61 g c. depth computed on the camera, so the Pi doesn't do stereo matching |
| 360° lidar plane (P-5, P-13) | LDROBOT STL-27L | Pi USB, through a UART adapter | a. 0.167° between readings, so a 7 cm ball shows at about 8 m (1.7 m with the LD19) b. 10 Hz, 25 m c. works in the dark d. 45 g |
| Downward flow \+ height (P-7) | MicoAir MTF-01P | Flight controller TELEM2, MAVLink at 115,200 baud | a. flow \+ 12 m rangefinder b. holds the drone when SLAM loses tracking |
| Later upgrade: side and rear 3D | 2 x IMX219-200 | Needs a Jetson (the Pi 5 can't run both cameras plus SLAM) | Adds 3D coverage above and below the lidar plane at the sides and rear |
| Optional: navigation outdoors | GPS | Flight controller GPS1 | GPS-denied by design, so not needed indoors |
| Optional: objects falling straight down | Upward rangefinder | A spare flight-controller port | Covers the blind cap above the drone |

**Notes:**

- OAK-D Lite has no IR projector. Blank surfaces break both depth and SLAM tracking, and a projector would only fix depth. See P-12. Upgrade if depth proves a problem in testing: OAK-D Pro, fixed focus.
- The OAK-D Lite's colour camera isn't used.
- The Pi 5 limits its USB ports to 600 mA unless `usb_max_current_enable=1` is set, because it isn't powered through its own USB-C. The OAK-D Lite needs close to 1 A, so set this and check it on the bench.

**Forward camera**

| Check  | Requirement | OAK-D Lite |
| :---- | :---- | :---- |
| Price | | \$376 CAD (Luxonis page, Nov 4); P1 lists about \$210-240 CAD, to confirm |
| Mass | P-10 | 61 g |
| Power | P-10 | 3.5 W expected; 5 W only if AI or colour encoding runs |
| HFOV x VFOV | P-5: 70o x 50o | 73o x 58o, tilted up 8o |
| Mounting | P-5 | Under the nose, front face 70 mm ahead of centre, 62 mm below the props: the lowest spot that keeps all four props out of view with a 3o margin |
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

**Lidar**

| Check | Requirement | STL-27L |
| :---- | :---- | :---- |
| Price | | \$219.09 CAD (Amazon.ca, Nov 10) |
| Mass | P-10 | 45 g |
| Power | P-10 | \< 1.5 W |
| Mounting | P-5 | Printed mast at the rear of the top plate, scan plane 73 mm above the props, above the battery |
| Angular step | P-4 | 0.167° (21,600 points/s at 10 Hz) |
| Range | P-13 | 25 m max; a 7 cm ball at about 8 m |
| Rate | P-6 | 10 Hz |

**Notes:**

- Confirm its ROS 2 driver runs on Jazzy.

**Downward optical flow \+ rangefinder**

| Check  | Requirement | MicoAir MTF-01P |
| :---- | :---- | :---- |
| Price | | \$40.99 USD (about \$57 CAD) |
| Mass | P-10 | About 8 g |
| Flow min light  | P-9: fly at \>= 200 lux | \> 60 lux |
| Flow min height | P-7: hover \>= 0.3 m | 8 cm; 84 mm on the ground with the CAD's legs |
| Rangefinder range | P-7: 0.1 \- 4 m | 0.02 \- 12 m |
| Rate | P-7: \>= 20 Hz | 100 Hz |
| Connection to PX4 | \-  | TELEM2, MAVLink, 115,200 baud |

**Notes:**

- MTF-01P setup: newer AUTO units detect the protocol themselves; on older units, set mav\_px4 in MicoAssistant. Either way, set the flight controller's serial port and enable flow and the rangefinder in PX4.
- Mounted under the rear of the frame, behind the Pi, facing down.

**Mass and power budget**

| Item  | Mass (g)  | Power (W) | Power peak (W) | Cost (CAD) |
| :---- | :---- | :---- | :---- | :---- |
| OAK-D Lite | 61 | 3.5 | 5 | 376 |
| STL-27L | 45 | \< 1.5 | \< 1.5 | 219 |
| MicoAir MTF-01P | \~ 8 | 0.5 | 0.5 | 57 |
| Mounts \+ cables | \~ 15 | \- | \- | \- |
| Total | \~ 129 | \~ 5.5 | \~ 7 | 652 |
| **P-10 limit** | **120** | **5 typical** | **Regulator 10 W+** | **\-** |

**NOTES**
- Prices checked Nov 2026 at about 1.40 CAD per USD. Excludes tax, shipping and duties.
- The sensors with their mounts are just over 120 g and 5 W typical, so P-10 still needs the team to agree. The whole drone stays under the 950 g limit (835 g, or 919 g with margin).
