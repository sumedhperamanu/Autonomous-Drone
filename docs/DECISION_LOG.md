# Decision Log (Gate 1 / M1)

Draft, Nov 10 2026. Not signed yet. Once signed, any change needs P1, P2 and P3 to agree.

The mission is to fly from point A to point B while avoiding still and moving obstacles. Hardware follows P1's hardware decision log (Nov 6 2026) and dodge analysis (Nov 7 2026).

## Sensors (decided by P3)

| Decision | Choice | Why |
|---|---|---|
| Forward depth and path obstacles | OAK-D Lite, fixed focus, under the nose, tilted up 8° | Passes P-1 to P-5, depth computed on the camera |
| Side and rear threats | 2 x IMX219-200 on the mast, 15° rearward, tilted up 25°, adjustable mounts | With the OAK-D Lite, a 3D view over 69% of directions and no gap behind |
| 360° lidar | STL-27L on top of the mast | Sees a 7 cm ball at about 8 m (the LD19 managed 1.7 m); flies phase 1 on its own; works in the dark; 45 g, under 1.5 W |
| Downward flow and height | MTF-01P (12 m), beside the battery | Holds the drone when SLAM loses tracking after a hard dodge |
| SLAM camera input | Left mono image + depth aligned to left mono | Stereo pair is global shutter, colour is rolling shutter |
| Which way the drone faces | Its direction of travel | The OAK-D Lite then always covers the path |
| AprilTag camera | Not used | P-8 removed; B is a pre-checked landing spot |
| Person tracking | Not used | Not part of the mission |
| GPS | Optional, on GPS1 | GPS-denied by design |
| Upward rangefinder | Optional, on a spare flight-controller port | Covers objects falling straight down |

This replaces the Nov 5 choices of MTF-01, no lidar and no GPS.

## Parts

From P1's hardware decision log, plus P3's sensors. Files still need downloading to the team folder.

| Part | Pick | Datasheet or STEP source | Files |
|---|---|---|---|
| Frame | TBS Source One V5 7", 320 mm (about \$53 CAD), with printed Jetson plate, mast, OAK-D mount, battery tray, legs and MTF-01P bracket | [source_one repo](https://github.com/tbs-trappy/source_one) | Only a V6 7" DXF found |
| Motors | 2807, about 1700 KV | Exact model not picked | Open |
| Props | 7x3.5 two-blade, balanced before first flight | Exact model not picked | Open |
| Battery | 4S 5000 mAh, 50C or more (Powerhobby 100C, 567 g, about 138 x 47 x 48 mm), underneath | Not located yet | Open |
| Flight controller | MicoAir743 V2, PX4 v1.17, soft-mounted | [MicoAir page](https://micoair.com/flightcontroller_micoair743v2/) | Page lists a STEP file |
| ESC | MicoAir 55A AM32 4-in-1, bidirectional DShot | [MicoAir page](https://micoair.com/?p=4485) | Page lists a STEP file; check it is the 55A |
| Jetson power | Matek BEC12S-PRO set to 12 V, Jetson only, 5.5 x 2.5 mm barrel pigtail | Not located yet | Open |
| RC | RadioMaster RP1 V2 + Pocket ELRS | Not located yet | Open |
| Telemetry | SiK radio | Not located yet | Open |
| Companion computer | Jetson Orin Nano Super Developer Kit, flown on its own carrier, 176 g | Not located yet | Open |
| Storage | NVMe SSD, M.2 2280, 128-256 GB (to buy) | | Open |
| Depth camera | OAK-D Lite, fixed focus | [Spec sheet](https://cdn.shopify.com/s/files/1/0630/8950/8583/files/OAK-D-Lite_Spec_Sheet.pdf), [depthai-hardware repo](https://github.com/luxonis/depthai-hardware) | Spec sheet points to STEP files |
| Side cameras | 2 x IMX219-200 | Not located yet | Open |
| Lidar | LDROBOT STL-27L | Not located yet | Open |
| Flow and range | MTF-01P | Not located yet | Open |

## Needs the team

| Question | Proposal | Owner |
|---|---|---|
| Accept the P-13 dodge speeds? | About half of P1's theory: 7 cm at 3 m/s ahead and 2 m/s elsewhere, 20 cm at 5 m/s | All |
| Can PX4 allow dodges above the 1.5 m/s cruise limit? | Confirm `MPC_XY_VEL_MAX` and PX4's climb speed and acceleration limits allow about 3 m/s sideways and 4 m/s up | P1, P3 |
| P-10 sensor limits | P1 proposes 120 g total, 5 W typical, regulator 10 W or more; the listed sensors already reach about 126 g and 5.5 W before the side cameras | All |
| P-11a stale depth | P1 proposes hold on flow, land after 2 s | P1, P3 |
| Budget | Estimate about \$1,790-2,390 CAD before tax, against a \$1,200-1,500 target: cut or raise | All |
| OAK-D Lite price | \$376 CAD from Luxonis's page on Nov 4 vs about \$210-240 CAD in P1's log | P3 |
| Battery height | P1's 33 mm below the props puts the 48 mm pack inside the frame in `cad/` | P1 |
| MTF-01P height | P1's 22 mm below the props sits inside the rear-right arm in `cad/` | P1 |
| Rear props in the side cameras' view | About 31% of each rear prop disc in `cad/` and in the sim; P1's log expects them below the lower edge | P3 |
| AprilTags | P1's log still lists AprilTags as an OAK-D Lite job; ask P1 to remove it | P1 |
| STL-27L driver | Confirm its ROS 2 driver runs on Jazzy | P3 |
| Side camera field of view | Measure the real lens and confirm the mounting angles | P3 |
| Custom printed parts | Design them from the Source One CAD files | P1, P3 |
| eCalc | Rerun at the measured weight | P1 |

## Sign-off

| Role | Name | Date |
|---|---|---|
| P1 | | |
| P2 | | |
| P3 | | |
