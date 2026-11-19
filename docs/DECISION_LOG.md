# Decision Log (Gate 1 / M1)

Draft, Nov 16 2026. Not signed yet. Once signed, any change needs P1, P2 and P3 to agree.

The mission is to fly from point A to point B while avoiding still and moving obstacles. The drone is P1's 7" on 4S (hardware decision log, Nov 6 2026) with a Jetson Orin Nano Super (P2's decision 0002, Nov 15 2026) and RTAB-Map SLAM (P2's decision 0001). The layout follows P1's hardware log and dodge analysis (Nov 7 2026); `cad/skeleton.py` checks it.

P1's requirements for this airframe: all-up weight 1.2–1.8 kg, thrust-to-weight of at least 2, hover of at least 10 minutes, hover throttle at most 50%.

## Sensors (decided by P3)

| Decision | Choice | Why |
|---|---|---|
| Forward depth and path obstacles | OAK-D Lite, fixed focus, under the nose at P1's position (front face 84 mm ahead of centre, 50 mm below the props), tilted up 8°, soft-mounted | Passes P-1 to P-5 with depth computed on the camera. It sees about 3% of each front prop disc along its top edge; mask that in software. Clearing it fully would put the camera 98 mm below the props, too close to the ground |
| Side and rear threats | 2 x IMX219-200 on the mast, 100 mm above the props and 22 mm either side of it, 15° rearward, tilted up 25°, on adjustable mounts | With the OAK-D Lite, a 3D view over 69% of directions and no gap behind (P1's dodge analysis). In CAD each one sees about 27% of a rear prop disc and the edge of the lidar; about 34° of tilt clears the props. P1's 16 mm either side leaves no room for the board beside the mast once tilted |
| 360° lidar | Waveshare D500 (LDROBOT STL-19P) on top of the mast, scan plane 138 mm above the props | About \$174 less than the STL-27L. 10 Hz, 0.03–12 m, 0.72° between readings, so a 7 cm ball shows at about 1.9 m (8 m with the STL-27L). Works in the dark; 45 g, 1.45 W. Also P2's phase 1 SLAM input |
| Downward flow and height | MTF-01P (12 m), under the rear-right arm, 50 mm aft and 42 mm right | 90 mm off the ground when landed, view clear of the battery. Holds the drone when SLAM loses tracking after a hard dodge |
| SLAM camera input | Left mono image + depth aligned to left mono, at 320 x 240 | The stereo pair is global shutter; the colour camera and the side cameras are rolling shutter and aren't SLAM inputs |
| Which way the drone faces | Its direction of travel | The OAK-D Lite then always covers the path |
| AprilTag camera | Not used | P-8 removed; B is a pre-checked landing spot |
| Person tracking | Not used | Not part of the mission |
| GPS | Optional, on GPS1 | GPS-denied by design |
| Upward rangefinder | Optional, on a spare flight-controller port | Covers objects falling straight down |

This replaces the Nov 14 choices for the 5" Pi 5 build, which the team dropped for this one.

## Parts

From P1's hardware log and P2's decisions 0001 and 0002, plus P3's sensors. Prices in CAD unless marked; confirm them before ordering. Files still need downloading to the team folder.

| Part | Pick | Datasheet or STEP source | Files |
|---|---|---|---|
| Frame | TBS Source One V5 7", 320 mm motor to motor (about \$53), with printed Jetson deck on dampers, mast bridge and mast, OAK-D cradle, battery tray, legs and MTF-01P holder | [source_one repo](https://github.com/tbs-trappy/source_one) | Look for the V5 7" files; `cad/` uses estimated plate sizes until then |
| Motors | 2807, about 1700 KV | Exact model not picked | Open |
| Props | 7x3.5 two-blade, balanced before first flight | Exact model not picked | Open |
| Battery | Powerhobby 4S 5200 mAh 50C, underneath on a tray (listed at 473 g or 567 g; weigh it) | Not located yet | Open |
| Flight controller | MicoAir743 V2, PX4 v1.17, soft-mounted | [MicoAir page](https://micoair.com/flightcontroller_micoair743v2/) | Page lists a STEP file |
| ESC | MicoAir 55A AM32 4-in-1, bidirectional DShot | [MicoAir page](https://micoair.com/?p=4485) | Page lists a STEP file; check it is the 55A |
| Jetson power | Matek BEC12S-PRO at 12 V, Jetson only, 5.5 x 2.5 mm right-angle barrel pigtail, centre positive | Not located yet | Open |
| RC | RadioMaster RP1 + Pocket ELRS | Not located yet | Open |
| Telemetry | The Jetson's Wi-Fi on 5 GHz, `mavlink-router` to QGC; no SiK radio | | |
| Companion computer | Jetson Orin Nano Super Developer Kit, 8 GB (US\$399.99), flown on its own carrier, 176 g, 100 x 79 mm | Not located yet | Open |
| Storage | Used 500–512 GB NVMe SSD, M.2 2280 (about \$47) | | |
| Depth camera | OAK-D Lite, fixed focus | [Spec sheet](https://cdn.shopify.com/s/files/1/0630/8950/8583/files/OAK-D-Lite_Spec_Sheet.pdf), [depthai-hardware repo](https://github.com/luxonis/depthai-hardware) | Spec sheet points to STEP files |
| Side cameras | 2 x IMX219-200 | Not located yet | Open |
| Lidar | Waveshare D500 (LDROBOT STL-19P, 54 x 46.29 x 35 mm, 45 g) | [STL-19P datasheet](https://files.waveshare.com/wiki/D500-LiDAR-Kit/LDROBOT_STL-19P_Datasheet_EN_v1.0(1).pdf) | Open |
| Flow and range | MTF-01P | Not located yet | Open |

## Mass

`cad/skeleton.py` gives 1,476 g with the 567 g battery figure, or about 1,382 g with 473 g. That's inside P1's 1.2–1.8 kg. P1's eCalc at 1,310 g gave 4.1 thrust-to-weight and 14.6 minutes of hover; rerun it at the weighed mass. Estimated masses are marked in `cad/skeleton.png`.

## Needs the team

| Question | Proposal | Owner |
|---|---|---|
| Accept the P-13 dodge speeds? | Front 7 cm at 3 m/s and 20 cm at 5 m/s; sides and behind 2 and 5 m/s; lidar only (phase 1, or a camera fails) 1 and 2.5 m/s with the D500 | All |
| D500 or STL-27L? | The D500 saves \$174 but drops the lidar-only limits from 3 to 1 m/s (7 cm) and 5 to 2.5 m/s (20 cm), and gives SLAM a sparser scan. Inside the cameras' views it doesn't matter | P3, with P2 |
| Can PX4 allow dodges above the 1.5 m/s cruise limit? | Confirm `MPC_XY_VEL_MAX` and PX4's climb speed and acceleration limits allow about 3 m/s sideways and 4 m/s up | P1, P3 |
| Side camera tilt | 25° (P1) sees the rear props and the lidar edge; about 34° clears the props but sees less below the horizon. Measure the real lens before choosing | P3 |
| Side-camera detection on the GPU | At 1,640 px wide it takes about 13 Jetson cores on the CPU (`bench/pi5` on `p3/perception-main`), so it must run on the GPU. Write and measure it on Day 6; the fallback is 410 px wide on the CPU, which cuts a 7 cm ball's side range to about 0.9 m | P3, P2 |
| OAK-D Lite prop mask | Mask the top edge of the depth image where the front props show | P3 |
| P-10 sensor limits | P1 proposes 120 g total, 5 W typical; the OAK-D Lite, D500 and MTF-01P come to about 114 g before mounts and the side cameras | All |
| P-11a stale depth | P1 proposes hold on flow, land after 2 s | P1, P3 |
| Budget | P2's Nov 14 parts list: \$2,036 in parts, \$2,341–2,381 all in, against earlier targets of \$700 and \$1,200–1,500 | All |
| eCalc | Rerun at the weighed mass, with about 2–2.5 A for the electronics | P1 |
| Sim | Motor spacing from 295 to 320 mm and the body mass from the weighed drone | P1 |
| Jetson mounting | Deck on dampers with airflow for the fan; Wi-Fi antennas off the carbon, at least 5 cm from the ELRS antenna | P1, P3 |
| D500 driver | Confirm LDROBOT's STL-19P driver runs on Jazzy | P3 |
| Printed parts | Design the deck, mast bridge, cradle, tray, legs and holder from the Source One CAD files; `cad/model.py` has drafts that fit this layout | P1, P3 |

## Sign-off

| Role | Name | Date |
|---|---|---|
| P1 | | |
| P2 | | |
| P3 | | |
