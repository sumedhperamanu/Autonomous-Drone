# Decision Log (Gate 1 / M1)

Draft, Nov 9 2026. Not signed yet. Once signed, any change needs P1, P2 and P3 to agree.

The mission now includes avoiding moving obstacles and tracking a person, so the sensor set follows P1's dodge analysis (Nov 7 2026).

## Sensors (decided by P3)

| Decision | Choice | Why |
|---|---|---|
| Forward depth, obstacles and people | OAK-D Lite, fixed focus, under the nose, tilted up 8° | Passes P-1 to P-5, depth and the person detector both run on the camera |
| Side and rear threats | 2 x IMX219-200 on the mast, 15° rearward, tilted up 25° | With the OAK-D Lite, a 3D view over 69% of directions and no gap behind |
| 360° lidar | STL-27L on top of the mast | Sees a 7 cm ball at about 8 m (the LD19 managed 1.7 m); flies phase 1 on its own; works in the dark |
| Downward flow and height | MTF-01P (12 m) | Holds the drone when SLAM loses tracking after a hard dodge |
| AprilTag camera | Not used, OV9281 dropped | P-8 removed |
| GPS | Carried for outdoor flights, not used indoors | Indoors is GPS-denied |
| SLAM camera input | Left mono image + depth aligned to left mono | Stereo pair is global shutter, colour is rolling shutter |
| Person detection | OAK-D Lite colour camera, detector on the camera | Keeps the load off the Jetson |

This replaces the Nov 5 choices of MTF-01, no lidar and no GPS.

## Parts

P1's picks from `docs/p1/day3-4-results.md` and the dodge analysis, plus P3's sensors. Files still need downloading to the team folder.

| Part | Pick | Datasheet or STEP source | Files |
|---|---|---|---|
| Frame | TBS Source One V5 7", 320 mm, with printed Jetson plate, sensor mast, OAK-D mount, battery tray and legs | [source_one repo](https://github.com/tbs-trappy/source_one) | Only a V6 7" DXF found |
| Motors | 2807, about 1700 KV | Exact model not picked | Open |
| Props | 7x3.5 two-blade | Exact model not picked | Open |
| Battery | 4S 5000 mAh, 567 g (from P1's price check) | Exact model not named | Open |
| Flight controller | MicoAir743 V2 | [MicoAir page](https://micoair.com/flightcontroller_micoair743v2/) | Page lists a STEP file |
| ESC | MicoAir 55A AM32 4-in-1 | [MicoAir page](https://micoair.com/?p=4485) | Page lists a STEP file; check it is the 55A |
| BEC | Matek BEC12S-PRO | Not located yet | Open |
| RC | RadioMaster RP1 V2 + Pocket ELRS | Not located yet | Open |
| Telemetry | SiK radio | Not located yet | Open |
| Companion computer | Jetson Orin Nano Super dev kit (P2, PR #9) | Not located yet | Open |
| Depth camera | OAK-D Lite, fixed focus | [Spec sheet](https://cdn.shopify.com/s/files/1/0630/8950/8583/files/OAK-D-Lite_Spec_Sheet.pdf), [depthai-hardware repo](https://github.com/luxonis/depthai-hardware) | Spec sheet points to STEP files |
| Side cameras | 2 x IMX219-200 | Not located yet | Open |
| Lidar | STL-27L | Not located yet | Open |
| Flow and range | MTF-01P | Not located yet | Open |
| GPS | Not named | | Open |

## Needs the team

| Question | Proposal | Owner |
|---|---|---|
| Accept the P-13 dodge speeds? | About half of P1's theory: 7 cm at 3 m/s ahead and 2 m/s elsewhere, 20 cm at 5 m/s | All |
| Can PX4 allow dodges above the 1.5 m/s cruise limit? | The offboard node exempts escape moves; set `MPC_XY_VEL_MAX`, `MPC_Z_VEL_MAX_UP` and `MPC_ACC_UP_MAX` to allow them | P1 |
| What is the new sensor budget (P-10)? | The listed sensors already pass the old 120 g before the side cameras are weighed | All |
| Is the $700 budget USD or CAD? | The sensor list has grown, so check the total | All |
| How is the regulator sized? | Count the Jetson, OAK-D Lite, side cameras and lidar | P1 |
| Where does the battery sit? | P1's 33 mm below the props sits inside the frame in `cad/`; 46 mm fits | P1 |
| Where does the MTF-01P sit? | P1's 22 mm below the props sits inside the rear-right arm in `cad/`; 33 mm fits | P1 |
| Do the side cameras need the rear props masked? | They see about 31% of each rear prop disc, in CAD and in the sim | P3 |
| What does the mission do with the tracked person? | Search, track and follow, with dodging in every phase | P2 |

## Sign-off

| Role | Name | Date |
|---|---|---|
| P1 | | |
| P2 | | |
| P3 | | |
