# Decision Log (Gate 1 / M1)

Draft, Nov 14 2026. Not signed yet. Once signed, any change needs P1, P2 and P3 to agree.

The mission is to fly from point A to point B while avoiding still and moving obstacles. The drone is now the 5" build from the team's 5" requirements: 5–5.1" props on a 210–230 mm frame, under 950 g with a 10% margin, thrust-to-weight of at least 4:1, a hover of at least 7 minutes, 270° or more of horizontal sensing plus forward depth, PX4 v1.17 with ROS 2 over XRCE-DDS, downward flow and range, and a Gazebo equivalent for every sensor.

## Sensors (decided by P3)

| Decision | Choice | Why |
|---|---|---|
| Forward depth and path obstacles | OAK-D Lite, fixed focus, under the nose, tilted up 8° | Passes P-1 to P-5 with depth computed on the camera. Its front face sits 70 mm ahead of centre and 62 mm below the props: the lowest spot that keeps all four props out of view with a 3° margin |
| 360° lidar | STL-27L on a rear mast, scan plane 73 mm above the props | Clears the battery. With the OAK-D Lite it meets the 270°+ requirement. A 7 cm ball shows at about 8 m. Works in the dark; 45 g, under 1.5 W |
| Downward flow and height | MTF-01P (12 m), under the rear of the frame | 84 mm off the ground when landed. Holds the drone when SLAM loses tracking after a hard dodge |
| Side cameras | Not in this build; a later upgrade that needs a Jetson | The Pi 5 can't run two more cameras alongside SLAM. The lidar plane covers the sides and rear |
| SLAM camera input | Left mono image + depth aligned to left mono | The stereo pair is global shutter; the colour camera is rolling shutter |
| Which way the drone faces | Its direction of travel | The OAK-D Lite then always covers the path |
| AprilTag camera | Not used | P-8 removed; B is a pre-checked landing spot |
| Person tracking | Not used | Not part of the mission |
| GPS | Optional, on GPS1 | GPS-denied by design |
| Upward rangefinder | Optional, on a spare flight-controller port | Covers objects falling straight down |

This replaces the Nov 10 choices for the 7" build (side cameras, Jetson, lidar on a centre mast).

## Parts

Picks from the 5" parts list. Prices are from Nov 2026, before tax and shipping. Files still need downloading to the team folder.

| Part | Pick | Datasheet or STEP source | Files |
|---|---|---|---|
| Frame | Pyrodrone Source One V5 5", 226 mm, 123.5 g (\$38.99 USD), with printed OAK-D cradle, lidar mast mounts, Pi carrier and tray, MTF-01P holder and legs | [source_one repo](https://github.com/tbs-trappy/source_one) | Look for the V5 5" files; `cad/model5.py` uses estimated plate sizes until then |
| Motors | iFlight XING2 2207 1855KV x 4 (\$27.99 USD each) | Not located yet | Open |
| Props | HQ R35 V2 5.1x3.5x3, 2 sets (\$4.99 USD a set) | Not located yet | Open |
| Battery | Tattu R-Line 5.0 6S 1800 mAh 150C, 287 g, 77 x 38 x 46 mm, on top under one strap (\$50.99 USD) | Not located yet | Open |
| Flight controller | MicoAir743 V2, PX4 v1.17, soft-mounted | [MicoAir page](https://micoair.com/flightcontroller_micoair743v2/) | Page lists a STEP file |
| ESC | MicoAir 55A AM32 4-in-1, bidirectional DShot | [MicoAir page](https://micoair.com/?p=4485) | Page lists a STEP file; check it is the 55A |
| ESC capacitor | Panasonic EEU-FR1H102, 1000 µF 50 V low-ESR (\$4.17 CAD) | [DigiKey](https://www.digikey.ca/en/products/detail/panasonic-industry/EEU-FR1H102/2433600) | Open |
| Pi power | Matek BEC12S-PRO, set for the Pi 5 | Not located yet | Open |
| RC | RadioMaster RP1 + Pocket ELRS | Not located yet | Open |
| Companion computer | Raspberry Pi 5 8 GB (\$249.50 CAD, PiShop) with the Active Cooler (\$7.15) and a microSD card | Not located yet | Open |
| Depth camera | OAK-D Lite, fixed focus | [Spec sheet](https://cdn.shopify.com/s/files/1/0630/8950/8583/files/OAK-D-Lite_Spec_Sheet.pdf), [depthai-hardware repo](https://github.com/luxonis/depthai-hardware) | Spec sheet points to STEP files |
| Lidar | LDROBOT STL-27L | Not located yet | Open |
| Flow and range | MTF-01P | Not located yet | Open |

No SiK radio and no Jetson in this build.

## Mass

From `cad/skeleton.py`: 835 g, or 919 g with the 10% margin, under the 950 g limit. The frame counts at its listed 123.5 g. Other estimated masses are marked in `cad/skeleton.png`.

## Needs the team

| Question | Proposal | Owner |
|---|---|---|
| Accept the P-13 dodge speeds? | 7 cm at 3 m/s and 20 cm at 5 m/s, ahead and on the lidar plane. Nothing above or below the plane outside the forward view. These are P1's 7" numbers; the smaller 5" should do at least as well | All |
| Can PX4 allow dodges above the 1.5 m/s cruise limit? | Confirm `MPC_XY_VEL_MAX` and PX4's climb speed and acceleration limits allow about 3 m/s sideways and 4 m/s up | P1, P3 |
| P-10 sensor limits | P1 proposes 120 g total, 5 W typical, regulator 10 W or more. The sensors with mounts come to about 129 g and 5.5 W | All |
| P-11a stale depth | P1 proposes hold on flow, land after 2 s | P1, P3 |
| Budget | The 5" parts list comes to about \$1,950 CAD, against \$900 for phase 1 in the team's doc. Cut or raise | All |
| eCalc | Rerun with the 2207 1855KV motors, 5.1x3.5 props and 835 g to confirm the 4:1 thrust-to-weight and the 7-minute hover | P1 |
| ESC eRPM | Confirm the 55A AM32 handles at least 250k eRPM with these motors on 6S | P1 |
| Sim body mass | P1's `fiveinch` model uses 0.819 kg; the CAD now says 835 g. Weigh the built drone and update it | P1 |
| OAK-D Lite power on the Pi 5 | Set `usb_max_current_enable=1` and check the camera gets close to 1 A on the bench | P3 |
| OAK-D Lite price | \$376 CAD from Luxonis's page on Nov 4 vs about \$210-240 CAD in P1's log | P3 |
| STL-27L driver | Confirm its ROS 2 driver runs on Jazzy | P3 |
| Printed parts | Check the draft cradle, mounts, carrier and legs in `cad/model5.py` against the real frame before printing | P1, P3 |

## Sign-off

| Role | Name | Date |
|---|---|---|
| P1 | | |
| P2 | | |
| P3 | | |
