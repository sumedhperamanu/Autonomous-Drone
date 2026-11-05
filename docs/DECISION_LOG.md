# Decision Log (Gate 1 / M1)

Draft, Nov 5 2026. Not signed yet. Once signed, any change needs P1, P2 and P3 to agree.

## Sensors (decided by P3)

| Decision | Choice | Why |
|---|---|---|
| Forward depth and obstacle sensing | OAK-D Lite, fixed focus | Passes P-1 to P-5, 61 g, depth computed on the camera |
| Downward flow and height | MTF-01 (8 m) | Flow and rangefinder in one 4.5 g unit, MAVLink to PX4 |
| 360 lidar | Not used, LD19 dropped | P-5 uses forward depth to save mass and cost |
| AprilTag camera | Not used, OV9281 dropped | P-8 removed |
| GPS | Not used | Requirements assume GPS-denied flight |
| SLAM camera input | Left mono image + depth aligned to left mono | Stereo pair is global shutter, colour is rolling shutter |

## Parts

P1's picks from `docs/p1/day3-4-results.md` on `sim-5-in-7-in`, plus P3's sensors. Files still need downloading to the team folder.

| Part | Pick | Datasheet or STEP source | Files |
|---|---|---|---|
| Frame | TBS Source One V5 7", 320 mm | [source_one repo](https://github.com/tbs-trappy/source_one) | STEP not confirmed |
| Motors | 2807, about 1700 KV | Exact model not picked | Open |
| Props | 7x3.5 two-blade | Exact model not picked | Open |
| Battery | 4S 5000 mAh, 50C or more | Exact model not picked | Open |
| Flight controller | MicoAir743 V2 | [MicoAir page](https://micoair.com/flightcontroller_micoair743v2/) | Page lists a STEP file |
| ESC | MicoAir 55A AM32 4-in-1 | [MicoAir page](https://micoair.com/?p=4485) | Page lists a STEP file; check it is the 55A |
| BEC | Matek BEC12S-PRO, 5.2 V 5 A | Not located yet | Open |
| RC | RadioMaster RP1 V2 + Pocket ELRS | Not located yet | Open |
| Companion computer | Raspberry Pi 5, 4 GB | [Raspberry Pi product files](https://pip.raspberrypi.com/categories/892-raspberry-pi-5) | Listed, not opened |
| Depth camera | OAK-D Lite, fixed focus | [Spec sheet](https://cdn.shopify.com/s/files/1/0630/8950/8583/files/OAK-D-Lite_Spec_Sheet.pdf), [depthai-hardware repo](https://github.com/luxonis/depthai-hardware) | Spec sheet points to STEP files |
| Flow and range | MTF-01 | [ArduPilot page](https://ardupilot.org/copter/docs/common-mtf-01.html) | Manual not checked |

## Needs the team

| Question | Proposal | Owner |
|---|---|---|
| Which airframe does the sim use? | P1's 7", with the sensors above | P1 |
| What replaces SEARCH, APPROACH and "land on the tag"? | Fly to set points avoiding obstacles, then land at a pre-checked spot | P2 |
| Is the $700 budget USD or CAD? | Sensors are about $422 CAD before tax and shipping | All |
| How is the regulator sized? | P-10 means typical draw; size for 2x headroom and count the Pi if the camera runs through it | P1 |
| What stops the drone near obstacles? | Depth pipeline, since PX4 Collision Prevention needs the dropped lidar | P2, P3 |
| Where does the OAK-D Lite mount? | See `cad/README.md` | All |
| What are the landing legs? | About 100 mm below the plate, to clear the camera and keep flow above its 80 mm minimum | All |

## Sign-off

| Role | Name | Date |
|---|---|---|
| P1 | | |
| P2 | | |
| P3 | | |
