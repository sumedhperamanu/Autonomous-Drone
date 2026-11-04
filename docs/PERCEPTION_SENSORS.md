

| Job | Pick | Reasoning |
| :---- | :---- | :---- |
| Forward depth (P-1 to P-5) | OAK-D Lite, fixed focus | a. passes every hard number b. lightest (61g) c. cheapest (\$376 CAD) d. depth computed on the camera  |
| Downward flow \+ height (P-7) | MicroAir MTF-01 | a. flow \+ 8m rangefinder b. 4.5g c. \$46 CAD d. MAVLink to PX4 |

**Notes:**

- OAK-D Lite has no IR projector. Blank surfaces break both depth and SLAM tracking, and a projector would only fix depth. See P-12. Upgrade if depth proves a problem in testing: OAK-D Pro, fixed focus.

**Forward depth camera**

| Check  | Requirement | OAK-D Lite |
| :---- | :---- | :---- |
| Price | assuming \~\$700 total | \$376 CAD |
| Mass | P-10: \<= 120 g total | 61 g |
| Power | P-10: \<= 5 W | 3.5 W exptected; 5 W only if AI or colour encoding runs |
| HFOV x VFOV | P-5: 70o x 50o | 73o x 58o |
| Range | P-1: \>= 5 m | ideal 0.8-12 m |
| Accuracy at 3 m | P-2 \<= 0.10 m | \< 2% \~ 0.06 m |
| Min depth | P-3 \<= 0.4 m | 0.2 m |
| 5 cm at 3 m | P-4: enough pixels | \~ 7 px at 640 wide |
| Shutter | P-9: global preferred  | Global (stereo pair); rolling (colour) |
| Active stereo | P-9: preferred | No |
| Depth comuted  | on camera saves the campanion computer | On camera |
| IMU | useful for SLAM | BMI270 |
| Baseline | used in P-2/P-3 math | 7.5 cm |

**Notes:** 

- Avoid Kickstarter-era OAK-D Lite units as they have no IMU; Get the fixed-focus variant   
- PX4’s gz\_x500\_depth model is based on the OAK-D, so the sim camera is close to what we’d buy
- SLAM input: left mono image + depth aligned to left mono. Colour camera not used in SLAM

**Downward optical flow \+ rangefinder**

| Check  | Requirement | MicroAIr MTF-01 |
| :---- | :---- | :---- |
| Price | assuming \~\$700 total | \$46 CAD |
| Mass | P-10: \<= 120 g total | 4.5 g |
| Power | P-10: \<= 5 W | 0.5 W |
| Flow sensor  | \- | PMW3901 |
| Flow min light  | P-9: fly at \>= 200 lux | \> 60 lux |
| Flow min height | P-7: hover \>= 0.3 m | 8 cm |
| Rangefinder range | P-7: 0.1 \- 4 m | 0.02 \- 8 m |
| Rate | P-7: \>= 20 Hz | 100 Hz |
| Connection to PX4 | \-  | UART, MAVLink |
| Built-in IMU | \-  | No |

**Notes:**

- MTF-01 setup: newer AUTO units detect the protocol themselves; on older units, set mav\_px4 in MicoAssistant. Either way, set the flight controller's serial port and enable flow and the rangefinder in PX4.  
- H-Flow and ARK Flow need a CAN port on the flight controller  
- Slim: PX4 has gz\_x500\_lidar\_down for the downward rangefinder

**Mass and power budget**

| Item  | Mass (g)  | Power (W) | Power peak (W) | Cost (CAD) |
| :---- | :---- | :---- | :---- | :---- |
| OAK-D Lite | 61 | 3.5 | 5 | 376 |
| MIcroAir MTF-01 | 4.5 | 0.5 | 0.5 | 46 |
| Mounts \+ cables | \~ 15 | \- | \- | \- |
| Total | \~ 81 | \~ 4 | \~ 5.5 | 422 |
| **P-10 limit** | **120** | **5** | **5** | **\-** |

**NOTES**
- Prices checked Nov 4, 2026, at about 1.40 CAD per USD. Excludes tax, shipping and duties

