

| Job | Pick | Reasoning |
| :---- | :---- | :---- |
| Forward depth (P-1 to P-5) | OAK-D Lite, fixed focus | a. passes every hard number b. lightest (61g) c. cheapest (\$376) d. depth computed on the camera  |
| Downward flow \+ height (P-7) | MicroAir MTF-01 | a. flow \+ 8m rangefinder b. 4.5g c. \$32 d. MAVLink to PX4 |
| AprilTag camera (P-8) | Waveshare OV9281 USB (A) | a. global shutter b. reads an 18cm tag from \~ 5m c. USB |

**Notes:**

- OAK-D Lite has no IR projector: if the flying room has plain white walls, need to add tape or posters 

**Forward depth camera**

| Check  | Requirement | OAK-D Lite |
| :---- | :---- | :---- |
| Price | assuming \~\$700 total | \$376 |
| Mass | P-10: \<= 120 g total | 61 g |
| Power | P-10: \<= 5 W | 3-3.5 W typically, up to 5 W |
| HFOV x VFOV | P-5: 70o x 50o | 73o x 58o |
| Range | P-1: \>= 5 m | ideal 0.8-12 m |
| Accuracy at 3 m | P-2 \<= 0.10 m | \< 2% \~ 0.06 m |
| Min depth | P-3 \<= 0.4 m | 0.2 m |
| 5 cm at 3 m | P-4: enough pixels | \~ 7 px at 640 wide |
| Shutter | P-9: global preferred  | Global |
| Active stereo | P-9: preferred | No |
| Depth comuted  | on camera saves the campanion computer | On camera |
| IMU | useful for SLAM | BMI270 |
| Baseline | used in P-2/P-3 math | 7.5 cm |

**Notes:** 

- Avoid Kickstarter-era OAK-D Lite units as they have no IMU; Get the fixed-focus variant   
- PX4’s gz\_x500\_depth model is based on the OAK-D, so the sim camera is close to what we’d buy

**Downward optical flow \+ rangefinder**

| Check  | Requirement | MicroAIr MTF-01 |
| :---- | :---- | :---- |
| Price | assuming \~\$700 total | \$175 |
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

- MTF-01 setup: set its protocol to mav\_px4 in MicroAssistant, then wire it to a spare TELEM/UART port  
- H-Flow and ARK Flow need a CAN port on the flight controller  
- Slim: PX4 has gz\_x500\_lidar\_down for the downward rangefinder

**Downward camera for the AprilTag**

| Check  | Requirement | Waveshare OV9281 USB (A) |
| :---- | :---- | :---- |
| Price | assuming \~\$700 total | \$36.32 |
| Mass | P-10: \<= 120 g total | 18 g |
| Connection  | works with a computer | USB 2.0 |
| Shutter | P-9: global | global |
| FOV | wide helps search | 105o |
| Frame rate | P-6 | 120 fps at 1280 x 720 |
| Focal length | used for P-8 | \~ 563 px |
| Max tag height, 18 cm tag | P-8: \>= 3.75 m | \~ 5.1 m |
| Floor detection at 0.3 m | P-8: \>= 18 cm tag fits | \~0.68 m |
| Focus | sharp from 0.3 to 2.5 m | fixed, M12 |

**Notes:**

- focus: the B0411 is fixed at 50 cm and beyon, so the tag may blur in the last 0.3 m of landing; On M12 cameras, set focus near 1 m and test at 0.3 m and 2.5 m	  
- Wide lenses bend the image; Calibrate the camera before trusting tag process	  
- Slim: PX4 has gz\_x500\_mono\_cam\_down for a downward camera

**Mass and power budget**

| Item  | Mass (g)  | Power (W) | Power peak (W) | Cost (CAD) |
| :---- | :---- | :---- | :---- | :---- |
| OAK-D Lite | 61 | 3.5 | 5 | 376 |
| MIcroAir MTF-01 | 4.5 | 0.5 | 0.5 | 32 |
| Waveshare OV9281 USB (A) | 18 | \~ 0.5 | \~ 0.5 | 36 |
| Mounts \+ cables | \~ 15 | \- | \- | \- |
| Total | \~ 99 | \~ 4.5 | \~ 6 | 441 |
| **P-10 limit** | **120** | **5** | **5** | **\-** |

