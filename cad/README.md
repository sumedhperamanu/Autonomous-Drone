# CAD skeleton

`skeleton.step` holds 32 parts plus the field-of-view solids. `skeleton.png` shows the labelled top, side and front views with the parts table. `skeleton.py` builds both, checks every part against every other part and view, and works out the mass and balance.

`render_3d.py` makes the 3D versions: `skeleton_3d.png` (views from above and below) and `skeleton_3d.html` (rotate it in a browser).

## View the 3D model

You need a browser and an internet connection, which loads the 3D library. You don't need ROS, Python or CAD software.

GitHub only shows the HTML file's code when you click it, so get a copy on your computer first. Use either way:

**Download just the file**

1. Open [`cad/skeleton_3d.html`](https://github.com/sumedhperamanu/Autonomous-Drone/blob/p3/interface-agreement/cad/skeleton_3d.html) on GitHub.
2. Click the **Download raw file** button at the top right of the file.
3. Double-click the downloaded file to open it in your browser.

**Or use a copy of the repo**

```bash
git fetch origin
git checkout p3/interface-agreement
git pull
```

Then open the file:

- Windows: `start cad\skeleton_3d.html`
- WSL Ubuntu: `explorer.exe "$(wslpath -w cad/skeleton_3d.html)"`
- Linux: `xdg-open cad/skeleton_3d.html`

Once this branch is merged, use `main` in place of `p3/interface-agreement`.

**Using the viewer**

| Do this | To do this |
| --- | --- |
| Left-drag | Rotate |
| Scroll | Zoom |
| Right-drag | Move the view |
| Hover over a part | See its name, mass and where the number came from |
| Use the checkboxes | Show or hide the camera view, flow view, range beam, props and legs |

Hide the props and camera view to see the parts in the middle. The camera view is drawn only 170 mm long so it doesn't cover the drone; the real camera sees 5 m or more.

If the page shows no model, check your internet connection. Offline, use `skeleton_3d.png` instead.

## Rebuild the files

```bash
pip install cadquery matplotlib
python3 skeleton.py
python3 render_3d.py
```

Lengths are mm in `base_link`: X forward, Y left, Z up, with Z = 0 at the top of the arms.

The parts are simple blocks, so the STEP file is a layout reference for the Day 5 models, not the final design.

## Current result

| Check | Result |
| --- | --- |
| Part overlaps and parts in a view | 0 problems |
| Total mass | 1136 g, with no margin added |
| Centre of mass | x -0.2, y 0, z 39.7 mm, which is 3.7 mm above the prop plane |
| Battery position | x -8 mm, set to balance the drone |
| Inertia | 0.0058, 0.0071, 0.0087 kg m^2 (roll, pitch, yaw) |
| On the ground | Flow sensor 83 mm up (80 mm minimum), camera clears by 42 mm |

P1's model uses 0.0040, 0.0054, 0.0074 kg m^2 at 1.32 kg, on a smaller rotor spacing. Don't copy these numbers into the sim until the real parts are modelled on Day 7.

## Check these first

The parts table in `skeleton.png` marks where each number came from. These estimates matter most:

- Frame: plate size (130 x 50 mm), arm size, 30 mm standoffs, prop height (35 mm above the arms). We found no V5 7" CAD files. The [source_one repo](https://github.com/tbs-trappy/source_one) has a 7" deadcat DXF, but it's labelled V6.
- Motor positions. They assume a square X at 320 mm. The frame is listed as a deadcat, so its front motors may sit further out, which would give the camera more room.
- Layout: the Pi above the top plate and the battery above the Pi are proposals, along with the leg positions. Stacking them raises the centre of mass.
- BEC, RC receiver, legs, bracket and the 60 g for wires and straps. The wires are counted for mass only, not modelled.
- Sensor sizes and view angles come from retailer listings.

The checks add a 3 deg view margin and 10 mm of body clearance around the props.

## Where the OAK-D Lite can go

Each row is the smallest height offset that works at that forward position.

| Front face x | Centre z | Past plate front |
| --- | --- | --- |
| 70 | -50 | 5 |
| 95 | -35 | 30 |
| 120 | -20 | 55 |
| 145 | 0 | 80 |

The skeleton uses the first row: it needs the shortest bracket, which helps with vibration, but the camera hangs below the plate. The last row sits level with the arms, but needs an 80 mm bracket out the front.

## Landing legs

The camera and MTF-01 sit under the frame, so the drone needs legs. Legs reaching 100 mm below the arms clear the camera by 42 mm and keep flow just above its 80 mm minimum, with only 3 mm to spare.
