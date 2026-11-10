# CAD skeleton

The drone laid out as in P1's hardware decision log (Nov 6 2026) and dodge analysis (Nov 7 2026), for flying from A to B while avoiding still and moving obstacles: the OAK-D Lite under the nose, two side cameras and the STL-27L lidar on a mast, the Jetson on a top deck and the battery underneath.

- `skeleton.step`: 38 parts plus the sensor views as solids.
- `skeleton.png`: labelled top, side and front views with the parts table and the check findings.
- `skeleton_3d.png` and `skeleton_3d.html`: 3D versions (views from above and below, and one you can rotate in a browser).
- `skeleton.py` builds the model, checks every part against every other part and every sensor view, and works out mass and balance. `render_3d.py` makes the 3D files.

Lengths are mm, X forward, Y left, Z up, with Z = 0 at the prop plane as in P1's layout. The sim's `base_link` sits 36 mm below the prop plane, so add 36 mm to Z for transforms from `base_link`.

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
| Use the checkboxes | Show or hide each sensor view, the props and the legs |

Hide the props and views to see the parts in the middle. Camera views are drawn only 200 mm long so they don't cover the drone; the real cameras see several metres.

If the page shows no model, check your internet connection. Offline, use `skeleton_3d.png` instead.

## Rebuild the files

```bash
pip install cadquery matplotlib
python3 skeleton.py
python3 render_3d.py
```

## Current result

| Check | Result |
| --- | --- |
| Total mass | 1,404 g (P1: about 1,383 g) |
| Centre of mass | 3 mm aft, 17.4 mm below the prop plane (P1: 4 mm aft, 5 mm below) |
| Inertia | 0.0072, 0.0087, 0.0095 kg m^2 (roll, pitch, yaw) |
| Flow sensor on the ground | 90 mm up (80 mm minimum) |
| Parts overlapping | None, after the two moves below |

The centre of mass sits lower than P1's because the battery had to move down (below).

## What the checks found

| Finding | What it means |
| --- | --- |
| The left side camera sees the rear-left prop, and the right sees the rear-right, about 31% of each disc | The props sit in the lower part of each side camera's view. Mask them in software, or tilt the cameras up more, and check on the real mount. The sim shows the same |
| The OAK-D Lite's top edge just clips both front props, about 3% of each disc | Small; confirm with the real mount and the 8° tilt |
| P1's battery height (centre 33 mm below the props) sits inside the frame | With motors about 20 mm tall, the 48 mm tall pack's top would cut through the bottom plate and arms. It's placed just under the frame here, centre 52 mm below |
| P1's MTF-01P height (22 mm below the props) sits inside the rear-right arm | It's placed just under the arm here, 33 mm below the props |

The battery and MTF-01P findings depend on my frame estimates, so check them against the real frame before moving anything.

The side cameras also see the lidar body at the top of their view in the sim, where they sit under it. This model's checks don't catch that, because they measure from the front of each camera.

## Check these first

The parts table in `skeleton.png` marks where each number came from: `P1` for P1's layout, `listing` for a seller's page, `estimate` for my guess.

- Frame: motor height (20 mm), plate sizes, arm size and the top deck height. We found no V5 7" CAD files. The [source_one repo](https://github.com/tbs-trappy/source_one) has a 7" deadcat DXF, but it's labelled V6.
- Motor positions. They assume a square X at 320 mm. The frame is listed as a deadcat, so its front motors may sit further out.
- Masses of the mast, legs, SiK radio, BEC and the 60 g of wires and straps, which are counted for mass only. The Jetson (176 g), STL-27L (45 g) and battery (567 g, 138 x 47 x 48 mm) come from P1's log.
- Leg positions (85 mm out along each diagonal). Legs at 60 mm sat in the flow sensor's view.
- Sensor sizes and view angles. The side cameras' 160° x 120° is an estimate from their 200° diagonal; measure it.
