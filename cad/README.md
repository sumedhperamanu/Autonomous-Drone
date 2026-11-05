# CAD skeleton

`skeleton.step` holds the motors, prop discs, plate, sensors and field-of-view solids. `skeleton.png` shows the top and side views. `skeleton.py` builds both and checks that no solid enters a view.

```bash
pip install cadquery matplotlib
python3 skeleton.py
```

Lengths are mm in `base_link`: X forward, Y left, Z up, with Z = 0 at the top of the arms.

## Check these first

These are estimates, so the result is a starting point until they're checked against real STEP files:

- Prop height (35 mm above the arms) and plate size (130 x 50 mm).
- Motor positions. They assume a square X at 320 mm. The Source One V5 7" is listed as a deadcat frame, so its front motors may sit further out, which would give the camera more room.
- Sensor sizes and view angles, which come from retailer listings.

The checks add a 3 deg view margin and 10 mm of body clearance around the props.

## Where the OAK-D Lite can go

Each row is the smallest height offset that works at that forward position.

| Front face x | Centre z | Past plate front | Centre of mass shift |
| --- | --- | --- | --- |
| 70 | -50 | 5 | 2.8 |
| 95 | -35 | 30 | 4.0 |
| 120 | -20 | 55 | 5.1 |
| 145 | 0 | 80 | 6.3 |

The skeleton uses the first row: it needs the shortest bracket, which helps with vibration, but the camera hangs below the plate. The last row sits level with the arms, but needs an 80 mm bracket out the front.

## Landing legs

The first row puts the bottom of the camera 64 mm below the arms, and the MTF-01 also sits under the plate, so the drone needs legs. About 100 mm below the plate clears the camera and keeps flow above its 80 mm minimum.
