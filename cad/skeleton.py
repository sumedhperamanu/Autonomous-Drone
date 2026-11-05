"""CAD skeleton for the 7" drone: motors, props, plate, sensors and field-of-view solids.

Finds where the OAK-D Lite can sit with its body clear of the props and props out of
its view, checks that with solid intersections, then writes skeleton.step and skeleton.png.

Run with cadquery and matplotlib installed: python3 skeleton.py
All lengths are mm, in base_link: X forward, Y left, Z up. Z = 0 is the top of the arms.
"""
import math

import cadquery as cq
from matplotlib import pyplot as plt

# From P1's parts list: 320 mm motor to motor on a square X, 7 inch props.
MOTOR_XY = 113.0
PROP_R = 88.9
# ESTIMATES until checked against the frame and motor STEP files.
PROP_Z = 35.0
PLATE = (130.0, 50.0, 2.5)
PLATE_Z = -8.5

# OAK-D Lite: 91 x 28 x 17.5 mm, 75 mm baseline, 73 x 58 deg stereo view.
CAM_W, CAM_H, CAM_D = 91.0, 28.0, 17.5
BASELINE = 75.0
HFOV, VFOV = 73.0, 58.0
# MTF-01: 29.3 x 17 x 14.5 mm, flow view 42 deg, range beam 6 deg, flow works above 80 mm.
MTF = (29.3, 17.0, 14.5)
FLOW_FOV, RANGE_FOV = 42.0, 6.0
MTF_POS = (20.0, 0.0, PLATE_Z - PLATE[2] - MTF[2] / 2)

MARGIN_DEG = 3.0
CLEAR_MM = 10.0
FOV_LEN = 1000.0


def prop_points(step=2.0):
    """Sample every prop disc as points."""
    n = int(PROP_R / step)
    for sx in (1, -1):
        for sy in (1, -1):
            for i in range(-n, n + 1):
                for j in range(-n, n + 1):
                    x, y = i * step, j * step
                    if x * x + y * y <= PROP_R * PROP_R:
                        yield sx * MOTOR_XY + x, sy * MOTOR_XY + y, PROP_Z


def in_view(p, lens, margin):
    """Return True if point p falls inside the camera view from this lens."""
    dx, dy, dz = p[0] - lens[0], p[1] - lens[1], p[2] - lens[2]
    if dx <= 0:
        return False
    return (abs(math.degrees(math.atan2(dy, dx))) < HFOV / 2 + margin
            and abs(math.degrees(math.atan2(dz, dx))) < VFOV / 2 + margin)


def body_clear(x_front, z):
    """Return True if the camera body stays clear of the prop discs."""
    if z + CAM_H / 2 > PROP_Z - CLEAR_MM:
        for x in (x_front, x_front - CAM_D):
            for y in (-CAM_W / 2, CAM_W / 2):
                if math.hypot(x - MOTOR_XY, abs(y) - MOTOR_XY) < PROP_R + CLEAR_MM:
                    return False
        cx = min(max(MOTOR_XY, x_front - CAM_D), x_front)
        if math.hypot(cx - MOTOR_XY, CAM_W / 2 - MOTOR_XY) < PROP_R + CLEAR_MM:
            return False
    return True


def find_mount():
    """Find the placement nearest the frame's centre that passes both checks."""
    points = list(prop_points())
    for x_front in range(70, 221, 5):
        for z in sorted(range(-60, 61, 5), key=abs):
            if not body_clear(x_front, z):
                continue
            lenses = [(x_front, s * BASELINE / 2, z) for s in (1, -1)]
            if any(in_view(p, lens, MARGIN_DEG) for lens in lenses for p in points):
                continue
            return x_front, z
    raise SystemExit('No placement passes')


def print_options():
    """Print the smallest height offset that works at each forward position."""
    points = list(prop_points(4.0))
    print('Placement options (front face x, nearest working centre z, mm):')
    for x_front in range(70, 221, 25):
        for z in sorted(range(-60, 61, 5), key=abs):
            lenses = [(x_front, s * BASELINE / 2, z) for s in (1, -1)]
            if body_clear(x_front, z) and not any(in_view(p, lens, MARGIN_DEG) for lens in lenses for p in points):
                print(f'  x = {x_front:3d}, z = {z:4d}, past plate front {x_front - PLATE[0] / 2:5.1f}, '
                      f'mass shift {61.0 * (x_front - CAM_D / 2) / 1320.0:.1f}')
                break
        else:
            print(f'  x = {x_front:3d}, none')


def box(size, center):
    return cq.Workplane('XY').box(*size).translate(center)


def pyramid(lens, hfov, vfov, length):
    """Solid view pyramid opening along +X from the lens."""
    y = length * math.tan(math.radians(hfov / 2))
    z = length * math.tan(math.radians(vfov / 2))
    return (cq.Workplane('YZ', origin=(lens[0] + 0.1, lens[1], lens[2]))
            .rect(0.2, 0.2)
            .workplane(offset=length)
            .rect(2 * y, 2 * z)
            .loft(combine=True))


def cone(apex, fov, length):
    """Solid view cone opening along -Z from the apex."""
    r = length * math.tan(math.radians(fov / 2))
    return cq.Workplane('XY').add(cq.Solid.makeCone(
        0.1, r, length, pnt=cq.Vector(*apex), dir=cq.Vector(0, 0, -1)))


def main():
    print_options()
    x_front, z = find_mount()
    cam_center = (x_front - CAM_D / 2, 0.0, z)
    lenses = {'left': (x_front, BASELINE / 2, z), 'right': (x_front, -BASELINE / 2, z)}

    props = [cq.Workplane('XY', origin=(sx * MOTOR_XY, sy * MOTOR_XY, PROP_Z)).circle(PROP_R).extrude(2)
             for sx in (1, -1) for sy in (1, -1)]
    motors = [cq.Workplane('XY', origin=(sx * MOTOR_XY, sy * MOTOR_XY, 0)).circle(17.5).extrude(PROP_Z)
              for sx in (1, -1) for sy in (1, -1)]
    plate = box(PLATE, (0, 0, PLATE_Z - PLATE[2] / 2))
    camera = box((CAM_D, CAM_W, CAM_H), cam_center)
    mtf = box(MTF, MTF_POS)
    apex = (MTF_POS[0], MTF_POS[1], MTF_POS[2] - MTF[2] / 2)
    views = {'left': pyramid(lenses['left'], HFOV, VFOV, FOV_LEN),
             'right': pyramid(lenses['right'], HFOV, VFOV, FOV_LEN)}
    flow = cone(apex, FLOW_FOV, 500.0)
    beam = cone(apex, RANGE_FOV, 2000.0)

    print('Solid intersection checks, volume in mm^3 (want 0):')
    bad = 0.0
    for name, view in views.items():
        total = 0.0
        for part in props + motors:
            try:
                total += view.intersect(part).val().Volume()
            except (ValueError, AttributeError):
                pass
        print(f'  {name} camera view vs props and motors: {total:.1f}')
        bad += total
    for name, view in (('flow', flow), ('range', beam)):
        total = 0.0
        for part in (plate, camera):
            try:
                total += view.intersect(part).val().Volume()
            except (ValueError, AttributeError):
                pass
        print(f'  {name} view vs plate and camera: {total:.1f}')
        bad += total

    assy = cq.Assembly(name='skeleton')
    assy.add(plate, name='plate', color=cq.Color(0.2, 0.2, 0.2))
    for i, (p, m) in enumerate(zip(props, motors)):
        assy.add(p, name=f'prop_{i}', color=cq.Color(0.7, 0.7, 0.7, 0.4))
        assy.add(m, name=f'motor_{i}', color=cq.Color(0.3, 0.3, 0.3))
    assy.add(camera, name='oakd_lite', color=cq.Color(0.1, 0.4, 0.8))
    assy.add(mtf, name='mtf01', color=cq.Color(0.8, 0.4, 0.1))
    for name, view in views.items():
        assy.add(view, name=f'fov_{name}', color=cq.Color(0.1, 0.6, 1.0, 0.15))
    assy.add(flow, name='fov_flow', color=cq.Color(1.0, 0.6, 0.1, 0.15))
    assy.add(beam, name='fov_range', color=cq.Color(1.0, 0.2, 0.1, 0.2))
    assy.export('skeleton.step')

    shift = 61.0 * cam_center[0] / 1320.0
    print(f'\nOAK-D Lite front face x = {x_front} mm, centre z = {z} mm')
    print(f'Overhang past the plate front: {x_front - PLATE[0] / 2:.1f} mm')
    print(f'Centre of mass shift from the camera alone: {shift:.1f} mm forward')
    print('\nStatic transforms from base_link (m):')
    print(f'  oakd_link          {x_front / 1000:.4f} 0.0000 {z / 1000:.4f}')
    print(f'  oakd_left lens     {x_front / 1000:.4f} {BASELINE / 2000:.4f} {z / 1000:.4f}')
    print(f'  flow_link          {apex[0] / 1000:.4f} {apex[1] / 1000:.4f} {apex[2] / 1000:.4f}')

    fig, (top, side) = plt.subplots(1, 2, figsize=(12, 5))
    for sx in (1, -1):
        for sy in (1, -1):
            top.add_patch(plt.Circle((sx * MOTOR_XY, sy * MOTOR_XY), PROP_R, fill=False, color='gray'))
    top.add_patch(plt.Rectangle((-PLATE[0] / 2, -PLATE[1] / 2), PLATE[0], PLATE[1], color='0.3'))
    top.add_patch(plt.Rectangle((x_front - CAM_D, -CAM_W / 2), CAM_D, CAM_W, color='tab:blue'))
    length = 450
    t = length * math.tan(math.radians(HFOV / 2))
    for s in (1, -1):
        y0 = s * BASELINE / 2
        for k in (-1, 1):
            top.plot([x_front, x_front + length], [y0, y0 + k * t], color='tab:blue', alpha=0.4)
    top.set(title='Top view (mm)', xlabel='X forward', ylabel='Y left', aspect='equal',
            xlim=(-220, 460), ylim=(-260, 260))
    side.plot([MOTOR_XY - PROP_R, MOTOR_XY + PROP_R], [PROP_Z, PROP_Z], color='gray', lw=3)
    side.plot([-MOTOR_XY - PROP_R, -MOTOR_XY + PROP_R], [PROP_Z, PROP_Z], color='gray', lw=3)
    side.add_patch(plt.Rectangle((-PLATE[0] / 2, PLATE_Z - PLATE[2]), PLATE[0], PLATE[2], color='0.3'))
    side.add_patch(plt.Rectangle((x_front - CAM_D, z - CAM_H / 2), CAM_D, CAM_H, color='tab:blue'))
    side.add_patch(plt.Rectangle((MTF_POS[0] - MTF[0] / 2, apex[2]), MTF[0], MTF[2], color='tab:orange'))
    v = length * math.tan(math.radians(VFOV / 2))
    for k in (-1, 1):
        side.plot([x_front, x_front + length], [z, z + k * v], color='tab:blue', alpha=0.4)
    d = 200 * math.tan(math.radians(FLOW_FOV / 2))
    for k in (-1, 1):
        side.plot([apex[0], apex[0] + k * d], [apex[2], apex[2] - 200], color='tab:orange', alpha=0.5)
    side.set(title='Side view (mm)', xlabel='X forward', ylabel='Z up', aspect='equal',
             xlim=(-220, 460), ylim=(-250, 280))
    fig.tight_layout()
    fig.savefig('skeleton.png', dpi=110)

    if bad > 1.0:
        raise SystemExit('A solid check failed')
    print('\nAll solid checks pass')


main()
