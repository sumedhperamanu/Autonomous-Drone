"""CAD skeleton for the 7" drone, with every major part, mass estimates and view solids.

Finds where the OAK-D Lite can sit with its body clear of the props and props out of its
view, balances the drone with the battery, then checks every part and view with solid
intersections. Writes skeleton.step and skeleton.png.

Run with cadquery and matplotlib installed: python3 skeleton.py
All lengths are mm and masses are g, in base_link: X forward, Y left, Z up. Z = 0 is the top of the arms.
"""
from itertools import combinations
import math

import cadquery as cq
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

# From P1's parts list: 320 mm motor to motor on a square X, 7 inch props.
MOTOR_XY = 113.0
PROP_R = 88.9
# ESTIMATES until checked against the frame and motor STEP files.
PROP_Z = 35.0
ARM_W = 16.0
STANDOFF_H = 30.0

# OAK-D Lite: 91 x 28 x 17.5 mm, 75 mm baseline, 73 x 58 deg stereo view.
CAM_W, CAM_H, CAM_D = 91.0, 28.0, 17.5
BASELINE = 75.0
HFOV, VFOV = 73.0, 58.0
# MTF-01: flow view 42 deg, range beam 6 deg, flow works above 80 mm.
FLOW_FOV, RANGE_FOV = 42.0, 6.0
FLOW_MIN = 80.0

MARGIN_DEG = 3.0
CLEAR_MM = 10.0
FOV_LEN = 1000.0
LEG_DROP = 100.0

SRC_P1 = 'P1'
SRC_LIST = 'listing'
SRC_EST = 'estimate'
# Counted for mass only, not modelled as a solid.
WIRES = 'wires_and_straps'


class Part:
    """One modelled part: a box or a vertical cylinder."""

    def __init__(self, name, shape, size, center, mass, src, color, yaw=0.0, inertia_size=None):
        self.name, self.shape, self.size, self.center = name, shape, size, center
        self.mass, self.src, self.color, self.yaw = mass, src, color, yaw
        self.inertia_size = inertia_size or size

    def solid(self):
        if self.shape == 'cyl':
            r, h = self.size
            base = (self.center[0], self.center[1], self.center[2] - h / 2)
            return cq.Workplane('XY', origin=base).circle(r).extrude(h)
        s = cq.Workplane('XY').box(*self.size)
        if self.yaw:
            s = s.rotate((0, 0, 0), (0, 0, 1), self.yaw)
        return s.translate(self.center)

    def half(self):
        """Half extents along X, Y and Z."""
        if self.shape == 'cyl':
            return self.size[0], self.size[0], self.size[1] / 2
        return self.size[0] / 2, self.size[1] / 2, self.size[2] / 2


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


def mount_works(x_front, z, points):
    lenses = [(x_front, s * BASELINE / 2, z) for s in (1, -1)]
    return body_clear(x_front, z) and not any(in_view(p, lens, MARGIN_DEG) for lens in lenses for p in points)


def mount_options():
    """Return the smallest height offset that works at each forward position."""
    points = list(prop_points(4.0))
    rows = []
    for x_front in range(70, 171, 25):
        for z in sorted(range(-60, 61, 5), key=abs):
            if mount_works(x_front, z, points):
                rows.append((x_front, z))
                break
    return rows


def build_parts(x_front, cam_z):
    """Return every part, with the battery placed to balance the drone."""
    parts = []

    def add(*args, **kwargs):
        parts.append(Part(*args, **kwargs))

    grey, dark, blue, orange = '0.75', '0.3', 'tab:blue', 'tab:orange'
    arm_len = math.hypot(MOTOR_XY, MOTOR_XY)
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        add(f'arm_{i}', 'box', (arm_len, ARM_W, 6.0), (sx * MOTOR_XY / 2, sy * MOTOR_XY / 2, -3.0), 18.5, SRC_EST,
            dark, yaw=math.degrees(math.atan2(sy, sx)), inertia_size=(MOTOR_XY, MOTOR_XY, 6.0))
        add(f'motor_{i}', 'cyl', (17.5, 30.0), (sx * MOTOR_XY, sy * MOTOR_XY, 15.0), 52.0, SRC_P1, '0.45')
        add(f'prop_{i}', 'cyl', (PROP_R, 2.0), (sx * MOTOR_XY, sy * MOTOR_XY, PROP_Z + 1), 7.0, SRC_P1, grey)
        add(f'leg_{i}', 'cyl', (4.0, LEG_DROP), (sx * 60.0, sy * 60.0, -6.0 - LEG_DROP / 2), 8.0, SRC_EST, 'tab:green')
        add(f'standoff_{i}', 'cyl', (2.5, STANDOFF_H), (sx * 40.0, sy * 18.0, 2.0 + STANDOFF_H / 2), 0.0, SRC_EST, '0.6')

    add('bottom_plate', 'box', (130.0, 50.0, 2.5), (0, 0, -7.25), 24.0, SRC_EST, dark)
    add('middle_plate', 'box', (130.0, 50.0, 2.0), (0, 0, 1.0), 10.0, SRC_EST, dark)
    top = 2.0 + STANDOFF_H
    add('top_plate', 'box', (130.0, 50.0, 2.0), (0, 0, top + 1), 15.4, SRC_EST, dark)
    add('esc', 'box', (44.0, 43.5, 5.2), (0, 0, 6.6), 13.5, SRC_LIST, 'tab:red')
    add('flight_controller', 'box', (36.0, 36.0, 8.0), (0, 0, 18.0), 10.0, SRC_LIST, 'tab:purple')
    add('bec', 'box', (40.0, 20.0, 8.0), (-42.0, 0, 7.0), 10.0, SRC_EST, 'tab:brown')
    add('rc_receiver', 'box', (25.0, 15.0, 5.0), (45.0, 0, 5.5), 2.0, SRC_EST, 'tab:pink')
    add('pi5', 'box', (85.0, 56.0, 20.0), (0, 0, top + 18.0), 56.0, SRC_P1, 'tab:olive')

    mtf_size = (29.3, 17.0, 14.5)
    add('mtf01', 'box', mtf_size, (20.0, 0, -8.5 - mtf_size[2] / 2), 4.5, SRC_LIST, orange)
    add('oakd_lite', 'box', (CAM_D, CAM_W, CAM_H), (x_front - CAM_D / 2, 0, cam_z), 61.0, SRC_LIST, blue)
    bracket_h = -8.5 - (cam_z + CAM_H / 2)
    add('camera_bracket', 'box', (12.0, 40.0, bracket_h), (x_front - CAM_D / 2, 0, -8.5 - bracket_h / 2), 8.0, SRC_EST, 'tab:cyan')
    add(WIRES, 'box', (60.0, 40.0, 10.0), (0, 0, 25.0), 60.0, SRC_EST, '0.85')

    moment = sum(p.mass * p.center[0] for p in parts)
    battery_x = round(-moment / 520.0)
    add('battery', 'box', (155.0, 48.0, 35.0), (battery_x, 0, top + 30.0 + 17.5), 520.0, SRC_P1, 'gold')
    return parts


def mass_props(parts):
    """Return total mass, centre of mass and principal moments in kg m^2."""
    total = sum(p.mass for p in parts)
    com = [sum(p.mass * p.center[i] for p in parts) / total for i in range(3)]
    inertia = [0.0, 0.0, 0.0]
    for p in parts:
        if p.shape == 'cyl':
            r, h = p.inertia_size
            own = (p.mass * (3 * r * r + h * h) / 12, p.mass * (3 * r * r + h * h) / 12, p.mass * r * r / 2)
        else:
            a, b, c = p.inertia_size
            own = (p.mass * (b * b + c * c) / 12, p.mass * (a * a + c * c) / 12, p.mass * (a * a + b * b) / 12)
        d = [p.center[i] - com[i] for i in range(3)]
        inertia[0] += own[0] + p.mass * (d[1] ** 2 + d[2] ** 2)
        inertia[1] += own[1] + p.mass * (d[0] ** 2 + d[2] ** 2)
        inertia[2] += own[2] + p.mass * (d[0] ** 2 + d[1] ** 2)
    return total, com, [i * 1e-9 for i in inertia]


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


def overlap(a, b):
    try:
        return a.intersect(b).val().Volume()
    except (ValueError, AttributeError):
        return 0.0


def check(parts, views):
    """Print every overlap between parts and every part inside a view. Return the problem count."""
    solids = {p.name: p.solid() for p in parts if p.name != WIRES}
    problems = 0
    for (na, a), (nb, b) in combinations(solids.items(), 2):
        if na.startswith('arm') and nb.startswith('arm'):
            continue
        v = overlap(a, b)
        if v > 1.0:
            print(f'  OVERLAP {na} and {nb}: {v:.0f} mm^3')
            problems += 1
    for name, (view, owner) in views.items():
        for part, solid in solids.items():
            if part == owner:
                continue
            v = overlap(view, solid)
            if v > 1.0:
                print(f'  IN VIEW {part} inside {name}: {v:.0f} mm^3')
                problems += 1
    print(f'  {len(solids)} parts and {len(views)} views checked, {problems} problems')
    return problems


def draw_part(ax, p, view):
    """Draw one part in the top (X-Y), side (X-Z) or front (Y-Z) view."""
    hx, hy, hz = p.half()
    cx, cy, cz = p.center
    if view == 'top':
        if p.shape == 'cyl':
            ax.add_patch(Circle((cx, cy), hx, fc=p.color, ec='0.2', lw=0.5, alpha=0.35 if 'prop' in p.name else 0.9))
            return
        if p.yaw:
            c, s = math.cos(math.radians(p.yaw)), math.sin(math.radians(p.yaw))
            pts = [(cx + a * c - b * s, cy + a * s + b * c)
                   for a, b in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
            ax.add_patch(Polygon(pts, fc=p.color, ec='0.2', lw=0.5))
            return
        box = (cx - hx, cy - hy, 2 * hx, 2 * hy)
    elif view == 'side':
        if p.yaw:
            hx = MOTOR_XY / 2
        box = (cx - hx, cz - hz, 2 * hx, 2 * hz)
    else:
        if p.yaw:
            hy = MOTOR_XY / 2
        box = (cy - hy, cz - hz, 2 * hy, 2 * hz)
    ax.add_patch(Rectangle(box[:2], box[2], box[3], fc=p.color, ec='0.2', lw=0.5, alpha=0.9))


def label(ax, text, xy, xytext):
    ax.annotate(text, xy=xy, xytext=xytext, fontsize=8, ha='center', va='center',
                arrowprops={'arrowstyle': '-', 'color': '0.35', 'lw': 0.6})


def draw(parts, x_front, cam_z, com, ground, apex, totals):
    """Write the labelled drawing: top, side and front views plus the parts table."""
    fig, axes = plt.subplots(2, 2, figsize=(17, 12), gridspec_kw={'width_ratios': [1.15, 1]})
    top, table_ax, side, front = axes[0][0], axes[0][1], axes[1][0], axes[1][1]
    order = {'top': ('prop', 'arm', 'bottom', 'middle', 'esc', 'flight', 'bec', 'rc', 'mtf', 'oakd', 'camera', 'top_plate', 'pi5', 'battery', 'motor'),
             'other': ('arm', 'leg', 'motor', 'prop', 'bottom', 'middle', 'top_plate', 'standoff', 'esc', 'flight', 'bec', 'rc', 'pi5', 'battery', 'mtf', 'camera', 'oakd')}

    def ordered(keys):
        return [p for k in keys for p in parts if p.name.startswith(k)]

    for p in ordered(order['top']):
        draw_part(top, p, 'top')
    for p in ordered(order['other']):
        draw_part(side, p, 'side')
        draw_part(front, p, 'front')

    # Camera views
    reach = 420.0
    t, v = math.tan(math.radians(HFOV / 2)), math.tan(math.radians(VFOV / 2))
    for s in (1, -1):
        for k in (1, -1):
            top.plot([x_front, x_front + reach], [s * BASELINE / 2, s * BASELINE / 2 + k * reach * t], color='tab:blue', alpha=0.4, lw=0.9)
    for k in (1, -1):
        side.plot([x_front, x_front + reach], [cam_z, cam_z + k * reach * v], color='tab:blue', alpha=0.5, lw=1)
    # Flow view and range beam, down to the ground
    drop = apex[2] - ground
    f, b = math.tan(math.radians(FLOW_FOV / 2)) * drop, math.tan(math.radians(RANGE_FOV / 2)) * drop
    for k in (1, -1):
        side.plot([apex[0], apex[0] + k * f], [apex[2], ground], color='tab:orange', alpha=0.8, lw=1)
        side.plot([apex[0], apex[0] + k * b], [apex[2], ground], color='tab:red', alpha=0.8, lw=1)
        front.plot([apex[1], apex[1] + k * f], [apex[2], ground], color='tab:orange', alpha=0.8, lw=1)
    for ax in (side, front):
        ax.axhline(ground, color='k', lw=1.2)
        ax.axhline(PROP_Z + 1, color='0.6', lw=0.7, ls=':')

    top.plot(com[0], com[1], 'x', color='k', ms=10, mew=2)
    side.plot(com[0], com[2], 'x', color='k', ms=10, mew=2)
    front.plot(com[1], com[2], 'x', color='k', ms=10, mew=2)

    battery = next(p for p in parts if p.name == 'battery')
    label(top, 'OAK-D Lite', (x_front - CAM_D / 2, 30), (x_front + 60, 150))
    label(top, 'Prop disc', (113, 113), (113, 225))
    label(top, 'Motor', (-113, 113), (-113, 225))
    label(top, 'Battery on top', (battery.center[0] - 30, 0), (-60, -230))
    label(top, 'Arm', (-90, -90), (-200, -230))
    label(top, 'Centre of mass', (com[0], com[1]), (150, -235))
    label(side, 'Battery', (battery.center[0], battery.center[2]), (-160, 125))
    label(side, 'Pi 5', (0, 50), (130, 90))
    label(side, 'Top plate', (55, 33), (170, 60))
    label(side, 'FC', (10, 18), (-170, 20) )
    label(side, 'Prop plane', (160, PROP_Z + 1), (245, 62))
    label(side, 'Motor', (113, 15), (240, 12))
    label(side, 'OAK-D Lite', (x_front - 8, cam_z), (180, -50))
    label(side, 'Bracket', (x_front - 8, -22), (150, -22))
    label(side, 'MTF-01', (20, -16), (-90, -40))
    label(side, 'Leg', (-60, -70), (-160, -70))
    label(side, 'Flow view', (apex[0] - f * 0.6, apex[2] - drop * 0.6), (-90, -92))
    label(side, 'Ground', (-190, ground), (-190, ground + 22))
    label(front, 'Battery', (0, battery.center[2]), (140, 125))
    label(front, 'OAK-D Lite', (30, cam_z), (150, -50))
    label(front, 'Prop disc', (113, PROP_Z + 1), (190, 70))
    label(front, 'Leg', (-60, -70), (-150, -70))

    top.set(title='Top view (mm), X forward', xlabel='X forward', ylabel='Y left', aspect='equal', xlim=(-230, 380), ylim=(-255, 255))
    side.set(title='Side view (mm), X forward', xlabel='X forward', ylabel='Z up', aspect='equal', xlim=(-230, 340), ylim=(-125, 155))
    front.set(title='Front view (mm), looking back at the drone', xlabel='Y left', ylabel='Z up', aspect='equal', xlim=(235, -235), ylim=(-125, 155))
    for ax in (top, side, front):
        ax.grid(alpha=0.2)

    # Parts table, with the repeated parts grouped
    rows, seen = [], set()
    for p in parts:
        base = p.name.rsplit('_', 1)[0] if p.name[-1].isdigit() else p.name
        if base in seen:
            continue
        seen.add(base)
        group = [q for q in parts if q.name.startswith(base)]
        s = '%.0f x %.0f x %.0f' % tuple(2 * h for h in p.half()) if p.shape == 'box' else 'r %.0f, h %.0f' % p.size
        if p.yaw:
            s = '%.0f x %.0f x %.0f' % p.size
        n = len(group)
        rows.append([f'{n} x {base}' if n > 1 else base, s, f'{sum(q.mass for q in group):.1f}',
                     f'{p.center[0]:.0f}, {p.center[1]:.0f}, {p.center[2]:.0f}', p.src])
    rows.sort(key=lambda r: -float(r[2]))
    table_ax.axis('off')
    tb = table_ax.table(cellText=rows, colLabels=['Part', 'Size (mm)', 'Mass (g)', 'First x, y, z (mm)', 'Source'],
                        bbox=[0, 0.27, 1, 0.73], cellLoc='left', colWidths=[0.27, 0.22, 0.12, 0.24, 0.14])
    tb.auto_set_font_size(False)
    tb.set_fontsize(9)
    table_ax.set_title('Parts and mass budget')
    table_ax.text(0.0, 0.0, totals, fontsize=10, family='monospace', transform=table_ax.transAxes, va='bottom')

    fig.suptitle('7" drone CAD skeleton: frame sizes and many masses are estimates, see cad/README.md', fontsize=13)
    fig.tight_layout()
    fig.savefig('skeleton.png', dpi=115)


def main():
    options = mount_options()
    print('Placement options (front face x, nearest working centre z, past plate front, mm):')
    for x, z in options:
        print(f'  x = {x:3d}, z = {z:4d}, past plate front {x - 65:3d}')
    x_front, cam_z = options[0]

    parts = build_parts(x_front, cam_z)
    mtf = next(p for p in parts if p.name == 'mtf01')
    apex = (mtf.center[0], mtf.center[1], mtf.center[2] - mtf.size[2] / 2)
    lenses = {'left camera view': (x_front, BASELINE / 2, cam_z), 'right camera view': (x_front, -BASELINE / 2, cam_z)}
    views = {name: (pyramid(lens, HFOV, VFOV, FOV_LEN), 'oakd_lite') for name, lens in lenses.items()}
    views['flow view'] = (cone(apex, FLOW_FOV, 500.0), 'mtf01')
    views['range beam'] = (cone(apex, RANGE_FOV, 2000.0), 'mtf01')

    print('\nCollision and view checks:')
    problems = check(parts, views)

    assy = cq.Assembly(name='skeleton')
    for p in parts:
        if p.name != WIRES:
            assy.add(p.solid(), name=p.name)
    for name, (view, _) in views.items():
        assy.add(view, name=name.replace(' ', '_'), color=cq.Color(0.1, 0.6, 1.0, 0.15))
    assy.export('skeleton.step')

    total, com, inertia = mass_props(parts)
    ground = -6.0 - LEG_DROP
    flow_height = apex[2] - ground
    cam_gap = (cam_z - CAM_H / 2) - ground
    battery = next(p for p in parts if p.name == 'battery')
    lines = [f'Total mass        {total:.0f} g',
             f'Centre of mass    x {com[0]:.1f}  y {com[1]:.1f}  z {com[2]:.1f} mm',
             f'CoM vs prop plane {com[2] - (PROP_Z + 1):+.1f} mm',
             f'Battery x         {battery.center[0]:.0f} mm, set to balance',
             f'Inertia kg m^2    {inertia[0]:.4f} {inertia[1]:.4f} {inertia[2]:.4f}',
             f'On the ground     flow {flow_height:.0f} mm up, camera clears by {cam_gap:.0f} mm']
    print('\n' + '\n'.join(lines))
    print(f'Flow height meets the {FLOW_MIN:.0f} mm minimum: {flow_height >= FLOW_MIN}')

    print('\nStatic transforms from base_link (m):')
    print(f'  oakd_link          {x_front / 1000:.4f} 0.0000 {cam_z / 1000:.4f}')
    print(f'  flow_link          {apex[0] / 1000:.4f} {apex[1] / 1000:.4f} {apex[2] / 1000:.4f}')

    draw(parts, x_front, cam_z, com, ground, apex, '\n'.join(lines))
    if problems:
        raise SystemExit(f'{problems} problems found')
    print('\nAll checks pass')


if __name__ == '__main__':
    main()
