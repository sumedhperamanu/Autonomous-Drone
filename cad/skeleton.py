"""CAD skeleton of the 7" drone in P1's layout (dodge analysis, Nov 7 2026), with mass estimates and view solids.

Places every major part where P1's layout puts it, then checks every part against every other part and
against every sensor's view with solid intersections, and works out mass and balance. Writes skeleton.step
and skeleton.png.

Run with cadquery and matplotlib installed: python3 skeleton.py
All lengths are mm and masses are g, X forward, Y left, Z up. Z = 0 is the prop plane, as in P1's layout.
The sim's base_link sits 36 mm below the prop plane, so add BASE_LINK_Z to get transforms from base_link.
"""
from itertools import combinations
import math

import cadquery as cq
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

BASE_LINK_Z = 36.0

# From P1's parts list: 320 mm motor to motor on a square X, 7 inch props.
MOTOR_XY = 113.0
PROP_R = 88.9
# ESTIMATES until checked against the frame and motor STEP files.
MOTOR_H = 20.0
ARM_W = 16.0
ARM_TOP = -MOTOR_H
DECK_Z = 21.0

# Sensor fields of view (deg)
OAK_HFOV, OAK_VFOV, OAK_TILT = 73.0, 58.0, 8.0
SIDE_HFOV, SIDE_VFOV, SIDE_BACK, SIDE_TILT = 160.0, 120.0, 15.0, 25.0
FLOW_FOV, RANGE_FOV = 42.0, 6.0
FLOW_MIN = 80.0
LIDAR_Z = 138.0
LEG_BOTTOM = -130.0

# P1 puts the battery centre 33 mm and the MTF-01P 22 mm below the props. With ~20 mm tall motors both
# would sit inside the frame, so they go just below it here, and main() reports the difference.
PLATE_BOTTOM = ARM_TOP - 6.0 - 2.5
BATTERY_SIZE = (155.0, 48.0, 35.0)
BATTERY_Z_P1, MTF_Z_P1 = -33.0, -22.0
BATTERY_Z = min(BATTERY_Z_P1, PLATE_BOTTOM - BATTERY_SIZE[2] / 2)
MTF_SIZE = (29.3, 17.0, 14.5)
MTF_Z = min(MTF_Z_P1, ARM_TOP - 6.0 - MTF_SIZE[2] / 2)

SRC_P1 = 'P1'
SRC_LIST = 'listing'
SRC_EST = 'estimate'
# Counted for mass only, not modelled as a solid.
WIRES = 'wires_and_straps'


class Part:
    """One modelled part: a box (optionally turned about Z) or a vertical cylinder."""

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

    def points(self, n=12):
        """A grid of points through the part, for checking what a view sees."""
        hx, hy, hz = self.half()
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        steps = [-1 + 2 * k / (n - 1) for k in range(n)]
        for a in steps:
            for b in steps:
                if self.shape == 'cyl' and a * a + b * b > 1:
                    continue
                for h in (steps if hz > 2 else [0.0]):
                    x, y = a * hx, b * hy
                    yield self.center[0] + x * c - y * s, self.center[1] + x * s + y * c, self.center[2] + h * hz


class View:
    """A sensor's field of view: a pyramid or a cone, or the lidar's flat scan plane."""

    def __init__(self, name, owner, kind, apex, color, yaw=0.0, tilt=0.0, hfov=0.0, vfov=0.0, length=1000.0):
        self.name, self.owner, self.kind, self.apex, self.color = name, owner, kind, apex, color
        self.yaw, self.tilt, self.hfov, self.vfov, self.length = yaw, tilt, hfov, vfov, length

    def rays(self):
        """Unit directions of a pyramid's four corner edges."""
        t, u = math.tan(math.radians(self.hfov / 2)), math.tan(math.radians(self.vfov / 2))
        out = []
        for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            out.append(turn((1.0, a * t, b * u), self.yaw, self.tilt))
        return [tuple(c / math.sqrt(sum(x * x for x in r)) for c in r) for r in out]

    def contains(self, p):
        """Return True if point p is inside this view."""
        d = [p[i] - self.apex[i] for i in range(3)]
        if self.kind == 'plane':
            r = math.hypot(d[0], d[1])
            return abs(d[2]) < 0.5 and self.hfov < r < self.length
        if self.kind == 'cone':
            down = -d[2]
            return 0 < down < self.length and math.degrees(math.atan2(math.hypot(d[0], d[1]), down)) < self.hfov / 2
        c, s = math.cos(math.radians(-self.yaw)), math.sin(math.radians(-self.yaw))
        x, y, z = d[0] * c - d[1] * s, d[0] * s + d[1] * c, d[2]
        c, s = math.cos(math.radians(-self.tilt)), math.sin(math.radians(-self.tilt))
        x, z = x * c - z * s, x * s + z * c
        return (0 < x < self.length and abs(math.degrees(math.atan2(y, x))) < self.hfov / 2
                and abs(math.degrees(math.atan2(z, x))) < self.vfov / 2)

    def solid(self):
        if self.kind == 'cone':
            r = self.length * math.tan(math.radians(self.hfov / 2))
            return cq.Workplane('XY').add(cq.Solid.makeCone(
                0.1, r, self.length, pnt=cq.Vector(*self.apex), dir=cq.Vector(0, 0, -1)))
        if self.kind == 'plane':
            return (cq.Workplane('XY', origin=(self.apex[0], self.apex[1], self.apex[2] - 0.5))
                    .circle(self.length).circle(self.hfov).extrude(1.0))
        y = self.length * math.tan(math.radians(self.hfov / 2))
        z = self.length * math.tan(math.radians(self.vfov / 2))
        s = (cq.Workplane('YZ', origin=(0.1, 0, 0)).rect(0.2, 0.2)
             .workplane(offset=self.length).rect(2 * y, 2 * z).loft(combine=True))
        s = s.rotate((0, 0, 0), (0, 1, 0), -self.tilt).rotate((0, 0, 0), (0, 0, 1), self.yaw)
        return s.translate(self.apex)


def turn(v, yaw, tilt):
    """Tilt a vector up by tilt deg, then turn it left by yaw deg."""
    x, y, z = v
    c, s = math.cos(math.radians(tilt)), math.sin(math.radians(tilt))
    x, z = x * c - z * s, x * s + z * c
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return x * c - y * s, x * s + y * c, z


def build_parts():
    """Return every part, placed as in P1's layout where P1 gives a position."""
    parts = []

    def add(*args, **kwargs):
        parts.append(Part(*args, **kwargs))

    dark, teal = '0.3', '#0e6e6e'
    arm_len = math.hypot(MOTOR_XY, MOTOR_XY)
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        add(f'arm_{i}', 'box', (arm_len, ARM_W, 6.0), (sx * MOTOR_XY / 2, sy * MOTOR_XY / 2, ARM_TOP - 3.0), 18.5,
            SRC_EST, dark, yaw=math.degrees(math.atan2(sy, sx)), inertia_size=(MOTOR_XY, MOTOR_XY, 6.0))
        add(f'motor_{i}', 'cyl', (17.5, MOTOR_H - 1.0), (sx * MOTOR_XY, sy * MOTOR_XY, -(MOTOR_H + 1.0) / 2), 52.0,
            SRC_P1, '0.45')
        add(f'prop_{i}', 'cyl', (PROP_R, 2.0), (sx * MOTOR_XY, sy * MOTOR_XY, 0.0), 7.0, SRC_P1, '0.75')
        add(f'leg_{i}', 'cyl', (4.0, ARM_TOP - 6.0 - LEG_BOTTOM), (sx * 85.0, sy * 85.0, (ARM_TOP - 6.0 + LEG_BOTTOM) / 2),
            10.0, SRC_EST, 'tab:green')
        add(f'standoff_{i}', 'cyl', (2.5, DECK_Z - ARM_TOP), (sx * 40.0, sy * 18.0, (DECK_Z + ARM_TOP) / 2), 0.0,
            SRC_EST, '0.6')

    add('bottom_plate', 'box', (130.0, 50.0, 2.5), (0, 0, ARM_TOP - 6.0 - 1.25), 24.0, SRC_EST, dark)
    add('deck_plate', 'box', (130.0, 80.0, 2.0), (0, 0, DECK_Z + 1.0), 20.0, SRC_EST, dark)
    add('esc', 'box', (44.0, 43.5, 5.2), (0, 0, -14.0), 13.5, SRC_LIST, 'tab:red')
    add('flight_controller', 'box', (36.0, 36.0, 8.0), (0, 0, 3.0), 10.0, SRC_P1, 'tab:purple')
    add('jetson', 'box', (100.0, 79.0, 30.0), (0, 0, 38.0), 150.0, SRC_P1, '0.2')
    add('mast', 'cyl', (6.0, 121.0 - 53.0), (0, 0, (121.0 + 53.0) / 2), 15.0, SRC_EST, dark)
    add('stl27l', 'cyl', (19.5, 34.0), (0, 0, LIDAR_Z), 47.0, SRC_P1, '#8c5a1a')
    for side, sy, yaw in (('left', 1, 90.0 + SIDE_BACK), ('right', -1, -90.0 - SIDE_BACK)):
        add(f'side_cam_{side}', 'box', (12.0, 25.0, 25.0), (0, sy * 16.0, 100.0), 5.0, SRC_P1, teal, yaw=yaw)
    add('oakd_lite', 'box', (17.5, 91.0, 28.0), (75.0, 0, -50.0), 61.0, SRC_P1, teal)
    mount_top, mount_bottom = PLATE_BOTTOM, -50.0 + 14.0
    add('oakd_mount', 'box', (8.0, 40.0, mount_top - mount_bottom), (71.0, 0, (mount_top + mount_bottom) / 2), 8.0,
        SRC_EST, 'tab:cyan')
    add('battery', 'box', BATTERY_SIZE, (-12.0, 0, BATTERY_Z), 567.0, SRC_P1, 'gold')
    add('mtf01p', 'box', MTF_SIZE, (-50.0, -42.0, MTF_Z), 5.0, SRC_P1, teal)
    add('gps', 'cyl', (15.0, 8.0), (-75.0, 0, 75.0), 10.0, SRC_P1, '#1f4e8c')
    add('gps_stalk', 'cyl', (2.5, 75.0 - 4.0 - DECK_Z - 2.0), (-75.0, 15.0, (75.0 - 4.0 + DECK_Z + 2.0) / 2), 3.0,
        SRC_EST, dark)
    add('sik_radio', 'box', (30.0, 20.0, 10.0), (-75.0, 0, 28.0), 15.0, SRC_P1, '#a0305a')
    add('bec', 'box', (40.0, 20.0, 8.0), (0, 37.0, PLATE_BOTTOM - 4.0), 10.0, SRC_EST, '#b5551a')
    add('elrs_rx', 'box', (15.0, 10.0, 4.0), (-70.0, -70.0, ARM_TOP + 2.0), 2.0, SRC_EST, '#a0305a')
    add(WIRES, 'box', (60.0, 40.0, 10.0), (0, 0, 0), 60.0, SRC_EST, '0.85')
    return parts


def build_views(parts):
    """Return each sensor's view, starting at the sensor's front face."""
    get = {p.name: p for p in parts}
    oak = get['oakd_lite']
    views = [View('OAK-D Lite view', 'oakd_lite', 'pyramid', (oak.center[0] + oak.size[0] / 2, 0, oak.center[2]),
                  '#1f77b4', 0.0, OAK_TILT, OAK_HFOV, OAK_VFOV)]
    for side, yaw in (('left', 90.0 + SIDE_BACK), ('right', -90.0 - SIDE_BACK)):
        cam = get[f'side_cam_{side}']
        f = turn((cam.size[0] / 2, 0, 0), yaw, 0)
        apex = (cam.center[0] + f[0], cam.center[1] + f[1], cam.center[2])
        views.append(View(f'{side} side camera view', f'side_cam_{side}', 'pyramid', apex, '#17becf', yaw,
                          SIDE_TILT, SIDE_HFOV, SIDE_VFOV))
    views.append(View('lidar scan plane', 'stl27l', 'plane', (0, 0, LIDAR_Z), '#d62728', hfov=19.6, length=1000.0))
    mtf = get['mtf01p']
    apex = (mtf.center[0], mtf.center[1], mtf.center[2] - mtf.size[2] / 2)
    views.append(View('flow view', 'mtf01p', 'cone', apex, '#ff7f0e', hfov=FLOW_FOV, length=500.0))
    views.append(View('range beam', 'mtf01p', 'cone', apex, '#9467bd', hfov=RANGE_FOV, length=2000.0))
    return views


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


def overlap(a, b):
    try:
        return a.intersect(b).val().Volume()
    except (ValueError, AttributeError):
        return 0.0


def touches_own_mount(a, b):
    """Parts that are meant to touch or sit inside each other in this simple model."""
    pairs = {('mast', 'stl27l'), ('mast', 'jetson'), ('mast', 'side_cam_left'), ('mast', 'side_cam_right'),
             ('gps', 'gps_stalk'), ('oakd_lite', 'oakd_mount'), ('flight_controller', 'standoff'), ('esc', 'standoff')}
    names = (a.split('_')[0] if a.startswith('standoff') else a, b.split('_')[0] if b.startswith('standoff') else b)
    return (a.startswith('arm') and b.startswith('arm')) or names in pairs or names[::-1] in pairs


def check(parts, views):
    """Print every overlap between parts and every part inside a view. Return the problems found."""
    solids = {p.name: p.solid() for p in parts if p.name != WIRES}
    problems = []
    for (na, a), (nb, b) in combinations(solids.items(), 2):
        if touches_own_mount(na, nb):
            continue
        v = overlap(a, b)
        if v > 1.0:
            problems.append(f'OVERLAP {na} and {nb}: {v:.0f} mm^3')
    # Views are checked by sampling points through each part: solid booleans of thin discs against
    # long pyramids proved unreliable.
    for view in views:
        for p in parts:
            if p.name in (WIRES, view.owner) or (view.owner == 'stl27l' and p.name == 'mast'):
                continue
            pts = list(p.points())
            share = sum(view.contains(q) for q in pts) / len(pts)
            if share > 0:
                problems.append(f'IN VIEW {p.name} in {view.name}: about {100 * share:.0f}% of the part')
    for line in problems:
        print('  ' + line)
    print(f'  {len(solids)} parts and {len(views)} views checked, {len(problems)} problems')
    return problems


def draw_part(ax, p, view):
    """Draw one part in the top (X-Y), side (X-Z) or front (Y-Z) view."""
    hx, hy, hz = p.half()
    cx, cy, cz = p.center
    alpha = 0.35 if p.name.startswith('prop') else 0.9
    if view == 'top':
        if p.shape == 'cyl':
            ax.add_patch(Circle((cx, cy), hx, fc=p.color, ec='0.2', lw=0.5, alpha=alpha))
            return
        if p.yaw:
            c, s = math.cos(math.radians(p.yaw)), math.sin(math.radians(p.yaw))
            pts = [(cx + a * c - b * s, cy + a * s + b * c) for a, b in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
            ax.add_patch(Polygon(pts, fc=p.color, ec='0.2', lw=0.5))
            return
        box = (cx - hx, cy - hy, 2 * hx, 2 * hy)
    elif view == 'side':
        if p.name.startswith('arm'):
            hx = MOTOR_XY / 2
        box = (cx - hx, cz - hz, 2 * hx, 2 * hz)
    else:
        if p.name.startswith('arm'):
            hy = MOTOR_XY / 2
        box = (cy - hy, cz - hz, 2 * hy, 2 * hz)
    ax.add_patch(Rectangle(box[:2], box[2], box[3], fc=p.color, ec='0.2', lw=0.5, alpha=alpha))


def draw_views(top, side, front, views):
    reach = 320.0
    for v in views:
        if v.kind == 'pyramid':
            for r in v.rays():
                end = [v.apex[i] + reach * r[i] for i in range(3)]
                top.plot([v.apex[0], end[0]], [v.apex[1], end[1]], color=v.color, alpha=0.45, lw=0.9)
                side.plot([v.apex[0], end[0]], [v.apex[2], end[2]], color=v.color, alpha=0.45, lw=0.9)
                front.plot([v.apex[1], end[1]], [v.apex[2], end[2]], color=v.color, alpha=0.45, lw=0.9)
        elif v.kind == 'cone':
            drop = v.apex[2] - LEG_BOTTOM
            r = math.tan(math.radians(v.hfov / 2)) * drop
            for k in (1, -1):
                side.plot([v.apex[0], v.apex[0] + k * r], [v.apex[2], LEG_BOTTOM], color=v.color, alpha=0.8, lw=1)
                front.plot([v.apex[1], v.apex[1] + k * r], [v.apex[2], LEG_BOTTOM], color=v.color, alpha=0.8, lw=1)
        else:
            for ax in (side, front):
                ax.axhline(v.apex[2], color=v.color, lw=1, ls='--', alpha=0.7)
            top.add_patch(Circle((0, 0), 300, fill=False, ec=v.color, ls='--', lw=1, alpha=0.6))


def label(ax, text, xy, xytext):
    ax.annotate(text, xy=xy, xytext=xytext, fontsize=8, ha='center', va='center',
                bbox={'boxstyle': 'round,pad=0.15', 'fc': 'white', 'ec': '0.75', 'lw': 0.4},
                arrowprops={'arrowstyle': '-', 'color': '0.35', 'lw': 0.6})


def draw(parts, views, com, totals, problems):
    """Write the labelled drawing: top, side and front views plus the parts table."""
    fig, axes = plt.subplots(2, 2, figsize=(17, 13), gridspec_kw={'width_ratios': [1.1, 1]})
    top, table_ax, side, front = axes[0][0], axes[0][1], axes[1][0], axes[1][1]
    under = ('prop', 'arm', 'bottom', 'battery', 'bec', 'mtf', 'oakd', 'esc', 'elrs', 'deck', 'flight', 'jetson',
             'sik', 'gps', 'mast', 'side', 'stl', 'motor', 'leg', 'standoff')
    drawn = [p for k in under for p in parts if p.name.startswith(k) and p.name != WIRES]
    for p in drawn:
        draw_part(top, p, 'top')
    for p in drawn:
        draw_part(side, p, 'side')
        draw_part(front, p, 'front')
    draw_views(top, side, front, views)
    for ax in (side, front):
        ax.axhline(LEG_BOTTOM, color='k', lw=1.2)
        ax.axhline(0, color='0.6', lw=0.7, ls=':')
    top.plot(com[0], com[1], 'x', color='k', ms=10, mew=2)
    side.plot(com[0], com[2], 'x', color='k', ms=10, mew=2)
    front.plot(com[1], com[2], 'x', color='k', ms=10, mew=2)

    label(top, 'OAK-D Lite', (75, -30), (200, -150))
    label(top, 'Side camera, left', (-4, 20), (-150, 210))
    label(top, 'Jetson', (40, 30), (190, 120))
    label(top, 'Lidar scan (dashed)', (210, 210), (250, 240))
    label(top, 'GPS', (-75, 0), (-190, -60))
    label(top, 'Centre of mass', (com[0], com[1]), (90, -235))
    label(side, 'STL-27L lidar', (0, LIDAR_Z), (-150, 170))
    label(side, 'Side cameras', (0, 100), (-160, 120))
    label(side, 'Jetson', (40, 38), (180, 70))
    label(side, 'GPS', (-75, 75), (-180, 75))
    label(side, 'SiK radio', (-85, 28), (-190, 35))
    label(side, 'Prop plane', (170, 0), (250, 20))
    label(side, 'Battery', (-60, -33), (-190, -20))
    label(side, 'OAK-D Lite', (80, -50), (200, -60))
    label(side, 'MTF-01P', (-50, -25), (-150, -80))
    label(side, 'Leg', (60, -100), (150, -110))
    label(side, 'Ground', (-220, LEG_BOTTOM), (-200, LEG_BOTTOM - 15))
    label(front, 'Side camera views', (-150, 30), (-190, 160))
    label(front, 'BEC', (37, -30), (150, -80))

    top.set(title='Top view (mm), X forward', xlabel='X forward', ylabel='Y left', aspect='equal',
            xlim=(-250, 330), ylim=(-260, 260))
    side.set(title='Side view (mm), X forward, Z = 0 at the prop plane', xlabel='X forward', ylabel='Z up',
             aspect='equal', xlim=(-250, 330), ylim=(-150, 210))
    front.set(title='Front view (mm), looking back at the drone', xlabel='Y left', ylabel='Z up', aspect='equal',
              xlim=(260, -260), ylim=(-150, 210))
    for ax in (top, side, front):
        ax.grid(alpha=0.2)

    rows, seen = [], set()
    for p in parts:
        base = p.name.rsplit('_', 1)[0] if p.name[-1].isdigit() else p.name
        base = 'side_cam' if base.startswith('side_cam') else base
        if base in seen:
            continue
        seen.add(base)
        group = [q for q in parts if q.name.startswith(base)]
        s = 'r %.0f, h %.0f' % p.size if p.shape == 'cyl' else '%.0f x %.0f x %.0f' % tuple(p.size)
        n = len(group)
        rows.append([f'{n} x {base}' if n > 1 else base, s, f'{sum(q.mass for q in group):.0f}',
                     f'{p.center[0]:.0f}, {p.center[1]:.0f}, {p.center[2]:.0f}', p.src])
    rows.sort(key=lambda r: -float(r[2]))
    table_ax.axis('off')
    tb = table_ax.table(cellText=rows, colLabels=['Part', 'Size (mm)', 'Mass (g)', 'First x, y, z (mm)', 'Source'],
                        bbox=[0, 0.33, 1, 0.67], cellLoc='left', colWidths=[0.26, 0.2, 0.11, 0.25, 0.14])
    tb.auto_set_font_size(False)
    tb.set_fontsize(8)
    table_ax.set_title('Parts and mass budget (P1 = position from P1\'s layout)')
    found = '\n'.join(['Check findings:'] + ['  ' + p for p in problems] if problems else ['Check findings: none'])
    table_ax.text(0.0, 0.0, totals + '\n\n' + found, fontsize=8.5, family='monospace',
                  transform=table_ax.transAxes, va='bottom')
    fig.suptitle('7" drone CAD skeleton in P1\'s layout: frame sizes and many masses are estimates, see cad/README.md',
                 fontsize=13)
    fig.tight_layout()
    fig.savefig('skeleton.png', dpi=110)


def main():
    parts = build_parts()
    views = build_views(parts)
    print('Collision and view checks:')
    problems = check(parts, views)
    total, com, inertia = mass_props(parts)

    assy = cq.Assembly(name='skeleton')
    for p in parts:
        if p.name != WIRES:
            assy.add(p.solid(), name=p.name)
    for v in views:
        assy.add(v.solid(), name=v.name.replace(' ', '_'), color=cq.Color(0.1, 0.6, 1.0, 0.15))
    assy.export('skeleton.step')

    mtf = next(p for p in parts if p.name == 'mtf01p')
    flow_height = (mtf.center[2] - mtf.size[2] / 2) - LEG_BOTTOM
    for name, mine, p1 in (('battery', BATTERY_Z, BATTERY_Z_P1), ('MTF-01P', MTF_Z, MTF_Z_P1)):
        if mine != p1:
            problems.append(f'MOVED {name} to z {mine:.0f} mm; P1\'s {p1:.0f} mm is inside the frame')
            print('  ' + problems[-1])
    lines = [f'Total mass        {total:.0f} g (P1: about 1,383 g)',
             f'Centre of mass    x {com[0]:.1f}  y {com[1]:.1f}  z {com[2]:.1f} mm (P1: 4 mm aft, 5 mm below)',
             f'Inertia kg m^2    {inertia[0]:.4f} {inertia[1]:.4f} {inertia[2]:.4f}',
             f'Flow height       {flow_height:.0f} mm on the ground ({FLOW_MIN:.0f} mm minimum)']
    print('\n' + '\n'.join(lines))

    print('\nStatic transforms from base_link (m), base_link 36 mm below the prop plane:')
    for name, (x, y, z) in (('oakd_link', (75.0, 0.0, -50.0)), ('side_left_link', (0.0, 16.0, 100.0)),
                            ('side_right_link', (0.0, -16.0, 100.0)), ('lidar (scan plane)', (0.0, 0.0, LIDAR_Z)),
                            ('flow_link', (-50.0, -42.0, MTF_Z))):
        print(f'  {name:20s} {x / 1000:.4f} {y / 1000:.4f} {(z + BASE_LINK_Z) / 1000:.4f}')

    draw(parts, views, com, '\n'.join(lines), problems)
    print(f'\n{len(problems)} findings; see above')


if __name__ == '__main__':
    main()
