"""CAD skeleton of the 5" drone: OAK-D Lite, STL-27L lidar, MTF-01P and Raspberry Pi 5 on a Source One V5 5" frame.

Places every major part, finds an OAK-D Lite mount that keeps the props out of its view, checks every part
against every other part and against every sensor view, and works out mass and balance. Writes skeleton.step
and skeleton.png. model5.py builds the detailed presentation model from the same layout.

Run with cadquery and matplotlib installed: python3 skeleton.py
All lengths are mm and masses are g, X forward, Y left, Z up. Z = 0 is the prop plane.
The sim's base_link sits 26.9 mm below the prop plane (P1's fiveinch model), so add BASE_LINK_Z for transforms.
"""
from itertools import combinations
import math

import cadquery as cq
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

BASE_LINK_Z = 26.9

# Source One V5 5": 226 mm motor to motor, 123.5 g. 5.1" props.
MOTOR_XY = 226.0 / 2 / math.sqrt(2)
PROP_R = 5.1 * 25.4 / 2
# ESTIMATES until checked against the frame and motor STEP files.
MOTOR_H = 18.0          # 2207 bell and base, from the arm to the prop hub
ARM_T, ARM_W = 5.0, 14.0
ARM_TOP = -MOTOR_H - 2.0
PLATE_T = 2.0
BOTTOM_PLATE = (110.0, 42.0)
STANDOFF_H = 25.0
TOP_PLATE = (110.0, 42.0)
TOP_Z = ARM_TOP + PLATE_T + STANDOFF_H          # bottom of the top plate
PLATE_UNDER = ARM_TOP - ARM_T - PLATE_T         # underside of the bottom plate

# Sensors (deg) and placements
OAK_HFOV, OAK_VFOV, OAK_TILT = 73.0, 58.0, 8.0
OAK_SIZE = (17.5, 91.0, 28.0)
FLOW_FOV, RANGE_FOV = 42.0, 6.0
FLOW_MIN = 80.0
BATTERY_SIZE = (77.0, 38.0, 46.0)               # Tattu R-Line 5.0 6S 1800, 287 g
BATTERY_X = 2.0
LIDAR_R, LIDAR_H = 19.5, 34.0
LIDAR_X = -62.0
LIDAR_Z = TOP_Z + PLATE_T + BATTERY_SIZE[2] + 18.0   # scan plane clears the battery
PI_SIZE = (85.0, 56.0, 21.0)                    # board + active cooler
MTF_SIZE = (29.3, 17.0, 14.5)
LEG_BOTTOM = -125.0

MARGIN_DEG = 3.0

SRC_LIST = 'listing'
SRC_EST = 'estimate'
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
        t, u = math.tan(math.radians(self.hfov / 2)), math.tan(math.radians(self.vfov / 2))
        out = [turn((1.0, a * t, b * u), self.yaw, self.tilt) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        return [tuple(c / math.sqrt(sum(x * x for x in r)) for c in r) for r in out]

    def contains(self, p, margin=0.0):
        d = [p[i] - self.apex[i] for i in range(3)]
        if self.kind == 'plane':
            r = math.hypot(d[0], d[1])
            return abs(d[2]) < 0.5 and self.hfov < r < self.length
        if self.kind == 'cone':
            down = -d[2]
            return 0 < down < self.length and math.degrees(math.atan2(math.hypot(d[0], d[1]), down)) < self.hfov / 2 + margin
        c, s = math.cos(math.radians(-self.yaw)), math.sin(math.radians(-self.yaw))
        x, y, z = d[0] * c - d[1] * s, d[0] * s + d[1] * c, d[2]
        c, s = math.cos(math.radians(-self.tilt)), math.sin(math.radians(-self.tilt))
        x, z = x * c - z * s, x * s + z * c
        return (0 < x < self.length and abs(math.degrees(math.atan2(y, x))) < self.hfov / 2 + margin
                and abs(math.degrees(math.atan2(z, x))) < self.vfov / 2 + margin)

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


def prop_points(step=2.0):
    n = int(PROP_R / step)
    for sx in (1, -1):
        for sy in (1, -1):
            for i in range(-n, n + 1):
                for j in range(-n, n + 1):
                    x, y = i * step, j * step
                    if x * x + y * y <= PROP_R * PROP_R:
                        yield sx * MOTOR_XY + x, sy * MOTOR_XY + y, 0.0


def oak_mount():
    """The OAK-D Lite position nearest the frame that keeps every prop out of its view (with margin)."""
    pts = list(prop_points(3.0))
    for front in range(70, 141, 5):
        for z in range(-34, -75, -2):
            lenses = [(front, s * 37.5, z) for s in (1, -1)]
            views = [View('', '', 'pyramid', lens, '', 0.0, OAK_TILT, OAK_HFOV, OAK_VFOV) for lens in lenses]
            if not any(v.contains(p, MARGIN_DEG) for v in views for p in pts):
                return front, z
    raise SystemExit('No OAK-D Lite mount keeps the props out of view')


OAK_FRONT, OAK_Z = oak_mount()


def build_parts():
    """Return every part of the 5" layout."""
    parts = []

    def add(*args, **kwargs):
        parts.append(Part(*args, **kwargs))

    dark, teal = '0.25', '#0e6e6e'
    arm_len = math.hypot(MOTOR_XY, MOTOR_XY) + 10
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        add(f'arm_{i}', 'box', (arm_len, ARM_W, ARM_T), (sx * MOTOR_XY / 2, sy * MOTOR_XY / 2, ARM_TOP - ARM_T / 2), 17.0,
            SRC_EST, dark, yaw=math.degrees(math.atan2(sy, sx)), inertia_size=(MOTOR_XY, MOTOR_XY, ARM_T))
        add(f'motor_{i}', 'cyl', (14.0, MOTOR_H - 1.0), (sx * MOTOR_XY, sy * MOTOR_XY, ARM_TOP + (MOTOR_H - 1.0) / 2), 31.6,
            SRC_LIST, '0.45')
        add(f'prop_{i}', 'cyl', (PROP_R, 2.0), (sx * MOTOR_XY, sy * MOTOR_XY, 0.0), 4.5, SRC_LIST, '0.75')
        add(f'leg_{i}', 'cyl', (3.5, PLATE_UNDER - LEG_BOTTOM), (sx * 40.0, sy * 40.0, (PLATE_UNDER + LEG_BOTTOM) / 2), 6.0,
            SRC_EST, 'tab:green')
        add(f'standoff_{i}', 'cyl', (2.5, STANDOFF_H), (sx * 15.25, sy * 15.25, ARM_TOP + PLATE_T + STANDOFF_H / 2), 1.875,
            SRC_EST, '0.6')

    add('bottom_plate', 'box', (*BOTTOM_PLATE, PLATE_T), (0, 0, ARM_TOP - ARM_T - PLATE_T / 2), 24.0, SRC_EST, dark)
    add('middle_plate', 'box', (*BOTTOM_PLATE, PLATE_T), (0, 0, ARM_TOP + PLATE_T / 2), 12.0, SRC_EST, dark)
    add('top_plate', 'box', (*TOP_PLATE, PLATE_T), (0, 0, TOP_Z + PLATE_T / 2), 12.0, SRC_EST, dark)
    add('esc', 'box', (44.0, 43.5, 5.2), (0, 0, ARM_TOP + PLATE_T + 6.0), 13.5, SRC_LIST, 'tab:red')
    add('flight_controller', 'box', (36.0, 36.0, 8.0), (0, 0, ARM_TOP + PLATE_T + 17.0), 10.0, SRC_LIST, 'tab:purple')
    add('battery', 'box', BATTERY_SIZE, (BATTERY_X, 0, TOP_Z + PLATE_T + BATTERY_SIZE[2] / 2), 287.0, SRC_LIST, 'gold')
    mast_top = LIDAR_Z - LIDAR_H / 2 - 9
    add('mast', 'cyl', (5.0, mast_top - TOP_Z - PLATE_T), (LIDAR_X, 0, (mast_top + TOP_Z + PLATE_T) / 2), 8.0, SRC_EST, dark)
    add('stl27l', 'cyl', (LIDAR_R, LIDAR_H), (LIDAR_X, 0, LIDAR_Z - 9), 45.0, SRC_LIST, '#8c5a1a')
    pi_top = PLATE_UNDER - 3.0
    add('pi5', 'box', PI_SIZE, (-4.0, 0, pi_top - PI_SIZE[2] / 2), 56.0, SRC_LIST, 'tab:olive')
    add('pi_cage', 'box', (90.0, 60.0, 2.0), (-4.0, 0, pi_top - PI_SIZE[2] - 1.0), 15.0, SRC_EST, '0.6')
    add('oakd_lite', 'box', OAK_SIZE, (OAK_FRONT - OAK_SIZE[0] / 2, 0, OAK_Z), 61.0, SRC_LIST, teal)
    mount_top, mount_bottom = PLATE_UNDER, OAK_Z + OAK_SIZE[2] / 2
    add('oakd_mount', 'box', (8.0, 40.0, mount_top - mount_bottom), (OAK_FRONT - OAK_SIZE[0] - 4.0, 0,
                                                                       (mount_top + mount_bottom) / 2), 8.0, SRC_EST, 'tab:cyan')
    add('mtf01p', 'box', MTF_SIZE, (-63.0, 0, PLATE_UNDER - MTF_SIZE[2] / 2), 8.0, SRC_LIST, teal)
    add('bec', 'box', (30.0, 14.0, 6.0), (36.0, 0, TOP_Z - 3.0), 8.0, SRC_EST, '#b5551a')
    add('rx', 'box', (13.0, 10.0, 3.0), (-35.0, 0, ARM_TOP + PLATE_T + 1.5), 2.0, SRC_EST, '#a0305a')
    add(WIRES, 'box', (40.0, 30.0, 10.0), (0, 0, -5), 22.0, SRC_EST, '0.85')
    return parts


def build_views(parts):
    get = {p.name: p for p in parts}
    oak = get['oakd_lite']
    views = [View('OAK-D Lite view', 'oakd_lite', 'pyramid', (OAK_FRONT, 0, oak.center[2]), '#1f77b4', 0.0,
                  OAK_TILT, OAK_HFOV, OAK_VFOV)]
    views.append(View('lidar scan plane', 'stl27l', 'plane', (LIDAR_X, 0, LIDAR_Z), '#d62728', hfov=LIDAR_R + 0.1,
                      length=1000.0))
    mtf = get['mtf01p']
    apex = (mtf.center[0], mtf.center[1], mtf.center[2] - mtf.size[2] / 2)
    views.append(View('flow view', 'mtf01p', 'cone', apex, '#ff7f0e', hfov=FLOW_FOV, length=500.0))
    views.append(View('range beam', 'mtf01p', 'cone', apex, '#9467bd', hfov=RANGE_FOV, length=2000.0))
    return views


def mass_props(parts):
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


def allowed_contact(a, b):
    """Parts that are meant to touch or sit inside each other in this simple model."""
    base = [n.rsplit('_', 1)[0] if n[-1].isdigit() else n for n in (a, b)]
    pairs = {('mast', 'stl27l'), ('oakd_lite', 'oakd_mount'), ('arm', 'arm'), ('pi5', 'pi_cage'),
             ('standoff', 'flight_controller'), ('standoff', 'esc'), ('arm', 'motor'), ('arm', 'standoff')}
    return tuple(base) in pairs or tuple(base[::-1]) in pairs


def check(parts, views):
    solids = {p.name: p.solid() for p in parts if p.name != WIRES}
    problems = []
    for (na, a), (nb, b) in combinations(solids.items(), 2):
        if allowed_contact(na, nb):
            continue
        v = overlap(a, b)
        if v > 1.0:
            problems.append(f'OVERLAP {na} and {nb}: {v:.0f} mm^3')
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
            hx = MOTOR_XY / 2 + 5
        box = (cx - hx, cz - hz, 2 * hx, 2 * hz)
    else:
        if p.name.startswith('arm'):
            hy = MOTOR_XY / 2 + 5
        box = (cy - hy, cz - hz, 2 * hy, 2 * hz)
    ax.add_patch(Rectangle(box[:2], box[2], box[3], fc=p.color, ec='0.2', lw=0.5, alpha=alpha))


def draw(parts, views, com, totals, problems):
    fig, axes = plt.subplots(2, 2, figsize=(16, 12), gridspec_kw={'width_ratios': [1.1, 1]})
    top, table_ax, side, front = axes[0][0], axes[0][1], axes[1][0], axes[1][1]
    order = ('prop', 'arm', 'bottom', 'middle', 'pi', 'mtf', 'oakd', 'esc', 'rx', 'bec', 'flight', 'top_plate',
             'battery', 'mast', 'stl', 'motor', 'leg', 'standoff')
    drawn = [p for k in order for p in parts if p.name.startswith(k) and p.name != WIRES]
    for p in drawn:
        draw_part(top, p, 'top')
        draw_part(side, p, 'side')
        draw_part(front, p, 'front')
    reach = 260.0
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
            top.add_patch(Circle((v.apex[0], v.apex[1]), 200, fill=False, ec=v.color, ls='--', lw=1, alpha=0.6))
    for ax in (side, front):
        ax.axhline(LEG_BOTTOM, color='k', lw=1.2)
        ax.axhline(0, color='0.6', lw=0.7, ls=':')
    for ax, (a, b) in ((top, (0, 1)), (side, (0, 2)), (front, (1, 2))):
        ax.plot(com[a], com[b], 'x', color='k', ms=10, mew=2)
    top.set(title='Top view (mm), X forward', xlabel='X forward', ylabel='Y left', aspect='equal',
            xlim=(-180, 260), ylim=(-180, 180))
    side.set(title='Side view (mm), Z = 0 at the prop plane', xlabel='X forward', ylabel='Z up', aspect='equal',
             xlim=(-180, 260), ylim=(-140, 140))
    front.set(title='Front view (mm), looking back at the drone', xlabel='Y left', ylabel='Z up', aspect='equal',
              xlim=(180, -180), ylim=(-140, 140))
    for ax in (top, side, front):
        ax.grid(alpha=0.2)
    rows, seen = [], set()
    for p in parts:
        base = p.name.rsplit('_', 1)[0] if p.name[-1].isdigit() else p.name
        if base in seen:
            continue
        seen.add(base)
        group = [q for q in parts if q.name.startswith(base)]
        s = 'r %.0f, h %.0f' % p.size if p.shape == 'cyl' else '%.0f x %.0f x %.0f' % tuple(p.size)
        rows.append([f'{len(group)} x {base}' if len(group) > 1 else base, s, f'{sum(q.mass for q in group):.1f}',
                     f'{p.center[0]:.0f}, {p.center[1]:.0f}, {p.center[2]:.0f}', p.src])
    rows.sort(key=lambda r: -float(r[2]))
    table_ax.axis('off')
    tb = table_ax.table(cellText=rows, colLabels=['Part', 'Size (mm)', 'Mass (g)', 'First x, y, z (mm)', 'Source'],
                        bbox=[0, 0.33, 1, 0.67], cellLoc='left', colWidths=[0.26, 0.2, 0.11, 0.25, 0.14])
    tb.auto_set_font_size(False)
    tb.set_fontsize(8)
    table_ax.set_title('Parts and mass budget')
    found = '\n'.join(['Check findings:'] + ['  ' + p for p in problems] if problems else ['Check findings: none'])
    table_ax.text(0.0, 0.0, totals + '\n\n' + found, fontsize=8.5, family='monospace', transform=table_ax.transAxes,
                  va='bottom')
    fig.suptitle('5" drone CAD skeleton: frame sizes and some masses are estimates, see cad/README.md', fontsize=13)
    fig.tight_layout()
    fig.savefig('skeleton.png', dpi=110)


def main():
    parts = build_parts()
    views = build_views(parts)
    print(f'OAK-D Lite mount: front face x {OAK_FRONT} mm, centre z {OAK_Z} mm')
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
    lines = [f'Total mass        {total:.0f} g, {total * 1.1:.0f} g with 10% margin (limit 950 g)',
             f'Centre of mass    x {com[0]:.1f}  y {com[1]:.1f}  z {com[2]:.1f} mm',
             f'Inertia kg m^2    {inertia[0]:.4f} {inertia[1]:.4f} {inertia[2]:.4f}',
             f'Flow height       {flow_height:.0f} mm on the ground ({FLOW_MIN:.0f} mm minimum)',
             f'Lidar scan plane  {LIDAR_Z:.0f} mm above the props']
    print('\n' + '\n'.join(lines))
    print('\nStatic transforms from base_link (m):')
    oak = next(p for p in parts if p.name == 'oakd_lite')
    for name, (x, y, z) in (('oakd_link', (OAK_FRONT, 0.0, oak.center[2])), ('lidar_link (scan plane)', (LIDAR_X, 0.0, LIDAR_Z)),
                            ('flow_link', (mtf.center[0], 0.0, mtf.center[2] - mtf.size[2] / 2))):
        print(f'  {name:24s} {x / 1000:.4f} {y / 1000:.4f} {(z + BASE_LINK_Z) / 1000:.4f}')
    draw(parts, views, com, '\n'.join(lines), problems)
    print(f'\n{len(problems)} findings')


if __name__ == '__main__':
    main()
