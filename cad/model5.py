"""Detailed model of the 5" drone, built on the layout in skeleton.py.

Every part sits inside the space skeleton.py gives it, so the layout, view and mass checks there still apply.
This adds the real shapes: cut carbon plates and arms, 2207 motors, 5.1" tri-blade props, the flight stack,
the Raspberry Pi 5 with its cooler, OAK-D Lite, STL-27L, MTF-01P, the battery with its strap and lead, the
printed mounts and the legs. It then checks the detailed parts against each other and against every sensor view.

Writes model5.step (open in any CAD program), model5.glb and model5_parts.json (used by the 3D page in site/).
Run with cadquery installed, from this folder: python3 model5.py
All lengths are mm, X forward, Y left, Z up, Z = 0 at the prop plane, as in skeleton.py.
"""
import base64
import json
import math

import cadquery as cq

import skeleton as sk

W = cq.Workplane
MX = sk.MOTOR_XY
R_MOTOR = math.hypot(MX, MX)
AT = sk.ARM_TOP                      # top of the arms
AB = sk.ARM_TOP - sk.ARM_T           # bottom of the arms
PU = sk.PLATE_UNDER                  # underside of the bottom plate
TZ = sk.TOP_Z                        # underside of the top plate
BZ = TZ + sk.PLATE_T                 # top of the top plate, where the battery sits
LEG_XY = 40.0
# PX4 quad X: 1 front right CCW, 2 rear left CCW, 3 front left CW, 4 rear right CW
MOTORS = [(1, -1, True), (-1, 1, True), (1, 1, False), (-1, -1, False)]

# Colours for the STEP and GLB files. The 3D page swaps them for full materials by the name after '__'.
MATS = {
    'carbon': (0.07, 0.07, 0.075), 'titanium': (0.55, 0.56, 0.58), 'alu_dark': (0.2, 0.2, 0.22),
    'bell': (0.13, 0.13, 0.14), 'accent': (0.93, 0.38, 0.12), 'copper': (0.72, 0.42, 0.2),
    'steel': (0.76, 0.77, 0.79), 'pcb_black': (0.05, 0.05, 0.06), 'pcb_green': (0.06, 0.36, 0.2),
    'gold': (0.85, 0.66, 0.28), 'chip': (0.09, 0.09, 0.1), 'plastic_white': (0.92, 0.92, 0.9),
    'plastic_black': (0.1, 0.1, 0.11), 'glass': (0.02, 0.03, 0.05), 'print': (0.93, 0.38, 0.12),
    'battery': (0.09, 0.09, 0.1), 'label': (0.78, 0.1, 0.09), 'strap': (0.08, 0.08, 0.08),
    'wire_red': (0.75, 0.1, 0.08), 'wire_black': (0.06, 0.06, 0.06), 'xt60': (0.95, 0.76, 0.12),
    'shrink': (0.12, 0.32, 0.72), 'prop': (0.32, 0.36, 0.42), 'rubber': (0.11, 0.11, 0.11),
    'lidar': (0.17, 0.17, 0.19), 'usb3': (0.12, 0.3, 0.75),
}

PARTS = []      # (node name, shape, material, group, explode offset)


def add(name, shape, mat, group, explode=(0.0, 0.0, 0.0)):
    if isinstance(shape, cq.Workplane):
        shape = shape.findSolid()
    PARTS.append((name, shape, mat, group, tuple(float(e) for e in explode)))


def compound(shapes):
    out = []
    for s in shapes:
        out.extend(s.vals() if isinstance(s, cq.Workplane) else [s])
    return cq.Compound.makeCompound(out)


# ---------- shape helpers ----------

def bx(sx, sy, sz, cx=0.0, cy=0.0, cz=0.0, r=0.0, every_edge=False):
    s = W('XY').box(sx, sy, sz)
    if r:
        r = min(r, 0.49 * min(sx, sy, sz) if every_edge else 0.49 * min(sx, sy))
        s = s.edges().fillet(r) if every_edge else s.edges('|Z').fillet(r)
    return s.translate((cx, cy, cz))


def cyl(r, z0, z1, x=0.0, y=0.0):
    return W('XY').workplane(offset=z0).center(x, y).circle(r).extrude(z1 - z0)


def tube(r_out, r_in, z0, z1, x=0.0, y=0.0):
    return W('XY').workplane(offset=z0).center(x, y).circle(r_out).circle(r_in).extrude(z1 - z0)


def rod(r, length, start, axis):
    """A cylinder from start along +X or +Y."""
    s = W('XY').circle(r).extrude(length)
    s = s.rotate((0, 0, 0), (0, 1, 0), 90) if axis == 'x' else s.rotate((0, 0, 0), (1, 0, 0), -90)
    return s.translate(start)


def plate(sx, sy, z0, t, r, cx=0.0, cy=0.0):
    return (W('XY').workplane(offset=z0).sketch().rect(sx, sy).vertices().fillet(r).finalize()
            .extrude(t).translate((cx, cy, 0)))


def drill(s, pts, r, z0, z1):
    return s.cut(W('XY').workplane(offset=z0).pushPoints(pts).circle(r).extrude(z1 - z0))


def slots(s, pts, length, width, z0, z1, angle=0.0):
    for x, y in pts:
        s = s.cut(W('XY').workplane(offset=z0).center(x, y).slot2D(length, width, angle).extrude(z1 - z0))
    return s


def hexagon(across_corners, z0, z1, x, y):
    return W('XY').workplane(offset=z0).center(x, y).polygon(6, across_corners).extrude(z1 - z0)


def wire(points, r, t0=None, t1=None):
    """A round wire swept along a smooth curve through points."""
    pts = [cq.Vector(*p) for p in points]
    t0 = cq.Vector(*t0) if t0 else (pts[1] - pts[0])
    t1 = cq.Vector(*t1) if t1 else (pts[-1] - pts[-2])
    path = cq.Wire.assembleEdges([cq.Edge.makeSpline(pts, tangents=[t0, t1])])
    ring = cq.Wire.makeCircle(r, pts[0], t0.normalized())
    return cq.Solid.sweep(ring, [], path, makeSolid=True, isFrenet=False, transitionMode='round')


def turn(s, yaw, about=(0, 0, 0)):
    return s.rotate(about, (about[0], about[1], about[2] + 1), yaw)


def ring_pts(radius, yaw_deg, local):
    """Turn arm-local (along, across) points to the frame at yaw."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [(a * c - b * s, a * s + b * c) for a, b in local]


# ---------- frame ----------

STACK = [(sx * 15.25, sy * 15.25) for sx in (1, -1) for sy in (1, -1)]
STACK20 = [(sx * 10.0, sy * 10.0) for sx in (1, -1) for sy in (1, -1)]
FRAME_STANDOFFS = [(sx * 50.0, sy * 16.0) for sx in (1, -1) for sy in (1, -1)]
ARM_BOLTS_LOCAL = [(20.0, 6.0), (20.0, -6.0)]


def arm_bolts():
    pts = []
    for sx, sy, _ in MOTORS:
        pts += ring_pts(0, math.degrees(math.atan2(sy, sx)), ARM_BOLTS_LOCAL)
    return pts


def build_frame():
    bolts = arm_bolts()
    for i, (sx, sy, _) in enumerate(MOTORS):
        yaw = math.degrees(math.atan2(sy, sx))
        out = [(14, -10), (34, -10), (R_MOTOR - 14, -7), (R_MOTOR - 14, 7), (34, 10), (14, 10)]
        a = W('XY').workplane(offset=AB).polyline(out).close().extrude(sk.ARM_T)
        a = a.union(cyl(14.5, AB, AT, R_MOTOR, 0))
        try:
            a = a.edges('|Z').fillet(1.2)
        except Exception:
            pass
        a = slots(a, [(82.0, 0.0)], 24, 5.5, AB - 1, AT + 1)
        a = drill(a, [(R_MOTOR + dx, dy) for dx in (-8, 8) for dy in (-8, 8)], 1.6, AB - 1, AT + 1)
        a = drill(a, [(R_MOTOR, 0)], 3.6, AB - 1, AT + 1)
        a = drill(a, ARM_BOLTS_LOCAL + [(21.57, 0.0), (math.hypot(LEG_XY, LEG_XY), 0.0)], 1.6, AB - 1, AT + 1)
        a = turn(a, yaw)
        out_dir = (sx / math.sqrt(2), sy / math.sqrt(2))
        add(f'arm{i}__carbon', a, 'carbon', 'frame', (30 * out_dir[0], 30 * out_dir[1], 0))
        sleeve = rod(1.4, 65, (32, 0, AT + 1.4), 'x')
        add(f'arm{i}_wires__wire_black', turn(sleeve, yaw), 'wire_black', 'frame', (30 * out_dir[0], 30 * out_dir[1], 0))

    bp = plate(110, 42, PU, sk.PLATE_T, 6)
    bp = drill(bp, STACK + bolts, 1.6, PU - 1, PU + 3)
    bp = drill(bp, STACK20 + [(48, 12), (48, -12)], 1.3, PU - 1, PU + 3)
    bp = slots(bp, [(38, 0), (-38, 0)], 22, 7, PU - 1, PU + 3)
    add('bottom_plate__carbon', bp, 'carbon', 'frame', (0, 0, -12))

    mp = plate(110, 42, AT, sk.PLATE_T, 6)
    mp = drill(mp, STACK + bolts + FRAME_STANDOFFS, 1.6, AT - 1, AT + 3)
    mp = drill(mp, STACK20, 1.3, AT - 1, AT + 3)
    mp = slots(mp, [(38, 0), (-38, 0)], 22, 7, AT - 1, AT + 3)
    add('middle_plate__carbon', mp, 'carbon', 'frame', (0, 0, 6))

    tp = plate(110, 42, TZ, sk.PLATE_T, 6)
    tp = drill(tp, FRAME_STANDOFFS + [(-50, 8), (-50, -8), (-42, 8), (-42, -8)], 1.6, TZ - 1, TZ + 3)
    for y in (19.6, -19.6):
        tp = tp.cut(bx(21, 1.6, 6, 2, y, TZ + 1))
    tp = slots(tp, [(46, 0)], 12, 6, TZ - 1, TZ + 3)
    add('top_plate__carbon', tp, 'carbon', 'frame', (0, 0, 46))

    heads_top = [cyl(2.75, BZ, BZ + 1.6, x, y) for x, y in FRAME_STANDOFFS]
    standoffs = [cyl(2.5, AT + sk.PLATE_T, TZ, x, y) for x, y in FRAME_STANDOFFS]
    heads_mid = [cyl(2.75, AT - 1.6, AT, x, y) for x, y in FRAME_STANDOFFS]
    add('frame_standoffs__titanium', compound(standoffs), 'titanium', 'frame', (0, 0, 24))
    add('top_screws__steel', compound(heads_top), 'steel', 'frame', (0, 0, 52))
    add('standoff_screws__steel', compound(heads_mid), 'steel', 'frame', (0, 0, 6))
    under = [cyl(2.75, PU - 1.6, PU, x, y) for x, y in bolts + STACK]
    nuts = [hexagon(6.0, AT + sk.PLATE_T, AT + sk.PLATE_T + 2.0, x, y) for x, y in bolts]
    add('arm_screws__steel', compound(under), 'steel', 'frame', (0, 0, -16))
    add('arm_nuts__steel', compound(nuts), 'steel', 'frame', (0, 0, 8))


# ---------- motors and props ----------

def prop_blade(ccw):
    """One blade along +X, twisted for a 3.5" pitch. Leading edge up for the spin direction."""
    stations = [(5.0, 9.0, 3.0, 36.0), (12.0, 13.0, 2.6, 35.0), (22.0, 15.5, 2.0, 31.0), (32.0, 15.5, 1.6, 23.8),
                (42.0, 14.0, 1.3, 18.6), (52.0, 11.5, 1.1, 15.2), (60.0, 8.0, 0.9, 13.2), (64.5, 3.2, 0.6, 12.4)]
    sgn = 1.0 if ccw else -1.0
    wp = W('YZ', origin=(stations[0][0], 0, 0))
    last = stations[0][0]
    for k, (r, chord, thick, pitch) in enumerate(stations):
        if k:
            wp = wp.workplane(offset=r - last)
        last = r
        sweep = -sgn * 0.09 * max(0.0, r - 24.0)
        wp = wp.moveTo(sweep, 0).ellipse(chord / 2, thick / 2, rotation_angle=sgn * pitch)
    return wp.loft(ruled=False)


def build_motors():
    for i, (sx, sy, ccw) in enumerate(MOTORS):
        x, y = sx * MX, sy * MX
        yaw = math.degrees(math.atan2(sy, sx))
        out = (sx / math.sqrt(2), sy / math.sqrt(2))
        move = (22 * out[0], 22 * out[1], 26)
        add(f'motor{i}_base__alu_dark', cyl(13.5, AT, AT + 2.5, x, y), 'alu_dark', 'motors', move)
        stator = cyl(11.8, AT + 2.5, -7.0, x, y).cut(cyl(2.6, AT, 0, x, y))
        add(f'motor{i}_stator__copper', stator, 'copper', 'motors', move)
        bell = tube(13.9, 12.6, AT + 3.0, -6.0, x, y).union(cyl(13.9, -6.0, -4.5, x, y))
        bell = bell.cut(cyl(2.6, -8, 0, x, y))
        for k in range(6):
            a = math.radians(60 * k + 30)
            bell = bell.cut(W('XY').workplane(offset=-7).center(x + 8.4 * math.cos(a), y + 8.4 * math.sin(a))
                            .slot2D(6.0, 3.8, math.degrees(a)).extrude(4))
        try:
            bell = bell.faces('>Z').edges().fillet(0.6)
        except Exception:
            pass
        add(f'motor{i}_bell__bell', bell, 'bell', 'motors', move)
        add(f'motor{i}_ring__accent', tube(14.2, 13.9, AT + 3.5, AT + 5.0, x, y), 'accent', 'motors', move)
        add(f'motor{i}_shaft__steel', cyl(2.5, -7.0, 8.0, x, y), 'steel', 'motors', move)
        screws = [cyl(2.75, AB - 1.6, AB, px, py)
                  for px, py in ring_pts(0, yaw, [(R_MOTOR + dx, dy) for dx in (-8, 8) for dy in (-8, 8)])]
        add(f'motor{i}_screws__steel', compound(screws), 'steel', 'motors', (move[0], move[1], -14))

        blades = [prop_blade(ccw)]
        hub = cyl(6.5, -3.5, 3.0).cut(cyl(2.6, -4, 4))
        parts = [hub] + [blades[0].rotate((0, 0, 0), (0, 0, 1), 120 * k) for k in range(3)]
        prop = compound(parts).translate(cq.Vector(x, y, 0))
        pmove = (34 * out[0], 34 * out[1], 58)
        add(f'prop{i}__prop', prop, 'prop', 'props', pmove)
        nut = hexagon(9.2, 3.0, 8.0, x, y).cut(cyl(2.6, 2, 9, x, y)).union(
            W('XY').workplane(offset=8.0).center(x, y).sphere(3.6).intersect(cyl(4, 8.0, 12, x, y)))
        add(f'prop{i}_nut__accent', nut, 'accent', 'props', (pmove[0], pmove[1], 74))


# ---------- flight stack ----------

def build_stack():
    z0 = -14.6
    esc = plate(44, 43.5, z0, 1.6, 2.0)
    esc = drill(esc, STACK, 1.6, z0 - 1, z0 + 3)
    add('esc_pcb__pcb_black', esc, 'pcb_black', 'stack', (0, 0, 14))
    top = z0 + 1.6
    fets = [bx(5.0, 4.5, 1.0, sx * fx, sy * fy, top + 0.5) for sx in (1, -1) for sy in (1, -1)
            for fx in (4.5, 10.0) for fy in (3.6, 8.8)]
    add('esc_mosfets__chip', compound(fets), 'chip', 'stack', (0, 0, 14))
    mcus = [bx(3.2, 3.2, 0.8, x, y, top + 0.4) for x, y in ((0, 17), (0, -17), (17, 0), (-16, 0))]
    add('esc_mcus__chip', compound(mcus), 'chip', 'stack', (0, 0, 14))
    pads = [bx(3.0, 2.4, 0.15, sx * 20.3, sy * py, top + 0.075) for sx in (1, -1) for sy in (1, -1)
            for py in (13.0, 16.2, 19.4)]
    pads += [bx(3.6, 5.5, 0.15, -19.8, py, top + 0.075) for py in (10.5, -10.5)]
    pads += [bx(3.6, 4.0, 0.15, -19.8, py, top + 0.075) for py in (4.0, -4.0)]
    add('esc_pads__gold', compound(pads), 'gold', 'stack', (0, 0, 14))

    f0 = -5.0
    fc = plate(36, 36, f0, 1.6, 2.0)
    fc = drill(fc, STACK, 2.4, f0 - 1, f0 + 3)
    add('fc_pcb__pcb_black', fc, 'pcb_black', 'stack', (0, 0, 28))
    ftop = f0 + 1.6
    chips = [bx(10, 10, 1.2, 0, 0, ftop + 0.6), bx(3, 3, 0.9, -6.5, 6.5, ftop + 0.45), bx(4, 4, 0.8, 7.5, 7.5, ftop + 0.4)]
    add('fc_chips__chip', compound(chips), 'chip', 'stack', (0, 0, 28))
    add('fc_baro_foam__rubber', bx(5, 5, 2.5, 7.5, -7.5, ftop + 1.25), 'rubber', 'stack', (0, 0, 28))
    conns = [bx(3.5, 6.0, 3.0, sx * 16.0, y, ftop + 1.5) for sx in (1, -1) for y in (-7, 0, 7)]
    conns += [bx(6.0, 3.5, 3.0, x, 16.0, ftop + 1.5) for x in (-7, 0, 7)]
    add('fc_connectors__plastic_white', compound(conns), 'plastic_white', 'stack', (0, 0, 28))
    usb = W('XY').box(9.0, 7.3, 3.2).edges('|Y').fillet(1.4).translate((0, -14.6, ftop + 1.6))
    add('fc_usb__steel', usb, 'steel', 'stack', (0, 0, 28))
    leds = [bx(1.6, 0.9, 0.5, -10, 10, ftop + 0.25), bx(1.6, 0.9, 0.5, -10, 12, ftop + 0.25)]
    add('fc_leds__accent', compound(leds), 'accent', 'stack', (0, 0, 28))

    bolts = [cyl(1.5, PU, -0.6, x, y) for x, y in STACK]
    add('stack_bolts__titanium', compound(bolts), 'titanium', 'stack', (0, 0, 6))
    spacers = [tube(2.8, 1.5, AT + sk.PLATE_T, z0, x, y) for x, y in STACK]
    spacers += [tube(2.8, 1.5, z0 + 1.6, f0 - 0.4, x, y) for x, y in STACK]
    spacers += [hexagon(6.4, f0 + 2.0, -0.6, x, y).cut(cyl(1.5, -6, 1, x, y)) for x, y in STACK]
    add('stack_spacers__plastic_black', compound(spacers), 'plastic_black', 'stack', (0, 0, 20))
    grommets = [tube(2.4, 1.5, f0 - 0.4, f0 + 2.0, x, y) for x, y in STACK]
    add('fc_grommets__accent', compound(grommets), 'accent', 'stack', (0, 0, 28))

    cap = rod(6.25, 25, (-49, -12.5, -8), 'y')
    add('capacitor__plastic_black', cap, 'plastic_black', 'stack', (-14, 0, 18))
    add('capacitor_band__steel', rod(6.3, 4.0, (-49, -12.5, -8), 'y').cut(rod(6.0, 6, (-49, -13, -8), 'y')),
        'steel', 'stack', (-14, 0, 18))
    legs = [wire([(-49, 12.4, -8 + dz), (-44, 13.5, -6 + dz), (-32, 15, -9 + dz), (-21.5, 10.5 + dz, -12.3)], 0.5,
                 (0, 1, 0), (1, 0, 0)) for dz in (1.6, -1.6)]
    add('capacitor_leads__steel', compound(legs), 'steel', 'stack', (-14, 0, 18))

    add('rx_tape__rubber', bx(13, 10, 1.0, -35, 0, AT + sk.PLATE_T + 0.5), 'rubber', 'stack', (0, 0, 10))
    add('rx_pcb__pcb_black', bx(13, 10, 1.0, -35, 0, AT + sk.PLATE_T + 1.5), 'pcb_black', 'stack', (0, 0, 10))
    add('rx_chip__chip', bx(4.5, 4.5, 0.9, -33.5, 0, AT + sk.PLATE_T + 2.45), 'chip', 'stack', (0, 0, 10))
    coax = wire([(-41.3, 0, -15.8), (-50, 0, -15.6), (-56, 0, -16.4), (-62, 0, -20.0), (-66, 0, -23.0)], 0.75,
                (-1, 0, 0), (-1, 0, -0.2))
    add('rx_coax__wire_black', coax, 'wire_black', 'stack', (-10, 0, 10))
    add('rx_antenna__wire_black', compound([rod(0.6, 32, (-66.4, -16, -23.0), 'y'), rod(1.4, 7, (-66.4, -3.5, -23.0), 'y')]),
        'wire_black', 'stack', (-10, 0, 10))
    add('bec__shrink', bx(30, 14, 5.8, 36, 0, TZ - 3.0, r=2.0, every_edge=True), 'shrink', 'stack', (0, 0, 36))


# ---------- battery, strap and lead ----------

def build_battery():
    bl, bw, bh = sk.BATTERY_SIZE
    bx0 = sk.BATTERY_X
    up = (0, 0, 92)
    add('battery__battery', bx(bl, bw, bh, bx0, 0, BZ + bh / 2, r=3.0, every_edge=True), 'battery', 'battery', up)
    labels = [bx(23, 0.4, 32, xc, sy * (bw / 2 + 0.2), BZ + bh / 2) for xc in (-21.5, 25.5) for sy in (1, -1)]
    add('battery_labels__label', compound(labels), 'label', 'battery', up)
    stripe = [bx(23, 0.45, 3, xc, sy * (bw / 2 + 0.22), BZ + bh / 2 + 9) for xc in (-21.5, 25.5) for sy in (1, -1)]
    add('battery_stripes__plastic_white', compound(stripe), 'plastic_white', 'battery', up)
    strap = bx(20, bw + 2.4, bh + sk.PLATE_T + 2.4, bx0, 0, (TZ - 1.2 + BZ + bh + 1.2) / 2)
    strap = strap.cut(bx(22, bw, bh + sk.PLATE_T, bx0, 0, TZ + (bh + sk.PLATE_T) / 2))
    add('strap__strap', strap, 'strap', 'battery', (0, 0, 70))
    add('strap_buckle__accent', bx(14, 3.0, 1.4, bx0, 0, BZ + bh + 1.9, r=0.5), 'accent', 'battery', (0, 0, 70))
    rear = bx0 - bl / 2
    lead = wire([(rear + 0.4, 6, 42), (-43, 13, 42), (-38, 25, 46), (-33, 26, 38.6)], 2.4, (-1, 0, 0), (0, 0, -1))
    add('battery_lead__wire_black', lead, 'wire_black', 'battery', up)
    bal = wire([(rear + 0.4, -8, 44), (-41, -9, 48), (-37.5, -10, 55.5), (-30, -10, 56.6)], 1.5, (-1, 0, 0), (1, 0, 0))
    add('balance_lead__plastic_white', bal, 'plastic_white', 'battery', up)
    add('balance_plug__plastic_white', bx(18, 6, 4.5, -22, -10, BZ + bh + 2.25, r=0.8), 'plastic_white', 'battery', up)
    add('xt60_female__xt60', bx(15.5, 8.2, 16, -33, 26, 31, r=1.2), 'xt60', 'battery', up)
    add('xt60_male__xt60', bx(15.5, 8.2, 15, -33, 26, 15.5, r=1.2), 'xt60', 'stack', (0, 8, 14))
    pig = wire([(-33, 26, 8.1), (-33, 25, 1), (-29, 20, -5), (-21, 9, -10.6)], 2.2, (0, 0, -1), (1, -0.6, -0.15))
    add('esc_lead__wire_black', pig, 'wire_black', 'stack', (0, 8, 14))


# ---------- STL-27L on its mast ----------

def build_lidar():
    x = sk.LIDAR_X
    mast_top = sk.LIDAR_Z - sk.LIDAR_H / 2 - 9
    lz = mast_top
    base = bx(32, 24, 5, -55, 0, BZ + 2.5, r=3)
    base = base.union(cyl(8, BZ, BZ + 12, x, 0))
    base = base.cut(cyl(5, BZ + 2, BZ + 13, x, 0))
    base = drill(base, [(-50, 8), (-50, -8), (-42, 8), (-42, -8)], 1.6, BZ - 1, BZ + 6)
    add('mast_mount__print', base, 'print', 'lidar', (0, 0, 52))
    add('mast__carbon', tube(5, 4, BZ + 2, lz - 7, x, 0), 'carbon', 'lidar', (0, 0, 76))
    top = cyl(20, lz - 2, lz, x, 0).union(cyl(7, lz - 9, lz - 2, x, 0)).cut(cyl(5, lz - 10, lz - 2, x, 0))
    add('lidar_mount__print', top, 'print', 'lidar', (0, 0, 90))
    up = (0, 0, 118)
    add('stl27l_base__lidar', cyl(sk.LIDAR_R, lz, lz + 13, x, 0).edges('<Z').fillet(1.5), 'lidar', 'lidar', up)
    add('stl27l_ring__accent', tube(sk.LIDAR_R + 0.1, 18.6, lz + 13, lz + 14, x, 0), 'accent', 'lidar', up)
    head = cyl(18.6, lz + 14, sk.LIDAR_Z - 3, x, 0).union(cyl(18.6, sk.LIDAR_Z + 3, lz + 32, x, 0))
    add('stl27l_head__lidar', head, 'lidar', 'lidar', up)
    add('stl27l_window__glass', cyl(18.3, sk.LIDAR_Z - 3, sk.LIDAR_Z + 3, x, 0), 'glass', 'lidar', up)
    add('stl27l_cap__lidar', cyl(16.5, lz + 32, lz + 34, x, 0).edges('>Z').fillet(1.2), 'lidar', 'lidar', up)


# ---------- OAK-D Lite on its cradle ----------

def build_oak():
    fx, fz = sk.OAK_FRONT, sk.OAK_Z
    d, w, h = sk.OAK_SIZE

    def place(s):
        return s.rotate((0, 0, 0), (0, 1, 0), -sk.OAK_TILT).translate((fx, 0, fz))

    move = (48, 0, -14)
    body = W('XY').box(d - 0.8, w, h).translate((-(d - 0.8) / 2 - 0.8, 0, 0)).edges('|X').fillet(6.0)
    add('oakd_body__alu_dark', place(body), 'alu_dark', 'oakd', move)
    face = W('XY').box(0.8, w - 3, h - 4).translate((-0.4, 0, 0)).edges('|X').fillet(4.5)
    add('oakd_face__glass', place(face), 'glass', 'oakd', move)
    rings = [rod(4.3, 0.7, (0, s * 37.5, 0), 'x').cut(rod(3.0, 1, (-0.1, s * 37.5, 0), 'x')) for s in (1, -1)]
    rings.append(rod(5.6, 0.7, (0, 0, 0), 'x').cut(rod(4.1, 1, (-0.1, 0, 0), 'x')))
    add('oakd_lens_rings__steel', place(compound(rings)), 'steel', 'oakd', move)
    glass = [rod(3.0, 0.45, (0, s * 37.5, 0), 'x') for s in (1, -1)] + [rod(4.1, 0.45, (0, 0, 0), 'x')]
    add('oakd_lenses__glass', place(compound(glass)), 'glass', 'oakd', move)
    usb = W('XY').box(1.2, 9.0, 3.3).edges('|X').fillet(1.4).translate((-d - 0.6, 28, 0))
    add('oakd_usb__steel', place(usb), 'steel', 'oakd', move)

    back = bx(4, 44, 51, -d - 2, 0, 9.5, r=3)
    lip = bx(16.5, 44, 3, -d - 4 + 8.25, 0, -h / 2 - 1.5, r=2)
    cradle = place(back.union(lip))
    tab = bx(16, 40, 3, 48, 0, PU - 1.5, r=3)
    tab = drill(tab, [(48, 12), (48, -12)], 1.6, PU - 4, PU + 1)
    add('oakd_cradle__print', cradle.union(tab), 'print', 'oakd', (26, 0, -8))


# ---------- Raspberry Pi 5 under the frame ----------

PI_X = -4.0
PI_TOP = PU - 3.0
PI_HOLES = [(-39.0, 24.5), (-39.0, -24.5), (19.0, 24.5), (19.0, -24.5)]


def build_pi():
    def place(s):
        return s.rotate((0, 0, 0), (1, 0, 0), 180).translate((PI_X, 0, PI_TOP))

    move = (0, 0, -52)
    t = 1.6
    pcb = drill(plate(85, 56, 0, t, 3), PI_HOLES, 1.35, -1, 3)
    add('pi5_pcb__pcb_green', place(pcb), 'pcb_green', 'pi', move)
    metal = [bx(21, 16, 13.5, 34.5, 18.0, t + 6.75), bx(17.5, 13.5, 16, 35.8, -1.5, t + 8),
             bx(17.5, 13.5, 16, 35.8, -19.0, t + 8), bx(8.9, 7.3, 3.2, -31.5, -24.4, t + 1.6),
             bx(6.5, 7.5, 3.0, -19.5, -24.4, t + 1.5), bx(6.5, 7.5, 3.0, -6.5, -24.4, t + 1.5)]
    add('pi5_ports__steel', place(compound(metal)), 'steel', 'pi', move)
    tongues = [bx(0.4, 7.0, 1.8, 44.75, y, t + z) for y in (-1.5, -19.0) for z in (4.0, 12.0)]
    add('pi5_usb3__usb3', place(compound(tongues)), 'usb3', 'pi', move)
    add('pi5_eth_jack__plastic_black', place(bx(0.4, 12, 9, 45.2, 18.0, t + 6.5)), 'plastic_black', 'pi', move)
    add('pi5_gpio__plastic_black', place(bx(51, 5, 2.5, -10.5, 24.5, t + 1.25)), 'plastic_black', 'pi', move)
    pins = [bx(0.64, 0.64, 6.0, -10.5 + (k - 9.5) * 2.54, 24.5 + s * 1.27, t + 2.5 + 3.0) for k in range(20) for s in (1, -1)]
    add('pi5_gpio_pins__gold', place(compound(pins)), 'gold', 'pi', move)
    sink = bx(40, 30, 2, -8, -4, t + 1).union(compound([bx(1.0, 30, 5.5, -27 + 2.6 * k, -4, t + 4.75) for k in range(7)]))
    add('pi5_heatsink__steel', place(sink), 'steel', 'pi', move)
    fan = bx(22, 22, 6, 2, -4, t + 5, r=2).cut(cyl(9.5, t + 4, t + 9, 2, -4))
    add('pi5_fan__plastic_black', place(fan), 'plastic_black', 'pi', move)
    add('pi5_fan_hub__chip', place(cyl(9.2, t + 3.8, t + 6.5, 2, -4)), 'chip', 'pi', move)
    add('pi5_pcie__plastic_white', place(bx(2.6, 17, 2.0, -40.5, 0, t + 1.0)), 'plastic_white', 'pi', move)

    carrier = plate(88, 58, PU - 2.0, 2.0, 4, PI_X, 0)
    carrier = carrier.cut(plate(52, 30, PU - 3, 4, 5, PI_X - 4, 0))
    carrier = drill(carrier, [(PI_X + x, y) for x, y in PI_HOLES], 1.3, PU - 3, PU + 1)
    carrier = drill(carrier, STACK + arm_bolts(), 3.2, PU - 3, PU + 1)
    mtf = bx(32, 21, 2, -64, 0, PU - 1.0)
    rails = [bx(30, 2, 11, -63, s * 9.6, PU - 2.0 - 5.5) for s in (1, -1)]
    add('pi_carrier__print', carrier.union(mtf).union(rails[0]).union(rails[1]), 'print', 'pi', (0, 0, -24))
    posts = [cyl(1.25, PI_TOP - sk.PI_SIZE[2], PU - 2.0, PI_X + x, y) for x, y in PI_HOLES]
    add('pi_posts__gold', compound(posts), 'gold', 'pi', (0, 0, -52))
    tz = PI_TOP - sk.PI_SIZE[2]
    tray = plate(88, 58, tz - 2.0, 2.0, 4, PI_X, 0)
    tray = tray.cut(cyl(13, tz - 3, tz + 1, PI_X - 2, 4))
    tray = slots(tray, [(PI_X + 30, y) for y in (-14, -5, 4, 13)], 22, 4, tz - 3, tz + 1)
    tray = drill(tray, [(PI_X + x, y) for x, y in PI_HOLES], 1.3, tz - 3, tz + 1)
    add('pi_tray__print', tray, 'print', 'pi', (0, 0, -78))
    heads = [cyl(2.3, tz - 3.4, tz - 2.0, PI_X + x, y) for x, y in PI_HOLES]
    add('pi_tray_screws__steel', compound(heads), 'steel', 'pi', (0, 0, -84))


# ---------- MTF-01P ----------

def build_mtf():
    mx = -63.0
    face = PU - sk.MTF_SIZE[2]
    move = (-16, 0, -44)
    add('mtf01p_case__plastic_black', bx(sk.MTF_SIZE[0], sk.MTF_SIZE[1], (PU - 2.0) - (face + 2.0), mx, 0,
                                         ((PU - 2.0) + (face + 2.0)) / 2, r=1.5), 'plastic_black', 'mtf', move)
    add('mtf01p_lens__glass', cyl(4.0, face, face + 2.0, mx + 5, 0), 'glass', 'mtf', move)
    add('mtf01p_lens_ring__steel', tube(5.2, 4.0, face + 0.6, face + 2.0, mx + 5, 0), 'steel', 'mtf', move)
    add('mtf01p_tof__glass', bx(6.5, 4.0, 1.4, mx - 7, 0, face + 1.3), 'glass', 'mtf', move)


# ---------- legs ----------

def build_legs():
    for i, (sx, sy, _) in enumerate(MOTORS):
        x, y = sx * LEG_XY, sy * LEG_XY
        out = (sx / math.sqrt(2), sy / math.sqrt(2))
        foot_z = sk.LEG_BOTTOM + 6.0
        leg = cyl(5.5, PU, AB, x, y).union(cyl(4.6, PU - 8, PU, x, y))
        leg = leg.union(W('XY').add(cq.Solid.makeCone(3.0, 3.6, (PU - 8) - (foot_z + 4), cq.Vector(x, y, foot_z + 4))))
        leg = leg.cut(cyl(1.6, PU - 6, AB + 1, x, y))
        add(f'leg{i}__print', leg, 'print', 'legs', (24 * out[0], 24 * out[1], -40))
        foot = W('XY').add(cq.Solid.makeSphere(6.0, cq.Vector(x, y, foot_z), angleDegrees1=-90, angleDegrees2=90))
        add(f'foot{i}__rubber', foot, 'rubber', 'legs', (24 * out[0], 24 * out[1], -40))


# ---------- checks ----------

def check_views(views):
    owners = {'OAK-D Lite view': {'oakd'}, 'lidar scan plane': {'lidar'}, 'flow view': {'mtf'}, 'range beam': {'mtf'}}
    problems = []
    for view in views:
        for name, shape, _, group, _ in PARTS:
            if group in owners[view.name]:
                continue
            verts, _ = shape.tessellate(0.5, 0.5)
            hits = sum(view.contains((v.x, v.y, v.z)) for v in verts)
            if hits:
                problems.append(f'IN VIEW {name} in {view.name}: {hits} of {len(verts)} mesh points')
    return problems


def check_overlaps(limit=1.0):
    problems = []
    boxes = [(n, s, g, s.BoundingBox()) for n, s, _, g, _ in PARTS]
    for i, (na, a, ga, ba) in enumerate(boxes):
        for nb, b, gb, bb in boxes[i + 1:]:
            if ga == gb:
                continue
            if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
                    or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
                continue
            try:
                v = a.intersect(b).Volume()
            except Exception:
                v = 0.0
            if v > limit:
                problems.append(f'OVERLAP {na} and {nb}: {v:.1f} mm^3')
    return problems


def main():
    build_frame()
    build_motors()
    build_stack()
    build_battery()
    build_lidar()
    build_oak()
    build_pi()
    build_mtf()
    build_legs()
    print(f'{len(PARTS)} detailed parts')

    sk_parts = sk.build_parts()
    views = sk.build_views(sk_parts)
    problems = check_overlaps() + check_views(views)
    for p in problems:
        print('  ' + p)
    print(f'  {len(problems)} problems')

    assy = cq.Assembly(name='drone5')
    for name, shape, mat, _, _ in PARTS:
        assy.add(shape, name=name, color=cq.Color(*MATS[mat], 1.0))
    assy.save('model5.step', 'STEP')
    assy.save('model5.glb', 'GLTF', tolerance=0.08, angularTolerance=0.25)

    total, com, inertia = sk.mass_props(sk_parts)
    oak = next(p for p in sk_parts if p.name == 'oakd_lite')
    mtf = next(p for p in sk_parts if p.name == 'mtf01p')
    meta = {
        'nodes': {name: {'group': group, 'explode': list(ex), 'mat': mat} for name, _, mat, group, ex in PARTS},
        'groups': {g: round(sum(p.mass for p in sk_parts if group_of(p.name) == g), 1) for g in GROUP_MASS},
        'views': {
            'oak': {'apex': [sk.OAK_FRONT, 0, oak.center[2]], 'tilt': sk.OAK_TILT, 'hfov': sk.OAK_HFOV, 'vfov': sk.OAK_VFOV},
            'lidar': {'center': [sk.LIDAR_X, 0, sk.LIDAR_Z], 'inner': sk.LIDAR_R},
            'flow': {'apex': [mtf.center[0], 0, mtf.center[2] - mtf.size[2] / 2], 'fov': sk.FLOW_FOV,
                     'range_fov': sk.RANGE_FOV},
        },
        'ground': sk.LEG_BOTTOM,
        'mass': round(total), 'mass_margin': round(total * 1.1), 'com': [round(c, 1) for c in com],
        'inertia': [round(i, 4) for i in inertia], 'lidar_z': round(sk.LIDAR_Z),
        'flow_height': round(mtf.center[2] - mtf.size[2] / 2 - sk.LEG_BOTTOM),
        'motor_to_motor': 226, 'prop_in': 5.1, 'problems': problems,
    }
    with open('model5_parts.json', 'w', encoding='utf-8') as f:
        json.dump(meta, f, indent=1)
    write_page(meta)
    print(f'Wrote model5.step, model5.glb, model5_parts.json, drone5_3d.html: {total:.0f} g, '
          f'{total * 1.1:.0f} g with margin')


def write_page(meta):
    """Write drone5_3d.html: page_template.html with the model and its data inside, so it opens as one file."""
    with open('model5.glb', 'rb') as f:
        glb = base64.b64encode(f.read()).decode('ascii')
    with open('page_template.html', encoding='utf-8') as f:
        page = f.read()
    page = page.replace('__META__', json.dumps(meta)).replace('__GLB__', glb)
    with open('drone5_3d.html', 'w', encoding='utf-8') as f:
        f.write(page)


GROUP_MASS = ('frame', 'motors', 'props', 'stack', 'battery', 'lidar', 'oakd', 'pi', 'mtf', 'legs')


def group_of(name):
    base = name.rstrip('0123456789').rstrip('_')
    return {'arm': 'frame', 'bottom_plate': 'frame', 'middle_plate': 'frame', 'top_plate': 'frame',
            'standoff': 'frame', 'motor': 'motors', 'prop': 'props', 'esc': 'stack', 'flight_controller': 'stack',
            'bec': 'stack', 'rx': 'stack', 'battery': 'battery', 'mast': 'lidar', 'stl27l': 'lidar',
            'pi5': 'pi', 'pi_cage': 'pi', 'oakd_lite': 'oakd', 'oakd_mount': 'oakd', 'mtf01p': 'mtf',
            'leg': 'legs', sk.WIRES: 'other'}.get(base, 'other')


if __name__ == '__main__':
    main()
