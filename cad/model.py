"""Detailed model of the 7" drone, built on the layout in skeleton.py.

Every part sits in the space skeleton.py gives it, so the layout, view and mass checks there still apply.
This adds the real shapes: cut carbon plates and arms, 2807 motors, 7x3.5 two-blade props, the flight stack,
the Jetson Orin Nano Super dev kit on its damped deck, the printed mast bridge, mast and camera collar, the
D500 lidar, both IMX219-200 side cameras, the OAK-D Lite, the MTF-01P, the 4S battery on its strapped tray with
its leads, and the legs. It then checks the detailed parts against each other and against every sensor view.

Writes model.step (open in any CAD program), model.glb and model_parts.json, and drone_3d.html (the 3D page,
page_template.html with the model inside, so it opens as one file).
Run with cadquery installed, from this folder: python3 model.py
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
TZ = sk.TOP_Z                        # underside of the frame's top plate
DZ = sk.DECK_Z                       # underside of the Jetson deck
BZ = sk.BRIDGE_Z                     # underside of the mast bridge
LB = sk.LIDAR_BASE                   # underside of the lidar
BAT_X = sk.BATTERY_X
BAT_TOP = PU - 3.0
BAT_BOTTOM = BAT_TOP - sk.BATTERY_SIZE[2]
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
    'lidar': (0.17, 0.17, 0.19), 'usb3': (0.12, 0.3, 0.75), 'alu': (0.78, 0.79, 0.8),
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


def rod_dir(r, start, direction, length):
    d = cq.Vector(*direction).normalized()
    return cq.Solid.makeCylinder(r, length, cq.Vector(*start), d)


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


def ring_pts(yaw_deg, local):
    """Turn arm-local (along, across) points to the frame at yaw."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [(a * c - b * s, a * s + b * c) for a, b in local]


def outward(sx, sy, k):
    return (k * sx / math.sqrt(2), k * sy / math.sqrt(2))


# ---------- frame ----------

STACK = [(sx * 15.25, sy * 15.25) for sx in (1, -1) for sy in (1, -1)]
STACK20 = [(sx * 10.0, sy * 10.0) for sx in (1, -1) for sy in (1, -1)]
FRAME_STANDOFFS = [(sx * 60.0, sy * 18.0) for sx in (1, -1) for sy in (1, -1)]
DAMPERS = [(sx * 45.0, sy * 18.0) for sx in (1, -1) for sy in (1, -1)]
ARM_BOLTS_LOCAL = [(22.0, 7.0), (22.0, -7.0)]
TRAY_BOLTS = [(sx * 40.0, sy * 15.0) for sx in (1, -1) for sy in (1, -1)]
OAK_TAB_BOLTS = [(62.5, 12.0), (62.5, -12.0)]


def arm_bolts():
    pts = []
    for sx, sy, _ in MOTORS:
        pts += ring_pts(math.degrees(math.atan2(sy, sx)), ARM_BOLTS_LOCAL)
    return pts


def build_frame():
    bolts = arm_bolts()
    leg_r = math.hypot(sk.LEG_XY, sk.LEG_XY)
    for i, (sx, sy, _) in enumerate(MOTORS):
        yaw = math.degrees(math.atan2(sy, sx))
        out = [(14, -11), (40, -11), (R_MOTOR - 18, -8), (R_MOTOR - 18, 8), (40, 11), (14, 11)]
        a = W('XY').workplane(offset=AB).polyline(out).close().extrude(sk.ARM_T)
        a = a.union(cyl(18.0, AB, AT, R_MOTOR, 0))
        try:
            a = a.edges('|Z').fillet(1.5)
        except Exception:
            pass
        a = slots(a, [(72.0, 0.0)], 34, 6.5, AB - 1, AT + 1)
        a = drill(a, [(R_MOTOR + dx, dy) for dx in (-9.5, 9.5) for dy in (-9.5, 9.5)], 1.6, AB - 1, AT + 1)
        a = drill(a, [(R_MOTOR, 0)], 4.0, AB - 1, AT + 1)
        a = drill(a, ARM_BOLTS_LOCAL + [(21.57, 0.0), (leg_r, 0.0)], 1.6, AB - 1, AT + 1)
        a = turn(a, yaw)
        o = outward(sx, sy, 40)
        add(f'arm{i}__carbon', a, 'carbon', 'frame', (o[0], o[1], 0))
        sleeve = rod(1.6, 100, (42, 0, AT + 1.6), 'x')
        add(f'arm{i}_wires__wire_black', turn(sleeve, yaw), 'wire_black', 'frame', (o[0], o[1], 0))

    bp = plate(*sk.BOTTOM_PLATE, PU, sk.PLATE_T, 8)
    bp = drill(bp, STACK + bolts + TRAY_BOLTS + OAK_TAB_BOLTS, 1.6, PU - 1, PU + 3)
    bp = drill(bp, STACK20, 1.3, PU - 1, PU + 3)
    bp = slots(bp, [(40, 0), (-40, 0)], 26, 8, PU - 1, PU + 3)
    add('bottom_plate__carbon', bp, 'carbon', 'frame', (0, 0, -14))

    mp = plate(*sk.BOTTOM_PLATE, AT, sk.PLATE_T, 8)
    mp = drill(mp, STACK + bolts + FRAME_STANDOFFS, 1.6, AT - 1, AT + 3)
    mp = drill(mp, STACK20, 1.3, AT - 1, AT + 3)
    mp = slots(mp, [(40, 0), (-40, 0)], 26, 8, AT - 1, AT + 3)
    add('middle_plate__carbon', mp, 'carbon', 'frame', (0, 0, 6))

    tp = plate(*sk.TOP_PLATE, TZ, sk.PLATE_T, 8)
    tp = drill(tp, FRAME_STANDOFFS + DAMPERS, 1.6, TZ - 1, TZ + 3)
    tp = slots(tp, [(0, 0)], 50, 14, TZ - 1, TZ + 3)
    add('top_plate__carbon', tp, 'carbon', 'frame', (0, 0, 34))

    standoffs = [cyl(2.5, AT + sk.PLATE_T, TZ, x, y) for x, y in FRAME_STANDOFFS]
    add('frame_standoffs__titanium', compound(standoffs), 'titanium', 'frame', (0, 0, 18))
    add('top_screws__steel', compound([cyl(2.75, TZ + sk.PLATE_T, TZ + sk.PLATE_T + 1.6, x, y) for x, y in FRAME_STANDOFFS]),
        'steel', 'frame', (0, 0, 40))
    add('standoff_screws__steel', compound([cyl(2.75, AT - 1.6, AT, x, y) for x, y in FRAME_STANDOFFS]),
        'steel', 'frame', (0, 0, 6))
    under = [hexagon(6.0, PU - 2.0, PU, x, y) for x, y in bolts] + [cyl(2.75, PU - 1.6, PU, x, y) for x, y in STACK]
    add('arm_nuts__steel', compound(under), 'steel', 'frame', (0, 0, -18))
    add('arm_screws__steel', compound([cyl(2.75, AT + sk.PLATE_T, AT + sk.PLATE_T + 1.6, x, y) for x, y in bolts]),
        'steel', 'frame', (0, 0, 8))


# ---------- motors and props ----------

def prop_blade(ccw):
    """One blade along +X, twisted for a 3.5" pitch. Leading edge up for the spin direction."""
    stations = [(6.0, 11.0, 3.2, 30.0), (15.0, 17.0, 2.8, 32.0), (28.0, 21.0, 2.2, 26.8), (42.0, 20.0, 1.8, 18.6),
                (56.0, 17.5, 1.5, 14.2), (70.0, 14.0, 1.2, 11.4), (80.0, 10.0, 1.0, 10.0), (86.0, 6.0, 0.8, 9.3),
                (88.6, 2.4, 0.6, 9.0)]
    if ccw:
        # Mirror the clockwise blade: lofting the CCW twist directly folds the surface over itself
        return prop_blade(False).mirror('XZ')
    sgn = -1.0
    wp = W('YZ', origin=(stations[0][0], 0, 0))
    last = stations[0][0]
    for k, (r, chord, thick, pitch) in enumerate(stations):
        if k:
            wp = wp.workplane(offset=r - last)
        last = r
        sweep = -sgn * 0.08 * max(0.0, r - 34.0)
        wp = wp.moveTo(sweep, 0).ellipse(chord / 2, thick / 2, rotation_angle=sgn * pitch)
    return wp.loft(ruled=False)


def build_motors():
    for i, (sx, sy, ccw) in enumerate(MOTORS):
        x, y = sx * MX, sy * MX
        yaw = math.degrees(math.atan2(sy, sx))
        o = outward(sx, sy, 30)
        move = (o[0], o[1], 34)
        add(f'motor{i}_base__alu_dark', cyl(17.0, AT, AT + 3.0, x, y), 'alu_dark', 'motors', move)
        add(f'motor{i}_stator__copper', cyl(15.0, AT + 3.0, -7.5, x, y).cut(cyl(2.6, AT, 0, x, y)), 'copper', 'motors', move)
        bell = tube(17.5, 16.2, AT + 3.5, -6.0, x, y).union(cyl(17.5, -6.0, -4.5, x, y))
        bell = bell.cut(cyl(2.6, -8, 0, x, y))
        for k in range(6):
            a = math.radians(60 * k + 30)
            bell = bell.cut(W('XY').workplane(offset=-7).center(x + 10.8 * math.cos(a), y + 10.8 * math.sin(a))
                            .slot2D(8.0, 5.0, math.degrees(a)).extrude(4))
        try:
            bell = bell.faces('>Z').edges().fillet(0.6)
        except Exception:
            pass
        add(f'motor{i}_bell__bell', bell, 'bell', 'motors', move)
        add(f'motor{i}_ring__accent', tube(17.8, 17.5, AT + 4.0, AT + 5.5, x, y), 'accent', 'motors', move)
        add(f'motor{i}_shaft__steel', cyl(2.5, -7.5, 9.9, x, y), 'steel', 'motors', move)
        screws = [cyl(2.75, AB - 1.6, AB, px, py)
                  for px, py in ring_pts(yaw, [(R_MOTOR + dx, dy) for dx in (-9.5, 9.5) for dy in (-9.5, 9.5)])]
        add(f'motor{i}_screws__steel', compound(screws), 'steel', 'motors', (move[0], move[1], -20))

        blade = prop_blade(ccw)
        hub = cyl(7.5, -3.0, 3.5).cut(cyl(2.6, -4, 4))
        prop = compound([hub, blade, blade.rotate((0, 0, 0), (0, 0, 1), 180)]).translate(cq.Vector(x, y, 1.5))   # hub seated on the bell
        po = outward(sx, sy, 50)
        add(f'prop{i}__prop', prop, 'prop', 'props', (po[0], po[1], 74))
        nut = hexagon(9.2, 5.0, 10.0, x, y).cut(cyl(2.6, 3, 11, x, y)).union(
            W('XY').workplane(offset=10.0).center(x, y).sphere(3.6).intersect(cyl(4, 10.0, 14.5, x, y)))
        add(f'prop{i}_nut__accent', nut, 'accent', 'props', (po[0], po[1], 92))


# ---------- flight stack, BEC and receiver ----------

def build_stack():
    z0 = -16.0
    esc = drill(plate(44, 43.5, z0, 1.6, 2.0), STACK, 1.6, z0 - 1, z0 + 3)
    up = (0, 0, 10)
    add('esc_pcb__pcb_black', esc, 'pcb_black', 'stack', up)
    top = z0 + 1.6
    fets = [bx(5.0, 4.5, 1.0, sx * fx, sy * fy, top + 0.5) for sx in (1, -1) for sy in (1, -1)
            for fx in (4.5, 10.0) for fy in (3.6, 8.8)]
    add('esc_mosfets__chip', compound(fets), 'chip', 'stack', up)
    add('esc_mcus__chip', compound([bx(3.2, 3.2, 0.8, x, y, top + 0.4) for x, y in ((0, 17), (0, -17), (17, 0), (-16, 0))]),
        'chip', 'stack', up)
    pads = [bx(3.0, 2.4, 0.15, sx * 20.3, sy * py, top + 0.075) for sx in (1, -1) for sy in (1, -1)
            for py in (13.0, 16.2, 19.4)]
    pads += [bx(3.6, 5.5, 0.15, -19.8, py, top + 0.075) for py in (9.0, -9.0)]
    add('esc_pads__gold', compound(pads), 'gold', 'stack', up)

    f0 = -1.0
    fup = (0, 0, 24)
    add('fc_pcb__pcb_black', drill(plate(36, 36, f0, 1.6, 2.0), STACK, 2.4, f0 - 1, f0 + 3), 'pcb_black', 'stack', fup)
    ftop = f0 + 1.6
    chips = [bx(10, 10, 1.2, 0, 0, ftop + 0.6), bx(3, 3, 0.9, -6.5, 6.5, ftop + 0.45), bx(4, 4, 0.8, 7.5, 7.5, ftop + 0.4)]
    add('fc_chips__chip', compound(chips), 'chip', 'stack', fup)
    add('fc_baro_foam__rubber', bx(5, 5, 2.5, 7.5, -7.5, ftop + 1.25), 'rubber', 'stack', fup)
    conns = [bx(3.5, 6.0, 3.0, sx * 16.0, y, ftop + 1.5) for sx in (1, -1) for y in (-7, 0, 7)]
    conns += [bx(6.0, 3.5, 3.0, x, 16.0, ftop + 1.5) for x in (-7, 0, 7)]
    add('fc_connectors__plastic_white', compound(conns), 'plastic_white', 'stack', fup)
    usb = W('XY').box(9.0, 7.3, 3.2).edges('|Y').fillet(1.4).translate((0, -14.6, ftop + 1.6))
    add('fc_usb__steel', usb, 'steel', 'stack', fup)
    add('fc_leds__accent', compound([bx(1.6, 0.9, 0.5, -10, 10, ftop + 0.25), bx(1.6, 0.9, 0.5, -10, 12, ftop + 0.25)]),
        'accent', 'stack', fup)

    add('stack_bolts__titanium', compound([cyl(1.5, PU, f0 + 4.4, x, y) for x, y in STACK]), 'titanium', 'stack', (0, 0, 4))
    spacers = [tube(2.8, 1.5, AT + sk.PLATE_T, z0, x, y) for x, y in STACK]
    spacers += [tube(2.8, 1.5, z0 + 1.6, f0 - 0.4, x, y) for x, y in STACK]
    spacers += [hexagon(6.4, f0 + 2.0, f0 + 4.4, x, y).cut(cyl(1.5, f0, f0 + 5, x, y)) for x, y in STACK]
    add('stack_spacers__plastic_black', compound(spacers), 'plastic_black', 'stack', (0, 0, 16))
    add('fc_grommets__accent', compound([tube(2.4, 1.5, f0 - 0.4, f0 + 2.0, x, y) for x, y in STACK]), 'accent', 'stack', fup)

    add('bec__shrink', bx(40, 16, 8, -40, -6, -6, r=2.5, every_edge=True), 'shrink', 'stack', (-30, 0, 6))

    # ELRS receiver on the middle plate at the rear, coax and dipole out the back
    rx_x, rx_y, z = -46.0, -19.0, AT + sk.PLATE_T
    rup = (-20, -10, 8)
    add('rx_tape__rubber', bx(15, 10, 1.0, rx_x, rx_y, z + 0.5), 'rubber', 'stack', rup)
    add('rx_pcb__pcb_black', bx(15, 10, 1.0, rx_x, rx_y, z + 1.5), 'pcb_black', 'stack', rup)
    add('rx_chip__chip', bx(5, 5, 0.9, rx_x + 2, rx_y, z + 2.45), 'chip', 'stack', rup)
    coax = wire([(rx_x - 7.4, -22, z + 1.5), (-66, -24, z + 1.6), (-80, -24, z + 1.6)], 0.75, (-1, 0, 0), (-1, 0, 0))
    ants = [rod_dir(0.6, (-80, -24, z + 1.6), (-1, -1, 0), 28), rod_dir(0.6, (-80, -24, z + 1.6), (-1, 1, 0), 28)]
    add('rx_antenna__wire_black', compound([coax] + ants), 'wire_black', 'stack', rup)


# ---------- Jetson deck, dev kit and power ----------

def build_jetson():
    up = (0, 0, 70)
    add('dampers__rubber', compound([cyl(4.0, TZ + sk.PLATE_T, DZ, x, y) for x, y in DAMPERS]), 'rubber', 'jetson', (0, 0, 52))
    deck = plate(*sk.DECK, DZ, sk.PLATE_T, 8)
    deck = drill(deck, DAMPERS + sk.BRIDGE_POSTS + [(sx * 43, sy * 32) for sx in (1, -1) for sy in (1, -1)], 1.6, DZ - 1, DZ + 3)
    deck = slots(deck, [(-30, 0), (30, 0)], 30, 10, DZ - 1, DZ + 3, 90)
    add('jetson_deck__carbon', deck, 'carbon', 'jetson', (0, 0, 60))
    d0 = DZ + sk.PLATE_T
    board_z = d0 + 4.0
    add('jetson_feet__steel', compound([cyl(2.5, d0, board_z, sx * 43, sy * 32) for sx in (1, -1) for sy in (1, -1)]),
        'steel', 'jetson', up)
    add('jetson_ssd__pcb_black', bx(80, 22, 2.2, -2, -8, d0 + 1.8), 'pcb_black', 'jetson', up)
    add('jetson_ssd_chips__chip', compound([bx(12, 12, 0.9, x, -8, d0 + 3.35) for x in (-20, 0, 20)]), 'chip', 'jetson', up)
    board = drill(plate(100, 79, board_z, 1.6, 3), [(sx * 43, sy * 32) for sx in (1, -1) for sy in (1, -1)], 1.4,
                  board_z - 1, board_z + 3)
    add('jetson_carrier__pcb_black', board, 'pcb_black', 'jetson', up)
    b1 = board_z + 1.6
    add('jetson_module__pcb_green', bx(70, 45, 1.2, -4, 0, b1 + 0.6), 'pcb_green', 'jetson', up)
    sink = bx(66, 50, 3, -4, 0, b1 + 1.2 + 1.5).union(
        compound([bx(1.2, 50, 10.0, -4 - 31 + 5.5 * k, 0, b1 + 4.2 + 5.0) for k in range(12)]))
    add('jetson_heatsink__alu', sink, 'alu', 'jetson', up)
    fan_z = b1 + 14.2
    fan = bx(44, 44, 8, -4, 0, fan_z + 4, r=4).cut(cyl(19.5, fan_z + 1, fan_z + 9, -4, 0))
    add('jetson_fan__plastic_black', fan, 'plastic_black', 'jetson', up)
    blades = [W('XY').box(16, 5, 1.2).translate((9, 0, 0)).rotate((0, 0, 0), (1, 0, 0), 25)
              .rotate((0, 0, 0), (0, 0, 1), 360 / 7 * k).translate((-4, 0, fan_z + 5)) for k in range(7)]
    add('jetson_fan_blades__chip', compound(blades + [cyl(7, fan_z + 2.5, fan_z + 7.5, -4, 0)]), 'chip', 'jetson', up)
    ports = [bx(14, 13.5, 15.5, 44, 24, b1 + 7.75), bx(14, 13.5, 15.5, 44, 8, b1 + 7.75),
             bx(16, 16, 13.5, 43, -10, b1 + 6.75), bx(9, 17, 6, 46, -30, b1 + 3)]
    add('jetson_ports__steel', compound(ports), 'steel', 'jetson', up)
    add('jetson_usb3__usb3', compound([bx(0.4, 7.0, 1.8, 51.2, y, b1 + z) for y in (24, 8) for z in (4.0, 11.5)]),
        'usb3', 'jetson', up)
    add('jetson_barrel__plastic_black', bx(12, 9, 11, -45, -20, b1 + 5.5), 'plastic_black', 'jetson', up)
    add('jetson_header__plastic_black', bx(51, 5, 2.5, -8, 34, b1 + 1.25), 'plastic_black', 'jetson', up)
    pins = [bx(0.64, 0.64, 6.0, -8 + (k - 9.5) * 2.54, 34 + s * 1.27, b1 + 2.5 + 3.0) for k in range(20) for s in (1, -1)]
    add('jetson_header_pins__gold', compound(pins), 'gold', 'jetson', up)
    ants, brackets, cables = [], [], []
    for sy in (1, -1):
        ants.append(bx(6, 2, 30, -70, sy * 40, d0 + 16, r=0.8))
        brackets.append(bx(10, 8, 4, -66, sy * 40, d0 + 2))
        cables.append(wire([(-70, sy * 40, d0 + 26), (-62, sy * 36, d0 + 22), (-48, sy * 30, b1 + 1.0)], 0.6,
                           (1, 0, 0), (1, 0, 0)))
    add('wifi_antennas__plastic_black', compound(ants), 'plastic_black', 'jetson', (-20, 0, 70))
    add('wifi_brackets__print', compound(brackets), 'print', 'jetson', (-20, 0, 70))
    add('wifi_cables__wire_black', compound(cables), 'wire_black', 'jetson', (-20, 0, 70))
    power = wire([(-60, -6, -6), (-72, -8, 2), (-72, -10, 18), (-68, -12, 26), (-51.2, -20, b1 + 5.5)], 1.3,
                 (-1, 0, 0), (1, 0, 0))
    add('jetson_power_lead__wire_black', power, 'wire_black', 'jetson', (0, 0, 30))


# ---------- mast bridge, mast, D500 and side cameras ----------

def build_mast():
    posts = [cyl(3.0, DZ + sk.PLATE_T, BZ, x, y) for x, y in sk.BRIDGE_POSTS]
    add('bridge_posts__print', compound(posts), 'print', 'lidar', (0, 0, 110))
    br = plate(124, 100, BZ, 3.0, 8)
    br = br.cut(cyl(25, BZ - 1, BZ + 4, -4, 0))
    hub = cyl(10, BZ, BZ + 3)
    spokes = [W('XY').box(52, 4, 3).rotate((0, 0, 0), (0, 0, 1), 60 * k + 30).translate((-2, 0, BZ + 1.5))
              .intersect(cyl(25.5, BZ, BZ + 3, -4, 0)) for k in range(3)]
    br = br.union(hub)
    for s_ in spokes:
        br = br.union(s_)
    br = br.union(cyl(9, BZ + 3, BZ + 12)).cut(cyl(6, BZ + 3, BZ + 13))
    br = drill(br, sk.BRIDGE_POSTS, 1.6, BZ - 1, BZ + 4)
    br = slots(br, [(40, 30), (40, -30), (-44, 30), (-44, -30)], 18, 8, BZ - 1, BZ + 4, 0)
    add('mast_bridge__print', br, 'print', 'lidar', (0, 0, 120))
    add('mast__carbon', tube(6, 5, BZ + 3, LB - 3), 'carbon', 'lidar', (0, 0, 140))
    mount = plate(60, 52, LB - 3, 3, 5).union(cyl(9, LB - 11, LB - 3)).cut(cyl(6, LB - 12, LB - 3))
    add('lidar_mount__print', mount, 'print', 'lidar', (0, 0, 160))

    up = (0, 0, 195)
    base_h = 12.0
    base = W('XY').box(54, 46.29, base_h).edges('|Z').fillet(8).translate((0, 0, LB + base_h / 2))
    try:
        base = base.edges('<Z').fillet(1.5)
    except Exception:
        pass
    add('d500_base__lidar', base, 'lidar', 'lidar', up)
    add('d500_connector__plastic_white', bx(6, 10, 5, -28, 0, LB + 4), 'plastic_white', 'lidar', up)
    top = LB + sk.LIDAR_SIZE[2]
    head = cyl(19.5, LB + base_h, sk.LIDAR_Z - 3).union(cyl(19.5, sk.LIDAR_Z + 3, top - 2))
    add('d500_head__lidar', head, 'lidar', 'lidar', up)
    add('d500_window__glass', cyl(19.2, sk.LIDAR_Z - 3, sk.LIDAR_Z + 3), 'glass', 'lidar', up)
    add('d500_ring__accent', tube(19.8, 19.5, LB + base_h, LB + base_h + 1), 'accent', 'lidar', up)
    add('d500_cap__lidar', cyl(17.5, top - 2, top).edges('>Z').fillet(1.2), 'lidar', 'lidar', up)


def build_side_cams():
    collar = tube(10, 6, sk.SIDE_Z - 14, sk.SIDE_Z)
    add('camera_collar__print', collar, 'print', 'side', (0, 0, 150))
    for side, sy in (('left', 1), ('right', -1)):
        yaw = sy * (90.0 + sk.SIDE_BACK)

        def place(s):
            return (s.rotate((0, 0, 0), (0, 1, 0), -sk.SIDE_TILT).rotate((0, 0, 0), (0, 0, 1), yaw)
                    .translate((0, sy * sk.SIDE_Y, sk.SIDE_Z)))

        move = (0, sy * 45, 150)
        add(f'side_{side}_board__pcb_black', place(bx(1.2, 25, 24, -5.4, 0, 0)), 'pcb_black', 'side', move)
        add(f'side_{side}_chips__chip', place(bx(1.0, 8, 8, -6.5, 4, -6)), 'chip', 'side', move)
        add(f'side_{side}_holder__plastic_black', place(bx(4.6, 14, 14, -2.5, 0, 0, r=1.5)), 'plastic_black', 'side', move)
        add(f'side_{side}_barrel__alu_dark', place(rod(6.0, 3.0, (-0.2, 0, 0), 'x')), 'alu_dark', 'side', move)
        dome = W('XY').sphere(5.6).intersect(W('XY').box(6, 12, 12).translate((3, 0, 0))).translate((2.8, 0, 0))
        add(f'side_{side}_lens__glass', place(dome), 'glass', 'side', move)
        arm = bx(9.0, 10, 10, -10.0, 0, -6)
        add(f'side_{side}_arm__print', place(arm), 'print', 'side', move)


# ---------- battery, tray, straps and leads ----------

STRAP_X = (-20.0, 30.0)


def build_battery():
    bl, bw, bh = sk.BATTERY_SIZE
    down = (0, 0, -95)
    cz = (BAT_TOP + BAT_BOTTOM) / 2
    add('battery__battery', bx(bl, bw, bh, BAT_X, 0, cz, r=3.0, every_edge=True), 'battery', 'battery', down)
    labels = [bx(xl, 0.4, 34, xc, sy * (bw / 2 + 0.2), cz) for xc, xl in ((-55.0, 46.0), (5.0, 26.0)) for sy in (1, -1)]
    add('battery_labels__label', compound(labels), 'label', 'battery', down)
    stripes = [bx(xl, 0.45, 3, xc, sy * (bw / 2 + 0.22), cz + 10) for xc, xl in ((-55.0, 46.0), (5.0, 26.0)) for sy in (1, -1)]
    add('battery_stripes__plastic_white', compound(stripes), 'plastic_white', 'battery', down)

    tray = plate(142, 61, BAT_TOP, 3.0, 6, BAT_X, 0)
    tray = drill(tray, TRAY_BOLTS, 1.6, BAT_TOP - 1, PU + 1)
    tray = drill(tray, arm_bolts() + STACK, 3.4, BAT_TOP - 1, PU + 1)        # room for the nuts and screw heads
    for x in STRAP_X:
        for sy in (1, -1):
            tray = tray.cut(bx(21, 1.6, 6, x, sy * 25.3, BAT_TOP + 1.5))
    tray = slots(tray, [(-45, 0), (5, 0)], 30, 10, BAT_TOP - 1, PU + 1)
    add('battery_tray__print', tray, 'print', 'battery', (0, 0, -55))
    straps, buckles = [], []
    for x in STRAP_X:
        outer = bx(20, 51.8, PU - (BAT_BOTTOM - 1.2), x, 0, (PU + BAT_BOTTOM - 1.2) / 2)
        inner = bx(22, 49.4, PU - BAT_BOTTOM + 1, x, 0, (PU + BAT_BOTTOM) / 2 + 0.5)
        straps.append(outer.cut(inner))
        buckles.append(bx(14, 3.0, 1.4, x, 0, BAT_BOTTOM - 1.9, r=0.5))
    add('straps__strap', compound(straps), 'strap', 'battery', (0, 0, -80))
    add('strap_buckles__accent', compound(buckles), 'accent', 'battery', (0, 0, -80))

    rear = BAT_X - bl / 2
    lead = wire([(rear + 0.4, 10, cz - 7), (-87, 11, cz - 9), (-92, 12, -58.0), (-92, 12, -55.6)], 2.4, (-1, 0, 0), (0, 0, 1))
    add('battery_lead__wire_black', lead, 'wire_black', 'battery', down)
    bal = wire([(rear + 0.4, -10, cz + 12), (-85, -12, cz + 10), (-86.5, -13, cz + 2)], 1.5, (-1, 0, 0), (0, 0, -1))
    add('balance_lead__plastic_white', bal, 'plastic_white', 'battery', down)
    add('balance_plug__plastic_white', bx(5, 7, 18, -86.5, -13, cz - 7, r=0.8), 'plastic_white', 'battery', down)
    add('xt60_female__xt60', bx(8.2, 15.5, 16, -92, 12, -47.5, r=1.2), 'xt60', 'battery', down)
    add('xt60_male__xt60', bx(8.2, 15.5, 15, -92, 12, -32.0, r=1.2), 'xt60', 'stack', (-18, 0, 4))
    pig = wire([(-92, 12, -24.4), (-91, 12, -18), (-82, 13, -13.0), (-60, 12, -12.0), (-22, 9, -12.0)], 2.2,
               (0, 0, 1), (1, -0.1, 0))
    add('esc_lead__wire_black', pig, 'wire_black', 'stack', (-18, 0, 4))


# ---------- OAK-D Lite on its cradle ----------

def build_oak():
    fx, fz = sk.OAK_FRONT, sk.OAK_Z
    d, w, h = sk.OAK_SIZE

    def place(s):
        return s.rotate((0, 0, 0), (0, 1, 0), -sk.OAK_TILT).translate((fx, 0, fz))

    move = (60, 0, -20)
    body = W('XY').box(d - 0.8, w, h).translate((-(d - 0.8) / 2 - 0.8, 0, 0)).edges('|X').fillet(6.0)
    add('oakd_body__alu_dark', place(body), 'alu_dark', 'oakd', move)
    face = W('XY').box(0.8, w - 3, h - 4).translate((-0.4, 0, 0)).edges('|X').fillet(4.5)
    add('oakd_face__glass', place(face), 'glass', 'oakd', move)
    rings = [rod(4.3, 0.7, (0, s * 37.5, 0), 'x').cut(rod(3.0, 1, (-0.1, s * 37.5, 0), 'x')) for s in (1, -1)]
    rings.append(rod(5.6, 0.7, (0, 0, 0), 'x').cut(rod(4.1, 1, (-0.1, 0, 0), 'x')))
    add('oakd_lens_rings__steel', place(compound(rings)), 'steel', 'oakd', move)
    glass = [rod(3.0, 0.45, (0, s * 37.5, 0), 'x') for s in (1, -1)] + [rod(4.1, 0.45, (0, 0, 0), 'x')]
    add('oakd_lenses__glass', place(compound(glass)), 'glass', 'oakd', move)
    usb = W('XY').box(1.2, 9.0, 3.3).edges('|X').fillet(1.4).translate((-d - 0.6, 24, 0))
    add('oakd_usb__steel', place(usb), 'steel', 'oakd', move)

    drop = PU - (fz + h / 2)                 # from the OAK-D's top to the bottom plate
    back = bx(4, 44, 16 + drop + 14, -d - 2, 0, (drop + 14 - 16) / 2, r=3)   # tall enough to reach the tab once tilted
    lip = bx(16.5, 44, 3, -d - 4 + 8.25, 0, -h / 2 - 1.5, r=2)
    tab = drill(bx(10, 40, 3, 64.5, 0, PU - 1.5, r=3), OAK_TAB_BOLTS, 1.6, PU - 4, PU + 1)
    add('oakd_cradle__print', place(back.union(lip)).union(tab), 'print', 'oakd', (36, 0, -10))
    gp = [(-d - 0.6 + 0.1, 24, 0)]
    c, s = math.cos(math.radians(sk.OAK_TILT)), math.sin(math.radians(sk.OAK_TILT))
    ux, uz = gp[0][0] * c - gp[0][2] * s + fx, gp[0][0] * s + gp[0][2] * c + fz
    cable = wire([(ux - 0.4, 24, uz), (72, 24, -40), (72, 24, 0), (72, 24, 18), (64, 24, 32), (51.4, 24, DZ + 2 + 5.6 + 4)],
                 2.0, (-1, 0, 0), (-1, 0, 0))
    add('oakd_cable__wire_black', cable, 'wire_black', 'oakd', (30, 0, 0))


# ---------- MTF-01P ----------

def build_mtf():
    mx, my = sk.MTF_XY
    face = AB - sk.MTF_SIZE[2]
    move = (-10, -20, -60)
    add('mtf01p_case__plastic_black', bx(sk.MTF_SIZE[0], sk.MTF_SIZE[1], (AB - 2.0) - (face + 2.0), mx, my,
                                         ((AB - 2.0) + (face + 2.0)) / 2, r=1.5), 'plastic_black', 'mtf', move)
    add('mtf01p_lens__glass', cyl(4.0, face, face + 2.0, mx + 5, my), 'glass', 'mtf', move)
    add('mtf01p_lens_ring__steel', tube(5.2, 4.0, face + 0.6, face + 2.0, mx + 5, my), 'steel', 'mtf', move)
    add('mtf01p_tof__glass', bx(6.5, 4.0, 1.4, mx - 7, my, face + 1.3), 'glass', 'mtf', move)
    holder = bx(32, 21, 2, mx, my, AB - 1.0)
    rails = [bx(30, 2, 11, mx, my + s * 9.6, AB - 2.0 - 5.5) for s in (1, -1)]
    add('mtf01p_holder__print', holder.union(rails[0]).union(rails[1]), 'print', 'mtf', (-10, -20, -40))


# ---------- legs ----------

def build_legs():
    for i, (sx, sy, _) in enumerate(MOTORS):
        x, y = sx * sk.LEG_XY, sy * sk.LEG_XY
        o = outward(sx, sy, 30)
        foot_z = sk.LEG_BOTTOM + 6.0
        leg = cyl(6.0, AB - 2.0, AB, x, y).union(cyl(5.0, AB - 10, AB - 2.0, x, y))
        leg = leg.union(W('XY').add(cq.Solid.makeCone(3.2, 4.0, (AB - 10) - (foot_z + 4), cq.Vector(x, y, foot_z + 4))))
        leg = leg.cut(cyl(1.6, AB - 8, AB + 1, x, y))
        add(f'leg{i}__print', leg, 'print', 'legs', (o[0], o[1], -55))
        foot = W('XY').add(cq.Solid.makeSphere(6.0, cq.Vector(x, y, foot_z), angleDegrees1=-90, angleDegrees2=90))
        add(f'foot{i}__rubber', foot, 'rubber', 'legs', (o[0], o[1], -55))


# ---------- checks ----------

OWNERS = {'OAK-D Lite view': {'oakd'}, 'left side camera view': {'side'}, 'right side camera view': {'side'},
          'lidar scan plane': {'lidar'}, 'flow view': {'mtf'}, 'range beam': {'mtf'}}
# Already known from skeleton.py and handled in software: masks for the props and the lidar corner
KNOWN = {('OAK-D Lite view', 'props'), ('left side camera view', 'props'), ('right side camera view', 'props'),
         ('left side camera view', 'lidar'), ('right side camera view', 'lidar')}


def check_views(views):
    problems, known = [], []
    cache = {}
    for view in views:
        for name, shape, _, group, _ in PARTS:
            if group in OWNERS[view.name]:
                continue
            if name not in cache:
                cache[name] = [(v.x, v.y, v.z) for v in shape.tessellate(0.5, 0.5)[0]]
            hits = sum(view.contains(p) for p in cache[name])
            if hits:
                line = f'IN VIEW {name} in {view.name}: {hits} of {len(cache[name])} mesh points'
                (known if (view.name, group) in KNOWN else problems).append(line)
    return problems, known


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


def build_all():
    """Fill PARTS with every detailed part. render.py uses this too."""
    if not PARTS:
        for build in (build_frame, build_motors, build_stack, build_jetson, build_mast, build_side_cams, build_battery,
                      build_oak, build_mtf, build_legs):
            build()
    return PARTS


GROUP_MASS = ('frame', 'motors', 'props', 'stack', 'jetson', 'lidar', 'side', 'oakd', 'battery', 'mtf', 'legs')


def group_of(name):
    base = name.rstrip('0123456789').rstrip('_')
    if base.startswith('side_cam'):
        return 'side'
    if base.startswith('wifi_antenna'):
        return 'jetson'
    return {'arm': 'frame', 'bottom_plate': 'frame', 'middle_plate': 'frame', 'top_plate': 'frame',
            'standoff': 'frame', 'motor': 'motors', 'prop': 'props', 'esc': 'stack', 'flight_controller': 'stack',
            'bec': 'stack', 'elrs_rx': 'stack', 'deck_plate': 'jetson', 'damper': 'jetson', 'jetson': 'jetson',
            'mast_bridge': 'lidar', 'bridge_post': 'lidar', 'mast': 'lidar', 'd500': 'lidar', 'oakd_lite': 'oakd',
            'oakd_mount': 'oakd', 'battery': 'battery', 'battery_tray': 'battery', 'mtf01p': 'mtf', 'leg': 'legs',
            sk.WIRES: 'other'}.get(base, 'other')


def main():
    build_all()
    print(f'{len(PARTS)} detailed parts')
    sk_parts = sk.build_parts()
    views = sk.build_views(sk_parts)
    overlaps = check_overlaps()
    in_view, known = check_views(views)
    problems = overlaps + in_view
    for p in problems:
        print('  ' + p)
    print(f'  {len(problems)} problems; {len(known)} known view overlaps (props and the lidar corner, masked in software)')

    assy = cq.Assembly(name='drone')
    for name, shape, mat, _, _ in PARTS:
        assy.add(shape, name=name, color=cq.Color(*MATS[mat], 1.0))
    assy.save('model.step', 'STEP')
    assy.save('model.glb', 'GLTF', tolerance=0.1, angularTolerance=0.3)

    total, com, inertia = sk.mass_props(sk_parts)
    oak = next(p for p in sk_parts if p.name == 'oakd_lite')
    mtf = next(p for p in sk_parts if p.name == 'mtf01p')
    side_views = [{'apex': list(v.apex), 'yaw': v.yaw, 'tilt': v.tilt, 'hfov': v.hfov, 'vfov': v.vfov}
                  for v in views if v.name.endswith('side camera view')]
    meta = {
        'nodes': {name: {'group': group, 'explode': list(ex), 'mat': mat} for name, _, mat, group, ex in PARTS},
        'groups': {g: round(sum(p.mass for p in sk_parts if group_of(p.name) == g), 1) for g in GROUP_MASS},
        'views': {
            'oak': {'apex': [sk.OAK_FRONT, 0, oak.center[2]], 'tilt': sk.OAK_TILT, 'hfov': sk.OAK_HFOV, 'vfov': sk.OAK_VFOV},
            'side': side_views,
            'lidar': {'center': [0, 0, sk.LIDAR_Z], 'inner': math.hypot(*sk.LIDAR_SIZE[:2]) / 2},
            'flow': {'apex': [mtf.center[0], mtf.center[1], mtf.center[2] - mtf.size[2] / 2], 'fov': sk.FLOW_FOV,
                     'range_fov': sk.RANGE_FOV},
        },
        'ground': sk.LEG_BOTTOM,
        'mass': round(total), 'com': [round(c, 1) for c in com],
        'inertia': [round(i, 4) for i in inertia], 'lidar_z': round(sk.LIDAR_Z),
        'flow_height': round(mtf.center[2] - mtf.size[2] / 2 - sk.LEG_BOTTOM),
        'motor_to_motor': 320, 'prop_in': 7, 'parts': len(PARTS), 'problems': problems, 'known': len(known),
    }
    with open('model_parts.json', 'w', encoding='utf-8') as f:
        json.dump(meta, f, indent=1)
    write_page(meta)
    print(f'Wrote model.step, model.glb, model_parts.json, drone_3d.html: {total:.0f} g')


def write_page(meta):
    """Write drone_3d.html: page_template.html with the model and its data inside, so it opens as one file."""
    with open('model.glb', 'rb') as f:
        glb = base64.b64encode(f.read()).decode('ascii')
    with open('page_template.html', encoding='utf-8') as f:
        page = f.read()
    page = page.replace('__META__', json.dumps(meta)).replace('__GLB__', glb)
    with open('drone_3d.html', 'w', encoding='utf-8') as f:
        f.write(page)


if __name__ == '__main__':
    main()
