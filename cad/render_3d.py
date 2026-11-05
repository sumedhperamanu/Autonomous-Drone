"""3D views of the skeleton: skeleton_3d.png (two angles) and skeleton_3d.html (rotate in a browser).

Uses the same parts and positions as skeleton.py. Run: python3 render_3d.py
"""
import json
import math

from matplotlib import colors as mcolors
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon

import skeleton as sk

REACH = 170.0
SIDES = 28
NAMES = {'oakd_lite': 'OAK-D Lite', 'mtf01': 'MTF-01', 'pi5': 'Raspberry Pi 5', 'esc': '4-in-1 ESC',
         'flight_controller': 'Flight controller', 'bec': 'BEC', 'rc_receiver': 'RC receiver',
         'battery': '4S 5000 mAh battery', 'camera_bracket': 'Camera bracket', 'top_plate': 'Top plate',
         'middle_plate': 'Middle plate', 'bottom_plate': 'Bottom plate'}


def display_name(name):
    if name in NAMES:
        return NAMES[name]
    return name.rsplit('_', 1)[0].capitalize()


def part_faces(p):
    """Return the part's faces as lists of (x, y, z) points."""
    cx, cy, cz = p.center
    if p.shape == 'cyl':
        r, h = p.size
        ring = [(r * math.cos(2 * math.pi * i / SIDES), r * math.sin(2 * math.pi * i / SIDES)) for i in range(SIDES)]
        low = [(cx + a, cy + b, cz - h / 2) for a, b in ring]
        high = [(cx + a, cy + b, cz + h / 2) for a, b in ring]
        sides = [[low[i], low[(i + 1) % SIDES], high[(i + 1) % SIDES], high[i]] for i in range(SIDES)]
        return sides + [low, high]
    hx, hy, hz = p.size[0] / 2, p.size[1] / 2, p.size[2] / 2
    c, s = math.cos(math.radians(p.yaw)), math.sin(math.radians(p.yaw))

    def corner(a, b, d):
        return cx + a * hx * c - b * hy * s, cy + a * hx * s + b * hy * c, cz + d * hz

    v = {(a, b, d): corner(a, b, d) for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)}
    return [[v[-1, -1, d], v[1, -1, d], v[1, 1, d], v[-1, 1, d]] for d in (-1, 1)] + \
           [[v[a, -1, -1], v[a, 1, -1], v[a, 1, 1], v[a, -1, 1]] for a in (-1, 1)] + \
           [[v[-1, b, -1], v[1, b, -1], v[1, b, 1], v[-1, b, 1]] for b in (-1, 1)]


def view_faces(x_front, cam_z, apex, ground):
    """Return translucent faces for the camera view, flow view and range beam, as (faces, color, alpha)."""
    t, u = math.tan(math.radians(sk.HFOV / 2)) * REACH, math.tan(math.radians(sk.VFOV / 2)) * REACH
    lens = (x_front, 0.0, cam_z)
    far = [(x_front + REACH, a * t, cam_z + b * u) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    views = {'Camera view': ([[lens, far[i], far[(i + 1) % 4]] for i in range(4)], 'tab:blue', 0.10)}
    drop = apex[2] - ground
    for name, fov, color, alpha in (('Flow view', sk.FLOW_FOV, 'tab:orange', 0.14), ('Range beam', sk.RANGE_FOV, 'tab:red', 0.35)):
        r = math.tan(math.radians(fov / 2)) * drop
        ring = [(apex[0] + r * math.cos(2 * math.pi * i / SIDES), apex[1] + r * math.sin(2 * math.pi * i / SIDES), ground)
                for i in range(SIDES)]
        views[name] = ([[apex, ring[i], ring[(i + 1) % SIDES]] for i in range(SIDES)], color, alpha)
    return views


def basis(azim, elev):
    """Return the screen right, screen up and toward-viewer directions."""
    a, e = math.radians(azim), math.radians(elev)
    toward = (math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e))
    right = (-math.sin(a), math.cos(a), 0.0)
    up = (-math.sin(e) * math.cos(a), -math.sin(e) * math.sin(a), math.cos(e))
    return right, up, toward


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def normal(face):
    a, b, c = face[0], face[1], face[2]
    u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    size = math.sqrt(dot(n, n)) or 1.0
    return n[0] / size, n[1] / size, n[2] / size


def draw_view(ax, parts, views, azim, elev, title, labels, ground):
    """Draw one projected view, sorting faces from far to near."""
    right, up, toward = basis(azim, elev)
    light = (0.35, 0.3, 0.89)
    items = []
    for p in parts:
        if p.name != sk.WIRES:
            alpha = 0.3 if p.name.startswith('prop') else 1.0
            items.extend((f, p.color, alpha) for f in part_faces(p))
    for faces, color, alpha in views.values():
        items.extend((f, color, alpha) for f in faces)
    items.sort(key=lambda it: sum(dot(pt, toward) for pt in it[0]) / len(it[0]))

    def project(pt):
        return dot(pt, right), dot(pt, up)

    for x in range(-240, 341, 40):
        ax.plot(*zip(project((x, -240, ground)), project((x, 240, ground))), color='0.88', lw=0.6, zorder=0)
    for y in range(-240, 241, 40):
        ax.plot(*zip(project((-240, y, ground)), project((340, y, ground))), color='0.88', lw=0.6, zorder=0)

    for z, (face, color, alpha) in enumerate(items, start=1):
        shade = 0.62 + 0.38 * abs(dot(normal(face), light))
        rgb = [c * shade for c in mcolors.to_rgb(color)]
        edge = (0, 0, 0, 0.0) if alpha < 0.5 else (0.1, 0.1, 0.1, 0.35)
        ax.add_patch(Polygon([project(pt) for pt in face], fc=(*rgb, alpha), ec=edge, lw=0.4, zorder=z))

    for text, pt, offset in labels:
        x, y = project(pt)
        ax.annotate(text, xy=(x, y), xytext=(x + offset[0], y + offset[1]), fontsize=9, ha='center', va='center',
                    zorder=len(items) + 5, bbox={'boxstyle': 'round,pad=0.2', 'fc': 'white', 'ec': '0.7', 'lw': 0.5},
                    arrowprops={'arrowstyle': '-', 'color': '0.25', 'lw': 0.7})
    ax.set_title(title)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_xlim(-320, 320)
    ax.set_ylim(-250, 270)


def write_html(parts, views, ground, summary):
    """Write the viewer, with the model data embedded in viewer_template.html."""
    data = [{'name': display_name(p.name), 'shape': p.shape, 'size': list(p.size), 'center': list(p.center),
             'yaw': p.yaw, 'color': mcolors.to_hex(p.color), 'opacity': 0.3 if p.name.startswith('prop') else 1.0,
             'source': p.src, 'mass': p.mass}
            for p in parts if p.name != sk.WIRES]
    view_data = [{'name': name, 'color': mcolors.to_hex(color), 'tris': faces}
                 for name, (faces, color, _) in views.items()]
    model = {'parts': data, 'views': view_data, 'ground': ground, 'summary': summary}
    with open('viewer_template.html', encoding='utf-8') as src:
        page = src.read().replace('__MODEL__', json.dumps(model))
    with open('skeleton_3d.html', 'w', encoding='utf-8') as out:
        out.write(page)


def main():
    x_front, cam_z = sk.mount_options()[0]
    parts = sk.build_parts(x_front, cam_z)
    mtf = next(p for p in parts if p.name == 'mtf01')
    apex = (mtf.center[0], mtf.center[1], mtf.center[2] - mtf.size[2] / 2)
    ground = -6.0 - sk.LEG_DROP
    views = view_faces(x_front, cam_z, apex, ground)
    total, com, _ = sk.mass_props(parts)
    battery = next(p for p in parts if p.name == 'battery')
    summary = f'{total:.0f} g, centre of mass {com[2] - (sk.PROP_Z + 1):+.1f} mm from the prop plane'

    above = [('Battery', battery.center, (-90, 110)), ('Raspberry Pi 5', (30, 28, 50), (-160, 40)),
             ('OAK-D Lite', (x_front, -30, cam_z), (50, -125)), ('Prop disc', (113, -113, 36), (150, 75)),
             ('Motor', (-113, -113, 30), (-55, 85)), ('Arm', (-70, 70, 0), (-175, -60)),
             ('Camera view', (x_front + 130, 0, cam_z), (135, 25)), ('Leg', (60, -60, -80), (-95, -70))]
    below = [('MTF-01', (20, 0, -23), (-55, -165)), ('OAK-D Lite', (x_front, 30, cam_z - 14), (165, -95)),
             ('Bottom plate', (-40, 0, -8.5), (-235, 15)), ('Leg', (-60, 60, -60), (65, -130)),
             ('Flow view', (apex[0], 18, ground + 20), (-120, -85)), ('Arm', (70, -70, -6), (150, 65)),
             ('Camera bracket', (61, -20, -20), (170, 25)), ('Camera view', (x_front + 130, 0, cam_z), (-140, 40))]

    fig, axes = plt.subplots(1, 2, figsize=(17, 8.5))
    draw_view(axes[0], parts, views, -35, 28, 'From above, front left', above, ground)
    draw_view(axes[1], parts, views, 35, -22, 'From below, front right', below, ground)
    fig.suptitle(f'7" drone 3D skeleton: simple blocks, sizes are estimates. {summary}', fontsize=13)
    fig.tight_layout()
    fig.savefig('skeleton_3d.png', dpi=120)
    write_html(parts, views, ground, summary)
    print('Wrote skeleton_3d.png and skeleton_3d.html:', summary)


if __name__ == '__main__':
    main()
