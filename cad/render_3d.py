"""3D views of the skeleton: skeleton_3d.png (two angles) and skeleton_3d.html (rotate in a browser).

Uses the same parts, positions and views as skeleton.py. Run: python3 render_3d.py
"""
import json
import math

from matplotlib import colors as mcolors
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon

import skeleton as sk

REACH = 200.0
SIDES = 28
NAMES = {'oakd_lite': 'OAK-D Lite', 'oakd_mount': 'OAK-D Lite mount', 'mtf01p': 'MTF-01P', 'esc': '4-in-1 ESC',
         'flight_controller': 'Flight controller', 'bec': 'BEC', 'elrs_rx': 'ELRS receiver', 'jetson': 'Jetson dev kit',
         'battery': '4S 5000 mAh battery', 'stl27l': 'STL-27L lidar', 'side_cam_left': 'Side camera, left',
         'side_cam_right': 'Side camera, right', 'gps': 'GPS', 'gps_stalk': 'GPS stalk', 'sik_radio': 'SiK radio',
         'deck_plate': 'Top deck', 'bottom_plate': 'Bottom plate', 'mast': 'Sensor mast'}


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


def view_faces(views):
    """Return translucent faces for every view, as {name: (faces, color, alpha)}."""
    out = {}
    for v in views:
        if v.kind == 'pyramid':
            far = [tuple(v.apex[i] + REACH * r[i] for i in range(3)) for r in v.rays()]
            faces = [[v.apex, far[i], far[(i + 1) % 4]] for i in range(4)]
            alpha = 0.10
        elif v.kind == 'cone':
            drop = v.apex[2] - sk.LEG_BOTTOM
            r = math.tan(math.radians(v.hfov / 2)) * drop
            ring = [(v.apex[0] + r * math.cos(2 * math.pi * i / SIDES), v.apex[1] + r * math.sin(2 * math.pi * i / SIDES),
                     sk.LEG_BOTTOM) for i in range(SIDES)]
            faces = [[v.apex, ring[i], ring[(i + 1) % SIDES]] for i in range(SIDES)]
            alpha = 0.14 if v.hfov > 10 else 0.35
        else:
            inner, outer = v.hfov, 260.0
            faces = []
            for i in range(SIDES * 2):
                a0, a1 = 2 * math.pi * i / (SIDES * 2), 2 * math.pi * (i + 1) / (SIDES * 2)
                faces.append([(inner * math.cos(a0), inner * math.sin(a0), v.apex[2]),
                              (outer * math.cos(a0), outer * math.sin(a0), v.apex[2]),
                              (outer * math.cos(a1), outer * math.sin(a1), v.apex[2]),
                              (inner * math.cos(a1), inner * math.sin(a1), v.apex[2])])
            alpha = 0.10
        out[v.name[0].upper() + v.name[1:]] = (faces, v.color, alpha)
    return out


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


def draw_view(ax, parts, views, azim, elev, title, labels):
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

    g = sk.LEG_BOTTOM
    for x in range(-240, 281, 40):
        ax.plot(*zip(project((x, -240, g)), project((x, 240, g))), color='0.88', lw=0.6, zorder=0)
    for y in range(-240, 241, 40):
        ax.plot(*zip(project((-240, y, g)), project((280, y, g))), color='0.88', lw=0.6, zorder=0)
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
    ax.set_xlim(-330, 330)
    ax.set_ylim(-260, 300)


def write_html(parts, views, summary):
    """Write the viewer, with the model data embedded in viewer_template.html."""
    data = [{'name': display_name(p.name), 'shape': p.shape, 'size': list(p.size), 'center': list(p.center),
             'yaw': p.yaw, 'color': mcolors.to_hex(p.color), 'opacity': 0.3 if p.name.startswith('prop') else 1.0,
             'source': p.src, 'mass': p.mass}
            for p in parts if p.name != sk.WIRES]
    view_data = [{'name': name, 'color': mcolors.to_hex(color), 'tris': [tri for f in faces for tri in fan(f)]}
                 for name, (faces, color, _) in views.items()]
    model = {'parts': data, 'views': view_data, 'ground': sk.LEG_BOTTOM, 'summary': summary}
    with open('viewer_template.html', encoding='utf-8') as src:
        page = src.read().replace('__MODEL__', json.dumps(model))
    with open('skeleton_3d.html', 'w', encoding='utf-8') as out:
        out.write(page)


def fan(face):
    """Split a polygon into triangles for the viewer."""
    return [[face[0], face[i], face[i + 1]] for i in range(1, len(face) - 1)]


def main():
    parts = sk.build_parts()
    views = view_faces(sk.build_views(parts))
    total, com, _ = sk.mass_props(parts)
    summary = f'{total:.0f} g, centre of mass {com[2]:+.1f} mm from the prop plane'
    get = {p.name: p.center for p in parts}

    above = [('STL-27L lidar', get['stl27l'], (-130, 60)), ('Side cameras', get['side_cam_left'], (-190, 20)),
             ('Jetson', get['jetson'], (-170, -20)), ('GPS', get['gps'], (-120, 70)),
             ('Prop disc', (113, -113, 0), (120, 50)), ('OAK-D Lite', get['oakd_lite'], (110, -80)),
             ('Lidar scan plane', (180, 120, sk.LIDAR_Z), (60, 70)), ('Leg', (85, -85, -100), (-60, -70))]
    below = [('MTF-01P', get['mtf01p'], (-150, -60)), ('Battery', get['battery'], (-190, 30)),
             ('OAK-D Lite', get['oakd_lite'], (150, -60)), ('Flow view', (-50, -42, -110), (-90, -80)),
             ('Leg', (85, 85, -100), (60, -90)), ('BEC', get['bec'], (150, 40))]

    fig, axes = plt.subplots(1, 2, figsize=(17, 9))
    draw_view(axes[0], parts, views, -35, 25, 'From above, front left', above)
    draw_view(axes[1], parts, views, 35, -20, 'From below, front right', below)
    fig.suptitle(f'7" drone 3D skeleton in P1\'s layout: simple blocks, sizes are estimates. {summary}', fontsize=13)
    fig.tight_layout()
    fig.savefig('skeleton_3d.png', dpi=120)
    write_html(parts, views, summary)
    print('Wrote skeleton_3d.png and skeleton_3d.html:', summary)


if __name__ == '__main__':
    main()
