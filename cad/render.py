"""Black-and-white renders of the detailed 7" model, in the style of a CAD studio render.

Writes drone_render.png (grey shaded with black edges) and drone_lineart.png (black lines on
white). Both draw the sharp edges of the CAD model plus silhouettes, so curved parts get clean outlines.
Run from this folder after model.py works: python3 render.py
Needs VTK (installed with cadquery) and Pillow.
"""
import math
import sys

import vtk
from PIL import Image, ImageDraw, ImageFont

import model as m5
import skeleton as sk

W, H = 3600, 2400
LINE = 2.0
# Grey for each material: a light studio palette, darker for carbon and electronics.
GREY = {
    'carbon': 0.30, 'titanium': 0.74, 'alu_dark': 0.42, 'bell': 0.34, 'accent': 0.6, 'copper': 0.62,
    'steel': 0.84, 'pcb_black': 0.28, 'pcb_green': 0.5, 'gold': 0.78, 'chip': 0.22, 'plastic_white': 0.95,
    'plastic_black': 0.3, 'glass': 0.12, 'print': 0.9, 'battery': 0.38, 'label': 0.58, 'strap': 0.24,
    'wire_red': 0.3, 'wire_black': 0.22, 'xt60': 0.85, 'shrink': 0.52, 'prop': 0.88, 'rubber': 0.2,
    'lidar': 0.4, 'usb3': 0.45,
}
VIEW = (1.0, 1.2, 0.72)      # from the front left, above


def split(shape):
    """Return the shaded faces of one part and its sharp edges (over 28 degrees), as two vtkPolyData.

    Sharp edges, like a CAD program's default view: tangent lines on fillets and cylinder seams are left out.
    """
    pd = shape.toVtkPolyData(0.03, 0.1, normals=True)
    faces = vtk.vtkPolyData()
    faces.SetPoints(pd.GetPoints())
    faces.SetPolys(pd.GetPolys())
    faces.GetPointData().SetNormals(pd.GetPointData().GetNormals())
    clean = vtk.vtkCleanPolyData()
    clean.SetInputData(faces)
    clean.SetTolerance(1e-6)
    clean.PointMergingOn()
    feat = vtk.vtkFeatureEdges()
    feat.SetInputConnection(clean.GetOutputPort())
    feat.FeatureEdgesOn()
    feat.SetFeatureAngle(28)
    feat.BoundaryEdgesOn()
    feat.NonManifoldEdgesOn()
    feat.ManifoldEdgesOff()
    feat.ColoringOff()
    feat.Update()
    return faces, feat.GetOutput()


def line_actor(source, width):
    mapper = vtk.vtkPolyDataMapper()
    if isinstance(source, vtk.vtkPolyData):
        mapper.SetInputData(source)
    else:
        mapper.SetInputConnection(source.GetOutputPort())
    mapper.SetResolveCoincidentTopologyToPolygonOffset()
    mapper.SetRelativeCoincidentTopologyLineOffsetParameters(-4.0, -8.0)
    a = vtk.vtkActor()
    a.SetMapper(mapper)
    p = a.GetProperty()
    p.SetColor(0.04, 0.04, 0.05)
    p.SetLineWidth(width)
    p.LightingOff()
    return a


def render(path, lineart):
    ren = vtk.vtkRenderer()
    ren.SetBackground(1, 1, 1)
    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.SetSize(W, H)
    win.AddRenderer(ren)

    cam = ren.GetActiveCamera()
    d = [v / math.sqrt(sum(c * c for c in VIEW)) for v in VIEW]
    focus = (0.0, 0.0, 6.0)
    cam.SetFocalPoint(*focus)
    cam.SetPosition(*[focus[i] + 2000 * d[i] for i in range(3)])
    cam.SetViewUp(0, 0, 1)
    cam.SetViewAngle(17)

    count = 0
    for name, shape, mat, group, _ in m5.build_all():
        faces, edges = split(shape)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(faces)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        p = actor.GetProperty()
        if lineart:
            p.SetColor(1, 1, 1)
            p.LightingOff()
        else:
            g = GREY.get(mat, 0.5)
            p.SetColor(g, g, g)
            p.SetInterpolationToPhong()
            p.SetAmbient(0.22)
            p.SetDiffuse(0.82)
            p.SetSpecular(0.35 if mat in ('steel', 'titanium', 'bell', 'glass', 'carbon', 'battery') else 0.12)
            p.SetSpecularPower(40)
        ren.AddActor(actor)
        ren.AddActor(line_actor(edges, LINE))
        sil = vtk.vtkPolyDataSilhouette()
        sil.SetInputData(faces)
        sil.SetCamera(cam)
        sil.SetEnableFeatureAngle(0)
        sil.BorderEdgesOff()
        ren.AddActor(line_actor(sil, LINE * 1.25))
        count += 1

    if not lineart:
        kit = vtk.vtkLightKit()
        kit.SetKeyLightIntensity(0.95)
        kit.SetKeyToFillRatio(2.2)
        kit.AddLightsToRenderer(ren)
    ren.SetPass(vtk.vtkRenderStepsPass())   # offscreen capture comes back black without a pass
    ren.UseFXAAOn()

    ren.ResetCameraClippingRange()
    win.Render()
    grab = vtk.vtkWindowToImageFilter()
    grab.SetInput(win)
    grab.SetInputBufferTypeToRGB()
    grab.ReadFrontBufferOff()
    grab.Update()
    out = vtk.vtkPNGWriter()
    out.SetFileName(path)
    out.SetInputConnection(grab.GetOutputPort())
    out.Write()
    frame(path, lineart)
    print(f'Wrote {path}: {count} parts')


def frame(path, lineart):
    """Crop to the drone, then add a thin border and a drawing-style title block."""
    img = Image.open(path).convert('RGB')
    grey = img.convert('L').point(lambda v: 255 if v < 250 else 0)
    box = grey.getbbox()
    pad = 160
    img = img.crop((max(0, box[0] - pad), max(0, box[1] - pad), min(W, box[2] + pad), min(H, box[3] + pad + 220)))
    w, h = img.size
    draw = ImageDraw.Draw(img)
    try:
        bold = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf', 44)
        small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 26)
    except OSError:
        bold = small = ImageFont.load_default()
    ink = (20, 20, 22)
    draw.rectangle((24, 24, w - 25, h - 25), outline=ink, width=3)
    bw, bh = 1400, 190
    x0, y0 = w - 25 - bw, h - 25 - bh
    draw.rectangle((x0, y0, w - 25, h - 25), outline=ink, width=3)
    draw.line((x0, y0 + 74, w - 25, y0 + 74), fill=ink, width=2)
    draw.text((x0 + 24, y0 + 14), '7" OBSTACLE-AVOIDANCE QUAD  ·  DETAILED ASSEMBLY', font=bold, fill=ink)
    rows = [f'MASS {m5.sk.mass_props(sk.build_parts())[0]:.0f} g (EST.)   SPAN 320 mm   PROPS 7 x 3.5 x 2',
            'OAK-D LITE  ·  2 x IMX219-200  ·  D500  ·  MTF-01P  ·  JETSON',
            'cad/model.py  ·  UNITS mm  ·  REV 2026-11-16' + ('  ·  LINE ART' if lineart else '')]
    for i, r in enumerate(rows):
        draw.text((x0 + 24, y0 + 88 + i * 32), r, font=small, fill=ink)
    img.save(path, optimize=True)


if __name__ == '__main__':
    which = sys.argv[1:] or ['render', 'lineart']
    if 'render' in which:
        render('drone_render.png', lineart=False)
    if 'lineart' in which:
        render('drone_lineart.png', lineart=True)
