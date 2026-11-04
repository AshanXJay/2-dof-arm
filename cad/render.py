# Draws the preview images in docs/: run with  freecadcmd cad/render.py
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import FreeCAD as App
import arm_2dof as arm

DOCS = os.path.join(os.path.dirname(HERE), "docs")
os.makedirs(DOCS, exist_ok=True)
LIGHT = np.array([0.45, -0.55, 0.70])
LIGHT = LIGHT / np.linalg.norm(LIGHT)


def triangles(shape, color):
    verts, faces = shape.tessellate(0.1)
    pts = np.array([[v.x, v.y, v.z] for v in verts])
    tris = pts[np.array(faces)]
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    keep = lengths > 1e-9
    tris, normals = tris[keep], normals[keep] / lengths[keep, None]
    shade = 0.5 + 0.5 * np.clip(normals @ LIGHT, 0, 1)
    cols = np.clip(np.outer(shade, np.array(color)), 0, 1)
    return tris, cols


def draw(ax, entries, elev, azim, lines=(), pad=1.05):
    all_tris, all_cols = [], []
    for shape, color in entries:
        t, c = triangles(shape, color)
        all_tris.append(t)
        all_cols.append(c)
    tris = np.concatenate(all_tris)
    cols = np.concatenate(all_cols)
    el, az = np.radians(elev), np.radians(azim)
    view = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    # Hide back faces; the solids are closed, so this removes draw-order artefacts
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    front = np.where(normals @ view > 0)[0]
    order = front[np.argsort(tris[front].mean(axis=1) @ view)]
    ax.add_collection3d(Poly3DCollection(tris[order], facecolors=cols[order], edgecolors=cols[order],
                                         linewidths=0.2))
    for (p0, p1, color) in lines:
        ax.plot(*zip(p0, p1), color=color, lw=1.6)
    lo, hi = tris.reshape(-1, 3).min(axis=0), tris.reshape(-1, 3).max(axis=0)
    centre, half = (lo + hi) / 2.0, (hi - lo).max() / 2.0 * pad
    ax.set_xlim(centre[0] - half, centre[0] + half)
    ax.set_ylim(centre[1] - half, centre[1] + half)
    ax.set_zlim(centre[2] - half, centre[2] + half)
    ax.set_box_aspect((1, 1, 1), zoom=1.35)
    ax.view_init(elev=elev, azim=azim)
    ax.set_proj_type("ortho")
    ax.set_axis_off()


def assembly(pan, tilt, offsets=None, vitamins=True):
    offsets = offsets or {}
    out = []
    groups = [arm.make_printed_parts()]
    if vitamins:
        groups.append(arm.make_vitamins())
    for group in groups:
        for name, (shape, motion) in group.items():
            posed = arm.posed(shape, motion, pan, tilt)
            if name in offsets:
                moved = posed.copy()
                moved.translate(App.Vector(*offsets[name]))
                posed = moved
            out.append((posed, arm.COLORS[name]))
    return out


def beam(pan, tilt, length=170.0):
    p, t = np.radians(pan), np.radians(tilt)
    d = np.array([np.cos(t) * np.cos(p), np.cos(t) * np.sin(p), np.sin(t)])
    a = np.array([0, 0, arm.PIVOT_H]) + d * arm.LH_HALF_LEN
    return [(a, a + d * length, "red")]


fig = plt.figure(figsize=(14, 7))
ax = fig.add_subplot(1, 2, 1, projection="3d")
draw(ax, assembly(0, 0), 22, -40, beam(0, 0, 110))
ax.set_title("Home pose: pan 0, tilt 0")
ax = fig.add_subplot(1, 2, 2, projection="3d")
draw(ax, assembly(35, -22), 22, -40, beam(35, -22, 140))
ax.set_title("Aiming at the table: pan 35, tilt -22")
fig.tight_layout()
fig.savefig(os.path.join(DOCS, "assembly.png"), dpi=130)

fig = plt.figure(figsize=(14, 7))
ax = fig.add_subplot(1, 2, 1, projection="3d")
draw(ax, assembly(0, 0), 0, -90, beam(0, 0, 60))
ax.set_title("Side view (laser fires to the right)")
ax = fig.add_subplot(1, 2, 2, projection="3d")
draw(ax, assembly(0, 0), 0, 0)
ax.set_title("Front view (looking into the laser)")
fig.tight_layout()
fig.savefig(os.path.join(DOCS, "side_front.png"), dpi=130)

explode = {
    "riser": (0, 0, 25), "pan_servo": (0, 0, 75), "pan_horn": (0, 0, 115),
    "pan_yoke": (0, 0, 150), "tilt_servo": (0, -35, 150), "tilt_horn": (0, -18, 150),
    "tilt_cradle": (0, 0, 205), "laser_holder": (0, 0, 245), "laser": (45, 0, 245),
    "pivot_screw": (0, 55, 150), "pivot_spacer": (0, 30, 150),
}
fig = plt.figure(figsize=(9, 12))
ax = fig.add_subplot(1, 1, 1, projection="3d")
draw(ax, assembly(0, 0, explode), 12, -55)
ax.set_title("Exploded view")
fig.tight_layout()
fig.savefig(os.path.join(DOCS, "exploded.png"), dpi=130)

parts = dict((n, s) for n, (s, _) in arm.make_printed_parts().items())
parts["fit_test"] = arm.make_fit_test()
fig = plt.figure(figsize=(18, 9))
for i, (name, shape) in enumerate(parts.items(), start=1):
    ax = fig.add_subplot(2, 4, i, projection="3d")
    oriented = arm.print_orientation(name, shape)
    draw(ax, [(oriented, arm.COLORS.get(name, (0.3, 0.6, 0.8)))], 32, -50)
    bb = oriented.BoundBox
    ax.set_title("%s\n%.0f x %.0f x %.0f mm, as printed" % (name, bb.XLength, bb.YLength, bb.ZLength))
fig.tight_layout()
fig.savefig(os.path.join(DOCS, "printed_parts.png"), dpi=120)

# Cross-section through the tilt axis plane (y = 0) and the pan axis
import Part
cutter_y = Part.makeBox(400, 200, 400, App.Vector(-200, 0.01, -50))
fig = plt.figure(figsize=(9, 9))
ax = fig.add_subplot(1, 1, 1, projection="3d")
section = [(s.cut(cutter_y), c) for s, c in assembly(0, 0)]
draw(ax, [(s, c) for s, c in section if s.Volume > 0.01], 0, 90, pad=0.8)
ax.set_title("Section at y = 0, showing the riser interior")
fig.tight_layout()
fig.savefig(os.path.join(DOCS, "section.png"), dpi=130)
print("RENDER_OK")
