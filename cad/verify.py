# Design checks: run with  freecadcmd cad/verify.py  (after build_all.py)
import glob
import itertools
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import FreeCAD as App
import Mesh
import Part
import arm_2dof as arm
from FreeCAD import Vector as V

ROOT = os.path.dirname(HERE)
BED_MM = 150.0
OVERLAP_LIMIT = 0.5  # mm3
EXPECTED = {frozenset(p) for p in (("pan_servo", "pan_horn"), ("tilt_servo", "tilt_horn"))}
failures = []


def check(ok, text):
    print(("PASS  " if ok else "FAIL  ") + text)
    if not ok:
        failures.append(text)


items = dict(arm.make_printed_parts())
items.update(arm.make_vitamins())

print("--- Dimensions")
print("tilt axis height %.1f, deck %.1f, plate %.1f-%.1f, collar top %.1f" % (
    arm.PIVOT_H, arm.Z_DECK, arm.Z_PLATE_BOT, arm.Z_PLATE_TOP, arm.Z_COLLAR_TOP))
check(100 <= arm.PIVOT_H <= 150, "tilt axis is 10-15 cm above the table")

print("--- Collisions over the full travel")
pans = (-90, -45, 0, 45, 90)
tilts = (-90, -60, -30, 0, 30, 60, 90)
worst = {}
for pan, tilt in itertools.product(pans, tilts):
    shapes = {n: arm.posed(s, g, pan, tilt) for n, (s, g) in items.items()}
    for a, b in itertools.combinations(shapes, 2):
        if frozenset((a, b)) in EXPECTED:
            continue
        if items[a][1] == items[b][1] and (pan, tilt) != (0, 0):
            continue  # parts moving together only need checking once
        if not shapes[a].BoundBox.intersect(shapes[b].BoundBox):
            continue
        vol = shapes[a].common(shapes[b]).Volume
        if vol > worst.get((a, b), (0.0,))[0]:
            worst[(a, b)] = (vol, pan, tilt)
hits = {k: v for k, v in worst.items() if v[0] > OVERLAP_LIMIT}
for (a, b), (vol, pan, tilt) in hits.items():
    print("   %s / %s: %.2f mm3 at pan %d, tilt %d" % (a, b, vol, pan, tilt))
check(not hits, "no collisions in %d poses" % (len(pans) * len(tilts)))

print("--- Beam path")
beam_block = None
for down in range(0, 90):
    beam = arm.xcyl(0.5, arm.LH_HALF_LEN + 0.1, 800.0, 0, arm.PIVOT_H)
    beam = arm.posed(beam, "tilt", 0, -down)
    blocked = False
    for name in ("pan_yoke", "riser", "pan_servo", "tilt_servo", "pan_horn"):
        shape, group = items[name]
        if beam.common(arm.posed(shape, group, 0, 0)).Volume > 0.01:
            blocked = name
            break
    if blocked:
        beam_block = (down, blocked)
        break
if beam_block:
    down, name = beam_block
    reach = arm.PIVOT_H / math.tan(math.radians(down))
    print("   first blocked at %d deg down by %s; nearest table point %.0f mm from the pan axis" % (
        down, name, reach))
else:
    down, reach = 90, 0.0
need = math.degrees(math.atan2(arm.PIVOT_H, 300.0))
far = math.degrees(math.atan2(arm.PIVOT_H, 500.0))
print("   table targets 300-500 mm away need %.1f-%.1f deg down" % (far, need))
check(down > need and reach < 300.0, "beam reaches table targets at 30-50 cm")

print("--- Solids and meshes")
for path in sorted(glob.glob(os.path.join(ROOT, "stl", "*.stl"))):
    name = os.path.basename(path)
    mesh = Mesh.Mesh(path)
    bb = mesh.BoundBox
    check(mesh.isSolid() and not mesh.hasNonManifolds() and mesh.countComponents() == 1,
          "%s is one watertight mesh" % name)
    check(not mesh.hasSelfIntersections(), "%s has no self-intersections" % name)
    check(max(bb.XLength, bb.YLength) <= BED_MM and bb.ZLength <= BED_MM,
          "%s fits a %d mm bed (%.0f x %.0f x %.0f)" % (name, BED_MM, bb.XLength, bb.YLength, bb.ZLength))
    check(abs(bb.ZMin) < 1e-3, "%s sits on z = 0" % name)
    # Downward faces steeper than 45 degrees and not on the bed would need bridging or support
    area = 0.0
    z_seen = []
    for facet in mesh.Facets:
        nz = facet.Normal.z
        lowest = min(p[2] for p in facet.Points)
        if nz < -0.7072 and lowest > 0.3:
            area += facet.Area
            z_seen.append(lowest)
    span = "z %.1f-%.1f" % (min(z_seen), max(z_seen)) if z_seen else "-"
    print("      overhang faces: %.0f mm2 (%s), volume %.1f cm3, about %.0f g solid PLA" % (
        area, span, mesh.Volume / 1000.0, mesh.Volume / 1000.0 * 1.24))

print("--- Wall thickness at the screw holes")
ear_edge = arm.SV_WID / 2.0 + arm.CL_POCKET
check(arm.ARM_HALF_W - ear_edge >= 4.0, "servo arm side wall is %.2f mm" % (arm.ARM_HALF_W - ear_edge))
check(arm.PIVOT_H - arm.AXIS_TO_PLATE < arm.PIVOT_H - arm.HOLE_B - arm.M2_PILOT,
      "lower ear slot is clear of the yoke plate")
floor_gap = (arm.PIVOT_H - (arm.LASER_D + arm.CL_BORE) / 2.0) - (
    arm.PIVOT_H - arm.CR_BELOW + arm.CR_FLOOR_T + arm.LH_BOLT_DEPTH)
check(floor_gap >= 0.5, "holder bolt holes stop %.2f mm short of the laser bore" % floor_gap)

print("--- Assembly order")
travel = (arm.SV_SHAFT_TOP - arm.HORN_HUB_BOTTOM) + 0.3
check(arm.PIVOT_SIDE_GAP >= travel,
      "cradle can slide %.1f mm to clear the spline (needs %.1f)" % (arm.PIVOT_SIDE_GAP, travel))
tip = arm.Y_PIVOT_ARM_OUT - arm.PIVOT_SCREW_LEN
check(tip > arm.LH_HALF_W, "pivot screw tip at y = %.1f clears the laser holder (%.1f)" % (tip, arm.LH_HALF_W))
nut_engaged = (arm.CR_INNER_HALF_W + arm.M3_NUT_H) - tip
check(nut_engaged >= 2.0, "pivot screw engages the nut by %.1f mm" % min(nut_engaged, arm.M3_NUT_H))

print("RESULT: %s" % ("ALL CHECKS PASSED" if not failures else "%d FAILED" % len(failures)))
for failure in failures:
    print("  - " + failure)
