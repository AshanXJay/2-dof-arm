# Rebuilds every output file: run with  freecadcmd cad/build_all.py
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import FreeCAD as App
import Mesh
import MeshPart
import Part
import arm_2dof as arm

ROOT = os.path.dirname(HERE)
STL_DIR = os.path.join(ROOT, "stl")
STEP_DIR = os.path.join(ROOT, "step")
for path in (STL_DIR, STEP_DIR):
    os.makedirs(path, exist_ok=True)

printed = {name: shape for name, (shape, _) in arm.make_printed_parts().items()}
printed["fit_test"] = arm.make_fit_test()

for index, (name, shape) in enumerate(printed.items(), start=1):
    oriented = arm.print_orientation(name, shape)
    stem = "%d_%s" % (index, name) if name != "fit_test" else "0_fit_test_print_first"
    mesh = MeshPart.meshFromShape(Shape=oriented, LinearDeflection=0.02,
                                  AngularDeflection=0.25, Relative=False)
    mesh.write(os.path.join(STL_DIR, stem + ".stl"))
    oriented.exportStep(os.path.join(STEP_DIR, stem + ".step"))
    bb = oriented.BoundBox
    print("%-26s %6.1f x %6.1f x %6.1f mm  %7.1f cm3  facets %d" % (
        stem, bb.XLength, bb.YLength, bb.ZLength, shape.Volume / 1000.0, mesh.CountFacets))

doc = arm.build_document()
doc.saveAs(os.path.join(HERE, "arm_2dof.FCStd"))
assembly = [obj.Shape for obj in doc.Objects
            if obj.isDerivedFrom("Part::Feature") and obj.Name != "fit_test_print_only"]
Part.makeCompound(assembly).exportStep(os.path.join(STEP_DIR, "assembly_home_pose.step"))
print("BUILD_OK")
