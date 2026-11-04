# 2-DOF direct-drive pan-tilt laser arm (SG90) - FreeCAD 1.x parametric model.
#
# Edit values in the PARAMETERS block, then either:
#   GUI:      Macro > Macros... > open this file > Execute (rebuilds and shows the assembly)
#   Headless: freecadcmd cad/build_all.py  (rebuilds FCStd, STEP and STL files)
#
# World frame at the home pose (pan = 0, tilt = 0):
#   Origin on the table under the pan axis. +Z up. Laser fires along +X.
#   Pan axis = Z through the origin. Tilt axis = Y at height PIVOT_H.
#   The beam passes through the point where both axes cross, so aiming has no offset.

import FreeCAD as App
import Part
from FreeCAD import Vector as V

# ---------------------------- PARAMETERS (mm) ----------------------------
# Tilt axis height above the table
PIVOT_H = 120.0

# SG90 servo, typical values. Local frame: shaft on Z, ear underside at z = 0.
SV_LEN = 22.7         # body length, without ears
SV_WID = 12.1         # body width
SV_EAR_SPAN = 32.4    # ear tip to ear tip
SV_EAR_T = 2.5        # ear thickness
SV_BELOW_EAR = 15.9   # case bottom to ear underside
SV_CASE_TOP = 6.8     # ear underside to top of case
SV_DOME_TOP = 11.2    # ear underside to top of gear dome
SV_SHAFT_TOP = 14.8   # ear underside to spline tip
SV_SHAFT_OFF = 5.9    # shaft centre to the nearest body end
SV_HOLE_SPACING = 27.8

# Servo horn (two-arm), typical values
HORN_TOP = 15.8       # ear underside to flat top of horn
HORN_ARM_T = 1.5
HORN_SPAN = 35.0
HORN_ARM_W = 6.0
HORN_HUB_D = 7.0
HORN_HUB_BOTTOM = 11.6

# Generic laser: 6 mm barrel module
LASER_D = 6.0
LASER_LEN = 12.0

# Printing clearances
CL_POCKET = 0.25      # per side, servo pockets
CL_BORE = 0.2         # on diameter, laser bore
CL_HORN_W = 0.7       # on width, horn recess
RECESS_DEPTH = 1.5    # horn recess depth

# Screw holes
M2_PILOT = 1.7        # self-tapping M2 into a slot
EAR_SLOT_LEN = 3.4    # slot length, absorbs hole spacing differences between brands
M3_TAP = 2.7          # M3 cut directly into plastic
M3_CLEAR = 3.3
M3_PIVOT = 3.1        # tilt pivot running hole, ream with a 3 mm drill
M3_NUT_AF = 5.7       # nut width across flats, with clearance
M3_NUT_H = 2.6
CENTER_ACCESS_D = 5.5  # access to the horn centre screw

# Base and riser
BASE_SIZE = 140.0
BASE_T = 6.0
BASE_CORNER_INSET = 10.0
BASE_CORNER_D = 5.0
FLANGE_R = 46.0
FLANGE_T = 5.0
FLANGE_BOLT_R = 40.0
COLUMN_R = 30.0
COLUMN_WALL = 4.0
COLLAR_INNER_R = 25.0
COLLAR_GAP = 0.8      # gap under the turntable, limits wobble of the servo shaft
DECK_T = 8.0
PORT_W = 18.0
PORT_H = 14.0

# Pan yoke
PLATE_T = 6.0
ARM_T = 5.0
ARM_HALF_W = 12.0
AXIS_TO_PLATE = 24.0  # tilt axis to the plate top
SERVO_ARM_ABOVE = 14.0
PIVOT_ARM_ABOVE = 10.0
REAR_X0 = -24.0
REAR_X1 = -20.0
REAR_TOP = -10.0      # relative to the tilt axis
PLATE_FRONT_X = 20.0

# Tilt cradle
CR_HALF_LEN = 14.0
CR_INNER_HALF_W = 12.0
CR_PLATE_T = 5.0
CR_BELOW = 12.0
CR_ABOVE = 9.0
CR_FLOOR_T = 3.0
PIVOT_SIDE_GAP = 4.5  # room to slide the cradle horn onto the spline; filled by the spacer
PIVOT_SPACER_W = 4.0  # printed spacer; shim the remainder with M3 washers
PIVOT_SPACER_D = 8.0
PIVOT_SCREW_LEN = 16.0

# Laser holder
LH_HALF_LEN = 10.0
LH_HALF_W = 10.0
LH_TOP = 8.0
LH_BOLT_X = 7.0
LH_BOLT_DEPTH = 5.0

# Pan and tilt angles for the assembly in the FCStd file
POSE_PAN = 0.0
POSE_TILT = 0.0

# ------------------------- DERIVED DIMENSIONS -------------------------
SV_FAR = SV_LEN - SV_SHAFT_OFF                    # shaft to the far body end
SV_CENTER = SV_FAR - SV_LEN / 2.0                 # shaft to body centre
HOLE_A = SV_CENTER - SV_HOLE_SPACING / 2.0        # ear hole, shaft end
HOLE_B = SV_CENTER + SV_HOLE_SPACING / 2.0        # ear hole, far end

Z_PLATE_TOP = PIVOT_H - AXIS_TO_PLATE
Z_PLATE_BOT = Z_PLATE_TOP - PLATE_T
Z_DECK = Z_PLATE_BOT + RECESS_DEPTH - HORN_TOP    # pan servo ears rest here
Z_COLLAR_TOP = Z_PLATE_BOT - COLLAR_GAP
Z_DECK_BOT = Z_DECK - DECK_T

CR_OUTER_HALF_W = CR_INNER_HALF_W + CR_PLATE_T
Y_HORN_FLOOR = -(CR_OUTER_HALF_W - RECESS_DEPTH)
Y_SERVO_EAR = Y_HORN_FLOOR - HORN_TOP             # tilt servo ear underside
Y_SERVO_ARM_OUT = Y_SERVO_EAR - ARM_T
Y_PIVOT_ARM_IN = CR_OUTER_HALF_W + PIVOT_SIDE_GAP
Y_PIVOT_ARM_OUT = Y_PIVOT_ARM_IN + ARM_T


# ------------------------------- HELPERS -------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def zcyl(r, z0, z1, x=0.0, y=0.0):
    return Part.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def ycyl(r, y0, y1, x=0.0, z=0.0):
    return Part.makeCylinder(r, y1 - y0, V(x, y0, z), V(0, 1, 0))


def xcyl(r, x0, x1, y=0.0, z=0.0):
    return Part.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def union(first, *rest):
    return first.fuse(list(rest)).removeSplitter() if rest else first


def solid(shape, name):
    shape = shape.removeSplitter()
    if len(shape.Solids) != 1:
        raise ValueError("%s has %d solids, expected 1" % (name, len(shape.Solids)))
    result = shape.Solids[0]
    if not result.isValid():
        raise ValueError("%s is not a valid solid" % name)
    return result


def zslot(cx, cy, along, length, width, z0, z1):
    """Vertical-axis slot. 'along' is 'x' or 'y'."""
    half = (length - width) / 2.0
    r = width / 2.0
    if along == "x":
        parts = [box(cx - half, cx + half, cy - r, cy + r, z0, z1),
                 zcyl(r, z0, z1, cx - half, cy), zcyl(r, z0, z1, cx + half, cy)]
    else:
        parts = [box(cx - r, cx + r, cy - half, cy + half, z0, z1),
                 zcyl(r, z0, z1, cx, cy - half), zcyl(r, z0, z1, cx, cy + half)]
    return union(*parts)


def yslot(cx, cz, along, length, width, y0, y1):
    """Y-axis slot. 'along' is 'x' or 'z'."""
    half = (length - width) / 2.0
    r = width / 2.0
    if along == "x":
        parts = [box(cx - half, cx + half, y0, y1, cz - r, cz + r),
                 ycyl(r, y0, y1, cx - half, cz), ycyl(r, y0, y1, cx + half, cz)]
    else:
        parts = [box(cx - r, cx + r, y0, y1, cz - half, cz + half),
                 ycyl(r, y0, y1, cx, cz - half), ycyl(r, y0, y1, cx, cz + half)]
    return union(*parts)


def hex_nut_pocket_z(af, x, y, z0, z1):
    import math
    r = af / 2.0 / math.cos(math.radians(30))
    pts = [V(x + r * math.cos(math.radians(60 * i)),
             y + r * math.sin(math.radians(60 * i)), z0) for i in range(7)]
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, z1 - z0))


def hex_nut_pocket_y(af, x, z, y0, y1):
    """Hex prism on a Y axis, with points up and down so the roof prints without support."""
    import math
    r = af / 2.0 / math.cos(math.radians(30))
    pts = [V(x + r * math.sin(math.radians(60 * i)), y0,
             z + r * math.cos(math.radians(60 * i))) for i in range(7)]
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, y1 - y0, 0))


def port_cutter(z0):
    """Cable window through the -Y column wall, with a 45 degree roof that needs no support."""
    half = PORT_W / 2.0
    wall = z0 + PORT_H - half
    y0, y1 = -COLUMN_R - 2.0, -COLUMN_R + COLUMN_WALL + 2.0
    pts = [V(-half, y0, z0), V(half, y0, z0), V(half, y0, wall),
           V(0, y0, wall + half), V(-half, y0, wall), V(-half, y0, z0)]
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, y1 - y0, 0))


def horn_recess_z(z0, z1):
    """Horn recess in a horizontal face, running along X."""
    half_w = (HORN_ARM_W + CL_HORN_W) / 2.0
    half_l = HORN_SPAN / 2.0 + 1.0
    cut = box(-half_l, half_l, -half_w, half_w, z0, z1)
    return union(cut, zcyl((HORN_HUB_D + CL_HORN_W) / 2.0, z0, z1))


def horn_pilot_slots_z(z0, z1):
    return union(zslot(-9.5, 0, "x", 8.0, M2_PILOT - 0.2, z0, z1),
                 zslot(9.5, 0, "x", 8.0, M2_PILOT - 0.2, z0, z1))


# -------------------------------- PARTS --------------------------------
def make_base_plate():
    half = BASE_SIZE / 2.0
    shape = box(-half, half, -half, half, 0, BASE_T)
    corner = half - BASE_CORNER_INSET
    cuts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            cuts.append(zcyl(BASE_CORNER_D / 2.0, -1, BASE_T + 1, sx * corner, sy * corner))
    bolt = FLANGE_BOLT_R * 0.7071
    for sx in (-1, 1):
        for sy in (-1, 1):
            cuts.append(zcyl(M3_CLEAR / 2.0, -1, BASE_T + 1, sx * bolt, sy * bolt))
            cuts.append(hex_nut_pocket_z(M3_NUT_AF, sx * bolt, sy * bolt, -1, M3_NUT_H))
    # Engraved arrow on the top face pointing at the laser's home direction
    pts = [V(half - 6, 0, BASE_T - 0.6), V(half - 16, 5, BASE_T - 0.6),
           V(half - 16, -5, BASE_T - 0.6), V(half - 6, 0, BASE_T - 0.6)]
    cuts.append(Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, 2)))
    return solid(shape.cut(cuts), "base_plate")


def make_riser():
    z_bot = BASE_T
    shape = union(zcyl(FLANGE_R, z_bot, z_bot + FLANGE_T),
                  zcyl(COLUMN_R, z_bot, Z_COLLAR_TOP))
    inner_r = COLUMN_R - COLUMN_WALL
    pocket_r = ((SV_FAR + CL_POCKET) ** 2 + (SV_WID / 2.0 + CL_POCKET) ** 2) ** 0.5 + 0.8
    z_cone_bot = Z_DECK_BOT - (inner_r - pocket_r)
    cuts = [
        zcyl(inner_r, z_bot - 1, z_cone_bot),
        Part.makeCone(inner_r, pocket_r, Z_DECK_BOT - z_cone_bot, V(0, 0, z_cone_bot), V(0, 0, 1)),
        zcyl(COLLAR_INNER_R, Z_DECK, Z_COLLAR_TOP + 1),
        box(-SV_SHAFT_OFF - CL_POCKET, SV_FAR + CL_POCKET,
            -SV_WID / 2.0 - CL_POCKET, SV_WID / 2.0 + CL_POCKET,
            Z_DECK_BOT - 1, Z_DECK + 1),
        zslot(HOLE_A, 0, "x", EAR_SLOT_LEN, M2_PILOT, Z_DECK - 7, Z_DECK + 1),
        zslot(HOLE_B, 0, "x", EAR_SLOT_LEN, M2_PILOT, Z_DECK - 7, Z_DECK + 1),
        port_cutter(z_bot + FLANGE_T + 2),
        port_cutter(z_bot + 36),
    ]
    bolt = FLANGE_BOLT_R * 0.7071
    for sx in (-1, 1):
        for sy in (-1, 1):
            cuts.append(zcyl(M3_CLEAR / 2.0, z_bot - 1, z_bot + FLANGE_T + 1, sx * bolt, sy * bolt))
    return solid(shape.cut(cuts), "riser")


def make_pan_yoke():
    H = PIVOT_H
    plate = union(zcyl(COLUMN_R, Z_PLATE_BOT, Z_PLATE_TOP),
                  box(REAR_X0, PLATE_FRONT_X, Y_SERVO_ARM_OUT, Y_PIVOT_ARM_OUT,
                      Z_PLATE_BOT, Z_PLATE_TOP))
    servo_arm = union(
        box(-ARM_HALF_W, ARM_HALF_W, Y_SERVO_ARM_OUT, Y_SERVO_EAR, Z_PLATE_TOP, H + SERVO_ARM_ABOVE),
        box(REAR_X0, -ARM_HALF_W + 0.01, Y_SERVO_ARM_OUT, Y_SERVO_EAR, Z_PLATE_TOP, H + REAR_TOP))
    pivot_arm = union(
        box(-ARM_HALF_W, ARM_HALF_W, Y_PIVOT_ARM_IN, Y_PIVOT_ARM_OUT, Z_PLATE_TOP, H + PIVOT_ARM_ABOVE),
        box(REAR_X0, -ARM_HALF_W + 0.01, Y_PIVOT_ARM_IN, Y_PIVOT_ARM_OUT, Z_PLATE_TOP, H + REAR_TOP))
    rear_wall = box(REAR_X0, REAR_X1, Y_SERVO_ARM_OUT, Y_PIVOT_ARM_OUT, Z_PLATE_TOP, H + REAR_TOP)
    shape = union(plate, servo_arm, pivot_arm, rear_wall)
    cuts = [
        zcyl(CENTER_ACCESS_D / 2.0, Z_PLATE_BOT - 1, Z_PLATE_TOP + 1),
        horn_recess_z(Z_PLATE_BOT - 1, Z_PLATE_BOT + RECESS_DEPTH),
        horn_pilot_slots_z(Z_PLATE_BOT - 1, Z_PLATE_TOP + 1),
        # Tilt servo window; the servo's far end points down
        box(-SV_WID / 2.0 - CL_POCKET, SV_WID / 2.0 + CL_POCKET,
            Y_SERVO_ARM_OUT - 1, Y_SERVO_EAR + 1,
            H - SV_FAR - CL_POCKET, H + SV_SHAFT_OFF + CL_POCKET),
        yslot(0, H - HOLE_A, "z", EAR_SLOT_LEN, M2_PILOT, Y_SERVO_ARM_OUT - 1, Y_SERVO_EAR + 1),
        yslot(0, H - HOLE_B, "z", EAR_SLOT_LEN, M2_PILOT, Y_SERVO_ARM_OUT - 1, Y_SERVO_EAR + 1),
        ycyl(M3_PIVOT / 2.0, Y_PIVOT_ARM_IN - 1, Y_PIVOT_ARM_OUT + 1, 0, H),
    ]
    return solid(shape.cut(cuts), "pan_yoke")


def make_tilt_cradle():
    H = PIVOT_H
    L = CR_HALF_LEN
    W_OUT = CR_OUTER_HALF_W
    W_IN = CR_INNER_HALF_W
    shape = union(
        box(-L, L, -W_OUT, -W_IN, H - CR_BELOW, H + CR_ABOVE),
        box(-L, L, W_IN, W_OUT, H - CR_BELOW, H + CR_ABOVE),
        box(-L, L, -W_OUT, W_OUT, H - CR_BELOW, H - CR_BELOW + CR_FLOOR_T))
    half_w = (HORN_ARM_W + CL_HORN_W) / 2.0
    cuts = [
        # Open-ended horn recess in the outer face, so horn length does not matter
        box(-L - 1, L + 1, -W_OUT - 1, Y_HORN_FLOOR, H - half_w, H + half_w),
        ycyl((HORN_HUB_D + CL_HORN_W) / 2.0, -W_OUT - 1, Y_HORN_FLOOR, 0, H),
        ycyl(CENTER_ACCESS_D / 2.0, -W_OUT - 1, -W_IN + 1, 0, H),
        yslot(-9.5, H, "x", 8.0, M2_PILOT - 0.2, -W_OUT - 1, -W_IN + 1),
        yslot(9.5, H, "x", 8.0, M2_PILOT - 0.2, -W_OUT - 1, -W_IN + 1),
        # Pivot side: clearance hole plus captive M3 nut on the inner face
        ycyl(M3_CLEAR / 2.0, W_IN - 1, W_OUT + 1, 0, H),
        hex_nut_pocket_y(M3_NUT_AF, 0, H, W_IN - 1, W_IN + M3_NUT_H),
        # Laser holder bolts through the floor
        zcyl(M3_CLEAR / 2.0, H - CR_BELOW - 1, H - CR_BELOW + CR_FLOOR_T + 1, -LH_BOLT_X, 0),
        zcyl(M3_CLEAR / 2.0, H - CR_BELOW - 1, H - CR_BELOW + CR_FLOOR_T + 1, LH_BOLT_X, 0),
    ]
    return solid(shape.cut(cuts), "tilt_cradle")


def make_laser_holder():
    H = PIVOT_H
    z_bot = H - CR_BELOW + CR_FLOOR_T
    shape = box(-LH_HALF_LEN, LH_HALF_LEN, -LH_HALF_W, LH_HALF_W, z_bot, H + LH_TOP)
    cuts = [
        xcyl((LASER_D + CL_BORE) / 2.0, -LH_HALF_LEN - 1, LH_HALF_LEN + 1, 0, H),
        zcyl(M3_TAP / 2.0, H, H + LH_TOP + 1),  # set screw onto the barrel
        zcyl(M3_TAP / 2.0, z_bot - 1, z_bot + LH_BOLT_DEPTH, -LH_BOLT_X, 0),
        zcyl(M3_TAP / 2.0, z_bot - 1, z_bot + LH_BOLT_DEPTH, LH_BOLT_X, 0),
    ]
    return solid(shape.cut(cuts), "laser_holder")


def make_pivot_spacer():
    y0 = CR_OUTER_HALF_W + (PIVOT_SIDE_GAP - PIVOT_SPACER_W) / 2.0
    shape = ycyl(PIVOT_SPACER_D / 2.0, y0, y0 + PIVOT_SPACER_W, 0, PIVOT_H)
    return solid(shape.cut(ycyl(M3_CLEAR / 2.0, y0 - 1, y0 + PIVOT_SPACER_W + 1, 0, PIVOT_H)),
                 "pivot_spacer")


def make_fit_test():
    """Small plate with every critical fit, to test on the real parts before the full print."""
    t = 5.0
    shape = box(-14, 62, -12, 12, 0, t)
    cuts = [
        box(-SV_SHAFT_OFF - CL_POCKET, SV_FAR + CL_POCKET,
            -SV_WID / 2.0 - CL_POCKET, SV_WID / 2.0 + CL_POCKET, -1, t + 1),
        zslot(HOLE_A, 0, "x", EAR_SLOT_LEN, M2_PILOT, -1, t + 1),
        zslot(HOLE_B, 0, "x", EAR_SLOT_LEN, M2_PILOT, -1, t + 1),
    ]
    # Horn recess along the plate's short side, open at both edges
    half_w = (HORN_ARM_W + CL_HORN_W) / 2.0
    cuts.append(box(40 - half_w, 40 + half_w, -13, 13, t - RECESS_DEPTH, t + 1))
    cuts.append(zcyl((HORN_HUB_D + CL_HORN_W) / 2.0, t - RECESS_DEPTH, t + 1, 40, 0))
    cuts.append(zcyl(CENTER_ACCESS_D / 2.0, -1, t + 1, 40, 0))
    cuts.append(zcyl((LASER_D + CL_BORE) / 2.0, -1, t + 1, 54, -5))
    cuts.append(zcyl(M3_PIVOT / 2.0, -1, t + 1, 54, 6))
    return solid(shape.cut(cuts), "fit_test")


# ----------------------- NON-PRINTED PARTS, FOR CHECKS -----------------------
def make_servo_local():
    half_w = SV_WID / 2.0
    ear0 = SV_CENTER - SV_EAR_SPAN / 2.0
    ear1 = SV_CENTER + SV_EAR_SPAN / 2.0
    return union(
        box(-SV_SHAFT_OFF, SV_FAR, -half_w, half_w, -SV_BELOW_EAR, SV_CASE_TOP),
        box(ear0, ear1, -half_w, half_w, 0, SV_EAR_T),
        zcyl(half_w, SV_CASE_TOP - 0.01, SV_DOME_TOP),
        zcyl(2.4, SV_DOME_TOP - 0.01, SV_SHAFT_TOP))


def make_horn_local():
    half_l = HORN_SPAN / 2.0
    half_w = HORN_ARM_W / 2.0
    return union(
        box(-half_l, half_l, -half_w, half_w, HORN_TOP - HORN_ARM_T, HORN_TOP),
        zcyl(HORN_HUB_D / 2.0, HORN_HUB_BOTTOM, HORN_TOP))


def _tilt_servo_matrix():
    # Local X to world -Z, local Y to world -X, local Z to world +Y
    return App.Matrix(0, -1, 0, 0,
                      0, 0, 1, Y_SERVO_EAR,
                      -1, 0, 0, PIVOT_H,
                      0, 0, 0, 1)


def _apply(shape, matrix):
    out = shape.copy()
    out.Placement = App.Placement(matrix).multiply(out.Placement)
    return out


def make_vitamins():
    """Returns {name: (shape, group)}. group is 'fixed', 'pan' or 'tilt'."""
    m_pan = App.Matrix()
    m_pan.move(V(0, 0, Z_DECK))
    m_tilt = _tilt_servo_matrix()
    # The tilt horn is fitted with its arms along the cradle recess (world X at home)
    tilt_horn = make_horn_local()
    tilt_horn.rotate(V(0, 0, 0), V(0, 0, 1), 90)
    laser = xcyl(LASER_D / 2.0, LH_HALF_LEN - LASER_LEN, LH_HALF_LEN, 0, PIVOT_H)
    pivot = union(
        ycyl(1.5, Y_PIVOT_ARM_OUT - PIVOT_SCREW_LEN, Y_PIVOT_ARM_OUT, 0, PIVOT_H),
        ycyl(2.75, Y_PIVOT_ARM_OUT, Y_PIVOT_ARM_OUT + 3.0, 0, PIVOT_H))
    return {
        "pan_servo": (_apply(make_servo_local(), m_pan), "fixed"),
        "pan_horn": (_apply(make_horn_local(), m_pan), "pan"),
        "tilt_servo": (_apply(make_servo_local(), m_tilt), "pan"),
        "tilt_horn": (_apply(tilt_horn, m_tilt), "tilt"),
        "laser": (laser, "tilt"),
        "pivot_screw": (pivot, "tilt"),
    }


def make_printed_parts():
    """Returns {name: (shape, group)} in world coordinates at the home pose."""
    return {
        "base_plate": (make_base_plate(), "fixed"),
        "riser": (make_riser(), "fixed"),
        "pan_yoke": (make_pan_yoke(), "pan"),
        "tilt_cradle": (make_tilt_cradle(), "tilt"),
        "laser_holder": (make_laser_holder(), "tilt"),
        "pivot_spacer": (make_pivot_spacer(), "tilt"),
    }


def pose_placement(group, pan_deg, tilt_deg):
    """Placement for a moving group. Positive tilt raises the laser."""
    p_pan = App.Placement(V(0, 0, 0), App.Rotation(V(0, 0, 1), pan_deg))
    p_tilt = App.Placement(V(0, 0, PIVOT_H), App.Rotation(V(0, 1, 0), -tilt_deg),
                           V(0, 0, 0))
    centre = App.Placement(V(0, 0, -PIVOT_H), App.Rotation())
    if group == "fixed":
        return App.Placement()
    if group == "pan":
        return p_pan
    return p_pan.multiply(p_tilt).multiply(centre)


def posed(shape, group, pan_deg, tilt_deg):
    out = shape.copy()
    out.Placement = pose_placement(group, pan_deg, tilt_deg).multiply(out.Placement)
    return out


def print_orientation(name, shape):
    """Part moved to its recommended print pose, sitting on z = 0 and centred in XY."""
    out = shape.copy()
    if name == "laser_holder":
        # Front face on the bed, so the laser bore prints vertical and truly round
        out.Placement = App.Placement(V(0, 0, 0), App.Rotation(V(0, 1, 0), 90)).multiply(out.Placement)
    elif name == "pivot_spacer":
        # Flat on the bed, so the hole prints round
        out.Placement = App.Placement(V(0, 0, 0), App.Rotation(V(1, 0, 0), 90)).multiply(out.Placement)
    bb = out.BoundBox
    move = App.Placement(V(-bb.Center.x, -bb.Center.y, -bb.ZMin), App.Rotation())
    out.Placement = move.multiply(out.Placement)
    return out


COLORS = {
    "base_plate": (0.25, 0.27, 0.30), "riser": (0.20, 0.45, 0.70),
    "pan_yoke": (0.90, 0.55, 0.15), "tilt_cradle": (0.20, 0.65, 0.40),
    "laser_holder": (0.80, 0.25, 0.25), "pivot_spacer": (0.65, 0.35, 0.75), "pan_servo": (0.15, 0.25, 0.70),
    "tilt_servo": (0.15, 0.25, 0.70), "pan_horn": (0.95, 0.95, 0.95),
    "tilt_horn": (0.95, 0.95, 0.95), "laser": (0.75, 0.55, 0.20),
    "pivot_screw": (0.6, 0.6, 0.6),
}


def parameter_table():
    return {k: v for k, v in sorted(globals().items())
            if k.isupper() and isinstance(v, (int, float)) and not isinstance(v, bool)}


def build_document(doc_name="arm_2dof", pan=None, tilt=None):
    pan = POSE_PAN if pan is None else pan
    tilt = POSE_TILT if tilt is None else tilt
    doc = App.newDocument(doc_name)
    sheet = doc.addObject("Spreadsheet::Sheet", "Parameters")
    sheet.set("A1", "Parameter (edit in cad/arm_2dof.py)")
    sheet.set("B1", "Value")
    for row, (key, value) in enumerate(parameter_table().items(), start=2):
        sheet.set("A%d" % row, key)
        sheet.set("B%d" % row, str(value))
    printed = doc.addObject("App::DocumentObjectGroup", "Printed_Parts")
    bought = doc.addObject("App::DocumentObjectGroup", "Non_Printed_Reference")
    for names, group in ((make_printed_parts(), printed), (make_vitamins(), bought)):
        for name, (shape, motion) in names.items():
            obj = doc.addObject("Part::Feature", name)
            obj.Shape = posed(shape, motion, pan, tilt)
            group.addObject(obj)
            if App.GuiUp:
                obj.ViewObject.ShapeColor = COLORS.get(name, (0.7, 0.7, 0.7))
    tester = doc.addObject("Part::Feature", "fit_test_print_only")
    tester.Shape = make_fit_test()
    tester.Placement = App.Placement(V(0, 0, -30), App.Rotation())
    printed.addObject(tester)
    if App.GuiUp:
        tester.ViewObject.Visibility = False
    doc.recompute()
    return doc


if __name__ == "__main__":
    document = build_document()
    if App.GuiUp:
        import FreeCADGui
        App.setActiveDocument(document.Name)
        view = FreeCADGui.getDocument(document.Name).activeView()
        if view is not None:
            view.viewIsometric()
            view.fitAll()
