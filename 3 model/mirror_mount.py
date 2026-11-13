"""
Smart Rear-View Mirror - 3D printable parts (Bambu Lab A1 mini, 180x180x180 mm)
Raspberry Pi 4B + USB camera module + 2x TowerPro MG995 (tilt + pan), no motor driver.

World axes (as installed in the car):
  +Z up (towards roof), +X towards the driver / rear of the car (camera side),
  +Y = lateral (left/right).  Tray top surface = Z 0.  Box hangs from the roof.

Run:  python3 mirror_mount.py      -> writes STLs (already in print orientation) to ./stl
All key dimensions are parameters below - edit and re-run if your parts differ.
"""
import math, os, sys
import numpy as np
import manifold3d as mf
from manifold3d import Manifold as M, CrossSection as CS
import trimesh

mf.set_circular_segments(64)

# ----------------------------------------------------------------------------- parameters
CLR = 0.3                 # general fit clearance
# enclosure (square)
IN = 140.0                # inner width (square)
WALL = 3.0
OUT = IN + 2 * WALL       # 146
H = 52.0                  # shell height (tray top -> roof face)
TOP = 3.0                 # top plate thickness
FLANGE_W = 14.0           # roof-mount flange width (each side, along Y) -> 174 mm total
FLANGE_T = 4.0
BAND_LOW = 18.0           # solid band at bottom of walls (ports live here)
BAND_TOP = 45.0
MESH_Z0, MESH_Z1 = 5.0, 47.5   # mesh area on the walls (z)
BOSS = 11.0               # corner boss size
BOSS_C = IN / 2 - BOSS / 2   # 64.5
PILOT_M3 = 2.8            # M3 self-tapping pilot
TRAY_T = 4.0
# Raspberry Pi 4B (85 x 56, holes 58 x 49, 3.5 from edges)
PI_X0, PI_Y0 = -58.0, -68.5      # board corner (SD-card end, port edge)
PI_STANDOFF = 6.0
PI_HOLES = [(PI_X0 + 3.5, PI_Y0 + 3.5), (PI_X0 + 61.5, PI_Y0 + 3.5),
            (PI_X0 + 3.5, PI_Y0 + 52.5), (PI_X0 + 61.5, PI_Y0 + 52.5)]
USBC_X = PI_X0 + 11.2            # USB-C centre along board
USBC_Z = PI_STANDOFF + 1.4 + 1.6 # centre height above tray
# camera (generic USB camera board 32x32 / 38x38, hole pattern 25..34 mm)
CAM_Y, CAM_Z = 12.0, 28.0
CAM_LENS_D = 19.0
# DC jack (5.5x2.1 panel jack / PG7 gland)
DC_D = 12.5
DC_Y, DC_Z = 35.0, 9.5
# MG995 servo
SV_L, SV_W = 40.9, 20.0          # case length / width
SV_TAB_Z = 26.8                  # case bottom -> underside of ears
SV_TAB_T = 2.5
SV_H = 42.7                      # case bottom -> spline tip
SV_CASE_H = 37.0
SV_SHAFT_OFF = 10.2              # shaft centre from near end of case
SV_HOLE_L = 48.5                 # ear hole spacing (slotted +/-1)
SV_HOLE_W = 10.0
SV_EARS = 54.0
HORN_FACE = 0.6                  # horn outer face beyond spline tip
# bracket / yoke stack
FL_T = 5.0                       # servo bracket flange thickness
FL_Z1 = -TRAY_T                  # flange top (= tray bottom) -4
FL_Z0 = FL_Z1 - FL_T             # -9
FL_BOLT = 23.0
AXIS_Z = -48.0                   # tilt axis height
C1 = 9.4                         # cheek1 (+Y face) position
SV1_BOT_Y = C1 - SV_TAB_Z        # case bottom of servo1  (-17.4)
CHEEK2_IN = SV1_BOT_Y - 2.0      # -19.4
CHEEK_T = 5.0
ARM_T = 5.0
ARM1_IN = C1 - SV_TAB_Z + SV_H + HORN_FACE   # 25.9 horn face
ARM2_IN = CHEEK2_IN - CHEEK_T - 1.5          # -25.9 (1.5 mm washer gap)
SV2_TOP = AXIS_Z - 28.0          # top (case bottom) of servo2, upside-down
PLATE_Z0 = SV2_TOP - SV_TAB_Z    # underside of yoke plate = ear contact
PLATE_Z1 = PLATE_Z0 + 5.0
SV2_TIP = PLATE_Z0 - SV_TAB_T - (SV_H - SV_TAB_Z - SV_TAB_T)
ADP_TOP = SV2_TIP - HORN_FACE

# ----------------------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return M.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))

def cyl(r, h, axis='z', at=(0, 0, 0), seg=0, r2=None):
    c = M.cylinder(h, r, r if r2 is None else r2, seg)
    if axis == 'x':
        c = c.rotate((0, 90, 0))
    elif axis == 'y':
        c = c.rotate((-90, 0, 0))
    return c.translate(at)

def U(parts):
    parts = [p for p in parts if p is not None]
    return M.batch_boolean(parts, mf.OpType.Add) if len(parts) > 1 else parts[0]

def hull_pts(pts):
    return M.hull_points(np.array(pts, dtype=np.float64))

def wall_prism(poly2d, normal, depth, at):
    """Extrude a 2D outline (u = along wall, v = world Z) through a wall.
    normal 'x': u -> world Y ; normal 'y': u -> world X. Centered on `at` along the normal."""
    pts = []
    for (u, v) in poly2d:
        for d in (-depth / 2, depth / 2):
            if normal == 'x':
                pts.append((at[0] + d, at[1] + u, at[2] + v))
            else:
                pts.append((at[0] + u, at[1] + d, at[2] + v))
    return hull_pts(pts)

def hexagon(R):        # flat top/bottom -> prints on vertical walls with tiny bridges only
    return [(R * math.cos(math.radians(a)), R * math.sin(math.radians(a))) for a in range(0, 360, 60)]

def teardrop(r, point_down=True, n=48):
    """circle with a 45-deg point; on the shell the print 'up' is world -Z, so point down."""
    pts = [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    pts.append((0, -r * math.sqrt(2) if point_down else r * math.sqrt(2)))
    return pts

def slot_pts(p0, p1, w, n=16):
    pts = []
    for (cx, cy) in (p0, p1):
        for i in range(n):
            a = 2 * math.pi * i / n
            pts.append((cx + w / 2 * math.cos(a), cy + w / 2 * math.sin(a)))
    return pts

def hex_field(u0, u1, v0, v1, R, web, keep=lambda u, v: True):
    """centres of flat-top hexes fully inside [u0,u1]x[v0,v1] (honeycomb packing)."""
    Rp = R + web / math.sqrt(3)
    dx, dy = 1.5 * Rp, math.sqrt(3) * Rp
    hh = R * math.sqrt(3) / 2
    nu = int(((u1 - u0) - 2 * R) / dx) + 1
    us = (u0 + u1) / 2 - (nu - 1) * dx / 2
    vc = (v0 + v1) / 2
    out = []
    for i in range(nu):
        u = us + i * dx
        off = dy / 2 if i % 2 else 0.0
        for j in range(-60, 61):
            v = vc + off + j * dy
            if v - hh >= v0 - 1e-6 and v + hh <= v1 + 1e-6 and keep(u, v):
                out.append((u, v))
    return out

def nut_pocket(at, af, depth, axis='z'):
    R = af / math.sqrt(3) + 0.15
    c = M.cylinder(depth, R, R, 6)
    if axis == 'x':
        c = c.rotate((0, 90, 0))
    elif axis == 'y':
        c = c.rotate((-90, 0, 0))
    return c.translate(at)

# ----------------------------------------------------------------------------- 1. SHELL
def make_shell():
    o, i = OUT / 2, IN / 2
    body = box(-o, o, -o, o, 0, H) - box(-i, i, -i, i, -1, H - TOP)
    # roof flanges (+/-Y) with gussets
    fl = [box(-o, o, o - 1, o + FLANGE_W, H - FLANGE_T, H), box(-o, o, -o - FLANGE_W, -o + 1, H - FLANGE_T, H)]
    for x in (-58, -20, 20, 58):
        for s in (1, -1):
            g = hull_pts([(x + dx, s * yy, zz) for dx in (-2, 2)
                          for (yy, zz) in ((o - 0.5, H - FLANGE_T), (o + FLANGE_W - 1.5, H - FLANGE_T), (o - 0.5, H - FLANGE_T - 12))])
            fl.append(g)
    # corner bosses
    bosses = [box(sx * i - (BOSS if sx > 0 else 0), sx * i + (0 if sx > 0 else BOSS),
                  sy * i - (BOSS if sy > 0 else 0), sy * i + (0 if sy > 0 else BOSS), 0, H - 1)
              for sx in (1, -1) for sy in (1, -1)]
    shell = U([body] + fl + bosses)

    cuts = []
    # roof screw slots (M4/M5) in flanges, along Y for adjustment
    for x in (-40, 40):
        for s in (1, -1):
            pts = slot_pts((x, s * (o + FLANGE_W / 2 - 2)), (x, s * (o + FLANGE_W / 2 + 2)), 5.2)
            cuts.append(hull_pts([(px, py, z) for (px, py) in pts for z in (H - FLANGE_T - 1, H + 1)]))
    # boss pilot holes (M3 self-tap from below)
    for sx in (1, -1):
        for sy in (1, -1):
            cuts.append(cyl(PILOT_M3 / 2, 16, 'z', (sx * BOSS_C, sy * BOSS_C, -1)))
    # hex mesh on the 4 walls
    R, web = 4.0, 1.8
    um = i - BOSS - 2
    def keep_cam(u, v):        # solid camera panel on +X wall
        return not (-18 < u < 42)
    def keep_dc(u, v):         # solid pad around the DC jack on -X wall
        return not (abs(u - DC_Y) < 13 and v < DC_Z + 12)
    def keep_usb(u, v):        # solid frame around the USB-C opening on -Y wall
        return not (abs(u - USBC_X) < 15 and v < USBC_Z + 13)
    wc = o - WALL / 2
    walls = [('x', wc, keep_cam), ('x', -wc, keep_dc), ('y', wc, lambda u, v: True), ('y', -wc, keep_usb)]
    for (n, pos, kf) in walls:
        for (u, v) in hex_field(-um, um, MESH_Z0, MESH_Z1, R, web, kf):
            at = (pos, 0, 0) if n == 'x' else (0, pos, 0)
            cuts.append(wall_prism([(u + a, v + b) for (a, b) in hexagon(R)], n, WALL + 2, at))
    # USB-C opening for the Pi (open at the bottom edge, -Y wall)
    cuts.append(box(USBC_X - 8, USBC_X + 8, -o - 1, -i + 1, -1, USBC_Z + 6.5))
    # DC power jack on -X wall
    cuts.append(wall_prism([(DC_Y + a, DC_Z + b) for (a, b) in teardrop(DC_D / 2)], 'x', WALL + 2, (-o + WALL / 2, 0, 0)))
    # camera: lens hole + diagonal slots (fits 25..34 mm square hole patterns, M2)
    cuts.append(wall_prism([(CAM_Y + a, CAM_Z + b) for (a, b) in teardrop(CAM_LENS_D / 2)], 'x', WALL + 2, (o - WALL / 2, 0, 0)))
    for ang in (45, 135, 225, 315):
        ca, sa = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        p0 = (CAM_Y + 17.0 * ca, CAM_Z + 17.0 * sa)
        p1 = (CAM_Y + 24.5 * ca, CAM_Z + 24.5 * sa)
        cuts.append(wall_prism(slot_pts(p0, p1, 2.4), 'x', WALL + 2, (o - WALL / 2, 0, 0)))
    return shell - U(cuts)

# ----------------------------------------------------------------------------- 2. TRAY (Pi base / bottom plate)
def make_tray():
    o, i = OUT / 2, IN / 2
    tray = box(-o, o, -o, o, -TRAY_T, 0)
    adds = []
    for (x, y) in PI_HOLES:
        adds.append(cyl(3.2, PI_STANDOFF, 'z', (x, y, 0)))
    # locating tabs (sit just inside the walls)
    t0, t1 = i - CLR - 2.0, i - CLR
    adds += [box(t0, t1, -40, -20, 0, 3), box(-t1, -t0, -40, -20, 0, 3),
             box(-10, 10, t0, t1, 0, 3), box(32, 52, -t1, -t0, 0, 3)]
    tray = U([tray] + adds)
    cuts = []
    for (x, y) in PI_HOLES:
        cuts.append(cyl(1.2, PI_STANDOFF + 3, 'z', (x, y, -2.5)))          # M2.5 self-tap
    for sx in (1, -1):
        for sy in (1, -1):
            cuts.append(cyl(1.7, 10, 'z', (sx * BOSS_C, sy * BOSS_C, -6)))        # M3 clearance
            cuts.append(cyl(3.3, 2.0, 'z', (sx * BOSS_C, sy * BOSS_C, -TRAY_T - 0.01)))  # head recess
            # servo bracket bolts: M3 countersunk from top
            cuts.append(cyl(1.7, 10, 'z', (sx * FL_BOLT, sy * FL_BOLT, -6)))
            cuts.append(cyl(1.7, 1.75, 'z', (sx * FL_BOLT, sy * FL_BOLT, -1.75), r2=3.45).translate((0, 0, 0.001)))
    # servo cable pass-through
    cuts.append(hull_pts([(x, y, z) for x in (-7, 7) for y in (-11, -3) for z in (-6, 2)]))
    # vents (hex), avoiding standoffs, bolts, flange zone, tabs and the module bay
    bay = (-62, -14, 14, 60)
    def keep(u, v):
        if abs(u) < 33 and abs(v) < 33: return False
        if bay[0] < u < bay[1] and bay[2] < v < bay[3]: return False
        for (x, y) in PI_HOLES:
            if math.hypot(u - x, v - y) < 7: return False
        return True
    for (u, v) in hex_field(-i + 8, i - 8, -i + 8, i - 8, 3.5, 2.0, keep):
        cuts.append(hull_pts([(u + a, v + b, z) for (a, b) in hexagon(3.5) for z in (-TRAY_T - 1, 1)]))
    # zip-tie slots for a buck converter (LM2596 / XL4015) in the bay
    for x in (-50, -26):
        for y in (37 - 16, 37 + 16):
            cuts.append(box(x - 2.5, x + 2.5, y - 1.5, y + 1.5, -TRAY_T - 1, 1))
    return tray - U(cuts)

# ----------------------------------------------------------------------------- 3. SERVO-1 (tilt) BRACKET
SV1_ZB = AXIS_Z - SV_SHAFT_OFF              # case end near shaft (bottom)
SV1_ZT = AXIS_Z + SV_L - SV_SHAFT_OFF       # far case end (top)
SV1_ZC = (SV1_ZB + SV1_ZT) / 2

def make_bracket():
    fl = box(-29, 29, -29, 29, FL_Z0, FL_Z1)
    zb1 = AXIS_Z - 20
    cheek1 = hull_pts([(x, y, z) for y in (C1 - CHEEK_T, C1) for (x, z) in
                       ((-15, FL_Z0), (15, FL_Z0), (15, zb1 + 5), (10, zb1), (-10, zb1), (-15, zb1 + 5))])
    zb2 = AXIS_Z - 12
    cheek2 = hull_pts([(x, y, z) for y in (CHEEK2_IN - CHEEK_T, CHEEK2_IN) for (x, z) in
                       ((-15, FL_Z0), (15, FL_Z0), (15, zb2 + 4), (9, zb2), (-9, zb2), (-15, zb2 + 4))])
    webs = [box(15, 19, CHEEK2_IN - CHEEK_T, C1, AXIS_Z - 4, FL_Z0 + 0.5),
            box(-19, -15, CHEEK2_IN - CHEEK_T, C1, AXIS_Z - 4, FL_Z0 + 0.5)]
    gus = []
    for x in (11.5, -15):
        gus.append(hull_pts([(xx, y, z) for xx in (x, x + 3.5) for (y, z) in
                             ((C1 - 0.5, FL_Z0 + 0.5), (C1 + 12, FL_Z0 + 0.5), (C1 - 0.5, FL_Z0 - 13))]))
    b = U([fl, cheek1, cheek2] + webs + gus)
    cuts = []
    # servo body window in cheek1
    cuts.append(box(-SV_W / 2 - CLR, SV_W / 2 + CLR, C1 - CHEEK_T - 1, C1 + 1, SV1_ZB - CLR, SV1_ZT + CLR))
    # ear slots (M3), slotted +/-1 mm along servo length
    for zc in (SV1_ZC - SV_HOLE_L / 2, SV1_ZC + SV_HOLE_L / 2):
        for x in (-SV_HOLE_W / 2, SV_HOLE_W / 2):
            pts = slot_pts((x, zc - 1), (x, zc + 1), 3.4)
            cuts.append(hull_pts([(px, y, pz) for (px, pz) in pts for y in (C1 - CHEEK_T - 1, C1 + 1)]))
    # pivot: M4 through cheek2 + counterbore for button head on inner face
    cuts.append(cyl(2.15, 20, 'y', (0, CHEEK2_IN - 12, AXIS_Z)))
    cuts.append(cyl(4.3, 2.6, 'y', (0, CHEEK2_IN - 2.6, AXIS_Z)))
    # flange: bolts + nut pockets (from below), cable slot
    for sx in (1, -1):
        for sy in (1, -1):
            cuts.append(cyl(1.7, 10, 'z', (sx * FL_BOLT, sy * FL_BOLT, FL_Z0 - 1)))
            cuts.append(nut_pocket((sx * FL_BOLT, sy * FL_BOLT, FL_Z0 - 0.01), 5.5, 2.6))
    cuts.append(hull_pts([(x, y, z) for x in (-7, 7) for y in (-17, -3) for z in (FL_Z0 - 1, FL_Z1 + 1)]))
    return b - U(cuts)

# ----------------------------------------------------------------------------- 4. TILT YOKE (carries servo-2)
SV2_X0 = -SV_SHAFT_OFF                      # servo2 case spans X (shaft at x=0)
SV2_X1 = SV_L - SV_SHAFT_OFF
SV2_XC = (SV2_X0 + SV2_X1) / 2

def horn_face_features(face_y, outward):
    """centre access hole + two engraved alignment rings (20 / 24 mm horns)."""
    cuts = [cyl(3.25, 30, 'y', (0, face_y - 15, AXIS_Z))]
    ring_d = 0.5
    for d in (20.4, 24.4):
        ring = cyl(d / 2, ring_d, 'y', (0, 0, 0)) - cyl(d / 2 - 0.5, ring_d + 2, 'y', (0, -1, 0))
        y0 = face_y if outward > 0 else face_y - ring_d   # engrave into the inner face
        cuts.append(ring.translate((0, y0 - (0 if outward > 0 else 0), AXIS_Z)))
    return cuts

def make_yoke():
    X0, X1 = -24, 44
    parts = []
    for (y0, y1) in ((ARM1_IN, ARM1_IN + ARM_T), (ARM2_IN - ARM_T, ARM2_IN)):
        prof = [(14 * math.cos(2 * math.pi * k / 48), AXIS_Z + 14 * math.sin(2 * math.pi * k / 48)) for k in range(48)]
        prof += [(X0, PLATE_Z0), (X1, PLATE_Z0), (X0, PLATE_Z1 + 6), (X1, PLATE_Z1 + 6)]
        parts.append(hull_pts([(x, y, z) for (x, z) in prof for y in (y0, y1)]))
    plate = box(X0, X1, ARM2_IN - ARM_T, ARM1_IN + ARM_T, PLATE_Z0, PLATE_Z1)
    parts.append(plate)
    # fillets between arms and plate (outside the servo/nut zone)
    for (xa, xb) in ((X0, -19), (39, X1)):
        for s in (1, -1):
            yin = ARM1_IN if s > 0 else ARM2_IN
            parts.append(hull_pts([(x, y, z) for x in (xa, xb) for (y, z) in
                                   ((yin + s * 0.5, PLATE_Z1 - 0.5), (yin - s * 8, PLATE_Z1 - 0.5), (yin + s * 0.5, PLATE_Z1 + 8))]))
    y = U(parts)
    cuts = []
    # lightening / cable window in arms
    for (y0, y1) in ((ARM1_IN - 1, ARM1_IN + ARM_T + 1), (ARM2_IN - ARM_T - 1, ARM2_IN + 1)):
        cuts.append(hull_pts([(x, yy, z) for yy in (y0, y1) for (x, z) in
                              ((8, PLATE_Z1 + 10), (32, PLATE_Z1 + 10), (8, AXIS_Z - 15))]))
    # arm1: horn interface ; arm2: M4 pivot
    cuts.append(cyl(3.25, ARM_T + 2, 'y', (0, ARM1_IN - 1, AXIS_Z)))
    for d in (20.4, 24.4):
        ring = cyl(d / 2, 0.5, 'y', (0, ARM1_IN - 0.01, AXIS_Z)) - cyl(d / 2 - 0.5, 2, 'y', (0, ARM1_IN - 1, AXIS_Z))
        cuts.append(ring)
    cuts.append(cyl(2.25, ARM_T + 2, 'y', (0, ARM2_IN - ARM_T - 1, AXIS_Z)))
    # servo-2 window + slotted ear holes (M3, nuts on top)
    cuts.append(box(SV2_X0 - CLR, SV2_X1 + CLR, -SV_W / 2 - CLR, SV_W / 2 + CLR, PLATE_Z0 - 1, PLATE_Z1 + 1))
    for xc in (SV2_XC - SV_HOLE_L / 2, SV2_XC + SV_HOLE_L / 2):
        for yy in (-SV_HOLE_W / 2, SV_HOLE_W / 2):
            pts = slot_pts((xc - 1, yy), (xc + 1, yy), 3.4)
            cuts.append(hull_pts([(px, py, z) for (px, py) in pts for z in (PLATE_Z0 - 1, PLATE_Z1 + 1)]))
    # zip-tie slots on plate edges for the servo cables
    for x in (-20, 40):
        cuts.append(box(x - 1.5, x + 1.5, 12, 17, PLATE_Z0 - 1, PLATE_Z1 + 1))
    return y - U(cuts)

# ----------------------------------------------------------------------------- 5. MIRROR ADAPTER + 6. CLAMP CAP
HUB_D, HUB_T = 36.0, 5.0
BLK_T, GROOVE_W = 14.0, 20.0
GAP = 2.0
CAP_T = 14.0
BOLTS = [(sx * 12, sy * 13.5) for sx in (1, -1) for sy in (1, -1)]

def vgroove(z_open, w, deep_dir, x0=-30, x1=30):
    h = w / 2
    return hull_pts([(x, y, z) for x in (x0, x1) for (y, z) in
                     ((-h - 0.6, z_open - 0.6 * deep_dir), (h + 0.6, z_open - 0.6 * deep_dir), (0, z_open + h * deep_dir))])

def make_adapter():
    z1 = ADP_TOP
    hub = cyl(HUB_D / 2, HUB_T, 'z', (0, 0, z1 - HUB_T))
    zb = z1 - HUB_T - BLK_T
    blk = box(-20, 20, -18, 18, zb, z1 - HUB_T + 0.01)
    a = hub + blk
    cuts = [cyl(3.25, 40, 'z', (0, 0, zb - 1))]                    # horn-screw access
    for d in (20.4, 24.4):
        cuts.append(cyl(d / 2, 0.5, 'z', (0, 0, z1 - 0.5)) - cyl(d / 2 - 0.5, 2, 'z', (0, 0, z1 - 1)))
    cuts.append(vgroove(zb, GROOVE_W, +1))
    for (x, y) in BOLTS:
        cuts.append(cyl(2.2, 11, 'z', (x, y, zb - 1)))
        s = 1 if y > 0 else -1
        R = 7.2 / math.sqrt(3) + 0.1
        cuts.append(box(x - 3.7, x + 3.7, min(y - s * R, s * 19), max(y - s * R, s * 19), zb + 5.5, zb + 9.0))
    return a - U(cuts)

def make_cap():
    zt = ADP_TOP - HUB_T - BLK_T - GAP
    cap = box(-20, 20, -18, 18, zt - CAP_T, zt)
    cuts = [vgroove(zt, GROOVE_W, -1)]
    for (x, y) in BOLTS:
        cuts.append(cyl(2.25, CAP_T + 2, 'z', (x, y, zt - CAP_T - 1)))
        cuts.append(cyl(4.1, 3.5, 'z', (x, y, zt - CAP_T - 0.01)))
    return cap - U(cuts)

# ----------------------------------------------------------------------------- dummies (fit check / render only)
def dummy_servo1():
    y0 = SV1_BOT_Y
    case = box(-SV_W / 2, SV_W / 2, y0, y0 + SV_CASE_H, SV1_ZB, SV1_ZT)
    ears = box(-SV_W / 2, SV_W / 2, C1, C1 + SV_TAB_T, SV1_ZC - SV_EARS / 2, SV1_ZC + SV_EARS / 2)
    spline = cyl(3, SV_H - SV_CASE_H, 'y', (0, y0 + SV_CASE_H, AXIS_Z))
    horn = cyl(10, 2.5, 'y', (0, ARM1_IN - 2.5, AXIS_Z))
    return U([case, ears, spline, horn])

def dummy_servo2():
    zt = SV2_TOP
    case = box(SV2_X0, SV2_X1, -SV_W / 2, SV_W / 2, zt - SV_CASE_H, zt)
    ears = box(SV2_XC - SV_EARS / 2, SV2_XC + SV_EARS / 2, -SV_W / 2, SV_W / 2, PLATE_Z0 - SV_TAB_T, PLATE_Z0)
    spline = cyl(3, SV_H - SV_CASE_H, 'z', (0, 0, SV2_TIP))
    horn = cyl(10, 2.5, 'z', (0, 0, ADP_TOP))
    return U([case, ears, spline, horn])

def dummy_pi():
    b = box(PI_X0, PI_X0 + 85, PI_Y0, PI_Y0 + 56, PI_STANDOFF, PI_STANDOFF + 1.4)
    z = PI_STANDOFF + 1.4
    usb = box(PI_X0 + 85 - 18 + 2, PI_X0 + 85 + 2.5, PI_Y0 + 2, PI_Y0 + 54, z, z + 16)
    usbc = box(USBC_X - 4.5, USBC_X + 4.5, PI_Y0 - 1.2, PI_Y0 + 6, z, z + 3.2)
    return U([b, usb, usbc])

def dummy_cam():
    xo = OUT / 2 - WALL
    board = box(xo - 12, xo - 10.4, CAM_Y - 19, CAM_Y + 19, CAM_Z - 19, CAM_Z + 19)
    lens = cyl(7, 13, 'x', (xo - 10.4, CAM_Y, CAM_Z))
    return board + lens

def rot_about_tilt(m, deg):
    return m.translate((0, 0, -AXIS_Z)).rotate((0, deg, 0)).translate((0, 0, AXIS_Z))

# ----------------------------------------------------------------------------- export
def to_tm(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3], faces=np.asarray(mesh.tri_verts), process=False)

def place_on_bed(m, flip=False):
    if flip:
        m = m.rotate((180, 0, 0))
    bb = m.bounding_box()
    return m.translate((-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2]))

if __name__ == '__main__':
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'stl')
    os.makedirs(out, exist_ok=True)
    parts = {
        '1_enclosure_shell': (make_shell(), True),
        '2_enclosure_tray_pi_base': (make_tray(), False),
        '3_servo1_tilt_bracket': (make_bracket(), True),
        '4_tilt_yoke_servo2_holder': (make_yoke(), False),
        '5_mirror_adapter': (make_adapter(), False),
        '6_mirror_clamp_cap': (make_cap(), False),
    }
    import pickle
    world = {}
    for name, (m, flip) in parts.items():
        world[name] = m
        p = place_on_bed(m, flip)
        bb = p.bounding_box()
        size = (bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2])
        ok = all(s <= 180 for s in size)
        tm = to_tm(p)
        tm.export(os.path.join(out, name + '.stl'))
        print(f"{name:32s} {size[0]:6.1f} x {size[1]:6.1f} x {size[2]:6.1f} mm  vol {p.volume()/1000:6.1f} cm3  "
              f"watertight={tm.is_watertight}  fits_A1mini={ok}")
    # assembly mesh for previews
    for k, m in world.items():
        to_tm(m).export(os.path.join(out, '..', 'world_' + k + '.stl'))
    for k, m in (('servo1', dummy_servo1()), ('servo2', dummy_servo2()), ('pi', dummy_pi()), ('cam', dummy_cam())):
        to_tm(m).export(os.path.join(out, '..', 'dummy_' + k + '.stl'))
