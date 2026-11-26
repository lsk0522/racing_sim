# LSK H/SEQ Shifter — author: Sunggwan
"""
LSK H/SEQ Shifter v0.4 — modular, CNC 6061 + ABS/PETG prints. Parametric CAD (build123d)
Origin = gimbal centre. X = forward, Y = left, Z = up. Units: mm
"""
import math, json
import numpy as np
from build123d import *
from shapely.geometry import Point as SPoint, Polygon as SPolygon
from shapely.ops import unary_union
import cam as camgen

Z_BASE = -66.0
BASE_T = 8.0
H_GATE = 55.0
GATE_T = 3.0
LEVER_D = 8.0
L_KNOB = camgen.L_KNOB
GATE_Y = {"1-2": 15.0, "3-4": 0.0, "5-6": -15.0, "7-R": -30.0}
PHI = {k: math.degrees(math.atan(v / H_GATE)) for k, v in GATE_Y.items()}
THETA_STOP = camgen.STOP_DEG      # 14
THETA_GATE_END = 14.5
SEQ_STOP = 8.0                    # SEQ lock pocket pitch limit
BRG = dict(od=16.0, w=5.0)
YC = -34.75                       # roller/cam centre Y
CAM_Y0, CAM_Y1 = -43.0, -26.5
ARM_P = (camgen.P[0], camgen.P[1])

# ======================= Helpers =======================
def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cyl_x(r, x0, x1, y=0.0, z=0.0):
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, x1 - x0)


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, y1 - y0)


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, z1 - z0)


def xz_prism(pts, y0, y1):
    """XZ polygon → solid over the Y range y0..y1."""
    f = Plane.XZ * Polygon(*pts, align=None)
    p = extrude(f, amount=y1 - y0)          # direction depends on point winding → realigned by bbox
    return p.moved(Location((0, y0 - p.bounding_box().min.Y, 0)))


def xy_prism(pts, z0, z1):
    f = Plane.XY * Polygon(*pts, align=None)
    p = extrude(f, amount=z1 - z0)
    return p.moved(Location((0, 0, z0 - p.bounding_box().min.Z)))


def hex_z(af, z0, z1, x=0.0, y=0.0):
    r = af / math.sqrt(3)
    return Pos(x, y, z0) * extrude(RegularPolygon(r, 6), amount=z1 - z0)


def hex_y(af, y0, y1, x=0.0, z=0.0):
    r = af / math.sqrt(3)
    return Pos(x, y1, z) * Rot(90, 0, 0) * extrude(RegularPolygon(r, 6), amount=y1 - y0)


def shapely_to_pts(poly, tol=0.03):
    p = poly.simplify(tol)
    c = list(p.exterior.coords)[:-1]
    return [(float(x), float(y)) for x, y in c]


def ring_xz(r_out, r_in, a0, a1, n=48):
    """XZ sector ring (angles in deg, 0° = +X, −90° = −Z)."""
    A = np.radians(np.linspace(a0, a1, n))
    outer = [(r_out * math.cos(a), r_out * math.sin(a)) for a in A]
    inner = [(r_in * math.cos(a), r_in * math.sin(a)) for a in A[::-1]]
    return outer + inner


# ---------- Gate pattern ----------
def lever_point(theta, phi, z):
    t, p = math.radians(theta), math.radians(phi)
    return (z * math.tan(t), z * math.tan(p) / math.cos(t))


def lever_section(theta, phi, z, r):
    """Cross-section of the lever cylinder at plane z = ellipse (approximation: stretched by 1/cos along the tilt direction)."""
    t, p = math.radians(theta), math.radians(phi)
    d = np.array([math.cos(p) * math.sin(t), math.sin(p), math.cos(p) * math.cos(t)])
    cx, cy = lever_point(theta, phi, z)
    tilt = math.acos(d[2])
    circ = SPoint(0, 0).buffer(r, 32)
    from shapely import affinity
    if tilt > 1e-6:
        ang = math.degrees(math.atan2(d[1], d[0]))
        e = affinity.rotate(affinity.scale(affinity.rotate(circ, -ang, origin=(0, 0)), 1 / math.cos(tilt), 1, origin=(0, 0)), ang, origin=(0, 0))
    else:
        e = circ
    return affinity.translate(e, cx, cy)


def gate_opening(clear=0.5):
    zb, zt = H_GATE - GATE_T / 2, H_GATE + GATE_T / 2
    shapes = []
    r = LEVER_D / 2 + clear
    poses = []
    # Neutral channel
    for phi in np.linspace(PHI["7-R"], PHI["1-2"], 60):
        for th in np.linspace(-0.4, 0.4, 3):
            poses.append((th, phi))
    for g, phi in PHI.items():
        for th in np.linspace(-THETA_GATE_END, THETA_GATE_END, 51):
            poses.append((th, phi))
    for th, phi in poses:
        for z in (zb, zt):
            shapes.append(lever_section(th, phi, z, r))
    return unary_union(shapes).buffer(0.01).simplify(0.02)



# ---------- Gate pattern ----------
def lever_point(theta, phi, z):
    t, p = math.radians(theta), math.radians(phi)
    return (z * math.tan(t), z * math.tan(p) / math.cos(t))


def lever_section(theta, phi, z, r):
    """Cross-section of the lever cylinder at plane z = ellipse (approximation: stretched by 1/cos along the tilt direction)."""
    t, p = math.radians(theta), math.radians(phi)
    d = np.array([math.cos(p) * math.sin(t), math.sin(p), math.cos(p) * math.cos(t)])
    cx, cy = lever_point(theta, phi, z)
    tilt = math.acos(d[2])
    circ = SPoint(0, 0).buffer(r, 32)
    from shapely import affinity
    if tilt > 1e-6:
        ang = math.degrees(math.atan2(d[1], d[0]))
        e = affinity.rotate(affinity.scale(affinity.rotate(circ, -ang, origin=(0, 0)), 1 / math.cos(tilt), 1, origin=(0, 0)), ang, origin=(0, 0))
    else:
        e = circ
    return affinity.translate(e, cx, cy)


def gate_opening(clear=0.5):
    zb, zt = H_GATE - GATE_T / 2, H_GATE + GATE_T / 2
    shapes = []
    r = LEVER_D / 2 + clear
    poses = []
    # Neutral channel
    for phi in np.linspace(PHI["7-R"], PHI["1-2"], 60):
        for th in np.linspace(-0.4, 0.4, 3):
            poses.append((th, phi))
    for g, phi in PHI.items():
        for th in np.linspace(-THETA_GATE_END, THETA_GATE_END, 51):
            poses.append((th, phi))
    for th, phi in poses:
        for z in (zb, zt):
            shapes.append(lever_section(th, phi, z, r))
    return unary_union(shapes).buffer(0.01).simplify(0.02)



# ======================= Parts (v0.2 compact) =======================

# ======================= v0.4 modular / CNC 6061 =======================
# Module code: [M1] base+electronics  [M2] towers  [M3] gimbal core  [M4] lateral spring cartridge
#              [M5] detent cassette   [M6] SEQ lock  [M7] gate  [M8] knob  [M9] sensor caps

def soft(p, r):
    """2D rounding (outer and inner corners at radius r)."""
    return p.buffer(r, join_style=1).buffer(-r, join_style=1).buffer(-r, join_style=1).buffer(r, join_style=1)


def plate_xz(p2d, y0, y1, ch=0.0):
    s = xz_prism(shapely_to_pts(SPolygon(p2d.exterior), 0.02), y0, y1)
    for h in p2d.interiors:
        s -= xz_prism([(float(x), float(z)) for x, z in list(h.coords)[:-1]], y0 - 1, y1 + 1)
    return _chamfer_faces(s, ch, Axis.Y)


def plate_xy(p2d, z0, z1, ch=0.0):
    s = xy_prism(shapely_to_pts(SPolygon(p2d.exterior), 0.02), z0, z1)
    for h in p2d.interiors:
        s -= xy_prism([(float(x), float(y)) for x, y in list(h.coords)[:-1]], z0 - 1, z1 + 1)
    return _chamfer_faces(s, ch, Axis.Z)


def _chamfer_faces(s, ch, ax):
    if ch <= 0:
        return s
    try:
        faces = s.faces().filter_by(ax)
        edges = faces.edges()
        return s.chamfer(ch, None, edges)
    except Exception:
        return s


# ---------- [M3] gimbal core ----------
def make_hub():                                   # 6061 CNC (ABS printing also possible)
    h = box(-10, 10, -7, 7, -14, 18)
    h += cyl_x(4.5, 10, 11) + cyl_x(4.5, -11, -10)
    h -= cyl_x(2.55, -12, 12)
    h -= cyl_z(LEVER_D / 2 + 0.01, 3, 19)          # Ø8 H7 lever (press fit + set screw)
    for z in (9, 14):
        h -= cyl_x(1.7, 0, 11, z=z)                # M4 tapped (set screw)
    h -= cyl_z(2.55, -15, -4)
    h -= box(-4.1, 4.1, -4.1, 8, -12.2, -7.8)
    return h


def make_lever():
    return cyl_z(LEVER_D / 2, 3, L_KNOB + 7)


def make_tail():
    t = cyl_z(2.5, -49.5, -4.5) + hex_z(8, -53, -49.5)
    cap = cyl_z(5.0, -54, -47.5) - hex_z(8.2, -53.2, -49.2) - cyl_z(2.7, -49.5, -46.5)
    cap -= cyl_z(2.0, -54.5, -52.5)                # (keep the tip-cap bottom flat)
    return t, cap


def make_cradle():                                # 6061 CNC — box frame only (springs are a separate cartridge)
    c = box(-18, 18, -26.5, 26.5, -12, 12) - box(-11, 11, -20.5, 20.5, -13, 13)
    try:
        c = c.fillet(1.0, c.edges().filter_by(Axis.Z))
    except Exception:
        pass
    c -= cyl_x(BRG["od"] / 2 + 0.01, 18 - BRG["w"] - 0.2, 19)
    c -= cyl_x(BRG["od"] / 2 + 0.01, -19, -18 + BRG["w"] + 0.2)
    c -= cyl_x(4.5, -19, 19)
    c -= cyl_y(2.55, -27, -19)                     # −Y: M5 bolt (clamped)
    c -= hex_y(8.2, -24.5, -20.5)
    c -= cyl_y(2.5, 19, 27)                        # +Y: Ø5 hollow pin (glued/pressed)
    c -= box(-1.5, 1.5, 19.5, 21, -1.5, 12.5)      # cable groove: pin inner end → top of the cradle (inner face)
    for x in (-11, 11):
        c -= cyl_y(1.25, -27, -21, x=x, z=-6)      # M3 tap (cam)
    for y0, y1 in ((-26.5, -20.5), (20.5, 26.5)):
        c += box(-10, 10, y0, y1, -20, -11)
    for x in (-7, 7):                              # M3 tap (spring cartridge)
        for y in (-23.5, 23.5):
            c -= cyl_z(1.25, -21, -14, x=x, y=y)
    c -= box(14, 19, -1.5, 1.5, 6, 13)             # roll sensor cable groove (+X face → top)
    return c


def make_roll_sensor_cap():                       # [M9] printed: roll sensor (cradle +X), magnet on the M5 bolt head
    cap = box(18, 27, -12.5, 12.5, -12, 12)
    cap -= box(17, 24.5, -11.6, 11.6, -11.6, 11.6)  # board space (MT6701/AS5600 ~ 23 mm)
    cap -= cyl_x(5.0, 17, 22)
    return cap.fillet(1.2, cap.edges().filter_by(Axis.X)) if False else cap


# ---------- [M4] lateral spring cartridge ----------
def make_cartridge():
    c = box(-10, 10, -26, 47, -39, -20)
    c -= box(-3.6, 3.6, -13.5, 24, -40, -19)
    c -= box(-5.5, 5.5, 12.3, 26.5, -38.2, -29.8)
    c -= cyl_y(4.3, -24.1, -11, z=-24)
    c -= cyl_y(4.3, 22, 30.1, z=-24)
    c -= cyl_y(4.3, 26, 41, z=-34)                 # S2 (OD8 wire1.0 L0 27, k≈5)
    c -= cyl_y(2.1, 40, 48, z=-34)                 # M5 tap (S2 preload, accessible from +Y)
    for x in (-7, 7):
        for y in (-23.5, 23.5):
            c -= cyl_z(1.7, -40, -19, x=x, y=y)
            c -= cyl_z(3.0, -40, -36, x=x, y=y)    # M3 counterbore
    return c


def make_plungers():
    p = {}
    p["plunger_S1_L"] = box(-3.4, 3.4, -6.5, -2.5, -27.8, -20.2) + cyl_y(2.4, -10.5, -6.5, z=-24)
    p["plunger_S1_R"] = box(-3.4, 3.4, 2.5, 6.5, -27.8, -20.2) + cyl_y(2.4, 6.5, 10.5, z=-24)
    p["plunger_S2"] = box(-5.3, 5.3, 12.3, 16.3, -37.8, -30.2) + cyl_y(2.4, 16.3, 20.3, z=-34)
    p["cup_S2"] = cyl_y(4.1, 38.0, 40.0, z=-34)
    return p


# ---------- [M5] detent cassette ----------
def cam2d(track):
    web = unary_union([SPolygon(ring_xz(30, 6, -90 - 38, -90 + 38)).buffer(0), SPoint(0, 0).buffer(16, 64)])
    p = unary_union([track, web]).buffer(0)
    p = p.difference(SPoint(0, 0).buffer(2.6, 32))
    for x in (-11, 11):
        p = p.difference(SPoint(x, -6).buffer(1.7, 24))
    p = p.difference(SPolygon(ring_xz(22, 10, -90 - 26, -90 + 26)).buffer(-1.5).buffer(1.5))   # lightening window (skeleton)
    return p


def make_cam(camH):
    return plate_xz(cam2d(camH), CAM_Y0, CAM_Y1, ch=0.5)


def make_rollers():
    r = Part()
    for y0 in (YC - 7.8, YC - 2.5, YC + 2.8):
        r += cyl_y(8.0, y0, y0 + 5.0, z=-45) - cyl_y(2.5, y0 - 1, y0 + 6, z=-45)
    return r


def arm2d():
    p = unary_union([
        SPolygon([(-71, -49), (0, -49), (0, -41), (-71, -41)]),
        SPoint(0, -45).buffer(4.5, 32), SPoint(-30, -45).buffer(4.5, 32),
    ])
    p = soft(p, 1.0)
    p = p.difference(SPoint(0, -45).buffer(2.55, 24)).difference(SPoint(-30, -45).buffer(2.55, 24))
    p = p.difference(SPoint(-65, -45).buffer(1.6, 16)).difference(SPoint(-15, -45).buffer(1.6, 16))
    return p


def make_arm():                                   # Al 3t ×2 (DXF) + spacers
    a = plate_xz(arm2d(), YC + 8.1, YC + 11.1) + plate_xz(arm2d(), YC - 11.1, YC - 8.1)
    a += box(-71, -59, YC - 8.1, YC + 8.1, -45, -41)       # rear spring pad (6061 block)
    a -= cyl_z(6.2, -42.5, -40, x=-65, y=YC)
    a += cyl_y(3, YC - 8.1, YC + 8.1, x=-15, z=-45)        # spacer
    return a


def make_cassette():                              # 6061 CNC — follower frame + spring tower
    zb = Z_BASE
    c = box(-78, -22, -49.2, -20.3, zb, zb + 5)
    c -= box(-60, -38, -44, -26, zb - 1, zb + 6)           # lightening
    for y0, y1 in ((-23.3, -20.3), (-49.2, -46.2)):
        c += box(-36, -24, y0, y1, zb + 5, -39)
    c -= cyl_y(2.55, -50, -19, x=-30, z=-45)
    c += box(-78, -73, YC - 7, YC + 7, zb + 5, -14.7)
    c += box(-78, -56, YC - 7, YC + 7, -18.7, -14.7)
    c -= cyl_z(2.5, -20, -14, x=-65, y=YC)                 # M6 tap (preload)
    for x, y in ((-72, -24), (-72, -46), (-28, -24), (-28, -46)):
        c -= cyl_z(1.7, zb - 1, zb + 6, x=x, y=y)          # M3 → base
    return c


# ---------- [M6] SEQ lock ----------
SEQ_LIFT = 6.0
LOCK_PIV = (-30.0, -60.0)
SEQ_ANG = math.degrees(math.asin(SEQ_LIFT / 30.0))
POCKET_X = 13.2      # +X end (θ<0 side, tilted outward)
POCKET_XN = 11.0     # −X end (θ>0 side)


def make_seq_lock(seq=False):
    a = box(-34, 16, -8.3, 8.3, -65, -56)
    a -= box(-POCKET_XN, POCKET_X, -5.3, 5.3, -62.5, -55)      # pocket: Y ±5.3 (lateral lock), X = SEQ pitch stop
    a -= box(-26, -14, -5, 5, -66, -60)                     # lightening
    a += box(-2, 6, 8.3, 17, -61, -58)                      # side tab (rides on the eccentric: H −61 / SEQ −55)
    a -= cyl_y(1.6, -9, 9, x=LOCK_PIV[0], z=LOCK_PIV[1])
    if seq:
        a = a.rotate(Axis((LOCK_PIV[0], 0, LOCK_PIV[1]), (0, 1, 0)), -SEQ_ANG)
    return a


def make_lock_pivot():
    p = Part()
    for y0, y1 in ((-12, -8.6), (8.6, 12)):
        p += box(-36, -24, y0, y1, Z_BASE, -54) - cyl_y(1.6, y0 - 1, y1 + 1, x=LOCK_PIV[0], z=LOCK_PIV[1])
    return p


SHAFT_Y, SHAFT_Z = 13.3, -63.5


def make_mode_shaft(seq=False):
    """Eccentric (R5.5, offset 3) on a Ø5 shaft. H: tab bottom −61, SEQ: rotated 180° → +6 mm (both ends on dead centre = self-locking)."""
    off = 3.0 if seq else -3.0
    s = cyl_x(2.5, -2, 51, y=SHAFT_Y, z=SHAFT_Z)
    s += cyl_x(5.5, -2, 6, y=SHAFT_Y, z=SHAFT_Z + off)
    return s


def make_mode_knob(seq=False):
    k = cyl_x(10, 42, 50, y=SHAFT_Y, z=SHAFT_Z)
    for i in range(20):
        k -= Pos(0, SHAFT_Y, SHAFT_Z) * Rot(18 * i, 0, 0) * box(41, 51, -0.5, 0.5, 9.2, 10.5)
    k += box(48, 50, SHAFT_Y - 1, SHAFT_Y + 1, SHAFT_Z + 5, SHAFT_Z + 9.5)   # pointer
    k -= cyl_x(2.5, 41, 51, y=SHAFT_Y, z=SHAFT_Z)
    if seq:
        k = k.rotate(Axis((0, SHAFT_Y, SHAFT_Z), (1, 0, 0)), 180)
    return k


def make_shaft_bearings():
    b = Part()
    for x0 in (18, 33):
        b += box(x0, x0 + 6, SHAFT_Y - 6, SHAFT_Y + 6, Z_BASE, SHAFT_Z + 6) - cyl_x(2.6, x0 - 1, x0 + 7, y=SHAFT_Y, z=SHAFT_Z)
    return b


# ---------- [M2] towers ----------
def tower2d(side):
    outer = SPolygon([(-80, Z_BASE), (36, Z_BASE), (36, H_GATE - GATE_T / 2), (-40, H_GATE - GATE_T / 2), (-80, -12)])
    p = outer
    if side < 0:
        win = soft(SPolygon([(-27, -57), (27, -57), (27, -20), (-27, -20)]), 8)       # window onto the cam/rollers
        p = p.difference(win)
        p = p.difference(soft(SPolygon([(-70, -57), (-44, -57), (-44, -16), (-62, -16)]), 5))
    else:
        win = soft(SPolygon([(-21, -57.5), (21, -57.5), (21, -16), (-21, -16)]), 4)   # cartridge + tail cap passage
        p = p.difference(win)
        p = p.difference(soft(SPolygon([(-70, -57), (-34, -57), (-34, -16), (-62, -16)]), 5))
    for r in ([(-34, 12), (-14, 12), (-14, 46), (-26, 46)], [(14, 12), (28, 12), (28, 46), (14, 46)]):
        p = p.difference(soft(SPolygon(r), 4))
    p = soft(p, 2)
    p = p.difference(SPoint(0, 0).buffer(BRG["od"] / 2 + 0.01, 64))                  # F625 bore
    for x, z in ((-11, 0), (11, 0)):
        p = p.difference(SPoint(x, z).buffer(1.25, 16))                              # M3 tap (sensor cap / pin collar)
    return p


def make_tower(side):
    y0, y1 = ((-56.0, -50.0) if side < 0 else (28.0, 34.0))
    t = plate_xz(tower2d(side), y0, y1, ch=0.6)
    yc = (y0 + y1) / 2
    for x in (-30, 26):
        t -= cyl_z(1.25, H_GATE - GATE_T / 2 - 8, H_GATE, x=x, y=yc)      # M3 tap top edge (gate)
    for x in (-70, 30):
        t -= cyl_z(1.7, Z_BASE - 1, Z_BASE + 9, x=x, y=yc)                # M4 tap bottom edge (base)
    # cable groove on the outer face (sensor → base), 3×2
    yo = y0 if side < 0 else y1
    sgn = -1 if side < 0 else 1
    t -= box(-11.5, -8.5, yo - 2 if sgn > 0 else yo, yo if sgn > 0 else yo + 2, Z_BASE - 1, 0)
    return t


# ---------- [M7] gate ----------
def gate2d():
    opening = gate_opening()
    outer = soft(SPolygon([(-36, -56), (32, -56), (32, 34), (-36, 34)]), 5)
    p = outer.difference(opening)
    for y in (-53.0, 31.0):
        for x in (-30, 26):
            p = p.difference(SPoint(x, y).buffer(1.7, 16))
    return p


def make_gate():
    p2 = gate2d()
    g = plate_xy(p2, H_GATE - GATE_T / 2, H_GATE + GATE_T / 2, ch=0.5)
    # Engraving: gear numbers (0.4 mm), 1/3/5/7 at the front (+X), 2/4/6/R at the rear
    zt = H_GATE + GATE_T / 2
    labels = {"1": (23, 15), "2": (-23, 15), "3": (23, 0), "4": (-23, 0), "5": (23, -15), "6": (-23, -15),
              "7": (23, -30), "R": (-23, -30)}
    for txt, (x, y) in labels.items():
        try:
            t = Pos(x, y, zt - 0.4) * extrude(Text(txt, font_size=6, font="Noto Sans CJK JP"), amount=1.0)
            g -= t
        except Exception:
            pass
    return g, p2


# ---------- [M8] knob ----------
def make_knob():
    k = Pos(0, 0, L_KNOB) * Sphere(17)
    k &= box(-20, 20, -20, 20, L_KNOB - 11, L_KNOB + 13)                      # flat top (Ø~23)
    k += cyl_z(12, L_KNOB - 22, L_KNOB - 10)
    for i in range(18):
        k -= Rot(0, 0, 20 * i + 10) * box(11.2, 13, -0.5, 0.5, L_KNOB - 20, L_KNOB - 14)
    k -= cyl_z(LEVER_D / 2 + 0.05, L_KNOB - 23, L_KNOB + 7)
    k -= cyl_x(1.7, 0, 14, z=L_KNOB - 16)
    zt = L_KNOB + 13
    # Top engraving: 7+R H pattern (line width 1.0, depth 0.5)
    lines = [box(-6, 6, -0.5, 0.5, zt - 0.5, zt + 1)]                        # neutral line (along Y = Y direction on screen)
    lines = [box(-0.5, 0.5, -7.5, 4.5, zt - 0.5, zt + 1)]
    for y in (4.5, 0.5, -3.5, -7.5):
        lines.append(box(-6, 6, y - 0.5, y + 0.5, zt - 0.5, zt + 1))
    for ln in lines:
        k -= ln.rotate(Axis.Z, 90)
    return k


# ---------- [M9] pitch sensor cap ----------
def make_pitch_sensor_cap():
    c = box(-14, 14, -64, -56, -14, 14)
    c -= box(-12, 12, -62.5, -56.5, -12, 12)                                # board + magnet holder space
    c -= cyl_y(1.6, -65, -55, x=-11, z=0) + cyl_y(1.6, -65, -55, x=11, z=0)
    c -= box(-11.5, -8.5, -57, -55, -15, -12)                               # cable exit (toward the groove)
    return c


def make_pin_collar():
    c = cyl_y(6, 34, 38) - cyl_y(2.55, 33, 39)
    c -= cyl_y(1.6, 33, 39, x=-11, z=0) if False else Part()
    return c


# ---------- [M1] base ----------
def base2d():
    p = soft(SPolygon([(-84, -64), (40, -64), (40, 40), (-84, 40)]), 6)
    holes = []
    for x in (-70, 30):
        holes += [(x, -53), (x, 31)]                                           # M4 countersunk (towers)
    holes += [(-72, -24), (-72, -46), (-28, -24), (-28, -46)]                  # cassette M3
    for h in holes:
        p = p.difference(SPoint(*h).buffer(2.2 if h[1] in (-53, 31) else 1.7, 16))
    return p


def make_base():
    p2 = base2d()
    b = plate_xy(p2, Z_BASE - BASE_T, Z_BASE, ch=0.8)
    # Bottom-side electronics pocket (RP2040-Zero + JST-SH) + cable channels + USB-C slot
    zb = Z_BASE - BASE_T
    b -= box(-74, -40, 4, 30, zb - 1, zb + 5)                                  # RP2040 pocket 34×26×5
    b -= box(-85, -74, 11, 23, zb - 1, zb + 4)                                 # USB-C exit (rear)
    b -= box(-12, -8, -60, 8, zb - 1, zb + 3) + box(-40, -8, 4, 8, zb - 1, zb + 3)   # −Y sensor cable channel
    b -= box(-12, -8, 30, 37, zb - 1, zb + 3) + box(-40, -8, 26, 30, zb - 1, zb + 3)   # +Y cable channel
    for x, y in ((-10, -58.5), (-10, 35.5)):
        b -= cyl_z(2.0, zb - 1, Z_BASE + 1, x=x, y=y)                          # cables up through the base
    b -= box(-3, 7, SHAFT_Y - 7, SHAFT_Y + 7, Z_BASE - 6.5, Z_BASE + 1)        # eccentric relief
    for x, y in ((-36, -11), (-36, 11), (-24, -11), (-24, 11)):
        b -= cyl_z(1.25, zb - 1, Z_BASE + 1, x=x, y=y)                         # M3 tap (lock pivot)
    for x, y in ((-60, -40), (-20, -40), (-20, 20), (20, 20)):
        b -= cyl_z(2.5, zb - 1, zb + 6, x=x, y=y)                              # M6 blind tap from the bottom (40 mm spacing, rig mount)
    return b, p2


def make_bottom_cover():
    p = soft(SPolygon([(-84, -64), (40, -64), (40, 40), (-84, 40)]), 6)
    return plate_xy(p, Z_BASE - BASE_T - 2, Z_BASE - BASE_T)


# ---------- assembly ----------
def build_all():
    camH = camgen.build_cam(camgen.H_PROFILE)
    tail, tipcap = make_tail()
    base, base2 = make_base()
    gate, g2 = make_gate()
    st = {"base": base, "bottom_cover": make_bottom_cover(), "tower_negY": make_tower(-1), "tower_posY": make_tower(+1),
          "gate": gate, "cassette": make_cassette(), "lock_pivot": make_lock_pivot(),
          "shaft_bearings": make_shaft_bearings(), "pitch_sensor_cap": make_pitch_sensor_cap(),
          "pin_collar": make_pin_collar()}
    cr = {"cradle": make_cradle(), "cam": make_cam(camH), "cartridge": make_cartridge(),
          "roll_sensor_cap": make_roll_sensor_cap(), "hollow_pin": cyl_y(2.5, 20.5, 38) - cyl_y(1.5, 20, 39)}
    cr.update(make_plungers())
    hb = {"hub": make_hub(), "lever_bar": make_lever(), "knob": make_knob(), "tail_bolt": tail, "tail_cap": tipcap}
    fo = {"arm": make_arm(), "rollers": make_rollers()}
    plates2d = {"base": base2, "gate": g2, "tower_negY": tower2d(-1), "tower_posY": tower2d(1),
                "cam_H": cam2d(camH), "arm_plate": arm2d()}
    return st, cr, hb, fo, plates2d, camH


def pose(parts, theta=0.0, phi=0.0, level="cradle"):
    out = {}
    for k, v in parts.items():
        s = v
        if level == "hub":
            s = s.rotate(Axis.X, -phi)
        out[k] = s.rotate(Axis.Y, theta)
    return out


# ======================= v0.5: curved exterior =======================
from scipy.interpolate import PchipInterpolator

# Outer section key frames (z, (x0, x1, y0, y1, r)) — lower part is vertical, then it tapers like a hand-shaped pod up to the gate
SHELL_KEYS = [(-66, (-85, 42, -69, 52, 22)), (-14, (-85, 42, -69, 52, 22)), (6, (-77, 41, -69, 44, 20)),
              (28, (-60, 39, -64, 40, 15)), (53.5, (-40, 36, -60, 38, 10))]
SHELL_T = 2.6
GATE_OUT = (-40, 36, -60, 38, 10)          # gate plate outline = shell top section (lid)


def _shell_frames(dz=2.4):
    zs = np.array([k[0] for k in SHELL_KEYS]); P = np.array([k[1] for k in SHELL_KEYS], float)
    f = PchipInterpolator(zs, P, axis=0)
    Z = np.concatenate([[-66.0, -14.0], np.arange(-11.6, 53.5, dz), [53.5]])
    return Z, f


def _loft(Z, f, inset=0.0, extend=0.0):
    secs = []
    for i, z in enumerate(Z):
        a, b, c, d, r = f(z)
        zz = z - extend if i == 0 else (z + extend if i == len(Z) - 1 else z)
        secs.append(Plane.XY.offset(zz) * Pos((a + b) / 2, (c + d) / 2) *
                    RectangleRounded(b - a - 2 * inset, d - c - 2 * inset, max(r - inset, 1.0)))
    return loft(secs, ruled=True)


def make_shell():
    """[M10] Exterior shell — ABS/PETG printed (one piece, upside down) or 3D-CNC from a 6061 block."""
    Z, f = _shell_frames()
    outer = _loft(Z, f)
    inner = _loft(Z, f, inset=SHELL_T, extend=1.0)
    s = outer - inner
    # −Y window: cam/rollers visible (ellipse)
    s -= Pos(0, -60, -38) * Rot(90, 0, 0) * extrude(Ellipse(30, 15), amount=20, both=True)
    # +Y window: orange spring cartridge visible + S2 adjustment access
    s -= Pos(0, 50, -30) * Rot(90, 0, 0) * extrude(Ellipse(21, 10), amount=12, both=True)
    # Front: mode shaft
    s -= cyl_x(3.2, 35, 46, y=SHAFT_Y, z=SHAFT_Z)
    # Bottom bosses (M3 insert, fastened from under the base)
    for x, y in SHELL_BOSSES:
        s += cyl_z(4.5, Z_BASE, Z_BASE + 9, x=x, y=y) - cyl_z(2.0, Z_BASE - 1, Z_BASE + 8, x=x, y=y)
    return s


SHELL_BOSSES = [(-72, -60), (28, -61), (-72, 43), (28, 43)]


def gate2d():
    opening = gate_opening()
    a, b, c, d, r = GATE_OUT
    outer = SPolygon([(a, c), (b, c), (b, d), (a, d)]).buffer(-r).buffer(r, resolution=32)
    p = outer.difference(opening)
    for y in (-53.0, 31.0):
        for x in (-30, 22):
            p = p.difference(SPoint(x, y).buffer(1.7, 16))
    return p


def tower2d(side):
    top = H_GATE - GATE_T / 2
    outer = SPolygon([(-80, Z_BASE), (32, Z_BASE), (32, top), (-36, top), (-80, -12)])
    p = outer
    if side < 0:
        p = p.difference(soft(SPolygon([(-27, -57), (27, -57), (27, -20), (-27, -20)]), 8))
        p = p.difference(soft(SPolygon([(-70, -57), (-44, -57), (-44, -16), (-62, -16)]), 5))
    else:
        p = p.difference(soft(SPolygon([(-21, -57.5), (21, -57.5), (21, -16), (-21, -16)]), 4))
        p = p.difference(soft(SPolygon([(-70, -57), (-34, -57), (-34, -16), (-62, -16)]), 5))
    for r in ([(-32, 12), (-14, 12), (-14, 46), (-22, 46)], [(14, 12), (25, 12), (25, 46), (14, 46)]):
        p = p.difference(soft(SPolygon(r), 4))
    p = soft(p, 2)
    p = p.difference(SPoint(0, 0).buffer(BRG["od"] / 2 + 0.01, 64))
    for x, z in ((-11, 0), (11, 0)):
        p = p.difference(SPoint(x, z).buffer(1.25, 16))
    return p


def make_tower(side):
    y0, y1 = ((-56.0, -50.0) if side < 0 else (28.0, 34.0))
    t = plate_xz(tower2d(side), y0, y1, ch=0.6)
    yc = (y0 + y1) / 2
    for x in (-30, 22):
        t -= cyl_z(1.25, H_GATE - GATE_T / 2 - 8, H_GATE, x=x, y=yc)
    for x in (-70, 26):
        t -= cyl_z(1.7, Z_BASE - 1, Z_BASE + 9, x=x, y=yc)
    yo = y0 if side < 0 else y1
    t -= box(-11.5, -8.5, yo - 2 if side > 0 else yo, yo if side > 0 else yo + 2, Z_BASE - 1, 0)
    return t


BASE_OUT = (-88, 45, -72, 55, 25)


def base2d():
    a, b, c, d, r = BASE_OUT
    p = SPolygon([(a, c), (b, c), (b, d), (a, d)]).buffer(-r).buffer(r, resolution=32)
    holes = [(x, y) for x in (-70, 26) for y in (-53, 31)]
    holes += [(-72, -24), (-72, -46), (-28, -24), (-28, -46)]
    holes += SHELL_BOSSES
    for h in holes:
        p = p.difference(SPoint(*h).buffer(2.2 if h[1] in (-53, 31) else 1.7, 16))
    return p


def make_bottom_cover():
    a, b, c, d, r = BASE_OUT
    p = SPolygon([(a, c), (b, c), (b, d), (a, d)]).buffer(-r).buffer(r, resolution=32)
    return plate_xy(p, Z_BASE - BASE_T - 2, Z_BASE - BASE_T)


def make_knob():
    """Teardrop knob (revolved spline): thin neck → palm bulge → flat top (engraved)."""
    L = L_KNOB
    from scipy.interpolate import PchipInterpolator as _P
    zk = np.array([L - 24, L - 18, L - 8, L + 2, L + 9, L + 12.5])
    rk = np.array([8.5, 9.5, 14.5, 17.5, 15.5, 11.0])
    fz = _P(zk, rk)
    zz = np.linspace(zk[0], zk[-1], 40)
    prof = [(0.0, float(zk[0]))] + [(float(fz(z)), float(z)) for z in zz] + [(0.0, float(zk[-1]))]
    face = Plane.XZ * Polygon(*prof, align=None)
    k = revolve(face, Axis.Z, 360)
    k = k.fillet(1.2, k.edges().filter_by(GeomType.CIRCLE).sort_by(Axis.Z)[-1:]) if False else k
    k -= cyl_z(LEVER_D / 2 + 0.05, L - 25, L + 7)
    k -= cyl_x(1.7, 0, 12, z=L - 17)
    zt = L + 12.5
    for y in (5.25, 0.0, -5.25):                      # top engraving: 7+R simplified H (3 lines + spine)
        k -= box(-0.45, 0.45, -7.9, 5.25, zt - 0.5, zt + 1) if y == 0 else Part()
    for y in (5.25, 1.75, -1.75, -5.25):
        k -= box(-5.5, 5.5, y - 0.45, y + 0.45, zt - 0.5, zt + 1)
    return k


_old_build_all = build_all


def build_all():
    st, cr, hb, fo, plates2d, camH = _old_build_all()
    st["shell"] = make_shell()
    plates2d["gate"] = gate2d()
    return st, cr, hb, fo, plates2d, camH


def make_mode_shaft(seq=False):
    off = 3.0 if seq else -3.0
    s = cyl_x(2.5, -2, 53, y=SHAFT_Y, z=SHAFT_Z)
    s += cyl_x(5.5, -2, 6, y=SHAFT_Y, z=SHAFT_Z + off)
    return s


def make_mode_knob(seq=False):
    """Mode selector outside the shell: knurled + pointer (H ↑ / SEQ ↓)."""
    k = cyl_x(9.5, 44, 52, y=SHAFT_Y, z=SHAFT_Z)
    for i in range(24):
        k -= Pos(0, SHAFT_Y, SHAFT_Z) * Rot(15 * i, 0, 0) * box(43, 53, -0.45, 0.45, 8.6, 10)
    k += box(50, 52, SHAFT_Y - 1, SHAFT_Y + 1, SHAFT_Z + 4.5, SHAFT_Z + 9.0)
    k -= cyl_x(2.5, 43, 53, y=SHAFT_Y, z=SHAFT_Z)
    if seq:
        k = k.rotate(Axis((0, SHAFT_Y, SHAFT_Z), (1, 0, 0)), 180)
    return k


_base2d_v05 = base2d


def base2d():
    p = _base2d_v05()
    return p.difference(SPoint(47, SHAFT_Y).buffer(12.5, 48))        # mode knob scallop (front)


def make_bottom_cover():
    a, b, c, d, r = BASE_OUT
    p = SPolygon([(a, c), (b, c), (b, d), (a, d)]).buffer(-r).buffer(r, resolution=32)
    p = p.difference(SPoint(47, SHAFT_Y).buffer(12.5, 48))
    return plate_xy(p, Z_BASE - BASE_T - 2, Z_BASE - BASE_T)
