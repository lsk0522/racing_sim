"""
LSK H/SEQ Shifter v0.3 (ultra-compact, aluminium plates + printed) — parametric CAD (build123d)
Origin = gimbal centre. X = forward, Y = left, Z = up. Units: mm
"""
import math, json
import numpy as np
from build123d import *
from shapely.geometry import Point as SPoint, Polygon as SPolygon
from shapely.ops import unary_union
import cam as camgen

Z_BASE = -66.0          # base top
BASE_T = 5.0            # Al 5t
H_GATE = 55.0           # gimbal centre → middle of the gate plate
GATE_T = 3.0            # Al 5052 3t (or SUS 2~3t)
LEVER_D = 8.0           # Ø8 hardened linear shaft
L_KNOB = camgen.L_KNOB  # 110
GATE_Y = {"1-2": 15.0, "3-4": 0.0, "5-6": -15.0, "7-R": -30.0}
PHI = {k: math.degrees(math.atan(v / H_GATE)) for k, v in GATE_Y.items()}
THETA_STOP = camgen.STOP_DEG      # 14
THETA_GATE_END = 14.5
BRG = dict(od=16.0, w=5.0)        # 625ZZ (towers: F625ZZ flanged)
Y_TRACK_H, Y_TRACK_S = -31.5, -48.5
TRACK_W = 10.0
MODE_TRAVEL = Y_TRACK_H - Y_TRACK_S   # 17
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

# ======================= Parts (v0.3 ultra-compact: aluminium plates + printed) =======================
# Al = aluminium plate (laser/CNC, DXF provided), P = printed (ABS/PETG)

def make_hub():                                                  # P (ABS/PETG, 100%)
    h = box(-10, 10, -7, 7, -14, 18)
    h += cyl_x(4.5, 10, 11) + cyl_x(4.5, -11, -10)
    h -= cyl_x(2.6, -12, 12)                                    # M5 roll axle
    h -= cyl_z(LEVER_D / 2 + 0.06, 3, 19)                       # Ø8 lever bore
    for z in (9, 14):
        h -= cyl_x(2.8, 0, 11, z=z)                             # M4 set screw inserts
    h -= cyl_z(2.6, -15, -4)                                    # M5 tail
    h -= box(-4.1, 4.1, -4.1, 8, -12.2, -7.8)                   # M5 nut slot
    return h


def make_lever():
    return cyl_z(LEVER_D / 2, 3, L_KNOB + 7)


def make_knob():                                                 # P (or aluminium knob)
    k = Pos(0, 0, L_KNOB) * Sphere(17)
    k &= box(-20, 20, -20, 20, L_KNOB - 11, L_KNOB + 20)
    k += cyl_z(12, L_KNOB - 22, L_KNOB - 10)
    for i in range(18):
        k -= Rot(0, 0, 20 * i + 10) * box(11.2, 13, -0.5, 0.5, L_KNOB - 20, L_KNOB - 14)
    k -= cyl_z(LEVER_D / 2 + 0.1, L_KNOB - 23, L_KNOB + 7)
    k -= cyl_x(1.7, 0, 14, z=L_KNOB - 16)
    return k


def make_tail():
    t = cyl_z(2.5, -49.5, -4.5) + hex_z(8, -53, -49.5)          # M5×45 (head down)
    cap = cyl_z(5.0, -54, -47.5) - hex_z(8.2, -53.2, -49.2) - cyl_z(2.7, -49.5, -46.5)
    return t, cap


def make_cradle():                                               # P (ABS/PETG, 6 walls 60%)
    c = box(-18, 18, -26.5, 26.5, -12, 12) - box(-11, 11, -20.5, 20.5, -13, 13)
    c -= cyl_x(BRG["od"] / 2 + 0.05, 18 - BRG["w"] - 0.2, 19)
    c -= cyl_x(BRG["od"] / 2 + 0.05, -19, -18 + BRG["w"] + 0.2)
    c -= cyl_x(4.5, -19, 19)
    c -= cyl_y(2.6, -27, 27)
    c -= hex_y(8.2, 20.5, 24.5) + hex_y(8.2, -24.5, -20.5)
    for x in (-11, 11):
        c -= cyl_y(2.0, -27, -21, x=x, z=-6)                    # M3 inserts (cam stack)
    for y0, y1 in ((-26.5, -20.5), (20.5, 26.5)):
        c += box(-10, 10, y0, y1, -21, -11)
    c += box(-10, 10, -26, 50, -39, -20)
    c -= box(-3.6, 3.6, -13.5, 24, -40, -19)
    c -= box(-5.5, 5.5, 12.3, 26.5, -38.2, -29.8)
    c -= cyl_y(4.3, -24.1, -11, z=-24)
    c -= cyl_y(4.3, 22, 30.1, z=-24)
    c -= cyl_y(4.3, 26, 44.5, z=-34)
    c -= cyl_y(3.2, 44, 50.5, z=-34)                            # M5 insert (S2 preload)
    return c


def make_plungers():
    p = {}
    p["plunger_S1_L"] = box(-3.4, 3.4, -6.5, -2.5, -27.8, -20.2) + cyl_y(2.4, -10.5, -6.5, z=-24)
    p["plunger_S1_R"] = box(-3.4, 3.4, 2.5, 6.5, -27.8, -20.2) + cyl_y(2.4, 6.5, 10.5, z=-24)
    p["plunger_S2"] = box(-5.3, 5.3, 12.3, 16.3, -37.8, -30.2) + cyl_y(2.4, 16.3, 20.3, z=-34)
    p["cup_S2"] = cyl_y(4.1, 42.2, 44.2, z=-34)
    return p


def cam_track_polys():
    return camgen.build_cam(camgen.H_PROFILE), camgen.build_cam(camgen.SEQ_PROFILE)


def cam_web2d():
    sector = SPolygon(ring_xz(30, 6, -90 - 38, -90 + 38)).buffer(0)
    return unary_union([sector, SPoint(0, 0).buffer(16, 64)])


def cam_plate2d(track):
    p = unary_union([track, cam_web2d()]).buffer(0)
    p = p.difference(SPoint(0, 0).buffer(2.6, 32))
    for x in (-11, 11):
        p = p.difference(SPoint(x, -6).buffer(1.7, 24))
    for x in (-8, 8):                                            # alignment holes for stacking (M3)
        p = p.difference(SPoint(x, -24).buffer(1.7, 24))
    return p


def plate_from2d_xz(p2d, y0, y1):
    s = xz_prism(shapely_to_pts(SPolygon(p2d.exterior)), y0, y1)
    for h in p2d.interiors:
        s -= xz_prism([(float(x), float(z)) for x, z in list(h.coords)[:-1]], y0 - 1, y1 + 1)
    return s


def make_cams(camH, camS):
    """Al 7075 5t × 2 per track (or one 10t). Spacer between them is printed."""
    d = {}
    pH, pS = cam_plate2d(camH), cam_plate2d(camS)
    d["cam_H"] = plate_from2d_xz(pH, Y_TRACK_H - 5, Y_TRACK_H + 5)
    d["cam_SEQ"] = plate_from2d_xz(pS, Y_TRACK_S - 5, Y_TRACK_S + 5)
    sp = cam_web2d().difference(SPoint(0, 0).buffer(2.6, 32))
    for x in (-11, 11):
        sp = sp.difference(SPoint(x, -6).buffer(1.7, 24))
    for x in (-8, 8):
        sp = sp.difference(SPoint(x, -24).buffer(1.7, 24))
    d["cam_spacer"] = plate_from2d_xz(sp, Y_TRACK_S + 5, Y_TRACK_H - 5)
    return d, {"cam_H": pH, "cam_SEQ": pS}


# ---------- Aluminium plates ----------
def tower2d(side):
    outer = SPolygon([(-86, Z_BASE), (38, Z_BASE), (38, H_GATE - GATE_T / 2), (-46, H_GATE - GATE_T / 2), (-86, -6)])
    if side < 0:
        win = SPolygon([(-76, Z_BASE - 1), (26, Z_BASE - 1), (26, -16), (-14, -16), (-14, -12), (-76, -12)])
    else:
        win = SPolygon([(-21, -60), (21, -60), (21, -16), (-21, -16)])
    p = outer.difference(win)
    p = p.difference(SPoint(0, 0).buffer(BRG["od"] / 2 + 0.05, 48))           # F625 pressed from outside
    for r in ([(-38, 12), (-14, 12), (-14, 46), (-34, 46)], [(14, 12), (30, 12), (30, 46), (14, 46)]):
        p = p.difference(SPolygon(r).buffer(3).buffer(-3))                       # decorative / lightening windows
    for x, z in ((-10, 4), (10, 4)):
        p = p.difference(SPoint(x, z + 6).buffer(1.6, 16))                       # M3 sensor/stop bracket
    for x in (-81, 32):
        p = p.difference(SPoint(x, Z_BASE + 9).buffer(2.2, 16))                  # M4 base bracket (clip)
    return p


def make_tower(side):
    y0, y1 = ((-61.0, -55.0) if side < 0 else (28.0, 34.0))
    t = plate_from2d_xz(tower2d(side), y0, y1)
    for x in (-36, 26):                                                          # M3 tapped holes in the top edge (gate)
        t -= cyl_z(1.25, H_GATE - GATE_T / 2 - 8, H_GATE, x=x, y=(y0 + y1) / 2)
    return t


def gate2d():
    opening = gate_opening()
    outer = SPolygon([(-32, -61), (32, -61), (32, 34), (-32, 34)])
    p = outer.difference(opening)
    for yc in (-58.0, 31.0):
        for x in (-36 + 0, 26):
            pass
    holes = [SPoint(x, y).buffer(1.7, 16) for y in (-58.0, 31.0) for x in (-36, 26)]
    outer2 = SPolygon([(-40, -61), (32, -61), (32, 34), (-40, 34)])
    p = outer2.difference(opening)
    for h in holes:
        p = p.difference(h)
    return p


def make_gate():
    p2 = gate2d()
    plate = xy_prism(shapely_to_pts(SPolygon(p2.exterior)), H_GATE - GATE_T / 2, H_GATE + GATE_T / 2)
    for hole in p2.interiors:
        plate -= xy_prism([(float(x), float(y)) for x, y in list(hole.coords)[:-1]], H_GATE - 5, H_GATE + 5)
    return plate, p2


def base2d():
    p = SPolygon([(-90, -66), (44, -66), (44, 40), (-90, 40)])
    p = p.difference(SPolygon([(-5.3, -37), (5.3, -37), (5.3, 22), (-5.3, 22)]))   # tongue slot
    holes = []
    for x in (-81, 32):
        holes += [(x, -63.5), (x, 37)]                                           # tower brackets (M4)
    holes += [(-78, -20), (-78, -50)]                                            # rear rail
    holes += [(-21, 0), (21, 0)]                                                  # fork guide
    holes += [(22, -47), (22, -33)]                                               # crank bracket
    for h in holes:
        p = p.difference(SPoint(*h).buffer(2.2, 16))
    for h in ((-60, 20), (-20, 20), (20, 20), (-60, -60 + 0), (36, -20)):         # M6 mount (40 mm grid)
        p = p.difference(SPoint(*h).buffer(3.3, 24))
    return p


def make_base():
    p2 = base2d()
    b = xy_prism(shapely_to_pts(SPolygon(p2.exterior)), Z_BASE - BASE_T, Z_BASE)
    for h in p2.interiors:
        b -= xy_prism([(float(x), float(y)) for x, y in list(h.coords)[:-1]], Z_BASE - BASE_T - 1, Z_BASE + 1)
    return b, p2


def make_tower_clips():
    """Printed L brackets: tower plate ↔ base (M4)."""
    out = {}
    for name, y0, y1 in (("clip_negY", -66, -61), ("clip_posY", 34, 40)):
        for x in (-81, 32):
            c = box(x - 6, x + 6, y0, y1, Z_BASE, Z_BASE + 14)
            c -= cyl_y(2.2, y0 - 1, y1 + 1, x=x, z=Z_BASE + 9)            # M4 → tower plate
            c -= cyl_z(2.8, Z_BASE - 1, Z_BASE + 6, x=x, y=(y0 + y1) / 2)  # M4 insert ← bolt from under the base
            out[f"{name}_{'r' if x < 0 else 'f'}"] = c
    return out


def make_stop_blocks():
    s = Part()
    for sgn in (-1, 1):
        x0, x1 = sorted((sgn * 21, sgn * 31))
        s += box(x0, x1, 34, 40, -36, -24) - cyl_x(2.8, x0 - 1, x1 + 1, y=37, z=-30)
    return s


# ---------- Follower / mode ----------
def make_carriage():                                             # P (ABS/PETG, 6 walls 60%)
    zb, yc = Z_BASE, Y_TRACK_H
    c = box(-73, 10, yc - 13, yc + 13, zb, zb + 5)
    for s in (-1, 1):
        a, b = sorted((yc + s * 9.6, yc + s * 12.6))
        c += box(-36, -24, a, b, zb + 5, -39)
    c -= cyl_y(2.6, yc - 14, yc + 14, x=-30, z=-45)
    c += box(-73, -67, yc - TR, yc + TR, zb + 5, -14.7)
    c += box(-73, -56, yc - TR, yc + TR, -18.7, -14.7)
    c -= cyl_z(4.0, -19.5, -14, x=-65, y=yc)                    # M6 insert (cam preload)
    prof = [(-18.5, Z_BASE - BASE_T), (21, Z_BASE - BASE_T), (21, -61.5), (16, -61.5), (4, -67.5), (-18.5, -67.5)]
    c += extrude(Plane.YZ * Polygon(*prof, align=None), amount=5, both=True)
    c += box(6, 10, yc - 7, yc + 7, zb, -40)                    # crank tab
    c -= box(5, 11, yc - 2.7, yc + 2.7, -62, -44)
    return c


TR = 6.5


def make_arm():
    yc = Y_TRACK_H
    a = Part()
    for s in (-1, 1):
        y0, y1 = sorted((yc + s * 5.6, yc + s * 9.1))
        a += box(-71, 0, y0, y1, -49, -41) + cyl_y(4.5, y0, y1, x=0, z=-45) + cyl_y(4.5, y0, y1, x=-30, z=-45)
    a += box(-71, -59, yc - 9.1, yc + 9.1, -45, -41)
    a -= cyl_z(6.2, -42.5, -40, x=-65, y=yc)
    a += box(-22, -14, yc - 9.1, yc + 9.1, -49, -46)
    a -= cyl_y(2.6, yc - 10, yc + 10, x=-30, z=-45)
    a -= cyl_y(2.6, yc - 10, yc + 10, x=0, z=-45)
    return a


def make_rollers():
    yc = Y_TRACK_H
    r = Part()
    for y0 in (yc - 5.2, yc + 0.2):
        r += cyl_y(8.0, y0, y0 + 5.0, z=-45) - cyl_y(2.5, y0 - 1, y0 + 6, z=-45)
    return r


def make_fork(seq=False):
    lift = 6.0 if seq else 0.0
    f = box(-5, 5, 0, 3, -67.5 + lift, -64.5 + lift)          # foot (rides on the ramp)
    f += box(-18.5, 18.5, -8.4, 8.4, -64.5 + lift, -62 + lift)  # web
    for s in (-1, 1):
        a, b = sorted((s * 5.4, s * 8.4))
        f += box(-18.5, 18.5, a, b, -62 + lift, -56 + lift)   # prongs (raised: top −50, 4 mm engaged on the tail cap)
    return f


def make_fork_guide():
    g = Part()
    for x in (-23, 19):
        g += box(x, x + 4, -10, 10, Z_BASE, -54)
    return g


SHAFT_Y, SHAFT_Z = (Y_TRACK_H + Y_TRACK_S) / 2, -53.0
CRANK_R = MODE_TRAVEL / 2


def make_mode_crank(seq=False):
    ang = math.pi if seq else 0.0
    py, pz = SHAFT_Y - CRANK_R * math.cos(ang) * -1, SHAFT_Z + CRANK_R * math.sin(ang)
    py = SHAFT_Y + CRANK_R * math.cos(ang)
    arm = cyl_x(4.5, 14, 17, y=SHAFT_Y, z=SHAFT_Z) + cyl_x(3.5, 14, 17, y=py, z=pz)
    arm += box(14, 17, min(SHAFT_Y, py), max(SHAFT_Y, py), SHAFT_Z - 2.5, SHAFT_Z + 2.5)
    arm += cyl_x(2.4, 5, 14, y=py, z=pz)
    arm -= cyl_x(2.6, 13, 18, y=SHAFT_Y, z=SHAFT_Z)
    return arm


def make_mode_knob():
    k = cyl_x(11, 30, 38, y=SHAFT_Y, z=SHAFT_Z)
    k += box(34, 38, SHAFT_Y, SHAFT_Y + 12, SHAFT_Z - 2.5, SHAFT_Z + 2.5)   # flip lever (+Y = H, −Y = SEQ)
    for i in range(16):
        k -= Pos(0, SHAFT_Y, SHAFT_Z) * Rot(22.5 * i, 0, 0) * box(29, 39, -0.5, 0.5, 10.2, 11.5)
    k -= cyl_x(2.6, 29, 39, y=SHAFT_Y, z=SHAFT_Z)
    return k


def make_crank_bracket():
    b = box(19, 25, SHAFT_Y - 9, SHAFT_Y + 9, Z_BASE, SHAFT_Z + 7)
    b -= cyl_x(2.6, 18, 26, y=SHAFT_Y, z=SHAFT_Z)
    return b


def make_rail():
    return box(-76, -73.5, -66, -16, Z_BASE, Z_BASE + 4)


def build_all():
    camH, camS = cam_track_polys()
    tail, tipcap = make_tail()
    base, base2 = make_base()
    gate, g2 = make_gate()
    st = {"base": base, "tower_negY": make_tower(-1), "tower_posY": make_tower(+1), "gate_plate": gate,
          "fork_guide": make_fork_guide(), "crank_bracket": make_crank_bracket(), "rail": make_rail(),
          "stop_blocks": make_stop_blocks()}
    st.update(make_tower_clips())
    cams, cam2d = make_cams(camH, camS)
    cr = {"cradle": make_cradle(), **cams}
    cr.update(make_plungers())
    hb = {"hub": make_hub(), "lever_bar": make_lever(), "knob": make_knob(), "tail_bolt": tail, "tail_cap": tipcap}
    fo = {"carriage": make_carriage(), "arm": make_arm(), "rollers": make_rollers()}
    plates2d = {"base": base2, "gate": g2, "tower_negY": tower2d(-1), "tower_posY": tower2d(1), **cam2d}
    return st, cr, hb, fo, plates2d, (camH, camS)


def pose(parts, theta=0.0, phi=0.0, level="cradle"):
    out = {}
    for k, v in parts.items():
        s = v
        if level == "hub":
            s = s.rotate(Axis.X, -phi)
        out[k] = s.rotate(Axis.Y, theta)
    return out
