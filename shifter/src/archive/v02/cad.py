"""
LSK H/SEQ Shifter v0.2 (compact) — parametric CAD (build123d)
Origin = gimbal centre. X = forward (1st/3rd/5th/7th), Y = left, Z = up. Units: mm
pitch θ: rotation about Y (cradle). roll φ: rotation about X (hub), φ>0 → lever top moves to +Y (left gate)
"""
import math, json
import numpy as np
from build123d import *
from shapely.geometry import Point as SPoint, Polygon as SPolygon
from shapely.ops import unary_union
import cam as camgen

# ======================= Key parameters =======================
Z_BASE = -84.0         # base top surface
BASE_T = 8.0
H_GATE = 75.0          # gimbal centre → middle of the gate plate
GATE_T = 3.0
LEVER_D = 10.0         # SUS304 Ø10
L_KNOB = camgen.L_KNOB # 140
GATE_Y = {"1-2": 15.5, "3-4": 0.0, "5-6": -15.5, "7-R": -31.0}
PHI = {k: math.degrees(math.atan(v / H_GATE)) for k, v in GATE_Y.items()}
THETA_STOP = 12.0
THETA_GATE_END = 12.5
BRG = dict(od=16.0, w=5.0)   # 625ZZ (5×16×5) — single bearing size for the whole build
Y_TRACK_H, Y_TRACK_S = -34.0, -52.0
TRACK_W = 11.0
CAM_Y0, CAM_Y1 = -57.5, -28.5
MODE_TRAVEL = Y_TRACK_H - Y_TRACK_S      # 18
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



# ======================= Parts (v0.2 compact) =======================
def make_hub():
    h = box(-11, 11, -8, 8, -16, 18)
    h += cyl_x(4.5, 11, 12) + cyl_x(4.5, -12, -11)               # spacers that only touch the 625 inner race
    h -= cyl_x(2.6, -13, 13)                                   # M5 roll axle
    h -= cyl_z(LEVER_D / 2 + 0.08, 5, 19)                      # Ø10 lever bore
    for z in (10, 15):                                         # M4 set screws (insert Ø5.6)
        h -= cyl_x(2.8, 0, 12, z=z)
    h -= cyl_z(3.15, -17, -5)                                  # M6 tail bolt hole
    h -= box(-5.1, 5.1, -5.1, 9, -12.2, -6.8)                  # M6 nut slot (inserted from +Y)
    return h


def make_lever():
    return cyl_z(LEVER_D / 2, 5, L_KNOB + 8)


def make_knob():
    k = Pos(0, 0, L_KNOB) * Sphere(20)
    k &= box(-25, 25, -25, 25, L_KNOB - 13, L_KNOB + 25)
    k += cyl_z(14, L_KNOB - 25, L_KNOB - 12)
    for i in range(20):
        a = 360 / 20 * i + 9
        k -= Rot(0, 0, a) * box(13.0, 15, -0.5, 0.5, L_KNOB - 23, L_KNOB - 16)
    k -= cyl_z(LEVER_D / 2 + 0.1, L_KNOB - 26, L_KNOB + 8)
    k -= cyl_x(1.9, 0, 16, z=L_KNOB - 19)                     # M4 set screw
    return k


def make_tail():
    t = cyl_z(3.0, -60, -6) + hex_z(10, -64, -60)
    cap = cyl_z(5.5, -66, -58) - hex_z(10.2, -64.2, -59.8) - cyl_z(3.2, -60.5, -57)
    return t, cap


def make_cradle():
    c = box(-19, 19, -26.5, 26.5, -13, 13) - box(-12, 12, -20.5, 20.5, -14, 14)
    c -= cyl_x(BRG["od"] / 2 + 0.05, 19 - BRG["w"] - 0.2, 20)
    c -= cyl_x(BRG["od"] / 2 + 0.05, -20, -19 + BRG["w"] + 0.2)
    c -= cyl_x(4.5, -20, 20)
    c -= cyl_y(2.6, -27, 27)                                   # M5 pitch axle
    c -= hex_y(8.2, 20.5, 24.5) + hex_y(8.2, -24.5, -20.5)     # M5 nuts
    for x in (-14, 14):                                        # M3 inserts for cam mounting
        c -= cyl_y(2.0, -27, -21, x=x, z=-7)
    for y0, y1 in ((-26.5, -20.5), (20.5, 26.5)):
        c += box(-10, 10, y0, y1, -27, -12)
    c += box(-10, 10, -27, 52, -48, -26)                       # lateral spring bar
    c -= box(-4.0, 4.0, -14, 24, -49, -25)                     # channel (tail + plungers)
    c -= box(-6.0, 6.0, 12.2, 26, -47.2, -36.8)                # S2 pocket (front shoulder)
    c -= cyl_y(4.3, -24.6, -12, z=-30)                         # S1 left (short spring L0 24)
    c -= cyl_y(4.3, 20, 30.6, z=-30)                           # S1 right (L0 30)
    c -= cyl_y(4.3, 25, 44, z=-42)                             # S2
    c -= cyl_y(4.0, 43, 52.5, z=-42)                           # M6 insert (S2 preload screw)
    return c


def make_plungers():
    p = {}
    p["plunger_S1_L"] = box(-3.8, 3.8, -7, -3, -33.8, -26.2) + cyl_y(2.5, -11, -7, z=-30)
    p["plunger_S1_R"] = box(-3.8, 3.8, 3, 7, -33.8, -26.2) + cyl_y(2.5, 7, 11, z=-30)
    p["plunger_S2"] = box(-5.8, 5.8, 12.2, 16.2, -46.8, -37.2) + cyl_y(2.5, 16.2, 20.2, z=-42)
    p["cup_S2"] = cyl_y(4.1, 41.6, 43.6, z=-42)
    return p


def cam_track_polys():
    return camgen.build_cam(camgen.H_PROFILE), camgen.build_cam(camgen.SEQ_PROFILE)


def make_cam(camH, camS):
    web_sector = SPolygon(ring_xz(44, 9, -90 - 32, -90 + 32)).buffer(0)
    web = unary_union([web_sector, SPoint(0, 0).buffer(10, 48)])
    c = xz_prism(shapely_to_pts(web), CAM_Y0, CAM_Y1)
    c += box(-19, 19, CAM_Y1, -26.5, -12, 12)                  # flange (−Y face of the cradle)
    yH0, yH1 = Y_TRACK_H - TRACK_W / 2, Y_TRACK_H + TRACK_W / 2
    yS0, yS1 = Y_TRACK_S - TRACK_W / 2, Y_TRACK_S + TRACK_W / 2
    c += xz_prism(shapely_to_pts(unary_union([camH, web_sector]).buffer(0)), yH0, yH1)
    c += xz_prism(shapely_to_pts(unary_union([camS, web_sector]).buffer(0)), yS0, yS1)
    c -= cyl_y(2.6, CAM_Y0 - 1, -26)
    for x in (-14, 14):
        c -= cyl_y(1.7, CAM_Y0 - 1, -26, x=x, z=-7)
    return c


def tower_body(y0, y1):
    pts = [(-96, Z_BASE), (40, Z_BASE), (40, 13), (-40, 13), (-96, -30)]
    return xz_prism(pts, y0, y1)


def make_tower(side):
    if side < 0:
        y0, y1 = -69.0, -59.0
        t = tower_body(y0, y1)
        t -= box(-86, 28, y0 - 1, y1 + 1, Z_BASE - 1, -24)          # follower/carriage passage
        t -= cyl_y(BRG["od"] / 2 + 0.05, y0 - 1, y0 + BRG["w"] + 0.2)
        t -= cyl_y(4.5, y0 - 1, y1 + 1)
        for x in (-14, 14):                                         # AS5600 bracket M3
            t -= cyl_y(2.0, y0 - 1, y0 + 6, x=x, z=-16)
        foot = box(-96, 40, y0 - 6, y1, Z_BASE, Z_BASE + 6) - box(-86, 28, y0 - 7, y1 + 1, Z_BASE - 1, Z_BASE + 7)
        fy = y0 - 3
    else:
        y0, y1 = 28.0, 38.0
        t = tower_body(y0, y1)
        t -= box(-23, 23, y0 - 1, y1 + 1, -72, -20)                 # cradle bar + tail tip passage
        t -= cyl_y(BRG["od"] / 2 + 0.05, y1 - BRG["w"] - 0.2, y1 + 1)
        t -= cyl_y(4.5, y0 - 1, y1 + 1)
        t -= cyl_x(2.8, 23, 41, y=33, z=-37) + cyl_x(2.8, -60, -23, y=33, z=-37)   # M4 stop screws
        foot = box(-96, 40, y0, y1 + 6, Z_BASE, Z_BASE + 6)
        fy = y1 + 3
    for x in (-34, 34):
        t -= cyl_z(2.7, 0, 14, x=x, y=(y0 + y1) / 2)
    t += foot
    for x in (-91, 34):
        t -= cyl_z(2.2, Z_BASE - 1, Z_BASE + 7, x=x, y=fy)
    return t


def make_posts():
    out = []
    for yc in (-64.0, 33.0):
        for x in (-34, 34):
            out.append(cyl_z(6, 13, H_GATE - GATE_T / 2, x=x, y=yc) - cyl_z(2.7, 12, H_GATE, x=x, y=yc))
    return out


def make_gate():
    opening = gate_opening()
    outer = SPolygon([(-42, -72), (42, -72), (42, 40), (-42, 40)])
    plate2d = outer.difference(opening)
    plate = xy_prism(shapely_to_pts(SPolygon(plate2d.exterior)), H_GATE - GATE_T / 2, H_GATE + GATE_T / 2)
    for hole in plate2d.interiors:
        plate -= xy_prism([(float(x), float(y)) for x, y in list(hole.coords)[:-1]], H_GATE - 5, H_GATE + 5)
    for yc in (-64.0, 33.0):
        for x in (-34, 34):
            plate -= cyl_z(2.75, H_GATE - 5, H_GATE + 5, x=x, y=yc)
    return plate, plate2d


TR = 6.5   # half width of the spring bracket


def make_carriage():
    zb, yc = Z_BASE, Y_TRACK_H
    c = box(-84, 12, yc - 13, yc + 13, zb, zb + 6)
    for s in (-1, 1):                                           # arm pivot clevis
        a, b = sorted((yc + s * 9.6, yc + s * 12.6))
        c += box(-41, -29, a, b, zb + 6, -56)
    c -= cyl_y(2.6, yc - 14, yc + 14, x=-35, z=-62)
    # Rear spring bracket (preload screw accessible from the top)
    c += box(-84, -78, yc - TR, yc + TR, zb + 6, -26)
    c += box(-84, -58, yc - TR, yc + TR, -30, -26)
    c -= cyl_z(4.0, -31, -25, x=-65, y=yc)                       # M6 insert
    # Tongue (ramp → fork), riding in the base groove
    prof = [(-21, zb - 2), (21, zb - 2), (21, -73), (18, -73), (4, -81), (-21, -81)]
    c += extrude(Plane.YZ * Polygon(*prof, align=None), amount=5, both=True)
    # Front tab (mode crank pin slot)
    c += box(11, 15, yc - 7, yc + 7, zb, -56)
    c -= box(10, 16, yc - 2.7, yc + 2.7, -80, -60)
    return c


def make_arm():
    yc = Y_TRACK_H
    a = Part()
    for s in (-1, 1):
        y0, y1 = sorted((yc + s * 5.6, yc + s * 9.1))
        a += box(-71, 0, y0, y1, -66, -58) + cyl_y(5, y0, y1, x=0, z=-62) + cyl_y(5, y0, y1, x=-35, z=-62)
    a += box(-71, -59, yc - 9.1, yc + 9.1, -62, -58)            # rear spring pad
    a -= cyl_z(6.2, -59.5, -57, x=-65, y=yc)                    # spring seat
    a += box(-26, -18, yc - 9.1, yc + 9.1, -66, -63)            # stiffening web
    a -= cyl_y(2.6, yc - 10, yc + 10, x=-35, z=-62)
    a -= cyl_y(2.6, yc - 10, yc + 10, x=0, z=-62)
    return a


def make_rollers():
    yc = Y_TRACK_H
    r = Part()
    for y0 in (yc - 5.2, yc + 0.2):
        r += cyl_y(8.0, y0, y0 + 5.0, z=-62) - cyl_y(2.5, y0 - 1, y0 + 6, z=-62)
    return r


def make_fork(seq=False):
    lift = 8.0 if seq else 0.0
    f = box(-5, 5, 0, 3, -81 + lift, -78 + lift)              # foot (rides on the ramp)
    f += box(-21, 21, -9.0, 9.0, -78 + lift, -75 + lift)      # web
    for s in (-1, 1):
        a, b = sorted((s * 5.9, s * 9.0))
        f += box(-20, 20, a, b, -75 + lift, -68 + lift)       # prongs (raised: top −60, 6 mm engaged on the tail cap)
    return f


def make_fork_guide():
    g = Part()
    for x in (-27, 23):
        g += box(x, x + 4, -12, 12, Z_BASE, -66)
    return g


SHAFT_Y, SHAFT_Z = Y_TRACK_H - MODE_TRAVEL / 2, -70.0       # mode crank shaft (along X)


def make_mode_crank(seq=False):
    """Crank arm + pin (M5 shaft). H: pin at +Y (0°), SEQ: 180°. At either end it sits on dead centre → self-locking."""
    ang = math.pi if seq else 0.0
    py, pz = SHAFT_Y + 9 * math.cos(ang), SHAFT_Z + 9 * math.sin(ang)
    arm = cyl_x(5, 20, 23, y=SHAFT_Y, z=SHAFT_Z) + cyl_x(4, 20, 23, y=py, z=pz)
    ym, zm = (SHAFT_Y + py) / 2, (SHAFT_Z + pz) / 2
    arm += box(20, 23, min(SHAFT_Y, py), max(SHAFT_Y, py), zm - 3, zm + 3)
    arm += cyl_x(2.4, 11, 20, y=py, z=pz)                       # pin
    arm -= cyl_x(2.6, 19, 24, y=SHAFT_Y, z=SHAFT_Z)
    return arm


def make_mode_knob():
    k = cyl_x(12.5, 42, 50, y=SHAFT_Y, z=SHAFT_Z)
    k += box(46, 50, SHAFT_Y, SHAFT_Y + 20, SHAFT_Z - 3, SHAFT_Z + 3)   # flip lever (+Y = H, −Y = SEQ)
    for i in range(18):
        a = 20 * i
        k -= Pos(0, SHAFT_Y, SHAFT_Z) * Rot(a, 0, 0) * box(41, 51, -0.5, 0.5, 11.7, 13)
    k -= cyl_x(2.6, 41, 51, y=SHAFT_Y, z=SHAFT_Z)
    return k


def make_crank_bracket():
    b = box(25, 31, SHAFT_Y - 9, SHAFT_Y + 9, Z_BASE, SHAFT_Z + 8)
    b -= cyl_x(2.6, 24, 32, y=SHAFT_Y, z=SHAFT_Z)
    return b


def make_base():
    b = box(-98, 52, -80, 46, Z_BASE - BASE_T, Z_BASE)
    b += box(-88, -85, -76, -18, Z_BASE, Z_BASE + 5)            # rear rail for the carriage
    b -= box(-5.3, 5.3, -42, 24, Z_BASE - 2, Z_BASE + 1)        # tongue groove
    for x in (-91, 34):
        for y in (-72, 41):
            b -= cyl_z(2.8, Z_BASE - 6, Z_BASE + 1, x=x, y=y)    # M4 inserts for the towers
    for x, y in ((-90, -72), (44, -72), (-90, 38), (44, 38), (-50, 38), (0, 38), (44, 0)):
        b -= cyl_z(3.3, Z_BASE - BASE_T - 1, Z_BASE + 1, x=x, y=y)   # M6 mounting (40 mm grid included)
    b -= box(18, 46, 6, 30, Z_BASE - BASE_T - 1, Z_BASE - 3)     # Pro Micro bay (bottom)
    return b


def build_all():
    camH, camS = cam_track_polys()
    tail, tipcap = make_tail()
    st = {"base": make_base(), "tower_negY": make_tower(-1), "tower_posY": make_tower(+1),
          "fork_guide": make_fork_guide(), "crank_bracket": make_crank_bracket()}
    for i, p in enumerate(make_posts()):
        st[f"gate_post_{i+1}"] = p
    gate, gate2d = make_gate()
    st["gate_plate"] = gate
    cr = {"cradle": make_cradle(), "cam_HS": make_cam(camH, camS)}
    cr.update(make_plungers())
    hb = {"hub": make_hub(), "lever_bar": make_lever(), "knob": make_knob(), "tail_bolt": tail, "tail_cap": tipcap}
    fo = {"carriage": make_carriage(), "arm": make_arm(), "rollers": make_rollers()}
    return st, cr, hb, fo, gate2d, (camH, camS)


def pose(parts, theta=0.0, phi=0.0, level="cradle"):
    out = {}
    for k, v in parts.items():
        s = v
        if level == "hub":
            s = s.rotate(Axis.X, -phi)
        out[k] = s.rotate(Axis.Y, theta)
    return out
