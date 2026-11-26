"""
LSK H/SEQ Shifter v0.1 — parametric CAD (build123d)
Origin = gimbal centre (pitch axis ∩ roll axis). X = forward (1st/3rd/5th/7th direction), Y = left, Z = up.
Units: mm.  pitch θ: rotation about Y (cradle).  roll φ: rotation about X (hub). φ>0 → lever top moves to +Y (left gates)
"""
import math, json
import numpy as np
from build123d import *
from shapely.geometry import Point as SPoint, Polygon as SPolygon, LineString
from shapely.ops import unary_union
import cam as camgen

# ======================= Key parameters =======================
Z_BASE = -125.0        # base top surface
BASE_T = 10.0
H_GATE = 118.0         # gimbal centre → middle of gate plate thickness
GATE_T = 3.0           # gate plate thickness (SUS 3t laser cut, or 5 mm printed)
LEVER_D = 12.0         # SUS304 Ø12 round bar
L_KNOB = camgen.L_KNOB # 190
GATE_Y = {"1-2": 17.0, "3-4": 0.0, "5-6": -17.0, "7-R": -34.0}   # lateral position at gate height
PHI = {k: math.degrees(math.atan(v / H_GATE)) for k, v in GATE_Y.items()}
THETA_STOP = 12.0      # pitch stop (gear position)
THETA_GATE_END = 12.5  # gate slot end (slightly past the stop → the stop screw takes the load)
R_S1, R_S2, R_TIP = 45.0, 62.0, 88.0   # radius of the lateral spring contact points / tail tip

# Cam / follower (same values as cam.py)
RR = camgen.RR
ARM_P = (camgen.P[0], camgen.P[1])       # (-60, -95)
Y_TRACK_H = -45.0                        # H track centre (Y)
Y_TRACK_S = -63.0                        # SEQ track centre (Y)
TRACK_W = 11.0
CAM_Y0, CAM_Y1 = -68.5, -39.5            # cam part Y range
MODE_TRAVEL = Y_TRACK_H - Y_TRACK_S      # 18

BRG608 = dict(od=22.0, w=7.0)
CLR = 0.2


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


# ======================= Parts =======================
def make_hub():
    h = box(-14, 14, -11, 11, -22, 24)
    h += cyl_x(6.5, 14, 15) + cyl_x(6.5, -15, -14)              # bearing inner-race spacer bosses
    h -= cyl_x(4.15, -16, 16)                                   # M8 roll axle
    h -= cyl_z(LEVER_D / 2 + 0.08, 7, 25)                       # lever bore
    for z in (12, 19):                                          # M5 set screws (heat-set inserts Ø6.4)
        h -= cyl_x(3.2, 0, 15, z=z)
    h -= cyl_z(4.15, -23, -6)                                   # tail bolt hole
    h -= box(-6.65, 6.65, -6.65, 12, -14.5, -7.5)               # M8 nut slot (inserted from the +Y side)
    return h


def make_lever():
    return cyl_z(LEVER_D / 2, 7, L_KNOB + 12)


def make_knob():
    """Round knob with machined-style grooves (Bugatti/Ferrari feel). Ø48, grips 25 mm of the lever bar."""
    k = Pos(0, 0, L_KNOB) * Sphere(24)
    k &= box(-30, 30, -30, 30, L_KNOB - 16, L_KNOB + 30)
    k += cyl_z(17, L_KNOB - 30, L_KNOB - 15)
    for i in range(24):                                          # ribs on the neck
        a = 360 / 24 * i + 7.5
        k -= Rot(0, 0, a) * box(16.0, 18, -0.6, 0.6, L_KNOB - 28, L_KNOB - 19)
    k -= cyl_z(LEVER_D / 2 + 0.1, L_KNOB - 31, L_KNOB - 31 + 37)
    k -= cyl_x(2.4, 0, 20, z=L_KNOB - 22)                        # M5 set screw (M5 insert or self-tap)
    return k


def make_tail():
    """M8×80 hex bolt (head down) + tip cap Ø14. Modelled as part of the hub group."""
    t = cyl_z(4.0, -86, -7)
    t += hex_z(13, -91.3, -86)
    cap = cyl_z(7.0, -93, -84) - hex_z(13.2, -91.5, -85.8) - cyl_z(4.2, -86, -83)
    return t, cap


def make_cradle():
    c = box(-24, 24, -36.5, 36.5, -18, 18) - box(-15, 15, -27.5, 27.5, -19, 19)
    # Roll bearing pockets (X walls)
    for s in (1, -1):
        c -= cyl_x(BRG608["od"] / 2 + 0.05, s * 24 - s * 7.2 if s > 0 else -24, 24 if s > 0 else -24 + 7.2)
        c -= cyl_x(6.0, -25, 25)
    # Pitch axle: M8 hole + inside nut pocket
    c -= cyl_y(4.15, -37, 37)
    c -= hex_y(13.2, 27.0, 34.0)
    c -= hex_y(13.2, -34.0, -27.0)
    # Cam mounting M4 inserts (−Y wall)
    for x in (-17, 17):
        c -= cyl_y(2.8, -37, -30, x=x, z=-10)
    # Legs + lateral spring bar
    for y0, y1 in ((-36.5, -27.5), (27.5, 36.5)):
        c += box(-12, 12, y0, y1, -39, -17)
    c += box(-12, 12, -39, 62, -71, -38)
    # Channel (tail + plungers): through top and bottom
    c -= box(-5.2, 5.2, -16, 30, -72, -37)
    # S2 pocket (wider, Y ≥ 13.3 only → front shoulder)
    c -= box(-7.2, 7.2, 13.3, 30, -67.2, -56.8)
    # Spring bores
    c -= cyl_y(5.3, -36, -15, z=-45)          # S1 left
    c -= cyl_y(5.3, 24, 36, z=-45)            # S1 right
    c -= cyl_y(5.3, 29, 52, z=-62)            # S2
    c -= cyl_y(5.0, 51, 63, z=-62)            # M8 insert (OD ~10) — S2 preload screw
    return c


def make_plungers():
    """Lateral plunger blocks (PETG). Each block stops at the channel end / shoulder."""
    p = {}
    # S1 left: front face Y=−4 (touches the tail), 4 thick + stem Ø6×6
    p["plunger_S1_L"] = box(-5.0, 5.0, -8, -4, -50, -40) + cyl_y(3.0, -14, -8, z=-45)
    p["plunger_S1_R"] = box(-5.0, 5.0, 4, 8, -50, -40) + cyl_y(3.0, 8, 14, z=-45)
    p["plunger_S2"] = box(-7.0, 7.0, 13.3, 17.3, -67, -57) + cyl_y(3.0, 17.3, 23.3, z=-62)
    p["cup_S2"] = cyl_y(5.1, 42.3, 45.3, z=-62)
    return p


def cam_track_polys():
    """Cam profile polygons (XZ) — the H and SEQ tracks."""
    camH = camgen.build_cam(camgen.H_PROFILE, inner_r=45.0)
    camS = camgen.build_cam(camgen.SEQ_PROFILE, inner_r=45.0)
    return camH, camS


def make_cam(camH, camS):
    # Web: sector r 10..60 + boss
    web_sector = SPolygon(ring_xz(60, 12, -90 - 30, -90 + 30)).buffer(0)
    boss = SPoint(0, 0).buffer(16, 48)
    web = unary_union([web_sector, boss])
    flange = box(-22, 22, -39.5, -36.5, -16, 16) - cyl_y(4.15, -40, -36) - cyl_y(2.2, -40, -36, x=-17, z=-10) \
        - cyl_y(2.2, -40, -36, x=17, z=-10)
    c = xz_prism(shapely_to_pts(web), CAM_Y0, CAM_Y1) + flange
    yH0, yH1 = Y_TRACK_H - TRACK_W / 2, Y_TRACK_H + TRACK_W / 2
    yS0, yS1 = Y_TRACK_S - TRACK_W / 2, Y_TRACK_S + TRACK_W / 2
    c += xz_prism(shapely_to_pts(unary_union([camH, web_sector]).buffer(0)), yH0, yH1)
    c += xz_prism(shapely_to_pts(unary_union([camS, web_sector]).buffer(0)), yS0, yS1)
    c -= cyl_y(4.15, CAM_Y0 - 1, -36)
    # M4 counterbores (bolt heads from the outside)
    for x in (-17, 17):
        c -= cyl_y(3.8, CAM_Y0 - 1, -40.5, x=x, z=-10)
    return c


def tower_outline(window, top_w=52, foot_x=(-100, 62)):
    pts = [(foot_x[0], Z_BASE), (foot_x[1], Z_BASE), (top_w, 14), (top_w, 22), (-top_w, 22), (-top_w, 14)]
    return pts


def make_tower(side):
    """side = −1 (−Y, pitch sensor + mode side) / +1 (+Y, stop screws)"""
    if side < 0:
        y0, y1 = -82.0, -70.0
        t = xz_prism(tower_outline(None), y0, y1)
        t -= box(-80, 42, y0 - 1, y1 + 1, Z_BASE - 1, -84)              # carriage / arm passage window
        t -= cyl_y(BRG608["od"] / 2 + 0.05, y0 - 1, y0 + BRG608["w"] + 0.2)
        t -= cyl_y(6.0, y0 - 1, y1 + 1)
        # lightening (exposed-mechanism look)
        t -= xz_prism([(-60, -72), (30, -72), (14, -26), (-14, -26)], y0 - 1, y1 + 1)
        # AS5600 bracket holes (M3 inserts)
        for x, z in ((-16, 14), (16, 14)):
            t -= cyl_y(2.0, y0 - 1, y0 + 6, x=x, z=z)
    else:
        y0, y1 = 38.0, 50.0
        t = xz_prism(tower_outline(None), y0, y1)
        t -= box(-28, 28, y0 - 1, y1 + 1, -76, -32)                     # cradle bar passage
        t -= cyl_y(BRG608["od"] / 2 + 0.05, y1 - BRG608["w"] - 0.2, y1 + 1)
        t -= cyl_y(6.0, y0 - 1, y1 + 1)
        t -= xz_prism([(-70, -86), (40, -86), (24, -80), (-34, -80)], y0 - 1, y1 + 1)
        # pitch stop screws M5 (inserts Ø6.4) — along X, Z=−55
        for s in (1, -1):
            t -= cyl_x(3.2, s * 28, s * 60, y=44, z=-55) if s > 0 else cyl_x(3.2, -60, -28, y=44, z=-55)
    # Gate post holes (M5 rod)
    for x in (-42, 42):
        t -= cyl_z(2.7, 0, 23, x=x, y=(y0 + y1) / 2)
    # Base fixing feet (M5)
    foot = box(-100, 62, y0 - 8 if side < 0 else y0, y1 if side < 0 else y1 + 8, Z_BASE, Z_BASE + 8)
    if side < 0:
        foot -= box(-80, 42, y0 - 9, y1 + 1, Z_BASE - 1, Z_BASE + 9)
    else:
        foot -= box(-28, 28, y0 - 1, y1 + 9, Z_BASE - 1, Z_BASE + 9)
    t += foot
    fy = y0 - 4 if side < 0 else y1 + 4
    for x in (-92, 54):
        t -= cyl_z(2.75, Z_BASE - 1, Z_BASE + 9, x=x, y=fy)
    return t


def make_posts():
    posts = []
    for yc in (-76.0, 44.0):
        for x in (-42, 42):
            p = cyl_z(7, 22, H_GATE - GATE_T / 2, x=x, y=yc) - cyl_z(2.7, 21, H_GATE, x=x, y=yc)
            posts.append(p)
    return posts


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


def make_gate():
    opening = gate_opening()
    outer = SPolygon([(-52, -86), (52, -86), (52, 54), (-52, 54)])
    plate2d = outer.difference(opening)
    plate = xy_prism(shapely_to_pts(SPolygon(plate2d.exterior)), H_GATE - GATE_T / 2, H_GATE + GATE_T / 2)
    for hole in plate2d.interiors:
        plate -= xy_prism([(float(x), float(y)) for x, y in list(hole.coords)[:-1]], H_GATE - 5, H_GATE + 5)
    for yc in (-76.0, 44.0):
        for x in (-42, 42):
            plate -= cyl_z(2.75, H_GATE - 5, H_GATE + 5, x=x, y=yc)
    return plate, plate2d


# ---------- Follower carriage / arm / fork / mode lever ----------
def make_carriage(y_c):
    """y_c = track centre (H: −45, SEQ: −63). The part is modelled at the H position and moved for SEQ."""
    zb = Z_BASE
    c = box(-72, 32, -58, -32, zb, zb + 12)
    for s in (-1, 1):                                                       # arm pivot clevis posts
        c += box(-66, -54, Y_TRACK_H + s * 9.7 - 2, Y_TRACK_H + s * 9.7 + 2, zb + 12, -89) if False else \
             box(-66, -54, min(Y_TRACK_H + s * 9.8, Y_TRACK_H + s * 13.8), max(Y_TRACK_H + s * 9.8, Y_TRACK_H + s * 13.8), zb + 12, -89)
    c -= cyl_y(2.6, -60, -30, x=-60, z=-95)
    # Spring cup tube
    c += cyl_z(9.5, zb + 12, -106, x=20, y=Y_TRACK_H)
    c -= cyl_z(6.3, zb + 3, -105, x=20, y=Y_TRACK_H)
    # Tongue (ramp → fork)
    lo, hi = zb + 4, zb + 14
    yl = lambda g: g  # global Y at H position
    prof = [(-32, zb), (22, zb), (22, hi), (18, hi), (4, lo), (-32, lo)]
    tongue = Pos(0, 0, 0) * extrude(Plane.YZ * Polygon(*prof, align=None), amount=6, both=True)
    c += tongue
    # Mode pushrod
    c += box(20, 30, -100, -58, zb, zb + 8)
    c -= cyl_z(2.6, zb - 1, zb + 9, x=25, y=-96)
    return c


def make_arm():
    yc = Y_TRACK_H
    a = Part()
    for s in (-1, 1):
        y0, y1 = sorted((yc + s * 5.6, yc + s * 9.6))
        plate = box(-60, 0, y0, y1, -101, -89) + cyl_y(6, y0, y1, x=-60, z=-95) + cyl_y(6, y0, y1, x=0, z=-95)
        plate += box(0, 26, y0, y1, -105, -95)
        a += plate
    a += box(12, 26, yc - 9.6, yc + 9.6, -105, -99)          # spring pad
    a += box(-50, -40, yc - 9.6, yc + 9.6, -101, -97)        # stiffening web
    a -= cyl_y(2.6, yc - 11, yc + 11, x=-60, z=-95)
    a -= cyl_y(2.6, yc - 11, yc + 11, x=0, z=-95)
    a -= cyl_z(4.0, -106, -98, x=20, y=yc)                    # M6 insert (preload screw)
    return a


def make_rollers():
    yc = Y_TRACK_H
    r = Part()
    for y0 in (yc - 5.2, yc + 0.2):
        r += cyl_y(8.0, y0, y0 + 5.0, z=-95) - cyl_y(2.5, y0 - 1, y0 + 6, z=-95)
    return r


def make_fork(seq=False):
    lift = 10.0 if seq else 0.0
    zfoot = Z_BASE + 4 + lift
    f = box(-6, 6, 0, 4, zfoot, zfoot + 14)                   # foot (sits on the ramp)
    f += box(-30, 30, -11.2, 11.2, zfoot + 14, zfoot + 18)    # web
    for s in (-1, 1):
        y0, y1 = sorted((s * 7.2, s * 11.2))
        f += box(-28, 28, y0, y1, zfoot + 18, zfoot + 24)     # prongs
    return f


def make_fork_guide():
    g = Part()
    for x in (-36, 32):
        g += box(x, x + 4, -14, 14, Z_BASE, -100)
    return g


def make_mode_lever():
    zb = Z_BASE
    piv = (45.0, -96.0)
    l = cyl_z(6, zb + 11, zb + 17, x=piv[0], y=piv[1])
    l += box(15, 110, -100, -92, zb + 11, zb + 17)
    l -= cyl_z(2.6, zb, zb + 20, x=piv[0], y=piv[1])
    l -= box(19, 31, -98.6, -93.4, zb + 10, zb + 18)         # pin slot
    l += cyl_z(7, zb + 17, zb + 37, x=104, y=-96)            # handle
    return l


def make_base():
    b = box(-110, 110, -108, 75, Z_BASE - BASE_T, Z_BASE)
    # Carriage guide rails
    b += box(-77, -73, -100, -28, Z_BASE, Z_BASE + 10) + box(33, 37, -100, -28, Z_BASE, Z_BASE + 10)
    # Tower fixing holes (M5 insert Ø6.4)
    for x in (-92, 54):
        for y in (-86, 54):
            b -= cyl_z(3.2, Z_BASE - 8, Z_BASE + 1, x=x, y=y)
    # Mounting holes (M6, 40 mm grid → 4040/8040 profiles, or desk clamp)
    for x in (-100, -60, 60, 100):
        for y in (-100, -60, 20, 60):
            if -80 < x < 40 and -100 < y < -28:
                continue
            b -= cyl_z(3.3, Z_BASE - BASE_T - 1, Z_BASE + 1, x=x, y=y)
    # Mode lever pivot + microswitch pocket
    b -= cyl_z(3.2, Z_BASE - 8, Z_BASE + 1, x=45, y=-96)
    b -= box(-20, 0, -106, -100, Z_BASE - 6, Z_BASE + 1)
    # Electronics bay (Pro Micro) — under the front of the base
    b -= box(60, 100, -20, 20, Z_BASE - BASE_T - 1, Z_BASE - 3)
    return b


def make_sensor_parts():
    """AS5600 (23×23 board) + magnet caps — for checking interfaces."""
    s = {}
    s["magcap_pitch"] = cyl_y(9, -91, -84.5) - hex_y(13.2, -91.5, -85) - cyl_y(3.05, -92, -89.5)
    s["as5600_pitch"] = box(-11.5, 11.5, -94.5, -93, -11.5, 11.5)
    s["magcap_roll"] = cyl_x(9, 30.5, 37) - hex_y(13.2, 0, 1)  # simplified
    s["as5600_roll"] = box(39, 40.5, -11.5, 11.5, -11.5, 11.5)
    return s


# ======================= Assembly / checks =======================
def build_all():
    camH, camS = cam_track_polys()
    tail, tipcap = make_tail()
    parts_static = {
        "base": make_base(),
        "tower_negY": make_tower(-1),
        "tower_posY": make_tower(+1),
        "fork_guide": make_fork_guide(),
    }
    for i, p in enumerate(make_posts()):
        parts_static[f"gate_post_{i+1}"] = p
    gate, gate2d = make_gate()
    parts_static["gate_plate"] = gate
    cradle_grp = {"cradle": make_cradle(), "cam_HS": make_cam(camH, camS)}
    cradle_grp.update(make_plungers())
    hub_grp = {"hub": make_hub(), "lever_bar": make_lever(), "knob": make_knob(),
               "tail_bolt": tail, "tail_cap": tipcap}
    follower = {"carriage": make_carriage(Y_TRACK_H), "arm": make_arm(), "rollers": make_rollers(),
                "mode_lever": make_mode_lever()}
    return parts_static, cradle_grp, hub_grp, follower, gate2d, (camH, camS)


def pose(parts, theta=0.0, phi=0.0, level="cradle"):
    out = {}
    for k, v in parts.items():
        s = v
        if level == "hub":
            s = s.rotate(Axis.X, -phi)
        s = s.rotate(Axis.Y, theta)
        out[k] = s
    return out


if __name__ == "__main__":
    import os, sys, time
    os.makedirs("out/stl", exist_ok=True)
    os.makedirs("out/step", exist_ok=True)
    t0 = time.time()
    st, cr, hb, fo, gate2d, cams = build_all()
    print("built", round(time.time() - t0, 1), "s")
    print("gate angles φ:", {k: round(v, 2) for k, v in PHI.items()})
    allp = {**st, **cr, **hb, **fo}
    for k, v in allp.items():
        print(f"{k:16s} vol {v.volume/1000:8.1f} cm3  valid={v.is_valid}")
