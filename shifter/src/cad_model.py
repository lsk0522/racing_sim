# LSK H/SEQ Shifter — author: Sunggwan
# v0.7: 이 파일은 "모델 좌표"(v0.6까지의 좌표, 반전 전). 최종 형상은 cad.py가 Y를 뒤집어서 내보낸다 → 바깥에서는 cad.py만 쓴다.
"""
LSK H/SEQ Shifter v0.4 — modular, CNC 6061 + ABS/PETG prints. Parametric CAD (build123d)
Origin = gimbal centre. X = forward, Y = left, Z = up. Units: mm
"""
import math, json
import numpy as np
from build123d import *
from shapely.geometry import Point as SPoint, Polygon as SPolygon
from shapely.ops import unary_union
import smooth2d
import cam as camgen

Z_BASE = -66.0
BASE_T = 8.0
H_GATE = 55.0
GATE_T = 3.0
LEVER_D = 8.0
L_KNOB = camgen.L_KNOB
GATE_Y = {"1-2": 15.0, "3-4": 0.0, "5-6": -15.0, "7-R": -30.0}
PHI_M = {k: math.degrees(math.atan(v / H_GATE)) for k, v in GATE_Y.items()}   # 모델 좌표 (cad.py가 Y 반전 → 최종 PHI = −PHI_M)
PHI = PHI_M
THETA_STOP = camgen.STOP_DEG      # 14
THETA_GATE_END = 14.5
SEQ_STOP = 8.0                    # SEQ lock pocket pitch limit
BRG = dict(od=16.0, w=5.0)
YC = -34.75                       # roller/cam centre Y
CAM_Y0, CAM_Y1 = -43.0, -26.5
ARM_P = (camgen.P[0], camgen.P[1])

# ======================= Helpers =======================
def _txt_premirror(shape, y):
    """v0.7: 최종 형상은 cad.py가 Y를 뒤집음 → 글자는 미리 제자리(y 평면 기준)에서 뒤집어 두면 최종에서 바로 읽힘."""
    return shape.mirror(Plane(origin=(0, y, 0), z_dir=(0, 1, 0)))


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cyl_x(r, x0, x1, y=0.0, z=0.0):
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, x1 - x0)


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, y1 - y0)


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, z1 - z0)


SMOOTH = True        # 2D 선분 묶음 → 직선·원호·스플라인 (매끄러운 STEP). False = 예전 다각형


def _face2d(pts):
    if SMOOTH:
        f = smooth2d.smooth_face(pts)
        if f is not None:
            return f
    return Polygon(*pts, align=None)


def xz_prism(pts, y0, y1):
    """XZ polygon → solid over the Y range y0..y1."""
    f = Plane.XZ * _face2d(pts)
    p = extrude(f, amount=y1 - y0)          # direction depends on point winding → realigned by bbox
    return p.moved(Location((0, y0 - p.bounding_box().min.Y, 0)))


def xy_prism(pts, z0, z1):
    f = Plane.XY * _face2d(pts)
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
    for phi in np.linspace(PHI_M["7-R"], PHI_M["1-2"], 60):
        for th in np.linspace(-0.4, 0.4, 3):
            poses.append((th, phi))
    for g, phi in PHI_M.items():
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
    for phi in np.linspace(PHI_M["7-R"], PHI_M["1-2"], 60):
        for th in np.linspace(-0.4, 0.4, 3):
            poses.append((th, phi))
    for g, phi in PHI_M.items():
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
    # v0.6 rev (11-27 Fusion 검토): 윗모서리 C1.8 → 7-R 게이트에서 카트리지 바닥과 0.31 → ≥1.2 mm (출력끼리 상대 운동)
    top = [e for e in cap.edges() if abs(e.center().Z + 47.5) < 1e-3 and abs(e.radius - 5.0) < 1e-3]
    cap = cap.chamfer(1.8, None, top)
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
    # v0.6 rev (11-27 Fusion 검토): 7-R에서 꼬리 볼트가 슬롯 끝 바닥 턱(y 24, 0.8t)에 0.12 mm 겹침 → S2 포켓 아래 바닥 턱을 터서 틈 ≥1.2
    c -= box(-5.5, 5.5, 23.0, 26.5, -40, -38.2)
    # v0.6 rev: θ ±14°에서 −Y 끝 바닥 모서리(와 M3 머리)가 디텐트 암 롤러 축 보스와 0.50 mm.
    # v0.7.1 (검증): 끝 바닥을 1.3 평평하게 들어 올렸더니 출력 때 베드에서 뜬 6.5 mm 처마(서포트 필요)가 됨
    # → 바닥 긴 모서리 양쪽 45° C2 모따기(전체 길이)로 변경: 출력 서포트 없음, 암 보스와 틈 ≥1.2
    for sx in (-1, 1):
        c -= xz_prism([(sx * 10.5, -39.5), (sx * 7.5, -39.5), (sx * 10.5, -36.5)], -27, 48)
    c -= cyl_y(4.3, -24.1, -11, z=-24)
    c -= cyl_y(4.3, 22, 30.1, z=-24)
    c -= cyl_y(4.3, 26, 41, z=-34)                 # S2 (OD8 wire1.0 L0 27, k≈5)
    c -= cyl_y(2.1, 40, 48, z=-34)                 # M5 tap (S2 preload, accessible from +Y)
    for x in (-7, 7):
        for y in (-23.5, 23.5):
            c -= cyl_z(1.7, -40, -19, x=x, y=y)
            c -= cyl_z(3.0, -40, -34.5, x=x, y=y)  # M3 counterbore (v0.6 rev: 1.5 깊게 → 머리 아래면 z −37.5, M3×20 그대로 물림 5.5)
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
    # side tab (rides on the eccentric: H −61 / SEQ −55).
    # v0.6 rev (11-27 Fusion 검토): 평평한 바닥은 SEQ에서 기울어 x −2 끝이 편심에 0.4 mm 파고듦(0.97 mm³, check 문턱 아래)
    # → 바닥을 볼록 원호 R10.9 (꼭짓점 x 1, z −61)로: H 접점 x 1 / SEQ 접점 x ≈ 3.2, 잠금 각도는 그대로 SEQ_ANG
    #   조건 (x0+30)·sinα + (R−1)·cosα − R = 5 → R = 1 + 9.9·x0. 탭 윗면 −58 → −57 (원호 끝 두께 확보)
    TAB_R, TAB_X0 = 10.9, 1.0
    a += box(-2, 6, 8.3, 17, -61.5, -57) & cyl_y(TAB_R, 8.3, 17, x=TAB_X0, z=-61 + TAB_R)
    # v0.6 rev: H에서 편심 원판이 몸통 +Y 옆면 아래 모서리를 0.29 mm 깎음(0.995 mm³) → 옆면 아래쪽 1 mm 릴리프 (틈 0.7)
    a -= box(-3, 7, 7.3, 8.4, -66, -62.5)
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


# ======================= v0.6: faceted block exterior (sandwich) =======================
# Mechanism = v0.4 unchanged. Changes are exterior + non-functional details only:
#   gate 3t → 4t (centre still at H_GATE), tower top edge −0.5 mm, gate tap X positions, lever cut to 104,
#   mode shaft shortened, RP2040 pocket / USB-C moved to rear centre.
from shapely.geometry import box as sbox

GATE_T = 4.0                         # gate plate 4t (centre stays at H_GATE = 55 → z 53..57)
Z_TOP = H_GATE + GATE_T / 2          # 57
X_FRONT, X_REAR = 46.0, -84.0
PANEL_T = 3.0
Y_NEG_IN, Y_POS_IN = -64.5, 48.0     # side-panel inner faces (clear pitch sensor cap −64 / spring cartridge +47)
Y_NEG_OUT, Y_POS_OUT = Y_NEG_IN - PANEL_T, Y_POS_IN + PANEL_T
Y_MID = (Y_NEG_OUT + Y_POS_OUT) / 2
WALL = 3.0
GAP = 1.0                            # shadow gap width/depth (metal–metal: gate–panel, base reveal)
GAP_PRINT = 1.2                      # printed-part boundary (cover edge rebate, gate–cover): print tolerance
GATE_X = (-34.0, 34.0)               # gate plate X range
GATE_Y = (Y_NEG_IN + GAP, Y_POS_IN - GAP)
GATE_BOLTS = [(x, y) for x in (-29.0, 28.0) for y in (-53.0, 31.0)]   # into tower top edges
L_LEVER_TOP = 104.0                  # Ø8 lever top (inside the Ø12 sleeve)
S_SLEEVE = (72.0, 108.0)             # Ø12 sleeve (M12×1.25 on 85..108)
S_NUT = (85.0, 94.0)                 # taper lock-nut
S_KNOB = (94.0, 126.0)               # knob (grip centre ≈ L_KNOB 110 → cam force curve unchanged)
MK_X = (41.5, 46.0)                  # mode knob X range (front face flush)
DISH_R, DISH_D = 22.0, 6.5           # scalloped finger dish around the mode knob
LABEL_SIZE, LABEL_D = 5.5, 0.5       # front H / SEQ engraving (v0.6 rev: 4.2 / 0.4 was hard to read)
LABEL_Y = (-7.5, 37.0)               # label centres, outside the dish (dish edge on the face y −2.3 .. 28.9)
SLEEVE_SET_Z = 88.0                  # M3 set screw in the sleeve, hidden under the lock-nut (85..94)


def _rear_slope():
    """Tower rear slope offset 4 mm outward → body rear facet line."""
    top = H_GATE - GATE_T / 2
    p0, p1 = np.array([-36.0, top]), np.array([-80.0, -12.0])
    d = (p0 - p1) / np.linalg.norm(p0 - p1)
    n = np.array([-d[1], d[0]])
    a = p0 + 4.0 * n
    x_top = a[0] + (Z_TOP - a[1]) * d[0] / d[1]
    z_rear = a[1] + (X_REAR - a[0]) * d[1] / d[0]
    return float(x_top), float(z_rear)


X_TOP_REAR, Z_REAR_KNEE = _rear_slope()
X_TOP_FRONT = 38.0                                   # front facet parallel to the rear facet
Z_FRONT_KNEE = Z_TOP - (X_FRONT - X_TOP_FRONT) * (Z_TOP - Z_REAR_KNEE) / (X_TOP_REAR - X_REAR)


def silhouette():
    return SPolygon([(X_FRONT, Z_BASE), (X_FRONT, Z_FRONT_KNEE), (X_TOP_FRONT, Z_TOP), (X_TOP_REAR, Z_TOP),
                     (X_REAR, Z_REAR_KNEE), (X_REAR, Z_BASE)])


def _dish():
    c = (X_FRONT + DISH_R - DISH_D, SHAFT_Y, SHAFT_Z)
    s = Pos(*c) * Sphere(DISH_R)
    s += cyl_x(10.3, MK_X[0] - 1.5, X_FRONT + 1, y=SHAFT_Y, z=SHAFT_Z)      # knob body relief
    return s


def _csk(x, y, z, axis="z", sign=1, d_hole=3.4, d_head=6.3):
    """Countersunk hole: surface at (x,y,z), going inward along −sign·axis."""
    h = (d_head - d_hole) / 2
    c = Cone(d_hole / 2, d_head / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
    c = Pos(0, 0, -h) * c
    c += Pos(0, 0, -12) * Cylinder(d_hole / 2, 12.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    c += Pos(0, 0, 0) * Cylinder(d_head / 2, 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    if axis == "z":
        r = Rot(0, 0, 0) if sign > 0 else Rot(180, 0, 0)
    else:                                                      # y
        r = Rot(-90, 0, 0) if sign > 0 else Rot(90, 0, 0)
    return Pos(x, y, z) * r * c


def _csk_head(x, y, z, axis="z", sign=1):
    """M3 DIN7991 head, top 0.1 below the surface, hex socket 2 AF."""
    h = 1.5
    c = Pos(0, 0, -0.1 - h) * Cone(1.5, 3.0, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
    c -= Pos(0, 0, -1.4) * extrude(RegularPolygon(2.0 / math.sqrt(3), 6), amount=1.4)
    r = (Rot(0, 0, 0) if sign > 0 else Rot(180, 0, 0)) if axis == "z" else (Rot(-90, 0, 0) if sign > 0 else Rot(90, 0, 0))
    return Pos(x, y, z) * r * c


def _chamfer_outer(s, size, normal):
    """Chamfer every edge of the planar faces whose normal ≈ `normal` (outer face of a plate)."""
    try:
        fs = [f for f in s.faces() if f.geom_type == GeomType.PLANE and
              (f.normal_at() - Vector(*normal)).length < 1e-3]
        es = []
        for f in fs:
            es += f.edges()
        return s.chamfer(size, None, es)
    except Exception as e:
        print("chamfer skipped:", e)
        return s


# ---------- [M10] side panels (6061 3t) ----------
PANEL_BOLTS = [(40.5, -60.5), (40.5, 36.0), (-78.5, -60.5)]     # 5.5 mm from the edges (+ one on the rear facet)
PANEL_BORDER, FIELD_D, ISLAND_R = 10.0, 0.8, 15.0              # raised frame / recessed field / pivot medallion


def _facet_boss_xz():
    x_top, _ = X_TOP_REAR, None
    zb = 26.0
    xo = X_TOP_REAR + (zb - Z_TOP) * (X_TOP_REAR - X_REAR) / (Z_TOP - Z_REAR_KNEE)
    d = np.array([X_TOP_REAR - X_REAR, Z_TOP - Z_REAR_KNEE]); d /= np.linalg.norm(d)
    n_in = np.array([d[1], -d[0]])                                 # inward normal of the rear facet
    p = np.array([xo, zb]) + 5.5 * n_in
    return float(p[0]), float(p[1])


def panel_bolts():
    return PANEL_BOLTS + [_facet_boss_xz()]


PANEL_WIN = (-20.0, 20.0, -52.0, -30.0, 3.0)                     # −Y window x0,x1,z0,z1,r
S2_ACCESS = (0.0, -34.0)                                          # +Y hex-key port for the 7·R preload screw


def panel_field():
    f = silhouette().buffer(-PANEL_BORDER, join_style=2).buffer(-3).buffer(3, resolution=16)
    return f.difference(SPoint(0, 0).buffer(ISLAND_R, 64))


def make_panel(side):
    """6061 3t, anodized. Raised 10 mm frame + 0.8 mm recessed field + Ø30 pivot medallion (bright chamfers)."""
    y0, y1 = (Y_NEG_OUT, Y_NEG_IN) if side < 0 else (Y_POS_IN, Y_POS_OUT)
    yo = y0 if side < 0 else y1
    sg = -1 if side < 0 else 1                                       # outward direction
    s = xz_prism(shapely_to_pts(silhouette(), 0.01), y0, y1)
    s = _chamfer_outer(s, 1.2, (0, sg, 0))
    fld = panel_field()
    pk = xz_prism(shapely_to_pts(SPolygon(fld.exterior), 0.01), min(yo, yo - sg * FIELD_D), max(yo, yo - sg * FIELD_D))
    pk = pk.moved(Location((0, sg * 0.5, 0))) if False else pk
    pk = xz_prism(shapely_to_pts(SPolygon(fld.exterior), 0.01), yo - FIELD_D if sg > 0 else yo - 1,
                  yo + 1 if sg > 0 else yo + FIELD_D)
    pk -= cyl_y(ISLAND_R, yo - 2, yo + 2)
    s -= pk
    try:
        from shapely.geometry import Point as _P
        es = [e for e in s.edges() if abs(e.center().Y - yo) < 0.01 and
              (fld.exterior.distance(_P(e.center().X, e.center().Z)) < 0.05 or
               abs(math.hypot(e.center().X, e.center().Z) - ISLAND_R) < 0.05 or
               (e.geom_type == GeomType.CIRCLE and abs(e.radius - ISLAND_R) < 0.05))]
        s = s.chamfer(0.5, None, es)
    except Exception as e:
        print("field chamfer skipped", e)
    yf = yo - sg * FIELD_D                                           # field floor plane
    if side < 0:
        x0, x1, z0, z1, r = PANEL_WIN
        win = sbox(x0, z0, x1, z1).buffer(-r).buffer(r, resolution=16)
        s -= xz_prism(shapely_to_pts(win, 0.01), y0 - 1, y1 + 1)
        seat = sbox(x0 - 3, z0 - 3, x1 + 3, z1 + 3).buffer(-r).buffer(r, resolution=16)
        s -= xz_prism(shapely_to_pts(seat, 0.01), Y_NEG_IN - 1.2, Y_NEG_IN + 0.5)   # acrylic seat 1.2 deep
        try:
            es = [e for e in s.edges() if abs(e.center().Y - yf) < 0.01 and
                  win.exterior.distance(__import__("shapely.geometry", fromlist=["Point"]).Point(e.center().X, e.center().Z)) < 0.05]
            s = s.chamfer(0.4, None, es)
        except Exception as e:
            print("window chamfer skipped", e)
    else:
        s -= cyl_y(3.0, y0 - 1, y1 + 1, x=S2_ACCESS[0], z=S2_ACCESS[1])
        s = _chamfer_hole(s, S2_ACCESS, yf)
    for (x, z) in panel_bolts():
        s -= _csk(x, yo, z, axis="y", sign=sg)
    s -= cyl_y(1.0, yo - 0.3 if sg > 0 else yo, yo if sg > 0 else yo + 0.3, x=0, z=0)   # centre mark on the medallion
    return s


def _chamfer_hole(s, xz, y):
    try:
        es = [e for e in s.edges() if e.geom_type == GeomType.CIRCLE and abs(e.center().Y - y) < 0.01
              and abs(e.radius - 3.0) < 0.01]
        return s.chamfer(0.5, None, es)
    except Exception:
        return s


def make_window():
    x0, x1, z0, z1, r = PANEL_WIN
    a = sbox(x0 - 2.8, z0 - 2.8, x1 + 2.8, z1 + 2.8).buffer(-r).buffer(r, resolution=16)
    return xz_prism(shapely_to_pts(a, 0.01), Y_NEG_IN - 1.2, Y_NEG_IN)


# ---------- [M10] front / rear covers (ABS or PETG, matte) ----------
COVER_BOSSES = [(39.0, -40.0), (39.0, 32.0), (-77.0, -10.0), (-77.0, 39.0)]   # vertical, M3 insert, screwed from under the base


def cover_shell2d():
    S = silhouette()
    S_ext = S.union(sbox(X_REAR - 5, Z_BASE - 10, X_FRONT + 5, Z_BASE + 0.001))
    inner = S_ext.buffer(-WALL, join_style=2)
    sh = S.difference(inner)
    sh = sh.difference(sbox(GATE_X[0] - GAP_PRINT, Z_TOP - WALL - 1, GATE_X[1] + GAP_PRINT, Z_TOP + 1))   # gate plate pocket
    return sh


def make_cover(front):
    sh = cover_shell2d()
    half = sbox(0, Z_BASE - 5, X_FRONT + 5, Z_TOP + 5) if front else sbox(X_REAR - 5, Z_BASE - 5, 0, Z_TOP + 5)
    p = sh.intersection(half)
    p = max(p.geoms, key=lambda g: g.area) if p.geom_type == "MultiPolygon" else p
    s = xz_prism(shapely_to_pts(p, 0.01), Y_NEG_IN, Y_POS_IN)
    # shadow gaps along both side edges: 1.2 × 1.2 rebate on the outer skin
    S = silhouette()
    rebate = S.difference(S.buffer(-GAP_PRINT, join_style=2)).intersection(p.buffer(0.01))
    if not rebate.is_empty:
        geoms = rebate.geoms if rebate.geom_type == "MultiPolygon" else [rebate]
        for g in geoms:
            if g.area < 0.5:
                continue
            pts = shapely_to_pts(g, 0.01)
            s -= xz_prism(pts, Y_NEG_IN - 1, Y_NEG_IN + GAP_PRINT)
            s -= xz_prism(pts, Y_POS_IN - GAP_PRINT, Y_POS_IN + 1)
    # panel-screw bosses (Y axis, M3 heat-set insert)
    for (x, z) in panel_bolts():
        if (x > 0) != front:
            continue
        for ya, yb in ((Y_NEG_IN, Y_NEG_IN + 6), (Y_POS_IN - 6, Y_POS_IN)):
            s += cyl_y(4.0, ya, yb, x=x, z=z) - cyl_y(2.0, ya - 1, yb + 1, x=x, z=z)
    # base bosses (Z axis)
    for (x, y) in COVER_BOSSES:
        if (x > 0) != front:
            continue
        s += cyl_z(4.0, Z_BASE, Z_BASE + 10, x=x, y=y) - cyl_z(2.0, Z_BASE - 1, Z_BASE + 9, x=x, y=y)
    if front:
        s -= _dish()
        s -= cyl_x(3.2, X_FRONT - WALL - 1, X_FRONT + 1, y=SHAFT_Y, z=SHAFT_Z)
        fz0, fz1 = -38.0, Z_FRONT_KNEE - 8.0                            # recessed field (0.8) above the control strip
        f = box(X_FRONT - 0.8, X_FRONT + 1, Y_NEG_IN + 8, Y_POS_IN - 8, fz0, fz1)
        s -= f.fillet(3.0, f.edges().filter_by(Axis.X))
        for txt, y in (("H", LABEL_Y[0]), ("SEQ", LABEL_Y[1])):        # mode labels (0.5 engraving)
            try:
                pl = Plane(origin=(X_FRONT - LABEL_D, y, SHAFT_Z + 3.0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
                s -= _txt_premirror(extrude(pl * Text(txt, font_size=LABEL_SIZE, font="Noto Sans CJK JP"), amount=1.0), y)
            except Exception as e:
                print("label skipped", e)
    else:
        f = box(X_REAR - 1, X_REAR + 0.8, Y_NEG_IN + 8, Y_POS_IN - 8, Z_BASE + 10, Z_REAR_KNEE - 10)
        s -= f.fillet(3.0, f.edges().filter_by(Axis.X))
        try:                                                            # maker mark in the rear field (0.3)
            pl = Plane(origin=(X_REAR + 0.8 + 0.3, Y_MID, (Z_BASE + Z_REAR_KNEE) / 2), x_dir=(0, -1, 0), z_dir=(-1, 0, 0))
            s -= _txt_premirror(extrude(pl * Text("SUNGGWAN", font_size=3.4, font="Noto Sans CJK JP"), amount=1.0), Y_MID)
        except Exception as e:
            print("label skipped", e)
    return s


# ---------- [M7] gate plate 4t ----------
def gate2d():
    opening = gate_opening()
    outer = sbox(GATE_X[0], GATE_Y[0], GATE_X[1], GATE_Y[1]).buffer(-1.5).buffer(1.5, resolution=16)
    return outer.difference(opening)


def make_gate():
    p2 = gate2d()
    g = plate_xy(p2, H_GATE - GATE_T / 2, Z_TOP, ch=0.6)
    for (x, y) in GATE_BOLTS:
        g -= _csk(x, y, Z_TOP, "z", 1)
    # v0.7: 운전석 = +X. 최종(반전 후) 홀수 단은 먼 쪽(−X), 1-2 칸은 운전자 왼쪽(−Y), 글자는 운전석에서 바로 읽힘.
    # 여기는 반전 전 모델 좌표라 y 부호가 반대 (1-2 칸 모델 y +15 → 최종 −15).
    labels = {"1": (-23, 15), "2": (23, 15), "3": (-23, 0), "4": (23, 0), "5": (-23, -15), "6": (23, -15),
              "7": (-23, -30), "R": (23, -30)}
    for txt, (x, y) in labels.items():
        try:
            t = Pos(x, y, Z_TOP - 0.4) * Rot(0, 0, 90) * extrude(Text(txt, font_size=6, font="Noto Sans CJK JP"), amount=1.0)
            g -= _txt_premirror(t, y)
        except Exception as e:
            print("gate label skipped", txt, e)
    for (x, y) in BOOT_FRAME_BOLTS:                                  # blind M3 (2.5 Ø, 3 deep) from below
        g -= cyl_z(1.25, H_GATE - GATE_T / 2 - 0.1, H_GATE - GATE_T / 2 + 3, x=x, y=y)
    return g, p2


# ---------- inner dust boot (fabric / TPU) — frame + lever collar ----------
BOOT_FRAME_BOLTS = [(-24.0, -41.0), (-24.0, 26.0), (24.0, -41.0), (24.0, 26.0)]


def make_boot_frame():
    op = gate_opening().buffer(1.5)
    bb = op.bounds
    outer = sbox(bb[0] - 4.5, bb[1] - 4.5, bb[2] + 4.5, bb[3] + 4.5).buffer(-2).buffer(2, resolution=12)
    inner = sbox(bb[0], bb[1], bb[2], bb[3]).buffer(-1).buffer(1, resolution=12)
    f = plate_xy(outer.difference(inner), H_GATE - GATE_T / 2 - 3, H_GATE - GATE_T / 2)
    for (x, y) in BOOT_FRAME_BOLTS:
        f -= cyl_z(1.7, H_GATE - 10, H_GATE, x=x, y=y)
    return f


def make_boot_collar():
    return cyl_z(7.0, 28.0, 34.0) - cyl_z(LEVER_D / 2, 27, 35)


def make_boot():
    """Visual only (flexible, excluded from the interference check): neutral shape of the inner boot."""
    op = gate_opening().buffer(1.5).bounds
    top = Plane.XY.offset(H_GATE - GATE_T / 2 - 3) * Pos((op[0] + op[2]) / 2, (op[1] + op[3]) / 2) * \
        RectangleRounded(op[2] - op[0], op[3] - op[1], 2)
    mid = Plane.XY.offset(42) * Pos(0, (op[1] + op[3]) / 2 * 0.6) * RectangleRounded((op[2] - op[0]) * 0.75, (op[3] - op[1]) * 0.7, 6)
    bot = Plane.XY.offset(33) * Circle(7.0)
    b = loft([top, mid, bot])
    try:
        return b.offset_3d(None, -0.8)
    except Exception:
        return b


# ---------- [M3]/[M8] lever, Ø12 sleeve, taper lock-nut, knob ----------
def make_lever():
    return cyl_z(LEVER_D / 2, 3, L_LEVER_TOP)


def make_sleeve():
    s = cyl_z(6.0, *S_SLEEVE) - cyl_z(LEVER_D / 2, S_SLEEVE[0] - 1, S_SLEEVE[1] + 1)
    s -= cyl_x(1.25, -7.0, 0.0, y=0.0, z=SLEEVE_SET_Z)               # M3 tap (Ø2.5), rear side, hidden by the lock-nut
    return s


def make_lock_nut():
    n = Pos(0, 0, S_NUT[0]) * Cone(7.2, 9.8, S_NUT[1] - S_NUT[0], align=(Align.CENTER, Align.CENTER, Align.MIN))
    for sgn in (-1, 1):                                                # 2 wrench flats, 17 AF
        n -= box(8.5 * sgn if sgn > 0 else -20, 20 if sgn > 0 else -8.5, -20, 20, S_NUT[0] + 2.0, S_NUT[1] + 1)
    n -= cyl_z(6.0, S_NUT[0] - 1, S_NUT[1] + 1)
    try:
        n = n.chamfer(0.4, None, n.edges().filter_by(GeomType.CIRCLE))
    except Exception:
        pass
    return n


def make_knob():
    """Slim racing teardrop, M12×1.25 female thread (aftermarket knobs fit the same sleeve)."""
    from scipy.interpolate import PchipInterpolator as _P
    a, b = S_KNOB
    zk = np.array([a, a + 6, a + 14, a + 20, a + 27, a + 31.4])
    rk = np.array([9.8, 10.6, 14.0, 15.4, 14.5, 12.2])
    fz = _P(zk, rk)
    zz = np.linspace(zk[0], zk[-1], 48)
    prof = [(0.0, float(a))] + [(float(fz(z)), float(z)) for z in zz] + [(11.2, float(b)), (0.0, float(b))]
    k = revolve(Plane.XZ * Polygon(*prof, align=None), Axis.Z, 360)
    k -= cyl_z(6.0, a - 1, a + 15)                                     # M12 thread bore (14 deep)
    k -= cyl_z(LEVER_D / 2 + 0.5, a + 14, a + 15)
    # tier line (V-groove) at the neck/body transition
    groove = revolve(Plane.XZ * Polygon((10.6, a + 7.2), (13.0, a + 6.0), (13.0, a + 8.4), align=None), Axis.Z, 360)
    k -= groove
    zt = b
    for y in (5.25, 1.75, -1.75, -5.25):                               # top engraving: 7+R H
        k -= box(-5.5, 5.5, y - 0.45, y + 0.45, zt - 0.5, zt + 1)
    k -= box(-0.45, 0.45, -5.7, 5.7, zt - 0.5, zt + 1)
    return k


# ---------- [M6] mode shaft / semi-recessed mode knob ----------
def make_mode_shaft(seq=False):
    off = 3.0 if seq else -3.0
    s = cyl_x(2.5, -2, MK_X[1] - 0.5, y=SHAFT_Y, z=SHAFT_Z)
    s += cyl_x(5.5, -2, 6, y=SHAFT_Y, z=SHAFT_Z + off)
    return s


def make_mode_knob(seq=False):
    """Ø19 knurled disc in the scalloped dish; face flush with the front. Pointer line: −Y = H, +Y = SEQ."""
    k = cyl_x(9.5, *MK_X, y=SHAFT_Y, z=SHAFT_Z)
    for i in range(30):
        k -= Pos(0, SHAFT_Y, SHAFT_Z) * Rot(12 * i, 0, 0) * box(MK_X[0] - 1, MK_X[1] + 1, -0.4, 0.4, 8.9, 10)
    k -= box(MK_X[1] - 0.5, MK_X[1] + 1, SHAFT_Y - 8.0, SHAFT_Y - 2.5, SHAFT_Z - 0.5, SHAFT_Z + 0.5)   # pointer → −Y in H
    k -= cyl_x(2.5, MK_X[0] - 1, MK_X[1] + 1, y=SHAFT_Y, z=SHAFT_Z)
    try:
        k = k.chamfer(0.4, None, [e for e in k.edges() if e.geom_type == GeomType.CIRCLE and e.radius > 9])
    except Exception:
        pass
    if seq:
        k = k.rotate(Axis((0, SHAFT_Y, SHAFT_Z), (1, 0, 0)), 180)
    return k


# ---------- [M2] towers: gate taps moved to the symmetric bolt pattern ----------
def make_tower(side):
    y0, y1 = ((-56.0, -50.0) if side < 0 else (28.0, 34.0))
    t = plate_xz(tower2d(side), y0, y1, ch=0.6)
    yc = (y0 + y1) / 2
    top = H_GATE - GATE_T / 2
    for x in (-29.0, 28.0):
        t -= cyl_z(1.25, top - 8, top + 1, x=x, y=yc)
    for x in (-70, 26):
        t -= cyl_z(1.7, Z_BASE - 1, Z_BASE + 9, x=x, y=yc)
    yo = y0 if side < 0 else y1
    t -= box(-11.5, -8.5, yo - 2 if side > 0 else yo, yo if side > 0 else yo + 2, Z_BASE - 1, 0)
    return t


# ---------- [M1] base 8t (flush with the body, reveal groove, rear-centre USB-C) ----------
BASE_R = 2.5
RP_Y = (Y_MID - 13.0, Y_MID + 13.0)
CORNER_SCREWS = [(X_REAR + 4.5, Y_NEG_OUT + 4.5), (X_FRONT - 4.5, Y_NEG_OUT + 4.5),
                 (X_REAR + 4.5, Y_POS_OUT - 4.5), (X_FRONT - 4.5, Y_POS_OUT - 4.5)]
M6_HOLES = [(-60, -40), (-20, -40), (-20, 20), (20, 20)]


def base_outline():
    return sbox(X_REAR, Y_NEG_OUT, X_FRONT, Y_POS_OUT).buffer(-BASE_R).buffer(BASE_R, resolution=16)


def base2d():
    p = base_outline()
    holes = [(x, y, 2.2) for x in (-70, 26) for y in (-53, 31)]
    holes += [(x, y, 1.7) for x, y in ((-72, -24), (-72, -46), (-28, -24), (-28, -46))]
    holes += [(x, y, 1.7) for x, y in COVER_BOSSES]
    for x, y, r in holes:
        p = p.difference(SPoint(x, y).buffer(r, 16))
    return p


def make_base():
    p2 = base2d()
    zb = Z_BASE - BASE_T
    b = xy_prism(shapely_to_pts(SPolygon(p2.exterior), 0.01), zb, Z_BASE)
    for h in p2.interiors:
        b -= xy_prism([(float(x), float(y)) for x, y in list(h.coords)[:-1]], zb - 1, Z_BASE + 1)
    # reveal groove (1 × 1) around the top outer edge
    ring = base_outline().difference(base_outline().buffer(-GAP, join_style=2))
    b -= xy_prism(shapely_to_pts(ring.exterior and SPolygon(ring.exterior), 0.01), Z_BASE - GAP, Z_BASE + 1) - \
        xy_prism(shapely_to_pts(base_outline().buffer(-GAP, join_style=2), 0.01), Z_BASE - GAP - 1, Z_BASE + 2)
    try:
        es = [e for e in b.edges() if abs(e.center().Z - zb) < 0.01]
        fs = [f for f in b.faces() if f.geom_type == GeomType.PLANE and abs(f.center().Z - zb) < 0.01]
        outer_edges = fs[0].outer_wire().edges() if fs else []
        b = b.chamfer(0.8, None, list(outer_edges))
    except Exception as e:
        print("base chamfer skipped", e)
    # bottom side: electronics pocket (rear centre), USB-C exit, cable channels
    b -= box(-80, -46, RP_Y[0], RP_Y[1], zb - 1, zb + 5)               # RP2040-Zero + JST pocket
    b -= box(X_REAR - 1, -79, Y_MID - 6.0, Y_MID + 6.0, zb - 1, zb + 4.5)   # USB-C plug slot 12 × 5.5 (rear centre)
    b -= box(-50, -44, RP_Y[0], 30, zb - 1, zb + 3)                    # link channel → +Y harness
    b -= box(-12, -8, -60, 8, zb - 1, zb + 3) + box(-46, -8, 4, 8, zb - 1, zb + 3)
    b -= box(-12, -8, 30, 37, zb - 1, zb + 3) + box(-46, -8, 26, 30, zb - 1, zb + 3)
    for x, y in ((-10, -58.5), (-10, 35.5)):
        b -= cyl_z(2.0, zb - 1, Z_BASE + 1, x=x, y=y)
    b -= box(-3, 7, SHAFT_Y - 7, SHAFT_Y + 7, Z_BASE - 6.5, Z_BASE + 1)
    for x, y in ((-36, -11), (-36, 11), (-24, -11), (-24, 11)):
        b -= cyl_z(1.25, zb - 1, Z_BASE + 1, x=x, y=y)
    for x, y in M6_HOLES:
        b -= cyl_z(2.5, zb - 1, zb + 6, x=x, y=y)
    for x, y in CORNER_SCREWS:
        b -= cyl_z(1.25, zb - 1, zb + 6, x=x, y=y)                     # M3 tap (bottom cover)
    # rear face: USB-C cable guide (1 mm recess around the slot)
    b -= box(X_REAR - 1, X_REAR + 1.0, Y_MID - 8.5, Y_MID + 8.5, zb - 1, zb + 6.5)
    b -= _dish()
    return b, p2


def make_bottom_cover():
    """2t print. Silicone anti-slip pads sit in 1.0 mm pockets; M6 rig/clamp holes left open."""
    zb = Z_BASE - BASE_T
    c = xy_prism(shapely_to_pts(base_outline(), 0.01), zb - 2, zb)
    for x, y in M6_HOLES:
        c -= cyl_z(3.6, zb - 3, zb + 1, x=x, y=y)
    for x, y in CORNER_SCREWS:
        c -= _csk(x, y, zb - 2, "z", -1, d_hole=3.4, d_head=6.3)
    for x0, x1, y0, y1 in ((-76, -50, -61, -47), (12, 36, -61, -47), (-76, -50, 32, 44), (12, 36, 32, 44)):
        c -= box(x0, x1, y0, y1, zb - 3, zb - 1.0)
    c -= _dish()
    return c


# ---------- [M12] desk clamp (Desker black wood desk, top ≈ 28 mm, fits 15..38 mm) ----------
# +X (mode-knob face) faces the driver → the desk front edge is on the +X side.
CL_PT = 6.0                          # top plate 6t (under the bottom cover)
CL_ST = 12.0                         # spine / jaw 12t
CL_Z0 = Z_BASE - BASE_T - 2.0        # −76: underside of the printed bottom cover = plate top
CL_ZP = CL_Z0 - CL_PT                # −82: plate bottom (silicone strips stick out 1.0 below)
SIL = 1.0                            # silicone 2t in 1.0 pockets → 1.0 proud
DESK_EDGE_X = 51.0                   # desk front edge (spine inner face 52 + silicone 1.0)
CL_OPEN = 48.0                       # plate bottom → jaw top
CL_ZJ = CL_ZP - CL_OPEN              # −130: jaw top
CL_X = (-87.0, 64.0)                 # plate X (spine 52..64 under the front lip)
CL_SX = (52.0, 64.0)
CL_JX = (4.0, 64.0)                  # jaw reaches 47 mm under the desk
CL_Y = (Y_NEG_OUT - 3.0, Y_POS_OUT + 3.0)
CL_SY = (Y_MID - 50.0, Y_MID + 50.0)
CL_M5 = [(58.0, Y_MID + d) for d in (-38.0, 0.0, 38.0)]
CL_SCREW = (24.0, Y_MID)             # M10 clamp screw, 27 mm in from the desk edge
CL_PAD = (44.0, 36.0, 6.0)           # printed pressure pad (+ silicone 2t on top)
DESK_T = 28.0                        # Desker top thickness (assembly pose)
DESK_TOP = CL_ZP - SIL


def clamp_plate2d():
    return sbox(CL_X[0], CL_Y[0], CL_X[1], CL_Y[1]).buffer(-4.0).buffer(4.0, resolution=16)


def make_clamp_plate():
    p = plate_xy(clamp_plate2d(), CL_ZP, CL_Z0, ch=0.5)
    for x, y in M6_HOLES:                                           # M6 DIN7991 from below → base blind taps
        p -= _csk(x, y, CL_ZP, "z", -1, d_hole=6.6, d_head=12.4)
    for x, y in CL_M5:                                              # M5 DIN7991 from above → spine top edge
        p -= _csk(x, y, CL_Z0, "z", 1, d_hole=5.5, d_head=10.4)
    for y0, y1 in ((CL_Y[0] + 4, CL_Y[0] + 22), (CL_Y[1] - 22, CL_Y[1] - 4)):
        p -= box(CL_X[0] + 6, CL_SX[0] - 4, y0, y1, CL_ZP - 1, CL_ZP + SIL)      # silicone pockets (desk top)
    return p


def make_clamp_spine():
    s = box(CL_SX[0], CL_SX[1], CL_SY[0], CL_SY[1], CL_ZJ, CL_ZP)
    for x, y in CL_M5:
        s -= cyl_z(2.1, CL_ZP - 12, CL_ZP + 1, x=x, y=y)            # M5 tap top edge, depth 12
        s -= cyl_z(2.1, CL_ZJ - 1, CL_ZJ + 12, x=x, y=y)            # M5 tap bottom edge
    s -= box(CL_SX[0] - 1, CL_SX[0] + SIL, CL_SY[0] + 6, CL_SY[1] - 6, CL_ZP - 30, CL_ZP - 6)   # silicone (desk edge)
    return _chamfer_loop(s, 0.5, (1, 0, 0))


def _chamfer_loop(s, size, normal):
    """Chamfer only the outer boundary of the planar face(s) whose normal ≈ `normal`."""
    try:
        es = []
        for f in s.faces():
            if f.geom_type == GeomType.PLANE and (f.normal_at() - Vector(*normal)).length < 1e-3:
                es += f.outer_wire().edges()
        return s.chamfer(size, None, es)
    except Exception as e:
        print("chamfer skipped:", e)
        return s


def make_clamp_jaw():
    j = box(CL_JX[0], CL_JX[1], CL_SY[0], CL_SY[1], CL_ZJ - CL_ST, CL_ZJ)
    for x, y in CL_M5:
        j -= _csk(x, y, CL_ZJ - CL_ST, "z", -1, d_hole=5.5, d_head=10.4)
    j -= cyl_z(4.25, CL_ZJ - CL_ST - 1, CL_ZJ + 1, x=CL_SCREW[0], y=CL_SCREW[1])   # M10 tap through
    return _chamfer_loop(_chamfer_loop(j, 0.5, (0, 0, -1)), 0.5, (0, 0, 1))


def clamp_pad_z(desk_t=DESK_T):
    top = DESK_TOP - desk_t - SIL * 2                               # silicone 2t between pad and desk
    return top - CL_PAD[2], top


def make_clamp_pad(desk_t=DESK_T):
    z0, z1 = clamp_pad_z(desk_t)
    x, y = CL_SCREW
    p = xy_prism(shapely_to_pts(sbox(x - CL_PAD[0] / 2, y - CL_PAD[1] / 2, x + CL_PAD[0] / 2, y + CL_PAD[1] / 2)
                                .buffer(-4).buffer(4, resolution=16), 0.01), z0, z1)
    p -= cyl_z(5.4, z0 - 1, z0 + 3.5, x=x, y=y)                     # bolt-tip socket Ø10.8 × 3.5 (lets the pad tilt)
    return p


def make_clamp_bolt(desk_t=DESK_T):
    """M10×50 hex bolt (buy) — tip 3 mm inside the pad socket."""
    z0, _ = clamp_pad_z(desk_t)
    tip = z0 + 3.0
    x, y = CL_SCREW
    b = cyl_z(4.9, tip - 50.0, tip, x=x, y=y)
    b += Pos(x, y, tip - 50.0 - 6.5) * extrude(RegularPolygon(8.5 / math.cos(math.pi / 6), 6), amount=6.5)
    return b


def make_clamp_knob(desk_t=DESK_T):
    """Printed star knob, M10 hex head (17 AF) pressed in + epoxy."""
    z0, _ = clamp_pad_z(desk_t)
    zh = z0 + 3.0 - 50.0                                            # bolt-head underside
    x, y = CL_SCREW
    lobes = SPoint(0, 0).buffer(13.0, 32)
    for k in range(5):
        a = 2 * math.pi * k / 5
        lobes = lobes.union(SPoint(17 * math.cos(a), 17 * math.sin(a)).buffer(6.0, 24))
    lobes = lobes.buffer(-2.0).buffer(2.0, resolution=16)
    pts = [(px + x, py + y) for px, py in shapely_to_pts(lobes, 0.01)]
    k = xy_prism(pts, zh - 6.5, zh + 5.0)                          # bottom flush with the bolt head
    k -= Pos(x, y, zh - 7.5) * extrude(RegularPolygon(8.6 / math.cos(math.pi / 6), 6), amount=7.5)
    k -= cyl_z(5.2, zh - 1, zh + 5, x=x, y=y)
    return k


def desk_ghost(desk_t=DESK_T):
    """Reference only (check.py): the desk top slab."""
    return box(DESK_EDGE_X - 200, DESK_EDGE_X, CL_Y[0] - 20, CL_Y[1] + 20, DESK_TOP - desk_t, DESK_TOP)


_build_all_v05 = build_all


def build_all():
    st, cr, hb, fo, plates2d, camH = _build_all_v05()
    st.pop("shell", None)
    st["base"], plates2d["base"] = make_base()
    st["bottom_cover"] = make_bottom_cover()
    st["tower_negY"], st["tower_posY"] = make_tower(-1), make_tower(+1)
    st["gate"], plates2d["gate"] = make_gate()
    plates2d["tower_negY"], plates2d["tower_posY"] = tower2d(-1), tower2d(1)
    st["panel_negY"], st["panel_posY"] = make_panel(-1), make_panel(+1)
    st["cover_front"], st["cover_rear"] = make_cover(True), make_cover(False)
    st["window"] = make_window()
    st["boot_frame"] = make_boot_frame()
    st["clamp_plate"], st["clamp_spine"], st["clamp_jaw"] = make_clamp_plate(), make_clamp_spine(), make_clamp_jaw()
    st["clamp_pad"], st["clamp_bolt"], st["clamp_knob"] = make_clamp_pad(), make_clamp_bolt(), make_clamp_knob()
    plates2d["clamp_plate"] = clamp_plate2d()
    heads = Part()
    for (x, y) in GATE_BOLTS:
        heads += _csk_head(x, y, Z_TOP, "z", 1)
    for (x, z) in panel_bolts():
        heads += _csk_head(x, Y_NEG_OUT, z, "y", -1) + _csk_head(x, Y_POS_OUT, z, "y", 1)
    st["bolt_heads"] = heads
    plates2d["panel_negY"] = silhouette().difference(
        sbox(PANEL_WIN[0], PANEL_WIN[2], PANEL_WIN[1], PANEL_WIN[3]).buffer(-PANEL_WIN[4]).buffer(PANEL_WIN[4], resolution=16))
    plates2d["panel_posY"] = silhouette().difference(SPoint(*S2_ACCESS).buffer(3.0, 32))
    hb["lever_bar"] = make_lever()
    hb["knob"] = make_knob()
    hb["sleeve"] = make_sleeve()
    hb["lock_nut"] = make_lock_nut()
    hb["boot_collar"] = make_boot_collar()
    return st, cr, hb, fo, plates2d, camH
