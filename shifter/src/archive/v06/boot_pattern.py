# LSK H/SEQ Shifter — author: Sunggwan
"""M11-03 inner dust boot — flat cutting pattern (fabric / TPU).

The boot is a ruled surface from the boot-frame inner edge (z 50, under the gate) to the top of the lever
collar (lever 34 mm, Ø14). Every generator must be long enough for the worst lever pose (all H gears
±14.5°, neutral channel, SEQ ±8°) plus SLACK, so the neutral shape is modelled as a "virtual" collar circle
lowered to z_v (the extra length hangs as a soft fold). The surface is triangulated and unrolled
(true lengths), seam at the rear (−X, hidden). Allowances: frame flap 8 (glued under the frame),
collar 6 (wrapped + glued / cable tie), seam overlap 6.

Usage: python3 boot_pattern.py [out_dir]   → M11-03_boot_pattern.dxf / .pdf / .png + numbers on stdout
"""
import math, os, sys
import numpy as np
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
import ezdxf
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import cad

N = 144                    # generators
SLACK = 1.05               # 5 % extra over the worst pose (lycra stretches; TPU needs ~1.10)
A_FRAME, A_COLLAR, A_SEAM = 8.0, 6.0, 6.0
R_COLLAR, L_COLLAR = 7.0, 34.0
Z_FRAME = cad.H_GATE - cad.GATE_T / 2 - 3          # underside of the boot frame (50)


def frame_inner():
    op = cad.gate_opening().buffer(1.5)
    bb = op.bounds
    return Polygon(cad.sbox(*bb).buffer(-1).buffer(1, resolution=12).exterior)


def ray_hit(poly, c, a):
    far = (c[0] + 500 * math.cos(a), c[1] + 500 * math.sin(a))
    p = LineString([c, far]).intersection(poly.exterior)
    p = min(p.geoms, key=lambda q: Point(c).distance(q)) if hasattr(p, "geoms") else p
    return np.array([p.x, p.y])


def rot(theta, phi, v):
    """cad.pose(level='hub'): rotate about X by −phi, then about Y by theta (degrees)."""
    a, b = math.radians(-phi), math.radians(theta)
    x, y, z = v
    y, z = y * math.cos(a) - z * math.sin(a), y * math.sin(a) + z * math.cos(a)
    x, z = x * math.cos(b) + z * math.sin(b), -x * math.sin(b) + z * math.cos(b)
    return np.array([x, y, z])


def poses():
    P = [(th, ph) for ph in cad.PHI.values() for th in np.linspace(-14.5, 14.5, 13)]
    P += [(0.0, ph) for ph in np.linspace(cad.PHI["7-R"], cad.PHI["1-2"], 25)]
    P += [(th, 0.0) for th in (-8.0, 8.0)]                       # SEQ
    return P


def build():
    fr = frame_inner()
    c = np.array(fr.centroid.coords[0])
    al = np.linspace(math.pi, 3 * math.pi, N, endpoint=False)  # start at −X (rear) → seam hidden at the back
    T = np.array([[*ray_hit(fr, c, a), Z_FRAME] for a in al])
    need = np.zeros(N)
    for th, ph in poses():
        for i, a in enumerate(al):
            b = rot(th, ph, (R_COLLAR * math.cos(a), R_COLLAR * math.sin(a), L_COLLAR))
            need[i] = max(need[i], np.linalg.norm(T[i] - b))
    B0 = np.array([[R_COLLAR * math.cos(a), R_COLLAR * math.sin(a)] for a in al])
    dxy = np.linalg.norm(T[:, :2] - B0, axis=1)
    dz = np.sqrt(np.maximum((SLACK * need) ** 2 - dxy ** 2, 0))
    z_v = Z_FRAME - dz.max()
    B = np.column_stack([B0, np.full(N, z_v)])
    return T, B, need, z_v, fr


def unroll(T, B):
    """Triangle strip T0 B0 T1 B1 … → 2D with true edge lengths."""
    T = np.vstack([T, T[:1]]); B = np.vstack([B, B[:1]])
    t2, b2 = [np.array([0.0, 0.0])], [np.array([0.0, -np.linalg.norm(T[0] - B[0])])]

    def third(p, q, dp, dq, side):
        d = np.linalg.norm(q - p)
        a = (dp ** 2 - dq ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(max(dp ** 2 - a ** 2, 0))
        u = (q - p) / d
        n = np.array([-u[1], u[0]]) * side
        return p + a * u + h * n

    for i in range(len(T) - 1):
        # triangle (T_i, B_i, T_i+1): T_i+1 on the far side of edge T_i→B_i
        ti1 = third(t2[-1], b2[-1], np.linalg.norm(T[i + 1] - T[i]), np.linalg.norm(T[i + 1] - B[i]), -1)
        bi1 = third(ti1, b2[-1], np.linalg.norm(B[i + 1] - T[i + 1]), np.linalg.norm(B[i + 1] - B[i]), -1)
        t2.append(ti1); b2.append(bi1)
    return np.array(t2), np.array(b2)


def offset_band(line_pts, dist, away_from):
    ls = LineString(line_pts)
    cands = [ls.offset_curve(dist), ls.offset_curve(-dist)]
    off = max(cands, key=lambda o: o.distance(Point(away_from)))
    return Polygon(list(ls.coords) + list(off.coords)[::-1]).buffer(0)


def main(out):
    os.makedirs(out, exist_ok=True)
    T, B, need, z_v, fr = build()
    t2, b2 = unroll(T, B)
    core = Polygon(list(t2) + list(b2[::-1])).buffer(0)
    mid = core.representative_point()
    bands = [offset_band(t2, A_FRAME, b2[len(b2) // 2]), offset_band(b2, A_COLLAR, t2[len(t2) // 2]),
             offset_band([t2[-1], b2[-1]], A_SEAM, (mid.x, mid.y))]
    cut = unary_union([core] + bands).buffer(0.3).buffer(-0.3)
    # normalise: put the pattern in the positive quadrant
    mnx, mny = cut.bounds[0], cut.bounds[1]
    sh = lambda a: np.asarray(a) - [mnx, mny]
    w, h = cut.bounds[2] - mnx, cut.bounds[3] - mny

    doc = ezdxf.new("R2010"); doc.units = ezdxf.units.MM
    for n, c, lt in (("CUT", 7, "CONTINUOUS"), ("GLUE_LINE", 1, "DASHED"), ("TEXT", 3, "CONTINUOUS")):
        doc.layers.add(n, color=c, linetype=lt if lt in doc.linetypes else "CONTINUOUS")
    msp = doc.modelspace()
    msp.add_lwpolyline([tuple(p) for p in sh(cut.exterior.coords)], close=True, dxfattribs={"layer": "CUT"})
    for line in (t2, b2, [t2[0], b2[0]], [t2[-1], b2[-1]]):
        msp.add_lwpolyline([tuple(p) for p in sh(line)], dxfattribs={"layer": "GLUE_LINE"})
    msp.add_text("M11-03 BOOT 1:1  SEAM=REAR  Sunggwan", dxfattribs={"layer": "TEXT", "height": 3.5}).set_placement(
        tuple(sh([mid.x - 30, mid.y])))
    doc.saveas(f"{out}/M11-03_boot_pattern.dxf")

    # 1:1 PDF (A4 landscape if it fits, else A3)
    W, H = (297, 210) if (w < 270 and h < 175) else (420, 297)
    fig = plt.figure(figsize=(W / 25.4, H / 25.4))
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, W); ax.set_ylim(0, H); ax.set_aspect("equal"); ax.axis("off")
    ox, oy = (W - w) / 2, (H - h) / 2 + 8
    S = lambda a: sh(a) + [ox, oy]
    x, y = S(cut.exterior.coords).T; ax.plot(x, y, "k-", lw=0.6)
    for line in (t2, b2, [t2[0], b2[0]], [t2[-1], b2[-1]]):
        x, y = S(line).T; ax.plot(x, y, "k--", lw=0.4)
    ax.add_patch(plt.Rectangle((10, 10), 50, 50, fill=False, lw=0.5))
    ax.text(12, 62, "50 mm 확인용 사각형 (100% 인쇄)", fontsize=6, family="Noto Sans CJK JP")
    tx, ty = S([[mid.x, mid.y]])[0]
    ax.text(tx, ty, "M11-03 내부 부트 1:1\n실선 = 재단 / 점선 = 접착선\n위 = 프레임 쪽(+8) · 아래 = 칼라 쪽(+6)\n솔기 겹침 6, 뒤(−X)",
            fontsize=7, ha="center", va="center", family="Noto Sans CJK JP")
    fig.savefig(f"{out}/M11-03_boot_pattern.pdf"); fig.savefig(f"{out}/M11-03_boot_pattern.png", dpi=110)
    print(f"frame inner {fr.bounds[2]-fr.bounds[0]:.1f} x {fr.bounds[3]-fr.bounds[1]:.1f}, "
          f"need {need.min():.1f}..{need.max():.1f} mm, virtual collar z {z_v:.1f}, "
          f"pattern {w:.0f} x {h:.0f} mm on {'A4' if W == 297 else 'A3'}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "../CAD/v0.6/buy")
