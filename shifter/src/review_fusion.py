# LSK H/SEQ Shifter — author: Sunggwan
"""v0.6 검토용 (Fusion 검토 대화, 11-27): 운전석 시점 렌더 · 기어별 자세 · 촘촘한 움직임 스윕 + 최소 틈.

python3 review_fusion.py render  [out_dir]   → 운전석 시점 / 기어별 자세 렌더
python3 review_fusion.py sweep   [out_dir]   → 중립 채널·각 게이트 슬롯을 촘촘히 훑어 움직이는 부품 ↔ 고정 부품 최소 틈(mm)
"""
import os, sys, json, math, itertools, time
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
import numpy as np, trimesh

MODE = sys.argv[1] if len(sys.argv) > 1 else "render"
OUTD = sys.argv[2] if len(sys.argv) > 2 else "out/review"
os.makedirs(OUTD, exist_ok=True)

import cad
import check as C

# 운전석: +X 쪽에 앉아 −X 방향을 봄 → 운전자 왼쪽 = −Y, 오른쪽 = +Y. 쉬프터는 운전자 오른쪽에 놓임.
EYE_DRIVER = (520, -430, 520)        # 앉은 눈 높이 (책상 윗면 위 약 600mm), 쉬프터 왼쪽·앞
MOVING = ("hub", "lever_bar", "sleeve", "lock_nut", "knob", "tail_bolt", "tail_cap", "boot_collar",
          "cradle", "cam", "cartridge", "hollow_pin", "roll_sensor_cap", "plunger_S1_L", "plunger_S1_R",
          "plunger_S2", "cup_S2", "arm", "rollers")


def render():
    import render_clamp as RC          # 무처리 6061 재질 + 책상
    RV = RC.RV
    imgs = []
    P0 = C.parts_at(0, 0, False)
    boot = lambda th, ph: cad.pose({"boot": cad.make_boot()}, th, ph, "hub")["boot"]
    desk = cad.desk_ghost()
    P = {**P0, "boot": boot(0, 0), "desk": desk}
    imgs.append(RV.shot(P, eye=EYE_DRIVER, target=(-10, -10, 20), fov=0.62, title="운전석 시점 (중립)"))
    top = RV.shot(P, eye=(300, 0, 520), target=(-8, 0, 52), fov=0.22, title="게이트 — 운전석에서 내려다봄", hide=("knob", "lock_nut", "sleeve"))
    imgs.append(top)
    RV.combine(imgs, f"{OUTD}/driver_view.png")
    print("saved driver_view", flush=True)
    poses = []
    for g, ph in cad.PHI.items():
        for th in (cad.THETA_STOP, -cad.THETA_STOP):
            poses.append((g, th, ph))
    tiles = []
    for g, th, ph in poses:
        RV._cache.clear()
        Pg = {**C.parts_at(th, ph, False), "boot": boot(th, ph)}
        gear = {"1-2": "21", "3-4": "43", "5-6": "65", "7-R": "R7"}[g][0 if th > 0 else 1]
        lab = f"{gear}단" if gear != "R" else "후진 R"
        tiles.append(RV.shot(Pg, eye=EYE_DRIVER, target=(-10, -10, 30), fov=0.62, size=(700, 700),
                             title=f"{lab}  ({g} 칸, {'몸 쪽' if th > 0 else '먼 쪽'})"))
        print("pose", g, th, flush=True)
    rows = [tiles[i:i + 4] for i in range(0, len(tiles), 4)]
    from PIL import Image
    W = 700 * 4; H = 700 * len(rows)
    sheet = Image.new("RGB", (W, H), "white")
    for r, row in enumerate(rows):
        for c, im in enumerate(row):
            sheet.paste(im, (c * 700, r * 700))
    sheet.save(f"{OUTD}/gear_poses.png")
    for th, nm in ((cad.SEQ_STOP, "seq_push"), (-cad.SEQ_STOP, "seq_pull")):
        Ps = {**C.parts_at(th, 0, True), "boot": boot(th, 0)}
        RV.shot(Ps, eye=EYE_DRIVER, target=(-10, -10, 30), fov=0.62, size=(700, 700), title=f"시퀀셜 θ {th:+.0f}°").save(f"{OUTD}/{nm}.png")
    print("done render")


def tri(shape, tol=0.08):
    v, t = shape.tessellate(tol, 0.3)
    return trimesh.Trimesh(np.array([[p.X, p.Y, p.Z] for p in v]), np.array(t), process=False)


_pts = {}


def _surf(m, step=0.6):
    """표면 점 구름 (꼭짓점 + 면적 비례 샘플, 평균 간격 약 step mm) — KD-tree 거리용, 캐시."""
    k = id(m)
    if k not in _pts:
        n = int(min(60000, max(2000, m.area / step ** 2)))
        _pts[k] = np.vstack([m.vertices, m.sample(n)])
    return _pts[k]


def min_gap(ma, mb):
    """두 점 구름 사이 최소 거리 근사 (점 구름 최근접, 오차 ≈ 샘플 간격/2). 0 근처 = 닿음/겹침."""
    from scipy.spatial import cKDTree
    pa, pb = ma, mb                                   # 이미 배치된 점 구름
    lo = np.maximum(pa.min(0), pb.min(0)) - 4.0
    hi = np.minimum(pa.max(0), pb.max(0)) + 4.0
    if np.any(lo > hi):
        return 9.9
    pa = pa[np.all((pa >= lo) & (pa <= hi), axis=1)]
    pb = pb[np.all((pb >= lo) & (pb <= hi), axis=1)]
    if len(pa) == 0 or len(pb) == 0:
        return 9.9
    d, _ = cKDTree(pb).query(pa, distance_upper_bound=10.0)
    return float(min(np.min(d), 9.9))


def _R(axis, deg, origin=(0, 0, 0)):
    return trimesh.transformations.rotation_matrix(math.radians(deg), axis, origin)


def sweep():
    """중립 메시를 한 번만 만들고 자세마다 회전 행렬만 적용 (cad.pose / check.follower_at 과 같은 변환)."""
    t0 = time.time()
    st, cr, hb, fo = C.st, C.cr, C.hb, C.fo
    grp = {**{k: "static" for k in st}, **{k: "cradle" for k in cr}, **{k: "hub" for k in hb}, **{k: "fol" for k in fo}}
    base = {}
    for d in (st, cr, hb, fo):
        for k, v in d.items():
            if k.startswith("clamp") or k in ("seq_lock", "mode_shaft", "mode_knob"):
                continue
            base[k] = tri(v)
    for seq in (False, True):
        for k, v in C.mode_parts(seq).items():
            base[(k, seq)] = tri(v)
    print(f"tessellated {len(base)} ({time.time()-t0:.0f}s)", flush=True)

    def placed(th, ph, seq):
        out = {}
        ps = float(np.interp(th, C.TH, C.PSI))
        Tc = _R((0, 1, 0), th)
        Th = Tc @ _R((1, 0, 0), -ph)
        Tf = _R((0, 1, 0), -ps, (cad.ARM_P[0], 0, cad.ARM_P[1]))
        for k, m in base.items():
            if isinstance(k, tuple):
                if k[1] != seq:
                    continue
                out[k[0]] = (_surf(m), "static")
                continue
            g = grp[k]
            T = {"static": None, "cradle": Tc, "hub": Th, "fol": Tf}[g]
            pts = _surf(m)
            if T is not None:
                pts = pts @ T[:3, :3].T + T[:3, 3]
            out[k] = (pts, g)
        return out

    poses = []
    for ph in np.linspace(cad.PHI["7-R"], cad.PHI["1-2"], 13):              # 중립 채널 (θ 0) 좌우
        poses.append((0.0, float(ph), False))
    for g, ph in cad.PHI.items():                                           # 각 슬롯 앞뒤 끝까지 1° 간격
        for th in np.arange(-cad.THETA_STOP, cad.THETA_STOP + 0.01, 1.0):
            poses.append((float(th), ph, False))
    for th in np.arange(-cad.SEQ_STOP, cad.SEQ_STOP + 0.01, 1.0):
        poses.append((float(th), 0.0, True))
    worst = {}
    for th, ph, seq in poses:
        P = placed(th, ph, seq)
        names = list(P)
        for a, b in itertools.combinations(names, 2):
            (ma, ga), (mb, gb) = P[a], P[b]
            if ga == gb or frozenset((a, b)) in C.IGNORE:
                continue                                    # 같이 움직이는 부품끼리는 상대 운동 없음
            g = min_gap(ma, mb)
            if g >= 9.9:
                continue
            key = f"{a}↔{b}"
            if key not in worst or g < worst[key][0]:
                worst[key] = (round(g, 2), th, round(ph, 2), seq)
        print(f"θ={th:+5.1f} φ={ph:+6.2f} {'SEQ' if seq else 'H'} ({time.time()-t0:.0f}s)", flush=True)
    rows = sorted(worst.items(), key=lambda kv: kv[1][0])
    with open(f"{OUTD}/sweep_min_gaps.json", "w") as f:
        json.dump({k: v for k, v in rows}, f, ensure_ascii=False, indent=1)
    print(f"{len(poses)} poses, {time.time()-t0:.0f}s")
    for k, v in rows[:25]:
        print(f"{v[0]:6.2f} mm  {k:40s} θ{v[1]:+.0f} φ{v[2]:+.1f} {'SEQ' if v[3] else ''}")


if __name__ == "__main__":
    {"render": render, "sweep": sweep}[MODE]()
