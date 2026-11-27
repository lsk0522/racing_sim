# LSK H/SEQ Shifter — author: Sunggwan
"""다각형(shapely 꼭짓점) → 매끄러운 경계 (직선 + 원호 + 스플라인).

shapely buffer/simplify로 만든 2D 형상은 원호가 짧은 선분 수십 개로 쪼개져 있어서,
그대로 돌출하면 STEP도 각진 면(STL처럼)이 된다. 여기서 선분 묶음을
  - 한 직선 위 → Line
  - 한 원 위 (오차 0.01mm) → 원호 / 전체 원 → Circle
  - 그 외 곡선 → 접선 연속 B-스플라인
으로 바꿔 매끄러운 면을 만든다.
"""
import math
import numpy as np
from build123d import Edge, Wire, Face, Vector

TOL_ARC = 0.012       # mm, 원 맞춤 허용 오차
TURN_CORNER = 28.0    # deg, 이보다 크게 꺾이면 모서리
TURN_STRAIGHT = 0.35  # deg, 이보다 작으면 같은 직선
L_STRAIGHT = 2.2      # mm, 이보다 긴 선분은 직선으로 취급


def _clean(pts):
    p = [tuple(map(float, q)) for q in pts]
    if len(p) > 1 and np.hypot(p[0][0] - p[-1][0], p[0][1] - p[-1][1]) < 1e-7:
        p = p[:-1]
    out = []
    for q in p:
        if not out or np.hypot(q[0] - out[-1][0], q[1] - out[-1][1]) > 1e-5:
            out.append(q)
    if len(out) > 1 and np.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) < 1e-5:
        out.pop()
    return np.array(out)


def _fit_circle(P):
    x, y = P[:, 0], P[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(P))]
    b = x * x + y * y
    (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = math.sqrt(max(c + cx * cx + cy * cy, 0.0))
    err = np.abs(np.hypot(x - cx, y - cy) - r).max()
    return (cx, cy), r, err


def _turns(P):
    n = len(P)
    t = np.zeros(n)
    for i in range(n):
        a = P[i] - P[i - 1]; b = P[(i + 1) % n] - P[i]
        cr = a[0] * b[1] - a[1] * b[0]; dt = a @ b
        t[i] = math.degrees(math.atan2(cr, dt))
    return t


def _V(p):
    return Vector(float(p[0]), float(p[1]), 0.0)


def _curve_edges(Q, t0=None, t1=None):
    """열린 곡선 점열 Q → 원호 하나 또는 스플라인 하나."""
    if len(Q) >= 4:
        c, r, err = _fit_circle(Q)
        sweep = abs(sum(_turns(Q)[1:-1]))
        if err < TOL_ARC and r < 5000 and sweep < 300:
            m = Q[len(Q) // 2]
            return [Edge.make_three_point_arc(_V(Q[0]), _V(m), _V(Q[-1]))]
    if len(Q) == 2:
        return [Edge.make_line(_V(Q[0]), _V(Q[1]))]
    if len(Q) == 3:
        c, r, err = _fit_circle(Q)
        return [Edge.make_three_point_arc(_V(Q[0]), _V(Q[1]), _V(Q[2]))]
    tans = None
    if t0 is not None and t1 is not None:
        tans = [_V(t0), _V(t1)]
    return [Edge.make_spline([_V(q) for q in Q], tangents=tans, scale=True)]


def smooth_wire(pts):
    P = _clean(pts)
    n = len(P)
    if n < 3:
        raise ValueError("too few points")
    # 1) 전체가 원
    c, r, err = _fit_circle(P)
    if n >= 10 and err < TOL_ARC:
        return Wire([Edge.make_circle(r).moved(Location_xy(c))])
    turns = _turns(P)
    seg = np.array([np.linalg.norm(P[(i + 1) % n] - P[i]) for i in range(n)])
    # 2) 꼭짓점 분류: corner / break(직선↔곡선 경계)
    straight = seg >= L_STRAIGHT                       # segment i: P[i] → P[i+1]
    # 짧아도 앞뒤가 거의 안 꺾이면 직선
    for i in range(n):
        if not straight[i] and abs(turns[i]) < TURN_STRAIGHT and abs(turns[(i + 1) % n]) < TURN_STRAIGHT:
            straight[i] = True
    corner = np.abs(turns) > TURN_CORNER
    brk = corner.copy()
    for i in range(n):
        if straight[i - 1] != straight[i]:
            brk[i] = True
        if straight[i - 1] and straight[i] and abs(turns[i]) >= TURN_STRAIGHT:
            brk[i] = True                                 # 직선과 직선 사이 꺾임
    if not brk.any():                                     # 모서리 없는 닫힌 곡선 → 주기 스플라인
        return Wire([Edge.make_spline([_V(q) for q in P] + [_V(P[0])], periodic=True)])
    start = int(np.argmax(brk))
    order = [(start + k) % n for k in range(n)] + [start]
    edges = []
    run = [order[0]]
    for k in range(1, len(order)):
        j = order[k]
        run.append(j)
        if brk[j] or k == len(order) - 1:
            i0 = run[0]
            Q = P[run]
            if straight[i0]:
                # 직선 구간: 끝점끼리 한 선
                edges.append(Edge.make_line(_V(Q[0]), _V(Q[-1])))
            else:
                t0 = t1 = None
                if not corner[run[0]]:
                    d = P[run[0]] - P[run[0] - 1]; t0 = d / np.linalg.norm(d)
                if not corner[run[-1]]:
                    d = P[(run[-1] + 1) % n] - P[run[-1]]; t1 = d / np.linalg.norm(d)
                if t0 is not None and t1 is not None:
                    edges += _curve_edges(Q, t0, t1)
                else:
                    edges += _curve_edges(Q)
            run = [j]
    return Wire(edges)


def Location_xy(c):
    from build123d import Location
    return Location((float(c[0]), float(c[1]), 0.0))


def smooth_face(pts):
    """XY 평면 위 매끄러운 면. 실패하면 None."""
    try:
        w = smooth_wire(pts)
        if not w.is_closed:
            return None
        f = Face(w)
        if not f.is_valid or f.area < 1e-6:
            return None
        return f
    except Exception:
        return None
