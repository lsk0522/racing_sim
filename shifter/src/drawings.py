# LSK H/SEQ Shifter — author: Sunggwan
"""
도면 생성기: 부품 STEP + 도면 스펙 YAML → A4 DXF (+ PDF/PNG 미리보기)

적용 규격
 - 용지·윤곽·중심 마크·구역 표시: ISO 5457 (A4 세로, 철 여백 20mm, 나머지 10mm)
 - 표제란: ISO 7200 (폭 180mm, 오른쪽 아래), 추가 정보란(재질·척도·투상법·일반공차·ISO 8015)
 - 선: ISO 128-20 선 그룹 0.35 (외형선 0.35 / 가는 선 0.18, 숨은선 02, 중심선 04)
 - 문자: ISO 3098 (치수·주기 2.5, 표제 5)
 - 투상: 3각법 ISO 128-30 / ISO 5456-2, 척도 ISO 5455
 - 치수: ISO 129-1, 끼워맞춤 ISO 286 (자동 계산), 일반공차 ISO 2768-mK
 - 기하공차 ISO 1101 (공차 기입틀 높이 = 2h), 데이텀 ISO 5459, 거칠기 기호 ISO 1302 / 값 ISO 21920
 - 모서리 ISO 13715
형상: 은선 처리(HLR)는 OpenCascade로 직접 — 날카로운 모서리 + 윤곽선만, 접선(부드러운) 모서리·이음선은 그리지 않음.
원·원호는 DXF CIRCLE/ARC로 그대로 출력.
"""
import os, math, datetime
import numpy as np
import ezdxf
from ezdxf.enums import TextEntityAlignment
import iso_tol

from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepLib import BRepLib
from OCP.GeomAbs import GeomAbs_Line, GeomAbs_Circle
from OCP.GCPnts import GCPnts_AbscissaPoint

A4 = (210.0, 297.0)
FRAME = (20.0, 10.0, 200.0, 287.0)            # ISO 5457: 철 여백 20, 나머지 10
TB_H, ADD_H = 32.0, 10.0                       # 표제란 / 추가 정보란 높이
TB_TOP = FRAME[1] + TB_H + ADD_H               # 52
REV_X0 = 110.0                                 # 개정 이력표 (오른쪽 위) 왼쪽 끝
TH = 2.5                                       # ISO 3098 문자 높이 (치수·주기)
STD_SCALES = [5, 2, 1, 0.5, 0.2, 0.1]          # ISO 5455
DIM_FIRST, DIM_STEP = 8.0, 7.0                 # 외형선 → 첫 치수선, 치수선 간격
FONT_DXF = "malgun.ttf"
FONT_PREVIEW = "NotoSansCJK-Regular.ttc"
DENSITY = {"Al 6061-T6": 2.70, "Al 7075-T6": 2.81, "SUS304": 7.93}

VIEWS = {   # 이름: (3D→2D, 시선 방향(관찰자 쪽), 도면 x축)
    "front": (lambda p: (p[0], p[2]), (0, -1, 0), (1, 0, 0)),
    "top":   (lambda p: (p[0], p[1]), (0, 0, 1), (1, 0, 0)),
    "right": (lambda p: (p[1], p[2]), (1, 0, 0), (0, 1, 0)),
}


# ---------------------------------------------------------------- 은선 처리 (HLR)
def _edges(compound):
    out = []
    if compound is None or compound.IsNull():
        return out
    BRepLib.BuildCurves3d_s(compound)
    ex = TopExp_Explorer(compound, TopAbs_EDGE)
    while ex.More():
        out.append(TopoDS.Edge(ex.Current()))
        ex.Next()
    return out


def _edge_geom(e):
    """edge → ('line', p0, p1) | ('circle', c, r) | ('arc', c, r, a0, a1) | ('poly', pts)"""
    c = BRepAdaptor_Curve(e)
    u0, u1 = c.FirstParameter(), c.LastParameter()
    P = lambda u: (c.Value(u).X(), c.Value(u).Y())
    t = c.GetType()
    if t == GeomAbs_Line:
        return ("line", P(u0), P(u1))
    if t == GeomAbs_Circle:
        circ = c.Circle(); ctr = (circ.Location().X(), circ.Location().Y()); r = circ.Radius()
        if abs((u1 - u0) - 2 * math.pi) < 1e-6:
            return ("circle", ctr, r)
        ang = lambda p: math.degrees(math.atan2(p[1] - ctr[1], p[0] - ctr[0])) % 360
        a0, a1, am = ang(P(u0)), ang(P(u1)), ang(P((u0 + u1) / 2))
        ccw = lambda a, b, m: ((m - a) % 360) < ((b - a) % 360)
        return ("arc", ctr, r, a0, a1) if ccw(a0, a1, am) else ("arc", ctr, r, a1, a0)
    L = GCPnts_AbscissaPoint.Length_s(c)
    n = max(16, int(L / 0.15))
    return ("poly", [P(u) for u in np.linspace(u0, u1, n)])


def project(shape, view):
    """절대 좌표 2D 결과: visible, hidden (각각 geom 목록), bbox"""
    _, d, xd = VIEWS[view]
    algo = HLRBRep_Algo()
    algo.Add(shape.wrapped)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*d), gp_Dir(*xd))))
    algo.Update(); algo.Hide()
    h = HLRBRep_HLRToShape(algo)
    vis = [g for comp in (h.VCompound(), h.OutLineVCompound()) for g in map(_edge_geom, _edges(comp))]
    hid = [g for comp in (h.HCompound(), h.OutLineHCompound()) for g in map(_edge_geom, _edges(comp))]
    pts = []
    for g in vis:
        if g[0] == "line":
            pts += [g[1], g[2]]
        elif g[0] in ("circle", "arc"):
            c, r = g[1], g[2]
            pts += [(c[0] - r, c[1] - r), (c[0] + r, c[1] + r)] if g[0] == "circle" else \
                   [(c[0] + r * math.cos(math.radians(a)), c[1] + r * math.sin(math.radians(a)))
                    for a in np.linspace(g[3], g[3] + (g[4] - g[3]) % 360, 9)]
        else:
            pts += g[1]
    P = np.array(pts) if pts else np.zeros((1, 2))
    return vis, hid, (P[:, 0].min(), P[:, 1].min(), P[:, 0].max(), P[:, 1].max())


def _len(g):
    if g[0] == "line":
        return math.dist(g[1], g[2])
    if g[0] == "circle":
        return 2 * math.pi * g[2]
    if g[0] == "arc":
        return math.radians((g[4] - g[3]) % 360) * g[2]
    return sum(math.dist(a, b) for a, b in zip(g[1], g[1][1:]))


def draw_geom(msp, g, T, s, layer):
    if _len(g) * s < 0.05:
        return
    a = {"layer": layer}
    if g[0] == "line":
        msp.add_line(T(g[1]), T(g[2]), dxfattribs=a)
    elif g[0] == "circle":
        msp.add_circle(T(g[1]), g[2] * s, dxfattribs=a)
    elif g[0] == "arc":
        msp.add_arc(T(g[1]), g[2] * s, g[3], g[4], dxfattribs=a)
    else:
        msp.add_lwpolyline([T(p) for p in g[1]], dxfattribs=a)


# ---------------------------------------------------------------- DXF 기본
def setup_doc():
    doc = ezdxf.new("R2018", setup=True, units=4)
    doc.header["$MEASUREMENT"] = 1
    doc.header["$LTSCALE"] = 1.0
    doc.header["$LWDISPLAY"] = 1
    doc.styles.add("KR", font=FONT_DXF)
    # ISO 128-20 선 종류 (가는 선 d=0.25 기준): 02 숨은선 대시 12d·간격 3d, 04 중심선 24d·3d·0.5d·3d
    doc.linetypes.add("ISO02", pattern=[3.75, 3.0, -0.75], description="ISO 128 02 dashed __ __ __")
    doc.linetypes.add("ISO04", pattern=[7.6, 6.0, -0.75, 0.1, -0.75], description="ISO 128 04 long dash dot __ . __")
    for name, color, lt, lw in [("VISIBLE", 7, "Continuous", 35), ("HIDDEN", 8, "ISO02", 18), ("CENTER", 1, "ISO04", 18),
                                ("DIM", 3, "Continuous", 18), ("GDT", 4, "Continuous", 18), ("TEXT", 7, "Continuous", 18),
                                ("FRAME", 7, "Continuous", 70), ("TB", 7, "Continuous", 35), ("THIN", 7, "Continuous", 18)]:
        doc.layers.add(name, color=color, linetype=lt, lineweight=lw)
    ds = doc.dimstyles.new("ISO-A4", dxfattribs={
        "dimtxsty": "KR", "dimtxt": TH, "dimasz": 2.5, "dimexe": 1.5, "dimexo": 1.0, "dimgap": 0.6,
        "dimtad": 1, "dimtih": 0, "dimtoh": 0, "dimdec": 2, "dimdsep": ord("."), "dimlunit": 2, "dimzin": 8,
        "dimscale": 1.0, "dimclrd": 3, "dimclre": 3, "dimclrt": 3, "dimtdec": 3, "dimtzin": 8, "dimtfac": 0.75,
        "dimadec": 0, "dimaunit": 0, "dimlwd": 18, "dimlwe": 18})
    ds.set_arrows(blk="")      # 닫힌 칠한 화살표 (ISO 129)
    return doc


def text_width(s, h):
    # DXF 문자 높이 = 대문자 높이(≈ 0.72 em) → 한글 ≈ 1.3h, 영문 대문자 ≈ 0.9h, 숫자·소문자 ≈ 0.75h
    w = 0.0
    for ch in s:
        o = ord(ch)
        w += h * (1.32 if o > 0x2E80 else 0.38 if ch == " " else 0.95 if ch.isupper() or ch in "Ø×±°—→①②③④⑤⑥" else 0.4 if ch in ".,:;()|'il1" else 0.78)
    return w


def wrap_text(s, width, h, indent="   "):
    out, cur = [], ""
    for tok in s.split(" "):
        cand = (cur + " " + tok) if cur else tok
        if text_width(cand, h) <= width or not cur:
            cur = cand
        else:
            out.append(cur); cur = indent + tok
    out.append(cur)
    return out


def text(msp, s, x, y, h=TH, layer="TEXT", align=TextEntityAlignment.MIDDLE_LEFT, rot=0):
    t = msp.add_text(s, height=h, dxfattribs={"layer": layer, "style": "KR", "rotation": rot})
    t.set_placement((x, y), align=align)
    return t


def rect(msp, x0, y0, x1, y1, layer="THIN"):
    msp.add_lwpolyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], close=True, dxfattribs={"layer": layer})


def arrow(msp, tip, frm, layer="GDT", size=2.5):
    d = np.array(tip, float) - np.array(frm, float); L = np.linalg.norm(d)
    if L < 1e-6:
        return
    d /= L; n = np.array([-d[1], d[0]])
    b = np.array(tip) - d * size
    pts = [tuple(tip), tuple(b + n * size * 0.17), tuple(b - n * size * 0.17)]     # 약 20° 닫힌 화살표
    h = msp.add_hatch(dxfattribs={"layer": layer}); h.paths.add_polyline_path(pts, is_closed=True)


# ---------------------------------------------------------------- 기호 (ISO 1101 / 5459 / 1302 / 5456-2)
def gdt_symbol(msp, kind, cx, cy, s=1.5, layer="GDT"):
    L = lambda pts: msp.add_lwpolyline(pts, dxfattribs={"layer": layer})
    C = lambda r, x=cx, y=cy: msp.add_circle((x, y), r, dxfattribs={"layer": layer})
    if kind == "flatness":
        L([(cx - s, cy - s * .5), (cx + s * .5, cy - s * .5), (cx + s, cy + s * .5), (cx - s * .5, cy + s * .5), (cx - s, cy - s * .5)])
    elif kind == "straightness":
        L([(cx - s, cy), (cx + s, cy)])
    elif kind == "parallelism":
        L([(cx - s * .8, cy - s), (cx - s * .1, cy + s)]); L([(cx + s * .1, cy - s), (cx + s * .8, cy + s)])
    elif kind == "perpendicularity":
        L([(cx - s, cy - s), (cx + s, cy - s)]); L([(cx, cy - s), (cx, cy + s)])
    elif kind == "angularity":
        L([(cx + s, cy - s), (cx - s, cy - s), (cx + s, cy + s * .6)])
    elif kind == "position":
        C(s * .65); L([(cx - s, cy), (cx + s, cy)]); L([(cx, cy - s), (cx, cy + s)])
    elif kind == "coaxiality":
        C(s * .5); C(s)
    elif kind == "circularity":
        C(s)
    elif kind == "cylindricity":
        C(s * .7); L([(cx - s * .6, cy - s), (cx - s * .2, cy + s)]); L([(cx + s * .2, cy - s), (cx + s * .6, cy + s)])
    elif kind == "profile_line":
        msp.add_arc((cx, cy - s * .5), s, 0, 180, dxfattribs={"layer": layer})
    elif kind == "profile_surface":
        msp.add_arc((cx, cy - s * .5), s, 0, 180, dxfattribs={"layer": layer}); L([(cx - s, cy - s * .5), (cx + s, cy - s * .5)])
    elif kind == "runout":
        L([(cx - s * .7, cy - s), (cx + s * .5, cy + s * .6)]); arrow(msp, (cx + s * .7, cy + s), (cx - s * .2, cy), layer, 1.0)
    elif kind == "symmetry":
        for dy, w in ((-s * .6, .6), (0, 1), (s * .6, .6)):
            L([(cx - s * w, cy + dy), (cx + s * w, cy + dy)])


def gdt_frame(msp, x, y, kind, tol, datums=(), target=None):
    """ISO 1101 공차 기입틀: 높이 2h, 기호 칸 정사각형. (x, y) = 왼쪽 아래."""
    h = 2 * TH
    w_tol = max(h * 1.6, text_width(tol, TH) + 2.5); w_d = h
    xs = [x, x + h, x + h + w_tol] + [x + h + w_tol + w_d * (i + 1) for i in range(len(datums))]
    for i in range(len(xs) - 1):
        rect(msp, xs[i], y, xs[i + 1], y + h, layer="GDT")
    gdt_symbol(msp, kind, x + h / 2, y + h / 2)
    text(msp, tol, x + h + w_tol / 2, y + h / 2, TH, "GDT", TextEntityAlignment.MIDDLE_CENTER)
    for i, dd in enumerate(datums):
        text(msp, dd, xs[2 + i] + w_d / 2, y + h / 2, TH, "GDT", TextEntityAlignment.MIDDLE_CENTER)
    if target is not None:
        start = (x, y + h / 2) if target[0] < x else (xs[-1], y + h / 2)
        msp.add_lwpolyline([start, target], dxfattribs={"layer": "GDT"})
        arrow(msp, target, start)


def datum_symbol(msp, letter, target, box_at):
    """ISO 5459 데이텀: 칠한 삼각형 + 지시선 + 정사각형(2h)."""
    bx, by = box_at; a = TH
    rect(msp, bx - a, by - a, bx + a, by + a, layer="GDT")
    text(msp, letter, bx, by, TH, "GDT", TextEntityAlignment.MIDDLE_CENTER)
    d = np.array(target, float) - np.array(box_at, float); L = np.linalg.norm(d); d /= max(L, 1e-6)
    start = np.array(box_at) + d * a * 1.05
    msp.add_lwpolyline([tuple(start), tuple(target)], dxfattribs={"layer": "GDT"})
    n = np.array([-d[1], d[0]])
    tri = [tuple(target), tuple(np.array(target) - d * 2.2 + n * 1.3), tuple(np.array(target) - d * 2.2 - n * 1.3)]
    h = msp.add_hatch(dxfattribs={"layer": "GDT"}); h.paths.add_polyline_path(tri, is_closed=True)


def roughness(msp, x, y, value, leader_to=None, general=False):
    """ISO 1302 제거 가공 기호 (H1 = 1.4h, H2 = 3h). (x, y) = 기호 끝점."""
    H1, H2 = 1.4 * TH, 3.0 * TH
    msp.add_lwpolyline([(x - H1 * 0.58, y + H1), (x, y), (x + H2 * 0.58, y + H2), (x + H2 * 0.58 + text_width(value, TH) + 1.5, y + H2)],
                       dxfattribs={"layer": "GDT"})
    msp.add_lwpolyline([(x - H1 * 0.58, y + H1), (x + H1 * 0.58, y + H1)], dxfattribs={"layer": "GDT"})
    text(msp, value, x + H2 * 0.58 + 0.8, y + H2 - TH * 0.9, TH, "GDT")
    if general:      # 괄호 안 기본 기호: "그 외 모든 면"
        gx = x + H2 * 0.58 + text_width(value, TH) + 5
        text(msp, "(", gx - 1.5, y + H1 * 0.6, TH * 1.4, "GDT")
        msp.add_lwpolyline([(gx + 1.2 - H1 * 0.4, y + H1 * 0.7), (gx + 1.2, y), (gx + 1.2 + H1 * 0.6, y + H1 * 1.1)], dxfattribs={"layer": "GDT"})
        text(msp, ")", gx + 2.6, y + H1 * 0.6, TH * 1.4, "GDT")
    if leader_to is not None:
        msp.add_lwpolyline([(x, y), leader_to], dxfattribs={"layer": "GDT"})


def third_angle_symbol(msp, x, y, k=0.8):
    """ISO 5456-2 3각법 기호 (원뿔대 정면 + 측면). (x, y) = 왼쪽 중심."""
    r1, r2, L = 2.5 * k, 5.0 * k, 6.0 * k
    msp.add_lwpolyline([(x, y - r1), (x + L, y - r2), (x + L, y + r2), (x, y + r1), (x, y - r1)], dxfattribs={"layer": "THIN"})
    cx = x + L + 3.5 * k + r2
    msp.add_circle((cx, y), r2, dxfattribs={"layer": "THIN"}); msp.add_circle((cx, y), r1, dxfattribs={"layer": "THIN"})
    msp.add_line((x - 1.5, y), (cx + r2 + 1.5, y), dxfattribs={"layer": "CENTER"})
    msp.add_line((cx, y - r2 - 1.5), (cx, y + r2 + 1.5), dxfattribs={"layer": "CENTER"})


# ---------------------------------------------------------------- 치수 (ISO 129-1)
def dim_tol_override(spec, size):
    if "fit" in spec:
        u, l = iso_tol.fit(size, spec["fit"])
        return f" {spec['fit']}", u, l
    if "tol" in spec:
        t = str(spec["tol"]).replace(" ", "")
        if t.startswith("±"):
            v = float(t[1:]); return "", v, -v
        if "/" in t:
            a, b = t.split("/"); return "", float(a), float(b)
    return "", None, None


def add_dim(msp, d, P, scale, vbox=None, stack=None):
    kind = d.get("type", "linear")
    ov = {"dimlfac": 1 / scale, "dimdec": 2, "dimzin": 8, "dimtxsty": "KR", "dimtxt": TH}
    if kind == "linear":
        p1, p2 = P(d["p1"]), P(d["p2"])
        ang = 90 if d.get("dir", "h") == "v" else 0
        side = d.get("side")
        if side and vbox is not None:
            k = stack.get((d["view"], side), 0); stack[(d["view"], side)] = k + 1
            x0, y0, x1, y1 = vbox; o = DIM_FIRST + DIM_STEP * k
            base = {"above": (p1[0], y1 + o), "below": (p1[0], y0 - o), "right": (x1 + o, p1[1]), "left": (x0 - o, p1[1])}[side]
        else:
            off = d.get("offset", 10)
            base = ((p1[0], (max if off > 0 else min)(p1[1], p2[1]) + off) if ang == 0 else
                    ((max if off > 0 else min)(p1[0], p2[0]) + off, p1[1]))
        size = abs((p2[0] - p1[0]) if ang == 0 else (p2[1] - p1[1])) / scale
        suf, u, l = dim_tol_override(d, size)
        txt = d.get("text", ("%%c" if d.get("dia") else "") + "<>" + suf)
        if u is not None:
            ov.update({"dimtol": 1, "dimtp": u, "dimtm": -l})
        msp.add_linear_dim(base=base, p1=p1, p2=p2, angle=ang, dimstyle="ISO-A4", override=ov, text=txt,
                           dxfattribs={"layer": "DIM"}).render()
    elif kind in ("diameter", "radius"):
        c = P(d["center"]); r = d["r"] * scale
        size = 2 * d["r"] if kind == "diameter" else d["r"]
        suf, u, l = dim_tol_override(d, size)
        if u is not None:
            ov.update({"dimtol": 1, "dimtp": u, "dimtm": -l})
        pre = d.get("prefix", "")
        a = np.radians(d.get("angle", 45)); ov.update({"dimtih": 1, "dimtoh": 1, "dimtmove": 0})
        loc = (c[0] + (r + d.get("lead", 8)) * np.cos(a), c[1] + (r + d.get("lead", 8)) * np.sin(a))
        if kind == "diameter":
            msp.add_diameter_dim(center=c, radius=r, location=loc, dimstyle="ISO-A4", override=ov,
                                 text=pre + "<>" + suf, dxfattribs={"layer": "DIM"}).render()
        else:
            msp.add_radius_dim(center=c, radius=r, location=loc, dimstyle="ISO-A4", override=ov,
                               text=pre + "R<>" + suf, dxfattribs={"layer": "DIM"}).render()


# ---------------------------------------------------------------- 용지 (ISO 5457 / 7200)
def sheet_frame(msp):
    W, H = A4
    x0, y0, x1, y1 = FRAME
    rect(msp, 0, 0, W, H, "THIN")                                   # 재단선
    rect(msp, x0, y0, x1, y1, "FRAME")                              # 윤곽선 0.7
    for (a, b) in [((W / 2, 0), (W / 2, y0 + 5)), ((W / 2, H), (W / 2, y1 - 5)),   # 중심 마크
                   ((0, H / 2), (x0 + 5, H / 2)), ((W, H / 2), (x1 - 5, H / 2))]:
        msp.add_line(a, b, dxfattribs={"layer": "FRAME"})
    # 구역 표시: 가로 1~4, 세로 A~F (구역 길이 약 50mm)
    nx, ny = 4, 6
    for i in range(nx + 1):
        x = x0 + (x1 - x0) * i / nx
        for (ya, yb) in ((y1, H - 5), (y0, 5)):
            if 0 < i < nx:
                msp.add_line((x, ya), (x, yb), dxfattribs={"layer": "THIN"})
        if i < nx:
            xm = x + (x1 - x0) / nx / 2
            text(msp, str(i + 1), xm, (y1 + H) / 2, 2.5, align=TextEntityAlignment.MIDDLE_CENTER)
            text(msp, str(i + 1), xm, y0 / 2, 2.5, align=TextEntityAlignment.MIDDLE_CENTER)
    rect(msp, 5, 5, W - 5, H - 5, "THIN")
    for j in range(ny + 1):
        y = y0 + (y1 - y0) * j / ny
        if 0 < j < ny:
            msp.add_line((x0, y), (5, y), dxfattribs={"layer": "THIN"}); msp.add_line((x1, y), (W - 5, y), dxfattribs={"layer": "THIN"})
        if j < ny:
            ym = y + (y1 - y0) / ny / 2; L = "ABCDEF"[ny - 1 - j]
            text(msp, L, 12.5, ym, 2.5, align=TextEntityAlignment.MIDDLE_CENTER)
            text(msp, L, (x1 + W - 5) / 2, ym, 2.5, align=TextEntityAlignment.MIDDLE_CENTER)


def field(msp, x0, y0, x1, y1, label, value, vh=TH, align="left"):
    rect(msp, x0, y0, x1, y1, "THIN")
    text(msp, label, x0 + 1.0, y1 - 1.4, 1.8)
    if value:
        vh = max(1.8, min(vh, vh * (x1 - x0 - 2.5) / max(text_width(value, vh), 1e-6)))
        if align == "center":
            text(msp, value, (x0 + x1) / 2, y0 + (y1 - y0) * 0.38, vh, align=TextEntityAlignment.MIDDLE_CENTER)
        else:
            text(msp, value, x0 + 1.5, y0 + (y1 - y0) * 0.38, vh)


def title_block(msp, spec, scale, mass, rev_date):
    x0, y0, x1, _ = FRAME
    r = TB_H / 4
    Y = [y0 + r * i for i in range(5)]
    # ISO 7200 표제란 (아래 → 위)
    field(msp, x0, Y[3], x0 + 30, Y[4], "책임 부서", spec.get("dept", "LSK 개인 제작"))
    field(msp, x0 + 30, Y[3], x0 + 60, Y[4], "기술 참조", spec.get("tech_ref", ""))
    field(msp, x0 + 60, Y[3], x0 + 120, Y[4], "문서 종류", spec.get("doc_type", "부품도 (가공)"))
    field(msp, x0 + 120, Y[3], x1, Y[4], "문서 상태", spec.get("status", "검토 중"))
    field(msp, x0, Y[2], x0 + 30, Y[3], "작성자", "Sunggwan")
    field(msp, x0 + 30, Y[2], x0 + 60, Y[3], "승인자", spec.get("approved", ""))
    field(msp, x0 + 60, Y[1], x1, Y[3], "제목", spec["name"], vh=5.0)
    field(msp, x0, Y[1], x0 + 60, Y[2], "법적 소유자", "Sunggwan")
    field(msp, x0, Y[0], x0 + 60, Y[1], "부제 (조립품)", spec.get("assembly", "LSK H/SEQ Shifter"))
    field(msp, x0 + 60, Y[0], x0 + 110, Y[1], "식별 번호", spec["part_no"], vh=3.0)
    field(msp, x0 + 110, Y[0], x0 + 128, Y[1], "개정", spec.get("rev", "A"), align="center")
    field(msp, x0 + 128, Y[0], x0 + 156, Y[1], "발행일", str(rev_date), align="center")
    field(msp, x0 + 156, Y[0], x0 + 166, Y[1], "언어", "ko", align="center")
    field(msp, x0 + 166, Y[0], x1, Y[1], "시트", "1/1", align="center")
    rect(msp, x0, y0, x1, Y[4], "TB")
    # 추가 정보란
    ya, yb = Y[4], Y[4] + ADD_H
    sc = f"{scale:g}:1" if scale >= 1 else f"1:{1 / scale:g}"
    cols = [(30, "재질", spec.get("material", "")), (34, "소재", spec.get("stock", "")),
            (18, "질량", f"{mass:.0f} g" if mass >= 10 else f"{mass:.1f} g"), (14, "척도", sc), (20, "투상법", ""),
            (30, "일반 공차", spec.get("general_tolerance", "ISO 2768-mK")), (22, "공차 원칙", "ISO 8015"), (12, "단위", "mm")]
    x = x0
    for w, lab, val in cols:
        field(msp, x, ya, x + w, yb, lab, val, vh=2.5)
        if lab == "투상법":
            third_angle_symbol(msp, x + 3.5, ya + 4.0, 0.62)
        x += w
    rect(msp, x0, ya, x1, yb, "TB")


def revision_table(msp, revs):
    x0, x1, y1 = REV_X0, FRAME[2], FRAME[3]
    cols = [("개정", 10), ("내용", 38), ("날짜", 22), ("작성", 20)]
    rh = 5.0
    rows = [[c for c, _ in cols]] + [[r.get("rev", ""), r.get("desc", ""), str(r.get("date", "")), r.get("by", "Sunggwan")] for r in revs]
    for i, row in enumerate(rows):
        yt = y1 - rh * i; x = x0
        for (lab, w), v in zip(cols, row):
            rect(msp, x, yt - rh, x + w, yt, "THIN")
            h = 1.8 if i == 0 else max(1.8, min(2.5, 2.5 * (w - 2) / max(text_width(str(v), 2.5), 1e-6)))
            text(msp, str(v), x + w / 2, yt - rh / 2, h, align=TextEntityAlignment.MIDDLE_CENTER)
            x += w
    rect(msp, x0, y1 - rh * len(rows), x1, y1, "TB")
    return rh * len(rows)


# ---------------------------------------------------------------- 주기
def build_notes(spec):
    n = [f"공정: {spec.get('process', '')}",
         f"지시 없는 모서리: {spec.get('edges', 'ISO 13715 — 바깥 −0.3 / 안쪽 +0.3, 바리 없을 것')}",
         "끼워맞춤 ISO 286-1/2, 기하공차 ISO 1101, 데이텀 ISO 5459"]
    fin = spec.get("finish", [])
    if fin:
        n.append("후처리 (순서대로): " + " → ".join(f"{chr(0x2460 + i)} {f}" for i, f in enumerate(fin)))
    n += spec.get("notes", [])
    return [f"{i + 1}. {t}" for i, t in enumerate(n)]


def layout_notes(notes, width, h=TH):
    lines = []
    for t in notes:
        lines += wrap_text(t, width, h, "    ")
    return lines


# ---------------------------------------------------------------- 메인
def make_drawing(spec, shape, outdir, rev_date=None):
    rev_date = rev_date or datetime.date.today()
    doc = setup_doc(); msp = doc.modelspace()
    sheet_frame(msp)
    views = spec.get("views", ["front", "top", "right"])
    proj = {v: project(shape, v) for v in views}
    sz = {v: (b[2] - b[0], b[3] - b[1]) for v, (_, _, b) in proj.items()}
    fw, fh = sz.get("front", (0, 0)); tw, th = sz.get("top", (0, 0)); rw, rh = sz.get("right", (0, 0))
    dims = spec.get("dims", [])
    cnt = lambda v, sd: sum(1 for d in dims if d.get("view") == v and d.get("side") == sd)
    auto = 1 if spec.get("auto_overall", True) else 0
    depth_view = "right" if "right" in views else ("top" if "top" in views else None)
    band = lambda k: (DIM_FIRST + DIM_STEP * (k - 1) + 4) if k else 3
    ml = band(max(cnt("front", "left") + auto, cnt("top", "left") + (auto if depth_view == "top" else 0)))
    mb = band(cnt("front", "below") + auto)
    mr = band(cnt("right", "right") if rw else max(cnt("front", "right"), cnt("top", "right")))
    mt = band(cnt("top", "above") + (auto if depth_view == "top" else 0)) + 2
    gap_h = band(cnt("front", "right") + cnt("right", "left")) + 6 if rw else 0
    gap_v = band(cnt("front", "above") + cnt("top", "below")) + 6 if th else 0
    if rw and not th:
        mt = max(mt, band(cnt("right", "above") + (auto if depth_view == "right" else 0)) + 2)

    notes = build_notes(spec)
    revs = spec.get("revisions") or [{"rev": spec.get("rev", "A"), "date": str(rev_date), "desc": "최초 발행", "by": "Sunggwan"}]
    rev_h = 5.0 * (len(revs) + 1)
    lh = TH * 1.5
    # 배치 후보: L1 주기 아래 전체 폭 / L2 주기 오른쪽 칼럼
    cands = []
    w1 = FRAME[2] - FRAME[0] - 6
    n1 = layout_notes(notes, w1)
    nh1 = len(n1) * lh + 10
    area1 = (FRAME[0], TB_TOP + nh1, FRAME[2], FRAME[3] - rev_h - 2)
    w2 = FRAME[2] - REV_X0 - 4
    n2 = layout_notes(notes, w2)
    nh2 = len(n2) * lh + 10
    area2 = (FRAME[0], TB_TOP, REV_X0 - 2, FRAME[3])
    area1b = (FRAME[0], TB_TOP + nh1, FRAME[2], FRAME[3] - 2)
    for name, area, lines in (("L1", area1, n1), ("L1b", area1b, n1), ("L2", area2, n2)):
        if name == "L2" and TB_TOP + nh2 > FRAME[3] - rev_h - 4:
            continue
        AW = area[2] - area[0] - ml - mr - gap_h
        AH = area[3] - area[1] - mb - mt - gap_v
        s = spec.get("scale") or next((s_ for s_ in STD_SCALES if (fw + rw) * s_ <= AW and (fh + th) * s_ <= AH), STD_SCALES[-1])
        fill = min(AW / max((fw + rw) * s, 1e-6), AH / max((fh + th) * s, 1e-6))
        yoff = max(0, (AH - (fh + th) * s) / 2)
        if name == "L1b":     # 위로 올린 배치: 뷰 묶음(치수 포함)이 개정 이력표와 겹치면 아래로 붙이고, 그래도 겹치면 제외
            gx0 = area[0] + max(0, (AW - (fw + rw) * s) / 2)
            gx1 = gx0 + ml + (fw + rw) * s + gap_h + mr
            top = lambda yo: area[1] + mb + yo + (fh + th) * s + gap_v + mt
            if gx1 > REV_X0 and top(yoff) > FRAME[3] - rev_h - 2:
                yoff = max(0, FRAME[3] - rev_h - 2 - top(0))
                if top(yoff) > FRAME[3] - rev_h - 2:
                    continue
        cands.append((s, -fill if name == "L2" else -fill - 0.3, name, area, lines, AW, AH, yoff))
    cands.sort(key=lambda c: (-c[0], c[1]))
    scale, _, lay, area, nlines, AW, AH, yoff = cands[0]

    x0 = area[0] + ml + max(0, (AW - (fw + rw) * scale) / 2)
    y0 = area[1] + mb + yoff
    origin = {"front": (x0, y0), "top": (x0, y0 + fh * scale + gap_v), "right": (x0 + fw * scale + gap_h, y0)}
    mappers, vboxes, stack = {}, {}, {}
    ov = {"dimlfac": 1 / scale, "dimdec": 1, "dimzin": 8, "dimtxsty": "KR", "dimtxt": TH}
    for v in views:
        vis, hid, b = proj[v]
        ox, oy = origin[v]
        T = (lambda b, ox, oy: (lambda q: (ox + (q[0] - b[0]) * scale, oy + (q[1] - b[1]) * scale)))(b, ox, oy)
        mapf = VIEWS[v][0]
        mappers[v] = (lambda T, mapf: (lambda p3: T(mapf(p3))))(T, mapf)
        if v in spec.get("hidden_views", ["front"]):
            for g in hid:
                draw_geom(msp, g, T, scale, "HIDDEN")
        for g in vis:
            draw_geom(msp, g, T, scale, "VISIBLE")
        # 중심 마크 (원 → 중심선 십자, 원 밖 2.5mm)
        cen = {}
        for g in vis + hid:
            if g[0] == "circle" or (g[0] == "arc" and (g[4] - g[3]) % 360 >= 180):
                k = (round(g[1][0], 2), round(g[1][1], 2)); cen[k] = max(cen.get(k, 0), g[2])
        for (cx, cy), r in cen.items():
            if r * scale < 1.0:
                continue
            X, Y = T((cx, cy)); e = r * scale + 2.5
            msp.add_line((X - e, Y), (X + e, Y), dxfattribs={"layer": "CENTER"})
            msp.add_line((X, Y - e), (X, Y + e), dxfattribs={"layer": "CENTER"})
        bx0, by0 = T((b[0], b[1])); bx1, by1 = T((b[2], b[3]))
        vboxes[v] = (bx0, by0, bx1, by1)
        def dup(length, direction):     # 같은 길이를 스펙 치수가 이미 지시하면 전체 치수 생략 (ISO 129: 한 번만)
            for d in dims:
                if d.get("view") == v and d.get("type", "linear") == "linear" and d.get("dir", "h") == direction:
                    a_, b_ = mapf(d["p1"]), mapf(d["p2"]); k = 0 if direction == "h" else 1
                    if abs(abs(a_[k] - b_[k]) - length) < 0.01:
                        return True
            return False
        if auto:          # 전체 치수: 정면도 가로·세로, 깊이는 한 번
            o = lambda side: DIM_FIRST + DIM_STEP * cnt(v, side)
            if v == "front":
                if not dup(b[2] - b[0], "h"):
                    msp.add_linear_dim(base=(bx0, by0 - o("below")), p1=(bx0, by0), p2=(bx1, by0), angle=0, dimstyle="ISO-A4",
                                       override=ov, dxfattribs={"layer": "DIM"}).render()
                if not dup(b[3] - b[1], "v"):
                    msp.add_linear_dim(base=(bx0 - o("left"), by0), p1=(bx0, by0), p2=(bx0, by1), angle=90, dimstyle="ISO-A4",
                                       override=ov, dxfattribs={"layer": "DIM"}).render()
            elif v == depth_view == "right":
                if not dup(b[2] - b[0], "h"):
                    msp.add_linear_dim(base=(bx0, by1 + o("above")), p1=(bx0, by1), p2=(bx1, by1), angle=0, dimstyle="ISO-A4",
                                       override=ov, dxfattribs={"layer": "DIM"}).render()
            elif v == depth_view == "top" and not dup(b[3] - b[1], "v"):
                msp.add_linear_dim(base=(bx0 - o("left"), by0), p1=(bx0, by0), p2=(bx0, by1), angle=90, dimstyle="ISO-A4",
                                   override=ov, dxfattribs={"layer": "DIM"}).render()

    P = lambda view: mappers[view]
    for d in dims:
        try:
            add_dim(msp, d, P(d["view"]), scale, vboxes.get(d["view"]), stack)
        except Exception as ex:
            print("  치수 건너뜀:", d, ex)
    for k, dd in (spec.get("datums") or {}).items():
        t = P(dd["view"])(dd["at"])
        datum_symbol(msp, k, t, (t[0] + dd.get("dx", -10), t[1] + dd.get("dy", -9)))
    for g in spec.get("gdt", []):
        t = P(g["view"])(g["at"])
        gdt_frame(msp, t[0] + g.get("dx", 14), t[1] + g.get("dy", 10), g["symbol"], g["tol"], g.get("datums", []), target=t)
    for r in spec.get("roughness", []):
        t = P(r["view"])(r["at"])
        roughness(msp, t[0] + r.get("dx", 5), t[1] + r.get("dy", 5), r["value"], leader_to=t)

    # 주기
    if lay in ("L1", "L1b"):
        nx, ntop = FRAME[0] + 3, TB_TOP + len(nlines) * lh + 7
        msp.add_line((FRAME[0], TB_TOP + len(nlines) * lh + 10), (FRAME[2], TB_TOP + len(nlines) * lh + 10), dxfattribs={"layer": "THIN"})
    else:
        nx, ntop = REV_X0 + 2, FRAME[3] - rev_h - 6
        msp.add_line((REV_X0, TB_TOP), (REV_X0, FRAME[3] - rev_h), dxfattribs={"layer": "THIN"})
    text(msp, "주기", nx, ntop, 3.5)
    roughness(msp, nx + 14, ntop - 1.5, spec.get("roughness_default", "Ra 3.2"), general=True)
    for i, ln in enumerate(nlines):
        text(msp, ln, nx, ntop - 6.5 - lh * i, TH)

    mat = spec.get("material", "Al 6061-T6")
    mass = shape.volume / 1000 * DENSITY.get(mat, 2.70) * spec.get("qty_per_part", 1)
    title_block(msp, spec, scale, mass, rev_date)
    revision_table(msp, revs)
    os.makedirs(outdir, exist_ok=True)
    fn = os.path.join(outdir, f"{spec['part_no']}_{spec['part']}.dxf")
    doc.saveas(fn)
    return fn, scale


def preview(dxf_path, out_png=None, out_pdf=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from ezdxf.addons.drawing import RenderContext, Frontend
    from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
    from ezdxf.addons.drawing.config import Configuration, LineweightPolicy, BackgroundPolicy, ColorPolicy, LinePolicy
    doc = ezdxf.readfile(dxf_path)
    doc.styles.get("KR").dxf.font = FONT_PREVIEW
    fig = plt.figure(figsize=(A4[0] / 25.4, A4[1] / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    cfg = Configuration(lineweight_policy=LineweightPolicy.ABSOLUTE, background_policy=BackgroundPolicy.WHITE,
                        color_policy=ColorPolicy.BLACK, line_policy=LinePolicy.ACCURATE)
    Frontend(RenderContext(doc), MatplotlibBackend(ax), config=cfg).draw_layout(doc.modelspace(), finalize=True)
    ax.set_xlim(0, A4[0]); ax.set_ylim(0, A4[1]); ax.set_aspect("equal"); ax.axis("off")
    if out_png:
        fig.savefig(out_png, dpi=200, facecolor="white")
    if out_pdf:
        fig.savefig(out_pdf, facecolor="white")
    plt.close(fig)
