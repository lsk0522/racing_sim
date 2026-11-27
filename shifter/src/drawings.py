# LSK H/SEQ Shifter — author: Sunggwan
"""
도면 생성기: 부품 STEP + 도면 스펙 YAML → A4 가로 DXF (+ PDF/PNG 미리보기), 흑백

적용 규격
 - 용지: A4 가로, 윤곽선(사방 10mm) + 중심 마크, 아래에 저작권 문구·용지 크기
 - 양식 (실무 도면 형식): 오른쪽 아래 표제란 = 부품 행(NO·DESCRIPTION·MATERIAL·QTY·SIZE·REMARKS)
   + 가공 기호표(CLASS OF FINISH) + 일반 공차표(ISO 2768-m) + 로고·결재(DESIGNED·CHECKED·APPROVED)·제목
   + 단위·척도·질량·투상법·도번 / 왼쪽 아래 개정란(△) / 오른쪽 위 표면 거칠기 일괄 지시 (ISO 1302)
 - 구멍·탭 지시: 화살표 지시선 + 밑줄 글자 (예: 2-M3 탭 깊이 8)
 - 투상도 배치: 표제란·개정란·주서를 피해서 가장 큰 척도로 자동 배치
 - 선: ISO 128-20 선 그룹 0.35 (외형선 0.35 / 가는 선 0.18, 숨은선 02, 중심선 04), 색은 모두 7 (흑백)
 - 문자: ISO 3098 / KS (치수 3.5, 주서 3.0, 제목 5)
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

A4 = (297.0, 210.0)                            # A4 가로 (기본)
FRAME = (10.0, 10.0, 287.0, 200.0)             # 윤곽선 (사방 10mm)
TH = 3.5                                       # 치수 문자 (KS·ISO 3098 3.5)
TN = 3.0                                       # 주서 문자
STD_SCALES = [5, 2, 1, 0.5, 0.2, 0.1]          # ISO 5455
DIM_FIRST, DIM_STEP = 10.0, 8.0                # 외형선 → 첫 치수선, 치수선 간격
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
    # 흑백: 모든 레이어 색 7 (흰 바탕 = 검정 출력), 굵기·선 종류로만 구분
    for name, color, lt, lw in [("VISIBLE", 7, "Continuous", 35), ("HIDDEN", 7, "ISO02", 18), ("CENTER", 7, "ISO04", 18),
                                ("DIM", 7, "Continuous", 18), ("GDT", 7, "Continuous", 18), ("TEXT", 7, "Continuous", 18),
                                ("FRAME", 7, "Continuous", 70), ("TB", 7, "Continuous", 50), ("THIN", 7, "Continuous", 18)]:
        doc.layers.add(name, color=color, linetype=lt, lineweight=lw)
    ds = doc.dimstyles.new("ISO-A4", dxfattribs={
        "dimtxsty": "KR", "dimtxt": TH, "dimasz": 3.0, "dimexe": 2.0, "dimexo": 1.0, "dimgap": 0.8,
        "dimtad": 1, "dimtih": 0, "dimtoh": 0, "dimdec": 2, "dimdsep": ord("."), "dimlunit": 2, "dimzin": 8,
        "dimscale": 1.0, "dimclrd": 7, "dimclre": 7, "dimclrt": 7, "dimtdec": 3, "dimtzin": 8, "dimtfac": 0.75,
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


def arrow(msp, tip, frm, layer="GDT", size=3.0):
    d = np.array(tip, float) - np.array(frm, float); L = np.linalg.norm(d)
    if L < 1e-6:
        return
    d /= L; n = np.array([-d[1], d[0]])
    b = np.array(tip) - d * size
    pts = [tuple(tip), tuple(b + n * size * 0.17), tuple(b - n * size * 0.17)]     # 약 20° 닫힌 화살표
    h = msp.add_hatch(dxfattribs={"layer": layer}); h.paths.add_polyline_path(pts, is_closed=True)


# ---------------------------------------------------------------- 기호 (ISO 1101 / 5459 / 1302 / 5456-2)
def gdt_symbol(msp, kind, cx, cy, s=2.0, layer="GDT"):
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


# ---------------------------------------------------------------- 용지 (윤곽선 + 중심 마크)
def sheet_frame(msp):
    W, H = A4
    x0, y0, x1, y1 = FRAME
    rect(msp, x0, y0, x1, y1, "FRAME")                              # 윤곽선 0.7
    for (a, b) in [((W / 2, 0), (W / 2, y0)), ((W / 2, H), (W / 2, y1 - 5)),   # 중심 마크 (아래는 표제란이 있어 윤곽선까지)
                   ((0, H / 2), (x0 + 5, H / 2)), ((W, H / 2), (x1 - 5, H / 2))]:
        msp.add_line(a, b, dxfattribs={"layer": "FRAME"})
    text(msp, "이 도면의 저작권은 Sunggwan에게 있으며, 허락 없이 복제·전달·사용할 수 없습니다.", x0 + 2, y0 / 2, 1.8)
    text(msp, f"A4 ({W:g}×{H:g})", x1, y0 / 2, 1.8, align=TextEntityAlignment.MIDDLE_RIGHT)


def cell(msp, x0, y0, x1, y1, value="", vh=TN, align="center", label="", lh=1.5):
    """칸 하나: 선택적 작은 라벨(왼쪽 위) + 값 (칸 폭에 맞게 글자 자동 축소)."""
    rect(msp, x0, y0, x1, y1, "THIN")
    if label:
        text(msp, label, x0 + 0.8, y1 - lh * 0.8, lh)
    if value:
        vh = max(1.5, min(vh, vh * (x1 - x0 - 1.6) / max(text_width(value, vh), 1e-6)))
        yy = y0 + (y1 - y0) * (0.4 if label else 0.5)
        if align == "center":
            text(msp, value, (x0 + x1) / 2, yy, vh, align=TextEntityAlignment.MIDDLE_CENTER)
        else:
            text(msp, value, x0 + 1.2, yy, vh)


def mini_finish(msp, x, y, k=1.0, removed=True):
    """표 안의 작은 거칠기 기호 (ISO 1302). removed=False → 제거 가공 금지(원)."""
    a = 1.6 * k
    msp.add_lwpolyline([(x - a * 0.58, y + a), (x, y), (x + 2 * a * 0.58, y + 2 * a)], dxfattribs={"layer": "THIN"})
    if removed:
        msp.add_line((x - a * 0.58, y + a), (x + a * 0.58, y + a), dxfattribs={"layer": "THIN"})
    else:
        msp.add_circle((x, y + a * 0.55), a * 0.45, dxfattribs={"layer": "THIN"})


def finish_short(spec):
    import re
    for f in spec.get("finish", []):
        if "아노다이징" in f:
            m = re.search(r"((?:하드|블랙|투명|그래파이트)\s*)?아노다이징[^—(,→]*?(?=\s*(?:또는|$|—|\(|,|→))", f)
            if m:
                return m.group(0).strip()
    return spec.get("remarks", "—")


GT_ROWS = [("0.5 ~ 6", "±0.1"), ("6 ~ 30", "±0.2"), ("30 ~ 120", "±0.3"), ("120 ~ 400", "±0.5"),
           ("400 ~ 1000", "±0.8"), ("1000 ~ 2000", "±1.2")]                      # ISO 2768-1 m
FIN_ROWS = [(False, "소재면"), (True, "Ra 12.5"), (True, "Ra 6.3"), (True, "Ra 3.2"), (True, "Ra 1.6"), (True, "Ra 0.8")]
TB_W, TB_H = 175.0, 46.0
TB_X0 = FRAME[2] - TB_W
REV_W = 90.0


def title_block(msp, spec, scale, mass, rev_date):
    """오른쪽 아래 표제란 (사진 양식): 부품 행 + 가공 기호표 + 일반 공차표 + 로고·결재·제목 + 단위·척도·투상·도번."""
    X0, X1, Y0 = TB_X0, FRAME[2], FRAME[1]
    Y1 = Y0 + TB_H
    # ① 부품 행 (값 위, 라벨 아래)
    yl, yv = Y0 + 35.0, Y0 + 39.0
    cols = [(18, "NO", spec["part_no"].replace("LSK-SH-", "")), (44, "DESCRIPTION", spec["name"]),
            (26, "MATERIAL", spec.get("material", "")), (10, "QTY", str(spec.get("qty", 1))),
            (37, "SIZE", spec.get("stock", "")), (40, "REMARKS", finish_short(spec))]
    x = X0
    for w, lab, val in cols:
        cell(msp, x, yv, x + w, Y1, val, 3.0)
        cell(msp, x, yl, x + w, yv, lab, 1.6)
        x += w
    # ② 가공 기호표 (CLASS OF FINISH, ISO 1302 / 21920)
    rh = (yl - Y0) / 7
    xa, xb = X0, X0 + 26
    cell(msp, xa, yl - rh, xb, yl, "CLASS OF FINISH", 1.6)
    for i, (rem, val) in enumerate(FIN_ROWS):
        y = yl - rh * (i + 2)
        rect(msp, xa, y, xa + 9, y + rh, "THIN"); mini_finish(msp, xa + 3.6, y + 0.6, 0.62, rem)
        cell(msp, xa + 9, y, xb, y + rh, val, 2.0)
    # ③ 일반 공차표 (ISO 2768-1 m)
    xc = xb + 36
    cell(msp, xb, yl - rh, xc, yl, "GENERAL TOL. ISO 2768-m", 1.6)
    for i, (rg, tol) in enumerate(GT_ROWS):
        y = yl - rh * (i + 2)
        cell(msp, xb, y, xb + 22, y + rh, rg, 2.0); cell(msp, xb + 22, y, xc, y + rh, tol, 2.0)
    # ④ 로고 · 결재 · 제목
    yA = Y0 + 11.0
    cell(msp, xc, yA, xc + 30, yl, "")
    text(msp, "SUNGGWAN", xc + 15, yA + 15, 3.5, align=TextEntityAlignment.MIDDLE_CENTER)
    text(msp, "LSK Racing Sim", xc + 15, yA + 8, 2.0, align=TextEntityAlignment.MIDDLE_CENTER)
    msp.add_line((xc + 4, yA + 11.6), (xc + 26, yA + 11.6), dxfattribs={"layer": "THIN"})
    xd = xc + 30
    for i, (lab, who) in enumerate((("DESIGNED", "Sunggwan"), ("CHECKED", spec.get("checked", "")), ("APPROVED", spec.get("approved", "")))):
        cell(msp, xd + 12 * i, yl - 4, xd + 12 * (i + 1), yl, lab, 1.4)
        cell(msp, xd + 12 * i, yA, xd + 12 * (i + 1), yl - 4, who, 1.9)
    xe = xd + 36
    cell(msp, xe, yA, X1, yl, "", label="TITLE")
    def fit(s, w, h):
        return max(1.8, min(h, h * w / max(text_width(s, h), 1e-6)))
    t1, t2 = spec.get("title", spec["name"]), spec.get("assembly", "LSK H/SEQ SHIFTER")
    text(msp, t1, (xe + X1) / 2, yA + 14.5, fit(t1, X1 - xe - 4, 4.5), align=TextEntityAlignment.MIDDLE_CENTER)
    text(msp, t2, (xe + X1) / 2, yA + 6.0, fit(t2, X1 - xe - 4, 3.0), align=TextEntityAlignment.MIDDLE_CENTER)
    # ⑤ 단위 · 척도 · 질량 · 투상 · 도번
    sc = f"{scale:g}/1" if scale >= 1 else f"1/{1 / scale:g}"
    m = f"{mass:.0f} g" if mass >= 10 else f"{mass:.1f} g"
    x = xc
    for w, lab, val in ((15, "UNIT", "MM"), (17, "SCALE", sc), (18, "MASS", m)):
        cell(msp, x, Y0, x + w, yA, val, 2.8, label=lab); x += w
    cell(msp, x, Y0, x + 16, yA, "", label="PROJ.")
    third_angle_symbol(msp, x + 4.0, Y0 + 4.6, 0.48); x += 16
    cell(msp, x, Y0, X1, yA, spec["part_no"] + f"  ({spec.get('rev', 'A')})", 3.5, label="DWG NO.")
    msp.add_line((X0, yl), (X1, yl), dxfattribs={"layer": "THIN"})
    rect(msp, X0, Y0, X1, Y1, "TB")
    return (X0, Y0, X1, Y1)


def revision_block(msp, revs):
    """왼쪽 아래 개정란 (사진 양식): 맨 아래 머리줄, 위로 △번호 이력."""
    X0, Y0 = FRAME[0], FRAME[1]
    cols = [("NO", 10), ("REVISION", 46), ("DATE", 18), ("APPROVED", 16)]
    rh = 5.5
    x = X0
    for lab, w in cols:
        cell(msp, x, Y0, x + w, Y0 + rh, lab, 1.8); x += w
    for i, r in enumerate(revs):
        y = Y0 + rh * (i + 1); x = X0
        vals = ["", r.get("desc", ""), str(r.get("date", "")), r.get("by", "Sunggwan")]
        for (lab, w), v in zip(cols, vals):
            cell(msp, x, y, x + w, y + rh, v, 2.2); x += w
        cx, cy = X0 + 5, y + rh / 2 - 0.4          # △ + 개정 기호
        msp.add_lwpolyline([(cx - 2.6, cy - 1.7), (cx + 2.6, cy - 1.7), (cx, cy + 2.6), (cx - 2.6, cy - 1.7)], dxfattribs={"layer": "THIN"})
        text(msp, r.get("rev", ""), cx, cy - 0.35, 1.8, align=TextEntityAlignment.MIDDLE_CENTER)
    top = Y0 + rh * (len(revs) + 1)
    rect(msp, X0, Y0, X0 + REV_W, top, "TB")
    return (X0, Y0, X0 + REV_W, top)


# ---------------------------------------------------------------- 주서 / 거칠기 일괄 기호 / 구멍 지시
def build_notes(spec):
    n = [f"공정: {spec.get('process', '')}",
         f"지시 없는 모서리: {spec.get('edges', 'ISO 13715 — 바깥 −0.3 / 안쪽 +0.3, 바리 없을 것')}",
         f"일반 공차 {spec.get('general_tolerance', 'ISO 2768-mK')} (표제란 표), ISO 8015, 끼워맞춤 ISO 286, 기하공차 ISO 1101"]
    fin = spec.get("finish", [])
    if fin:
        n.append("후처리: " + " → ".join(f"{chr(0x2460 + i)} {f}" for i, f in enumerate(fin)))
    n += spec.get("notes", [])
    return [f"{i + 1}. {t}" for i, t in enumerate(n)]


def layout_notes(notes, width, h=TN):
    lines = []
    for t in notes:
        lines += wrap_text(t, width, h, "    ")
    return lines


def general_roughness(msp, spec):
    """오른쪽 위: √Ra 3.2 (√Ra 1.6, √Ra 0.8) — ISO 1302 간략 지시. 반환: 차지하는 사각형."""
    gen = spec.get("roughness_default", "Ra 3.2")
    vals = sorted({r["value"] for r in spec.get("roughness", [])} - {gen}, key=lambda v: float(v.split()[-1]))
    w1 = lambda v: 3.0 * TH * 0.58 + text_width(v, TH) + 6
    total = w1(gen) + (sum(w1(v) for v in vals) + 6 if vals else 0)
    x = FRAME[2] - 4 - total; y = FRAME[3] - 13
    roughness(msp, x + 2, y, gen); xs = x + 2 + w1(gen)
    if vals:
        text(msp, "(", xs - 2, y + 4, 6.0, align=TextEntityAlignment.MIDDLE_CENTER); xs += 2
        for v in vals:
            roughness(msp, xs + 2, y, v); xs += w1(v)
        text(msp, ")", xs - 3, y + 4, 6.0, align=TextEntityAlignment.MIDDLE_CENTER)
    return (x - 2, y - 2, FRAME[2], FRAME[3])


def callout(msp, target, at, lines):
    """구멍·탭 지시 (사진 양식): 화살표 지시선 → 밑줄 친 글자. 예) 2-M3 탭 깊이 8"""
    lines = [lines] if isinstance(lines, str) else lines
    w = max(text_width(s, TH) for s in lines) + 2
    left = at[0] < target[0]
    x0 = at[0] - w if left else at[0]
    msp.add_lwpolyline([target, at], dxfattribs={"layer": "DIM"})
    arrow(msp, target, at, "DIM", 3.0)
    msp.add_line((x0, at[1]), (x0 + w, at[1]), dxfattribs={"layer": "DIM"})
    for i, s in enumerate(lines):
        text(msp, s, x0 + 1, at[1] + 1.2 + TH * 0.5 + (TH * 1.5) * (len(lines) - 1 - i), TH, "DIM")


# ---------------------------------------------------------------- 배치 (장애물 피해서 가장 큰 척도)
def _overlap(a, b, pad=0.0):
    return not (a[2] + pad <= b[0] or b[2] + pad <= a[0] or a[3] + pad <= b[1] or b[3] + pad <= a[1])


def place_views(sizes, margins, obstacles, scales, inner, ann_fn=None):
    """sizes = (fw, fh, tw, th, rw, rh), margins dict, ann_fn(s) → 주석 사각형(정면도 원점 기준) → (scale, x0, y0) or None"""
    fw, fh, tw, th, rw, rh = sizes
    M = margins
    ix0, iy0, ix1, iy1 = inner
    cx_ref, cy_ref = (ix0 + ix1) / 2, (iy0 + iy1) / 2 + 8
    for s in scales:
        best = None
        def rects(x0, y0):
            R = [(x0 - M["ml"], y0 - M["mb"], x0 + fw * s + (M["gap_h"] / 2 if rw else M["mr"]),
                  y0 + fh * s + (M["gap_v"] / 2 if th else M["mt_f"]))]
            if th:
                R.append((x0 - M["ml"], y0 + fh * s + M["gap_v"] / 2, x0 + tw * s + (M["mr_t"] if not rw else M["gap_h"] / 2),
                          y0 + fh * s + M["gap_v"] + th * s + M["mt"]))
            if rw:
                R.append((x0 + fw * s + M["gap_h"] / 2, y0 - M["mb_r"], x0 + fw * s + M["gap_h"] + rw * s + M["mr"],
                          y0 + rh * s + M["mt_r"]))
            return R
        A = ann_fn(s) if ann_fn else []
        for x0 in np.arange(ix0, ix1, 1.5):
            for y0 in np.arange(iy0, iy1, 1.5):
                R = rects(x0, y0) + [(a[0] + x0, a[1] + y0, a[2] + x0, a[3] + y0) for a in A]
                if any(r[0] < ix0 or r[1] < iy0 or r[2] > ix1 or r[3] > iy1 for r in R):
                    continue
                if any(_overlap(r, o, 2.0) for r in R for o in obstacles):
                    continue
                gx0 = min(r[0] for r in R); gx1 = max(r[2] for r in R)
                gy0 = min(r[1] for r in R); gy1 = max(r[3] for r in R)
                d = ((gx0 + gx1) / 2 - cx_ref) ** 2 + ((gy0 + gy1) / 2 - cy_ref) ** 2
                if best is None or d < best[0]:
                    best = (d, x0, y0)
        if best:
            return s, best[1], best[2]
    return None


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
    band = lambda k: (DIM_FIRST + DIM_STEP * (k - 1) + 5) if k else 4
    M = dict(
        ml=band(max(cnt("front", "left") + auto, cnt("top", "left") + (auto if depth_view == "top" else 0))),
        mb=band(cnt("front", "below") + auto),
        mr=band(cnt("right", "right") if rw else cnt("front", "right")),
        mr_t=band(cnt("top", "right")),
        mt=band(cnt("top", "above")) + 2,
        mt_f=band(cnt("front", "above")) + 2,
        mt_r=band(cnt("right", "above") + (auto if depth_view == "right" else 0)) + 2,
        mb_r=band(cnt("right", "below")),
        gap_h=(band(cnt("front", "right") + cnt("right", "left")) + 4) if rw else 0,
        gap_v=(band(cnt("front", "above") + cnt("top", "below")) + 4) if th else 0)

    k = 1.0                                           # 스펙 dx/dy = 용지 위 mm

    def ann_rects(s):
        """치수 밖 주석(기하공차틀·데이텀·거칠기·구멍 지시·지름 치수 글자)의 사각형 — 정면도 왼쪽 아래 = (0, 0)"""
        org = {"front": (0, 0), "top": (0, fh * s + M["gap_v"]), "right": (fw * s + M["gap_h"], 0)}
        def P0(view, p3):
            b = proj[view][2]; u, w = VIEWS[view][0](p3); o = org[view]
            return (o[0] + (u - b[0]) * s, o[1] + (w - b[1]) * s)
        R = []
        for g in spec.get("gdt", []):
            t = P0(g["view"], g["at"]); x, y = t[0] + g.get("dx", 14) * k, t[1] + g.get("dy", 10) * k
            w = 2 * TH + max(2 * TH * 1.6, text_width(g["tol"], TH) + 2.5) + 2 * TH * len(g.get("datums", []))
            R.append((x, y, x + w, y + 2 * TH))
        for dd in (spec.get("datums") or {}).values():
            t = P0(dd["view"], dd["at"]); x, y = t[0] + dd.get("dx", -10) * k, t[1] + dd.get("dy", -9) * k
            R.append((x - TH, y - TH, x + TH, y + TH))
        for r in spec.get("roughness", []):
            t = P0(r["view"], r["at"]); x, y = t[0] + r.get("dx", 5) * k, t[1] + r.get("dy", 5) * k
            R.append((x - 2, y, x + 3 * TH * 0.58 + text_width(r["value"], TH) + 2, y + 3 * TH + 1))
        for c in spec.get("callouts", []):
            t = P0(c["view"], c["at"]); ax, ay = t[0] + c.get("dx", 15), t[1] + c.get("dy", 12)
            L = [c["text"]] if isinstance(c["text"], str) else c["text"]
            w = max(text_width(q, TH) for q in L) + 2
            x0 = ax - w if ax < t[0] else ax
            R.append((x0, ay - 1, x0 + w, ay + 1.5 + TH * 1.5 * len(L)))
        for d in spec.get("dims", []):
            if d.get("type") in ("diameter", "radius"):
                t = P0(d["view"], d["center"]); a = math.radians(d.get("angle", 45)); r = d["r"] * s + d.get("lead", 8)
                x, y = t[0] + r * math.cos(a), t[1] + r * math.sin(a)
                w = text_width("Ø00.0 " + d.get("fit", ""), TH) + (10 if ("fit" in d or "tol" in d) else 0)
                R.append((x - w if math.cos(a) < -0.2 else x, y - 1, x + w if math.cos(a) >= -0.2 else x, y + TH + 2))
        return R

    revs = spec.get("revisions") or [{"rev": spec.get("rev", "A"), "date": str(rev_date), "desc": "최초 발행", "by": "Sunggwan"}]
    mat = spec.get("material", "Al 6061-T6")
    mass = shape.volume / 1000 * DENSITY.get(mat, 2.70)
    # 고정 요소 (위치가 정해진 것) — 크기만 미리 계산
    tb = (TB_X0, FRAME[1], FRAME[2], FRAME[1] + TB_H)
    rv = (FRAME[0], FRAME[1], FRAME[0] + REV_W, FRAME[1] + 5.5 * (len(revs) + 1))
    gen = spec.get("roughness_default", "Ra 3.2")
    vals = {r["value"] for r in spec.get("roughness", [])} - {gen}
    rg_w = (3.0 * TH * 0.58 + text_width(gen, TH) + 6) * (1 + len(vals)) + (6 if vals else 0)
    rg = (FRAME[2] - 6 - rg_w, FRAME[3] - 15, FRAME[2], FRAME[3])
    notes = build_notes(spec)
    lh = TN * 1.5
    # 주서 위치 후보: (a) 표제란 위 넓게 (b) 개정란 위 왼쪽 (c) 왼쪽 위
    cands = []
    for name in ("over_tb", "over_rev", "top_left"):
        if name == "over_tb":
            w = TB_W - 4; L = layout_notes(notes, w); h = len(L) * lh + 8
            box_ = (TB_X0, tb[3] + 2, FRAME[2], tb[3] + 2 + h)
        elif name == "over_rev":
            w = TB_X0 - FRAME[0] - 8; L = layout_notes(notes, w); h = len(L) * lh + 8
            box_ = (FRAME[0], rv[3] + 2, TB_X0 - 4, rv[3] + 2 + h)
        else:
            w = 130.0; L = layout_notes(notes, w); h = len(L) * lh + 8
            box_ = (FRAME[0], FRAME[3] - h, FRAME[0] + w + 4, FRAME[3])
        if box_[3] > FRAME[3] or (name == "top_left" and _overlap(box_, rg)):
            continue
        obst = [tb, rv, rg, box_]
        inner = (FRAME[0] + 2, FRAME[1] + 2, FRAME[2] - 2, FRAME[3] - 2)
        scales = [spec["scale"]] if spec.get("scale") else STD_SCALES
        r = place_views((fw, fh, tw, th, rw, rh), M, obst, scales, inner, ann_rects)
        if r:
            cands.append((r[0], name, box_, L, r[1], r[2]))
    if not cands:                                     # 마지막 수단: 가장 작은 척도로 왼쪽 아래에 강제 배치
        L = layout_notes(notes, TB_W - 4); h = len(L) * lh + 8
        cands = [(STD_SCALES[-1], "over_tb", (TB_X0, tb[3] + 2, FRAME[2], tb[3] + 2 + h), L, FRAME[0] + 20, rv[3] + 20)]
    order = {"over_tb": 0, "over_rev": 1, "top_left": 2}
    cands.sort(key=lambda c: (-c[0], order[c[1]]))
    scale, nname, nbox, nlines, x0, y0 = cands[0]

    origin = {"front": (x0, y0), "top": (x0, y0 + fh * scale + M["gap_v"]), "right": (x0 + fw * scale + M["gap_h"], y0)}
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
        cen = {}
        for g in vis + hid:
            if g[0] == "circle" or (g[0] == "arc" and (g[4] - g[3]) % 360 >= 180):
                ck = (round(g[1][0], 2), round(g[1][1], 2)); cen[ck] = max(cen.get(ck, 0), g[2])
        for (cx, cy), r in cen.items():
            if r * scale < 1.0:
                continue
            X, Y = T((cx, cy)); e = r * scale + 3
            msp.add_line((X - e, Y), (X + e, Y), dxfattribs={"layer": "CENTER"})
            msp.add_line((X, Y - e), (X, Y + e), dxfattribs={"layer": "CENTER"})
        bx0, by0 = T((b[0], b[1])); bx1, by1 = T((b[2], b[3]))
        vboxes[v] = (bx0, by0, bx1, by1)

        def dup(length, direction):
            for d in dims:
                if d.get("view") == v and d.get("type", "linear") == "linear" and d.get("dir", "h") == direction:
                    a_, b_ = mapf(d["p1"]), mapf(d["p2"]); ax_ = 0 if direction == "h" else 1
                    if abs(abs(a_[ax_] - b_[ax_]) - length) < 0.01:
                        return True
            return False
        if auto:
            o = lambda side: DIM_FIRST + DIM_STEP * cnt(v, side)
            L_ = lambda **kw: msp.add_linear_dim(dimstyle="ISO-A4", override=ov, dxfattribs={"layer": "DIM"}, **kw).render()
            if v == "front":
                if not dup(b[2] - b[0], "h"):
                    L_(base=(bx0, by0 - o("below")), p1=(bx0, by0), p2=(bx1, by0), angle=0)
                if not dup(b[3] - b[1], "v"):
                    L_(base=(bx0 - o("left"), by0), p1=(bx0, by0), p2=(bx0, by1), angle=90)
            elif v == depth_view == "right" and not dup(b[2] - b[0], "h"):
                L_(base=(bx0, by1 + o("above")), p1=(bx0, by1), p2=(bx1, by1), angle=0)
            elif v == depth_view == "top" and not dup(b[3] - b[1], "v"):
                L_(base=(bx0 - o("left"), by0), p1=(bx0, by0), p2=(bx0, by1), angle=90)

    P = lambda view: mappers[view]
    for d in dims:
        try:
            add_dim(msp, d, P(d["view"]), scale, vboxes.get(d["view"]), stack)
        except Exception as ex:
            print("  치수 건너뜀:", d, ex)
    for key, dd in (spec.get("datums") or {}).items():
        t = P(dd["view"])(dd["at"])
        datum_symbol(msp, key, t, (t[0] + dd.get("dx", -10) * k, t[1] + dd.get("dy", -9) * k))
    for g in spec.get("gdt", []):
        t = P(g["view"])(g["at"])
        gdt_frame(msp, t[0] + g.get("dx", 14) * k, t[1] + g.get("dy", 10) * k, g["symbol"], g["tol"], g.get("datums", []), target=t)
    for r in spec.get("roughness", []):
        t = P(r["view"])(r["at"])
        roughness(msp, t[0] + r.get("dx", 5) * k, t[1] + r.get("dy", 5) * k, r["value"], leader_to=t)
    for c in spec.get("callouts", []):
        t = P(c["view"])(c["at"])
        callout(msp, t, (t[0] + c.get("dx", 15), t[1] + c.get("dy", 12)), c["text"])

    general_roughness(msp, spec)
    title_block(msp, spec, scale, mass, rev_date)
    revision_block(msp, revs)
    # 주서
    nx, ny = nbox[0] + 2, nbox[3] - 3
    text(msp, "주서", nx, ny, 3.5)
    for i, ln in enumerate(nlines):
        text(msp, ln, nx + 0.5, ny - 6 - lh * i, TN)

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
