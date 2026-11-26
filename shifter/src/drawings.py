# LSK H/SEQ Shifter — author: Sunggwan
"""
Drawing generator: drawing spec YAML → A3 DXF (+ PDF/PNG preview)
 - Third-angle projection (KS / ISO 128-30): front (XZ), top (XY, above front), right side (YZ, right of front), hidden-line removal
 - Dimensions: linear, diameter, radius — ISO 286 fit codes (e.g. M7) give the deviations automatically, or ±/+− tolerances directly
 - Geometric tolerances (ISO 1101): flatness, parallelism, perpendicularity, position, coaxiality, circularity, cylindricity, profile, runout, symmetry, angularity, straightness
 - Datums, surface roughness (ISO 21920/1302), notes (general tolerances ISO 2768-mK, edges ISO 13715, post-processing), title block, revision table
Usage: python3 drawings.py specs/*.yaml --out ../docs/dxf/v0.5
"""
import sys, os, math, argparse, datetime
import numpy as np
import yaml, ezdxf
from ezdxf.enums import TextEntityAlignment
from build123d import GeomType
import iso_tol

A3 = (420.0, 297.0)
TITLE = (296.0, 10.0, 410.0, 62.0)
AREA = (25.0, 15.0, 290.0, 287.0)     # drawing-view area (left), right column = notes/title
STD_SCALES = [5, 2, 1, 0.5, 0.2, 0.1]     # ISO 5455 권장 척도
FONT_DXF = "malgun.ttf"         # what the Windows CAD viewer uses (Malgun Gothic)
FONT_PREVIEW = "NotoSansCJK-Regular.ttc"   # preview only; DXF keeps malgun.ttf

VIEW_MAP = {
    "front": (lambda p: (p[0], p[2]), (0, -1000, 0), (0, 0, 1)),
    "top":   (lambda p: (p[0], p[1]), (0, 0, 1000), (0, 1, 0)),
    "right": (lambda p: (p[1], p[2]), (1000, 0, 0), (0, 0, 1)),
}


# ---------------------------------------------------------------- geometry
def edge_poly(e, mapf):
    n = 2 if e.geom_type == GeomType.LINE else max(24, int(e.length / 0.4))
    pts = [e.position_at(t) for t in np.linspace(0, 1, n)]
    return [(p.X, p.Y) for p in pts]


def project(shape, view):
    mapf, origin, up = VIEW_MAP[view]
    bb = shape.bounding_box()
    c = bb.center()
    o = (c.X + origin[0], c.Y + origin[1], c.Z + origin[2])
    vis, hid = shape.project_to_viewport(o, up, (c.X, c.Y, c.Z))
    # project_to_viewport returns coordinates relative to the look-at centre → shift back to absolute
    cu, cv = mapf((c.X, c.Y, c.Z))
    V = [[(p[0] + cu, p[1] + cv) for p in edge_poly(e, mapf)] for e in vis]
    H = [[(p[0] + cu, p[1] + cv) for p in edge_poly(e, mapf)] for e in hid]
    circles = []
    for e in vis:
        if e.geom_type == GeomType.CIRCLE:
            try:
                ctr = e.arc_center; r = e.radius
                circles.append(((ctr.X + cu, ctr.Y + cv), r))
            except Exception:
                pass
    allp = np.array([p for pl in V for p in pl]) if V else np.zeros((1, 2))
    box = (allp[:, 0].min(), allp[:, 1].min(), allp[:, 0].max(), allp[:, 1].max())
    return V, H, circles, box


# ---------------------------------------------------------------- DXF helpers
def setup_doc():
    doc = ezdxf.new("R2018", setup=True, units=4)
    doc.styles.add("KR", font=FONT_DXF)
    for name, color, lt, lw in [("VISIBLE", 7, "Continuous", 50), ("HIDDEN", 8, "DASHED", 25), ("CENTER", 1, "CENTER", 18),
                                ("DIM", 3, "Continuous", 18), ("GDT", 4, "Continuous", 25), ("TEXT", 7, "Continuous", 25),
                                ("FRAME", 7, "Continuous", 70), ("THIN", 7, "Continuous", 18)]:
        doc.layers.add(name, color=color, linetype=lt, lineweight=lw)
    ds = doc.dimstyles.new("ISO-25", dxfattribs={
        "dimtxsty": "KR", "dimtxt": 3.0, "dimasz": 2.5, "dimexe": 1.25, "dimexo": 0.625, "dimgap": 0.8,
        "dimtad": 1, "dimtih": 0, "dimtoh": 0, "dimdec": 2, "dimdsep": ord("."), "dimlunit": 2, "dimzin": 8,
        "dimscale": 1.0, "dimclrd": 3, "dimclre": 3, "dimclrt": 3, "dimtdec": 3, "dimtzin": 8, "dimtfac": 0.7,
        "dimadec": 0, "dimaunit": 0})
    ds.set_arrows(blk="")
    return doc


def text_width(s, h):
    w = 0.0
    for ch in s:
        w += h * (1.08 if ord(ch) > 0x2E80 else 0.72 if ch.isupper() or ch in "Ø×±°~|" else 0.6)
    return w


def wrap_text(s, width, h, indent="    "):
    out, cur = [], ""
    for tok in s.split(" "):
        cand = (cur + " " + tok) if cur else tok
        if text_width(cand, h) <= width or not cur:
            cur = cand
        else:
            out.append(cur); cur = indent + tok
    out.append(cur)
    return out


def text(msp, s, x, y, h=3.0, layer="TEXT", align=TextEntityAlignment.MIDDLE_LEFT, rot=0):
    t = msp.add_text(s, height=h, dxfattribs={"layer": layer, "style": "KR", "rotation": rot})
    t.set_placement((x, y), align=align)
    return t


def rect(msp, x0, y0, x1, y1, layer="THIN"):
    msp.add_lwpolyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], close=True, dxfattribs={"layer": layer})


def arrow(msp, tip, frm, layer="GDT", size=2.5, filled=True):
    d = np.array(tip) - np.array(frm); L = np.linalg.norm(d)
    if L < 1e-6:
        return
    d /= L; n = np.array([-d[1], d[0]])
    b = np.array(tip) - d * size
    pts = [tuple(tip), tuple(b + n * size * 0.3), tuple(b - n * size * 0.3)]
    if filled:
        h = msp.add_hatch(dxfattribs={"layer": layer}); h.paths.add_polyline_path(pts, is_closed=True)
    msp.add_lwpolyline(pts, close=True, dxfattribs={"layer": layer})


# GD&T symbols (drawn with lines inside a 6×6 cell, centre cx, cy)
def gdt_symbol(msp, kind, cx, cy, s=2.2, layer="GDT"):
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
        L([(cx - s * .7, cy - s), (cx + s * .5, cy + s * .6)]); arrow(msp, (cx + s * .7, cy + s), (cx - s * .2, cy), layer, 1.2)
    elif kind == "symmetry":
        for dy, w in ((-s * .6, .6), (0, 1), (s * .6, .6)):
            L([(cx - s * w, cy + dy), (cx + s * w, cy + dy)])


def gdt_frame(msp, x, y, kind, tol, datums=(), target=None, h=6.0):
    """Feature control frame: [symbol][tolerance][A][B][C] with its lower-left corner at (x, y). target = arrowhead position."""
    w_sym = 7.0; w_tol = max(10.0, len(tol) * 2.1 + 3); w_d = 6.5
    xs = [x, x + w_sym, x + w_sym + w_tol] + [x + w_sym + w_tol + w_d * (i + 1) for i in range(len(datums))]
    for i in range(len(xs) - 1):
        rect(msp, xs[i], y, xs[i + 1], y + h, layer="GDT")
    gdt_symbol(msp, kind, x + w_sym / 2, y + h / 2)
    text(msp, tol, x + w_sym + w_tol / 2, y + h / 2, 2.8, "GDT", TextEntityAlignment.MIDDLE_CENTER)
    for i, d in enumerate(datums):
        text(msp, d, xs[2 + i] + w_d / 2, y + h / 2, 2.8, "GDT", TextEntityAlignment.MIDDLE_CENTER)
    if target is not None:
        start = (x, y + h / 2) if target[0] < x else (xs[-1], y + h / 2)
        msp.add_lwpolyline([start, target], dxfattribs={"layer": "GDT"})
        arrow(msp, target, start)


def datum_symbol(msp, letter, target, box_at):
    """Datum feature: filled triangle on the feature + leader + square box."""
    bx, by = box_at
    rect(msp, bx - 3, by - 3, bx + 3, by + 3, layer="GDT")
    text(msp, letter, bx, by, 3.2, "GDT", TextEntityAlignment.MIDDLE_CENTER)
    d = np.array(target) - np.array(box_at); L = np.linalg.norm(d); d /= max(L, 1e-6)
    start = np.array(box_at) + d * 3.2
    msp.add_lwpolyline([tuple(start), tuple(target)], dxfattribs={"layer": "GDT"})
    n = np.array([-d[1], d[0]])
    tri = [tuple(target), tuple(np.array(target) - d * 2.6 + n * 1.6), tuple(np.array(target) - d * 2.6 - n * 1.6)]
    h = msp.add_hatch(dxfattribs={"layer": "GDT"}); h.paths.add_polyline_path(tri, is_closed=True)


def roughness(msp, x, y, value, leader_to=None):
    """ISO 1302 machined-surface symbol (√ + horizontal bar) and Ra value."""
    msp.add_lwpolyline([(x - 2.2, y + 2.2), (x, y), (x + 4.4, y + 6.6), (x + 12, y + 6.6)], dxfattribs={"layer": "GDT"})
    msp.add_lwpolyline([(x - 1.5, y + 2.2), (x + 1.5, y + 2.2)], dxfattribs={"layer": "GDT"})
    text(msp, value, x + 5, y + 8.2, 2.5, "GDT")
    if leader_to is not None:
        msp.add_lwpolyline([(x, y), leader_to], dxfattribs={"layer": "GDT"})


def third_angle_symbol(msp, x, y):
    msp.add_circle((x + 16, y), 3, dxfattribs={"layer": "THIN"}); msp.add_circle((x + 16, y), 1.6, dxfattribs={"layer": "THIN"})
    msp.add_lwpolyline([(x, y - 1.6), (x + 8, y - 3), (x + 8, y + 3), (x, y + 1.6), (x, y - 1.6)], dxfattribs={"layer": "THIN"})
    msp.add_lwpolyline([(x - 2, y), (x + 21, y)], dxfattribs={"layer": "CENTER"})


# ---------------------------------------------------------------- dimensions
def dim_tol_override(spec, size):
    """Tolerance part of a spec → (text suffix, upper, lower)."""
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
    ov = {"dimlfac": 1 / scale, "dimdec": 2, "dimzin": 8, "dimtxsty": "KR", "dimtxt": 3.0}
    if kind == "linear":
        p1, p2 = P(d["p1"]), P(d["p2"])
        ang = 90 if d.get("dir", "h") == "v" else 0
        side = d.get("side")
        if side and vbox is not None:
            k = stack.get((d["view"], side), 0); stack[(d["view"], side)] = k + 1
            x0, y0, x1, y1 = vbox
            base = {"above": (p1[0], y1 + 9 + 8 * k), "below": (p1[0], y0 - 10 - 8 * k),
                    "right": (x1 + 10 + 9 * k, p1[1]), "left": (x0 - 10 - 9 * k, p1[1])}[side]
        else:
            off = d.get("offset", 10)
            if ang == 0:
                base = (p1[0], max(p1[1], p2[1]) + off) if off > 0 else (p1[0], min(p1[1], p2[1]) + off)
            else:
                base = (max(p1[0], p2[0]) + off, p1[1]) if off > 0 else (min(p1[0], p2[0]) + off, p1[1])
        size = abs((p2[0] - p1[0]) if ang == 0 else (p2[1] - p1[1])) / scale
        suf, u, l = dim_tol_override(d, size)
        txt = d.get("text", ("%%c" if d.get("dia") else "") + "<>" + suf)
        if u is not None:
            ov.update({"dimtol": 1, "dimtp": u, "dimtm": -l, "dimtfac": 0.7, "dimtdec": 3, "dimtzin": 8})
        dim = msp.add_linear_dim(base=base, p1=p1, p2=p2, angle=ang, dimstyle="ISO-25", override=ov, text=txt,
                                 dxfattribs={"layer": "DIM"})
        dim.render()
    elif kind in ("diameter", "radius"):
        c = P(d["center"]); r = d["r"] * scale
        size = 2 * d["r"] if kind == "diameter" else d["r"]
        suf, u, l = dim_tol_override(d, size)
        if u is not None:
            ov.update({"dimtol": 1, "dimtp": u, "dimtm": -l, "dimtfac": 0.7, "dimtdec": 3, "dimtzin": 8})
        pre = d.get("prefix", "")
        if kind == "diameter":
            a = np.radians(d.get("angle", 45)); ov.update({"dimtih": 1, "dimtoh": 1, "dimtmove": 0})
            loc = (c[0] + (r + d.get("lead", 10)) * np.cos(a), c[1] + (r + d.get("lead", 10)) * np.sin(a))
            dim = msp.add_diameter_dim(center=c, radius=r, location=loc, dimstyle="ISO-25", override=ov,
                                       text=pre + "<>" + suf, dxfattribs={"layer": "DIM"})
        else:
            a = np.radians(d.get("angle", 45)); ov.update({"dimtih": 1, "dimtoh": 1})
            loc = (c[0] + (r + d.get("lead", 10)) * np.cos(a), c[1] + (r + d.get("lead", 10)) * np.sin(a))
            dim = msp.add_radius_dim(center=c, radius=r, location=loc, dimstyle="ISO-25", override=ov,
                                     text=pre + "R<>" + suf, dxfattribs={"layer": "DIM"})
        dim.render()


# ---------------------------------------------------------------- main
def make_drawing(spec, shape, outdir, rev_date=None):
    doc = setup_doc(); msp = doc.modelspace()
    W, Hh = A3
    rect(msp, 20, 10, W - 10, Hh - 10, "FRAME")
    # Projections
    views = spec.get("views", ["front", "top", "right"])
    proj = {v: project(shape, v) for v in views}
    sz = {v: (b[2] - b[0], b[3] - b[1]) for v, (_, _, _, b) in proj.items()}
    fw, fh = sz.get("front", (0, 0)); tw, th = sz.get("top", (0, 0)); rw, rh = sz.get("right", (0, 0))
    cnt = lambda v, sd: sum(1 for d in spec.get("dims", []) if d.get("view") == v and d.get("side") == sd)
    auto = 1 if spec.get("auto_overall", True) else 0
    depth_view = "right" if "right" in views else ("top" if "top" in views else None)
    gap_v = 14 + 8 * (cnt("front", "above") + cnt("top", "below")) + 6
    gap_h = 16 + 9 * (cnt("front", "right") + cnt("right", "left")) + 6
    ml = 9 * (cnt("front", "left") + auto) + 6; mb = 8 * (cnt("front", "below") + auto) + 6
    mr = 9 * cnt("right", "right") + 6; mt = 8 * (cnt("top", "above") + (auto if depth_view == "top" else 0)) + 10
    AW = AREA[2] - AREA[0] - ml - mr - (gap_h if rw else 0)
    AH = AREA[3] - AREA[1] - mb - mt - (gap_v if th else 0)
    scale = spec.get("scale")
    if not scale:
        scale = next((s_ for s_ in STD_SCALES if (fw + rw) * s_ <= AW and (fh + th) * s_ <= AH), STD_SCALES[-1])
    # centre the view group in the area
    gw = (fw + rw) * scale + (gap_h if rw else 0); gh = (fh + th) * scale + (gap_v if th else 0)
    x0 = AREA[0] + ml + max(0, (AW - (fw + rw) * scale) / 2)
    y0 = AREA[1] + mb + max(0, (AH - (fh + th) * scale) / 2)
    origin = {"front": (x0, y0), "top": (x0, y0 + fh * scale + gap_v), "right": (x0 + fw * scale + gap_h, y0)}
    mappers = {}; vboxes = {}; stack = {}
    for v in views:
        V, H, circles, b = proj[v]
        ox, oy = origin[v]
        T = (lambda b, ox, oy: (lambda q: (ox + (q[0] - b[0]) * scale, oy + (q[1] - b[1]) * scale)))(b, ox, oy)
        mapf = VIEW_MAP[v][0]
        mappers[v] = (lambda T, mapf: (lambda p3: T(mapf(p3))))(T, mapf)
        for pl in (H if v in spec.get("hidden_views", ["front"]) else []):
            msp.add_lwpolyline([T(p) for p in pl], dxfattribs={"layer": "HIDDEN"})
        for pl in V:
            msp.add_lwpolyline([T(p) for p in pl], dxfattribs={"layer": "VISIBLE"})
        seen = set()
        for (c, r) in circles:
            k = (round(c[0], 2), round(c[1], 2))
            if k in seen or r * scale < 1.0:
                continue
            seen.add(k)
            cx, cy = T(c); e = r * scale + 2.5
            msp.add_line((cx - e, cy), (cx + e, cy), dxfattribs={"layer": "CENTER"})
            msp.add_line((cx, cy - e), (cx, cy + e), dxfattribs={"layer": "CENTER"})
        name = {"front": "정면도", "top": "평면도", "right": "우측면도"}[v]
        text(msp, name, ox + (b[2] - b[0]) * scale / 2, oy + (b[3] - b[1]) * scale + 4, 3.0,
             align=TextEntityAlignment.BOTTOM_CENTER) if False else None
        bx0, by0 = T((b[0], b[1])); bx1, by1 = T((b[2], b[3]))
        vboxes[v] = (bx0, by0, bx1, by1)
        # Automatic overall dimensions — front: width & height, depth once (right or top)
        if auto:
            ov = {"dimlfac": 1 / scale, "dimdec": 1, "dimzin": 8, "dimtxsty": "KR", "dimtxt": 3.0}
            nb = cnt(v, "below"); nl = cnt(v, "left"); na = cnt(v, "above")
            if v == "front":
                msp.add_linear_dim(base=(bx0, by0 - 10 - 8 * nb), p1=(bx0, by0), p2=(bx1, by0), angle=0, dimstyle="ISO-25",
                                   override=ov, dxfattribs={"layer": "DIM"}).render()
                msp.add_linear_dim(base=(bx0 - 10 - 9 * nl, by0), p1=(bx0, by0), p2=(bx0, by1), angle=90, dimstyle="ISO-25",
                                   override=ov, dxfattribs={"layer": "DIM"}).render()
            elif v == depth_view == "right":
                msp.add_linear_dim(base=(bx0, by1 + 9 + 8 * na), p1=(bx0, by1), p2=(bx1, by1), angle=0, dimstyle="ISO-25",
                                   override=ov, dxfattribs={"layer": "DIM"}).render()
            elif v == depth_view == "top":
                msp.add_linear_dim(base=(bx0 - 10 - 9 * nl, by0), p1=(bx0, by0), p2=(bx0, by1), angle=90, dimstyle="ISO-25",
                                   override=ov, dxfattribs={"layer": "DIM"}).render()

    def P(view):
        return mappers[view]

    for d in spec.get("dims", []):
        try:
            add_dim(msp, d, P(d["view"]), scale, vboxes.get(d["view"]), stack)
        except Exception as ex:
            print("  dim skipped:", d, ex)
    for k, dd in (spec.get("datums") or {}).items():
        t = P(dd["view"])(dd["at"]); box = (t[0] + dd.get("dx", -12), t[1] + dd.get("dy", -10))
        datum_symbol(msp, k, t, box)
    for g in spec.get("gdt", []):
        t = P(g["view"])(g["at"])
        gdt_frame(msp, t[0] + g.get("dx", 18), t[1] + g.get("dy", 12), g["symbol"], g["tol"], g.get("datums", []), target=t)
    for r in spec.get("roughness", []):
        t = P(r["view"])(r["at"])
        roughness(msp, t[0] + r.get("dx", 6), t[1] + r.get("dy", 6), r["value"], leader_to=t)

    # ---- notes (right column, wrapped)
    gt = spec.get("general_tolerance", "ISO 2768-mK")
    notes = [f"1. 일반 공차: {gt} (지시 없는 치수·형상)",
             "   선형 ±  0.5~3: 0.1 | 3~6: 0.1 | 6~30: 0.2 | 30~120: 0.3 | 120~400: 0.5",
             "   모따기·R ±  0.5~3: 0.2 | 3~6: 0.5 | 6~: 1   각도 ±  ~10: 1° | 10~50: 0°30'",
             "   형상(K)  진직·평면도 ~10: 0.05 | ~30: 0.1 | ~100: 0.2 | ~300: 0.4 / 직각도 ~100: 0.4 / 대칭 0.6 / 흔들림 0.2",
             f"2. 모서리: {spec.get('edges', 'ISO 13715 — 외측 −0.3 / 내측 +0.3 (지시 없는 곳 바리 제거)')}",
             f"3. 표면 거칠기(지시 없는 곳): {spec.get('roughness_default', 'Ra 3.2')} (ISO 21920)",
             "4. 끼워맞춤 ISO 286-1/2, 기하공차 ISO 1101 (독립의 원칙 ISO 8015), 투상 3각법, 단위 mm"]
    fin = spec.get("finish", [])
    if fin:
        notes.append("5. 후처리 (순서대로):")
        notes += [f"   {i+1}) {f}" for i, f in enumerate(fin)]
    for i, n in enumerate(spec.get("notes", [])):
        notes.append(f"{6 + i if fin else 5 + i}. {n}")
    nx0, nx1 = TITLE[0], TITLE[2]
    n_top = 287.0 - 6 * (len(spec.get("revisions", [1])) + 1) - 8
    n_bot = TITLE[3] + 4
    for h in (2.5, 2.3, 2.1, 1.9, 1.7):
        lines = []
        for n in notes:
            ind = "      " if n.startswith("   ") else "    "
            lines += wrap_text(n, (nx1 - nx0 - 9), h, ind)
        if len(lines) * h * 1.65 <= n_top - n_bot:
            break
    text(msp, "주기", nx0, n_top + 4, 3.0)
    for i, ln in enumerate(lines):
        text(msp, ln, nx0 + 1, n_top - h * 1.65 * i, h)
    # ---- title block
    x0, y0, x1, y1 = TITLE
    rect(msp, x0, y0, x1, y1, "FRAME")
    rows = [("제작", "Sunggwan"), ("품명", spec["name"]), ("품번", spec["part_no"]), ("재질", spec.get("material", "")),
            ("소재", spec.get("stock", "")), ("공정", spec.get("process", "")),
            ("척도", f"{scale:g}:1" if scale >= 1 else f"1:{1/scale:g}"), ("일반공차", gt)]
    rh_ = (y1 - y0) / len(rows)
    for i, (k, v) in enumerate(rows):
        yy = y1 - rh_ * (i + 1)
        msp.add_line((x0, yy), (x1, yy), dxfattribs={"layer": "THIN"})
        text(msp, k, x0 + 2, yy + rh_ / 2, 2.6)
        text(msp, v, x0 + 22, yy + rh_ / 2, 3.0 if k in ("품명", "품번") else 2.6)
    msp.add_line((x0 + 20, y0), (x0 + 20, y1), dxfattribs={"layer": "THIN"})
    text(msp, "LSK H/SEQ Shifter", x1 - 2, y1 - rh_ / 2, 2.6, align=TextEntityAlignment.MIDDLE_RIGHT)
    text(msp, f"Rev {spec.get('rev', 'v0.5')}  ·  {rev_date or datetime.date.today()}", x1 - 2, y1 - rh_ * 1.5, 2.4,
         align=TextEntityAlignment.MIDDLE_RIGHT)
    text(msp, "3각법", x1 - 32, y0 + rh_ * 0.5, 2.4)
    third_angle_symbol(msp, x1 - 22, y0 + rh_ * 0.5)
    # ---- revision table (top right)
    revs = spec.get("revisions", [{"rev": spec.get("rev", "v0.5"), "date": str(rev_date or datetime.date.today()), "desc": "최초 작성"}])
    rx0, ry1 = TITLE[0], 287.0
    rect(msp, rx0, ry1 - 6 * (len(revs) + 1), 410, ry1, "THIN")
    text(msp, "REV   날짜          내용", rx0 + 2, ry1 - 3, 2.4)
    for i, r in enumerate(revs):
        text(msp, f"{r['rev']}   {r['date']}   {r['desc']}", rx0 + 2, ry1 - 3 - 6 * (i + 1), 2.4)

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
    from ezdxf.addons.drawing.config import Configuration, LineweightPolicy, BackgroundPolicy, ColorPolicy
    doc = ezdxf.readfile(dxf_path)
    doc.styles.get("KR").dxf.font = FONT_PREVIEW
    fig = plt.figure(figsize=(16.54, 11.69))
    ax = fig.add_axes([0, 0, 1, 1])
    ctx = RenderContext(doc)
    cfg = Configuration(lineweight_policy=LineweightPolicy.ABSOLUTE, background_policy=BackgroundPolicy.WHITE,
                        color_policy=ColorPolicy.BLACK)
    Frontend(ctx, MatplotlibBackend(ax), config=cfg).draw_layout(doc.modelspace(), finalize=True)
    ax.set_xlim(0, A3[0]); ax.set_ylim(0, A3[1]); ax.set_aspect("equal"); ax.axis("off")
    if out_png:
        fig.savefig(out_png, dpi=150, facecolor="white")
    if out_pdf:
        fig.savefig(out_pdf, facecolor="white")
    plt.close(fig)
