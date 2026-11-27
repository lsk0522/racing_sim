# LSK H/SEQ Shifter — author: Sunggwan
"""v0.6 → v0.7 도면 사양(YAML) 변환: 기구 좌우(Y) 반전 (11-27 Fusion 검토 결정).

- 모든 3D 좌표(center·p1·p2·at)의 y 부호를 바꾼다.
- top 뷰(도면 세로축 = y): above↔below, dy → −dy, 지름 지시 각도 → −각도
- right 뷰(도면 가로축 = y): left↔right, dx → −dx, 각도 → 180 − 각도
- front 뷰(도면 = x·z): 좌표만 (y는 깊이)
- −Y/+Y 이름이 붙은 쌍(타워 M2-01/02, 사이드 패널 M10-01/02)은 내용을 맞바꾼다
  (반전 후 −Y 자리에 오는 부품 = 반전 전 +Y 부품의 거울상)
- 모든 도면 rev 한 단계 올림
사용: python3 mirror_specs.py ../docs/dxf/v0.6/specs ../docs/dxf/v0.7/specs 2026-11-27
"""
import sys, os, glob, copy
import yaml

SRC, DST, DATE = sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else "2026-11-27"
SWAP = {"M2-01_tower_negY": "M2-02_tower_posY", "M2-02_tower_posY": "M2-01_tower_negY",
        "M10-01_panel_negY": "M10-02_panel_posY", "M10-02_panel_posY": "M10-01_panel_negY"}
KEEP = ("part", "part_no", "name")              # 맞바꿀 때 자리(품번)에 붙은 값은 유지
NAMES = {"M10-01_panel_negY": "사이드 패널 −Y (렌치 구멍)", "M10-02_panel_posY": "사이드 패널 +Y (창)"}


def flip_pt(p):
    return [p[0], -p[1], p[2]] if isinstance(p, list) and len(p) == 3 else p


def flip_item(d, view=None):
    if not isinstance(d, dict):
        return d
    v = d.get("view", view)
    out = {}
    for k, val in d.items():
        if k in ("center", "p1", "p2", "at"):
            val = flip_pt(val)
        elif k == "side" and v == "top":
            val = {"above": "below", "below": "above"}.get(val, val)
        elif k == "side" and v == "right":
            val = {"left": "right", "right": "left"}.get(val, val)
        elif k == "dy" and v == "top":
            val = -val
        elif k == "dx" and v == "right":
            val = -val
        elif k == "angle" and v == "top":
            val = -val
        elif k == "angle" and v == "right":
            val = 180 - val
        out[k] = val
    return out


def next_rev(r):
    return chr(ord(str(r)[0]) + 1) if r else "A"


def convert(spec):
    s = copy.deepcopy(spec)
    for key in ("dims", "gdt", "roughness", "callouts"):
        if key in s and s[key]:
            s[key] = [flip_item(x) for x in s[key]]
    if s.get("datums"):
        s["datums"] = {k: flip_item(v) for k, v in s["datums"].items()}
    return s


if __name__ == "__main__":
    os.makedirs(DST, exist_ok=True)
    src = {os.path.basename(f)[:-5]: yaml.safe_load(open(f, encoding="utf-8")) for f in glob.glob(os.path.join(SRC, "*.yaml"))}
    for stem, spec in src.items():
        body = convert(src[SWAP.get(stem, stem)])
        if stem in SWAP:                                   # 자리 고유값은 원래 사양 것으로
            for k in KEEP:
                body[k] = spec[k]
            body["name"] = NAMES.get(stem, spec["name"])
            body["revisions"] = spec.get("revisions", [])
            body["rev"] = spec.get("rev")
        r = next_rev(body.get("rev"))
        body["rev"] = r
        body.setdefault("revisions", []).append({"rev": r, "date": DATE, "desc": "v0.7 좌우 반전 (운전석 +X 표준 7+R)"})
        body["change"] = "좌우(Y) 반전" + (f" — {SWAP[stem].split('_', 1)[0]} 거울상" if stem in SWAP else "")
        with open(os.path.join(DST, stem + ".yaml"), "w", encoding="utf-8") as f:
            yaml.safe_dump(body, f, allow_unicode=True, sort_keys=False, width=200, default_flow_style=None)
        print(stem, "rev", r)
