# LSK H/SEQ Shifter — author: Sunggwan
"""Fusion 작업용 묶음 만들기.

python3 fusion_pack.py [--out ../CAD/v0.6/fusion]
  → 00_ASSEMBLY_<ver>_fusion.step : 부품마다 "품번_부품" 이름 + 색, 책상(28mm) 참고 형상 포함 → Fusion에서 바로 컴포넌트로 열림
  → PARAMS.csv                    : cad.py 주요 치수 상수 (이름·값·주석)
STEP 원본(부품별)은 export.py 산출물(CAD/<ver>/mach·print·buy)을 그대로 쓴다.
"""
import argparse, csv, os, re
from build123d import *
import cad
from export import CATALOG, VERSION

COLOR = {"mach": Color(0.72, 0.73, 0.75), "print": Color(0.16, 0.16, 0.17), "buy": Color(0.55, 0.56, 0.58),
         "desk": Color(0.05, 0.05, 0.06)}


SKIP = re.compile(r"^(SHELL_|SMOOTH$|BASE_OUT$|GATE_OUT$)")      # v0.5 곡면 셸 잔재·스위치 — v0.6에서 안 씀


def _fmt(v):
    if isinstance(v, (list, tuple)):
        inner = ", ".join(_fmt(x) for x in v)
        return f"[{inner}]" if isinstance(v, list) else f"({inner})"
    try:
        f = float(v)
        return str(int(f)) if f == int(f) else f"{f:.3f}".rstrip("0")
    except (TypeError, ValueError):
        return repr(v)


def params_rows(src):
    rows = {}
    for line in open(src, encoding="utf-8"):
        m = re.match(r"^([A-Z][A-Z0-9_]*(?:\s*,\s*[A-Z][A-Z0-9_]*)*)\s*=\s*([^#]+?)\s*(?:#\s*(.*))?$", line.rstrip("\n"))
        if not m:
            continue
        note = (m.group(3) or "").strip()
        for name in [n.strip() for n in m.group(1).split(",")]:     # "X_FRONT, X_REAR = 46, -84" 도 읽기
            if SKIP.match(name):
                continue
            v = getattr(cad, name, None)
            if v is None or isinstance(v, (bool, str, dict)):
                continue
            if isinstance(v, (list, tuple)) and not all(isinstance(x, (int, float, tuple, list)) or hasattr(x, "__float__") for x in v):
                continue
            rows[name] = (name, _fmt(v), note or rows.get(name, ("", "", ""))[2])   # 마지막 정의 = 실제 값
    return list(rows.values())


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join("..", "CAD", VERSION, "fusion"))
    out = ap.parse_args().out
    os.makedirs(out, exist_ok=True)

    st, cr, hb, fo, P2, camH = cad.build_all()
    ex = cad.extra_parts()
    parts = {**st, **cr, **hb, **fo, "seq_lock": cad.make_seq_lock(False), "mode_knob": cad.make_mode_knob(False),
             "eccentric": ex["eccentric"]}
    label = {name: (f"{pno}_{name}", folder.split("/")[0]) for pno, name, mod, folder, mat, ori, qty in CATALOG}

    children = []
    for k, v in parts.items():
        if k == "arm_plate":
            continue
        lab, grp = label.get(k, (f"X_{k}", "mach" if k in ("arm",) else "buy"))
        s = v.moved(Location((0, 0, 0)))
        s.label, s.color = lab, COLOR[grp]
        children.append(s)
    desk = cad.desk_ghost()
    desk.label, desk.color = "REF_desk_Desker_28mm", COLOR["desk"]
    children.append(desk)
    fn = os.path.join(out, f"00_ASSEMBLY_LSK_Shifter_{VERSION}_fusion.step")
    export_step(Compound(children=children, label=f"LSK_HSEQ_Shifter_{VERSION}"), fn)

    with open(os.path.join(out, "PARAMS.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["이름", "값 (mm·deg)", "설명 (cad_model.py 주석)"])
        w.writerow(["# 주의", "", "v0.7: 아래 값은 cad_model.py 모델 좌표(좌우 반전 전). Fusion·조립 STEP은 Y가 뒤집힌 최종 좌표 → Y 값은 부호를 바꿔 읽기. PHI_M 대신 PHI(최종) 사용"])
        w.writerows(params_rows(os.path.join(os.path.dirname(os.path.abspath(__file__)), "cad_model.py")))
    print("saved", fn, len(children), "bodies")
