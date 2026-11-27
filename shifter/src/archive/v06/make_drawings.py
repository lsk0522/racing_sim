# LSK H/SEQ Shifter — author: Sunggwan
"""도면 생성: docs/dxf/<VERSION>/specs/*.yaml → docs/dxf/<VERSION>/ 에 DXF, pdf/, png/ + INDEX.md
사용: cd shifter/src && python3 make_drawings.py            (전체)
      python3 make_drawings.py ../docs/dxf/v0.5/specs/M5-01_cam.yaml   (한 장만)
형상은 export.py가 만든 STEP(CAD/<버전>/mach/…)을 읽는다 → 먼저 export.py 실행."""
import sys, os, glob, argparse, datetime
import yaml
from build123d import import_step
import drawings

HERE = os.path.dirname(os.path.abspath(__file__))


def shape_of(spec, version):
    """기본: CAD/<버전>/mach/**/<품번>_<부품>.step (export.py 산출물, 매끄러운 B-rep).
    YAML에 source: 를 주면 그 STEP을 사용 (레포 shifter/ 기준 상대 경로)."""
    if spec.get("source"):
        fn = os.path.join(HERE, "..", spec["source"])
    else:
        pno = spec["part_no"].replace("LSK-SH-", "")
        hits = glob.glob(os.path.join(HERE, "..", "CAD", version, "**", f"{pno}_{spec['part']}.step"), recursive=True)
        if not hits:
            raise FileNotFoundError(f"{pno}_{spec['part']}.step 없음 — 먼저 export.py 실행")
        fn = hits[0]
    return import_step(fn), os.path.relpath(fn, os.path.join(HERE, ".."))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", default="v0.5")
    ap.add_argument("--out", default=None)
    ap.add_argument("--date", default=str(datetime.date.today()))
    ap.add_argument("specs", nargs="*")
    a = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    a.out = a.out or os.path.join(here, "..", "docs", "dxf", a.version)
    natural = lambda fn: [int(t) if t.isdigit() else t for t in __import__("re").split(r"(\d+)", os.path.basename(fn))]
    specs = a.specs or sorted(glob.glob(os.path.join(a.out, "specs", "*.yaml")), key=natural)
    for d in ("pdf", "png"):
        os.makedirs(os.path.join(a.out, d), exist_ok=True)
    index = []
    for fn in specs:
        spec = yaml.safe_load(open(fn, encoding="utf-8"))
        shp, src = shape_of(spec, a.version)
        spec.setdefault("tech_ref", os.path.basename(src))
        dxf, scale = drawings.make_drawing(spec, shp, a.out, rev_date=a.date)
        stem = os.path.basename(dxf)[:-4]
        drawings.preview(dxf, out_png=os.path.join(a.out, "png", stem + ".png"),
                         out_pdf=os.path.join(a.out, "pdf", stem + ".pdf"))
        index.append((spec["part_no"], spec["name"], spec.get("material", ""), spec.get("process", ""), scale,
                      len(spec.get("dims", [])), len(spec.get("gdt", [])), stem, spec.get("rev", ""),
                      spec.get("change", ""), spec.get("paper", "A4")))
        print("drawn", os.path.basename(dxf), "scale", scale)
    if not a.specs:        # 전체 생성 때만 목록 갱신
        with open(os.path.join(a.out, "INDEX.md"), "w", encoding="utf-8") as f:
            prev = f"v{float(a.version[1:]) - 0.1:.1f}"
            f.write(f"# 도면 목록 {a.version} — 제작: Sunggwan\n\n"
                    f"`변경` 칸이 비어 있는 부품은 {prev}와 형상이 같음 (도면은 {a.version} STEP에서 다시 그림, 개정 기호 그대로).\n\n"
                    "| 품번 | 품명 | 재질 | 공정 | 용지 | 척도 | 치수 지시 | 기하공차 | 개정 | 변경 | 파일 |\n|---|---|---|---|---|---|---|---|---|---|---|\n")
            for r in index:
                sc = f"{r[4]:g}:1" if r[4] >= 1 else f"1:{1/r[4]:g}"
                f.write(f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} | {r[10]} | {sc} | {r[5]} | {r[6]} | {r[8]} | {r[9] or '—'} | [DXF]({r[7]}.dxf) · [PDF](pdf/{r[7]}.pdf) · [PNG](png/{r[7]}.png) |\n")
