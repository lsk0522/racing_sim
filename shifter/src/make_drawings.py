# LSK H/SEQ Shifter — author: Sunggwan
"""도면 생성: docs/dxf/<VERSION>/specs/*.yaml → docs/dxf/<VERSION>/ 에 DXF, pdf/, png/ + INDEX.md
사용: cd shifter/src && python3 make_drawings.py            (전체)
      python3 make_drawings.py ../docs/dxf/v0.5/specs/M5-01_cam.yaml   (한 장만)"""
import sys, os, glob, argparse, datetime
import yaml
import cad, drawings

camH = None


def shape_of(part):
    global camH
    if part == "cam":
        camH = camH or cad.camgen.build_cam(cad.camgen.H_PROFILE)
        return cad.make_cam(camH)
    return {
        "base": lambda: cad.make_base()[0],
        "tower_negY": lambda: cad.make_tower(-1),
        "tower_posY": lambda: cad.make_tower(+1),
        "gate": lambda: cad.make_gate()[0],
        "cradle": cad.make_cradle,
        "hub": cad.make_hub,
        "cassette": cad.make_cassette,
        "arm_plate": lambda: cad.plate_xz(cad.arm2d(), cad.YC + 8.1, cad.YC + 11.1),
        "seq_lock": lambda: cad.make_seq_lock(False),
        "eccentric": lambda: cad.cyl_x(5.5, -2, 6, y=cad.SHAFT_Y, z=cad.SHAFT_Z - 3)
                             - cad.cyl_x(2.5, -3, 7, y=cad.SHAFT_Y, z=cad.SHAFT_Z),
    }[part]()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", default="v0.5")
    ap.add_argument("--out", default=None)
    ap.add_argument("--date", default=str(datetime.date.today()))
    ap.add_argument("specs", nargs="*")
    a = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    a.out = a.out or os.path.join(here, "..", "docs", "dxf", a.version)
    specs = a.specs or sorted(glob.glob(os.path.join(a.out, "specs", "*.yaml")))
    for d in ("pdf", "png"):
        os.makedirs(os.path.join(a.out, d), exist_ok=True)
    index = []
    for fn in specs:
        spec = yaml.safe_load(open(fn, encoding="utf-8"))
        shp = shape_of(spec["part"])
        dxf, scale = drawings.make_drawing(spec, shp, a.out, rev_date=a.date)
        stem = os.path.basename(dxf)[:-4]
        drawings.preview(dxf, out_png=os.path.join(a.out, "png", stem + ".png"),
                         out_pdf=os.path.join(a.out, "pdf", stem + ".pdf"))
        index.append((spec["part_no"], spec["name"], spec.get("material", ""), spec.get("process", ""), scale,
                      len(spec.get("dims", [])), len(spec.get("gdt", [])), stem))
        print("drawn", os.path.basename(dxf), "scale", scale)
    if not a.specs:        # 전체 생성 때만 목록 갱신
        with open(os.path.join(a.out, "INDEX.md"), "w", encoding="utf-8") as f:
            f.write(f"# 도면 목록 {a.version} — 제작: Sunggwan\n\n| 품번 | 품명 | 재질 | 공정 | 척도 | 치수 지시 | 기하공차 | 파일 |\n|---|---|---|---|---|---|---|---|\n")
            for r in index:
                sc = f"{r[4]:g}:1" if r[4] >= 1 else f"1:{1/r[4]:g}"
                f.write(f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} | {sc} | {r[5]} | {r[6]} | [DXF]({r[7]}.dxf) · [PDF](pdf/{r[7]}.pdf) · [PNG](png/{r[7]}.png) |\n")
