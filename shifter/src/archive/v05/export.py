# LSK H/SEQ Shifter — author: Sunggwan
"""CAD 내보내기 — 버전 폴더 하나에 제작 방식별로 정리.

  shifter/CAD/<VERSION>/
    mach/sheet/   판재에서 가공 (STEP + 1:1 외곽 DXF)
    mach/block/   블록·봉에서 가공 (STEP)
    print/        3D 출력 (출력 방향으로 돌린 STL + STEP)
    buy/          구매품 (참고용 STEP + BOM.csv)
    assembly/     전체 조립 STEP, 서브 조립, parts_table.csv, geometry.json

사용: cd shifter/src && python3 export.py        (→ ../CAD/v0.5)
"""
import os, json, shutil, csv, argparse
from build123d import *
import ezdxf, trimesh
import cad

VERSION = "v0.5"

# (품번, 부품, 모듈, 폴더, 소재/두께 또는 출력재, STL 출력 방향, 수량)
CATALOG = [
    ("M1-01", "base",             "M1 베이스·전자부",   "mach/sheet", "6061 8t",       None, 1),
    ("M1-02", "bottom_cover",     "M1 베이스·전자부",   "print",      "PETG",          None, 1),
    ("M2-01", "tower_negY",       "M2 타워",           "mach/sheet", "6061 6t",       None, 1),
    ("M2-02", "tower_posY",       "M2 타워",           "mach/sheet", "6061 6t",       None, 1),
    ("M3-01", "cradle",           "M3 짐벌 코어",       "mach/block", "6061 블록",      None, 1),
    ("M3-02", "hub",              "M3 짐벌 코어",       "mach/block", "6061 블록",      None, 1),
    ("M3-03", "tail_cap",         "M3 짐벌 코어",       "print",      "PETG",          None, 1),
    ("M3-04", "lever_bar",        "M3 짐벌 코어",       "buy",        "Ø8 h6 경화축",   None, 1),
    ("M3-05", "hollow_pin",       "M3 짐벌 코어",       "buy",        "Ø5×3 중공 핀",   None, 1),
    ("M3-06", "tail_bolt",        "M3 짐벌 코어",       "buy",        "M5 볼트",        None, 1),
    ("M4-01", "cartridge",        "M4 좌우 스프링",     "print",      "PETG",          None, 1),
    ("M4-02", "plunger_S1_L",     "M4 좌우 스프링",     "print",      "PETG",          (Axis.X, 90), 1),
    ("M4-03", "plunger_S1_R",     "M4 좌우 스프링",     "print",      "PETG",          (Axis.X, -90), 1),
    ("M4-04", "plunger_S2",       "M4 좌우 스프링",     "print",      "PETG",          (Axis.X, -90), 1),
    ("M4-05", "cup_S2",           "M4 좌우 스프링",     "print",      "PETG",          (Axis.X, 90), 1),
    ("M5-01", "cam",              "M5 디텐트 카세트",   "mach/sheet", "6061 17t→16.5", None, 1),
    ("M5-02", "cassette",         "M5 디텐트 카세트",   "mach/block", "6061 블록",      None, 1),
    ("M5-03", "arm_plate",        "M5 디텐트 카세트",   "mach/sheet", "6061 3t",       None, 2),
    ("M5-04", "rollers",          "M5 디텐트 카세트",   "buy",        "625ZZ ×3",      None, 1),
    ("M6-01", "seq_lock",         "M6 SEQ 잠금",       "mach/sheet", "6061 10t",      None, 1),
    ("M6-02", "eccentric",        "M6 SEQ 잠금",       "mach/block", "6061 Ø14 봉",    None, 1),
    ("M6-03", "lock_pivot",       "M6 SEQ 잠금",       "print",      "PETG",          None, 1),
    ("M6-04", "shaft_bearings",   "M6 SEQ 잠금",       "print",      "PETG",          None, 1),
    ("M6-05", "mode_knob",        "M6 SEQ 잠금",       "print",      "ABS/PETG",      (Axis.Y, -90), 1),
    ("M7-01", "gate",             "M7 게이트",          "mach/sheet", "6061 3t",       None, 1),
    ("M8-01", "knob",             "M8 노브",           "print",      "ABS (도장)",     None, 1),
    ("M9-01", "pitch_sensor_cap", "M9 센서 캡",         "print",      "PETG",          (Axis.X, -90), 1),
    ("M9-02", "roll_sensor_cap",  "M9 센서 캡",         "print",      "PETG",          (Axis.Y, -90), 1),
    ("M9-03", "pin_collar",       "M9 센서 캡",         "print",      "PETG",          (Axis.X, 90), 1),
    ("M10-01", "shell",           "M10 외장 셸",        "print",      "ABS (도장)",     (Axis.X, 180), 1),
]
# 판재 부품의 1:1 외곽 DXF: 부품 → (2D 키, 두께 표기)
SHEET_DXF = {"base": ("base", "8t"), "tower_negY": ("tower_negY", "6t"), "tower_posY": ("tower_posY", "6t"),
             "cam": ("cam_H", "16.5t"), "arm_plate": ("arm_plate", "3t_x2"), "gate": ("gate", "3t")}
SUB_ASSEMBLIES = ["arm"]          # 여러 부품이 합쳐진 형상 → assembly/sub


def stl_out(shape, fn, ori):
    s2 = shape.rotate(*ori) if ori else shape
    bb = s2.bounding_box()
    s2 = s2.moved(Location((-bb.center().X, -bb.center().Y, -bb.min.Z)))
    export_stl(s2, fn, tolerance=0.02, angular_tolerance=0.15)
    m = trimesh.load(fn)
    if not m.is_watertight:
        m.merge_vertices(digits_vertex=3); m.update_faces(m.nondegenerate_faces()); m.remove_unreferenced_vertices()
        trimesh.repair.fix_normals(m); m.export(fn)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join("..", "CAD", VERSION))
    a = ap.parse_args()
    OUT = a.out
    for d in ["mach", "print", "buy", "assembly"]:      # 산출물만 지우고 README.md 등 손으로 쓴 파일은 유지
        shutil.rmtree(os.path.join(OUT, d), ignore_errors=True)
    for d in ["mach/sheet", "mach/block", "print", "buy", "assembly/sub"]:
        os.makedirs(f"{OUT}/{d}", exist_ok=True)

    st, cr, hb, fo, P2, camH = cad.build_all()
    eccentric = cad.cyl_x(5.5, -2, 6, y=cad.SHAFT_Y, z=cad.SHAFT_Z - 3) - cad.cyl_x(2.55, -3, 7, y=cad.SHAFT_Y, z=cad.SHAFT_Z)
    parts = {**st, **cr, **hb, **fo, "seq_lock": cad.make_seq_lock(False), "mode_knob": cad.make_mode_knob(False),
             "eccentric": eccentric,
             "arm_plate": cad.plate_xz(cad.arm2d(), cad.YC + 8.1, cad.YC + 11.1)}

    table = []
    for pno, name, mod, folder, mat, ori, qty in CATALOG:
        s = parts[name]
        base = f"{OUT}/{folder}/{pno}_{name}"
        export_step(s, base + ".step")
        files = [os.path.basename(base) + ".step"]
        if folder == "print":
            stl_out(s, base + ".stl", ori); files.append(os.path.basename(base) + ".stl")
        if name in SHEET_DXF:
            key, thk = SHEET_DXF[name]
            p = P2[key]
            doc = ezdxf.new(units=4); msp = doc.modelspace()
            for ring in [p.exterior] + list(p.interiors):
                msp.add_lwpolyline([(float(x), float(y)) for x, y in ring.coords], close=True)
            fn = f"{base}_{thk}.dxf"; doc.saveas(fn); files.append(os.path.basename(fn))
        bb = s.bounding_box()
        table.append([pno, name, mod, folder, mat, qty, round(bb.size.X, 1), round(bb.size.Y, 1), round(bb.size.Z, 1),
                      " ".join(files)])
    for name in SUB_ASSEMBLIES:
        export_step(parts[name], f"{OUT}/assembly/sub/{name}.step")

    children = []
    for k, v in parts.items():
        if k == "arm_plate":
            continue
        v2 = v.moved(Location((0, 0, 0))); v2.label = k; children.append(v2)
    export_step(Compound(children=children, label=f"LSK_HSEQ_Shifter_{VERSION}"),
                f"{OUT}/assembly/_ASSEMBLY_LSK_Shifter_{VERSION}.step")
    json.dump({"PHI_deg": cad.PHI, "THETA_STOP_deg": cad.THETA_STOP, "SEQ_STOP_deg": cad.SEQ_STOP, "H_GATE": cad.H_GATE,
               "L_KNOB": cad.L_KNOB}, open(f"{OUT}/assembly/geometry.json", "w"), indent=2, ensure_ascii=False)
    with open(f"{OUT}/assembly/parts_table.csv", "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["품번", "부품", "모듈", "폴더", "소재", "수량", "X", "Y", "Z", "파일"])
        w.writerows(table)
    shutil.copy(os.path.join(os.path.dirname(os.path.abspath(__file__)), "BOM.csv"), f"{OUT}/buy/BOM.csv")
    for r in table:
        print(r[:6])
