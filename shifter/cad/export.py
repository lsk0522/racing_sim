# LSK H/SEQ Shifter — author: Sunggwan
"""v0.4 export — folders per module (STEP=CNC/reference, STL=printing, DXF=plate outlines)."""
import os, json, shutil
from build123d import *
import ezdxf, trimesh
import cad

OUT = "out/LSK_Shifter_v0.5"
shutil.rmtree(OUT, ignore_errors=True)
st, cr, hb, fo, P2, camH = cad.build_all()
ecc = cad.make_mode_shaft(False) - cyl_x(2.5, -3, 52, y=cad.SHAFT_Y, z=cad.SHAFT_Z) if False else None
eccentric = cad.cyl_x(5.5, -2, 6, y=cad.SHAFT_Y, z=cad.SHAFT_Z - 3) - cad.cyl_x(2.55, -3, 7, y=cad.SHAFT_Y, z=cad.SHAFT_Z)
parts = {**st, **cr, **hb, **fo, "seq_lock": cad.make_seq_lock(False), "mode_knob": cad.make_mode_knob(False),
         "eccentric": eccentric}

# module: [(part, method, orientation)]  method: CNC / PRINT / BUY
MODULES = {
    "M1_베이스_전자부": [("base", "CNC", None), ("bottom_cover", "PRINT", None)],
    "M2_타워": [("tower_negY", "CNC", None), ("tower_posY", "CNC", None)],
    "M3_짐벌코어": [("cradle", "CNC", None), ("hub", "CNC", None), ("tail_cap", "PRINT", None),
                 ("lever_bar", "BUY", None), ("hollow_pin", "BUY", None), ("tail_bolt", "BUY", None)],
    "M4_좌우스프링_카트리지": [("cartridge", "PRINT", None), ("plunger_S1_L", "PRINT", (Axis.X, 90)),
                        ("plunger_S1_R", "PRINT", (Axis.X, -90)), ("plunger_S2", "PRINT", (Axis.X, -90)),
                        ("cup_S2", "PRINT", (Axis.X, 90))],
    "M5_디텐트_카세트": [("cam", "CNC", None), ("cassette", "CNC", None), ("arm", "CNC", None), ("rollers", "BUY", None)],
    "M6_SEQ_잠금": [("seq_lock", "CNC", None), ("eccentric", "CNC", (Axis.Y, 90)), ("lock_pivot", "PRINT", None),
                  ("shaft_bearings", "PRINT", None), ("mode_knob", "PRINT", (Axis.Y, -90))],
    "M7_게이트": [("gate", "CNC", None)],
    "M8_노브": [("knob", "PRINT", None)],
    "M9_센서캡": [("pitch_sensor_cap", "PRINT", (Axis.X, -90)), ("roll_sensor_cap", "PRINT", (Axis.Y, -90)),
                ("pin_collar", "PRINT", (Axis.X, 90))],
    "M10_외장셸": [("shell", "PRINT", (Axis.X, 180))],
}
DXF = {"M1_베이스_전자부": [("base_outline_8t", "base")], "M2_타워": [("tower_negY_6t", "tower_negY"), ("tower_posY_6t", "tower_posY")],
       "M5_디텐트_카세트": [("cam_profile_16.5t", "cam_H"), ("arm_plate_3t_x2", "arm_plate")], "M7_게이트": [("gate_3t", "gate")]}

table = []
for mod, items in MODULES.items():
    d = f"{OUT}/{mod}"
    os.makedirs(d, exist_ok=True)
    for name, how, ori in items:
        s = parts[name]
        export_step(s, f"{d}/{name}.step")
        if how == "PRINT":
            s2 = s.rotate(*ori) if ori else s
            bb = s2.bounding_box()
            s2 = s2.moved(Location((-bb.center().X, -bb.center().Y, -bb.min.Z)))
            fn = f"{d}/{name}.stl"
            export_stl(s2, fn, tolerance=0.02, angular_tolerance=0.15)
            m = trimesh.load(fn)
            if not m.is_watertight:
                m.merge_vertices(digits_vertex=3); m.update_faces(m.nondegenerate_faces()); m.remove_unreferenced_vertices()
                trimesh.repair.fix_normals(m); m.export(fn)
        bb = s.bounding_box()
        table.append((mod, name, how, round(bb.size.X, 1), round(bb.size.Y, 1), round(bb.size.Z, 1)))
    for fname, key in DXF.get(mod, []):
        p = P2[key]
        doc = ezdxf.new(units=4); msp = doc.modelspace()
        for ring in [p.exterior] + list(p.interiors):
            msp.add_lwpolyline([(float(x), float(y)) for x, y in ring.coords], close=True)
        doc.saveas(f"{d}/{fname}.dxf")

children = []
for k, v in parts.items():
    v2 = v.moved(Location((0, 0, 0))); v2.label = k; children.append(v2)
export_step(Compound(children=children, label="LSK_HSEQ_Shifter_v0.5"), f"{OUT}/_ASSEMBLY_LSK_Shifter_v0.5.step")
json.dump({"PHI_deg": cad.PHI, "THETA_STOP_deg": cad.THETA_STOP, "SEQ_STOP_deg": cad.SEQ_STOP, "H_GATE": cad.H_GATE,
           "L_KNOB": cad.L_KNOB}, open(f"{OUT}/geometry.json", "w"), indent=2, ensure_ascii=False)
with open(f"{OUT}/parts_table.csv", "w") as f:
    f.write("모듈,부품,제작,X,Y,Z\n")
    for r in table:
        f.write(",".join(map(str, r)) + "\n")
for r in table:
    print(r)
