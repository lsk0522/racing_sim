import os, json
from build123d import *
import ezdxf
import trimesh
import cad, check as C

OUT = "out/LSK_Shifter_v0.3"
for d in ("STL_출력물", "STEP", "DXF_알루미늄판"):
    os.makedirs(f"{OUT}/{d}", exist_ok=True)

st, cr, hb, fo = C.st, C.cr, C.hb, C.fo
_, _, _, _, plates2d, _ = cad.build_all()

parts = {**st, **cr, **hb, **fo, "fork": cad.make_fork(False), "mode_crank": cad.make_mode_crank(False),
         "mode_knob": cad.make_mode_knob(), "gate_plate_PRINT_TEST": st["gate_plate"]}

# Printed parts + print orientation
ORIENT = {
    "arm": (Axis.X, 90), "plunger_S1_L": (Axis.X, 90), "plunger_S1_R": (Axis.X, -90), "plunger_S2": (Axis.X, -90),
    "cup_S2": (Axis.X, 90), "cam_spacer": (Axis.X, 90), "mode_crank": (Axis.Y, 90), "mode_knob": (Axis.Y, -90),
    "crank_bracket": (Axis.Y, 90), "stop_blocks": (Axis.X, 90),
}
PRINTED = ["cradle", "hub", "carriage", "arm", "knob", "tail_cap", "plunger_S1_L", "plunger_S1_R", "plunger_S2",
           "cup_S2", "cam_spacer", "fork", "fork_guide", "rail", "crank_bracket", "mode_crank", "mode_knob",
           "stop_blocks", "clip_negY_r", "gate_plate_PRINT_TEST"]

for name in PRINTED:
    s = parts[name]
    if name in ORIENT:
        ax, ang = ORIENT[name]
        s = s.rotate(ax, ang)
    bb = s.bounding_box()
    s = s.moved(Location((-bb.center().X, -bb.center().Y, -bb.min.Z)))
    fn = f"{OUT}/STL_출력물/{'tower_clip_x4' if name == 'clip_negY_r' else name}.stl"
    export_stl(s, fn, tolerance=0.02, angular_tolerance=0.15)
    m = trimesh.load(fn)
    if not m.is_watertight:
        m.merge_vertices(digits_vertex=3); m.update_faces(m.nondegenerate_faces()); m.remove_unreferenced_vertices()
        trimesh.repair.fix_normals(m); m.export(fn)

# STEP
children = []
for name, s in parts.items():
    if name == "gate_plate_PRINT_TEST":
        continue
    s2 = s.moved(Location((0, 0, 0))); s2.label = name
    children.append(s2)
    export_step(s, f"{OUT}/STEP/{name}.step")
export_step(Compound(children=children, label="LSK_HSEQ_Shifter_v0.3"), f"{OUT}/STEP/_ASSEMBLY_LSK_Shifter_v0.3.step")

# DXF — aluminium/metal plates (1:1 mm)
PLATES = {
    "base_Al5t": plates2d["base"],
    "tower_negY_Al6t": plates2d["tower_negY"],
    "tower_posY_Al6t": plates2d["tower_posY"],
    "gate_Al3t_or_SUS3t": plates2d["gate"],
    "cam_H_Al7075_5t_x2": plates2d["cam_H"],
    "cam_SEQ_Al7075_5t_x2": plates2d["cam_SEQ"],
}
for nm, p in PLATES.items():
    doc = ezdxf.new(units=4); msp = doc.modelspace()
    for ring in [p.exterior] + list(p.interiors):
        msp.add_lwpolyline([(float(x), float(y)) for x, y in ring.coords], close=True)
    doc.saveas(f"{OUT}/DXF_알루미늄판/{nm}.dxf")

geo = {"PHI_deg": cad.PHI, "THETA_STOP_deg": cad.THETA_STOP, "H_GATE": cad.H_GATE, "L_KNOB": cad.L_KNOB,
       "bearing": "625ZZ (towers: F625ZZ)", "lever": "Ø8 linear shaft"}
json.dump(geo, open(f"{OUT}/geometry.json", "w"), indent=2, ensure_ascii=False)
print("STL", len(os.listdir(f"{OUT}/STL_출력물")), "STEP", len(os.listdir(f"{OUT}/STEP")), "DXF", len(os.listdir(f"{OUT}/DXF_알루미늄판")))
for nm, p in PLATES.items():
    b = p.bounds
    print(nm, round(b[2] - b[0], 1), "x", round(b[3] - b[1], 1))
