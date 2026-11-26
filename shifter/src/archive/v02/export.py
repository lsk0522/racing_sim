import os, math, json
from build123d import *
import ezdxf
import cad, check as C

OUT = "out/LSK_Shifter_v0.2"
os.makedirs(f"{OUT}/STL", exist_ok=True)
os.makedirs(f"{OUT}/STEP", exist_ok=True)
os.makedirs(f"{OUT}/DXF", exist_ok=True)

st, cr, hb, fo = C.st, C.cr, C.hb, C.fo
fork = cad.make_fork(False)
fork_guide = st["fork_guide"]

# Print orientation: (rotation axis, angle)
ORIENT = {
    "tower_negY": (Axis.X, 90), "tower_posY": (Axis.X, -90), "cam_HS": (Axis.X, 90),
    "arm": (Axis.X, 90), "plunger_S1_L": (Axis.X, 90), "plunger_S1_R": (Axis.X, -90),
    "plunger_S2": (Axis.X, -90), "cup_S2": (Axis.X, 90),
}
PRINTED = ["base", "tower_negY", "tower_posY", "fork_guide", "crank_bracket", "cradle", "cam_HS", "hub", "knob",
           "tail_cap", "carriage", "arm", "mode_crank", "mode_knob", "plunger_S1_L", "plunger_S1_R", "plunger_S2",
           "cup_S2", "gate_post_1", "fork", "gate_plate_PRINT_TEST"]
ORIENT.update({"mode_crank": (Axis.Y, 90), "mode_knob": (Axis.Y, -90), "crank_bracket": (Axis.Y, 90)})

parts = {**st, **cr, **hb, **fo, "fork": fork, "mode_crank": cad.make_mode_crank(False), "mode_knob": cad.make_mode_knob()}
parts["gate_plate_PRINT_TEST"] = st["gate_plate"]

for name in PRINTED:
    s = parts[name]
    if name in ORIENT:
        ax, ang = ORIENT[name]
        s = s.rotate(ax, ang)
    bb = s.bounding_box()
    s = s.moved(Location((-bb.center().X, -bb.center().Y, -bb.min.Z)))
    export_stl(s, f"{OUT}/STL/{name}.stl", tolerance=0.02, angular_tolerance=0.15)

# STEP: per part (assembly coordinates) + full assembly
asm_children = []
for name, s in parts.items():
    if name == "gate_plate_PRINT_TEST":
        continue
    s2 = s.moved(Location((0, 0, 0)))
    s2.label = name
    asm_children.append(s2)
    export_step(s, f"{OUT}/STEP/{name}.step")
asm = Compound(children=asm_children, label="LSK_HSEQ_Shifter_v0.2")
export_step(asm, f"{OUT}/STEP/_ASSEMBLY_LSK_Shifter_v0.2.step")

# DXF: gate plate (SUS304 3t laser) + cam profiles (reference / metal upgrade)
_, g2 = cad.make_gate()
doc = ezdxf.new(); msp = doc.modelspace()
def add_ring(coords):
    pts = [(float(y), float(x)) for x, y in coords]  # Drawing: horizontal = Y (left/right), vertical = X (fore/aft)
    msp.add_lwpolyline(pts, close=True)
add_ring(list(g2.exterior.coords))
for h in g2.interiors:
    add_ring(list(h.coords))
for yc in (-64.0, 33.0):
    for x in (-34, 34):
        msp.add_circle((yc, x), 2.75)
doc.saveas(f"{OUT}/DXF/gate_plate_SUS304_3t.dxf")

camH, camS = C.camH, C.camS
for nm, poly in (("cam_track_H", camH), ("cam_track_SEQ", camS)):
    d = ezdxf.new(); m = d.modelspace()
    m.add_lwpolyline([(float(x), float(z)) for x, z in poly.exterior.coords], close=True)
    m.add_circle((0, 0), 4.15)
    d.saveas(f"{OUT}/DXF/{nm}_profile.dxf")

# Key dimensions JSON (shared with firmware)
geo = {"PHI_deg": cad.PHI, "THETA_STOP_deg": cad.THETA_STOP, "H_GATE": cad.H_GATE, "L_KNOB": cad.L_KNOB,
       "seq_click_deg": 7.2, "gear_engage_deg": 8.5, "bearing": "625ZZ"}
json.dump(geo, open(f"{OUT}/geometry.json", "w"), indent=2, ensure_ascii=False)
print("exported", len(os.listdir(f"{OUT}/STL")), "STL,", len(os.listdir(f"{OUT}/STEP")), "STEP")
