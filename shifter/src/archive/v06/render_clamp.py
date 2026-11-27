# LSK H/SEQ Shifter — author: Sunggwan
"""v0.6 + M12 desk clamp render (no anodizing: blasted raw 6061, dark printed covers, black Desker desk)."""
import sys
sys.argv = [sys.argv[0], "v06", sys.argv[1] if len(sys.argv) > 1 else "out/v06_clamp"]
import numpy as np, trimesh, pyrender
import render_v06 as RV
import cad

RV.MAT06["anod"] = ((0.70, 0.71, 0.73), 0.85, 0.55)          # bead-blasted raw aluminium
RV.MAT06["desk"] = ((0.035, 0.035, 0.04), 0.0, 0.55)          # Desker black wood top
for k in ("clamp_plate", "clamp_spine", "clamp_jaw"):
    RV.PART06[k] = ("anod", 2, None)
RV.PART06.update({"clamp_pad": ("matte", None, None), "clamp_knob": ("matte", None, None),
                  "clamp_bolt": ("steel", None, None), "desk": ("desk", None, None)})


def scene_for(parts, hide=()):
    sc = pyrender.Scene(bg_color=[0.955, 0.955, 0.96, 1.0], ambient_light=[0.30, 0.30, 0.32])
    for k, s in parts.items():
        if k in hide or k.startswith(("plunger", "cup")):
            continue
        key = (k, id(s))
        if key not in RV._cache:
            RV._cache[key] = RV.meshes_for(k, s)
        for mesh, (col, met, rough, alpha) in RV._cache[key]:
            mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=[*col, alpha], metallicFactor=met,
                                                     roughnessFactor=rough, alphaMode="BLEND" if alpha < 1 else "OPAQUE")
            sc.add(pyrender.Mesh.from_trimesh(mesh, material=mat, smooth=True))
    return sc


RV.scene_for = scene_for

if __name__ == "__main__":
    P = RV.C.parts_at(0, 0, False)
    P = {**P, "boot": cad.pose({"boot": cad.make_boot()}, 0, 0)["boot"], "desk": cad.desk_ghost()}
    a = RV.shot(P, eye=(330, -360, 120), target=(-10, -8, -40), title="v0.6 + 책상 클램프 (데스커 28mm)")
    b = RV.shot(P, eye=(420, -760, -150), target=(30, -8, -95), fov=0.36, title="옆에서 — C자 클램프", hide=("desk",))
    RV.combine([a, b], f"{RV.OUT}.png")
    print("saved", f"{RV.OUT}.png")
