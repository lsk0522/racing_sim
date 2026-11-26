# LSK H/SEQ Shifter — author: Sunggwan
"""v0.6 PBR renders (pyrender + EGL). Monotone graphite, contrast by finish only:
matte print (covers) / satin anodize (panels, base) / bright machined faces (chamfers, gate top).
Usage: python3 render_v06.py [v06|v05] [out_prefix]   (run from the CAD folder of that version)
"""
import os, sys, pickle
os.environ["PYOPENGL_PLATFORM"] = "egl"
sys.path.insert(0, os.getcwd())
import numpy as np, trimesh, pyrender
from PIL import Image, ImageDraw, ImageFont
import cad, check as C

MODE = sys.argv[1] if len(sys.argv) > 1 else "v06"
OUT = sys.argv[2] if len(sys.argv) > 2 else "out/v06"
os.makedirs(os.path.dirname(OUT) or ".", exist_ok=True)
FONT = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 30)

G = (0.155, 0.16, 0.17)                     # one graphite hue for everything outside
MAT06 = {
    "matte": (G, 0.0, 0.78), "anod": (G, 0.55, 0.46), "bright": ((0.80, 0.81, 0.83), 1.0, 0.22),
    "machined": ((0.60, 0.61, 0.63), 1.0, 0.34), "steel": ((0.70, 0.71, 0.73), 1.0, 0.25),
    "inner": ((0.10, 0.10, 0.11), 0.0, 0.6), "alu": ((0.80, 0.81, 0.83), 0.95, 0.32),
    "smoke": ((0.05, 0.05, 0.06), 0.1, 0.05), "fabric": ((0.07, 0.07, 0.075), 0.0, 0.95),
}
# part → (base material, chamfer axis or None, top-face material or None)
PART06 = {
    "cover_front": ("matte", None, None), "cover_rear": ("matte", None, None),
    "panel_negY": ("anod", 1, None), "panel_posY": ("anod", 1, None),
    "base": ("anod", 2, None), "bottom_cover": ("matte", None, None),
    "gate": ("anod", 2, "machined"), "knob": ("anod", None, None), "mode_knob": ("anod", 0, None),
    "lock_nut": ("machined", None, None), "sleeve": ("steel", None, None), "lever_bar": ("steel", None, None),
    "bolt_heads": ("steel", None, None), "window": ("smoke", None, None), "boot": ("fabric", None, None),
    "cam": ("alu", None, None), "rollers": ("steel", None, None), "arm": ("alu", None, None),
    "cradle": ("alu", None, None), "hub": ("alu", None, None), "tail_bolt": ("steel", None, None),
}
MAT05 = {"alu": ((0.86, 0.87, 0.89), 0.95, 0.32), "black": ((0.05, 0.05, 0.06), 0.55, 0.42),
         "shell": ((0.13, 0.135, 0.15), 0.05, 0.55), "accent": ((0.95, 0.36, 0.06), 0.0, 0.45),
         "steel": ((0.72, 0.73, 0.75), 1.0, 0.25), "print": ((0.10, 0.10, 0.11), 0.0, 0.6)}
try:
    import render_pbr as RP
    PART05 = RP.PART_MAT
except Exception:
    PART05 = {}


def tess(shape, tol=0.05):
    v, t = shape.tessellate(tol, 0.15)
    return trimesh.Trimesh(np.array([[p.X, p.Y, p.Z] for p in v]), np.array(t), process=True)


def meshes_for(name, shape):
    m = tess(shape)
    m.fix_normals()
    if MODE == "v05":
        col, met, rough = MAT05[PART05.get(name, "print")]
        return [(trimesh.graph.smooth_shade(m, angle=np.radians(28)), (col, met, rough, 1.0))]
    base, ax, top = PART06.get(name, ("inner", None, None))
    fn = m.face_normals
    groups = {base: np.ones(len(fn), bool)}
    if ax is not None:
        a = np.abs(fn[:, ax])
        ch = (a > 0.3) & (a < 0.95)
        groups[base] &= ~ch
        groups["bright"] = ch
    if top is not None:
        t = fn[:, 2] > 0.95
        groups[base] &= ~t
        groups[top] = groups.get(top, np.zeros(len(fn), bool)) | t
    out = []
    for mat, mask in groups.items():
        if mask.sum() == 0:
            continue
        sm = m.submesh([np.where(mask)[0]], append=True)
        col, met, rough = MAT06[mat]
        alpha = 0.62 if mat == "smoke" else 1.0
        out.append((trimesh.graph.smooth_shade(sm, angle=np.radians(28)), (col, met, rough, alpha)))
    return out


_cache = {}


def scene_for(parts, hide=()):
    sc = pyrender.Scene(bg_color=[0.955, 0.955, 0.96, 1.0], ambient_light=[0.30, 0.30, 0.32])
    for k, s in parts.items():
        if k in hide or k.startswith(("plunger", "cup")):
            continue
        key = (k, id(s))
        if key not in _cache:
            _cache[key] = meshes_for(k, s)
        for mesh, (col, met, rough, alpha) in _cache[key]:
            mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=[*col, alpha], metallicFactor=met,
                                                     roughnessFactor=rough, alphaMode="BLEND" if alpha < 1 else "OPAQUE")
            sc.add(pyrender.Mesh.from_trimesh(mesh, material=mat, smooth=True))
    floor = trimesh.creation.box((1400, 1400, 2)); floor.apply_translation((0, 0, cad.Z_BASE - cad.BASE_T - 3.0 - (2 if MODE == "v05" else 2)))
    sc.add(pyrender.Mesh.from_trimesh(floor, material=pyrender.MetallicRoughnessMaterial(
        baseColorFactor=[0.93, 0.93, 0.94, 1], metallicFactor=0.0, roughnessFactor=0.9)))
    return sc


def look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = map(np.asarray, (eye, target, up))
    f = target - eye; f = f / np.linalg.norm(f)
    s = np.cross(f, up)
    if np.linalg.norm(s) < 1e-3:
        s = np.cross(f, (1, 0, 0))
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    M = np.eye(4); M[:3, 0] = s; M[:3, 1] = u; M[:3, 2] = -f; M[:3, 3] = eye
    return M


def shot(parts, eye, target=(-20, -8, 20), size=(1100, 1100), title="", hide=(), fov=0.55):
    sc = scene_for(parts, hide)
    cam = pyrender.PerspectiveCamera(yfov=fov, aspectRatio=size[0] / size[1])
    sc.add(cam, pose=look_at(eye, target))
    for d, inten in (((-0.35, -0.75, 0.9), 3.6), ((0.85, 0.45, 0.45), 1.7), ((0.2, 0.9, 0.6), 1.3), ((-0.9, 0.2, 0.3), 1.0)):
        d = np.array(d) / np.linalg.norm(d)
        sc.add(pyrender.DirectionalLight(color=np.ones(3), intensity=inten), pose=look_at(np.array(target) + 300 * d, target))
    r = pyrender.OffscreenRenderer(*size)
    color, _ = r.render(sc, flags=pyrender.RenderFlags.RGBA)
    r.delete()
    img = Image.fromarray(color[:, :, :3])
    if title:
        ImageDraw.Draw(img).text((30, 22), title, fill=(30, 30, 35), font=FONT)
    return img


def combine(imgs, fname):
    W = sum(i.width for i in imgs); H = max(i.height for i in imgs)
    out = Image.new("RGB", (W, H), "white"); x = 0
    for i in imgs:
        out.paste(i, (x, 0)); x += i.width
    out.save(fname)


VIEWS = {
    "hero": dict(eye=(240, -340, 200), target=(-20, -8, 15)),
    "rear": dict(eye=(-390, 270, 210), target=(-20, -8, 15)),
    "side": dict(eye=(-19, -1100, 10), target=(-19, -8, 22), fov=0.24),
    "front": dict(eye=(1100, -8, 10), target=(0, -8, 22), fov=0.24),
    "top": dict(eye=(-19, -8, 1100), target=(-19, -8, 0), fov=0.17),
    "detail": dict(eye=(190, -235, -5), target=(15, -45, -45), fov=0.42),
}

if __name__ == "__main__":
    P0 = C.parts_at(0, 0, False)
    if MODE == "v06":
        P0 = {**P0, "boot": cad.pose({"boot": cad.make_boot()}, 0, 0)["boot"]}
    imgs = {}
    for k, v in VIEWS.items():
        imgs[k] = shot(P0, **v)
        imgs[k].save(f"{OUT}_{k}.png")
        print("saved", k, flush=True)
    if MODE == "v06":
        P7 = C.parts_at(cad.THETA_STOP, cad.PHI["7-R"], False)
        imgs["g7"] = shot(P7, **VIEWS["hero"]); imgs["g7"].save(f"{OUT}_g7.png")
        imgs["inside"] = shot(P0, **VIEWS["hero"], hide=("cover_front", "cover_rear", "panel_negY", "window", "boot"))
        imgs["inside"].save(f"{OUT}_inside.png")
    print("done")
