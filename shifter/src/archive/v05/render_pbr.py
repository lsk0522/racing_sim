# LSK H/SEQ Shifter — author: Sunggwan
"""PBR renders (pyrender + EGL). Usage: python3 render_pbr.py"""
import os
os.environ["PYOPENGL_PLATFORM"] = "egl"
import numpy as np, trimesh, pyrender
from PIL import Image, ImageDraw, ImageFont
import cad, check as C

MAT = {  # base color RGB, metallic, roughness
    "alu": ((0.86, 0.87, 0.89), 0.95, 0.32), "black": ((0.05, 0.05, 0.06), 0.55, 0.42),
    "shell": ((0.13, 0.135, 0.15), 0.05, 0.55), "accent": ((0.95, 0.36, 0.06), 0.0, 0.45),
    "steel": ((0.72, 0.73, 0.75), 1.0, 0.25), "print": ((0.10, 0.10, 0.11), 0.0, 0.6),
}
PART_MAT = {
    "base": "black", "bottom_cover": "black", "tower_negY": "black", "tower_posY": "black", "gate": "alu",
    "cassette": "black", "lock_pivot": "black", "shaft_bearings": "black", "pitch_sensor_cap": "print",
    "pin_collar": "accent", "cradle": "alu", "cam": "alu", "cartridge": "accent", "roll_sensor_cap": "print",
    "hollow_pin": "steel", "hub": "alu", "lever_bar": "steel", "knob": "alu", "tail_bolt": "steel",
    "tail_cap": "print", "arm": "alu", "rollers": "steel", "seq_lock": "accent", "mode_shaft": "steel",
    "mode_knob": "alu", "shell": "shell",
}
FONT = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 30)


def to_trimesh(shape, tol=0.08):
    v, t = shape.tessellate(tol, 0.2)
    V = np.array([[p.X, p.Y, p.Z] for p in v]); F = np.array(t)
    m = trimesh.Trimesh(V, F, process=True)
    m.fix_normals()
    return trimesh.graph.smooth_shade(m, angle=np.radians(28))


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


_mesh_cache = {}


def scene_for(parts, hide=()):
    sc = pyrender.Scene(bg_color=[0.96, 0.96, 0.97, 1.0], ambient_light=[0.28, 0.28, 0.30])
    for k, s in parts.items():
        if k in hide or k.startswith(("plunger", "cup")):
            continue
        key = (k, id(s))
        if key not in _mesh_cache:
            _mesh_cache[key] = to_trimesh(s)
        col, met, rough = MAT[PART_MAT.get(k, "print")]
        mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=[*col, 1.0], metallicFactor=met, roughnessFactor=rough)
        sc.add(pyrender.Mesh.from_trimesh(_mesh_cache[key], material=mat, smooth=True))
    floor = trimesh.creation.box((900, 900, 2)); floor.apply_translation((0, 0, cad.Z_BASE - cad.BASE_T - 3))
    sc.add(pyrender.Mesh.from_trimesh(floor, material=pyrender.MetallicRoughnessMaterial(
        baseColorFactor=[0.93, 0.93, 0.94, 1], metallicFactor=0.0, roughnessFactor=0.9)))
    return sc


def shot(parts, eye, target=(-20, -5, 10), size=(1100, 1100), title="", hide=(), fov=0.55):
    sc = scene_for(parts, hide)
    cam = pyrender.PerspectiveCamera(yfov=fov, aspectRatio=size[0] / size[1])
    sc.add(cam, pose=look_at(eye, target))
    for d, inten in (((-0.4, -0.7, 0.9), 4.0), ((0.8, 0.5, 0.4), 1.6), ((0.2, 0.9, 0.6), 1.2)):
        d = np.array(d) / np.linalg.norm(d)
        sc.add(pyrender.DirectionalLight(color=np.ones(3), intensity=inten), pose=look_at(np.array(target) + 300 * d, target))
    r = pyrender.OffscreenRenderer(*size)
    color, _ = r.render(sc)
    r.delete()
    img = Image.fromarray(color)
    if title:
        ImageDraw.Draw(img).text((30, 22), title, fill=(30, 30, 35), font=FONT)
    return img


def combine(imgs, fname):
    W = sum(i.width for i in imgs); H = max(i.height for i in imgs)
    out = Image.new("RGB", (W, H), "white"); x = 0
    for i in imgs:
        out.paste(i, (x, 0)); x += i.width
    out.save(fname)


if __name__ == "__main__":
    P0 = C.parts_at(0, 0, False)
    combine([shot(P0, (230, -330, 190), title="LSK 쉬프터 v0.5 — 앞 3/4"),
             shot(P0, (-380, 260, 200), title="뒤 3/4")], "out/v05_hero.png")
    combine([shot(P0, (-20, -420, 40), title="−Y 측면 (캠 창)"),
             shot(P0, (-19, -9, 470), target=(-20, -10, 0), title="위에서 (게이트 각인)")], "out/v05_side_top.png")
    P7 = C.parts_at(cad.THETA_STOP, cad.PHI["7-R"], False)
    combine([shot(P7, (230, -330, 190), title="7단"),
             shot(P0, (230, -330, 190), title="셸 벗긴 내부 (v0.4 메커니즘 그대로)", hide=("shell",))], "out/v05_gear_inside.png")
