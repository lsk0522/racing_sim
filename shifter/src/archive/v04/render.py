import sys, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import *
import cad
import check as C

# Design: black-anodized structure + raw aluminium moving parts + one orange accent
COL = {"black": (0.16, 0.17, 0.19), "alu": (0.80, 0.82, 0.85), "steel": (0.68, 0.70, 0.73),
       "accent": (0.93, 0.40, 0.10), "print": (0.24, 0.25, 0.28)}
MAT = {
    "base": "black", "bottom_cover": "black", "tower_negY": "black", "tower_posY": "black", "gate": "alu",
    "cassette": "black", "lock_pivot": "black", "shaft_bearings": "black", "pitch_sensor_cap": "print",
    "pin_collar": "accent", "cradle": "alu", "cam": "alu", "cartridge": "accent", "roll_sensor_cap": "print",
    "hollow_pin": "steel", "hub": "alu", "lever_bar": "steel", "knob": "alu", "tail_bolt": "steel",
    "tail_cap": "print", "arm": "alu", "rollers": "steel", "seq_lock": "accent", "mode_shaft": "steel",
    "mode_knob": "alu",
}


def tris(shape, tol=0.25):
    v, t = shape.tessellate(tol, 0.4)
    V = np.array([[p.X, p.Y, p.Z] for p in v])
    return V[np.array(t)]


def render(theta=0, phi=0, seq=False, views=((22, -58),), fname="out/render.png", title="", hide=()):
    parts = C.parts_at(theta, phi, seq)
    polys, cols = [], []
    light = np.array([0.35, -0.55, 0.75]); light /= np.linalg.norm(light)
    for k, s in parts.items():
        if k in hide or k.startswith(("plunger", "cup")):
            continue
        T = tris(s)
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        n /= (np.linalg.norm(n, axis=1)[:, None] + 1e-9)
        d = np.clip(n @ light, -1, 1)
        shade = 0.42 + 0.58 * np.abs(d)
        base = np.array(COL[MAT.get(k, "print")])
        spec = np.clip(np.abs(d) ** 12, 0, 1)[:, None] * 0.25
        for tri, sh, sp in zip(T, shade, spec):
            polys.append(tri); cols.append(np.clip(base * sh + sp, 0, 1))
    fig = plt.figure(figsize=(6.5 * len(views), 7.5))
    for i, (el, az) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(polys, facecolors=cols, edgecolors="none", linewidths=0))
        ax.set_xlim(-88, 52); ax.set_ylim(-75, 65); ax.set_zlim(-78, 130)
        ax.set_box_aspect((140, 140, 208))
        ax.view_init(el, az)
        ax.set_axis_off()
    fig.suptitle(title, fontsize=13)
    fig.tight_layout()
    fig.savefig(fname, dpi=120, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    render(0, 0, False, views=((18, -52), (18, 40)), fname="out/render_neutral.png",
           title="LSK 쉬프터 v0.4 — 중립 (H 모드)")
    render(cad.THETA_STOP, cad.PHI["7-R"], False, views=((24, -60),), fname="out/render_7th.png",
           title="v0.4 — 7단")
    render(0, 0, False, views=((12, -95),), fname="out/render_side_cam.png",
           title="v0.4 — −Y 측면 (캠·롤러 창)")
    render(cad.SEQ_STOP - 0.2, 0, True, views=((22, -58),), fname="out/render_seq.png",
           title="v0.4 — 시퀀셜 모드 (잠금 포켓 올라옴)", hide=("tower_posY", "gate"))
