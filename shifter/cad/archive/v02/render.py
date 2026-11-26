import sys, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import cad
import check as C

COL = {
    "metal": (0.78, 0.80, 0.83), "dark": (0.18, 0.19, 0.21), "accent": (0.80, 0.33, 0.10),
    "print": (0.28, 0.30, 0.34), "brass": (0.80, 0.62, 0.30),
}
MAT = {
    "base": "dark", "tower_negY": "print", "tower_posY": "print", "fork_guide": "dark",
    "gate_plate": "metal", "cradle": "accent", "cam_HS": "metal", "hub": "accent", "lever_bar": "metal",
    "knob": "metal", "tail_bolt": "metal", "tail_cap": "dark", "carriage": "print", "arm": "accent",
    "rollers": "metal", "mode_crank": "accent", "mode_knob": "metal", "fork": "brass", "crank_bracket": "dark",
}


def tris(shape, tol=0.4):
    v, t = shape.tessellate(tol, 0.5)
    V = np.array([[p.X, p.Y, p.Z] for p in v])
    return V[np.array(t)]


def render(theta=0, phi=0, seq=False, views=((22, -58),), fname="out/render.png", title=""):
    crp = cad.pose(C.cr, theta, 0, "cradle")
    hbp = cad.pose(C.hb, theta, phi, "hub")
    fop = C.follower_at(theta, seq)
    parts = {**C.st, **crp, **hbp, **fop}
    polys, cols = [], []
    light = np.array([0.4, -0.5, 0.75]); light /= np.linalg.norm(light)
    for k, s in parts.items():
        if k.startswith("gate_post"):
            m = "metal"
        else:
            m = MAT.get(k, "print")
        T = tris(s)
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        n /= (np.linalg.norm(n, axis=1)[:, None] + 1e-9)
        shade = 0.45 + 0.55 * np.clip(np.abs(n @ light), 0, 1)
        base = np.array(COL[m])
        for tri, sh in zip(T, shade):
            polys.append(tri); cols.append(np.clip(base * sh, 0, 1))
    fig = plt.figure(figsize=(7 * len(views), 8))
    for i, (el, az) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        pc = Poly3DCollection(polys, facecolors=cols, edgecolors="none", linewidths=0)
        ax.add_collection3d(pc)
        ax.set_xlim(-105, 65); ax.set_ylim(-95, 75); ax.set_zlim(-95, 165)
        ax.set_box_aspect((170, 170, 260))
        ax.view_init(el, az)
        ax.set_axis_off()
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(fname, dpi=110, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    render(0, 0, False, views=((20, -55), (12, 35)), fname="out/render_neutral.png", title="v0.2 소형 — 중립 (H 모드)")
    render(12, cad.PHI["7-R"], False, views=((20, -55),), fname="out/render_7th.png", title="v0.2 소형 — 7단")
