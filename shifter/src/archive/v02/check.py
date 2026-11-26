"""Interference check + renders."""
import itertools, math, sys, time
import numpy as np
from build123d import *
import cad, cam as camgen

st, cr, hb, fo, gate2d, cams = cad.build_all()
camH, camS = cams


def follower_at(theta, seq):
    th, psi, _ = camgen.target_path(camgen.SEQ_PROFILE if seq else camgen.H_PROFILE)
    ps = float(np.interp(theta, th, psi))
    piv = Axis((cad.ARM_P[0], 0, cad.ARM_P[1]), (0, 1, 0))
    d = {}
    for k, v in fo.items():
        s = v
        if k in ("arm", "rollers"):
            # build123d rotate about +Y: x' = x cos + z sin. Arm angle ψ (in the XZ plane, +ψ = up) ⇒ use −ψ about +Y
            s = s.rotate(piv, -ps)
        if seq:
            s = s.moved(Location((0, -cad.MODE_TRAVEL, 0)))
        d[k] = s
    d["fork"] = cad.make_fork(seq)
    d["mode_crank"] = cad.make_mode_crank(seq)
    d["mode_knob"] = cad.make_mode_knob() if not seq else cad.make_mode_knob().rotate(Axis((0, cad.SHAFT_Y, cad.SHAFT_Z), (1, 0, 0)), 180)
    return d


def check(theta, phi, seq, verbose=True):
    crp = cad.pose(cr, theta, 0, "cradle")
    hbp = cad.pose(hb, theta, phi, "hub")
    fop = follower_at(theta, seq)
    groups = {"static": st, "cradle": {k: v for k, v in crp.items() if not k.startswith(("plunger", "cup"))},
              "hub": hbp, "follower": fop}
    hits = []
    names = list(groups)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            for ka, a in groups[names[i]].items():
                for kb, b in groups[names[j]].items():
                    if not a.bounding_box().overlaps(b.bounding_box()):
                        continue
                    try:
                        v = (a & b).volume
                    except Exception as e:
                        v = -1
                    if v > 1.0:   # mm³
                        hits.append((ka, kb, round(v, 1)))
    if verbose:
        print(f"θ={theta:+.1f} φ={phi:+.2f} {'SEQ' if seq else 'H  '} → {hits if hits else 'OK'}")
    return hits


if __name__ == "__main__":
    t0 = time.time()
    poses = [(0, 0, False)]
    for g, ph in cad.PHI.items():
        for th in (-cad.THETA_STOP, cad.THETA_STOP):
            poses.append((th, ph, False))
    poses += [(0, cad.PHI["1-2"], False), (0, cad.PHI["7-R"], False)]
    poses += [(-12, 0, True), (12, 0, True), (0, 0, True)]
    for p in poses:
        check(*p)
    print("done", round(time.time() - t0, 1), "s")
