"""v0.4 interference check — all pairs at every pose."""
import itertools, math, time, pickle, os
import numpy as np
from build123d import *
import cad, cam as camgen

st, cr, hb, fo, plates2d, camH = cad.build_all()

TH, PSI, _ = camgen.target_path(camgen.H_PROFILE)
IGNORE = {frozenset(p) for p in [
    ("plunger_S1_L", "tail_bolt"), ("plunger_S1_R", "tail_bolt"), ("plunger_S2", "tail_bolt"),
    ("hollow_pin", "cradle"), ("hollow_pin", "pin_collar"),
]}


def follower_at(theta):
    ps = float(np.interp(theta, TH, PSI))
    piv = Axis((cad.ARM_P[0], 0, cad.ARM_P[1]), (0, 1, 0))
    return {k: v.rotate(piv, -ps) for k, v in fo.items()}


def mode_parts(seq):
    return {"seq_lock": cad.make_seq_lock(seq), "mode_shaft": cad.make_mode_shaft(seq), "mode_knob": cad.make_mode_knob(seq)}


_mode_cache = {}


def parts_at(theta, phi, seq):
    if seq not in _mode_cache:
        _mode_cache[seq] = mode_parts(seq)
    return {**st, **_mode_cache[seq], **cad.pose(cr, theta, 0), **cad.pose(hb, theta, phi, "hub"), **follower_at(theta)}


def check(theta, phi, seq):
    P = parts_at(theta, phi, seq)
    names = list(P)
    bbs = {k: P[k].bounding_box() for k in names}
    hits = []
    for a, b in itertools.combinations(names, 2):
        if frozenset((a, b)) in IGNORE or not bbs[a].overlaps(bbs[b]):
            continue
        try:
            v = (P[a] & P[b]).volume
        except Exception:
            v = -1
        if v > 1.0:
            hits.append((a, b, round(v, 1)))
    print(f"θ={theta:+5.1f} φ={phi:+6.2f} {'SEQ' if seq else 'H  '} → {hits if hits else 'OK'}", flush=True)
    return hits


if __name__ == "__main__":
    t0 = time.time()
    poses = [(0, 0, False)]
    for ph in cad.PHI.values():
        for th in (-cad.THETA_STOP, -7, 7, cad.THETA_STOP):
            poses.append((th, ph, False))
    poses += [(0, ph, False) for ph in cad.PHI.values() if ph != 0]
    poses += [(-7.8, 0, True), (-4, 0, True), (0, 0, True), (4, 0, True), (7.8, 0, True)]
    bad = 0
    for p in poses:
        bad += bool(check(*p))
    print(f"{len(poses)} poses, {bad} with interference, {time.time()-t0:.0f}s")
