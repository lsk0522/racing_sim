"""
Detent cam generator — back-calculates the cam profile from the target knob force curve (virtual work method).

Coordinates (XZ plane, cam rotation axis = Y axis = origin)
  - Cam rotates with the cradle by the pitch angle θ.
  - Follower: swing arm. Pivot P, roller (608ZZ, R=11) at the arm tip, compression spring further out pushes the arm up (toward the cam).
  - Virtual work: T(θ)·dθ = dU(ψ)  (U = spring energy, ψ = arm angle)
    → U(ψ(θ)) = ∫T dθ  → solve for ψ(θ) → roller centre path → cam = (region inside path) − (roller discs)
"""
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from shapely import affinity

# ---------------- Parameters ----------------
L_KNOB = 190.0          # pivot → knob centre (mm). Default lever length
RR = 8.0                # roller radius (625ZZ OD 16, 2 side by side = 10 mm wide)
P = np.array([-60.0, -95.0])   # arm pivot (mm)
L_ARM = 60.0            # pivot → roller centre
A_SPR = 80.0            # pivot → spring contact point
K_SPR = 20.0            # spring rate N/mm
F0_SPR = 112.0          # spring preload at ψ=0 (N) — the screw adjusts this
THETA_EXT = 13.5        # cam generation range (deg)
STOP_DEG = 12.0         # pitch hard stop (H gear position) — stop screw

# Target torque at the cam axis T(θ) [N·m] (θ ≥ 0, mirrored for θ < 0)
# Knob force = T / (L_KNOB/1000)
H_PROFILE = [  # deg, Nm
    (0.0, 0.0),
    (0.8, 1.3),    # neutral detent — a firm hold in neutral
    (2.0, 0.5),    # free travel toward the gate
    (3.0, 0.6),
    (5.2, 3.8),    # synchro hump (≈20 N at the knob)
    (5.8, 3.8),
    (8.0, -1.5),   # past the peak it pulls the lever in → "thunk" into gear
    (9.0, -2.0),
    (11.5, -1.6),
    (12.4, -1.6),  # pressed against the 12.0° stop screw (≈8.4 N to pull it out of gear)
    (12.9, 5.0),   # beyond the stop (safety wall)
    (13.5, 5.0),
]
SEQ_PROFILE = [
    (0.0, 0.0),
    (0.8, 1.5),    # centre detent
    (2.0, 1.0),
    (6.5, 2.2),    # rising spring force
    (7.3, 3.8),    # click hump
    (8.5, 1.2),    # drops right after the click → "click"
    (9.3, 2.0),
    (10.5, 5.0),   # end-of-travel wall (always positive = springs back to centre)
    (13.5, 5.0),
]


def torque_fn(profile):
    d = np.array([p[0] for p in profile])
    t = np.array([p[1] for p in profile])
    return lambda th: np.sign(th) * np.interp(np.abs(th), d, t)


def roller_center(psi_deg):
    psi = np.radians(psi_deg)
    return P + L_ARM * np.array([np.cos(psi), np.sin(psi)])


def spring_energy(psi_deg):
    """ψ<0 (arm pushed down) compresses the spring. Energy in N·mm"""
    delta = -A_SPR * np.sin(np.radians(psi_deg))  # extra compression (mm)
    return F0_SPR * delta + 0.5 * K_SPR * delta ** 2


def spring_force(psi_deg):
    delta = -A_SPR * np.sin(np.radians(psi_deg))
    return F0_SPR + K_SPR * delta


def psi_from_energy(E):
    # F0·δ + k/2·δ² = E  → δ
    delta = (-F0_SPR + np.sqrt(F0_SPR ** 2 + 2 * K_SPR * E)) / K_SPR
    return -np.degrees(np.arcsin(delta / A_SPR))


def target_path(profile, n=1101):
    T = torque_fn(profile)
    th = np.linspace(0, THETA_EXT, n)
    tq = T(th) * 1000.0  # N·mm
    E = np.concatenate([[0], np.cumsum(0.5 * (tq[1:] + tq[:-1]) * np.diff(np.radians(th)))])
    if E.min() < -F0_SPR ** 2 / (2 * K_SPR) + 1:
        raise ValueError("Energy is too negative — spring preload is insufficient")
    psi = psi_from_energy(E)
    th_full = np.concatenate([-th[::-1], th[1:]])
    psi_full = np.concatenate([psi[::-1], psi[1:]])
    return th_full, psi_full, E


def to_cam_frame(pt, theta_deg):
    """Base-frame point → cam frame (rotate back by −θ)."""
    a = np.radians(-theta_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([c * pt[0] - s * pt[1], s * pt[0] + c * pt[1]])


def path_min_convex_radius(centers):
    """Minimum curvature radius at points where the path bends toward the axis (convex outward) — must be ≥ RR."""
    x, z = centers[:, 0], centers[:, 1]
    dx, dz = np.gradient(x), np.gradient(z)
    ddx, ddz = np.gradient(dx), np.gradient(dz)
    k = (dx * ddz - dz * ddx) / (dx ** 2 + dz ** 2) ** 1.5
    # Curvature sign for a path going CW around the axis: positive → curves toward axis
    r = np.where(np.abs(k) > 1e-9, 1 / np.abs(k), np.inf)
    toward = k < 0
    return np.min(np.where(toward, r, np.inf))


def build_cam(profile, inner_r=45.0):
    th, psi, _ = target_path(profile)
    centers = np.array([to_cam_frame(roller_center(p), t) for t, p in zip(th, psi)])
    build_cam.min_r = path_min_convex_radius(centers)
    # Region inside the path: path + closed toward the axis
    a0 = np.radians(-90 - THETA_EXT - 6)
    a1 = np.radians(-90 + THETA_EXT + 6)
    inner = [(inner_r * np.cos(a), inner_r * np.sin(a)) for a in np.linspace(a0, a1, 40)]
    # Extend both ends radially
    ends = []
    for idx in (0, -1):
        c = centers[idx]
        ends.append(c)
    ring = list(map(tuple, centers)) + inner
    region = Polygon(ring).buffer(0)
    discs = unary_union([Point(c).buffer(RR, 64) for c in centers[::2]])
    cam = region.difference(discs)
    # Remove the tail parts left at the end of the path (sector trim)
    sector = Polygon([(0, 0)] + [(200 * np.cos(a), 200 * np.sin(a))
                                 for a in np.linspace(np.radians(-90 - THETA_EXT - 4),
                                                      np.radians(-90 + THETA_EXT + 4), 60)])
    cam = cam.intersection(sector)
    if cam.geom_type == "MultiPolygon":
        cam = max(cam.geoms, key=lambda g: g.area)
    return cam.simplify(0.01)


def simulate(cam, th_range=(-12.6, 12.6), n=505):
    """Rotate the cam by θ and find the roller contact ψ → compute torque/knob force."""
    ths = np.linspace(*th_range, n)
    psis = []
    for t in ths:
        cam_b = affinity.rotate(cam, t, origin=(0, 0), use_radians=False)
        lo, hi = -20.0, 8.0  # lo: definitely clear, hi: penetrating
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            c = roller_center(mid)
            if Point(c).buffer(RR, 64).intersection(cam_b).area > 1e-6:
                hi = mid
            else:
                lo = mid
        psis.append(lo)
    psis = np.array(psis)
    U = spring_energy(psis)
    T = np.gradient(U, np.radians(ths)) / 1000.0  # N·m
    F_knob = T / (L_KNOB / 1000.0)
    # Pressure angle (approximate): direction of roller-centre motion vs. line of force
    return ths, psis, T, F_knob


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out = {}
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
    for name, prof, ax in (("H", H_PROFILE, axs[0]), ("SEQ", SEQ_PROFILE, axs[1])):
        cam = build_cam(prof)
        ths, psis, T, Fk = simulate(cam)
        tgt = torque_fn(prof)(ths) / (L_KNOB / 1000)
        ax.plot(ths, tgt, "--", label="target")
        ax.plot(ths, Fk, label="simulated")
        ax.axhline(0, color="k", lw=0.5)
        ax.set_title(f"{name} track: knob force (lever {L_KNOB:.0f} mm)")
        ax.set_xlabel("pitch angle (deg)")
        ax.set_ylabel("knob force (N)")
        ax.legend(); ax.grid(alpha=0.3)
        out[name] = (cam, ths, Fk, psis)
        m = (np.abs(ths) < 12.0)
        err = np.abs(Fk[m] - tgt[m])
        print(f"  path min convex radius {build_cam.min_r:.1f} mm (roller {RR})")
        print(f"{name}: peak {Fk.max():.1f} N, max err {err.max():.2f} N, mean err {err.mean():.2f} N, "
              f"ψ range {psis.min():.2f}..{psis.max():.2f} deg, max spring F {spring_force(psis.min()):.0f} N")
    fig.tight_layout()
    fig.savefig("/home/claude/shifter/out/feel_curves.png", dpi=130)
