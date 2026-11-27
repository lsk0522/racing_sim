# LSK H/SEQ Shifter — author: Sunggwan
"""
Tolerance tables per international standards (only the ranges this project uses; unit mm)
 - ISO 286-1/-2 : IT grades, fundamental deviations → fits (e.g. 'H7', 'M7', 'h6', 'g6')
 - ISO 2768-1   : general linear/angular tolerances (f, m, c, v)
 - ISO 2768-2   : general geometrical tolerances (H, K, L)
"""
import bisect

# Nominal size steps (upper limit, mm): (0,3], (3,6], (6,10], (10,18], (18,30], (30,50], (50,80], (80,120]
_STEPS = [3, 6, 10, 18, 30, 50, 80, 120]
IT = {  # µm
    5: [4, 5, 6, 8, 9, 11, 13, 15],
    6: [6, 8, 9, 11, 13, 16, 19, 22],
    7: [10, 12, 15, 18, 21, 25, 30, 35],
    8: [14, 18, 22, 27, 33, 39, 46, 54],
    9: [25, 30, 36, 43, 52, 62, 74, 87],
    10: [40, 48, 58, 70, 84, 100, 120, 140],
    11: [60, 75, 90, 110, 130, 160, 190, 220],
    12: [100, 120, 150, 180, 210, 250, 300, 350],
}
# Shaft fundamental deviations (µm). a–h: es (upper), j–zc: ei (lower)
SHAFT_ES = {"f": [-6, -10, -13, -16, -20, -25, -30, -36], "g": [-2, -4, -5, -6, -7, -9, -10, -12],
            "h": [0] * 8}
SHAFT_EI = {"k": [0, 1, 1, 1, 2, 2, 2, 3], "m": [2, 4, 6, 7, 8, 9, 11, 13], "n": [4, 8, 10, 12, 15, 17, 20, 23],
            "p": [6, 12, 15, 18, 22, 26, 32, 37]}
HOLE_EI = {"F": [6, 10, 13, 16, 20, 25, 30, 36], "G": [2, 4, 5, 6, 7, 9, 10, 12], "H": [0] * 8}


def _idx(size):
    i = bisect.bisect_left(_STEPS, size)
    if i >= len(_STEPS):
        raise ValueError(f"size {size} outside table")
    return i


def fit(size, code):
    """ISO 286: code e.g. 'H7', 'M7', 'h6', 'g6', 'js7' → (upper, lower) deviation [mm]."""
    letter = code.rstrip("0123456789"); g = int(code[len(letter):]); i = _idx(size)
    t = IT[g][i]
    if letter == "js" or letter == "JS":
        return (t / 2000, -t / 2000)
    if letter in SHAFT_ES:
        es = SHAFT_ES[letter][i]; return (es / 1000, (es - t) / 1000)
    if letter in SHAFT_EI:
        ei = SHAFT_EI[letter][i]
        if letter == "k" and (g > 7 or size <= 3):
            ei = 0
        return ((ei + t) / 1000, ei / 1000)
    if letter in HOLE_EI:
        ei = HOLE_EI[letter][i]; return ((ei + t) / 1000, ei / 1000)
    if letter in ("K", "M", "N", "P"):
        shaft = letter.lower()
        delta = (IT[g][i] - IT[g - 1][i]) if (g <= 8 and size > 3) else 0
        es = -SHAFT_EI[shaft][i] + delta
        if letter == "K" and g > 8:
            es = 0
        return (es / 1000, (es - t) / 1000)
    raise ValueError(code)


def fit_text(size, code, prefix="Ø"):
    u, l = fit(size, code)
    f = lambda v: "0" if abs(v) < 1e-9 else f"{v:+.3f}".replace("-", "−")
    return f"{prefix}{size:g} {code} ({f(u)} / {f(l)})", u, l


# ISO 2768-1 general linear tolerances (±mm), class m. Ranges: 0.5–3, 3–6, 6–30, 30–120, 120–400, 400–1000
ISO2768_1 = {
    "f": [0.05, 0.05, 0.1, 0.15, 0.2, 0.3],
    "m": [0.1, 0.1, 0.2, 0.3, 0.5, 0.8],
    "c": [0.2, 0.3, 0.5, 0.8, 1.2, 2.0],
}
ISO2768_1_RANGES = ["0.5~3", "3~6", "6~30", "30~120", "120~400", "400~1000"]
ISO2768_1_RADIUS_m = ["0.5~3: ±0.2", "3~6: ±0.5", ">6: ±1"]
ISO2768_1_ANGLE_m = ["~10: ±1°", "10~50: ±0°30′", "50~120: ±0°20′", "120~400: ±0°10′"]
# ISO 2768-2 class K
ISO2768_2_K = {
    "진직도·평면도": ["~10: 0.05", "10~30: 0.1", "30~100: 0.2", "100~300: 0.4"],
    "직각도": ["~100: 0.4", "100~300: 0.6"],
    "대칭도": ["~100: 0.6"],
    "원주 흔들림": ["0.2"],
}

if __name__ == "__main__":
    for s, c in [(16, "M7"), (16, "K7"), (16, "N7"), (8, "H7"), (8, "h6"), (5, "H7"), (5, "H8"), (11, "h9"), (5, "g6")]:
        print(fit_text(s, c)[0])
