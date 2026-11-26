# LSK H/SEQ Shifter v0.1 — Design Notes

Personal sim-racing shifter. 7-speed + R H-pattern (standard / dogleg) ↔ sequential, parts budget 150,000 KRW (est. 114,000 KRW).

## 1. Design targets (what we took from the 4 brands)
| Reference | What it means here | How it shows up in the design |
|---|---|---|
| Pagani Utopia | Gated 7-speed manual, dogleg 1st, exposed linkage | Gate plate floating on 4 posts, skeletonized towers exposing the mechanism, **dogleg layout selectable in firmware** |
| Ferrari (gated manuals, 12Cilindri Manuale) | Metal open gate, narrow neutral channel, metallic click | **SUS304 3t laser-cut gate**, neutral channel = lever + 1.0 mm, steel lever bar hitting the steel gate |
| Porsche GT (GT4/GT3 shifter) | Short throw, no slack, lateral spring return | 608 bearing gimbal clamped by axle bolts (zero backlash), adjustable lever length, **preloaded lateral centering springs** |
| Bugatti Tourbillon | Machined aluminium, watch-like exposed mechanism | Visible cam/roller, round metal knob, metal-look parts |

## 2. Mechanism
- **Gimbal**: pitch (fore/aft, θ) = cradle on 2× 608ZZ in the towers. Roll (lateral, φ) = hub on 2× 608ZZ inside the cradle. Axes intersect at a single point.
- **Fore/aft feel = roller cam detent**: the cam (fixed to the cradle) + a swing-arm follower (2× 625ZZ roller) + die spring.
  - The cam profile is **back-calculated with virtual work (T·dθ = dU) from the target knob force curve** (`source/cam.py`).
  - H track: neutral detent ~7 N → synchro hump **~20 N @5.5°** → past the peak it **pulls the lever in** (−8 to −10 N) → metal stop at 12°.
  - SEQ track: centre detent → rising spring → **click hump ~20 N @7.3°** → wall → always springs back to centre.
  - Simulated vs. target error is ~0.6 N mean (`images/feel_curves.png`).
  - Limit: the path's curvature radius must be ≥ the roller radius, so the hump has to span 2–3°. That's why the roller is 625 (R8) and the cam radius is 95 mm.
- **Lateral feel = staged springs**: rests at the 3-4 gate (preload ≈5 N). 1-2/5-6 ≈9.5 N. Going to 7-R, the 2nd-stage spring (S2) adds in → ~30 N (effectively a reverse lockout).
- **Mode switch**: one mode lever → (1) the follower carriage slides 18 mm in Y (H track ↔ SEQ track), (2) a ramp lifts a fork that clamps the tail tip → lateral motion locked. Switch only in neutral.
- **Sensing**: 2× AS5600 (12-bit, contactless). Firmware maps θ/φ → gate/gear (hysteresis, gate locked while in gear).

## 3. Key dimensions
| Item | Value |
|---|---|
| Pivot → gate | 118 mm |
| Pivot → knob centre | 190 mm (adjustable) |
| Gate pitch (lateral) | 17 mm @ gate → 27 mm at the knob |
| Fore/aft throw | ±12° → ±40 mm at the knob |
| Lateral angles | 1-2 +8.2°, 3-4 0°, 5-6 −8.2°, 7-R −16.1° |
| Overall height | ~350 mm from the base bottom to the top of the knob |
| Base | 220 × 183 mm, M6 holes on a 40 mm grid (4040/8040 profile or desk-clamp plate) |

## 4. Verification done
- CAD interference check at 14 poses (neutral, every gear, each gate, SEQ ±12°): **no interference** (threshold 1 mm³) — `source/check.py`
- Cam simulation: knob force vs. target error 0.6 N mean, max spring force 184 N (die spring within ~37 % compression)
- Firmware: unit tests for gear decode logic (both layouts, hysteresis, gate lock in gear, SEQ edges) **ALL PASS**, sketch syntax check passed
- **Not done**: physical printing/assembly, real tolerances, board compile (couldn't install the Arduino toolchain in this environment)

## 5. Print guide
- Material: PETG or ASA (the Voron can do ASA). The cam (`cam_HS`) and `hub` at **100 % infill**.
- Structural parts (cradle, towers, arm, carriage): 6 walls, 50 % gyroid. Others: 4 walls, 30 %.
- STLs are already oriented for printing. The cradle needs supports under the box bottom.
- Bearing pockets 22.1 mm (608), 16 mm (625 — the arm holds them via the axle). Do a tolerance test print first.

## 6. Assembly order (summary)
1. Heat-set inserts: hub (M5 ×2), cradle (M4 ×2 on the −Y wall, M8 at the S2 end), arm (M6), towers (M5 stops, M3 sensor bracket)
2. Hub + lever bar + tail (M8×80, nut in the side slot) → roll axle M8×60 + 608 ×2 into the cradle
3. Lateral plungers/springs S1 ×2, S2 + cup + M8 set screw
4. Bolt the cam to the −Y face of the cradle (M4 ×2)
5. Towers onto the base → cradle (pitch axles: −Y M8×60, +Y M8×25 + spacer)
6. Carriage + arm (M5 pivot) + 2× 625 rollers + die spring → mode lever, fork, guide
7. Posts (M5 rod) + gate plate → knob
8. Wiring → calibration (neutral → 1st) → tuning

## 7. Tuning
| What | Adjust with | Effect |
|---|---|---|
| Fore/aft weight overall | M6 preload screw on top of the arm | Scales the whole curve |
| 7/R lockout | M8 screw at the cradle +Y end | Stage-2 spring preload |
| Gear landing feel | M5 stop screws ×2 on the +Y tower | Bare steel = solid metal feel / TPU cap = soft |
| Throw | Lever height in the hub | Shorter = shorter throw, heavier |
| Sound | Gate material | SUS = Ferrari click, printed = quiet |
| Feel character | `cam.py` H_PROFILE | Regenerate the cam to taste |

## 8. Known risks / next version
- Printed cam wear (PETG/ASA contact stress est. ~42 MPa) → if it wears, make a metal cam (DXF provided) in v0.2
- Mode-switch force est. ~60 N at the carriage → the 3:1 mode lever brings it down to ~20 N. Lower it with PTFE tape on the carriage if needed
- AS5600 board sizes/hole spacing vary between sellers → check the bracket hole positions against your actual board
- Height 350 mm — if that's too tall, shorten the lever bar and lower the posts and the gate together (H_GATE in `cad.py`)
