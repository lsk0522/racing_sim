// LSK H/SEQ Shifter — gear decode logic (no hardware dependency → can be unit-tested on a PC)
#pragma once
#include <stdint.h>
#include <math.h>

namespace lsk {

// ---- Geometry (same as the CAD) ----
// Lateral angle of each gate (deg, + = left). Order: gate0(1-2) gate1(3-4, rest) gate2(5-6) gate3(7-R)
static const float GATE_PHI[4] = {8.20f, 0.0f, -8.20f, -16.07f};
static const float GATE_HYST   = 0.8f;   // gate-boundary hysteresis (deg)
static const float GEAR_ON     = 8.0f;   // gear engage angle (past the synchro peak, inside the pull-in region)
static const float GEAR_OFF    = 6.5f;   // gear release angle
static const float SEQ_ON      = 7.6f;   // sequential: just past the click hump
static const float SEQ_OFF     = 4.0f;

enum Layout : uint8_t { LAYOUT_STANDARD = 0, LAYOUT_DOGLEG = 1 };

// Gear numbers: 0 = neutral, 1..7, 8 = R
// [layout][gate][0=front(+θ),1=rear(−θ)]
static const uint8_t GEAR_MAP[2][4][2] = {
  { {1, 2}, {3, 4}, {5, 6}, {7, 8} },   // standard: 1-2 | 3-4 | 5-6 | 7-R
  { {8, 1}, {2, 3}, {4, 5}, {6, 7} },   // dogleg: R-1 | 2-3 | 4-5 | 6-7 (Pagani-style dog-leg 1st)
};

struct State {
  int8_t gate = 1;      // currently selected gate (with hysteresis)
  uint8_t gear = 0;     // output gear
  int8_t seqLatch = 0;  // +1 push held, −1 pull held, 0 none
};

// Lateral angle → gate (with hysteresis)
inline int8_t decodeGate(float phi, int8_t prev) {
  int8_t best = 0; float bestD = 1e9f;
  for (int8_t i = 0; i < 4; i++) {
    float d = fabsf(phi - GATE_PHI[i]);
    if (d < bestD) { bestD = d; best = i; }
  }
  if (best == prev) return prev;
  // Only switch when we're at least GATE_HYST past the boundary
  float boundary = 0.5f * (GATE_PHI[best] + GATE_PHI[prev]);
  float margin = fabsf(phi - boundary);
  return (margin >= GATE_HYST) ? best : prev;
}

// H mode: θ (+ forward), φ (+ left) → gear
inline uint8_t updateH(State &s, float theta, float phi, Layout lay) {
  float a = fabsf(theta);
  // Only allow a gate change while in neutral (the real gate does this too — reject sensor noise)
  if (s.gear == 0) s.gate = decodeGate(phi, s.gate);
  if (s.gear == 0) {
    if (a > GEAR_ON) s.gear = GEAR_MAP[lay][s.gate][theta > 0 ? 0 : 1];
  } else {
    if (a < GEAR_OFF) s.gear = 0;
  }
  return s.gear;
}

// SEQ mode: returns +1 (push edge), −1 (pull edge), 0
inline int8_t updateSeq(State &s, float theta) {
  int8_t edge = 0;
  if (s.seqLatch == 0) {
    if (theta > SEQ_ON) { s.seqLatch = 1; edge = 1; }
    else if (theta < -SEQ_ON) { s.seqLatch = -1; edge = -1; }
  } else if (fabsf(theta) < SEQ_OFF) {
    s.seqLatch = 0;
  }
  return edge;
}

// AS5600 raw (0..4095) → angle relative to zero (deg, −180..180)
inline float rawToDeg(uint16_t raw, uint16_t zero, int8_t dir) {
  int32_t d = (int32_t)raw - (int32_t)zero;
  while (d > 2048) d -= 4096;
  while (d < -2048) d += 4096;
  return dir * d * (360.0f / 4096.0f);
}

}  // namespace lsk
