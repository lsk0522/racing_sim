// LSK H/SEQ 쉬프터 — 기어 판정 로직 (제작: Sunggwan)
// 하드웨어 의존 없음 → PC에서 단위 테스트 (test/test_logic.cpp)
#pragma once
#include <stdint.h>
#include <math.h>

namespace lsk {

// ---- 기하 기본값 (CAD v0.4~v0.6 공통) ----
// 게이트별 좌우 각도 (deg, + = 왼쪽). gate0(1-2) gate1(3-4, 기본 위치) gate2(5-6) gate3(7-R)
static const float GATE_PHI_DEFAULT[4] = {15.26f, 0.0f, -15.26f, -28.61f};
static const float THROW_DEG = 14.0f;       // 앞뒤 금속 스톱
static const float SEQ_STOP_DEG = 8.0f;     // 시퀀셜 잠금 포켓 스톱

enum Layout : uint8_t { LAYOUT_STANDARD = 0, LAYOUT_DOGLEG = 1 };

// 기어 번호: 0 = 중립, 1..7, 8 = R   [레이아웃][게이트][0=앞(+θ), 1=뒤(−θ)]
static const uint8_t GEAR_MAP[2][4][2] = {
  { {1, 2}, {3, 4}, {5, 6}, {7, 8} },   // 표준: 1-2 | 3-4 | 5-6 | 7-R
  { {8, 1}, {2, 3}, {4, 5}, {6, 7} },   // 도그레그: R-1 | 2-3 | 4-5 | 6-7
};

// 판정 파라미터 (설정 프로토콜로 바꿀 수 있음)
struct Params {
  float gatePhi[4] = {15.26f, 0.0f, -15.26f, -28.61f};
  float gateHyst = 1.2f;   // 게이트 경계 히스테리시스 (deg)
  float gearOn = 9.5f;     // 기어 판정 각도 (싱크로 고개 넘어 빨려 들어가는 구간)
  float gearOff = 8.0f;    // 기어 해제 각도
  float seqOn = 7.0f;      // 시퀀셜: 클릭(6.4°) 직후, 스톱(8°) 전
  float seqOff = 3.5f;
  uint8_t layout = LAYOUT_STANDARD;
};

struct State {
  int8_t gate = 1;      // 현재 게이트 (히스테리시스 적용)
  uint8_t gear = 0;     // 출력 기어
  int8_t seqLatch = 0;  // +1 미는 중, −1 당기는 중, 0 없음
};

// 좌우 각도 → 게이트 (히스테리시스). 게이트 각도는 정렬돼 있지 않아도 됨
inline int8_t decodeGate(float phi, int8_t prev, const Params &p) {
  int8_t best = 0; float bestD = 1e9f;
  for (int8_t i = 0; i < 4; i++) {
    float d = fabsf(phi - p.gatePhi[i]);
    if (d < bestD) { bestD = d; best = i; }
  }
  if (best == prev) return prev;
  float boundary = 0.5f * (p.gatePhi[best] + p.gatePhi[prev]);
  return (fabsf(phi - boundary) >= p.gateHyst) ? best : prev;
}

// H 모드: θ(+ 앞), φ(+ 왼쪽) → 기어. 게이트는 중립에서만 바뀜 (실제 게이트도 그렇고 노이즈 차단)
inline uint8_t updateH(State &s, float theta, float phi, const Params &p) {
  float a = fabsf(theta);
  if (s.gear == 0) {
    s.gate = decodeGate(phi, s.gate, p);
    if (a > p.gearOn) s.gear = GEAR_MAP[p.layout & 1][s.gate][theta > 0 ? 0 : 1];
  } else if (a < p.gearOff) {
    s.gear = 0;
  }
  return s.gear;
}

// 시퀀셜: 반환 +1(밀기 순간), −1(당기기 순간), 0
inline int8_t updateSeq(State &s, float theta, const Params &p) {
  int8_t edge = 0;
  if (s.seqLatch == 0) {
    if (theta > p.seqOn) { s.seqLatch = 1; edge = 1; }
    else if (theta < -p.seqOn) { s.seqLatch = -1; edge = -1; }
  } else if (fabsf(theta) < p.seqOff) {
    s.seqLatch = 0;
  }
  return edge;
}

// 센서 raw(0..cpr-1) → 영점 기준 각도 (deg, −180..180). AS5600 cpr=4096, MT6701 cpr=16384
inline float rawToDeg(uint16_t raw, uint16_t zero, int8_t dir, int32_t cpr = 4096) {
  int32_t d = (int32_t)raw - (int32_t)zero;
  while (d > cpr / 2) d -= cpr;
  while (d < -cpr / 2) d += cpr;
  return dir * d * (360.0f / (float)cpr);
}

// ±range° → HID 16비트 축 (−32767..32767)
inline int16_t axis16(float deg, float range) {
  float v = deg / range * 32767.0f;
  if (v > 32767.0f) v = 32767.0f;
  if (v < -32767.0f) v = -32767.0f;
  return (int16_t)lrintf(v);
}

// One-Euro 필터 (Casiez 2012): 정지 시 떨림은 강하게, 빠르게 움직일 때는 지연이 적게
struct OneEuro {
  float minCut = 4.0f, beta = 0.08f, dCut = 20.0f;   // Hz, 1/deg, Hz
  float x = 0, dx = 0; bool init = false;
  static float alpha(float cut, float dt) { float tau = 1.0f / (6.2831853f * cut); return 1.0f / (1.0f + tau / dt); }
  float push(float v, float dt) {
    if (!init) { x = v; dx = 0; init = true; return x; }
    float d = (v - x) / dt;
    dx += alpha(dCut, dt) * (d - dx);
    float cut = minCut + beta * fabsf(dx);
    x += alpha(cut, dt) * (v - x);
    return x;
  }
};

}  // namespace lsk
