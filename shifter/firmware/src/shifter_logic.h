// LSK H/SEQ 쉬프터 — 기어 판정 로직 (제작: Sunggwan) (하드웨어 의존 없음 → PC에서 단위 테스트 가능)
#pragma once
#include <stdint.h>
#include <math.h>

namespace lsk {

// ---- 기하 (CAD v0.4와 동일) ----
// 게이트별 좌우 각도 (deg, + = 왼쪽). 순서: gate0(1-2) gate1(3-4, 기본 위치) gate2(5-6) gate3(7-R)
static const float GATE_PHI[4] = {15.26f, 0.0f, -15.26f, -28.61f};
static const float GATE_HYST   = 1.2f;   // 게이트 경계 히스테리시스 (deg)
static const float GEAR_ON     = 9.5f;   // 기어 판정 각도 (싱크로 고개를 넘어 빨려 들어가는 구간)
static const float GEAR_OFF    = 8.0f;   // 기어 해제 각도
static const float SEQ_ON      = 7.0f;   // 시퀀셜: 클릭(6.4°) 직후, 잠금 포켓 스톱(8°) 전
static const float SEQ_OFF     = 3.5f;

enum Layout : uint8_t { LAYOUT_STANDARD = 0, LAYOUT_DOGLEG = 1 };

// 기어 번호: 0 = 중립, 1..7, 8 = R
// [레이아웃][게이트][0=앞(+θ), 1=뒤(−θ)]
static const uint8_t GEAR_MAP[2][4][2] = {
  { {1, 2}, {3, 4}, {5, 6}, {7, 8} },   // 표준: 1-2 | 3-4 | 5-6 | 7-R
  { {8, 1}, {2, 3}, {4, 5}, {6, 7} },   // 도그레그: R-1 | 2-3 | 4-5 | 6-7 (파가니식 1단 도그레그)
};

struct State {
  int8_t gate = 1;      // 현재 게이트 (히스테리시스 적용)
  uint8_t gear = 0;     // 출력 기어
  int8_t seqLatch = 0;  // +1 미는 중, −1 당기는 중, 0 없음
};

// 좌우 각도 → 게이트 (히스테리시스)
inline int8_t decodeGate(float phi, int8_t prev) {
  int8_t best = 0; float bestD = 1e9f;
  for (int8_t i = 0; i < 4; i++) {
    float d = fabsf(phi - GATE_PHI[i]);
    if (d < bestD) { bestD = d; best = i; }
  }
  if (best == prev) return prev;
  float boundary = 0.5f * (GATE_PHI[best] + GATE_PHI[prev]);
  return (fabsf(phi - boundary) >= GATE_HYST) ? best : prev;
}

// H 모드: θ(+ 앞), φ(+ 왼쪽) → 기어
inline uint8_t updateH(State &s, float theta, float phi, Layout lay) {
  float a = fabsf(theta);
  // 게이트 변경은 중립에서만 (실제 게이트도 그렇고, 센서 노이즈 차단)
  if (s.gear == 0) s.gate = decodeGate(phi, s.gate);
  if (s.gear == 0) {
    if (a > GEAR_ON) s.gear = GEAR_MAP[lay][s.gate][theta > 0 ? 0 : 1];
  } else {
    if (a < GEAR_OFF) s.gear = 0;
  }
  return s.gear;
}

// 시퀀셜 모드: +1(밀기 순간), −1(당기기 순간), 0
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

// 센서 raw(0..cpr-1) → 영점 기준 각도 (deg, −180..180). AS5600 cpr=4096, MT6701 cpr=16384
inline float rawToDeg(uint16_t raw, uint16_t zero, int8_t dir, int32_t cpr = 4096) {
  int32_t d = (int32_t)raw - (int32_t)zero;
  while (d > cpr / 2) d -= cpr;
  while (d < -cpr / 2) d += cpr;
  return dir * d * (360.0f / (float)cpr);
}

}  // namespace lsk
