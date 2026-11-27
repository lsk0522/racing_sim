// LSK H/SEQ 쉬프터 — 설정 (플래시 저장 형식 + 키 이름으로 읽기/쓰기) (제작: Sunggwan)
// 하드웨어 의존 없음 → PC에서 단위 테스트
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "shifter_logic.h"

namespace lsk {

static const uint32_t CFG_MAGIC = 0x4C534B36;   // "LSK6"
static const uint16_t CFG_VERSION = 2;

struct Config {
  uint32_t magic = CFG_MAGIC;
  uint16_t version = CFG_VERSION;
  uint16_t size = sizeof(Config);
  // 센서
  uint16_t zeroP = 0, zeroR = 0;   // 중립 raw
  float kP = 1.0f, kR = 1.0f;      // 각도 배율 (부호 = 방향)
  // 판정
  Params p;
  // 출력
  uint8_t seqSwap = 0;             // 시퀀셜 밀기/당기기 반전
  uint16_t seqPulseMs = 0;         // 0 = 누르고 있는 동안 버튼 유지, >0 = 고정 길이 펄스
  float axisRangeP = 16.0f;        // 축 범위 ±deg
  float axisRangeR = 32.0f;
  // 필터
  float fMinCut = 4.0f, fBeta = 0.08f;
  uint16_t i2cKHz = 400;
  uint32_t crc = 0;
};

inline uint32_t crc32(const uint8_t *d, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}
inline uint32_t cfgCrc(const Config &c) { return crc32((const uint8_t *)&c, offsetof(Config, crc)); }
inline void cfgSeal(Config &c) { c.magic = CFG_MAGIC; c.version = CFG_VERSION; c.size = sizeof(Config); c.crc = cfgCrc(c); }
inline bool cfgValid(const Config &c) {
  return c.magic == CFG_MAGIC && c.version == CFG_VERSION && c.size == sizeof(Config) && c.crc == cfgCrc(c);
}

// ---- 키 표 (이름 → 값). 범위 밖이면 거부 ----
enum KType : uint8_t { K_F, K_U8, K_U16 };
struct Key { const char *name; KType t; size_t off; float lo, hi; const char *help; };
#define LSK_OFF(f) offsetof(Config, f)
static const Key KEYS[] = {
  {"layout",      K_U8,  LSK_OFF(p.layout),      0, 1,      "0 표준(1-2|3-4|5-6|7-R) / 1 도그레그(R-1|2-3|4-5|6-7)"},
  {"gear_on",     K_F,   LSK_OFF(p.gearOn),      3, 13.5f,  "기어 들어감 판정 각도 (deg)"},
  {"gear_off",    K_F,   LSK_OFF(p.gearOff),     2, 13,     "기어 빠짐 판정 각도 (deg, gear_on보다 작게)"},
  {"gate_hyst",   K_F,   LSK_OFF(p.gateHyst),    0, 6,      "게이트 경계 히스테리시스 (deg)"},
  {"gate0",       K_F,   LSK_OFF(p.gatePhi[0]),  -45, 45,   "1-2 게이트 좌우 각도 (deg)"},
  {"gate1",       K_F,   LSK_OFF(p.gatePhi[1]),  -45, 45,   "3-4 게이트"},
  {"gate2",       K_F,   LSK_OFF(p.gatePhi[2]),  -45, 45,   "5-6 게이트"},
  {"gate3",       K_F,   LSK_OFF(p.gatePhi[3]),  -45, 45,   "7-R 게이트"},
  {"seq_on",      K_F,   LSK_OFF(p.seqOn),       2, 7.9f,   "시퀀셜 판정 각도 (deg)"},
  {"seq_off",     K_F,   LSK_OFF(p.seqOff),      0.5f, 7,   "시퀀셜 해제 각도 (deg)"},
  {"seq_swap",    K_U8,  LSK_OFF(seqSwap),       0, 1,      "시퀀셜 밀기/당기기 반전"},
  {"seq_pulse",   K_U16, LSK_OFF(seqPulseMs),    0, 500,    "시퀀셜 버튼 펄스 길이 ms (0 = 누르는 동안 유지)"},
  {"k_p",         K_F,   LSK_OFF(kP),            -5, 5,     "앞뒤 각도 배율 (부호 = 방향)"},
  {"k_r",         K_F,   LSK_OFF(kR),            -5, 5,     "좌우 각도 배율 (부호 = 방향)"},
  {"zero_p",      K_U16, LSK_OFF(zeroP),         0, 16383,  "앞뒤 중립 raw"},
  {"zero_r",      K_U16, LSK_OFF(zeroR),         0, 16383,  "좌우 중립 raw"},
  {"axis_p",      K_F,   LSK_OFF(axisRangeP),    5, 45,     "X축 범위 ±deg"},
  {"axis_r",      K_F,   LSK_OFF(axisRangeR),    5, 60,     "Y축 범위 ±deg"},
  {"f_mincut",    K_F,   LSK_OFF(fMinCut),       0.5f, 50,  "필터 최소 차단 주파수 Hz (작을수록 정지 떨림 적음)"},
  {"f_beta",      K_F,   LSK_OFF(fBeta),         0, 2,      "필터 속도 계수 (클수록 빠른 동작 지연 적음)"},
  {"i2c_khz",     K_U16, LSK_OFF(i2cKHz),        100, 1000, "센서 I2C 속도 kHz (재부팅 후 적용)"},
};
static const int NKEYS = sizeof(KEYS) / sizeof(KEYS[0]);

inline const Key *findKey(const char *n) {
  for (int i = 0; i < NKEYS; i++) if (strcmp(KEYS[i].name, n) == 0) return &KEYS[i];
  return nullptr;
}
inline float keyGet(const Config &c, const Key &k) {
  const uint8_t *b = (const uint8_t *)&c + k.off;
  if (k.t == K_F) { float v; memcpy(&v, b, 4); return v; }
  if (k.t == K_U8) return (float)*b;
  uint16_t v; memcpy(&v, b, 2); return (float)v;
}
// 반환: 0 성공, 1 숫자 아님, 2 범위 밖, 3 관계 오류(gear_off ≥ gear_on 등)
inline int keySet(Config &c, const Key &k, const char *s) {
  char *end; float v = strtof(s, &end);
  if (end == s || *end != 0) return 1;
  if (!(v >= k.lo && v <= k.hi)) return 2;
  Config t = c;
  uint8_t *b = (uint8_t *)&t + k.off;
  if (k.t == K_F) memcpy(b, &v, 4);
  else if (k.t == K_U8) *b = (uint8_t)lrintf(v);
  else { uint16_t u = (uint16_t)lrintf(v); memcpy(b, &u, 2); }
  if (t.p.gearOff >= t.p.gearOn || t.p.seqOff >= t.p.seqOn || t.kP == 0 || t.kR == 0) return 3;
  c = t;
  return 0;
}
// 설정 전체 → JSON
inline int cfgJson(const Config &c, char *out, size_t n) {
  int w = snprintf(out, n, "{\"cfg\":{");
  for (int i = 0; i < NKEYS && w < (int)n; i++) {
    float v = keyGet(c, KEYS[i]);
    if (KEYS[i].t == K_F) w += snprintf(out + w, n - w, "%s\"%s\":%.4g", i ? "," : "", KEYS[i].name, v);
    else w += snprintf(out + w, n - w, "%s\"%s\":%d", i ? "," : "", KEYS[i].name, (int)v);
  }
  if (w < (int)n) w += snprintf(out + w, n - w, "}}");
  return w;
}

}  // namespace lsk
