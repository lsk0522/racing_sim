// LSK H/SEQ 쉬프터 — USB 시리얼 설정 프로토콜 (제작: Sunggwan)
// 한 줄 명령 → 한 줄 JSON 응답. CLI(tools/lsk_shifter.py)와 웹(tools/web/index.html)이 같은 프로토콜을 씀.
// 하드웨어 의존 없음 → PC에서 단위 테스트
//
//   info                 장치 정보            → {"ok":true,"product":...,"fw":...,"maker":...,"proto":1,...}
//   get                  설정 전체            → {"ok":true,"cfg":{...}}
//   keys                 설정 키 목록·범위·설명
//   set <키> <값>         설정 변경 (RAM, save 해야 유지)
//   save / load          플래시 저장 / 다시 읽기
//   defaults             판정·출력 설정 기본값 (캘리브레이션은 유지)
//   factory              전부 초기화 (캘리브레이션 포함)
//   cal zero             지금 위치를 중립으로 (레버를 3-4 게이트 중립에 두고)
//   cal gate <0-3>       지금 좌우 각도를 해당 게이트 중심으로 (그 게이트 중립에 두고)
//   cal throw            앞 스톱에 댄 상태 → 앞뒤 배율 보정 (스톱 = 14°)
//   cal gear1            1단(표준: 왼쪽 앞)에 넣은 상태 → 앞뒤·좌우 배율 보정 (버튼 2단계 캘리브레이션과 같음)
//   status               실시간 값 한 번     → {"s":{...}}
//   stream <Hz>          실시간 값 계속 보내기 (0 = 끔, 최대 500)
//   reboot / bootsel     재부팅 / 펌웨어 업데이트 모드(RPI-RP2 드라이브)
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "config.h"

namespace lsk {

static const int PROTO_VERSION = 1;

struct Live {                       // 펌웨어가 채워 주는 실시간 값
  uint32_t ms = 0;
  uint16_t rawP = 0, rawR = 0;
  float theta = 0, phi = 0;         // 필터·배율 적용 후 (deg)
  uint8_t gate = 1, gear = 0, seq = 0;
  int8_t seqLatch = 0;
  bool okP = true, okR = true;
  uint16_t errP = 0, errR = 0;      // 누적 읽기 실패
  uint32_t loopHz = 0, sensHz = 0;
};

struct Info { const char *product, *fw, *maker, *sensor; int32_t cpr; };

struct Actions {                    // 하드웨어 동작 (펌웨어에서 연결, 테스트에서는 가짜)
  bool (*save)(const Config &) = nullptr;
  bool (*load)(Config &) = nullptr;
  void (*reboot)() = nullptr;
  void (*bootsel)() = nullptr;
};

inline int liveJson(const Live &L, char *o, size_t n) {
  return snprintf(o, n,
    "{\"s\":{\"t\":%lu,\"th\":%.3f,\"ph\":%.3f,\"gate\":%d,\"gear\":%d,\"mode\":\"%s\",\"latch\":%d,"
    "\"raw\":[%u,%u],\"ok\":[%d,%d],\"err\":[%u,%u],\"hz\":[%lu,%lu]}}",
    (unsigned long)L.ms, L.theta, L.phi, L.gate, L.gear, L.seq ? "SEQ" : "H", L.seqLatch,
    L.rawP, L.rawR, L.okP, L.okR, L.errP, L.errR, (unsigned long)L.loopHz, (unsigned long)L.sensHz);
}

inline int err(char *o, size_t n, const char *msg) { return snprintf(o, n, "{\"ok\":false,\"err\":\"%s\"}", msg); }
inline int ok(char *o, size_t n, const char *extra = nullptr) {
  return extra ? snprintf(o, n, "{\"ok\":true,%s}", extra) : snprintf(o, n, "{\"ok\":true}");
}

// 버튼 캘리브레이션과 공용: 1단에 넣은 상태에서 배율 보정
inline const char *calGear1(Config &c, const Live &L) {
  if (fabsf(L.theta) < 7 || fabsf(L.phi) < 6) return "움직임 부족 — 1단에 끝까지 넣고 다시";
  float tgt = (c.p.layout == LAYOUT_STANDARD) ? THROW_DEG : -THROW_DEG;   // 도그레그 1단 = gate0 뒤
  c.kP *= tgt / L.theta;
  c.kR *= c.p.gatePhi[0] / L.phi;
  return nullptr;
}

// 한 줄 처리. stream은 in/out (Hz). 반환: 응답 길이
inline int handleLine(const char *line, Config &cfg, const Live &L, const Info &I, const Actions &A,
                      int &streamHz, char *o, size_t n) {
  char buf[96];
  strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
  for (char *p = buf; *p; p++) if (*p == '\r' || *p == '\n') *p = 0;
  char *tok[4] = {0}; int nt = 0;
  for (char *s = strtok(buf, " \t"); s && nt < 4; s = strtok(nullptr, " \t")) tok[nt++] = s;
  if (nt == 0) return 0;
  const char *cmd = tok[0];

  if (!strcmp(cmd, "info")) {
    return snprintf(o, n, "{\"ok\":true,\"product\":\"%s\",\"fw\":\"%s\",\"maker\":\"%s\",\"proto\":%d,"
                    "\"sensor\":\"%s\",\"cpr\":%ld,\"keys\":%d}",
                    I.product, I.fw, I.maker, PROTO_VERSION, I.sensor, (long)I.cpr, NKEYS);
  }
  if (!strcmp(cmd, "get")) {
    char t[900]; cfgJson(cfg, t, sizeof(t));
    t[strlen(t) - 1] = 0;                        // 바깥 } 제거 → ok와 합침
    return snprintf(o, n, "{\"ok\":true,%s}", t + 1);
  }
  if (!strcmp(cmd, "keys")) {
    int w = snprintf(o, n, "{\"ok\":true,\"keys\":[");
    for (int i = 0; i < NKEYS && w < (int)n; i++)
      w += snprintf(o + w, n - w, "%s{\"k\":\"%s\",\"lo\":%g,\"hi\":%g,\"int\":%d,\"help\":\"%s\"}", i ? "," : "",
                    KEYS[i].name, KEYS[i].lo, KEYS[i].hi, KEYS[i].t != K_F, KEYS[i].help);
    if (w < (int)n) w += snprintf(o + w, n - w, "]}");
    return w;
  }
  if (!strcmp(cmd, "set")) {
    if (nt < 3) return err(o, n, "사용법: set <키> <값>");
    const Key *k = findKey(tok[1]);
    if (!k) return err(o, n, "없는 키");
    int r = keySet(cfg, *k, tok[2]);
    if (r == 1) return err(o, n, "숫자가 아님");
    if (r == 2) return err(o, n, "범위 밖");
    if (r == 3) return err(o, n, "관계 오류 (off < on, 배율 ≠ 0)");
    char t[64]; snprintf(t, sizeof(t), "\"%s\":%.4g", k->name, keyGet(cfg, *k));
    return ok(o, n, t);
  }
  if (!strcmp(cmd, "save")) { cfgSeal(cfg); return (A.save && A.save(cfg)) ? ok(o, n) : err(o, n, "저장 실패"); }
  if (!strcmp(cmd, "load")) {
    Config t;
    if (A.load && A.load(t) && cfgValid(t)) { cfg = t; return ok(o, n); }
    return err(o, n, "저장된 설정 없음");
  }
  if (!strcmp(cmd, "defaults")) {
    Config d; d.zeroP = cfg.zeroP; d.zeroR = cfg.zeroR; d.kP = cfg.kP; d.kR = cfg.kR;   // 캘리브레이션 유지
    for (int i = 0; i < 4; i++) d.p.gatePhi[i] = cfg.p.gatePhi[i];
    cfg = d; return ok(o, n);
  }
  if (!strcmp(cmd, "factory")) { cfg = Config(); return ok(o, n); }
  if (!strcmp(cmd, "cal")) {
    if (nt < 2) return err(o, n, "사용법: cal zero|gate N|throw|gear1");
    if (!strcmp(tok[1], "zero")) {
      if (!L.okP || !L.okR) return err(o, n, "센서 오류");
      cfg.zeroP = L.rawP; cfg.zeroR = L.rawR;
      char t[48]; snprintf(t, sizeof(t), "\"zero_p\":%u,\"zero_r\":%u", L.rawP, L.rawR); return ok(o, n, t);
    }
    if (!strcmp(tok[1], "gate")) {
      if (nt < 3) return err(o, n, "사용법: cal gate 0-3");
      int g = atoi(tok[2]);
      if (g < 0 || g > 3) return err(o, n, "게이트 번호 0-3");
      if (fabsf(L.theta) > 4) return err(o, n, "중립(앞뒤 ±4° 안)에서 하세요");
      cfg.p.gatePhi[g] = L.phi;
      char t[48]; snprintf(t, sizeof(t), "\"gate%d\":%.3f", g, L.phi); return ok(o, n, t);
    }
    if (!strcmp(tok[1], "throw")) {
      if (fabsf(L.theta) < 7) return err(o, n, "앞 스톱까지 밀고 하세요");
      cfg.kP *= THROW_DEG / L.theta;
      char t[48]; snprintf(t, sizeof(t), "\"k_p\":%.4f", cfg.kP); return ok(o, n, t);
    }
    if (!strcmp(tok[1], "gear1")) {
      const char *e = calGear1(cfg, L);
      if (e) return err(o, n, e);
      char t[64]; snprintf(t, sizeof(t), "\"k_p\":%.4f,\"k_r\":%.4f", cfg.kP, cfg.kR); return ok(o, n, t);
    }
    return err(o, n, "cal zero|gate N|throw|gear1");
  }
  if (!strcmp(cmd, "status")) return liveJson(L, o, n);
  if (!strcmp(cmd, "stream")) {
    int hz = nt > 1 ? atoi(tok[1]) : 0;
    if (hz < 0 || hz > 500) return err(o, n, "0~500 Hz");
    streamHz = hz;
    char t[24]; snprintf(t, sizeof(t), "\"stream\":%d", hz); return ok(o, n, t);
  }
  if (!strcmp(cmd, "reboot")) { if (A.reboot) A.reboot(); return ok(o, n); }
  if (!strcmp(cmd, "bootsel")) { if (A.bootsel) A.bootsel(); return ok(o, n); }
  if (!strcmp(cmd, "help")) {
    return ok(o, n, "\"cmds\":[\"info\",\"get\",\"keys\",\"set\",\"save\",\"load\",\"defaults\",\"factory\","
                    "\"cal zero\",\"cal gate N\",\"cal throw\",\"cal gear1\",\"status\",\"stream Hz\",\"reboot\",\"bootsel\"]");
  }
  return err(o, n, "모르는 명령 (help)");
}

}  // namespace lsk
