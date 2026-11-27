// LSK H/SEQ 쉬프터 — 판정 로직 · 설정 · 프로토콜 단위 테스트 (PC에서: g++ -std=c++17 test/test_logic.cpp -o t && ./t)
#include <cstdio>
#include <cstring>
#include <string>
#include "../src/shifter_logic.h"
#include "../src/config.h"
#include "../src/proto.h"
using namespace lsk;
int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); fails++; } } while (0)
Params P;   // 기본값
uint8_t go(State &s, float th, float ph, uint8_t lay = LAYOUT_STANDARD) { Params p = P; p.layout = lay; return updateH(s, th, ph, p); }

// ---- 가짜 하드웨어 ----
Config flash; bool flashOk = false;
bool fsave(const Config &c) { flash = c; flashOk = true; return true; }
bool fload(Config &c) { if (!flashOk) return false; c = flash; return true; }
std::string cmd(const char *line, Config &c, Live &L, int &hz) {
  static Info I = {"LSK HSEQ Shifter", "0.6.0", "Sunggwan", "MT6701", 16384};
  Actions A; A.save = fsave; A.load = fload;
  static char o[4096];
  int n = handleLine(line, c, L, I, A, hz, o, sizeof(o));
  return std::string(o, n > 0 ? n : 0);
}
bool has(const std::string &s, const char *t) { return s.find(t) != std::string::npos; }

int main() {
  // ---- 판정 로직 ----
  struct { float ph; float th; uint8_t g; } cases[] = {
    {15.26f, 14, 1}, {15.26f, -14, 2}, {0, 14, 3}, {0, -14, 4}, {-15.26f, 14, 5}, {-15.26f, -14, 6}, {-28.61f, 14, 7}, {-28.61f, -14, 8}};
  for (auto &c : cases) {
    State s;
    for (float p = 0; fabsf(p) < fabsf(c.ph); p += (c.ph > 0 ? 0.5f : -0.5f)) go(s, 0, p);
    go(s, 0, c.ph);
    uint8_t g = 0; for (float t = 0; fabsf(t) <= 14; t += (c.th > 0 ? 0.5f : -0.5f)) g = go(s, t, c.ph);
    char m[64]; sprintf(m, "std gear expect %d got %d", c.g, g); CHECK(g == c.g, m);
    for (float t = c.th; fabsf(t) > 0; t -= (c.th > 0 ? 0.5f : -0.5f)) g = go(s, t, c.ph);
    CHECK(go(s, 0, c.ph) == 0, "returns to neutral");
  }
  { State s; go(s, 0, 15.26f, LAYOUT_DOGLEG); uint8_t g = 0; for (float t = 0; t >= -14; t -= 0.5f) g = go(s, t, 15.26f, LAYOUT_DOGLEG); CHECK(g == 1, "dogleg 1st"); }
  { State s; go(s, 0, -28.61f, LAYOUT_DOGLEG); uint8_t g = 0; for (float t = 0; t <= 14; t += 0.5f) g = go(s, t, -28.61f, LAYOUT_DOGLEG); CHECK(g == 6, "dogleg 6th"); }
  { State s; for (int i = 0; i < 100; i++) { float p = -7.63f + ((i % 2) ? 0.5f : -0.5f); go(s, 0, p); } CHECK(s.gate == 1, "hysteresis keeps gate"); }
  { State s; go(s, 0, 0); for (float t = 0; t <= 14; t += 0.5f) go(s, t, 0);
    uint8_t g = go(s, 14, -12.0f); CHECK(g == 3, "gear held despite phi noise"); }
  { State s; uint8_t g = 0; for (float t = 0; t <= 9.4f; t += 0.1f) g = go(s, t, 0); CHECK(g == 0, "not engaged before gear_on"); }
  { State s; int edges = 0; for (float t = 0; t <= 7.9f; t += 0.1f) edges += (updateSeq(s, t, P) != 0);
    for (float t = 7.9f; t >= 0; t -= 0.1f) edges += (updateSeq(s, t, P) != 0);
    for (float t = 0; t >= -7.9f; t -= 0.1f) edges += (updateSeq(s, t, P) == -1);
    CHECK(edges == 2, "seq edges"); }
  { State s; int e = 0; for (float t = 0; t <= 6.9f; t += 0.1f) e += (updateSeq(s, t, P) != 0); CHECK(e == 0, "no seq edge before click"); }
  CHECK(fabsf(rawToDeg(10, 4090, 1) - (16 * 360.0f / 4096)) < 1e-3, "wrap");
  CHECK(fabsf(rawToDeg(20, 16380, 1, 16384) - (24 * 360.0f / 16384)) < 1e-4, "wrap14");
  // 캘리브레이션된 게이트 각도 (실물 오차) 반영
  { Params p = P; p.gatePhi[0] = 13.0f; State s; for (float x = 0; x <= 13; x += 0.5f) updateH(s, 0, x, p);
    uint8_t g = 0; for (float t = 0; t <= 14; t += 0.5f) g = updateH(s, t, 13.0f, p); CHECK(g == 1, "calibrated gate0"); }
  // 16비트 축
  CHECK(axis16(16, 16) == 32767 && axis16(-40, 16) == -32767 && axis16(0, 16) == 0, "axis16");

  // ---- One-Euro 필터: 정지 떨림 감소, 계단 응답 지연 짧음 ----
  { OneEuro f; float dt = 0.0005f, mx = -1e9, mn = 1e9;
    for (int i = 0; i < 4000; i++) { float y = f.push(((i * 7919) % 13 - 6) * 0.01f, dt); if (i > 2000) { mx = fmaxf(mx, y); mn = fminf(mn, y); } }
    CHECK(mx - mn < 0.03f, "one-euro jitter < 0.03 deg (input ±0.06)");
    OneEuro g; g.push(0, dt); int k = 0; float y = 0; for (; k < 400 && y < 9.0f; k++) y = g.push(10.0f, dt);
    char m[64]; sprintf(m, "one-euro step 90%% in %d samples (%.1f ms)", k, k * 0.5f); CHECK(k * 0.5f < 15.0f, m); }

  // ---- 설정 저장 형식 ----
  { Config c; cfgSeal(c); CHECK(cfgValid(c), "seal valid"); c.kP = 2; CHECK(!cfgValid(c), "crc detects change"); }
  { Config c; const Key *k = findKey("gear_on");
    CHECK(k && keySet(c, *k, "10.5") == 0 && fabsf(c.p.gearOn - 10.5f) < 1e-6, "set gear_on");
    CHECK(keySet(c, *k, "abc") == 1, "not a number");
    CHECK(keySet(c, *k, "20") == 2, "range");
    CHECK(keySet(c, *findKey("gear_off"), "11") == 3, "gear_off >= gear_on rejected");
    CHECK(keySet(c, *findKey("layout"), "1") == 0 && c.p.layout == 1, "layout u8");
    CHECK(keySet(c, *findKey("seq_pulse"), "60") == 0 && c.seqPulseMs == 60, "u16"); }

  // ---- 프로토콜 ----
  { Config c; Live L; int hz = 0; std::string r;
    r = cmd("info", c, L, hz); CHECK(has(r, "\"ok\":true") && has(r, "\"maker\":\"Sunggwan\"") && has(r, "\"proto\":1"), "info");
    r = cmd("get", c, L, hz); CHECK(has(r, "\"ok\":true") && has(r, "\"gear_on\":9.5") && has(r, "\"layout\":0"), "get");
    r = cmd("keys", c, L, hz); CHECK(has(r, "\"k\":\"seq_pulse\"") && r.back() == '}', "keys");
    r = cmd("set gear_on 10", c, L, hz); CHECK(has(r, "\"gear_on\":10") && c.p.gearOn == 10.0f, "set via proto");
    r = cmd("set nope 1", c, L, hz); CHECK(has(r, "\"ok\":false"), "unknown key");
    r = cmd("set gear_on", c, L, hz); CHECK(has(r, "\"ok\":false"), "missing value");
    r = cmd("save", c, L, hz); CHECK(has(r, "\"ok\":true") && flashOk && cfgValid(flash), "save");
    c.p.gearOn = 11; r = cmd("load\r\n", c, L, hz); CHECK(has(r, "\"ok\":true") && c.p.gearOn == 10.0f, "load");
    L.rawP = 8000; L.rawR = 9000; r = cmd("cal zero", c, L, hz); CHECK(c.zeroP == 8000 && c.zeroR == 9000, "cal zero");
    L.theta = 0.5f; L.phi = 14.1f; r = cmd("cal gate 0", c, L, hz); CHECK(fabsf(c.p.gatePhi[0] - 14.1f) < 1e-4, "cal gate");
    L.theta = 9; r = cmd("cal gate 1", c, L, hz); CHECK(has(r, "\"ok\":false"), "cal gate needs neutral");
    L.theta = 12.5f; float k0 = c.kP; r = cmd("cal throw", c, L, hz); CHECK(fabsf(c.kP - k0 * 14.0f / 12.5f) < 1e-4, "cal throw");
    Config d = c; L.theta = 7.0f; L.phi = 7.0f; r = cmd("cal gear1", d, L, hz);
    CHECK(has(r, "\"ok\":true") && fabsf(d.kP - c.kP * 2.0f) < 1e-4 && fabsf(d.kR - c.kR * c.p.gatePhi[0] / 7.0f) < 1e-4, "cal gear1");
    r = cmd("stream 200", c, L, hz); CHECK(hz == 200, "stream on");
    r = cmd("stream 900", c, L, hz); CHECK(has(r, "\"ok\":false") && hz == 200, "stream range");
    r = cmd("status", c, L, hz); CHECK(has(r, "{\"s\":{") && has(r, "\"mode\":\"H\""), "status");
    c.p.layout = 1; c.p.gearOn = 12; r = cmd("defaults", c, L, hz);
    CHECK(c.p.layout == 0 && c.p.gearOn == 9.5f && c.zeroP == 8000 && fabsf(c.p.gatePhi[0] - 14.1f) < 1e-4, "defaults keeps calibration");
    r = cmd("factory", c, L, hz); CHECK(c.zeroP == 0 && c.p.gatePhi[0] == 15.26f, "factory");
    r = cmd("frobnicate", c, L, hz); CHECK(has(r, "\"ok\":false"), "unknown cmd");
    CHECK(cmd("   ", c, L, hz).empty(), "blank line"); }

  printf(fails ? "%d FAILED\n" : "ALL PASS\n", fails);
  return fails;
}
