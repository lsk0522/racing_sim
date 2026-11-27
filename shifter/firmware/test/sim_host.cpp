// LSK H/SEQ 쉬프터 — PC 시뮬레이터 (제작: Sunggwan)
// 펌웨어와 같은 proto.h / config.h / shifter_logic.h로 시리얼 프로토콜을 흉내 냄 → CLI·웹 도구 시험용.
// 레버는 3단 → 중립 → 2단을 천천히 반복. 표준 입출력 = 시리얼.
//   g++ -std=c++17 test/sim_host.cpp -o sim && ./sim
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>
#include <cmath>
#include "../src/shifter_logic.h"
#include "../src/config.h"
#include "../src/proto.h"
using namespace lsk;

static Config flash; static bool flashOk = false;
static bool fsave(const Config &c) { flash = c; flashOk = true; return true; }
static bool fload(Config &c) { if (!flashOk) return false; c = flash; return true; }
static double now_s() { timeval t; gettimeofday(&t, nullptr); return t.tv_sec + t.tv_usec * 1e-6; }

int main() {
  setvbuf(stdout, nullptr, _IONBF, 0);
  Config cfg; cfg.zeroP = cfg.zeroR = 8192;
  Info I = {"LSK HSEQ Shifter (sim)", "0.6.0-sim", "Sunggwan", "MT6701", 16384};
  Actions A; A.save = fsave; A.load = fload;
  Live L; State st; int hz = 0; char line[128]; int ln = 0; static char o[4096];
  double t0 = now_s(), tLast = 0;
  while (true) {
    fd_set rf; FD_ZERO(&rf); FD_SET(0, &rf); timeval tv = {0, 1000};
    if (select(1, &rf, nullptr, nullptr, &tv) > 0) {
      char c; ssize_t r = read(0, &c, 1);
      if (r <= 0) return 0;
      if (c == '\n' || c == '\r') {
        if (ln) { line[ln] = 0; ln = 0; int n = handleLine(line, cfg, L, I, A, hz, o, sizeof(o)); if (n > 0) { fwrite(o, 1, n, stdout); fputc('\n', stdout); } }
      } else if (ln < 127) line[ln++] = c;
    }
    double t = now_s() - t0;
    // 실제 각도 (가상 레버): 3-4 게이트에서 앞 14° ↔ 뒤 14° 천천히
    float thTrue = 14.0f * sinf((float)t * 1.2f), phTrue = 0.15f * sinf((float)t * 7.0f);
    L.rawP = (uint16_t)((int)lrintf(cfg.zeroP + thTrue / 360.0f * 16384) & 16383);
    L.rawR = (uint16_t)((int)lrintf(cfg.zeroR + phTrue / 360.0f * 16384) & 16383);
    L.theta = rawToDeg(L.rawP, cfg.zeroP, 1, 16384) * cfg.kP;
    L.phi = rawToDeg(L.rawR, cfg.zeroR, 1, 16384) * cfg.kR;
    updateH(st, L.theta, L.phi, cfg.p);
    L.gate = st.gate; L.gear = st.gear; L.ms = (uint32_t)(t * 1000); L.loopHz = 1000; L.sensHz = 2000;
    if (hz > 0 && t - tLast >= 1.0 / hz) { tLast = t; int n = liveJson(L, o, sizeof(o)); fwrite(o, 1, n, stdout); fputc('\n', stdout); }
  }
}
