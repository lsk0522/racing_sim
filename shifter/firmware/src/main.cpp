/*
  LSK H/SEQ Shifter firmware
  제작: Sunggwan  (펌웨어·소프트웨어·하드웨어 자체 설계)
  보드: Waveshare RP2040-Zero (Arduino-Pico 코어, PlatformIO)
  USB 이름: 제조사 "Sunggwan" / 제품 "LSK HSEQ Shifter" (platformio.ini에서 설정)

  배선 (베이스 바닥 포켓, JST-SH 4핀)
    J1 앞뒤 센서 (−Y 타워 캡)   : 3V3, GND, SDA→GP4, SCL→GP5   (I2C0 = Wire)
    J2 좌우 센서 (크래들 +X 캡) : 3V3, GND, SDA→GP2, SCL→GP3   (I2C1 = Wire1, 중공 핀 통과)
    J3 모드 스위치 (SEQ 잠금)   : GP6 ↔ GND (눌림 = 시퀀셜)
    J4 캘리브레이션 버튼        : GP7 ↔ GND

  캘리브레이션: 중립에서 버튼 → 1단(왼쪽 앞)에 넣고 버튼. 플래시에 저장.
  시리얼 명령(115200): i=장치 정보  p=실시간 값  l=표준/도그레그  x=시퀀셜 방향 반전  r=초기화  c=캘리브레이션 단계
*/
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Joystick.h>
#include "shifter_logic.h"

#ifndef FW_MAKER
#define FW_MAKER "Sunggwan"
#endif
#ifndef FW_PRODUCT
#define FW_PRODUCT "LSK HSEQ Shifter"
#endif
#ifndef FW_VERSION
#define FW_VERSION "0.5.0"
#endif

using namespace lsk;

#define SENSOR_MT6701 1          // 0 → AS5600

#if SENSOR_MT6701
const uint8_t SENS_ADDR = 0x06, SENS_REG = 0x03;  // 14비트 각도 = (r[0]<<8 | r[1]) >> 2
const int32_t CPR = 16384;
#else
const uint8_t SENS_ADDR = 0x36, SENS_REG = 0x0C;  // 12비트 RAW ANGLE
const int32_t CPR = 4096;
#endif

const uint8_t PIN_SDA0 = 4, PIN_SCL0 = 5, PIN_SDA1 = 2, PIN_SCL1 = 3, PIN_MODE = 6, PIN_CAL = 7;
const uint32_t LOOP_US = 1000;   // 1 kHz 측정 루프

struct Config {
  uint16_t magic;
  uint16_t zeroP, zeroR;
  float kP, kR;
  uint8_t layout, seqSwap;
};
const uint16_t MAGIC = 0x5A18;
Config cfg;
State st;
bool live = false;

bool readSensor(TwoWire &w, uint16_t &raw) {
  w.beginTransmission(SENS_ADDR);
  w.write(SENS_REG);
  if (w.endTransmission(false) != 0) return false;
  if (w.requestFrom(SENS_ADDR, (uint8_t)2) != 2) return false;
  uint8_t a = w.read(), b = w.read();
#if SENSOR_MT6701
  raw = (uint16_t)(((uint16_t)a << 8 | b) >> 2);
#else
  raw = (uint16_t)(((uint16_t)(a & 0x0F) << 8) | b);
#endif
  return true;
}

// 3점 중앙값 + 약한 IIR
struct Filt {
  float h[3] = {0, 0, 0}; uint8_t i = 0; float y = 0; bool init = false;
  float push(float x) {
    h[i] = x; i = (i + 1) % 3;
    float a = h[0], b = h[1], c = h[2];
    float m = max(min(a, b), min(max(a, b), c));
    if (!init) { y = m; init = true; }
    y += 0.6f * (m - y);
    return y;
  }
} fP, fR;

uint16_t rawP = 0, rawR = 0;
uint8_t calStep = 0;

void saveCfg() { EEPROM.put(0, cfg); EEPROM.commit(); }
void defaultCfg() { cfg = {MAGIC, (uint16_t)(CPR / 2), (uint16_t)(CPR / 2), 1.0f, 1.0f, LAYOUT_STANDARD, 0}; }

void calibrateStep() {
  if (calStep == 0) {
    cfg.zeroP = rawP; cfg.zeroR = rawR; cfg.kP = 1; cfg.kR = 1; calStep = 1;
    Serial.println("[CAL] 중립 저장 → 1단(왼쪽 앞)에 넣고 다시 누르세요");
    return;
  }
  float th = rawToDeg(rawP, cfg.zeroP, 1, CPR), ph = rawToDeg(rawR, cfg.zeroR, 1, CPR);
  calStep = 0;
  if (fabsf(th) < 7 || fabsf(ph) < 6) { Serial.println("[CAL] 움직임 부족 — 처음부터"); return; }
  float tgt = (cfg.layout == LAYOUT_STANDARD) ? 14.0f : -14.0f;   // 도그레그: 1단 = gate0 뒤
  cfg.kP = tgt / th;
  cfg.kR = GATE_PHI[0] / ph;
  saveCfg();
  Serial.printf("[CAL] 완료 kP=%.3f kR=%.3f\n", cfg.kP, cfg.kR);
}

void setup() {
  pinMode(PIN_MODE, INPUT_PULLUP);
  pinMode(PIN_CAL, INPUT_PULLUP);
  Wire.setSDA(PIN_SDA0); Wire.setSCL(PIN_SCL0); Wire.setClock(400000); Wire.begin();
  Wire1.setSDA(PIN_SDA1); Wire1.setSCL(PIN_SCL1); Wire1.setClock(400000); Wire1.begin();
  Serial.begin(115200);
  EEPROM.begin(256);
  EEPROM.get(0, cfg);
  if (cfg.magic != MAGIC) { defaultCfg(); saveCfg(); }
  Joystick.begin();
  Joystick.useManualSend(true);
}

void printInfo() {
  Serial.printf("%s  fw %s  — made by %s\n", FW_PRODUCT, FW_VERSION, FW_MAKER);
  Serial.printf("sensor=%s  layout=%s  seqSwap=%d  kP=%.3f kR=%.3f\n", SENSOR_MT6701 ? "MT6701" : "AS5600",
                cfg.layout ? "dogleg" : "standard", cfg.seqSwap, cfg.kP, cfg.kR);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'i') printInfo();
    else if (c == 'p') live = !live;
    else if (c == 'l') { cfg.layout ^= 1; saveCfg(); Serial.println(cfg.layout ? "배열: 도그레그" : "배열: 표준"); }
    else if (c == 'x') { cfg.seqSwap ^= 1; saveCfg(); Serial.println("시퀀셜 방향 반전"); }
    else if (c == 'r') { defaultCfg(); saveCfg(); Serial.println("초기화 — 캘리브레이션 필요"); }
    else if (c == 'c') calibrateStep();
  }
}

static int axisVal(float deg, float range) {          // ±range° → 0..1023
  int v = (int)(512.0f + deg / range * 511.0f);
  return v < 0 ? 0 : (v > 1023 ? 1023 : v);
}

uint32_t tNext = 0, tBtn = 0, tPrint = 0;
bool btnPrev = true;

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - tNext) < 0) return;
  tNext = now + LOOP_US;

  uint16_t r;
  bool okP = readSensor(Wire, r);  if (okP) rawP = r;
  bool okR = readSensor(Wire1, r); if (okR) rawR = r;
  float theta = fP.push(rawToDeg(rawP, cfg.zeroP, 1, CPR) * cfg.kP);
  float phi   = fR.push(rawToDeg(rawR, cfg.zeroR, 1, CPR) * cfg.kR);

  bool b = digitalRead(PIN_CAL);
  if (!b && btnPrev && millis() - tBtn > 300) { tBtn = millis(); calibrateStep(); }
  btnPrev = b;
  handleSerial();

  bool seq = (digitalRead(PIN_MODE) == LOW);
  for (uint8_t i = 1; i <= 10; i++) Joystick.button(i, false);
  if (!seq) {
    st.seqLatch = 0;
    uint8_t g = updateH(st, theta, phi, (Layout)cfg.layout);
    if (g) Joystick.button(g, true);                   // 버튼 1..7 = 기어, 8 = R
  } else {
    st.gear = 0;
    updateSeq(st, theta);
    int8_t l = cfg.seqSwap ? -st.seqLatch : st.seqLatch;
    if (l > 0) Joystick.button(9, true);               // 밀기
    if (l < 0) Joystick.button(10, true);              // 당기기
  }
  Joystick.X(axisVal(theta, 16.0f));
  Joystick.Y(axisVal(phi, 32.0f));
  Joystick.send_now();

  if (live && millis() - tPrint > 100) {
    tPrint = millis();
    Serial.printf("θ=%6.2f φ=%6.2f 게이트=%d 기어=%d %s 센서=%d%d\n", theta, phi, st.gate, st.gear, seq ? "SEQ" : "H", okP, okR);
  }
}
