/*
  LSK H/SEQ Shifter firmware 0.6
  제작: Sunggwan  (펌웨어·소프트웨어·하드웨어 자체 설계)
  보드: Waveshare RP2040-Zero (Arduino-Pico 코어, PlatformIO)
  USB: HID 게임 컨트롤러 + CDC 시리얼(설정) 복합 장치. 이름 "Sunggwan / LSK HSEQ Shifter"

  성능
    - USB HID 폴링 1 ms (코어 기본값 10 ms → 1000 Hz 보고)
    - 코어 1: 센서 2 kHz 측정 + One-Euro 필터 / 코어 0: 판정 · USB · 설정 (1 kHz)
    - 축 16비트, 바뀐 것만 보고
    - 센서 읽기 실패가 이어지면 I2C 버스 복구, 그동안 마지막 상태 유지

  배선 (베이스 바닥 포켓, JST-SH 4핀)
    J1 앞뒤 센서 (−Y 타워 캡)   : 3V3, GND, SDA→GP4, SCL→GP5   (I2C0 = Wire)
    J2 좌우 센서 (크래들 +X 캡) : 3V3, GND, SDA→GP2, SCL→GP3   (I2C1 = Wire1, 중공 핀 통과)
    J3 모드 스위치 (SEQ 잠금)   : GP6 ↔ GND (눌림 = 시퀀셜)
    J4 캘리브레이션 버튼        : GP7 ↔ GND

  버튼만으로 캘리브레이션: 중립에서 버튼 → 1단에 넣고 버튼 → 저장.
  자세한 설정은 USB 시리얼 프로토콜 (src/proto.h) — tools/lsk_shifter.py (CLI) 또는 tools/web/index.html (웹)
*/
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Joystick.h>
#include <pico/critical_section.h>
#include "shifter_logic.h"
#include "config.h"
#include "proto.h"

#ifndef FW_MAKER
#define FW_MAKER "Sunggwan"
#endif
#ifndef FW_PRODUCT
#define FW_PRODUCT "LSK HSEQ Shifter"
#endif
#ifndef FW_VERSION
#define FW_VERSION "0.6.0"
#endif

using namespace lsk;

// Arduino-Pico 코어의 HID 폴링 간격(약한 심볼, 기본 10 ms) → 1 ms
int usb_hid_poll_interval = 1;

#define SENSOR_MT6701 1          // 0 → AS5600
#if SENSOR_MT6701
const uint8_t SENS_ADDR = 0x06, SENS_REG = 0x03;  // 14비트 각도 = (r[0]<<8 | r[1]) >> 2
const int32_t CPR = 16384;
#else
const uint8_t SENS_ADDR = 0x36, SENS_REG = 0x0C;  // 12비트 RAW ANGLE
const int32_t CPR = 4096;
#endif

const uint8_t PIN_SDA0 = 4, PIN_SCL0 = 5, PIN_SDA1 = 2, PIN_SCL1 = 3, PIN_MODE = 6, PIN_CAL = 7;
const uint32_t SENS_US = 500;    // 코어 1 센서 주기 (2 kHz)
const uint32_t LOOP_US = 1000;   // 코어 0 판정·보고 주기 (1 kHz)
const uint16_t FAIL_RECOVER = 20;  // 연속 실패 → 버스 복구 (10 ms)

Config cfg;
State st;
Info info = {FW_PRODUCT, FW_VERSION, FW_MAKER, SENSOR_MT6701 ? "MT6701" : "AS5600", CPR};

// ---------------- 코어 사이 공유 (코어 1이 씀, 코어 0이 읽음) ----------------
struct Shared { uint16_t rawP, rawR; float th, ph; bool okP, okR; uint16_t errP, errR; uint32_t sensHz; };
critical_section_t cs;
Shared sh = {0, 0, 0, 0, false, false, 0, 0, 0};
volatile bool core1Ready = false, filtReset = false;

// ---------------- 센서 (코어 1) ----------------
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

void i2cBegin(TwoWire &w, uint8_t sda, uint8_t scl) {
  w.setSDA(sda); w.setSCL(scl);
  w.setClock((uint32_t)cfg.i2cKHz * 1000u);
  w.setTimeout(2, true);           // 2 ms 안에 안 끝나면 포기 (코어 1이 멈추지 않게)
  w.begin();
}

// 버스가 SDA low로 걸렸을 때: SCL 9번 + STOP
void i2cRecover(TwoWire &w, uint8_t sda, uint8_t scl) {
  w.end();
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT);
  for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
    digitalWrite(scl, LOW); delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
  }
  pinMode(sda, OUTPUT); digitalWrite(sda, LOW); delayMicroseconds(5);
  digitalWrite(scl, HIGH); delayMicroseconds(5); digitalWrite(sda, HIGH); delayMicroseconds(5);
  i2cBegin(w, sda, scl);
}

void setup1() {
  while (!core1Ready) delay(1);    // 코어 0이 설정을 읽은 뒤 시작
  i2cBegin(Wire, PIN_SDA0, PIN_SCL0);
  i2cBegin(Wire1, PIN_SDA1, PIN_SCL1);
}

void loop1() {
  static OneEuro fP, fR;
  static uint32_t tNext = 0, tPrev = 0, tHz = 0, nHz = 0;
  static uint16_t failP = 0, failR = 0, errP = 0, errR = 0, rawP = 0, rawR = 0;
  uint32_t now = micros();
  if ((int32_t)(now - tNext) < 0) return;
  tNext = now + SENS_US;
  float dt = tPrev ? (now - tPrev) * 1e-6f : SENS_US * 1e-6f;
  tPrev = now;

  uint16_t r;
  bool okP = readSensor(Wire, r);  if (okP) { rawP = r; failP = 0; } else { errP++; if (++failP >= FAIL_RECOVER) { i2cRecover(Wire, PIN_SDA0, PIN_SCL0); failP = 0; } }
  bool okR = readSensor(Wire1, r); if (okR) { rawR = r; failR = 0; } else { errR++; if (++failR >= FAIL_RECOVER) { i2cRecover(Wire1, PIN_SDA1, PIN_SCL1); failR = 0; } }

  if (filtReset) { fP.init = fR.init = false; filtReset = false; }
  fP.minCut = fR.minCut = cfg.fMinCut; fP.beta = fR.beta = cfg.fBeta;
  float th = fP.push(rawToDeg(rawP, cfg.zeroP, 1, CPR) * cfg.kP, dt);
  float ph = fR.push(rawToDeg(rawR, cfg.zeroR, 1, CPR) * cfg.kR, dt);

  nHz++;
  uint32_t hz = 0;
  if (now - tHz >= 1000000u) { hz = nHz; nHz = 0; tHz = now; }

  critical_section_enter_blocking(&cs);
  sh.rawP = rawP; sh.rawR = rawR; sh.th = th; sh.ph = ph;
  sh.okP = failP < 5; sh.okR = failR < 5; sh.errP = errP; sh.errR = errR;
  if (hz) sh.sensHz = hz;
  critical_section_exit(&cs);
}

// ---------------- 설정 저장 ----------------
bool saveCfg(const Config &c) { Config t = c; cfgSeal(t); EEPROM.put(0, t); return EEPROM.commit(); }
bool loadCfg(Config &c) { EEPROM.get(0, c); return cfgValid(c); }
void doReboot() { Serial.flush(); delay(20); rp2040.reboot(); }
void doBootsel() { Serial.flush(); delay(20); rp2040.rebootToBootloader(); }
Actions act;

// ---------------- 코어 0 ----------------
void setup() {
  pinMode(PIN_MODE, INPUT_PULLUP);
  pinMode(PIN_CAL, INPUT_PULLUP);
  critical_section_init(&cs);
  EEPROM.begin(512);
  if (!loadCfg(cfg)) { cfg = Config(); cfg.zeroP = cfg.zeroR = (uint16_t)(CPR / 2); saveCfg(cfg); }
  act.save = saveCfg; act.load = loadCfg; act.reboot = doReboot; act.bootsel = doBootsel;
  Serial.begin(115200);
  Joystick.begin();
  Joystick.use16bit();
  Joystick.useManualSend(true);
  core1Ready = true;
}

Live live;
int streamHz = 0;
char rx[96]; uint8_t rxn = 0;
char tx[3072];

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxn) {
        rx[rxn] = 0; rxn = 0;
        Config before = cfg;
        int n = handleLine(rx, cfg, live, info, act, streamHz, tx, sizeof(tx));
        if (n > 0) { Serial.write(tx, min(n, (int)sizeof(tx) - 1)); Serial.write('\n'); }
        if (before.zeroP != cfg.zeroP || before.zeroR != cfg.zeroR || before.kP != cfg.kP || before.kR != cfg.kR) filtReset = true;
      }
    } else if (rxn < sizeof(rx) - 1) {
      rx[rxn++] = c;
    }
  }
}

// 버튼 2단계 캘리브레이션: 중립 → 1단 → 저장
uint8_t calStep = 0;
void buttonCal() {
  if (calStep == 0) {
    cfg.zeroP = live.rawP; cfg.zeroR = live.rawR; cfg.kP = cfg.kP < 0 ? -1.0f : 1.0f; cfg.kR = cfg.kR < 0 ? -1.0f : 1.0f;
    filtReset = true; calStep = 1;
    Serial.println("{\"cal\":\"zero 저장 — 1단에 끝까지 넣고 버튼\"}");
    return;
  }
  calStep = 0;
  const char *e = calGear1(cfg, live);
  if (e) { Serial.printf("{\"cal\":\"%s\"}\n", e); return; }
  filtReset = true;
  saveCfg(cfg);
  Serial.printf("{\"cal\":\"완료\",\"k_p\":%.4f,\"k_r\":%.4f}\n", cfg.kP, cfg.kR);
}

struct Report { uint16_t buttons; int16_t x, y; };
Report last = {0xFFFF, 0, 0};

void loop() {
  static uint32_t tNext = 0, tBtn = 0, tStream = 0, tHz = 0, nHz = 0, tPulse = 0, tModeChg = 0;
  static bool btnPrev = true, seqMode = false, modeRaw = false;
  static int8_t pulseDir = 0;
  uint32_t now = micros();
  handleSerial();
  if ((int32_t)(now - tNext) < 0) return;
  tNext = now + LOOP_US;
  nHz++;
  if (now - tHz >= 1000000u) { live.loopHz = nHz; nHz = 0; tHz = now; }

  critical_section_enter_blocking(&cs);
  Shared s = sh;
  critical_section_exit(&cs);
  live.ms = millis(); live.rawP = s.rawP; live.rawR = s.rawR; live.theta = s.th; live.phi = s.ph;
  live.okP = s.okP; live.okR = s.okR; live.errP = s.errP; live.errR = s.errR; live.sensHz = s.sensHz;

  // 캘리브레이션 버튼 (300 ms 디바운스)
  bool b = digitalRead(PIN_CAL);
  if (!b && btnPrev && millis() - tBtn > 300) { tBtn = millis(); buttonCal(); }
  btnPrev = b;

  // 모드 스위치 (5 ms 디바운스)
  bool m = (digitalRead(PIN_MODE) == LOW);
  if (m != modeRaw) { modeRaw = m; tModeChg = millis(); }
  if (seqMode != modeRaw && millis() - tModeChg > 5) { seqMode = modeRaw; st = State(); pulseDir = 0; }
  live.seq = seqMode;

  uint16_t buttons = 0;
  bool sensorsOk = s.okP && s.okR;
  if (!seqMode) {
    if (sensorsOk) updateH(st, s.th, s.ph, cfg.p);      // 센서 오류 중에는 마지막 기어 유지
    if (st.gear) buttons |= (uint16_t)1 << (st.gear - 1); // 버튼 1..7 = 기어, 8 = R
  } else {
    int8_t e = sensorsOk ? updateSeq(st, s.th, cfg.p) : 0;
    int8_t l = st.seqLatch;
    if (cfg.seqSwap) { e = -e; l = -l; }
    if (cfg.seqPulseMs == 0) {
      if (l > 0) buttons |= 1u << 8;                       // 버튼 9 = 밀기
      if (l < 0) buttons |= 1u << 9;                       // 버튼 10 = 당기기
    } else {
      if (e) { pulseDir = e; tPulse = millis(); }
      if (pulseDir && millis() - tPulse < cfg.seqPulseMs) buttons |= pulseDir > 0 ? (1u << 8) : (1u << 9);
      else pulseDir = 0;
    }
  }
  live.gate = st.gate; live.gear = st.gear; live.seqLatch = st.seqLatch;

  Report r = {buttons, axis16(s.th, cfg.axisRangeP), axis16(s.ph, cfg.axisRangeR)};
  if (r.buttons != last.buttons || r.x != last.x || r.y != last.y) {
    for (uint8_t i = 0; i < 10; i++) Joystick.button(i + 1, (r.buttons >> i) & 1);
    Joystick.X(r.x);
    Joystick.Y(r.y);
    Joystick.send_now();
    last = r;
  }

  if (streamHz > 0 && millis() - tStream >= 1000u / (uint32_t)streamHz) {
    tStream = millis();
    int n = liveJson(live, tx, sizeof(tx));
    Serial.write(tx, n); Serial.write('\n');
  }
}
