/*
  LSK H/SEQ 쉬프터 펌웨어 v0.3
  보드   : Arduino Pro Micro (ATmega32U4, 5V/16MHz) — Arduino IDE에서 "Arduino Leonardo" 선택
  라이브러리: "Joystick" by Matthew Heironimus (ArduinoJoystickLibrary 2.x) — GitHub에서 ZIP으로 설치

  배선
    AS5600 #1 (앞뒤, −Y 타워)  : VCC→VCC(5V)  GND→GND  SDA→D2  SCL→D3   DIR→GND
    AS5600 #2 (좌우, 크래들 +X) : VCC→VCC(5V)  GND→GND  SDA→D4  SCL→D5   DIR→GND   (소프트웨어 I2C)
    모드 마이크로스위치 (SEQ)    : D7 ↔ GND  (눌림 = 시퀀셜)
    캘리브레이션 버튼            : D8 ↔ GND
    * 두 센서 주소가 같아서(0x36) 버스를 둘로 나눔. AS5600 보드에 전압 점퍼가 있으면 5V로.

  캘리브레이션 (처음 / 재조립 후)
    1) 레버를 중립(3-4 게이트)에 두고 버튼 짧게 → 영점 기록
    2) 1단(표준 배열: 왼쪽 앞)에 넣고 다시 버튼 → 방향·배율 학습
    EEPROM에 저장됨. 시리얼 모니터 115200: p=실시간 값  l=배열 전환  x=시퀀셜 방향 반전  r=초기화  m=자석 확인
*/
#include <Wire.h>
#include <EEPROM.h>
#include <Joystick.h>
#include "shifter_logic.h"

using namespace lsk;

// ---------------- 핀 ----------------
const uint8_t PIN_SDA2 = 4, PIN_SCL2 = 5, PIN_MODE = 7, PIN_CAL = 8;
const uint8_t AS5600_ADDR = 0x36, REG_STATUS = 0x0B, REG_RAW = 0x0C;

// ---------------- 설정 (EEPROM) ----------------
struct Config {
  uint16_t magic;
  uint16_t zeroP, zeroR;
  float kP, kR;          // 부호 × 배율
  uint8_t layout;        // 0 표준, 1 도그레그
  uint8_t seqSwap;       // 1 → 밀기/당기기 버튼 교환
};
const uint16_t MAGIC = 0x5A17;
Config cfg;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
                   10, 0,                 // 버튼 1..7 = 기어, 8 = R, 9 = SEQ 밀기, 10 = SEQ 당기기
                   true, true, false,     // X = 앞뒤, Y = 좌우
                   false, false, false, false, false, false, false, false);

State st;
bool live = false;

// ---------------- 소프트웨어 I2C (좌우 센서) ----------------
static inline void sdaHigh() { pinMode(PIN_SDA2, INPUT_PULLUP); }
static inline void sdaLow()  { pinMode(PIN_SDA2, OUTPUT); digitalWrite(PIN_SDA2, LOW); }
static inline void sclHigh() { pinMode(PIN_SCL2, INPUT_PULLUP); delayMicroseconds(4); }
static inline void sclLow()  { pinMode(PIN_SCL2, OUTPUT); digitalWrite(PIN_SCL2, LOW); delayMicroseconds(4); }

static void i2cStart() { sdaHigh(); sclHigh(); sdaLow(); delayMicroseconds(4); sclLow(); }
static void i2cStop()  { sdaLow(); sclHigh(); sdaHigh(); delayMicroseconds(4); }
static bool i2cWrite(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) { (b & 0x80) ? sdaHigh() : sdaLow(); sclHigh(); sclLow(); b <<= 1; }
  sdaHigh(); sclHigh(); bool ack = !digitalRead(PIN_SDA2); sclLow();
  return ack;
}
static uint8_t i2cRead(bool ack) {
  uint8_t b = 0; sdaHigh();
  for (uint8_t i = 0; i < 8; i++) { b <<= 1; sclHigh(); if (digitalRead(PIN_SDA2)) b |= 1; sclLow(); }
  ack ? sdaLow() : sdaHigh(); sclHigh(); sclLow(); sdaHigh();
  return b;
}
static bool readSoft(uint8_t reg, uint8_t *buf, uint8_t n) {
  i2cStart();
  if (!i2cWrite(AS5600_ADDR << 1)) { i2cStop(); return false; }
  i2cWrite(reg);
  i2cStart();
  if (!i2cWrite((AS5600_ADDR << 1) | 1)) { i2cStop(); return false; }
  for (uint8_t i = 0; i < n; i++) buf[i] = i2cRead(i < n - 1);
  i2cStop();
  return true;
}
static bool readHw(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(AS5600_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
static bool readRaw(bool soft, uint16_t &raw) {
  uint8_t b[2];
  bool ok = soft ? readSoft(REG_RAW, b, 2) : readHw(REG_RAW, b, 2);
  if (ok) raw = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
  return ok;
}
static bool magnetOK(bool soft) {
  uint8_t s;
  bool ok = soft ? readSoft(REG_STATUS, &s, 1) : readHw(REG_STATUS, &s, 1);
  return ok && (s & 0x20);  // MD 비트(자석 감지)
}

// ---------------- 필터 (3점 중앙값 + 약한 IIR) ----------------
struct Filt {
  float h[3] = {0, 0, 0}; uint8_t i = 0; float y = 0; bool init = false;
  float push(float x) {
    h[i] = x; i = (i + 1) % 3;
    float a = h[0], b = h[1], c = h[2];
    float m = max(min(a, b), min(max(a, b), c));
    if (!init) { y = m; init = true; }
    y += 0.5f * (m - y);
    return y;
  }
} fP, fR;

// ---------------- 캘리브레이션 ----------------
uint8_t calStep = 0;
uint16_t lastRawP = 0, lastRawR = 0;

void saveCfg() { EEPROM.put(0, cfg); }
void defaultCfg() {
  cfg.magic = MAGIC; cfg.zeroP = 2048; cfg.zeroR = 2048; cfg.kP = 1.0f; cfg.kR = 1.0f;
  cfg.layout = LAYOUT_STANDARD; cfg.seqSwap = 0;
}

void calibrateStep() {
  if (calStep == 0) {
    cfg.zeroP = lastRawP; cfg.zeroR = lastRawR; cfg.kP = 1; cfg.kR = 1;
    calStep = 1;
    Serial.println(F("[CAL] 중립 저장. 이제 1단(왼쪽 앞)에 넣고 다시 누르세요."));
  } else {
    float th = rawToDeg(lastRawP, cfg.zeroP, 1);
    float ph = rawToDeg(lastRawR, cfg.zeroR, 1);
    if (fabsf(th) < 6 || fabsf(ph) < 4) {
      Serial.println(F("[CAL] 움직임이 부족해요 — 1단계부터 다시.")); calStep = 0; return;
    }
    // 1단(표준) = 앞(+θ, 스톱 14°) / 왼쪽 게이트(+15.26°)
    float tgtTh = (cfg.layout == LAYOUT_STANDARD) ? 14.0f : -14.0f;   // 도그레그: 1단 = gate0 뒤
    cfg.kP = tgtTh / th;
    cfg.kR = GATE_PHI[0] / ph;
    calStep = 0;
    saveCfg();
    Serial.print(F("[CAL] 완료 kP=")); Serial.print(cfg.kP, 3); Serial.print(F(" kR=")); Serial.println(cfg.kR, 3);
  }
}

// ---------------- setup / loop ----------------
void setup() {
  pinMode(PIN_MODE, INPUT_PULLUP);
  pinMode(PIN_CAL, INPUT_PULLUP);
  sdaHigh(); sclHigh();
  Wire.begin(); Wire.setClock(400000);
  Serial.begin(115200);
  EEPROM.get(0, cfg);
  if (cfg.magic != MAGIC) { defaultCfg(); saveCfg(); }
  Joystick.setXAxisRange(-1500, 1500);   // ±15.00°
  Joystick.setYAxisRange(-3200, 3200);   // ±32.00°
  Joystick.begin(false);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'p') live = !live;
    else if (c == 'l') { cfg.layout ^= 1; saveCfg(); Serial.println(cfg.layout ? F("배열: 도그레그") : F("배열: 표준")); }
    else if (c == 'x') { cfg.seqSwap ^= 1; saveCfg(); Serial.println(F("시퀀셜 방향 반전")); }
    else if (c == 'r') { defaultCfg(); saveCfg(); Serial.println(F("초기화됨 — 캘리브레이션 필요")); }
    else if (c == 'c') calibrateStep();
    else if (c == 'm') { Serial.print(F("자석 감지 앞뒤/좌우: ")); Serial.print(magnetOK(false)); Serial.println(magnetOK(true)); }
  }
}

uint32_t lastBtn = 0; bool btnPrev = true;
uint32_t lastPrint = 0;

void loop() {
  uint16_t rp, rr;
  bool okP = readRaw(false, rp), okR = readRaw(true, rr);
  if (okP) lastRawP = rp;
  if (okR) lastRawR = rr;

  float theta = fP.push(rawToDeg(lastRawP, cfg.zeroP, 1) * cfg.kP);
  float phi   = fR.push(rawToDeg(lastRawR, cfg.zeroR, 1) * cfg.kR);

  // 캘리브레이션 버튼 (디바운스)
  bool b = digitalRead(PIN_CAL);
  if (!b && btnPrev && millis() - lastBtn > 300) { lastBtn = millis(); calibrateStep(); }
  btnPrev = b;
  handleSerial();

  bool seqMode = (digitalRead(PIN_MODE) == LOW);
  for (uint8_t i = 0; i < 10; i++) Joystick.setButton(i, 0);
  if (!seqMode) {
    st.seqLatch = 0;
    uint8_t g = updateH(st, theta, phi, (Layout)cfg.layout);
    if (g) Joystick.setButton(g - 1, 1);       // 1..7 → 버튼 1..7, R(8) → 버튼 8
  } else {
    st.gear = 0;
    updateSeq(st, theta);
    int8_t l = st.seqLatch;
    if (cfg.seqSwap) l = -l;
    if (l > 0) Joystick.setButton(8, 1);       // 밀기
    if (l < 0) Joystick.setButton(9, 1);       // 당기기
  }
  Joystick.setXAxis((int16_t)(theta * 100));
  Joystick.setYAxis((int16_t)(phi * 100));
  Joystick.sendState();

  if (live && millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(F("θ=")); Serial.print(theta, 2); Serial.print(F(" φ=")); Serial.print(phi, 2);
    Serial.print(F(" 게이트=")); Serial.print(st.gate); Serial.print(F(" 기어=")); Serial.print(st.gear);
    Serial.print(seqMode ? F(" SEQ") : F(" H")); Serial.print(F(" 센서 ")); Serial.print(okP); Serial.println(okR);
  }
  delay(1);
}
