/*
  LSK H/SEQ Shifter firmware v0.1
  Board : Arduino Pro Micro (ATmega32U4, 5V/16MHz) — choose "Arduino Leonardo" in the Arduino IDE
  Library: "Joystick" by Matthew Heironimus (ArduinoJoystickLibrary 2.x) — install from GitHub as a ZIP

  Wiring
    AS5600 #1 (pitch, −Y tower)  : VCC→VCC(5V)  GND→GND  SDA→D2  SCL→D3   DIR→GND
    AS5600 #2 (roll,  cradle +X)  : VCC→VCC(5V)  GND→GND  SDA→D4  SCL→D5   DIR→GND   (software I2C)
    Mode microswitch (SEQ)        : D7 ↔ GND  (closed = SEQ)
    Calibration button            : D8 ↔ GND
    * Same address (0x36), so they run on two separate buses. Set the AS5600 boards' voltage jumper to 5V if they have one.

  Calibration (first time / after reassembly)
    1) Hold the lever in neutral (3-4 gate) → short-press the button (records zero)
    2) Put it in 1st (standard layout: front-left gate) → short-press again (learns direction & scale)
    The values are saved to EEPROM. Serial monitor 115200: p=live data l=layout toggle x=swap SEQ direction r=reset
*/
#include <Wire.h>
#include <EEPROM.h>
#include <Joystick.h>
#include "shifter_logic.h"

using namespace lsk;

// ---------------- Pins ----------------
const uint8_t PIN_SDA2 = 4, PIN_SCL2 = 5, PIN_MODE = 7, PIN_CAL = 8;
const uint8_t AS5600_ADDR = 0x36, REG_STATUS = 0x0B, REG_RAW = 0x0C;

// ---------------- Settings (EEPROM) ----------------
struct Config {
  uint16_t magic;
  uint16_t zeroP, zeroR;
  float kP, kR;          // sign × scale
  uint8_t layout;        // 0 standard, 1 dogleg
  uint8_t seqSwap;       // 1 → swap push/pull buttons
};
const uint16_t MAGIC = 0x5A17;
Config cfg;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
                   10, 0,                 // buttons 1..7 gears, 8=R, 9=SEQ push, 10=SEQ pull
                   true, true, false,     // X = pitch, Y = roll
                   false, false, false, false, false, false, false, false);

State st;
bool live = false;

// ---------------- Software I2C (roll sensor) ----------------
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
  return ok && (s & 0x20);  // MD bit
}

// ---------------- Filter (median of 3 + light IIR) ----------------
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

// ---------------- Calibration ----------------
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
    Serial.println(F("[CAL] Neutral saved. Now put it in 1st (front-left) and press again."));
  } else {
    float th = rawToDeg(lastRawP, cfg.zeroP, 1);
    float ph = rawToDeg(lastRawR, cfg.zeroR, 1);
    if (fabsf(th) < 6 || fabsf(ph) < 4) {
      Serial.println(F("[CAL] Not enough movement — try again from step 1.")); calStep = 0; return;
    }
    // 1st gear (standard) = front(+θ, THETA_STOP 12°) / left gate(+8.2°)
    float tgtTh = (cfg.layout == LAYOUT_STANDARD) ? 12.0f : -12.0f;   // dogleg: 1st = gate0 rear
    cfg.kP = tgtTh / th;
    cfg.kR = GATE_PHI[0] / ph;
    calStep = 0;
    saveCfg();
    Serial.print(F("[CAL] Done kP=")); Serial.print(cfg.kP, 3); Serial.print(F(" kR=")); Serial.println(cfg.kR, 3);
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
  Joystick.setYAxisRange(-2000, 2000);   // ±20.00°
  Joystick.begin(false);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'p') live = !live;
    else if (c == 'l') { cfg.layout ^= 1; saveCfg(); Serial.println(cfg.layout ? F("Layout: DOGLEG") : F("Layout: STANDARD")); }
    else if (c == 'x') { cfg.seqSwap ^= 1; saveCfg(); Serial.println(F("SEQ direction swapped")); }
    else if (c == 'r') { defaultCfg(); saveCfg(); Serial.println(F("Reset — calibration required")); }
    else if (c == 'c') calibrateStep();
    else if (c == 'm') { Serial.print(F("magnet P/R: ")); Serial.print(magnetOK(false)); Serial.println(magnetOK(true)); }
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

  // Calibration button (debounce)
  bool b = digitalRead(PIN_CAL);
  if (!b && btnPrev && millis() - lastBtn > 300) { lastBtn = millis(); calibrateStep(); }
  btnPrev = b;
  handleSerial();

  bool seqMode = (digitalRead(PIN_MODE) == LOW);
  for (uint8_t i = 0; i < 10; i++) Joystick.setButton(i, 0);
  if (!seqMode) {
    st.seqLatch = 0;
    uint8_t g = updateH(st, theta, phi, (Layout)cfg.layout);
    if (g) Joystick.setButton(g - 1, 1);       // 1..7 → button 1..7, R(8) → button 8
  } else {
    st.gear = 0;
    updateSeq(st, theta);
    int8_t l = st.seqLatch;
    if (cfg.seqSwap) l = -l;
    if (l > 0) Joystick.setButton(8, 1);       // push
    if (l < 0) Joystick.setButton(9, 1);       // pull
  }
  Joystick.setXAxis((int16_t)(theta * 100));
  Joystick.setYAxis((int16_t)(phi * 100));
  Joystick.sendState();

  if (live && millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(F("θ=")); Serial.print(theta, 2); Serial.print(F(" φ=")); Serial.print(phi, 2);
    Serial.print(F(" gate=")); Serial.print(st.gate); Serial.print(F(" gear=")); Serial.print(st.gear);
    Serial.print(seqMode ? F(" SEQ") : F(" H")); Serial.print(F(" sensors ")); Serial.print(okP); Serial.println(okR);
  }
  delay(1);
}
