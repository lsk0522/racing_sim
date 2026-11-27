# 펌웨어 — LSK HSEQ Shifter (제작: Sunggwan)

- 보드: Waveshare RP2040-Zero · 코어: Arduino-Pico (earlephilhower) · 빌드: PlatformIO
- USB: **HID 게임 컨트롤러 + CDC 시리얼(설정)** 복합 장치. 이름: 제조사 **Sunggwan**, 제품 **LSK HSEQ Shifter**
- 버전: **0.6.0** (2026-11-27) · 이전 0.5.0은 `bin/archive/`

## 성능 (0.6)
| 항목 | 0.5 | **0.6** |
|---|---|---|
| USB 보고 주기 | 10 ms (코어 기본값) | **1 ms (1000 Hz)** — `usb_hid_poll_interval = 1` |
| 센서 측정 | 1 kHz, 판정과 같은 루프 | **2 kHz, 코어 1 전용** (코어 0은 판정·USB·설정) |
| 필터 | 3점 중앙값 + IIR | **One-Euro** — 정지 떨림은 강하게 누르고, 빠르게 움직일 때 지연은 짧게 (계단 90 % 약 3 ms) |
| 축 | 10비트 | **16비트** |
| 보고 | 매 루프 | **바뀐 것만** |
| 센서 오류 | 없음 | 연속 실패 10 ms → **I2C 버스 복구**, 그동안 마지막 기어 유지, 오류 횟수 기록 |
| 설정 | 한 글자 명령 | **줄 단위 JSON 프로토콜** — CLI·웹 공용, CRC 붙여 플래시 저장 |
| 시퀀셜 | 누르는 동안 버튼 | 유지 또는 **고정 펄스(ms)** 선택 (펄스만 받는 게임용) |
| 캘리브레이션 | 중립 + 1단 | + **게이트 4개 각도 각각**, 앞 스톱 (실물 가공 오차 보정) |

## 굽기
1. RP2040-Zero의 BOOT 버튼을 누른 채 USB 연결 → `RPI-RP2` 드라이브가 뜸 (이미 0.6이면 `lsk_shifter.py flash` 또는 웹의 "펌웨어 업데이트 모드")
2. `bin/LSK_HSEQ_Shifter_fw0.6.0_Sunggwan.uf2`를 드라이브에 복사 → 자동 재부팅

## 빌드·시험
```bash
pip install platformio
pio run                                                   # 빌드 (RAM 5.1 %, Flash 3.4 %)
g++ -std=c++17 test/test_logic.cpp -o t && ./t            # 판정·필터·설정·프로토콜 단위 테스트
g++ -std=c++17 test/sim_host.cpp -o /tmp/lsk_sim          # PC 시뮬레이터 (같은 프로토콜 코드)
python3 ../tools/test_cli.py /tmp/lsk_sim                 # CLI ↔ 프로토콜 종단 시험
```

## 출력
| 입력 | 출력 |
|---|---|
| H 모드 | 버튼 1~7 = 1~7단, 버튼 8 = R (표준 / 도그레그 배열) |
| 시퀀셜 모드 (모드 노브 SEQ) | 버튼 9 = 밀기, 버튼 10 = 당기기 (`seq_pulse` 0 = 누르는 동안, >0 = 펄스 ms) |
| 축 | X = 앞뒤 각도 (±`axis_p`°), Y = 좌우 각도 (±`axis_r`°), 16비트 |

## 설정 도구
- **CLI**: `tools/lsk_shifter.py` (pip install pyserial) — `monitor`, `wizard`(캘리브레이션 안내), `set 키 값 --save`, `backup/restore`, `flash`
- **웹**: `tools/web/index.html`을 Chrome/Edge로 열기 → "연결" (WebSerial). 실시간 H 패턴, 설정표, 캘리브레이션, 백업/복원. 장치 없이 "데모"로 미리 보기 가능

## 시리얼 프로토콜 (`src/proto.h`, 한 줄 명령 → 한 줄 JSON)
`info` · `get` · `keys` · `set <키> <값>` · `save` · `load` · `defaults` · `factory` · `cal zero` · `cal gate 0~3` · `cal throw` · `cal gear1` · `status` · `stream <Hz>` · `reboot` · `bootsel`
- 응답: `{"ok":true,...}` / `{"ok":false,"err":"..."}` · 실시간: `{"s":{"th":..,"ph":..,"gate":..,"gear":..,"mode":"H","ok":[1,1],"hz":[1000,2000],...}}`
- 설정 키(21개): `layout gear_on gear_off gate_hyst gate0~3 seq_on seq_off seq_swap seq_pulse k_p k_r zero_p zero_r axis_p axis_r f_mincut f_beta i2c_khz` — 범위·설명은 `keys`

## 캘리브레이션
- 버튼만: 중립(3-4 게이트)에서 버튼(GP7) → 1단(표준: 왼쪽 앞)에 넣고 다시 버튼 → 저장
- 정밀: `lsk_shifter.py wizard` 또는 웹 — 중립 → 앞 스톱 → 게이트 4개 → 저장

## 핀
| 신호 | 핀 |
|---|---|
| 앞뒤 센서 I2C0 | SDA GP4, SCL GP5 |
| 좌우 센서 I2C1 | SDA GP2, SCL GP3 |
| 모드 스위치 | GP6 ↔ GND |
| 캘리브레이션 버튼 | GP7 ↔ GND |

센서: `src/main.cpp`의 `SENSOR_MT6701` (1 = MT6701 14비트, 0 = AS5600 12비트)

## 참고
- Windows는 같은 VID/PID 장치의 이름을 캐시함 → 이전에 Pico로 인식된 적 있으면 장치 관리자에서 제거 후 다시 연결
- 실물 확인 필요: 센서 방향(`k_p`/`k_r` 부호), MT6701 I2C 1 MHz 동작 여부(기본 400 kHz), 1 kHz 보고 시 게임 인식
