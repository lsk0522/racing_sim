# 펌웨어 — LSK HSEQ Shifter (제작: Sunggwan)

- 보드: Waveshare RP2040-Zero · 코어: Arduino-Pico (earlephilhower) · 빌드: PlatformIO
- USB 이름: 제조사 **Sunggwan**, 제품 **LSK HSEQ Shifter** (`platformio.ini`)
- 버전: 0.5.0

## 굽기 (가장 쉬운 방법)
1. RP2040-Zero의 BOOT 버튼을 누른 채 USB 연결 → `RPI-RP2` 드라이브가 뜸
2. `bin/LSK_HSEQ_Shifter_fw0.5.0_Sunggwan.uf2`를 드라이브에 복사 → 자동 재부팅

## 빌드
```bash
pip install platformio
pio run                 # 빌드
pio run -t upload       # 업로드 (BOOTSEL 모드)
g++ -std=c++17 test/test_logic.cpp -o t && ./t   # 판정 로직 단위 테스트 (PC)
```

## 동작
| 입력 | 출력 |
|---|---|
| H 모드 | 버튼 1~7 = 1~7단, 버튼 8 = R (표준 / 도그레그 배열) |
| 시퀀셜 모드 (모드 노브 ↓) | 버튼 9 = 밀기, 버튼 10 = 당기기 (7.0°에서 트리거) |
| 축 | X = 앞뒤 각도, Y = 좌우 각도 |

## 캘리브레이션
1. 중립(3-4 게이트)에서 캘리브레이션 버튼 (GP7)
2. 1단(왼쪽 앞, 도그레그면 왼쪽 뒤)에 넣고 다시 버튼 → 플래시에 저장

## 시리얼 명령 (115200)
`i` 장치 정보 · `p` 실시간 값 · `l` 표준/도그레그 · `x` 시퀀셜 방향 반전 · `r` 초기화 · `c` 캘리브레이션 단계

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
- 빌드 확인: 2026-11-26, RAM 3.6% / Flash 2.3%
