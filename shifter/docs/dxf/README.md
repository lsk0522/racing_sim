# 도면 (DXF) — 제작: Sunggwan

리비전별 폴더: `v0.5/` …  각 폴더 구성
```
vX.Y/
  specs/*.yaml   ← 원본. 치수·공차·기하공차·후처리를 여기에 적는다
  *.dxf          A3 도면 (CAD에서 열기, 글꼴 맑은 고딕)
  pdf/ png/      미리보기
  INDEX.md       도면 목록
```
재생성: `cd shifter/src && python3 make_drawings.py --version v0.5` (한 장만: 뒤에 YAML 경로)

## 적용 규격
| 항목 | 규격 | 도면 표기 |
|---|---|---|
| 일반 치수·각도 공차 | ISO 2768-1 **m** | 주기 1 (선형 ±0.1~0.5, 모따기, 각도 표) |
| 일반 기하공차 | ISO 2768-2 **K** | 주기 1 (진직·평면도, 직각도, 대칭, 흔들림) |
| 끼워맞춤 | ISO 286-1/2 | `fit: M7` → 치수 옆 공차 자동 계산 (IT5~12, 축 f g h k m n p / 구멍 F G H K M N P) |
| 기하공차 | ISO 1101, 독립의 원칙 ISO 8015 | 공차 기입틀 + 데이텀 |
| 모서리 | ISO 13715 | 주기 2 |
| 표면 거칠기 | ISO 21920 (기호 ISO 1302) | 주기 3 + 개별 기호 |
| 투상 / 척도 | 3각법 / ISO 5455 (5:1, 2:1, 1:1, 1:2 …) | 표제란 |

## YAML 키
| 키 | 내용 |
|---|---|
| `part`, `part_no`, `name`, `material`, `stock`, `process`, `rev` | 표제란. `part`는 `src/make_drawings.py`의 형상 이름 |
| `views`, `hidden_views`, `scale` | 투상도 (기본 front/top/right), 숨은선 표시할 투상도 (기본 front), 척도 (없으면 자동) |
| `general_tolerance`, `edges`, `roughness_default` | 기본 ISO 2768-mK / ISO 13715 −0.3·+0.3 / Ra 3.2 |
| `finish` | 후처리 순서 목록 (바리 제거, 블라스트, 아노다이징 종류·두께·마스킹 …) |
| `dims` | `type: linear` (`p1`, `p2`, `dir: h/v`, `side: above/below/left/right`) · `type: diameter/radius` (`center`, `r`, `angle`) |
| — 공차 | `fit: H7` (ISO 286) 또는 `tol: "±0.02"` / `tol: "+0.05/0"`, 참고 치수 `text: "(<>)"` |
| `datums` | `A: {view, at, dx, dy}` — 형체 위 점과 기호 위치 |
| `gdt` | `symbol`: flatness, straightness, circularity, cylindricity, profile_line, profile_surface, parallelism, perpendicularity, angularity, position, coaxiality, symmetry, runout / `tol: "Ø0.03"` / `datums: [A, B]` / `view, at, dx, dy` |
| `roughness` | `{view, at, value: "Ra 0.8"}` |
| `notes` | 추가 주기 (탭, 접시, 마스킹 등) |
| `revisions` | `[{rev, date, desc}]` 리비전 표 |

좌표는 모두 **조립 좌표계(mm)** — X 앞, Y 왼쪽, Z 위. 정면도 XZ, 평면도 XY, 우측면도 YZ. `CAD/vX.Y/assembly`의 조립 STEP에서 그대로 읽을 수 있다.
