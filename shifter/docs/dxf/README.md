# 도면 (DXF) — 제작: Sunggwan

리비전별 폴더: `v0.5/` …  각 폴더 구성
```
vX.Y/
  specs/*.yaml   ← 원본. 치수·공차·기하공차·후처리를 여기에 적는다
  *.dxf          A4 가로 흑백 도면 (CAD에서 열기, 글꼴 맑은 고딕, 원·원호는 CIRCLE/ARC 실제 곡선)
  pdf/ png/      미리보기
  INDEX.md       도면 목록
```
재생성: `cd shifter/src && python3 export.py && python3 make_drawings.py --version v0.5` (한 장만: 뒤에 YAML 경로)

형상은 `CAD/vX.Y/mach/…/<품번>_<부품>.step`(export.py 산출물)을 읽어 은선 처리한다 — 메시(STL)는 쓰지 않는다.
날카로운 모서리와 윤곽선만 그리고, 접선(매끄럽게 이어지는) 모서리·이음선은 그리지 않는다 (ISO 128).

## 적용 규격
| 항목 | 규격 | 도면 표기 |
|---|---|---|
| 일반 치수·각도 공차 | ISO 2768-1 **m** | 주기 1 (선형 ±0.1~0.5, 모따기, 각도 표) |
| 일반 기하공차 | ISO 2768-2 **K** | 주기 1 (진직·평면도, 직각도, 대칭, 흔들림) |
| 끼워맞춤 | ISO 286-1/2 | `fit: M7` → 치수 옆 공차 자동 계산 (IT5~12, 축 f g h k m n p / 구멍 F G H K M N P) |
| 기하공차 | ISO 1101, 독립의 원칙 ISO 8015 | 공차 기입틀 + 데이텀 |
| 모서리 | ISO 13715 | 주기 2 |
| 표면 거칠기 | ISO 21920 (기호 ISO 1302) | 주기 3 + 개별 기호 |
| 투상 / 척도 | 3각법 ISO 128-30 · 기호 ISO 5456-2 / ISO 5455 (5:1, 2:1, 1:1, 1:2 …) | 추가 정보란 |
| 용지·윤곽 | **A4 가로**, 윤곽선(0.7) + 중심 마크만, 철 여백 20 · 나머지 10 (ISO 5457 / KS), 구역 표시 없음, **흑백** | — |
| 배치 (KS 양식) | 왼쪽 위: 품번 ① + 표면 거칠기 일괄 기호 √Ra 3.2 (√Ra …) / 오른쪽 칸(폭 100) 위→아래: 개정 이력 → 주서 → 부품란 → 표제란 | — |
| 부품란 | 품번 · 품명 · 재질 · 수량 · 소재 · 질량(STEP 부피 × 밀도) | — |
| 표제란 | ISO 7200 항목: 제목 · 도면 번호 · 개정 · 단위 · 시트 · 조립품 · 척도 · 투상법(3각법) · 작성자 · 승인자 · 발행일 · 법적 소유자 · 문서 종류 · 문서 상태 | — |
| 선 | ISO 128-20 선 그룹 0.35 (외형선 0.35, 가는 선 0.18, 숨은선 02, 중심선 04), 윤곽선 0.7 | — |
| 문자 | ISO 3098 (치수 3.5, 주서 3.0, 제목 5) | — |
| 개정 이력 | 오른쪽 위 표 (개정·내용·날짜·작성) | `revisions` |

## YAML 키
| 키 | 내용 |
|---|---|
| `part`, `part_no`, `name`, `material`, `stock`, `process`, `rev` | 표제란. `part`는 `src/make_drawings.py`의 형상 이름 |
| `views`, `hidden_views`, `scale` | 투상도 (기본 front/top/right), 숨은선 표시할 투상도 (기본 front), 척도 (없으면 ISO 5455에서 A4에 맞는 가장 큰 값) |
| `source` | (선택) 다른 STEP 사용 — `shifter/` 기준 경로. 없으면 `CAD/<버전>/`에서 품번으로 찾음 |
| `dept`, `approved`, `status`, `doc_type` | (선택) 표제란 책임 부서·승인자·문서 상태·문서 종류 |
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
