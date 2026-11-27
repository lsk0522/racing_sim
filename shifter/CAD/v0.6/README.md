# LSK H/SEQ Shifter v0.6 — CAD 산출물 (제작: Sunggwan)

`src/export.py`로 생성 (직접 수정하지 말 것). 품번·소재·크기 전체 목록: [`assembly/parts_table.csv`](assembly/parts_table.csv)
설계 문서: [`../../docs/history/DESIGN_v0.6.md`](../../docs/history/DESIGN_v0.6.md) · 렌더: [`images/`](images/)

v0.5 대비 바뀐 부품: M1-01 베이스 · M1-02 바닥 커버 · M2-01/02 타워(윗변·탭) · M3-04 레버(길이 101) · M6-05 모드 노브 · M7-01 게이트 4t · M8-01 노브
새 부품: M8-02 슬리브 · M8-03 록너트 · M10-01/02 사이드 패널 · M10-03/04 앞·뒤 커버 · M10-05 스모크 창 · M11-01 부트 프레임 · M11-02 레버 칼라 (M10-01 셸 삭제)

| 폴더 | 부품 |
|---|---|
| `mach/sheet` | M1-01 베이스 8t · M2-01/02 타워 6t · M5-01 캠 · M5-03 암 판 3t ×2 · M6-01 SEQ 잠금 암 · M7-01 게이트 4t · M10-01/02 사이드 패널 3t |
| `mach/block` | M3-01 크래들 · M3-02 허브 · M5-02 카세트 · M6-02 편심 캠 · M8-02 슬리브 · M8-03 록너트 |
| `print` | M1-02 바닥 커버 · M3-03 · M4-01~05 · M6-03~05 · M8-01 노브 · M9-01~03 · M10-03/04 커버 · M11-01/02 |
| `buy` | M3-04 레버 · M3-05 중공 핀 · M3-06 꼬리 볼트 · M5-04 625ZZ ×3 · M10-05 스모크 창 · M11-03 부트 재단 패턴 · `BOM.csv` |

가공 도면: [`../../docs/dxf/v0.6/`](../../docs/dxf/v0.6/INDEX.md) (18장 — 책상 클램프 3장 포함).

책상 클램프 (M12, 데스커 상판 28mm): `mach/sheet` M12-01 윗판 6t · M12-02 기둥 12t · M12-03 아래 턱 12t / `print` M12-04 누름 패드 · M12-05 손잡이 / `buy` M12-06 M10×50 볼트.

**Fusion용** [`fusion/`](fusion/): `00_ASSEMBLY_..._fusion.step` (부품마다 `품번_부품` 이름·색, 책상 참고 형상) · `PARAMS.csv` (주요 치수) · [`README_FUSION.md`](fusion/README_FUSION.md) (Fusion 작업 대화 시작 프롬프트). `src/fusion_pack.py`로 생성 (`export.py` 다음에 실행). 부트 재단 패턴: `buy/M11-03_boot_pattern.pdf` (A4 1:1).
