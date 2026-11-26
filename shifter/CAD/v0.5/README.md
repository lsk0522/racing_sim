# LSK H/SEQ Shifter v0.5 — CAD 산출물 (제작: Sunggwan)

`src/export.py`로 생성 (직접 수정하지 말 것). 품번·소재·크기 전체 목록: [`assembly/parts_table.csv`](assembly/parts_table.csv)

| 폴더 | 부품 |
|---|---|
| `mach/sheet` | M1-01 베이스 8t · M2-01/02 타워 6t · M5-01 캠 17t(→16.5) · M5-03 암 판 3t ×2 · M6-01 SEQ 잠금 암 10t · M7-01 게이트 3t |
| `mach/block` | M3-01 크래들 · M3-02 허브 · M5-02 카세트 · M6-02 편심 캠 (Ø14 봉) |
| `print` | M1-02 바닥 커버 · M3-03 꼬리 캡 · M4-01~05 스프링 카트리지 · M6-03~05 · M8-01 노브 · M9-01~03 센서 캡 · M10-01 셸 |
| `buy` | M3-04 레버 Ø8 h6 · M3-05 중공 핀 · M3-06 꼬리 볼트 · M5-04 625ZZ ×3 · `BOM.csv` |
| `assembly` | `_ASSEMBLY_LSK_Shifter_v0.5.step`, `sub/arm.step`, `geometry.json` |

가공 부품 11개의 도면: [`../../docs/dxf/v0.5/`](../../docs/dxf/v0.5/INDEX.md)
