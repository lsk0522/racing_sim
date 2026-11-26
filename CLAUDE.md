# 작업 규칙 (이 레포에서 작업하는 사람/AI 공통)

- 언어: 한국어. 제작자 표기: **Sunggwan**
- 작업을 시작하면 `PROGRESS.md`부터 읽는다.
- 작업이 끝나면 `PROGRESS.md`를 갱신한다:
  - §1 이력 표 (새 버전이 나왔으면 한 줄)
  - §2 현재 상태 체크박스
  - §3 로드맵 / §4 미결 질문
  - §5 작업 로그에 날짜 + 한 줄
- 쉬프터 CAD 원본은 `shifter/src/`의 파이썬 소스. 수정 후 `check.py`(간섭 검사) 통과 → `export.py`로 `shifter/CAD/vX.Y/` 갱신.
  - `CAD/vX.Y/mach/sheet` 판재 가공 · `mach/block` 블록·봉 가공 · `print` 출력 · `buy` 구매품 · `assembly` 조립
  - 파일 이름은 `품번_부품` (예: `M2-01_tower_negY.step`). 새 부품은 `export.py`의 `CATALOG`에 품번·폴더를 등록.
- 도면은 `shifter/docs/dxf/vX.Y/specs/*.yaml`이 원본 (치수·공차·기하공차·후처리). 수정 후 `src/make_drawings.py`로 DXF·PDF·PNG 재생성.
  - 일반 공차 ISO 2768-mK, 끼워맞춤 ISO 286, 기하공차 ISO 1101, 모서리 ISO 13715, 거칠기 ISO 21920, 3각법, 척도 ISO 5455.
  - 도면을 고치면 해당 YAML의 `rev`/`revisions`도 올린다.
- 펌웨어 수정 후 `test/test_logic.cpp` 단위 테스트와 `pio run` 빌드를 둘 다 통과시킨다.
- 버전을 올리면(리비전):
  1. 이전 소스는 `shifter/src/archive/vXX/`로 복사
  2. `export.py`의 `VERSION`을 올려 `shifter/CAD/v새버전/` 새 폴더 생성 (이전 버전 폴더는 그대로 둔다)
  3. `shifter/docs/dxf/v이전` 폴더를 `v새버전`으로 복사 → 바뀐 부품의 YAML만 수정 → 재생성
  4. 이전 설계 문서는 `shifter/docs/history/`로 옮긴다.
- 폴더 이름 주의: 소스는 `src/`, 산출물은 `CAD/` (대소문자만 다른 `cad/`는 Windows·macOS에서 충돌하므로 쓰지 않는다).
