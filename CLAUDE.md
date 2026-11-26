# 작업 규칙 (이 레포에서 작업하는 사람/AI 공통)

- 언어: 한국어. 제작자 표기: **Sunggwan**
- 작업을 시작하면 `PROGRESS.md`부터 읽는다.
- 작업이 끝나면 `PROGRESS.md`를 갱신한다:
  - §1 이력 표 (새 버전이 나왔으면 한 줄)
  - §2 현재 상태 체크박스
  - §3 로드맵 / §4 미결 질문
  - §5 작업 로그에 날짜 + 한 줄
- 쉬프터 CAD는 `shifter/cad/`의 파이썬 소스가 원본. 수정 후 `check.py`(간섭 검사) 통과 → `export.py`로 `shifter/hardware/` 갱신.
- 펌웨어 수정 후 `test/test_logic.cpp` 단위 테스트와 `pio run` 빌드를 둘 다 통과시킨다.
- 버전을 올리면 이전 소스는 `shifter/cad/archive/vXX/`, 이전 설계 문서는 `shifter/docs/history/`로 옮긴다.
