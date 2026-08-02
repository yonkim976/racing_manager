# 프로젝트 문서 안내

이 디렉터리에는 현재 권위 문서, 재현 가능한 운영 기준과 미완료 설계만 유지한다.
완료된 작업 지시서와 에이전트용 실행 프롬프트는 현재 상태에 흡수한 뒤 Git 기록에만
남긴다.

## 문서 구성과 우선순위

1. [`SIMULATION_FOUNDATION.md`](SIMULATION_FOUNDATION.md): `FULL` 모드의 제품·물리·데이터 권위 기준
2. [`ABSTRACT_RACE_SIMULATION_DESIGN.md`](ABSTRACT_RACE_SIMULATION_DESIGN.md): 기존 물리를 보존한 추상 결과·방송·즉시 시뮬레이션 실험 설계
3. [`CURRENT_PROJECT_STATUS.md`](CURRENT_PROJECT_STATUS.md): 구현 현황, 최신 검증 결과와 로드맵
4. [`FULL_MODE_CIRCUIT_CALIBRATION_STATUS.md`](FULL_MODE_CIRCUIT_CALIBRATION_STATUS.md): Bahrain·Red Bull Ring의 교통 승인과 남은 기본선 보정
5. [`TIRE_COMPOUND_SPEC.md`](TIRE_COMPOUND_SPEC.md): C1~C5 권위 계약, provisional 사양과 남은 C4/C5 승인
6. [`COLD_OUTLAP_CONTROL_DESIGN.md`](COLD_OUTLAP_CONTROL_DESIGN.md): 출발 90°C·피트 70°C 분리와 콜드 아웃랩 미구현 설계
7. [`ADDING_REAL_CIRCUITS.md`](ADDING_REAL_CIRCUITS.md): 실제 서킷 데이터 추가·검증 절차와 출처 기록
8. [`CIRCUIT_THERMAL_PRESET_SOURCES.md`](CIRCUIT_THERMAL_PRESET_SOURCES.md): 9개 회로 COOL/NORMAL/HOT 시나리오 근거
9. [`AI_RACECRAFT_BENCHMARK.md`](AI_RACECRAFT_BENCHMARK.md): `FULL` 모드 Bahrain AI 판단 회귀 계약
10. [`MODULE_DECOMPOSITION_LOG.md`](MODULE_DECOMPOSITION_LOG.md): 현재 `RaceEngine` 모듈 경계와 후속 분리 규칙
11. [`../desktop/README.md`](../desktop/README.md): Electron 진단 앱 실행·패키징·수명주기·보안 계약

문서와 코드가 충돌하면 코드를 자동으로 정답 처리하지 않는다. 의도된 변경인지 확인하고
권위 문서, 데이터, 테스트와 구현을 같은 변경에서 갱신한다.

## 문서 관리 규칙

- `FULL` 물리 권위와 `ABSTRACT` 논리 결과 권위를 명시적으로 구분한다.
- 현재 상태와 목표 상태를 섞지 않고 `완료`, `부분 완료`, `미구현`을 표시한다.
- 트랙·텔레메트리 데이터에는 출처, 라이선스, 취득일과 변환 도구를 남긴다.
- 실측값, 공식 공개값, 파생값, 게임 보정값과 수치 guard를 구분한다.
- 완료 기록은 재현 명령 또는 테스트 이름과 핵심 수치를 포함한다.
- 일회성 `/private/tmp` 경로, 로컬 사용자 절대 경로와 패키징 산출물은 권위 자료로 취급하지 않는다.

마지막 정리일: **2026-08-03**
