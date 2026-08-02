# 프로젝트 문서 안내

현재 사용하는 프로젝트 문서만 이 디렉터리에 유지한다. 완료된 지시서, 임시 체크리스트와 과거 인수인계 문서는 현재 상태 문서에 필요한 내용을 흡수한 뒤 삭제한다.

## 문서 구성과 우선순위

1. [`SIMULATION_FOUNDATION.md`](SIMULATION_FOUNDATION.md): 변경하면 안 되는 제품·물리·데이터 권위 기준
2. [`CURRENT_PROJECT_STATUS.md`](CURRENT_PROJECT_STATUS.md): 구현 현황, 검증 결과, 그래픽 방향과 다음 로드맵
3. [`ADDING_REAL_CIRCUITS.md`](ADDING_REAL_CIRCUITS.md): 실제 서킷 데이터 추가·검증 절차
4. [`AI_RACECRAFT_BENCHMARK.md`](AI_RACECRAFT_BENCHMARK.md): 바레인 AI 판단 벤치마크 계약과 실행법
5. [`MODULE_DECOMPOSITION_LOG.md`](MODULE_DECOMPOSITION_LOG.md): `RaceEngine` 도메인 분리 변경 이력 (추출할 때마다 갱신)
6. [`RACE_LIFECYCLE_AND_SC_TAIL_FIX_DIRECTIVE.md`](RACE_LIFECYCLE_AND_SC_TAIL_FIX_DIRECTIVE.md): 레이스 종료 메모리와 SC 후미 65km/h 고착 문제의 재현·수정 지시서
7. [`MACOS_DESKTOP_DIAGNOSTIC_APP_PLAN.md`](MACOS_DESKTOP_DIAGNOSTIC_APP_PLAN.md): Electron 기반 macOS 진단 앱, Python sidecar 수명주기와 프로세스별 메모리 계측 구현 계획
8. [`TIRE_THERMAL_FOUNDATION_PHASE_1_5_DIRECTIVE.md`](TIRE_THERMAL_FOUNDATION_PHASE_1_5_DIRECTIVE.md): C1~C5·동적 날씨 도입 전 전후축 에너지, 열수지, 환경 입력, 클램프 진단과 장기 열 평형 1~5단계 작업 지시서
9. [`CIRCUIT_THERMAL_ENVIRONMENT_PHASE_1_DIRECTIVE.md`](CIRCUIT_THERMAL_ENVIRONMENT_PHASE_1_DIRECTIVE.md): C1~C5 물성 도입 전 회로별 정적 열환경 프리셋과 세션 전달 계약
10. [`CIRCUIT_THERMAL_PRESET_SOURCES.md`](CIRCUIT_THERMAL_PRESET_SOURCES.md): 9개 회로 COOL/NORMAL/HOT 시나리오 근거와 provisional 상태
11. [`TIRE_COMPOUND_C1_C5_PHASE_2_DIRECTIVE.md`](TIRE_COMPOUND_C1_C5_PHASE_2_DIRECTIVE.md): 물리 C1~C5와 주말 Hard/Medium/Soft 역할 분리, 회로별 지명, 컴파운드 열창·전략·UI·승인 작업 지시서
12. [`COLD_OUTLAP_CONTROL_DESIGN.md`](COLD_OUTLAP_CONTROL_DESIGN.md): 출발 90°C와 피트 70°C 계약 분리, 콜드 피트아웃 상태·AI 제어·합류·검증 설계
13. [`RED_BULL_RING_OPTIMIZATION_PHASE_1_DIRECTIVE.md`](RED_BULL_RING_OPTIMIZATION_PHASE_1_DIRECTIVE.md): Red Bull Ring 형상·텔레메트리·단독 5랩·20대·피트·SC·C2~C4 수직 승인 작업 지시서
14. [`RACE_RACING_LINE_CALIBRATION_PHASE_1_DIRECTIVE.md`](RACE_RACING_LINE_CALIBRATION_PHASE_1_DIRECTIVE.md): Bahrain·Red Bull Ring의 레이스 연료·Medium·Standard 기준 기본 레이싱라인, progress 정렬, 연속 코너 추종과 5랩·15랩·20대 승인 작업 지시서. 퀄리파잉 보정은 후속 단계로 분리

문서와 코드가 충돌하면 코드를 무조건 정답으로 간주하지 않는다. 기준 설계와 현재 구현을 함께 확인하고, 의도된 변경이면 기준 문서를 먼저 갱신한다.

## 문서 관리 규칙

- 중요한 설계 결정은 기준 문서에 먼저 기록한 뒤 코드로 옮긴다.
- 현재 상태와 목표 상태를 섞지 않는다. 미구현 항목은 명시적으로 `미구현` 또는 `부분 구현`으로 표시한다.
- 트랙과 텔레메트리 데이터에는 출처, 원본 식별자, 라이선스, 취득일, 변환 도구와 변환 버전을 남긴다.
- 실측값, 공식값, 추정값, 게임 밸런스값을 서로 다른 필드와 이름으로 관리한다.
- 완료된 작업은 테스트 이름 또는 재현 명령과 함께 기록한다.

마지막 정리일: 2026-08-02
