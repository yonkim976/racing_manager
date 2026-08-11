# 프로젝트 문서 안내

이 디렉터리에는 현재 권위 문서, 재현 가능한 운영 기준과 미완료 설계만 유지한다.
완료된 작업 지시서와 에이전트용 실행 프롬프트는 현재 상태에 흡수한 뒤 Git 기록에만
남긴다.

## 문서 구성과 우선순위

1. [`SIMULATION_FOUNDATION.md`](SIMULATION_FOUNDATION.md): `FULL` 모드의 제품·물리·데이터 권위 기준
2. [`ABSTRACT_RACE_SIMULATION_DESIGN.md`](ABSTRACT_RACE_SIMULATION_DESIGN.md): 기존 물리를 보존한 추상 결과·방송·즉시 시뮬레이션 실험 설계
3. [`DUAL_ENGINE_ARCHITECTURE.md`](DUAL_ENGINE_ARCHITECTURE.md): FULL·ABSTRACT 독립 엔진 경계, 공유 계약과 물리적 이동 순서
4. [`ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md): 현재 추상 prototype의 저장·좌표·운동·교통·피트·사건·제품 연결을 단계별로 재승인하는 작업 지시
5. [`ABSTRACT_RACE_SIMULATION_STAGE4_CORRECTION_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE4_CORRECTION_DIRECTIVE.md): 단계 4 독립 검증에서 남은 reservation·conflict 계측·상태기·전체 매트릭스 최종 보완 지시
6. [`ABSTRACT_RACE_SIMULATION_STAGE4_WORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE4_WORK_DIRECTIVE.md): 단계 4의 20대 교통·car-following·corridor·crossing·full-field broadcast 최초 작업 지시
7. [`ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json`](ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json): 단계 4 Bahrain/RBR 10 seed·10랩, 80-run Instant/controlled Broadcast 전체 승인 매트릭스 (`abstract-stage4-traffic-v3`, SHA-256 `4946e797…`)
8. [`ABSTRACT_RACE_SIMULATION_STAGE3_BROADCAST_CORRECTION_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE3_BROADCAST_CORRECTION_DIRECTIVE.md): 단계 3의 checkpoint-snap 제거와 실제 0.10초 single-probe 방송 재보완 기록
9. [`ABSTRACT_RACE_SIMULATION_STAGE3_EVENT_TIMELINE_CORRECTION_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE3_EVENT_TIMELINE_CORRECTION_DIRECTIVE.md): 단계 3의 race-start event 지연 최종 보완 기록
10. [`CURRENT_PROJECT_STATUS.md`](CURRENT_PROJECT_STATUS.md): 구현 현황, 최신 검증 결과와 로드맵
11. [`FULL_MODE_CIRCUIT_CALIBRATION_STATUS.md`](FULL_MODE_CIRCUIT_CALIBRATION_STATUS.md): Bahrain·Red Bull Ring의 교통 승인과 남은 기본선 보정
12. [`TIRE_COMPOUND_SPEC.md`](TIRE_COMPOUND_SPEC.md): C1~C5 권위 계약, provisional 사양과 남은 C4/C5 승인
13. [`COLD_OUTLAP_CONTROL_DESIGN.md`](COLD_OUTLAP_CONTROL_DESIGN.md): 출발 90°C·피트 70°C 분리와 콜드 아웃랩 미구현 설계
14. [`ADDING_REAL_CIRCUITS.md`](ADDING_REAL_CIRCUITS.md): 실제 서킷 데이터 추가·검증 절차와 출처 기록
15. [`CIRCUIT_THERMAL_PRESET_SOURCES.md`](CIRCUIT_THERMAL_PRESET_SOURCES.md): 9개 회로 COOL/NORMAL/HOT 시나리오 근거
16. [`AI_RACECRAFT_BENCHMARK.md`](AI_RACECRAFT_BENCHMARK.md): `FULL` 모드 Bahrain AI 판단 회귀 계약
17. [`MODULE_DECOMPOSITION_LOG.md`](MODULE_DECOMPOSITION_LOG.md): 현재 `RaceEngine` 모듈 경계와 후속 분리 규칙
18. [`../desktop/README.md`](../desktop/README.md): Electron 진단 앱 실행·패키징·수명주기·보안 계약

단계 4 진단 artifact는 다음 단일 명령으로 run 기록과 matrix/quality/resources 집계를 함께 재생성한다.

```bash
PYTHONPATH=backend backend/.venv/bin/python backend/tools/diagnose_abstract_stage4_traffic.py \
  --matrix --output docs/ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json
```

문서와 코드가 충돌하면 코드를 자동으로 정답 처리하지 않는다. 의도된 변경인지 확인하고
권위 문서, 데이터, 테스트와 구현을 같은 변경에서 갱신한다.

현재 `ABSTRACT_BROADCAST` 제품 경로는 전체 레이스를 먼저 계산한 뒤 다시 재생하는 구조가 아니다.
앱은 `PROGRESS_V5` 권위를 명시하고, 예선·그리드만 준비한 뒤 하나의 진행률 cursor와 presentation
adapter가 최대 30초 선행 버퍼를 생산하며 프런트엔드는 `F1P1` binary pose를 재생한다. 사용자
pace·pit 명령은 현재 표시 tick checkpoint로 되감아 아직 공개하지 않은 버퍼를 폐기·재생산한다.
API에서 authority를 생략한 Stage 4 호환 경로도 삭제하지 않았다. 시작 지연·전송 계약은
[`CURRENT_PROJECT_STATUS.md`](CURRENT_PROJECT_STATUS.md)의 36절, 전략 개입·checkpoint 계약은 37절을
기준으로 한다.

## 문서 관리 규칙

- `FULL` 물리 권위와 `ABSTRACT` 논리 결과 권위를 명시적으로 구분한다.
- 현재 상태와 목표 상태를 섞지 않고 `완료`, `부분 완료`, `미구현`을 표시한다.
- 트랙·텔레메트리 데이터에는 출처, 라이선스, 취득일과 변환 도구를 남긴다.
- 실측값, 공식 공개값, 파생값, 게임 보정값과 수치 guard를 구분한다.
- 완료 기록은 재현 명령 또는 테스트 이름과 핵심 수치를 포함한다.
- 일회성 `/private/tmp` 경로, 로컬 사용자 절대 경로와 패키징 산출물은 권위 자료로 취급하지 않는다.

현재 제품 방향은 `FULL`을 기준·보정·회귀 엔진으로 보존하고, 단계 5~7 제품 gate를
통과한 `ABSTRACT`를 레이싱 매니지먼트 기본 경로로 사용하는 것이다. 단계 4까지는
구현됐지만 기본 모드 전환은 아직 승인되지 않았다. 출발·교통 보완 결과는
[`CURRENT_PROJECT_STATUS.md`](CURRENT_PROJECT_STATUS.md)의 38절, 별도 진행률 권위 커널의 첫
검증 결과는 39절, 논리 교통·사고 2단계 결과는 40절을 기준으로 한다.
진행률 기반 피트·타이어 stint 3단계 결과는 같은 문서의 41절을 기준으로 한다.
진행률 기반 접촉·손상·퇴역 4단계 결과는 같은 문서의 42절을 기준으로 한다.
진행률 기반 local yellow·VSC·SC·restart 5단계 결과는 같은 문서의 43절을 기준으로 한다.
진행률 v5의 방송·명령·결과 UI 연결은 같은 문서의 44절을 기준으로 한다.
ABSTRACT 시간 GAP/INT와 속도 UI 분리는 같은 문서의 45절을 기준으로 한다.
ABSTRACT DRS·dirty air·추월 전술과 연출 v2는 같은 문서의 46절을 기준으로 한다.
macOS 재패키징, 실행 후 코드 서명 불변성과 단기 ABSTRACT smoke 결과는 같은 문서의 47절을
기준으로 한다.
ABSTRACT DRS train, 3대 전투 그룹과 pair-only rank 권위는 같은 문서의 48절을 기준으로 한다.
Racecraft Authority v2 macOS 패키징과 실행 전 검증 결과는 같은 문서의 49절을 기준으로 한다.
FULL·ABSTRACT 듀얼 엔진 factory·adapter와 남은 물리적 이동 상태는 같은 문서의 50절을 기준으로 한다.
FULL 예선·중앙 RaceEngine runtime의 첫 물리 이동과 compatibility shim 상태는 같은 문서의 51절을 기준으로 한다.
FULL 핵심 차량 물리 6개 모듈 이동과 package 지연 로딩 경계는 같은 문서의 52절을 기준으로 한다.
FULL 경기 운영 계층 13개 모듈 이동과 patch-safe alias 상태는 같은 문서의 53절을 기준으로 한다.
공용 `TrackGeometryProfile` 계약과 ABSTRACT solver 직접 의존 제거는 같은 문서의 54절을 기준으로 한다.

마지막 정리일: **2026-08-12**
