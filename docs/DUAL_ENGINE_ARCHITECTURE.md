# FULL·ABSTRACT 듀얼 엔진 아키텍처

상태: **FULL·ABSTRACT runtime 물리 이동 완료 · compatibility 정리 대기**
기준일: **2026-08-12**

## 1. 결정

제품은 하나의 혼합 엔진을 만들지 않고 두 엔진을 독립 개발한다.

- `FULL`: 50Hz 차량 물리, 차체 pose와 정밀 텔레메트리 권위
- `ABSTRACT`: 진행률·시간 간격·전략·확률 사건 권위와 파생 지도 pose

두 엔진은 콘텐츠와 외부 API 계약만 공유한다. 상대 엔진의 private 함수, 상태, RNG,
checkpoint, 접촉 판정과 presentation 코드를 import하지 않는다.

## 2. 현재 구현 경계

```text
backend/engines/
├── contracts.py           공통 family·capability·adapter protocol
├── factory.py             SimulationMode의 단일 선택 지점
├── full/adapter.py        FULL qualifying·RaceEngine 생성 경계
├── full/runtime/
│   ├── qualifying.py      FULL 예선 구현 권위
│   ├── race_engine.py     FULL 50Hz 경기 조정자 권위
│   ├── fixed_step.py      결정적 50Hz 누적기
│   ├── speed_profile.py   물리 목표 속도 profile
│   ├── vehicle_physics.py 차량 상태 적분
│   ├── brake_model.py     브레이크 열 모델
│   ├── collision.py       차체 충돌 geometry
│   ├── wake_model.py      tow·dirty-air 물리
│   ├── *_ops.py           사고·피트·전략·출발·타이밍·racecraft
│   ├── safety_car.py      FULL SC·VSC 통제
│   ├── events.py          FULL 확률 event
│   ├── incidents.py       FULL 사고 분류
│   ├── pit_stop.py        FULL pit timing
│   ├── state_contract.py  FULL tick 내부 계약
│   ├── local_trajectory_planner.py  FULL 단기 trajectory lattice
│   ├── planner_scheduler.py         FULL planner cadence
│   ├── physics.py         FULL lap-time·progress 계산
│   ├── tire_model.py      FULL 타이어 열·마모·force
│   ├── vehicle_dynamics.py FULL force primitives
│   ├── trajectory_physics.py FULL vehicle-specific line 평가
│   ├── vehicle_track_solver.py FULL 차량별 전역 주행선 최적화·캐시
│   ├── track_surface.py   FULL 4-wheel surface 접촉
│   └── car_performance.py FULL constructor 성능 변환
├── abstract/adapter.py    snapshot·qualifying·instant·broadcast 생성 경계
└── abstract/runtime/
    ├── engine.py          ABSTRACT 결과 엔진 권위
    ├── progress_race.py   진행률·전략·사건 결과 권위
    ├── broadcast.py       bounded logical broadcast 수명주기
    ├── race.py            Stage 4 호환 교통 결과
    ├── state.py           immutable snapshot·결과·canonical hash
    └── pose.py·kinematics.py·racecraft.py 등 파생 presentation·전술

backend/simulation/
└── track_contracts.py     엔진 중립 주행선·폭·좌표·pose protocol

frontend/src/engines/
├── full/profile.js        physics telemetry·physics pose
├── abstract/profile.js    logical event·strategy map
└── runtimeProfiles.js     simulation mode 선택 지점
```

`main.py`의 qualifying/race setup은 `simulation_engine_factory.for_mode()`만 호출한다.
`SessionManager`도 `RaceEngine`을 직접 생성하지 않고 `FullEngineAdapter.build_race()`를 사용한다.
FULL adapter는 예선과 중앙 경기 구현을 `engines.full.runtime`에서 직접 호출한다. 기존
`simulation.qualifying`은 공개 symbol 재수출 shim이고, `simulation.race_engine`은 module-level patch까지
동일하게 작동하도록 runtime 모듈 객체 자체를 가리키는 alias shim이다. 새 FULL 코드는 이 경로를
사용하지 않는다. FULL adapter는 `simulation.abstract`를, ABSTRACT adapter는 `simulation.race_engine`과
`simulation.vehicle_physics`를 import할 수 없다. 이 규칙은 AST 기반 자동 테스트가 검사한다.
`engines`, `engines.full`, `engines.abstract`의 package initializer는 adapter를 지연 로딩한다.
따라서 공용 geometry가 compatibility alias를 import해도 두 엔진 factory 전체가 재귀 초기화되지 않는다.

## 3. 공유 가능한 계약

- 드라이버·팀·차량 성능 콘텐츠
- 서킷, sector, timing line, DRS와 pit 데이터
- `TrackGeometryProfile`, 주행선 이름, 폭 샘플과 로컬 좌표 변환
- 타이어 nomination과 환경 preset
- `SimulationMode`, 사용자 command, 순위·GAP·이벤트·결과 schema
- 저장 게임, 시즌과 커리어 데이터

다음은 공유하지 않는다.

- 차량 진행과 시간 적분 방식
- 추월·접촉·사고·SC 결과 공식
- 내부 RNG와 checkpoint
- pose 생성과 접촉 geometry
- FULL 물리 텔레메트리와 ABSTRACT 논리 상태

## 4. 프런트엔드 계약

FULL은 `PHYSICS VIEW`, ABSTRACT broadcast는 `STRATEGY MAP` profile을 선택한다. 현재 ABSTRACT도
기존 track renderer를 호환 사용하지만 선택 경계는 분리됐다. 다음 presentation 단계에서
ABSTRACT만 확대 지도·마커·이벤트 효과로 교체하고 FULL renderer는 변경하지 않는다.
ABSTRACT pose·kinematics·grid는 `track_physics` solver 타입을 직접 import하지 않고
`track_contracts.TrackGeometryProfile`만 소비한다.

## 5. 물리 이동 상태와 남은 정리

FULL 예선과 중앙 `RaceEngine`, fixed-step·속도 profile·차량 integrator·브레이크·충돌·wake 구현은
`backend/engines/full/runtime/`으로 이동했다. 사고·피트·전략·SC·출발·타이밍·racecraft 운영 계층도 FULL runtime으로
이동했다. FULL local trajectory planner·scheduler·lap physics도 runtime으로 이동했다.
공용 track geometry·기본 주행선 compiler는 기존 위치에 남고, 차량·타이어별 전역 주행선 최적화와
그 전용 캐시는 `vehicle_track_solver.py`로 분리됐다. 타이어·vehicle dynamics·surface·trajectory
physics·constructor 성능 계산은 FULL runtime으로 이동했다. 공용 compiler의 기존 차량 solver symbol은
호환 import 시에만 FULL runtime으로 지연 연결된다. 데이터 compiler는 아직 기존
`backend/simulation/` 아래에 있다.
ABSTRACT 결과·방송·진행률·pose·교통의 16개 구현 모듈도
`backend/engines/abstract/runtime/`으로 이동했다. adapter, API, 진단 도구와 ABSTRACT 테스트는 새
runtime 경로를 직접 사용하며, 기존 `simulation.abstract.*`에는 동일 모듈 객체를 가리키는 alias만
남는다.
한 번에 이동하면 수백 개 import와 회귀 기준이 동시에 바뀌므로 다음 순서를 지킨다.

1. 현재 듀얼 경계를 커밋해 이동 전 기준점 확보 — 완료 (`2d0236a`)
2. FULL 내부 import를 package-relative 경계로 변환 — 예선·중앙·핵심 물리·경기 운영·local planner 완료
3. 물리 전용 모듈을 `backend/engines/full/runtime/`으로 기계적 이동 — 차량별 track solver까지 완료
4. 기존 `simulation.*` 경로에는 경고 없는 얇은 compatibility shim만 유지 — 이동 모듈 전체 적용
5. ABSTRACT 기존 모듈을 `backend/engines/abstract/runtime/`으로 이동 — 완료
6. API·도구·테스트의 public import를 `engines.*`로 전환 — ABSTRACT 완료, FULL legacy 사용처 감사 대기
7. compatibility 사용처 0 확인 후 shim 삭제 여부 결정

파일 이동 단계에서는 물리 상수, 확률, 결과 hash와 테스트 기대값을 변경하지 않는다.

## 6. 검증 기준

- 모든 `SimulationMode`가 정확히 한 family에 매핑
- adapter 사이 교차 import 0
- FULL과 ABSTRACT 결과·presentation authority 문자열이 서로 다름
- API setup/qualifying 내부 직접 엔진 생성 0
- FULL 세션과 ABSTRACT instant/broadcast 수명주기 회귀 통과
- 각 엔진 전용 변경은 상대 엔진 표적 테스트도 최소 1묶음 실행

코드 디렉터리 분리는 Git 장기 브랜치 분리를 의미하지 않는다. 작업 브랜치는 기능별로 만들고
공통 기본 브랜치에 통합해 콘텐츠·API 계약의 장기 분기를 방지한다.
