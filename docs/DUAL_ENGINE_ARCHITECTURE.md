# FULL·ABSTRACT 듀얼 엔진 아키텍처

상태: **1차 실행 경계 분리 완료 · FULL runtime 물리 이동 진행 중**
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
│   └── qualifying.py      FULL 예선 구현 권위
└── abstract/adapter.py    snapshot·qualifying·instant·broadcast 생성 경계

frontend/src/engines/
├── full/profile.js        physics telemetry·physics pose
├── abstract/profile.js    logical event·strategy map
└── runtimeProfiles.js     simulation mode 선택 지점
```

`main.py`의 qualifying/race setup은 `simulation_engine_factory.for_mode()`만 호출한다.
`SessionManager`도 `RaceEngine`을 직접 생성하지 않고 `FullEngineAdapter.build_race()`를 사용한다.
FULL adapter는 예선 구현을 `engines.full.runtime.qualifying`에서 직접 호출한다. 기존
`simulation.qualifying`은 기존 도구·테스트를 깨지 않기 위한 얇은 재수출 shim이며 새 FULL 코드는
이 경로를 사용하지 않는다. FULL adapter는 `simulation.abstract`를, ABSTRACT adapter는 `simulation.race_engine`과
`simulation.vehicle_physics`를 import할 수 없다. 이 규칙은 AST 기반 자동 테스트가 검사한다.

## 3. 공유 가능한 계약

- 드라이버·팀·차량 성능 콘텐츠
- 서킷, sector, timing line, DRS와 pit 데이터
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

## 5. 남은 물리 이동 단계

FULL 예선 구현은 `backend/engines/full/runtime/`으로 이동했다. `RaceEngine`과 나머지 물리 전용
모듈은 호환성을 위해 아직 기존 `backend/simulation/` 아래에 있다. 한 번에 이동하면 수백 개 import와
회귀 기준이 동시에 바뀌므로 다음 순서를 지킨다.

1. 현재 듀얼 경계를 커밋해 이동 전 기준점 확보 — 완료 (`2d0236a`)
2. FULL 내부 import를 package-relative 경계로 변환 — 예선 완료, 나머지 대기
3. 물리 전용 모듈을 `backend/engines/full/runtime/`으로 기계적 이동 — 예선 완료, 나머지 대기
4. 기존 `simulation.*` 경로에는 경고 없는 얇은 compatibility shim만 유지 — 예선 적용
5. ABSTRACT 기존 모듈을 `backend/engines/abstract/runtime/`으로 이동
6. API·도구·테스트의 public import를 `engines.*`로 전환
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
