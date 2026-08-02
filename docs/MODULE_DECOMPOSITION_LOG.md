# RaceEngine 모듈 경계

상태: **1차 도메인 분해 완료 · facade 추가 축소 필요**
기준일: **2026-08-03**

이 문서는 과거 추출 일지가 아니라 현재 `FULL` 엔진의 의존 경계와 다음 분리 규칙을
기록한다. 상세 변경 이력은 Git에서 확인한다.

## 현재 구조

```text
RaceEngine facade + physics tick/telemetry core
├── SafetyCarMixin      safety_car.py
├── PitOpsMixin         pit_ops.py
├── RacecraftMixin      racecraft_ops.py
├── TimingOpsMixin      timing_ops.py
├── StrategyOpsMixin    strategy_ops.py
├── IncidentOpsMixin    incident_ops.py
└── StartOpsMixin       start_ops.py

독립 지원 모듈
├── local_trajectory_planner.py
├── vehicle_physics.py
├── vehicle_dynamics.py
├── track_physics.py
└── runtime_constants.py
```

현재 파일 규모는 `race_engine.py` 약 5,877줄이며 mixin을 포함한 핵심 8개 파일은
약 15,937줄이다. 분해는 공개 표면을 유지했지만 facade가 여전히 상태 소유와 50Hz
오케스트레이션을 과도하게 담당한다.

## 의존 원칙

1. 하위 도메인 모듈은 `race_engine.py` facade를 역으로 import하지 않는다.
2. 공용 상수·타입은 독립 모듈로 옮기고 순환 import를 만들지 않는다.
3. 기존 `engine._method(...)` 호출과 테스트 import가 필요한 이름만 명시적으로
   re-export한다.
4. 동작 변경과 모듈 이동을 한 커밋에 섞지 않는다.
5. 한 번에 한 도메인만 추출하고 해당 도메인 회귀와 전체 import 검사를 통과한다.
6. `ABSTRACT` 엔진은 이 facade의 private 메서드를 호출하지 않고 별도 패키지와 상태를
   사용한다.

## 다음 분리 후보

- 50Hz 차량별 step orchestration과 물리 입력 조립
- dashboard/diagnostic payload 생성
- 차량 간 물리 neighborhood와 collision arbitration
- 세션 phase/race-control 전환의 공통 facade

추출 전에는 호출 그래프, 상태 소유자, mutable collection과 테스트의 private import를
먼저 기록한다. 줄 수 감소만으로 완료하지 않고 단방향 의존성과 결정성 불변을 확인한다.

## 검증

```bash
cd backend
.venv/bin/python -m compileall simulation
.venv/bin/python -m unittest discover -s tests
```

새 모듈을 추가하거나 경계를 바꾸면 이 문서의 구조와 실제 클래스 선언이 일치하는지
함께 갱신한다.
