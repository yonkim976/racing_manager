# RaceEngine 모듈 분해 변경 이력

상태: **1차 도메인 분해 완료 · 후속 정리 진행 중**
목적: `backend/simulation/race_engine.py` God Object를 도메인 모듈로 분리하되, 공개 메서드명·테스트 import·런타임 동작은 유지한다.

이 문서는 분해·정리 작업이 끝날 때마다 갱신한다. 새 추출을 할 때 **같은 날짜 항목을 추가**하고, 파일 줄 수·테스트 결과를 남긴다.

## 분해 원칙

1. 동작 변경 없이 코드 위치만 옮긴다 (behavior-preserving refactor).
2. `RaceEngine`은 mixin을 상속해 기존 `engine._method(...)` 호출을 유지한다.
3. 내부 테스트/도구가 `from simulation.race_engine import X`로 쓰는 이름은
   `race_engine.__all__`에 넣어 re-export한다 (분해 이전 module-level 전체 호환은 목표 아님).
4. 한 번에 한 도메인만 분리하고, 해당 도메인 회귀 테스트를 통과시킨 뒤 다음으로 간다.
5. 분리된 하위 모듈은 `race_engine` facade를 다시 import하지 않는다.
6. 이 문서를 갱신하지 않은 분해는 완료로 보지 않는다.

## 현재 구조

```
RaceEngine  (facade + physics tick / telemetry payload core)
    ├── SafetyCarMixin     (simulation/safety_car.py)
    ├── PitOpsMixin        (simulation/pit_ops.py)
    ├── RacecraftMixin     (simulation/racecraft_ops.py)
    ├── TimingOpsMixin     (simulation/timing_ops.py)
    ├── StrategyOpsMixin   (simulation/strategy_ops.py)
    ├── IncidentOpsMixin   (simulation/incident_ops.py)
    └── StartOpsMixin      (simulation/start_ops.py)

공유 상수 (facade 비의존):
    ├── runtime_constants.py              → GRID_LAUNCH_MERGE_DISTANCE_M
    └── local_trajectory_planner.py       → LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS
```

### 현재 규모 (줄 수)

| 파일 | 줄 수 |
|---|---:|
| `race_engine.py` | 4,119 |
| `racecraft_ops.py` | 3,118 |
| `incident_ops.py` | 1,435 |
| `safety_car.py` | 1,050 |
| `pit_ops.py` | 898 |
| `timing_ops.py` | 608 |
| `strategy_ops.py` | 555 |
| `start_ops.py` | 315 |
| `runtime_constants.py` | 10 |

관련 기존 모듈 (분해 대상 아님):

- `simulation/pit_stop.py` — 피트 정지 시간 계산 헬퍼
- `simulation/ai_strategy.py` — 피트 컴파운드 선택 헬퍼 (`strategy_ops`와 별개)
- `simulation/incidents.py` — 사고 분류/확률 헬퍼 (`incident_ops`와 별개)
- `simulation/collision.py` — 기하/스윕 충돌 헬퍼 (`incident_ops`와 별개)
- `simulation/racecraft_benchmark.py` — AI 벤치마크 러너 (`racecraft_ops`와 별개)
- `simulation/local_trajectory_planner.py` — 로컬 궤적 플래너 본체
- `simulation/runtime_constants.py` — mixin 간 공유 상수 (facade 역참조 방지)

---

## 2026-07-25 — Safety Car / VSC 분리

### 변경

- 추가: `backend/simulation/safety_car.py` (`SafetyCarMixin`)
- 수정: `backend/simulation/race_engine.py`가 `SafetyCarMixin` 상속
- 이동: SC/VSC 상수, 상태 초기화, 대열·언랩·속도 캡·following constraint·페이스 계수·AI SC 피트 판단

### 유지 (공용 유틸)

- `_on_track_leader`, `_set_state_total_progress`, `_total_progress_at_or_after`
- hazard clearance, 테스트용 race-control API

### 규모 (당시)

| 파일 | 대략 줄 수 |
|---|---:|
| `race_engine.py` | ~11,500 → ~10,600 |
| `safety_car.py` | ~1,050 |

### 검증

- SC/VSC 관련 엔진 테스트 37개 통과

### 비고

- 직전 SC order yield(물리 역전 시 속도 캡 give-back) 변경분도 함께 `safety_car.py`에 포함된다.

---

## 2026-07-25 — Pit Ops 분리

### 변경

- 추가: `backend/simulation/pit_ops.py` (`PitOpsMixin`, `PitMergeDecision`)
- 수정: `RaceEngine(SafetyCarMixin, PitOpsMixin)`
- 이동: `PIT_*` 상수, 피트 경로/박스/개러지 횡이동, `request_pit`, 진입·정차·출구·merge yield/hold

### 유지

- `simulation/pit_stop.py`의 타이어 교체 시간 헬퍼
- `_progress_distance` / `_crossed_progress` (트랙 공용)

### 규모 (당시)

| 파일 | 대략 줄 수 |
|---|---:|
| `race_engine.py` | ~10,600 → ~9,700 |
| `pit_ops.py` | ~900 |

### 검증

- 피트 + SC 관련 엔진 테스트 56개 통과

### 부수 수정

- `test_pit_rejoin_rule_yields_holds_then_merges_when_occupancy_clears`:
  엔진의 합류 ETA 5초 클램프와 테스트 배치가 어긋나 있던 기존 불일치를 테스트 쪽에 맞춤 (추출 회귀가 아님).

---

## 2026-07-26 — Racecraft Ops 분리

### 변경

- 추가: `backend/simulation/racecraft_ops.py` (`RacecraftMixin`)
- 수정: `RaceEngine(SafetyCarMixin, PitOpsMixin, RacecraftMixin)`
- 이동 대상:
  - 타입: `BattleEffect`, `SideBySideBattle`, `ManeuverGroup`, `OvertakeOpportunityAssessment`, `LocalPullOutDecision`, `PendingOvertakeCommand`, `BattleIntent`, `CornerCorridorPrediction`, `ForcedWideAftermath`
  - 상수: `BATTLE_*`, `MANEUVER_*`, `ATTACK_LINE_*`, `DEFENDER_LINE_*`, `CORNER_*`(corridor), `AI_ATTACK_*` / `AI_DEFEND_*`, `TRAFFIC_*` 공격 관련, `DRS_MAX_BONUS` 등
  - 메서드: side-by-side 수명주기, maneuver group, attack/defend 라인, corner corridor, forced-wide, battle event/effect, overtake assessment

### 유지 (의도적)

- `DriverInputError` (이후 timing 분리 시 `TimingLoop`는 `timing_ops`로 이동)
- local trajectory planner 본체 (`_update_local_trajectory_plan` 등)
- hazard corridor / incident
- AI pace strategy (`_tick_ai_pace_cooldowns` 등)
- `GRID_LAUNCH_MERGE_DISTANCE_M`, `LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS` —
  당시에는 `race_engine`에 남기고 racecraft가 facade를 런타임 조회했음
  (이후 **Facade 역참조 제거** 단계에서 원본 모듈/`runtime_constants`로 이동)

### 규모

| 파일 | 줄 수 |
|---|---:|
| `race_engine.py` | ~9,700 → **6,679** |
| `racecraft_ops.py` | **3,127** |
| `safety_car.py` | 1,051 (유지) |
| `pit_ops.py` | 899 (유지) |

### 검증

- 레이스크래프트 관련 엔진 테스트 **57개 통과** (~68초)
- 바레인 10대 50초 smoke tick 통과

### 주의

- `racecraft_ops.py`와 `racecraft_benchmark.py`는 다른 모듈이다. 벤치마크는 엔진 API를 호출만 한다.
- `race_engine`은 남은 물리 경로가 쓰는 racecraft 상수/타입을 re-export한다.

---

## 2026-07-26 — Timing Ops 분리

### 변경

- 추가: `backend/simulation/timing_ops.py` (`TimingOpsMixin`, `TimingLoop`)
- 수정: `RaceEngine(..., TimingOpsMixin)`
- 이동 대상:
  - 타입/상수: `TimingLoop`, `TIMING_CROSSING_LAPS_TO_RETAIN`
  - 상태 초기화: `_init_timing_state` (루프·crossing·GAP/INT·미니섹터 베스트)
  - 메서드: sector/mini-sector 루프, crossing 기록, live GAP/INT, lap history,
    `_complete_lap`, `build_timing_payload`, `build_history_state`

### 유지 (의도적)

- `_update_positions` — SC/배틀 순위와 강결합
- `_finalize_event_positions`, `_leader_lap`
- `_gap_seconds_between` / `_refresh_lap_variation` — 이후 strategy 분리로 이동

### 규모

| 파일 | 줄 수 |
|---|---:|
| `race_engine.py` | ~6,679 → **6,113** |
| `timing_ops.py` | **608** |
| `racecraft_ops.py` | 3,127 (유지) |
| `safety_car.py` | 1,051 (유지) |
| `pit_ops.py` | 899 (유지) |

### 검증

- 타이밍/섹터/갭/히스토리 관련 엔진·세션 테스트 **23개 통과** (~71초)
- `TIMING_CROSSING_LAPS_TO_RETAIN` / `TimingLoop`는 `race_engine`에서 re-export

### 비고

- `_complete_lap`은 피트 요청 처리(`_start_pit_stop`)를 포함하지만, 타이밍·랩 완료
  진입점이라 mixin에 두고 `self`로 pit ops를 호출한다.

---

## 2026-07-26 — Strategy Ops 분리

### 변경

- 추가: `backend/simulation/strategy_ops.py` (`StrategyOpsMixin`)
- 수정: `RaceEngine(..., StrategyOpsMixin)`
- 이동 대상:
  - 상수: `PACE_MODE_*`, `FUEL_*` / `MAX_*_FUEL_*` / `IDLE_FUEL_*`,
    `TIRE_SLIDE_WEAR_LAPS_PER_JOULE`, `AI_PACE_*`, `AI_*_TIRE_LIFE`, `AI_FRESH_TIRE_USAGE`
  - 상태 초기화: `_init_strategy_state` (페이스 전환·랩 variance·AI cooldown)
  - 메서드: pace transition/interpolation, fuel/tire wear·thermal, base lap time,
    AI pace/strategy (`_choose_ai_pace_mode`, `_run_ai_strategy`, `_run_ai_pace_modes`),
    `set_pace_mode`, `_gap_seconds_between`, `_refresh_lap_variation`

### 유지 (의도적)

- `simulation/ai_strategy.py`의 `should_pit` / `choose_pit_tire` 헬퍼
- wake/DRS / local trajectory planner 본체
- `AI_TACTICAL_DECISION_INTERVAL_SECONDS` (플래너 주기)
- `AI_ATTACK_*` / `AI_DEFEND_*` 갭 상수 — `racecraft_ops`에 유지, strategy가 import

### 규모

| 파일 | 줄 수 |
|---|---:|
| `race_engine.py` | ~6,113 → **5,622** |
| `strategy_ops.py` | **556** |
| `timing_ops.py` | 608 (유지) |
| `racecraft_ops.py` | 3,127 (유지) |
| `safety_car.py` | 1,051 (유지) |
| `pit_ops.py` | 899 (유지) |

### 검증

- pace/fuel/tire/AI strategy 관련 테스트 **30개 통과** (~12초)
- `AI_PACE_*` 등 상수는 `race_engine`에서 re-export

### 비고

- 당시 `_apply_pace_mode`는 순환 import 방지를 위해
  `LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS`를 `race_engine`에서 lazy import로 조회했음
  (이후 **Facade 역참조 제거** 단계에서 `local_trajectory_planner` 직접 import로 변경)

---

## 2026-07-26 — Incident Ops 분리

### 변경

- 추가: `backend/simulation/incident_ops.py` (`IncidentOpsMixin`, `DriverInputError`)
- 수정: `RaceEngine(..., IncidentOpsMixin)`
- 이동 대상:
  - 상수: `INCIDENT_*`, `COLLISION_*`, `HAZARD_*`, `FOLLOWING_MIN_BUMPER_GAP_M`
  - 상태 초기화: `_init_incident_ops_state`
  - 메서드: stopped-hazard 클러스터/회피 코리도, collision resolve/facts,
    `_apply_incident`, `_tick_stopped_hazard_clearance`, `retire_driver_for_testing`,
    driver input error tick, local yellow / overtake race-control gate

### 유지 (의도적)

- `simulation/incidents.py` / `simulation/collision.py` 헬퍼
- `_body_pose`, `_tick_safety_car_after_cars` (SC가 hazard clearance를 `self`로 호출)
- `_total_progress_at_or_after` (SC/공용)
- `MANEUVER_CLEARANCE_MARGIN_M` — `racecraft_ops`에 유지, incident가 import

### 규모

| 파일 | 줄 수 |
|---|---:|
| `race_engine.py` | ~5,622 → **4,256** |
| `incident_ops.py` | **1,435** |
| `strategy_ops.py` | 556 (유지) |
| `timing_ops.py` | 608 (유지) |
| `racecraft_ops.py` | 3,127 (유지) |
| `safety_car.py` | 1,051 (유지) |
| `pit_ops.py` | 899 (유지) |

### 검증

- hazard/incident/collision 관련 테스트 **25개 통과** (~13초)
- `FOLLOWING_MIN_BUMPER_GAP_M` / `DriverInputError` 등은 `race_engine`에서 re-export

### 비고

- SC cleanup 경로(`_tick_safety_car_after_cars`)는 hazard clearance에 의존하지만
  SC 도메인에 남겨 두고 mixin 메서드를 호출한다.

---

## 2026-07-26 — Start Ops 분리

### 변경

- 추가: `backend/simulation/start_ops.py` (`StartOpsMixin`)
- 수정: `RaceEngine(..., StartOpsMixin)`
- 이동 대상:
  - 상수: `GRID_*` (슬롯 간격, 라이트 타이밍, launch merge/lockout)
  - 상태 초기화: `_init_start_ops_state`
  - 메서드: `_init_grid`, grid launch lateral helpers, `get_grid_order` /
    `get_grid_slots`, `_advance_start_sequence`, `_tick_start_display`

### 유지 (의도적)

- `_initial_speed_kph` — 그리드 외 부트스트랩/물리에서도 사용
- 물리 tick 본체 (`tick` / `_tick_fixed_step`) — RaceEngine 코어로 잔류
- `GRID_LAUNCH_MERGE_DISTANCE_M`는 당시 `race_engine` re-export로 유지
  (이후 역참조 정리에서 원본을 `runtime_constants`로 이동)

### 규모 (당시)

| 파일 | 줄 수 |
|---|---:|
| `race_engine.py` | ~4,256 → **3,985** |
| `start_ops.py` | **316** |

### 검증

- grid/start_sequence/lights_out/launch 관련 테스트 **10개 통과** (~20초)

### 비고

- `_init_grid`는 타이밍·페이스·연료 상태도 채우지만, 그리드 배치 진입점이라
  start mixin에 두고 다른 mixin API를 `self`로 호출한다.

---

## 2026-07-26 — Facade 역참조 제거 · 공개 표면 정리

### 변경

- 추가: `backend/simulation/runtime_constants.py`
  - `GRID_LAUNCH_MERGE_DISTANCE_M` 원본 소유
- 이동: `LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS` → `local_trajectory_planner.py`
- 수정:
  - `racecraft_ops.py` — `race_engine` lazy import 제거, 원본 모듈에서 직접 import
  - `strategy_ops.py` — `race_engine` lazy import 제거, planner 상수 직접 import
  - `start_ops.py` — launch merge 상수를 `runtime_constants`에서 사용
  - `race_engine.py` — 위 상수를 re-export하고 지원 공개 표면을 `__all__`로 고정
    (분해 이전 module-level 이름 전체 호환이 아니라, 내부 사용·의도적 re-export만)

### 의존 방향 (정리 후)

```
runtime_constants / local_trajectory_planner
        ↑
 domain mixins (racecraft, strategy, start, ...)
        ↑
   race_engine facade  (re-export only)
```

하위 mixin → `race_engine` 런타임 import: **없음**
(`racecraft_benchmark.py`는 엔진 소비자이므로 facade import 유지)

### 유지한 공개 re-export (호환)

지원 범위는 `race_engine.__all__`에 **명시된 이름만**이다.
프로젝트 내부 테스트/도구가 쓰는 facade import는 모두 포함하되,
분해 이전 module-level 이름 전체를 재현하지는 않는다.
다른 심볼은 소유 도메인 모듈에서 직접 import한다.

### 규모 (정리 후)

| 파일 | 줄 수 |
|---|---:|
| `race_engine.py` | **4,119** (`__all__` 명시 포함) |
| `racecraft_ops.py` | **3,118** |
| `incident_ops.py` | **1,435** |
| `safety_car.py` | **1,050** |
| `pit_ops.py` | **898** |
| `timing_ops.py` | **608** |
| `strategy_ops.py` | **555** |
| `start_ops.py` | **315** |
| `runtime_constants.py` | **10** |

### 검증

- mixin 간 중복 메서드 없음 / `RaceEngine.__mro__` 정상
- domain mixin → `race_engine` 역참조 없음
- `race_engine.__all__` 130개 — 내부 테스트/도구 facade import 전부 포함
- `python -m compileall backend/simulation backend/tests` 성공
- `PYTHONPATH=backend pytest backend/tests` — **426 passed, 27 subtests passed**
- `frontend npm run build` 성공
- `git diff --check` 통과

### 남은 구조적 한계

- `race_engine.py` (~4k)에 물리 tick·텔레메트리·페이로드 조립 코어가 남아 있다.
- 선택적 후속: `_tick_fixed_step` / `build_*_state` 분리, re-export `__all__` 자동화.

---

## 다음 추출용 체크리스트

새 도메인 추출을 할 때만 사용한다 (1차 분해 자체는 완료).

- [ ] 새 모듈 파일 추가 및 mixin 연결
- [ ] `race_engine` re-export로 기존 테스트 import 유지
- [ ] 하위 모듈이 `race_engine` facade를 import하지 않음
- [ ] 해당 도메인 회귀 테스트 통과
- [ ] 이 문서에 날짜 항목 추가 (동기/파일/검증)
- [ ] 필요 시 `docs/README.md` 문서 목록에 링크 유지

마지막 갱신: 2026-07-26
