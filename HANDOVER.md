# F1 Race Manager Handover

이 문서는 현재 `/Users/kimyongjin/Desktop/f1` 프로젝트를 다음 작업자가 바로 이어받을 수 있도록 정리한 인수인계 문서입니다.

## 현재 상태

F1 매니지먼트 게임 프로토타입은 현재 실행 가능한 수직 슬라이스까지 구현되어 있습니다.

- Backend: FastAPI + WebSocket + 인메모리 레이스 세션
- Frontend: React + Vite + Pixi.js 트랙 뷰
- 데이터: 20명 드라이버, 10개 팀, 가상 서킷 3개
- 레이스 조작: 시작 타이어, 랩 수, 피트콜, 페이스 모드, 배속, 일시정지
- 실시간 표시: 라이브 타이밍, GAP/INT 전환 컬럼, 디테일 트랙 뷰, 피트 타이머, DRS/AIR 배지, 최종 결과 모달

현재 `127.0.0.1:8000`에서 서버가 실행 중이며, 마지막 확인 시 active session은 없습니다.

## 실행 방법

Backend:

```bash
cd /Users/kimyongjin/Desktop/f1/backend
.venv/bin/uvicorn main:app --host 127.0.0.1 --port 8000 --reload
```

Frontend는 production build가 이미 `frontend/dist`에 생성되어 있고, FastAPI가 이를 정적 서빙합니다. 개발 서버를 별도로 띄우려면:

```bash
cd /Users/kimyongjin/Desktop/f1/frontend
npm run dev
```

검증 명령:

```bash
cd /Users/kimyongjin/Desktop/f1/backend
.venv/bin/python -m unittest discover -s tests -v
```

```bash
cd /Users/kimyongjin/Desktop/f1/frontend
npm run lint
npm run build
```

마지막 확인 기준:

- Backend unit tests: 59 tests OK
- Frontend lint: OK
- Frontend build: OK
- Vite build 경고: Pixi 관련 chunk size 경고만 있음

## 주요 파일 구조

```text
f1/
├── backend/
│   ├── main.py
│   ├── session.py
│   ├── data_loader.py
│   ├── models/
│   │   └── schemas.py
│   ├── simulation/
│   │   ├── race_engine.py
│   │   ├── physics.py
│   │   ├── tire_model.py
│   │   ├── ai_strategy.py
│   │   ├── pit_stop.py
│   │   └── events.py
│   ├── data/
│   │   ├── circuits.json
│   │   ├── drivers.json
│   │   └── teams.json
│   └── tests/
│       └── test_engine.py
├── frontend/
│   ├── src/
│   │   ├── App.jsx
│   │   ├── hooks/useRaceWebSocket.js
│   │   └── components/
│   │       ├── RaceSetup.jsx
│   │       ├── Dashboard/
│   │       │   ├── TimingBoard.jsx
│   │       │   └── StrategyPanel.jsx
│   │       ├── TrackView/TrackCanvas.jsx
│   │       └── Controls/
│   │           ├── SpeedControl.jsx
│   │           └── EventFeed.jsx
│   └── dist/
├── HANDOVER.md
├── NEXT_AGENT_BRIEF.md
└── ROADMAP_NEXT_STEPS.md
```

## Backend 구현 내용

### API

`backend/main.py`

- `GET /api/health`
- `GET /api/drivers`
- `GET /api/teams`
- `GET /api/circuits`
- `POST /api/race/setup`
- `DELETE /api/race/session`
- `WS /ws/race`
- `frontend/dist`가 있으면 `/`와 `/assets`를 정적 서빙

`DELETE /api/race/session`은 나가기 버튼이나 테스트 후 세션 정리에 사용됩니다.

### 세션 루프

`backend/session.py`

- 단일 활성 레이스 세션만 유지합니다.
- broadcast interval은 `0.2s`.
- 내부 시뮬레이션은 `GAME_TICK_SECONDS = 0.1` substep으로 진행합니다.
- WebSocket 연결 시 `race_info`와 현재 `tick`을 즉시 전송합니다.
- 레이스 종료 시 모든 non-retired 드라이버가 finish해야 `race_end`를 broadcast합니다.

지원 명령:

```json
{"type": "pit_call", "driver_id": 1, "tire_choice": "HARD"}
{"type": "set_pace_mode", "driver_id": 1, "pace_mode": "ATTACK"}
{"type": "set_speed", "multiplier": 5}
{"type": "pause"}
{"type": "resume"}
```

### 서킷

`backend/data/circuits.json`

- 현재 서킷은 가상 서킷 3개입니다.
- `F1 Test Oval Circuit`: 직선, 커브, 직선, 커브 형태의 테스트 오벌입니다. 한쪽 직선에 pit lane이 있습니다.
- `Technical Park Circuit`: 비대칭 테크니컬 레이아웃입니다. 초반 스윕, 헤어핀/백스트레이트, 후반 스타디움 복합 코너로 구성되어 있습니다.
- `Bahrain Inspired Circuit`: 바레인 GP 레이아웃을 참고한 가상화 서킷입니다. 실제 Grand Prix layout 흐름에 맞춰 하단 메인 스트레이트, 좌측 T1-4, 중앙 T5-10, 우측 T11-15 루프, DRS zone 4개를 포함하도록 재설계되어 있습니다.
- 모든 서킷은 pit lane, DRS zone, start/finish, landmark, driving segment 데이터를 포함합니다.
- 모든 서킷은 `layout_segments` / `pit_lane_segments` 곡선 원본 데이터를 포함합니다.
- `backend/simulation/track_compiler.py`가 `straight`/`bezier` 원본 segment를 일정 거리 간격의 `track_coords` / `pit_lane_coords`로 컴파일합니다.
- landmark는 `progress` 값을 가질 수 있고, 로딩 시 컴파일된 track index로 다시 매핑됩니다.
- `segments`는 `straight`, `heavy_braking`, `technical`, `traction`, `sweeping` 타입을 사용하며, 다음 드라이빙 모델 고도화의 기반입니다.
- `backend/simulation/track_geometry.py`는 서킷 geometry validation과 `segment_at_progress()` lookup을 제공합니다.
- DRS zone은 driving `straight` segment 내부에만 있어야 하며, validation/test로 확인합니다.
- 기본 랩 수는 30랩이지만, setup 화면에서 5-70랩 사이로 덮어쓸 수 있습니다.

### 드라이빙 segment modifier

`backend/simulation/race_engine.py`

- 현재 차량 progress에 따라 `segment_at_progress()`로 driving segment를 찾습니다.
- `straight`, `sweeping`, `heavy_braking`, `technical`, `traction`별로 작은 lap-time delta를 적용합니다.
- 직선은 차량 성능/드라이버 pace 영향이 조금 더 큽니다.
- 급제동은 overtaking, consistency, tire wear 영향을 받습니다.
- 테크니컬/트랙션 구간은 pace, consistency, tire wear, tire management 영향을 더 받습니다.
- 이 단계는 큰 밸런스 변경이 아니라, 이후 추월/실수 모델을 얹기 위한 1차 프로토타입입니다.

### 랩 수 선택

`RaceSetupRequest.total_laps`

- default: 30
- min: 5
- max: 70
- `SessionManager.create_session()`에서 원본 circuit을 `model_copy(update={"total_laps": request.total_laps})`로 복사해 사용합니다.
- 원본 JSON 데이터는 변형하지 않습니다.

### 레이스 엔진

`backend/simulation/race_engine.py`

핵심 상태:

- `DriverRaceState.progress`: 현재 랩 내 위치, 0.0-1.0
- `current_lap`: 완료한 랩 수
- `total_progress`: 완료 랩 + 현재 progress
- `total_time`: 누적 레이스 시간
- `tire_age`: 표시용 완료 랩 기준 타이어 랩 수
- `tire_usage`: 실시간 누적 타이어 사용량
- `tire_wear`: 0.0 fresh, 1.0 fully worn
- `pace_mode`: `CONSERVE`, `STANDARD`, `ATTACK`
- `dirty_air_active`: 1초 이내 추격/배틀 상태에서 true
- `drs_active`: 1초 이내 추격 상태이고 현재 progress가 DRS zone 안이면 true

포지션:

- active driver: `(-total_progress, total_time)`
- finished driver: finish order와 total time
- retired driver: 뒤로
- P1 gap은 항상 `LEADER`
- finished driver gap은 `FIN`

레이스 종료:

- 리더만 finish했다고 끝나지 않습니다.
- 모든 non-retired 드라이버가 finish해야 `engine.finished = True`.
- 최종 결과는 `build_results()`로 반환됩니다.

### 성능 압축

`backend/simulation/physics.py`

차량/드라이버 성능 격차가 너무 커서 배틀이 적게 발생하던 문제를 줄였습니다.

현재 매핑:

```python
def car_performance_multiplier(car_performance: float) -> float:
    return 1.0 + (car_performance - 1.0) * 0.50

def driver_pace_multiplier(pace_stat: float) -> float:
    return 1.0 + (pace_stat - 0.88) * 0.28
```

새 미디엄 기준 이론 랩타임 범위:

- 변경 전: VER 60.935s, SAR 76.247s, range 15.311s
- 변경 후: VER 61.956s, SAR 67.011s, range 5.055s

### 타이어 모델

`backend/simulation/tire_model.py`

현재 컴파운드 스펙:

```python
SOFT   grip=1.005, degradation_rate=0.022, cliff_threshold=16
MEDIUM grip=0.990, degradation_rate=0.014, cliff_threshold=31
HARD   grip=0.970, degradation_rate=0.010, cliff_threshold=44
```

특징:

- wear는 비선형 증가: `age_ratio ** 1.35`
- 성능 손실도 비선형 곡선
- 기존 `0.85` 최저 성능 제한은 제거됨
- tire performance variation은 마모도에 따라 매 랩 랜덤으로 반영
- 0% 근처 dead tire penalty는 변동성이 아니라 기준 랩타임 하락폭으로 반영

현재 스틴트 창:

- 8-12랩: Soft 우위
- 15-28랩: Medium 우위
- 30랩 이상: Hard 우위

### 타이어 매니지먼트

드라이버 `stats.tire_management`가 실제 마모에 반영됩니다.

- 좋은 타이어 매니지먼트는 `tire_usage`가 모델상 덜 낡은 타이어처럼 계산되게 합니다.
- 나쁜 타이어 매니지먼트는 같은 사용량에서도 더 낡은 타이어처럼 계산됩니다.
- AI 피트 판단도 같은 managed tire age 기준을 사용합니다.

### 페이스 모드

`PaceMode`

```python
CONSERVE: lap_time_delta +0.55s, tire_usage_multiplier 0.82
STANDARD: lap_time_delta +0.00s, tire_usage_multiplier 1.00
ATTACK:   lap_time_delta -0.45s, tire_usage_multiplier 1.28
```

플레이어 팀 드라이버는 사용자가 변경합니다.
AI 드라이버는 엔진이 자동으로 변경합니다.

AI pace mode 판단:

- 판단 주기: 8-12초 cooldown
- 앞차와 1초 이내이고 타이어가 충분하면 `ATTACK`
- 뒤차와 0.85초 이내이고 방어 스탯/타이어가 충분하면 `ATTACK`, 아니면 `STANDARD`
- 타이어 수명 25% 이하이고 레이스 후반이 아니면 `CONSERVE`
- 타이어 수명 12% 이하는 `CONSERVE`
- 피트 요청이 잡힌 in-lap 또는 피트 직후 fresh tire 구간은 `ATTACK`
- 레이스 막판에는 타이어가 충분하면 `ATTACK`, 부족하면 `STANDARD`

### AI 전략

`backend/simulation/ai_strategy.py`

피트 판단:

- player driver는 AI가 피트하지 않음
- retired/in_pit/pending pit request면 skip
- `wear >= 0.7` 또는 다음 랩에 cliff 도달 예정이면 pit
- 긴 레이스에서 “현재 타이어로 남은 레이스 전체를 못 가면 즉시 pit”하던 조건은 제거됨

타이어 선택:

```python
remaining_laps <= 12 -> SOFT
remaining_laps <= 28 -> MEDIUM
else                 -> HARD
```

### 배틀, DRS, 더티 에어 1차 모델

현재 배틀 모델은 세 층입니다.

- 근접 주행 랩타임 보정
- 이벤트 피드용 `attack`, `defend`, `side_by_side`, `forced_wide`, `lockup`, `pass` 이벤트
- battle 결과에 따른 단기 lap-time effect

조건:

- 바로 앞 position의 차량과 gap이 1.0초 이내이면 battle 상태
- 추격 차량 `dirty_air_active = True`
- 추격 차량이 circuit `drs_zones` 안에 있으면 `drs_active = True`
- `attack`/`defend` 이벤트는 `straight` 또는 `heavy_braking` segment에서만 낮은 확률로 발생
- `side_by_side`와 `forced_wide`는 heavy braking segment의 attack/defense score 차이로 발생
- `side_by_side`는 순간 이벤트 후 약 6초 동안 지속 상태로 유지되며, 두 차량 모두 `SBS` 상태가 표시됨
- `side_by_side` 발생 시 공격자는 `inside`/`outside`, 방어자는 `racing_line`/`defensive_line`을 선택함
- `inside`는 코너 진입에서 유리하지만 탈출에서 손해, `outside`는 진입에서 손해지만 탈출에서 이득
- `defensive_line`은 방어에는 유리한 선택이지만 본인도 코너 중/탈출 페이스 손실을 받음
- side-by-side 종료 시 낮은 확률로 `run_wide`, `traction_loss`, `minor_contact` 결과 이벤트가 발생함
- `forced_wide` 발생 후 짧은 후속 상태가 예약되며, 코너 탈출에서 방어자 `run_wide`/`traction_loss`, 공격자 tight exit `traction_loss`, 또는 `minor_contact`가 발생할 수 있음
- `lockup`은 heavy braking segment에서 ATTACK mode, tire wear, 낮은 consistency, 근접도에 따라 발생
- 드라이버별 battle event cooldown으로 이벤트 스팸을 방지
- `pass` 이벤트는 실제 `total_progress` 정렬 결과로 position이 개선됐을 때 발생

보정 개념:

```text
battle_delta = dirty_air_penalty - traffic_attack_bonus - drs_bonus + defense_block
```

반영 요소:

- 추격 차량 `overtaking`
- 추격 차량 `pace_mode`
- 추격 차량 tire life
- 앞차 `defending`
- circuit `overtaking_difficulty`

상수:

```python
TRAFFIC_GAP_SECONDS = 1.0
DIRTY_AIR_MAX_PENALTY = 0.32
TRAFFIC_ATTACK_MAX_BONUS = 0.12
DRS_MAX_BONUS = 0.30
DEFENSE_MAX_BLOCK = 0.22
BATTLE_EVENT_GAP_SECONDS = 0.80
BATTLE_EVENT_COOLDOWN_SECONDS = 14.0
BATTLE_ATTACK_EFFECT_SECONDS = 4.5
BATTLE_DEFEND_EFFECT_SECONDS = 3.5
BATTLE_LOCKUP_EFFECT_SECONDS = 4.0
BATTLE_ATTACKER_LAP_TIME_DELTA = -3.2
BATTLE_DEFENDER_PRESSURE_LAP_TIME_DELTA = 1.15
BATTLE_DEFEND_ATTACKER_LAP_TIME_DELTA = 2.35
BATTLE_LOCKUP_LAP_TIME_DELTA = 5.5
BATTLE_SIDE_BY_SIDE_SCORE_MARGIN = 0.06
BATTLE_FORCED_WIDE_SCORE_MARGIN = 0.18
BATTLE_SIDE_BY_SIDE_EFFECT_SECONDS = 6.0
BATTLE_SIDE_BY_SIDE_BREAK_GAP_SECONDS = 1.25
BATTLE_CORNER_EXIT_EFFECT_SECONDS = 2.5
BATTLE_INSIDE_ENTRY_LAP_TIME_DELTA = -0.15
BATTLE_OUTSIDE_ENTRY_LAP_TIME_DELTA = 0.12
BATTLE_INSIDE_EXIT_LAP_TIME_DELTA = 0.75
BATTLE_OUTSIDE_EXIT_LAP_TIME_DELTA = -0.55
BATTLE_DEFENSIVE_LINE_LAP_TIME_DELTA = 0.18
BATTLE_DEFENSIVE_LINE_EXIT_LAP_TIME_DELTA = 0.35
BATTLE_TRACTION_LOSS_LAP_TIME_DELTA = 1.4
BATTLE_RUN_WIDE_LAP_TIME_DELTA = 2.6
BATTLE_MINOR_CONTACT_ATTACKER_LAP_TIME_DELTA = 3.6
BATTLE_MINOR_CONTACT_DEFENDER_LAP_TIME_DELTA = 2.6
BATTLE_FORCED_WIDE_EXIT_DELAY_SECONDS = 1.8
BATTLE_FORCED_WIDE_RESULT_BASE_PROBABILITY = 0.42
BATTLE_FORCED_WIDE_ATTACKER_EXIT_LAP_TIME_DELTA = 1.25
```

결과 효과:

- `attack`: 공격자에게 짧은 lap-time bonus, 앞차에게 압박 손실
- `defend`: 추격 차량에게 짧은 lap-time penalty
- `side_by_side`: 두 차량 모두 약 6초 동안 lap-time/tire usage penalty, 선택 라인에 따라 진입/탈출 효과 차등
- `run_wide`: side-by-side 종료 시 공격자가 코너 밖으로 밀리며 시간 손실
- `traction_loss`: side-by-side 종료 시 공격자가 탈출 가속을 잃으며 시간 손실
- `minor_contact`: side-by-side 종료 시 두 차량 모두 접촉에 따른 시간/타이어 손실
- `forced_wide`: 공격자에게 짧은 bonus, 방어 차량에게 큰 lap-time penalty, 이후 코너 탈출 후속 결과 가능
- `lockup`: 추격 차량에게 더 큰 lap-time penalty와 tire usage penalty
- `pass`: 단기 효과가 아니라 실제 position 개선 결과를 보고 표시

가상 오벌 DRS 구간:

- Main Straight: progress `0.00-0.14`
- Back Straight: progress `0.43-0.60`

## Frontend 구현 내용

### Setup 화면

`frontend/src/components/RaceSetup.jsx`

- circuit 선택
- player team 선택
- 랩 수 선택: 5-70
- 플레이어 팀 드라이버별 starting tire 선택: Soft/Medium/Hard
- start 시 `/api/race/setup` 호출

### Start lights

`frontend/src/App.jsx`

- setup 완료 후 즉시 WebSocket 연결하지 않고 start lights overlay 실행
- 5개 red lights 점등 후 `LIGHTS OUT`
- 이후 WebSocket 연결 시작

### Race 화면

구성:

- 좌측: `TimingBoard`
- 중앙: `TrackCanvas`, `EventFeed`
- 우측: `SpeedControl`, `StrategyPanel`

### EventFeed

`frontend/src/components/Controls/EventFeed.jsx`

- 이벤트 메시지는 `EN`/`KO` 토글로 전환 가능
- 백엔드는 기존 영어 `message`를 유지하고 한국어 `message_ko`를 추가로 전송
- `message_ko`가 없는 이벤트는 영어 `message`로 fallback
- command ack/error 이벤트도 `message_ko`를 보존

### TimingBoard

`frontend/src/components/Dashboard/TimingBoard.jsx`

표시:

- position
- driver abbreviation
- status column: `PIT`, `SBS`, `DRS`, `AIR`
- GAP/INT 전환 컬럼: 헤더 클릭으로 leader gap과 앞차 interval을 전환
- last lap
- tire compound + tire life %
- pit count 또는 pit remaining
- finished/retired 상태

랩타임은 millisecond precision으로 표시합니다.

### StrategyPanel

`frontend/src/components/Dashboard/StrategyPanel.jsx`

플레이어 팀 드라이버용:

- tire compound
- tire age
- tire life %
- tire wear bar
- pace mode buttons: `SAVE`, `STD`, `ATK`
- tire select
- `BOX BOX`

### TrackCanvas

`frontend/src/components/TrackView/TrackCanvas.jsx`

- Pixi.js 기반 2D 트랙 렌더링
- 트랙 외곽선, 두꺼운 아스팔트 레이어, 중앙 가이드 라인
- 코너 구간 kerb 표시
- DRS zone 하이라이트와 DRS 라벨
- start/finish 라인 강조
- pit entry / pit exit 라벨
- pit box 10칸 표시
- car marker는 팀 컬러 방향성 마커 + label
- label은 `position abbreviation`
- pit lane 표시
- pit 중인 차량은 pit lane 쪽에 표시
- pit remaining 표시
- 차량 위치, DRS zone, kerb, pit slot은 point index가 아니라 실제 polyline 길이 기반 progress로 보간합니다.

### Race end

`frontend/src/App.jsx`

- 모든 non-retired 드라이버 finish 후 race_end 수신
- 최종 순위 모달 표시
- winner total time과 gap/DNF 표시

## 테스트 커버리지

`backend/tests/test_engine.py`

현재 포함된 주요 테스트:

- setup lap count 5-70 validation
- session lap count copy, original circuit data not mutated
- curve layout segment compile
- seed circuits compile from `layout_segments`
- landmark progress resolves to compiled track index
- DRS zones stay inside straight segments
- tire wear/performance curve
- tire model distinct stint windows
- AI tire choice windows
- AI pace attacks when close to car ahead
- AI pace conserves on critical tires
- AI pace attacks on fresh tires after stop
- AI pace cooldown keeps current mode
- AI pace does not change player drivers
- AI fresh tire does not pit immediately in long race
- AI pits when tires are worn
- compressed performance mapping
- P1 is always LEADER
- gaps/intervals non-negative
- live gaps update after first lap
- lap finish interpolation
- pit resets tire age and tire usage
- pause stops progress
- speed multiplier affects progress
- starting tires applied
- pace mode commands
- attack faster but uses more tire
- battle delta rewards attack mode
- battle event probability only applies to overtaking segments
- forced battle event creates attack
- defended battle adds time cost to attacker
- close heavy braking scores create side-by-side event
- side-by-side event includes Korean message
- side-by-side state blocks repeat event and expires
- inside line gains entry but loses corner exit
- outside line costs entry but gains corner exit
- defensive line slows defender and corner exit
- side-by-side can end with run wide / traction loss / minor contact
- strong heavy braking attack can force defender wide
- forced-wide aftermath can make defender run wide / lose traction
- forced-wide aftermath can cost attacker on tight exit
- forced-wide aftermath can end with minor contact
- heavy braking attack can create lockup penalty
- pass event reports position gain
- DRS activates within 1 second and DRS zone
- dirty air can be active without DRS outside DRS zone
- race waits for all drivers after leader finishes
- final lap does not loop forever

## 최근 검증 결과

마지막 전체 검증:

```bash
cd backend
.venv/bin/python -m unittest discover -s tests -v
```

결과: 76 tests OK

```bash
cd frontend
npm run lint
npm run build
```

결과: OK

`npm run build`는 Pixi bundle 때문에 500 kB chunk warning이 나오지만 빌드는 성공합니다.

## 알려진 한계와 다음 개선 제안

1. 추월/방어 모델 고도화
   - 현재 `attack`/`defend`/`forced_wide`/`lockup`은 단기 lap-time effect까지 반영합니다.
   - `side_by_side`는 짧은 지속 상태, 라인 선택, 코너 탈출 효과, 결과 이벤트, `SBS` 상태 표시까지 반영합니다.
   - `forced_wide`는 delayed corner-exit aftermath까지 반영합니다.
   - position 강제 교환은 하지 않고, 실제 `total_progress` 정렬로 추월이 발생해야 `pass`가 뜹니다.
   - 다음 단계에서는 접촉 누적 리스크, 명시적 추월 성공 확률 보정을 추가할 수 있습니다.

2. 전략 고도화
   - 현재 AI는 마모/클리프 기준으로만 pit합니다.
   - undercut/overcut, traffic, pit window, safety car를 추가하면 좋습니다.

3. DRS zone 고도화
   - 현재는 progress range 기반이지만, straight segment 내부에만 있도록 검증합니다.
   - 추후 트랙 섹터/직선 길이 기반 또는 circuit editor 방식으로 확장할 수 있습니다.

4. 안전차/랜덤 이벤트
   - `events.py` 구조는 있지만 현재 `roll_events(..., enable_events=False)`로 사실상 꺼져 있습니다.
   - 이벤트 시스템을 켜려면 race finish/gap/pit flow에 영향이 없는지 테스트를 먼저 추가해야 합니다.

6. 저장과 캠페인 구조
   - 현재는 단일 인메모리 세션입니다.
   - 장기적으로 시즌/세이브/팀 성장 요소를 추가하려면 DB가 필요합니다.

7. 저장소 정리
   - 현재 `backend/.git`은 일반 git repo로 인식되지 않는 상태로 보입니다.
   - 루트 저장소 정책, `.gitignore`, generated files(`dist`, `.venv`, `__pycache__`) 정리가 필요합니다.

## 현재 서버 상태

사용자 요청에 따라 서버를 종료했습니다.

확인 명령:

```bash
lsof -nP -iTCP:8000 -sTCP:LISTEN
```

결과: 리스닝 프로세스 없음
