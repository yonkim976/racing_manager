# Agent Handoff - F1 Race Manager

> **빠른 시작**: `NEXT_AGENT_BRIEF.md`를 먼저 읽으십시오. 이 문서는 상세 레퍼런스입니다.

이 문서는 다음 에이전트가 `/Users/kimyongjin/Desktop/f1` 프로젝트를 바로 이어서 작업할 수 있도록 정리한 최신 인수인계 문서입니다.

작성 기준: 2026-06-26 (최종 갱신)

## 1. 현재 프로젝트 상태

F1 매니지먼트/레이스 시뮬레이션 프로토타입입니다. 현재는 레이스 셋업, 퀄리파잉, 실시간 레이스 진행, 피트/타이어/페이스 관리, 트랙 렌더링, 이벤트 피드까지 실행 가능한 수직 슬라이스가 구현되어 있습니다.

- Backend: FastAPI + WebSocket + 인메모리 단일 레이스 세션
- Frontend: React + Vite + Pixi.js 트랙 뷰
- 데이터: 20명 드라이버, 10개 팀, 4개 서킷
- 최근 검증:
  - Backend unit tests: **106 OK**
  - Frontend lint: OK
  - Frontend build: OK
  - Vite build의 Pixi chunk size warning은 기존 경고

## 2. 실행 및 검증

Backend 서버:

```bash
cd /Users/kimyongjin/Desktop/f1/backend
.venv/bin/uvicorn main:app --host 127.0.0.1 --port 8000 --reload
```

Backend 테스트:

```bash
cd /Users/kimyongjin/Desktop/f1/backend
.venv/bin/python -m unittest discover -s tests
```

Frontend 개발 서버가 필요할 때:

```bash
cd /Users/kimyongjin/Desktop/f1/frontend
npm run dev
```

Frontend 검증:

```bash
cd /Users/kimyongjin/Desktop/f1/frontend
npm run lint
npm run build
```

세션 정리:

```bash
curl --max-time 3 -s -X DELETE http://127.0.0.1:8000/api/race/session
```

## 3. 주요 파일

```text
backend/
  main.py                         API 엔트리포인트
  session.py                      WebSocket 세션/게임 루프
  data_loader.py                  JSON 로딩 및 서킷 컴파일
  models/schemas.py               API/시뮬레이션 Pydantic 모델
  data/circuits.json              서킷, DRS, pit lane, segment 데이터
  data/drivers.json               드라이버/스탯 데이터
  data/teams.json                 팀/차량 성능 데이터
  simulation/race_engine.py        레이스 진행 핵심 엔진 (그리드, SC/VSC, 피트 3단계, 번칭업)
  simulation/qualifying.py         Q1/Q2/Q3 녹아웃 예선 계산
  simulation/incidents.py          사고 분류(원인×심각도)와 발생 확률
  simulation/physics.py            랩타임/진행률/성능 압축
  simulation/tire_model.py         타이어 성능/마모/수명
  simulation/ai_strategy.py        AI 피트 전략
  simulation/pit_stop.py           피트 시간 분리(레인 주행 + 타이어 교체)
  simulation/track_compiler.py     straight/bezier 원본 트랙 컴파일
  simulation/track_geometry.py     서킷 검증 및 segment lookup
  tests/test_engine.py             엔진/데이터 핵심 테스트
  tests/test_api.py                API validation 테스트

frontend/src/
  App.jsx                          앱 상태, race setup/race view 전환
  components/RaceSetup.jsx         셋업, 퀄리파잉 실행, 결과표, 레이스 시작
  components/RaceSetup.css         셋업/퀄리파잉 레이아웃
  components/TrackView/TrackCanvas.jsx
                                   Pixi 트랙, 차량, DRS, pit, label 렌더링
  components/Dashboard/TimingBoard.jsx
                                   라이브 타이밍
  components/Dashboard/StrategyPanel.jsx
                                   피트콜/페이스/타이어 조작
```

## 4. API 요약

`backend/main.py`

- `GET /api/health`
- `GET /api/drivers`
- `GET /api/teams`
- `GET /api/circuits`
- `POST /api/circuits/validate`
- `POST /api/qualifying/run`
- `POST /api/race/setup`
- `DELETE /api/race/session`
- `WS /ws/race`
- `frontend/dist`가 있으면 `/`와 `/assets`를 정적 서빙

### 퀄리파잉 API

`POST /api/qualifying/run`

요청:

```json
{
  "circuit_id": 3,
  "player_team_id": 1,
  "attempt_laps": 3
}
```

`attempt_laps`는 각 세션(Q1/Q2/Q3)당 어택 랩 수입니다.

응답:

- `results`: position, driver_id, name, full_name, team, team_color, best_lap_time, gap, laps(마지막 참가 세션 어택랩), `knockout`(Q1/Q2/Q3), `q1_time`, `q2_time`, `q3_time`
- `grid_order`: 예선 결과 기반 driver_id 배열

현재 예선은 실제 F1식 Q1/Q2/Q3 녹아웃입니다.

- `SESSION_PLAN`: Q1(15명 진출) → Q2(10명 진출) → Q3(폴 결정). 드라이버 수가 진출 정원 이하이면 그 세션이 최종으로 처리됩니다.
- 각 세션마다 모든 참가자가 새 SOFT 기준 `attempt_laps`회 어택 랩을 수행하고 세션 베스트 랩으로 정렬됩니다.
- track evolution: `Q1=1.0, Q2=0.997, Q3=0.994`로 세션이 진행될수록 base lap time이 소폭 단축됩니다.
- 최종 그리드: Q3 순위(P1~10) + Q2 탈락(P11~15) + Q1 탈락(P16~20). 각 블록 내부는 해당 세션 베스트 랩 순.
- 각 드라이버 표시 베스트랩/gap은 자신이 마지막으로 참가한 세션 기준입니다.
- 차량 성능, 드라이버 pace, consistency, 소량 랜덤이 반영됩니다.

### 레이스 셋업 API

`RaceSetupRequest`는 현재 다음 필드를 받습니다.

```json
{
  "circuit_id": 3,
  "player_team_id": 1,
  "total_laps": 57,
  "starting_tires": {
    "1": "MEDIUM",
    "2": "HARD"
  },
  "grid_order": [1, 3, 5, 6]
}
```

- `total_laps`: 5-100
- `starting_tires`: 현재 플레이어 팀 드라이버에게만 적용
- `grid_order`: 퀄리파잉 결과가 있으면 RaceEngine이 이 순서를 우선 적용. 누락된 드라이버는 기존 성능 기반 fallback 순서로 뒤에 채움.

## 5. 서킷/트랙 시스템

현재 서킷:

1. `F1 Test Oval Circuit`
   - 기본 30랩
   - DRS 2개
   - driving segment 5개
2. `Technical Park Circuit`
   - 기본 42랩
   - DRS 2개
   - driving segment 6개
3. `Bahrain Inspired Circuit`
   - 기본 57랩
   - DRS 4개
   - driving segment 10개
   - 최근 DRS 조정:
     - `Main Straight`: `0.010-0.130`
     - `T3-T4 Climb`: `0.222-0.292`
     - `Back Straight`: `0.590-0.662`
     - `T13-T14 Straight`: `0.840-0.908`
4. `Red Bull Ring Inspired Circuit`
   - 기본 71랩
   - DRS 3개
   - driving segment 12개

서킷 데이터는 `backend/data/circuits.json`에 있습니다.

주요 구조:

- `layout_segments`: 원본 중심선. `straight`, `bezier` 지원.
- `pit_lane_segments`: pit lane 원본 선.
- `track_coords`: 컴파일된 polyline. API 응답에 포함.
- `pit_lane_coords`, `pit_wall_coords`: 컴파일/계산된 pit 관련 좌표.
- `drs_zones`: normalized progress range. validation상 `straight` segment 안에 있어야 함.
- `landmarks`: T1, T2, S/F, PIT 등 표시용. `progress` 기반으로 track index가 재매핑됨.
- `segments`: driving model용 구간. `straight`, `heavy_braking`, `technical`, `traction`, `sweeping`.

`TrackCanvas.jsx`는 차량/DRS/kerb/pit lane 위치를 point index가 아니라 실제 polyline 길이 기반으로 보간합니다.

서킷 기하 검증(2026-06-26 확인):

- 4개 seed circuit 모두 `validate_circuit_geometry()` 통과 (자기교차 없음, segments 0→1 연속, DRS가 straight 내부)
- Bahrain/Red Bull Ring 비인접 구간 최소 간격 37~46px (트랙 폭 대비 충분)
- 임시 시각화 도구(`backend/tools/plot_tracks.py`)는 검증 후 삭제됨

## 6. 레이스/드라이빙 모델

핵심 진행 구조:

- 각 차량은 `progress`와 `total_progress`로 움직입니다.
- 매 tick마다 현재 추정 lap time을 계산하고 `compute_progress_delta()`로 전진량을 계산합니다.
- 실제 순간속도/물리 모델이 아니라 랩타임 기반 진행률 모델입니다.

랩타임 영향 요소:

- circuit base lap time
- team car performance
- driver pace
- tire compound/performance/wear
- tire management
- driver consistency
- pace mode
- DRS/dirty air
- battle effect
- track segment modifier

성능 압축:

```python
car_performance_multiplier = 1.0 + (car_performance - 1.0) * 0.50
driver_pace_multiplier = 1.0 + (pace_stat - 0.88) * 0.28
```

Segment modifier:

- `straight`: 차량 성능 영향이 큼
- `sweeping`: pace/차량 성능/타이어 마모 영향
- `heavy_braking`: overtaking, pace, tire wear, consistency 영향
- `technical`: pace, tire wear, consistency 영향
- `traction`: pace, tire wear, tire management 영향

## 7. 타이어/페이스/전략

타이어 모델:

```python
SOFT   grip=1.005, degradation_rate=0.022, cliff_threshold=16
MEDIUM grip=0.990, degradation_rate=0.014, cliff_threshold=31
HARD   grip=0.970, degradation_rate=0.010, cliff_threshold=44
```

- 타이어 마모는 실시간 `tire_usage` 기반.
- 타이어 성능 저하는 비선형.
- 0% 근처에서는 기준치에서 하락 갭이 커지도록 설계.
- `tire_management`는 effective tire age에 반영됩니다.

페이스 모드:

```python
CONSERVE: +0.55s/lap, tire usage x0.82
STANDARD: +0.00s/lap, tire usage x1.00
ATTACK:   -0.45s/lap, tire usage x1.28
```

AI:

- AI는 pace mode를 자동 선택합니다.
- 앞차와 1초 안쪽이면 더 공격적으로 갈 수 있음.
- 뒤차가 가까우면 STANDARD/ATTACK.
- 타이어 수명 낮으면 CONSERVE.
- 피트 직전/직후, 레이스 막판은 더 공격적.
- 너무 자주 바뀌지 않도록 cooldown이 있음.

AI 피트:

- 지나치게 긴 레이스에서 매랩 피트하던 조건은 제거됨.
- 현재는 마모/클리프 기반으로 피트 판단.
- SC 발동 시 `_ai_sc_pit_decisions`로 공짜 피트(마모≥0.30, prob 0.4) — SC pit window와 연동됨.
- 남은 항목: undercut/overcut, traffic-aware pit timing, entry/exit progress 연동.

피트 스톱 진행(3단계, 순간이동 제거):

- 랩 완료 시 피트가 시작되면 `in`(핏레인 진입) → `stop`(박스 정지/타이어 교체) → `out`(핏레인 진출) 3단계를 거칩니다(`_tick_in_pit`).
- 시간 분리(`compute_pit_components`): `pit_loss_time`을 레인 주행(진입·진출 각 절반)으로, 타이어 교체 시간(`2.0 + rng/skill`)을 정지 단계로 둡니다. 총 손실 = `pit_loss_time + tire_change`로 보존.
- 타이어 교체는 `stop` 종료(=`out` 시작) 시점에 적용, `pit_count`는 `out` 종료(트랙 복귀) 시 +1.
- 위치 보고: `pit_lane_progress(0=entry, 1=exit)`를 계산해 프론트가 차를 핏레인 경로(`pit_lane_coords`)를 따라 그립니다(`PIT_BOX_LANE_FRACTION=0.5`가 박스 위치).
- 시간 표시는 **증가형**: `pit_elapsed`(전체 경과), `pit_stop_elapsed`(정지 경과)를 0부터 누적해 전달(기존 카운트다운 `pit_remaining` 제거).
- `DriverPositionInfo`: `pit_phase`, `pit_lane_progress`, `pit_elapsed`, `pit_stop_elapsed` 추가. 프론트 라벨은 `PIT IN` / `BOX 2.4s` / `PIT OUT`, 타이밍보드는 `pit_elapsed` 표시.

## 8. 배틀/추월/이벤트

현재 구현된 요소:

- dirty air
- DRS
- attack/defend 이벤트
- side-by-side 상태
- forced wide
- lockup
- pass event
- run wide
- traction loss
- minor contact
- EventFeed EN/KO 언어 토글

중요 수치:

```python
TRAFFIC_GAP_SECONDS = 1.0
DIRTY_AIR_MAX_PENALTY = 0.32
TRAFFIC_ATTACK_MAX_BONUS = 0.12
DRS_MAX_BONUS = 0.30
DEFENSE_MAX_BLOCK = 0.22
BATTLE_EVENT_GAP_SECONDS = 0.80
BATTLE_EVENT_COOLDOWN_SECONDS = 14.0
```

주의:

- 추월 이벤트는 실제 position 변화와 별개로 “공방 상황”을 표현합니다.
- `pass` 이벤트는 실제 `total_progress` 정렬 결과로 포지션이 바뀔 때 생성됩니다.
- side-by-side는 단기 상태와 lap-time effect가 있지만, 아직 실제 레이싱 라인/차폭 물리 모델은 아닙니다.

## 8b. 사고 분류 / 세이프티카 (SC/VSC)

사고는 `backend/simulation/incidents.py`에서 원인 × 심각도 2축으로 분류합니다.

- 원인(`IncidentCause`): `driver_error`, `collision`, `mechanical`
- 심각도(`IncidentSeverity`): `minor`(시간/타이어 손실), `car_stopped`(리타이어 → VSC), `crash`(1~2대 리타이어 → SC)

발생 모델:

- 사고 확률은 게임초당 rate로 정의되어 simulation speed multiplier와 무관합니다.
- driver error는 consistency↓, 타이어마모↑, `heavy_braking`/`technical` 세그먼트, ATTACK, dirty air에서 증가합니다.
- 고속 구간/고마모일수록 `crash` 비율이 올라갑니다.
- mechanical은 대체로 일정하며 레이스 후반에 약간 증가, 보통 `car_stopped`로 이어집니다.
- 배틀의 `minor_contact`는 `escalate_collision`으로 `car_stopped`/`crash`로 번질 수 있습니다(`_pending_incidents` 큐 → tick에서 flush).

SC/VSC 상태머신(`race_engine`):

- `self.race_phase`: `green` | `vsc` | `sc`. `self.safety_car`는 `sc`와 동기화되는 하위호환 플래그입니다.
- VSC는 `×1.4`(`VSC_LAP_TIME_FACTOR`) 델타로 감속하며 현재는 `VSC_DURATION_SECONDS=25` 시간 기준입니다.
- SC 내부 단계는 `deploying` → `collecting` → `queued` → 선택적 `unlapping` → `in_this_lap` → `restart` → `green`입니다.
- SC는 각 서킷의 `pit_exit_progress`에서 실제로 출동하고 독립 진행도로 트랙을 주행한 뒤, `pit_entry_progress`에서 피트레인으로 들어갑니다.
- 고정 3랩과 즉시 번칭을 제거했습니다. 사고 처리 타이머와 대열 완성 여부가 모두 충족돼야 철수 절차를 시작하며 추가 사고는 처리 시간을 연장합니다.
- 미합류 차량은 앞 대열과의 거리에 따라 `×1.08~×1.25`, SC 대열 차량은 기본 `×1.8`로 주행합니다. 10대 차량 길이(56m) 안에서 합류로 판정하고, 합류 후에는 7대 차량 길이(39.2m) 목표 간격까지 완만하게 압축합니다. 다시 56m 밖으로 벌어지면 자동으로 추격 상태로 복귀합니다.
- 트랙 위 순서는 동결하지만 피트 차량은 실제 `total_progress`로 대열에 삽입되어 SC/VSC 중에도 정상적으로 순위를 잃거나 얻습니다.
- `SAFETY CAR IN THIS LAP` 이후 SC가 피트로 들어가도 즉시 green이 되지 않습니다. 리더가 재시작 가속을 통제하고 시작/결승선을 통과해야 `sc_end`가 발생합니다.
- 이벤트 타입에는 `sc_track_join`, `sc_queue`, `unlap_start`, `unlap`, `sc_in_this_lap`, `sc_pit`, `sc_end`가 포함됩니다.
- `RaceTickState`는 `safety_car_stage`, `safety_car_route`, `safety_car_visible`, queue/restart 상태와 SC 진행도를 프론트에 전달합니다.

명시적 피트 기회(SC 한정):

- SC 발동 시 `pit_window_open=True` + `pit_window` 이벤트로 "공짜 피트" 기회를 알립니다(VSC는 열지 않음).
- 메커니즘: 피트 중 차량은 트랙 진행이 멈추는데 SC 중에는 트랙 차량도 `×1.8`로 느려(+번칭) 상대 손실이 작아 자연히 유리합니다(별도 보정 없음).
- AI는 SC 발동 시 **한 번만**(`_ai_sc_pit_decisions`) 판단: 타이어 마모 `≥ SC_PIT_WEAR_THRESHOLD(0.30)` & 잔여 3랩 초과인 차량이 `SC_PIT_PROBABILITY(0.4)` 확률로 피트(전원 동시 피트 방지).
- 프론트: 헤더 `PIT WINDOW OPEN` 배지 + StrategyPanel 배너 + BOX 버튼 녹색 강조.

백마커 언랩:

- `unlapping` 단계에서 대상 차량이 실제로 추가 한 랩을 주행합니다. 완료 후 기존 순서를 유지한 채 SC 대열 뒤에 합류하며 두 랩 이상 뒤처졌다면 한 랩만 회복합니다.

검증된 발생 빈도(Bahrain 57랩, 20명, 12회 풀 시뮬레이션):

- SC 평균 약 0.42회/레이스, VSC 약 0.92회/레이스, 리타이어 약 1.5대/레이스

아직 하지 않은 것:

- 적색기(red flag), 더블 스태킹/피트 출구 정체 모델, 저시야 20대 길이 간격, Race Director의 `OVERTAKING WILL NOT BE PERMITTED` 선택

## 9. 프론트엔드 상태

### Race Setup

현재 셋업 흐름:

1. 서킷 선택
2. 랩 수 선택: 5-100
3. 팀 선택
4. 시작 타이어 선택
5. `RUN QUALIFYING`
6. 예선 결과표 표시 (Pos / Driver / Q1 / Q2 / Q3, Q2·Q1 탈락 구간 색/구분선 표시)
7. `START RACE`

최근 수정:

- 퀄리파잉 결과표가 뜨면 화면이 잘리던 문제 해결.
- `RaceSetup.css`에서 셋업 화면/패널을 스크롤 가능하게 변경.
- 결과표 높이를 제한.
- `START RACE` 버튼을 패널 하단 sticky로 유지.
- 브라우저 검증:
  - `1280x720`: 결과표 후 버튼 보임
  - `390x720`: 결과표 후 버튼 보임

### Track View

구현된 표시:

- 트랙 폭/외곽선
- kerb
- start/finish line
- DRS zone 하이라이트
- pit entry/exit/pit boxes/pit wall
- 차량 방향성 marker
- 차량 label
- pit 중 차량은 `pit_lane_coords` 경로를 `pit_lane_progress(0~1)`로 따라 이동
- pit 라벨: `PIT IN` / `BOX x.xs` / `PIT OUT` (증가형 `pit_elapsed`/`pit_stop_elapsed`)
- SC/VSC 헤더 배너, `PIT WINDOW OPEN` 배지(StrategyPanel BOX 버튼 강조)
- start lights

현재 그리드/출발 로직:

- 거리기반 그리드가 엔진에 통합되어 있습니다(`race_engine._init_grid`).
- `GRID_SLOT_PROGRESS_GAP = 0.0028`로 그리드 간격을 정의합니다.
- 각 포지션은 시작 시 `progress = total_progress = -(position-1) * GAP`로 스타트라인 뒤에 정렬됩니다.
- 라이트아웃 시 전원이 동시에 전진하며(순차 출발 아님), 그리드 거리 간격이 자연스러운 초기 gap으로 변환됩니다.
- `segment_at_progress`/`_is_drs_zone`는 `progress % 1.0`로 wrap하므로 음수 progress에서도 정상 동작합니다.
- 프론트(`TrackCanvas`)는 백엔드 progress를 `((p % 1) + 1) % 1`로 wrap해 표시합니다. 과거의 표시 전용 grid offset 로직은 제거되었습니다.

피트 렌더링:

- `in_pit`이면 `pit_lane_progress`로 `pit_lane_coords` polyline 위 위치를 보간 (고정 슬롯 순간이동 제거)
- pit 시작은 여전히 **랩 완료(결승선)** 시점. `pit_lane.entry_progress`/`exit_progress`와 트랙 progress 미연동 → 입·출구 위치 미세 어긋남 가능 (다음 작업 후보)

## 10. 완료된 작업 전체 목록

### A. 그리드 / 출발
- 거리기반 동시 출발 (`GRID_SLOT_PROGRESS_GAP=0.0028`)
- 순차 0.3초 출발·표시용 offset 제거
- 테스트: `test_grid_uses_distance_offset_at_lights_out`, `test_grid_initial_gaps_match_distance_offset`

### B. 예선
- Q1/Q2/Q3 녹아웃 (15→10→pole), track evolution (Q1=1.0, Q2=0.997, Q3=0.994)
- `QualifyingResult`: `knockout`, `q1_time`, `q2_time`, `q3_time`
- `RaceSetup.jsx` Q1/Q2/Q3 결과표 + 탈락 구간 스타일
- 테스트: `test_qualifying_knockout_format_builds_grid`, `test_qualifying_session_times_match_knockout_stage`

### C. 사고 / SC / VSC
- `simulation/incidents.py`: 원인×심각도 2축, 게임초당 rate
- `race_phase` (green/vsc/sc), VSC ×1.4 / SC ×1.8
- `car_stopped`→VSC, `crash`→SC, 배틀 contact 에스컬레이션
- SC 물리 동선과 단계별 대열 형성, SC/VSC 중 온트랙 순위 고정·피트 순위 변동
- 사고 처리 시간+대열 완성 기반 SC 철수, VSC 25초 시간 해제
- SC 피트 윈도우, AI 공짜 피트, 실제 추가 랩 방식 백마커 언랩, 리스타트 라인 GREEN
- 프론트: SC/VSC/pit window 배너, EventFeed 스타일
- SC 발생 빈도: Bahrain 57랩 12회 시뮬 — SC ~0.42/레이스, VSC ~0.92/레이스

### D. 피트 스톱
- 3단계: `in`(레인 진입) → `stop`(타이어 교체) → `out`(레인 진출)
- `compute_pit_components`: `pit_loss_time`(레인) + tire change(정지) 분리
- `pit_lane_progress`, `pit_elapsed`, `pit_stop_elapsed` (증가형, `pit_remaining` 제거)
- TrackCanvas 핏레인 경로 이동, TimingBoard/라벨 갱신
- 테스트: `test_pit_progresses_through_lane_phases`, `test_pit_lane_drive_and_change_times_are_separate`

### E. 서킷 / UI 기타
- Bahrain DRS zone 3구간 연장
- RaceSetup 퀄리파잉 결과 레이아웃 스크롤/sticky 수정
- 4개 서킷 geometry validation 통과 확인

### F. 검증 현황
- Backend: **106 tests OK** (`test_engine.py` + `test_api.py`)
- Frontend lint/build OK
- pytest는 `.venv`에 없음 → `python -m unittest discover -s tests` 사용

## 11. 다음 작업 (우선순위)

### P1 — 피트 / 레이스 (가장 자연스러운 연속 작업)
1. **피트 entry/exit progress 연동** — `circuit.pit_lane.entry_progress`/`exit_progress` 기준 피트인·아웃 트리거, 트랙 복귀 위치 정렬 (현재는 결승선 랩 완료 시점)
2. undercut / overcut, traffic-aware pit timing
3. 더블 스태킹, 피트 출구 정체

### P2 — SC/VSC 잔여
4. 적색기 (red flag)
5. SC 재출발(restart): 리더 컨트롤, 대시, 재가속 구간

### P3 — UI
6. Race Setup 서킷 미리보기 (`CircuitPreview` — track/pit/DRS/S-F, driver marker 없음)
7. CircuitDesigner validation UX

### P4 — 예선
8. 타이어 세트/소모, out lap / cool-down lap, traffic/impeding
9. 세션 **내** track evolution (현재는 Q1/Q2/Q3 세션 단위만)

### P5 — 장기
10. segment speed class (low/medium/high speed corner)
11. 차량 성능 세분화 (power/aero/grip/tire_wear/reliability)
12. 거리 기반 track model
13. 루트 git 버전관리 정비

## 12. 작업 시 주의사항

- `backend/.git`은 정상적인 git repository로 동작하지 않았습니다. `git status`/`git diff`에 의존하지 마십시오.
- `frontend/dist`는 FastAPI 정적 서빙에 사용됩니다. build 후 반영됩니다.
- JSON 데이터 수정 후 uvicorn reload가 놓치는 경우가 있었습니다. 서킷 데이터 변경 후에는 서버 재시작을 권장합니다.
- `node_modules`, `.venv`, `__pycache__`는 건드리지 마십시오.
- `rg` 사용 시 `frontend/node_modules`를 제외하십시오. 예:

```bash
rg -n "qualifying" backend frontend/src
```

- 레이스 엔진 변경 시 최소한 다음 검증을 수행하십시오.

```bash
cd /Users/kimyongjin/Desktop/f1/backend
.venv/bin/python -m unittest discover -s tests

cd /Users/kimyongjin/Desktop/f1/frontend
npm run lint
npm run build
```
