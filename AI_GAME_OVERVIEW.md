# AI Game Overview - F1 Race Manager

이 문서는 다른 AI에게 현재 게임의 전체 맥락을 빠르게 전달하기 위한 설명서입니다. 작업 로그가 아니라, 제품 의도와 구현 구조를 한 번에 이해시키는 컨텍스트 문서입니다.

## 1. 한 줄 요약

`F1 Race Manager`는 플레이어가 한 F1 팀을 선택해 예선, 스타팅 타이어, 피트 전략, 페이스 모드를 관리하고, 나머지 18대는 AI가 운영하는 실시간 레이스 매니지먼트/시뮬레이션 프로토타입입니다.

현재는 완성된 상용 게임이 아니라, 레이스 셋업부터 Q1/Q2/Q3 예선, 실시간 레이스, 피트콜, 타이어 마모, DRS/더티에어, 추월 이벤트, 사고, VSC/SC, 최종 결과까지 이어지는 실행 가능한 수직 슬라이스입니다.

## 2. 다른 AI에게 바로 줄 짧은 설명

다음 내용을 먼저 전달하면 됩니다.

```text
우리는 FastAPI 백엔드와 React/Vite/Pixi.js 프론트엔드로 만든 F1 레이스 매니저 게임을 개발 중이다.

플레이어는 팀과 서킷을 선택하고, 플레이어 팀 드라이버 2명의 스타팅 타이어를 정한다. 먼저 Q1/Q2/Q3 예선을 시뮬레이션해 그리드를 만들고, 이후 실시간 레이스를 시작한다. 레이스 중 플레이어는 자기 팀 드라이버에게 BOX BOX 피트콜, 교체 타이어 선택, CONSERVE/STANDARD/ATTACK 페이스 모드, 배속/일시정지 명령을 내릴 수 있다.

시뮬레이션은 실제 물리 엔진이 아니라 랩타임 기반 진행률 모델이다. 각 차량은 progress, current_lap, total_progress, total_time으로 위치와 순위가 계산된다. 랩타임에는 서킷 base lap time, 팀 차량 성능, 드라이버 pace/consistency/tire_management/overtaking/defending, 타이어 컴파운드와 마모, 페이스 모드, 현재 트랙 segment, DRS, 더티에어, 배틀 효과, SC/VSC 상태가 반영된다.

백엔드는 단일 인메모리 RaceSession을 유지하고 WebSocket으로 race_info, tick, race_end를 보낸다. 프론트는 Pixi.js TrackCanvas로 트랙과 차량을 그리고, TimingBoard, StrategyPanel, SpeedControl, EventFeed로 레이스 조작과 정보를 보여준다.

핵심 파일은 backend/simulation/race_engine.py, backend/session.py, backend/models/schemas.py, backend/data/circuits.json, frontend/src/App.jsx, frontend/src/components/RaceSetup.jsx, frontend/src/components/TrackView/TrackCanvas.jsx, frontend/src/components/Dashboard/StrategyPanel.jsx, frontend/src/components/Dashboard/TimingBoard.jsx 이다.
```

## 3. 플레이어 경험

### 기본 흐름

1. 셋업 화면에서 서킷과 플레이어 팀을 선택한다.
2. 레이스 랩 수를 고른다. 현재 허용 범위는 5-100랩이다.
3. 플레이어 팀 드라이버 2명의 스타팅 타이어를 고른다. UI에서는 SOFT, MEDIUM, HARD를 제공한다.
4. `RUN QUALIFYING`을 누르면 Q1/Q2/Q3 예선이 자동 계산된다.
5. 예선 결과표를 확인한 뒤 `START RACE`를 누른다.
6. 스타트 라이트 연출 후 WebSocket 연결이 열리고 실시간 레이스가 시작된다.
7. 레이스 중 플레이어는 피트콜, 타이어 선택, 페이스 모드, 배속, 일시정지를 조작한다.
8. 모든 non-retired 드라이버가 완주하거나 리타이어하면 최종 결과 모달이 나온다.

### 레이스 화면 구성

- 좌측: `TimingBoard` - F1 TV 스타일 라이브 타이밍, GAP/INT 토글, LAST, SPD, TIRE, PIT 표시
- 중앙: `TrackCanvas` - Pixi.js 트랙 뷰, 차량 마커, DRS zone, pit lane, landmark, start/finish, zoom/pan/follow
- 중앙 하단: `EventFeed` - 추월/방어/사고/SC/VSC/피트/명령 이벤트, EN/KO 전환
- 우측: `SpeedControl` - 1x/2x/5x, pause/resume
- 우측: `StrategyPanel` - 플레이어 드라이버별 타이어 수명, 페이스 모드, 피트콜, 랩타임 통계 모달

## 4. 현재 데이터

### 팀과 드라이버

- 10개 팀, 20명 드라이버가 `backend/data/teams.json`, `backend/data/drivers.json`에 있다.
- 팀은 차량 성능, 피트 크루 스킬, 신뢰도, 전략 성향, 컬러를 가진다.
- 드라이버는 pace, consistency, tire_management, wet_skill, overtaking, defending 스탯을 가진다.
- 현재 팀 목록:
  - Red Bull Racing: VER, PER
  - Ferrari: LEC, SAI
  - McLaren: NOR, PIA
  - Mercedes: HAM, RUS
  - Aston Martin: ALO, STR
  - Alpine: GAS, OCO
  - Williams: ALB, SAR
  - RB: TSU, RIC
  - Kick Sauber: BOT, ZHO
  - Haas: MAG, HUL

### 서킷

현재 `backend/data/circuits.json`에는 5개 서킷이 있다.

| ID | 서킷 | 국가 | 기본 랩 | 길이 | DRS | driving segment |
|---:|---|---|---:|---:|---:|---:|
| 1 | F1 Test Oval Circuit | Virtual | 30 | 4300m | 2 | 5 |
| 2 | Technical Park Circuit | Virtual | 42 | 4600m | 2 | 6 |
| 3 | Bahrain Inspired Circuit | Virtual | 57 | 5412m | 4 | 11 |
| 4 | Red Bull Ring Inspired Circuit | Austria | 71 | 4318m | 3 | 13 |
| 5 | Silverstone Circuit | United Kingdom | 52 | 5891m | 4 | 15 |

서킷 데이터는 단순 좌표 목록이 아니라 다음 정보를 포함한다.

- `layout_segments`: 트랙 중심선 원본. `straight`, `bezier`를 지원한다.
- `track_coords`: 컴파일된 폐곡선 polyline. API 응답과 렌더링에 사용된다.
- `pit_lane`: pit entry/exit progress, lane/wall/box offset 등 pit lane 생성 설정.
- `pit_lane_segments`: 명시적 pit lane 원본이 필요한 경우 사용한다.
- `pit_lane_coords`, `pit_wall_coords`: 컴파일된 pit lane 좌표.
- `drs_zones`: normalized progress 기준 DRS 구간.
- `landmarks`: T1, S/F, PIT 같은 표시용 마커.
- `segments`: driving model용 구간. `straight`, `heavy_braking`, `technical`, `traction`, `sweeping` 타입을 쓴다.

## 5. 백엔드 구조

백엔드는 `backend/main.py`의 FastAPI 앱이다.

주요 API:

- `GET /api/health`
- `GET /api/drivers`
- `GET /api/teams`
- `GET /api/circuits`
- `POST /api/circuits/validate`
- `POST /api/qualifying/run`
- `POST /api/race/setup`
- `DELETE /api/race/session`
- `WS /ws/race`

`frontend/dist`가 존재하면 FastAPI가 `/`와 `/assets`로 프론트 빌드 결과도 정적 서빙한다.

### 세션 모델

`backend/session.py`는 단일 활성 레이스 세션만 유지한다.

- `SessionManager.create_session()`이 기존 세션을 지우고 새 `RaceSession`을 만든다.
- `RaceSession`은 `RaceEngine`과 WebSocket client set을 가진다.
- broadcast interval은 real time 기준 `0.2s`이다.
- 내부 시뮬레이션은 `GAME_TICK_SECONDS = 0.1` 고정 substep으로 쪼개 진행한다.
- WebSocket 접속 시 `race_info`와 현재 `tick`을 즉시 보낸다.
- 레이스 종료 시 `race_end`를 broadcast한다.

지원 WebSocket 명령:

```json
{"type": "pit_call", "driver_id": 1, "tire_choice": "HARD"}
{"type": "set_pace_mode", "driver_id": 1, "pace_mode": "ATTACK"}
{"type": "set_speed", "multiplier": 5}
{"type": "pause"}
{"type": "resume"}
```

## 6. 레이스 엔진 핵심

핵심 파일은 `backend/simulation/race_engine.py`이다.

각 드라이버는 `DriverRaceState`로 관리된다.

- `progress`: 현재 랩에서의 위치. 대략 0.0-1.0
- `current_lap`: 완료한 랩 수
- `total_progress`: `current_lap + progress`
- `total_time`: 누적 레이스 시간
- `tire_compound`, `tire_age`, `tire_usage`, `tire_wear`
- `pace_mode`: CONSERVE, STANDARD, ATTACK
- `in_pit`, `pit_count`, `pit_request`
- `retired`, `finished`
- `speed_kph`, `drs_active`, `dirty_air_active`, `side_by_side_active`

순위는 기본적으로 `total_progress`가 높은 순서이며, finish order와 retired 상태가 반영된다. SC/VSC 중에는 순위가 고정되어 불필요한 순위 섞임을 막는다.

### 랩타임 기반 진행률 모델

이 게임은 타이어 접지력 기반의 정밀 물리 시뮬레이터가 아니다. 매 tick마다 해당 차량의 추정 랩타임을 만들고, 그 랩타임과 순간 speed profile에 따라 progress를 증가시키는 방식이다.

랩타임에 반영되는 요소:

- 서킷의 `base_lap_time`
- 팀의 `car_performance`
- 드라이버의 `pace`
- 타이어 컴파운드와 마모
- 드라이버 `tire_management`
- 드라이버 `consistency`에 따른 랜덤 변동
- 페이스 모드
- 현재 주행 중인 driving segment
- DRS, 더티에어, 방어 효과
- 추월/방어/실수에서 생기는 일시적 battle effect
- SC/VSC phase multiplier

성능 격차는 너무 벌어지지 않도록 압축된다.

```python
car_performance_multiplier = 1.0 + (car_performance - 1.0) * 0.50
driver_pace_multiplier = 1.0 + (pace_stat - 0.88) * 0.28
```

### 페이스 모드

- `CONSERVE`: 랩타임 +0.55초, 타이어 사용량 x0.82
- `STANDARD`: 기본값
- `ATTACK`: 랩타임 -0.45초, 타이어 사용량 x1.28

AI 드라이버도 상황에 따라 페이스 모드를 바꾼다. 앞차와 1초 이내면 공격, 뒤차가 0.85초 이내면 방어, 타이어가 낮으면 보존 쪽으로 기운다.

### 타이어 모델

파일: `backend/simulation/tire_model.py`

현재 컴파운드:

```python
SOFT   grip=1.005, degradation_rate=0.022, cliff_threshold=16
MEDIUM grip=0.990, degradation_rate=0.014, cliff_threshold=31
HARD   grip=0.970, degradation_rate=0.010, cliff_threshold=44
INTER  grip=0.960, degradation_rate=0.025, cliff_threshold=20
WET    grip=0.950, degradation_rate=0.020, cliff_threshold=25
```

현재 UI의 스타팅/피트 타이어 선택은 SOFT, MEDIUM, HARD 중심이다. INTER/WET은 스키마와 모델에는 있지만 날씨 시스템이 아직 dry 고정이라 실제 게임플레이 축으로는 활성화되어 있지 않다.

타이어 마모는 `tire_usage`를 기반으로 비선형 증가한다. `tire_management`가 좋은 드라이버는 effective tire age가 낮아진다. 0% 근처 dead tire penalty도 랩타임 손실로 반영된다.

### DRS, 더티에어, 배틀

앞차와 1초 이내로 붙으면 더티에어가 켜지고, 현재 progress가 DRS zone이면 DRS가 켜진다.

배틀 이벤트는 주로 `straight`, `heavy_braking` segment에서 발생한다.

대표 이벤트:

- `attack`: 추격 차량이 공격
- `defend`: 앞차가 방어 성공
- `side_by_side`: 두 차량이 나란히 코너 진입
- `forced_wide`: 공격자가 강하게 밀어붙임
- `lockup`: 공격 실패로 브레이크 락업
- `run_wide`, `traction_loss`, `minor_contact`
- `pass`: 실제 순위가 바뀌었을 때 발생

side-by-side와 forced-wide는 짧은 지속 상태를 가진다. 이후 코너 탈출 효과나 접촉으로 이어질 수 있고, 일부 접촉은 사고 모델로 escalation될 수 있다.

### 사고, VSC, SC

파일: `backend/simulation/incidents.py`

사고는 원인과 심각도로 분리된다.

- 원인: `driver_error`, `collision`, `mechanical`
- 심각도: `minor`, `car_stopped`, `crash`

효과:

- `minor`: 시간/타이어 손실 후 계속 주행
- `car_stopped`: 해당 드라이버 리타이어, VSC 발동
- `crash`: 리타이어, full Safety Car 발동

SC/VSC 동작:

- VSC는 시간 기준 25초 유지
- SC는 리더가 `SC_DURATION_LAPS = 3`랩을 완료하면 해제
- SC 중에는 필드가 `SC_BUNCH_PROGRESS_GAP = 0.0045` 간격으로 bunch up
- SC가 나오면 `pit_window_open = true`
- AI는 타이어 마모가 0.30 이상이면 일정 확률로 SC 피트 기회를 잡는다
- SC 해제 직전 lapped car unlap 처리도 있다

### 피트스톱

파일: `backend/simulation/pit_stop.py`, `backend/simulation/race_engine.py`

피트는 단일 countdown이 아니라 3단계다.

1. `in`: pit lane 진입 후 박스까지 주행
2. `stop`: 정지해서 타이어 교체
3. `out`: pit exit까지 주행

프론트는 `pit_phase`, `pit_lane_progress`, `pit_elapsed`, `pit_stop_elapsed`를 받아 pit lane 위에 차량을 애니메이션한다.

현재 pit entry/exit progress anchor가 있는 경우, 차량은 pit entry progress를 통과했을 때 피트에 들어가고, pit exit progress로 복귀한다. entry/exit가 랩 경계를 걸치는 경우 lap time 기록도 보정한다.

## 7. 예선 시스템

파일: `backend/simulation/qualifying.py`

예선은 Q1/Q2/Q3 녹아웃 방식이다.

- Q1: 상위 15명 Q2 진출
- Q2: 상위 10명 Q3 진출
- Q3: 폴 포지션 결정
- 각 세션은 SOFT 타이어 기준 attack lap을 여러 번 굴리고 best lap으로 정렬한다.
- 기본 `attempt_laps`는 3이고 API 허용 범위는 1-6이다.
- track evolution:
  - Q1: 1.000
  - Q2: 0.997
  - Q3: 0.994

예선 결과의 `grid_order`가 레이스 셋업으로 전달된다. 누락된 드라이버가 있으면 RaceEngine이 성능 기반 fallback order로 뒤에 붙인다.

## 8. 트랙/서킷 시스템

핵심 파일:

- `backend/simulation/track_compiler.py`
- `backend/simulation/track_geometry.py`
- `backend/simulation/speed_profile.py`
- `frontend/src/components/TrackView/TrackCanvas.jsx`
- `frontend/src/components/CircuitDesigner.jsx`

### 컴파일 방식

서킷 JSON의 `layout_segments`는 사람이 편집하기 쉬운 원본이다. 백엔드 로딩 시 `compile_circuit_layout()`이 이를 균일 간격의 `track_coords`로 변환한다.

- track point spacing: 16px
- pit lane point spacing: 14px
- `straight`, `bezier` 지원
- closed track는 마지막 좌표가 첫 좌표와 이어진다
- landmark의 `progress`가 있으면 컴파일 후 `track_index`로 매핑된다

### 검증

`validate_circuit_geometry()`는 다음을 확인한다.

- track point 수
- 시작/끝 폐곡선 거리
- start/finish index 범위
- 자기교차
- 너무 짧거나 긴 segment
- 캔버스 지원 bounds
- landmark index
- DRS zone 길이
- DRS zone이 straight segment 내부인지
- driving segment가 0.0부터 1.0까지 연속인지

### 속도 프로파일

`speed_profile.py`는 track geometry의 곡률을 사용해 local speed factor를 만든다. 코너에서는 느리고 직선에서는 빠른 움직임이 되도록 하며, 가속/감속 제한을 여러 pass로 적용한다. 그래도 최종 주행 모델은 여전히 랩타임 기반이다.

### Circuit Maker

`CircuitDesigner.jsx`는 setup 화면의 `Maker` 버튼으로 들어갈 수 있다.

현재 역할:

- 기존 서킷을 draft로 복제
- layout segment point/control point 편집
- DRS zone 편집
- landmark progress 편집
- driving segment type/range 편집
- backend validation 호출
- export JSON 표시

주의: 현재는 영구 저장 API가 없다. export JSON을 사람이 `circuits.json`에 반영하는 개발자 도구 성격이다.

## 9. 프론트엔드 구조

기술:

- React 19
- Vite
- Pixi.js 8
- reconnecting-websocket

주요 파일:

- `frontend/src/App.jsx`: setup/race phase 전환, start lights, race layout, final result modal
- `frontend/src/hooks/useRaceWebSocket.js`: race_info/tick/race_end 수신, command ack/error 이벤트 처리
- `frontend/src/components/RaceSetup.jsx`: 서킷/팀/랩/타이어 선택, 예선 실행, 레이스 시작
- `frontend/src/components/CircuitDesigner.jsx`: 서킷 draft 편집과 validation/export
- `frontend/src/components/TrackView/TrackCanvas.jsx`: Pixi 트랙 렌더링과 차량 마커 애니메이션
- `frontend/src/components/Dashboard/TimingBoard.jsx`: 전체 드라이버 라이브 타이밍
- `frontend/src/components/Dashboard/StrategyPanel.jsx`: 플레이어 팀 전략 조작과 랩타임 분석
- `frontend/src/components/Controls/SpeedControl.jsx`: 배속/일시정지
- `frontend/src/components/Controls/EventFeed.jsx`: 레이스 이벤트 로그

TrackCanvas 특징:

- progress를 polyline 길이 기준으로 보간한다.
- 서버 tick 사이를 부드럽게 보이도록 marker interpolation/prediction을 한다.
- pit 중인 차량은 track route가 아니라 pit lane route를 따른다.
- zoom, pan, player driver follow가 있다.
- DRS, pit lane, pit boxes, S/F, landmark label을 그린다.

## 10. 현재 구현된 것과 아직 아닌 것

### 구현됨

- 팀/드라이버/서킷 데이터 로딩
- Q1/Q2/Q3 예선
- 예선 결과 기반 그리드
- 스타팅 타이어 선택
- 실시간 WebSocket 레이스
- 랩타임 기반 진행률 레이스 엔진
- 타이어 마모와 컴파운드 성능
- 플레이어 피트콜과 AI 피트 전략
- 3단계 피트스톱과 pit lane 애니메이션
- 페이스 모드와 AI 페이스 모드
- DRS, 더티에어, 추월/방어/side-by-side/forced-wide 이벤트
- 실수/접촉/기계 문제 사고 모델
- VSC/SC, 필드 번칭업, SC pit window, unlap
- Pixi 트랙 뷰, 라이브 타이밍, 이벤트 피드, 랩타임 통계
- 서킷 validation API와 Circuit Maker draft editor

### 아직 제한적이거나 미구현

- 멀티 세션/멀티 유저가 아니다. 백엔드는 단일 인메모리 레이스 세션만 가진다.
- 레이스 저장/불러오기, 커리어 모드, 시즌 진행은 없다.
- 날씨는 스키마에는 있지만 레이스는 사실상 dry 고정이다.
- INTER/WET 타이어는 모델에 있으나 UI/날씨 gameplay에 거의 연결되어 있지 않다.
- 연료, ERS, 부품 업그레이드, 차량 setup, 드라이버 계약은 없다.
- 실제 meter 단위 물리 엔진은 아니다. progress와 랩타임 기반이다.
- 피트 더블 스태킹, pit exit traffic, undercut/overcut의 정교한 traffic-aware 전략은 아직 약하다.
- Safety Car restart의 리더 컨트롤/재출발 가속 연출은 단순하다.
- Circuit Maker는 저장 API가 없는 draft/export 도구다.
- 루트 디렉터리는 현재 정상 git repository가 아니다. `git status`에 의존하지 않는 편이 좋다.

## 11. 실행과 검증

Backend:

```bash
cd /Users/kimyongjin/Desktop/f1/backend
.venv/bin/uvicorn main:app --host 127.0.0.1 --port 8000 --reload
```

Frontend 개발 서버:

```bash
cd /Users/kimyongjin/Desktop/f1/frontend
npm run dev
```

Frontend production build:

```bash
cd /Users/kimyongjin/Desktop/f1/frontend
npm run build
```

Backend 테스트:

```bash
cd /Users/kimyongjin/Desktop/f1/backend
.venv/bin/python -m unittest discover -s tests
```

Frontend 검증:

```bash
cd /Users/kimyongjin/Desktop/f1/frontend
npm run lint
npm run build
```

## 12. 작업할 때 가장 먼저 볼 파일

레이스 로직을 바꿀 때:

- `backend/simulation/race_engine.py`
- `backend/simulation/tire_model.py`
- `backend/simulation/ai_strategy.py`
- `backend/simulation/incidents.py`
- `backend/tests/test_engine.py`

API/WebSocket을 바꿀 때:

- `backend/main.py`
- `backend/session.py`
- `backend/models/schemas.py`
- `frontend/src/hooks/useRaceWebSocket.js`

프론트 레이스 화면을 바꿀 때:

- `frontend/src/App.jsx`
- `frontend/src/components/TrackView/TrackCanvas.jsx`
- `frontend/src/components/Dashboard/TimingBoard.jsx`
- `frontend/src/components/Dashboard/StrategyPanel.jsx`
- `frontend/src/components/Controls/EventFeed.jsx`

서킷/트랙을 바꿀 때:

- `backend/data/circuits.json`
- `backend/simulation/track_compiler.py`
- `backend/simulation/track_geometry.py`
- `backend/simulation/speed_profile.py`
- `frontend/src/components/CircuitDesigner.jsx`
- `frontend/src/components/TrackView/TrackCanvas.jsx`

## 13. 다음 개발 우선순위 제안

1. 날씨와 INTER/WET 타이어를 실제 gameplay에 연결한다.
2. pit exit traffic, double stacking, undercut/overcut 판단을 강화한다.
3. SC restart와 restart battle을 더 명확한 이벤트로 만든다.
4. Circuit Maker에 저장/불러오기 또는 file export workflow를 정리한다.
5. race replay, save/load, season/career layer를 추가한다.
6. 차량 성능을 power/aero/grip/tire_wear/reliability 등으로 세분화한다.
7. 테스트와 문서를 최신 상태로 유지한다. 특히 서킷 수와 랩 범위는 기존 문서와 달라질 수 있으므로 코드/JSON을 우선 확인한다.
