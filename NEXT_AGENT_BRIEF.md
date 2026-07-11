# Next Agent Brief

> **다음 에이전트는 이 파일을 먼저 읽고, 상세 내용은 `AGENT_HANDOFF_2026-06-26.md`를 참고하십시오.**

작성 기준: 2026-06-26 (최종 갱신)

## 프로젝트 한 줄 요약

F1 매니지먼트/레이스 시뮬레이션 프로토타입. 레이스 셋업 → Q1/Q2/Q3 예선 → 실시간 레이스(피트/타이어/페이스/SC·VSC/사고) → Pixi 트랙 뷰까지 동작하는 수직 슬라이스.

- 위치: `/Users/kimyongjin/Desktop/f1`
- Backend: FastAPI + WebSocket + 인메모리 단일 레이스 세션
- Frontend: React + Vite + Pixi.js
- 데이터: 20명 드라이버, 10개 팀, 4개 서킷
- **최근 검증: Backend tests 106 OK, frontend lint/build OK**

## 실행

```bash
# Backend
cd /Users/kimyongjin/Desktop/f1/backend
.venv/bin/uvicorn main:app --host 127.0.0.1 --port 8000 --reload

# Backend tests
cd /Users/kimyongjin/Desktop/f1/backend
.venv/bin/python -m unittest discover -s tests

# Frontend
cd /Users/kimyongjin/Desktop/f1/frontend
npm run dev
npm run lint
npm run build
```

## 이번 세션에서 완료된 작업 (시간순)

| # | 작업 | 핵심 파일 |
|---|------|-----------|
| 1 | **거리기반 그리드 동시 출발** — 순차 0.3초 출발 제거, `GRID_SLOT_PROGRESS_GAP=0.0028` | `race_engine._init_grid`, `TrackCanvas.jsx` |
| 2 | **Q1/Q2/Q3 녹아웃 예선** + track evolution(Q1=1.0, Q2=0.997, Q3=0.994) | `qualifying.py`, `RaceSetup.jsx`, `schemas.py` |
| 3 | **사고 분류 2축** (원인×심각도) + **SC/VSC 상태머신** | `incidents.py`, `race_engine.py` |
| 4 | **SC 물리 출동/철수** + 10대 길이(56m) 합류 판정, 7대 길이(39.2m) 목표 간격 | `_advance_safety_car`, `_sync_safety_car_queue` |
| 5 | **사고 처리+대열 상태 기반 SC 해제**, VSC는 시간 기준(25s) 유지 | `_tick_safety_car_after_cars`, `_tick_race_phase` |
| 6 | **SC 피트 기회** + AI 공짜 피트 + 실제 추가 랩 **백마커 언랩/리스타트** | `_ai_sc_pit_decisions`, `_start_sc_unlapping`, `_begin_sc_in_this_lap` |
| 7 | **피트 3단계 자연화** — `in`/`stop`/`out`, 핏레인 경로 이동, 시간 증가형 표시 | `pit_stop.py`, `_tick_in_pit`, `TrackCanvas.jsx` |
| 8 | **서킷 기하 검증** — 4개 서킷 모두 `validate_circuit_geometry` 통과 (임시 plot 도구는 삭제됨) | `track_geometry.py`, `circuits.json` |

## 현재 동작 요약 (핵심 상수)

```text
그리드:     GRID_SLOT_PROGRESS_GAP = 0.0028
SC/VSC:     VSC ×1.4 / SC 대열 ×1.8 / 미합류 거리별 ×1.08~×1.25, VSC 25s
SC 간격:    10 car lengths × 5.6m를 서킷 길이에 맞춰 progress로 환산
SC 피트:    pit_window_open (SC만), AI wear≥0.30 & prob 0.4
피트:       in → stop → out, pit_loss_time(레인) + tire_change(정지)
            pit_lane_progress 0→1, pit_elapsed / pit_stop_elapsed (증가형)
사고:       minor / car_stopped→VSC / crash→SC
```

## 알려진 한계 / 미완성 (다음 에이전트가 알아야 할 것)

1. **SC Race Director 선택지**: 저시야 20대 길이 간격 및 `OVERTAKING WILL NOT BE PERMITTED` 분기는 아직 없습니다.
2. **VSC 종료 예고**: 현재 25초 뒤 바로 green이며 실제 FIA의 `VSC ENDING` 후 10~15초 랜덤 재개는 아직 없습니다.
3. **SC 리스타트 AI**: 리더 가속 지점은 구현됐지만 팀/드라이버 성향별 재출발 전술은 없습니다.
4. **테스트 실행**: `backend/.venv/bin/python -m unittest discover -s tests`를 사용합니다.

## 추천 다음 작업 (우선순위)

### P1 — 레이스/피트 고도화
- [ ] **undercut/overcut**, traffic-aware pit timing
- [ ] **더블 스태킹 / 피트 출구 정체** 모델

### P2 — SC/VSC 잔여
- [ ] **적색기(red flag)**
- [ ] **VSC ENDING 10~15초 랜덤 재개**, 저시야/언랩 금지 Race Director 분기

### P3 — UI/셋업
- [ ] **Race Setup 서킷 미리보기** (driver marker 없는 `CircuitPreview` 컴포넌트)
- [ ] CircuitDesigner validation UX 개선

### P4 — 예선 고도화
- [ ] 타이어 세트/소모, out lap / cool-down lap, traffic/impeding
- [ ] 세션 내 track evolution (현재는 세션 단위만)

### P5 — 장기
- [ ] segment speed class (low/medium/high speed corner)
- [ ] 차량 성능 세분화 (power/aero/grip/tire_wear/reliability)
- [ ] 거리 기반 track model (meter 단위)
- [ ] 루트 git 버전관리 정비

## 주요 파일 맵

```text
backend/simulation/race_engine.py   ← 레이스 핵심 (그리드, SC/VSC, 피트, 배틀)
backend/simulation/incidents.py     ← 사고 분류·확률
backend/simulation/qualifying.py    ← Q1/Q2/Q3
backend/simulation/pit_stop.py      ← compute_pit_components (lane + tire change)
backend/models/schemas.py           ← API/시뮬 모델 (RaceTickState, DriverPositionInfo)
backend/data/circuits.json          ← 4개 서킷 데이터
backend/tests/test_engine.py        ← 106 tests 중 대부분

frontend/src/App.jsx                ← SC/VSC/pit window 배너
frontend/src/components/RaceSetup.jsx ← 셋업/예선
frontend/src/components/TrackView/TrackCanvas.jsx ← 트랙·핏레인·차량 렌더
frontend/src/components/Dashboard/StrategyPanel.jsx ← BOX/pit window UI
frontend/src/components/Dashboard/TimingBoard.jsx   ← pit_elapsed 표시
```

## WebSocket tick 필드 (최근 추가/변경)

`RaceTickState`:
- `race_phase`: `green` | `vsc` | `sc`
- `pit_window_open`: bool (SC 중만 true)

`DriverPositionInfo` (피트 관련):
- `pit_phase`: `in` | `stop` | `out` | null
- `pit_lane_progress`: 0.0~1.0 (핏레인 위치)
- `pit_elapsed`: 피트 전체 경과 시간 (증가)
- `pit_stop_elapsed`: 타이어 교체 경과 시간 (증가)
- ~~`pit_remaining`~~ 제거됨

## 이벤트 타입 (EventFeed)

`incident`, `retirement`, `vsc_start`, `vsc_end`, `sc_start`, `sc_end`, `pit_window`, `unlap`, `pit_entry`, `pit_exit`, `pass`, 배틀 이벤트 등 — CSS는 `EventFeed.css`의 `event-feed__item--{type}`

## 작업 시 필수 검증

```bash
cd /Users/kimyongjin/Desktop/f1/backend && .venv/bin/python -m unittest discover -s tests
cd /Users/kimyongjin/Desktop/f1/frontend && npm run lint && npm run build
```

서킷 JSON 변경 후에는 uvicorn **재시작** 권장.

## 상세 문서

- `AGENT_HANDOFF_2026-06-26.md` — API, 모델, 상수, 프론트 상태, 완료 작업 전체 기록
- `ROADMAP_NEXT_STEPS.md` — 기술 로드맵, 구현 순서, 작업 단위(A/B/C) 템플릿
