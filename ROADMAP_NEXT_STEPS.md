# Roadmap Next Steps

> **인수인계용 빠른 요약은 `NEXT_AGENT_BRIEF.md`를 참고하십시오.**

이 문서는 다음 구현자를 위한 기술 로드맵입니다. 현재 프로젝트는 레이스가 실행되는 수직 슬라이스까지 완성되어 있으며, 다음 단계는 크게 두 축입니다.

1. 서킷/트랙 시스템 고도화
2. 드라이빙/레이스 모델 고도화

**검증 현황 (2026-06-26): Backend 106 tests OK, frontend lint/build OK**

## 1. 서킷/트랙 시스템

### 현재 상태

- 서킷 데이터는 `backend/data/circuits.json`에 정적 JSON으로 정의됩니다.
- 프론트엔드는 API의 서킷 목록을 받아 setup 화면에서 선택합니다.
- 트랙 렌더링은 `frontend/src/components/TrackView/TrackCanvas.jsx`에서 Pixi.js로 처리합니다.
- 현재 트랙은 `layout_segments` / `pit_lane_segments` 원본 데이터를 백엔드에서 컴파일한 `track_coords` / `pit_lane_coords`를 그립니다.
- 지원 source segment는 `straight`, `bezier`입니다.
- 컴파일러는 `backend/simulation/track_compiler.py`에 있으며, 곡선 원본을 일정 거리 간격의 좌표로 resample합니다.
- 기존 API contract는 `track_coords` 중심으로 유지되므로 프론트/시뮬레이션 기존 경로가 깨지지 않습니다.
- `pit_lane_coords`, `drs_zones`, `landmarks`도 JSON에 포함됩니다.
- `landmarks`는 `progress`를 가질 수 있고, 로딩 시 컴파일된 track index로 다시 매핑됩니다.
- `segments`는 직선/급제동/테크니컬/트랙션/스위핑 구간을 progress range로 정의합니다.
- `backend/simulation/track_geometry.py`에는 서킷 geometry validation과 `segment_at_progress()`가 있습니다.
- `frontend/src/components/TrackView/TrackCanvas.jsx`는 차량 위치, DRS zone, kerb, pit slot을 실제 polyline 길이 기준으로 보간합니다.
- 백엔드 테스트에는 서킷 중심선 자기교차, geometry validation, segment lookup 검증이 있습니다.
- **2026-06-26 확인**: 4개 seed circuit 모두 `validate_circuit_geometry()` 통과. 임시 시각화 도구는 삭제됨.

### 확인된 문제

- JSON에서 직접 곡선 control point를 편집하는 방식이라, 시각 피드백 없이 복잡한 서킷을 만들기는 어렵습니다.
- 실제 서킷을 더 닮게 만들려면 `bezier` 외에 `arc` 또는 clothoid-like transition 개념이 필요할 수 있습니다.
- DRS zone과 driving `segments`는 아직 normalized progress range입니다. 길이 기반 렌더링에는 맞지만, 실제 meter 단위 metadata는 없습니다.
- 트랙 폭, kerb 위치, runoff, braking marker는 아직 렌더링 장식이며 driving model에는 직접 연결되지 않습니다.
- setup 화면에서 서킷을 선택하기 전에는 레이아웃 미리보기가 없습니다.

### 1차 구현 완료: 검증 도구 강화

기본 서킷 geometry validation은 `backend/simulation/track_geometry.py`와 `backend/tests/test_engine.py`에 구현되어 있습니다.

현재 검증:

- `track_coords` 첫 점과 마지막 점이 너무 멀지 않은지
- `landmarks[*].track_index`가 유효 범위인지
- `drs_zones`의 start/end가 `0.0 <= start < end <= 1.0`인지
- DRS zone 길이가 너무 짧거나 긴지
- DRS zone이 driving `straight` segment 내부에만 있는지
- 선분 길이가 지나치게 긴 구간이 있는지
- 전체 bounding box가 렌더 캔버스 안에 들어오는지
- `segments`가 0.0부터 1.0까지 연속으로 정의되어 있는지
- `segment_at_progress()`가 주요 progress에서 올바른 segment를 반환하는지

추후 추가하면 좋은 검증:

- 마지막 구간과 시작 직선이 과도하게 평행/근접하지 않는지
- `pit_lane_coords`가 track 중심선과 과도하게 겹치지 않는지
- 같은 landmark label이 불필요하게 중복되지 않는지

### 2차 구현 추천: 서킷 미리보기

`frontend/src/components/RaceSetup.jsx`에 서킷 미리보기를 추가하십시오.

권장 방식:

- Race Setup 화면에서 선택한 circuit을 작은 preview canvas로 렌더링
- 기존 `TrackCanvas` 전체를 재사용하기보다, driver marker 없는 가벼운 `CircuitPreview` 컴포넌트 분리
- 표시 정보:
  - 레이아웃
  - pit lane
  - DRS zone
  - start/finish
  - 기본 랩 수
  - base lap time
  - overtaking difficulty

### 곡선 기반 트랙 1-3차 구현 완료

구현된 범위:

- `Circuit.layout_segments` / `Circuit.pit_lane_segments` schema 추가
- `TrackLayoutSegment` 타입: `straight`, `bezier`
- `backend/simulation/track_compiler.py` 추가
- 세 seed circuit 모두 곡선 원본 데이터 추가
- 기존 `track_coords` / `pit_lane_coords` API contract 유지
- landmark `progress` 추가 및 컴파일 후 index resolve
- `TrackCanvas`를 실제 polyline 길이 기반 interpolation으로 변경
- 백엔드 테스트 추가:
  - layout segment compile
  - seed circuit compile
  - landmark progress resolve

검증:

- `cd backend && .venv/bin/python -m unittest discover -s tests -v`
- `cd frontend && npm run lint`
- `cd frontend && npm run build`

### 다음 4차 구현 추천: 서킷 metadata/디자이너 기반 확장

현재 렌더링은 길이 기반이지만, 서킷 metadata는 아직 사람이 JSON으로 편집합니다. 다음 단계에서는 이 구조를 “관리 가능한 서킷 시스템”으로 만드는 것이 좋습니다.

추가할 개념:

- track polyline total length
- source segment별 길이/곡률/방향 metadata export
- progress range를 distance ratio 또는 meter-like unit으로 명시
- DRS zone, sector, driving segment를 같은 distance helper로 검증
- hard braking zone 후보를 곡률/직선 이후 감속 구간으로 자동 제안
- Race Setup 화면의 서킷 preview
- 간단한 서킷 디자이너:
  - control point 이동
  - segment 추가/삭제
  - pit lane 편집
  - DRS zone 범위 지정
  - landmark 지정

이 작업은 드라이빙 모델 고도화와 연결됩니다.
- 실시간 검증
- JSON export

처음부터 저장 API까지 만들 필요는 없습니다. 1차는 프론트에서 JSON을 생성하고 개발자가 `circuits.json`에 붙여 넣는 방식이어도 충분합니다.

## 2. 드라이빙/레이스 모델

### 현재 상태

- 차량은 트랙 위에서 움직이지만, 실제 주행은 대부분 랩타임/진행률 계산으로 처리됩니다.
- 주요 영향 요소:
  - car performance
  - driver pace
  - tire compound/performance/wear
  - tire management
  - pace mode
  - consistency randomness
  - dirty air
  - DRS
- 현재 차량은 “어느 코너/직선에 있는지”에 따른 성능 차이를 거의 받지 않습니다.

### 목표

차량을 단순한 점 이동에서 벗어나, 현재 주행 중인 구간 특성에 따라 다르게 달리게 만드십시오.

예:

- 바레인풍 서킷은 T1/T4/T10 같은 급제동과 DRS 추월이 중요
- 테크니컬 서킷은 코너링, 타이어 관리, 실수 가능성이 중요
- 오벌은 slipstream/DRS/최고속 영향이 큼

### 1차 구현 추천: track segment type

`circuits.json`에 구간 정보를 추가하거나, 기존 `sectors`보다 더 세밀한 `segments`를 추가하십시오.

예시:

```json
"segments": [
  { "name": "Main Straight", "start": 0.00, "end": 0.16, "type": "straight" },
  { "name": "T1 Braking", "start": 0.16, "end": 0.21, "type": "heavy_braking" },
  { "name": "T3-T4 Climb", "start": 0.21, "end": 0.36, "type": "straight" },
  { "name": "Middle Technical", "start": 0.36, "end": 0.62, "type": "technical" },
  { "name": "Back Straight", "start": 0.62, "end": 0.78, "type": "straight" },
  { "name": "Final Sector", "start": 0.78, "end": 1.00, "type": "traction" }
]
```

처음에는 backend schema에 optional field로 추가하고, 없으면 기존 방식으로 fallback하십시오.

### 2차 구현 완료: segment별 pace modifier

`simulation/race_engine.py`에서 현재 driver progress가 어떤 segment에 있는지 계산하고, lap pace에 작은 delta를 주는 1차 프로토타입이 구현되어 있습니다.

예시 영향:

- `straight`
  - car power 영향 증가
  - DRS/slipstream 영향 증가
  - tire wear 영향 감소
- `heavy_braking`
  - driver braking/racecraft 영향 증가
  - 추월 이벤트 가능성 증가
  - worn tire일 때 lock-up 가능성 증가
- `technical`
  - cornering/consistency 영향 증가
  - tire wear 영향 증가
  - attack mode 실수 가능성 증가
- `traction`
  - tire compound/tire wear 영향 증가
  - rear-limited 구간 느낌 구현 가능

현재 driver/team data에 세부 스탯이 없어서 기존 값을 임시 매핑해 사용합니다.

현재/추후 임시 매핑 예:

```python
braking = driver.stats.pace * 0.6 + driver.stats.consistency * 0.4
cornering = driver.stats.pace * 0.7 + driver.stats.consistency * 0.3
racecraft = driver.stats.pace * 0.5 + driver.stats.consistency * 0.5
```

### 3차 구현 완료: overtaking attempt model

DRS/dirty air 랩타임 영향 위에 `attack`, `defend`, `side_by_side`, `forced_wide`, `lockup`, `pass` 이벤트가 추가되어 있습니다.

현재 반영:

- 추격 차량이 1초 이내
- 현재 segment가 `straight` 또는 `heavy_braking`이면 attack/defend 이벤트 가능
- DRS와 ATTACK mode는 attack 이벤트 가능성을 높임
- 공격자 overtaking, 방어자 defending, circuit overtaking difficulty 반영
- `attack` 성공 시 공격자 단기 lap-time bonus와 앞차 압박 손실 적용
- `defend` 성공 시 추격 차량 단기 lap-time penalty 적용
- heavy braking에서 점수가 비슷하면 `side_by_side` 가능
- `side_by_side` 발생 후 약 6초 동안 지속 상태 유지
- 지속 중인 두 차량은 라이브 타이밍 status column에 `SBS` 표시
- side-by-side 지속 중에는 같은 배틀 이벤트 반복 생성을 차단
- side-by-side 발생 시 공격자는 `inside`/`outside`, 방어자는 `racing_line`/`defensive_line` 선택
- `inside`: 진입 이득, 코너 탈출 손해
- `outside`: 진입 손해, 코너 탈출 이득
- `defensive_line`: 방어 선택이지만 본인도 코너 중/탈출 손실
- side-by-side 종료 시 `run_wide`, `traction_loss`, `minor_contact` 결과 이벤트 가능
- `run_wide`와 `traction_loss`는 공격자 시간 손실
- `minor_contact`는 두 차량 모두 시간/타이어 손실
- heavy braking에서 공격자가 크게 이기면 `forced_wide` 가능
- `forced_wide` 이후 delayed corner-exit aftermath 가능
- forced-wide aftermath는 방어자 `run_wide`/`traction_loss`, 공격자 tight-exit `traction_loss`, `minor_contact`로 분기
- heavy braking attack 실패 시 consistency/tire wear/pace mode 기반 `lockup` 가능
- 실제 position이 개선되면 `pass` 이벤트 발생

아직 하지 않은 것:

- 강제 position 교환
- 접촉 누적 리스크
- side-by-side/forced-wide 이후 명시적 추월 성공률 보정

이벤트 예:

- `ATTACK`: DRS로 접근
- `DEFEND`: 앞차가 방어 성공
- `SIDE_BY_SIDE`: 두 차량이 나란히 코너 진입
- `RUN_WIDE`: 코너 탈출에서 바깥으로 밀림
- `TRACTION_LOSS`: 코너 탈출 가속 손실
- `MINOR_CONTACT`: 경미한 접촉으로 양쪽 손실
- `FORCED_WIDE`: 공격자가 안쪽 라인으로 방어 차량을 밀어냄
- `PASS`: 추월 성공
- `LOCKUP`: 공격 실패, 시간 손실

### 4차 구현 완료: mistake/result effect model

드라이버 consistency와 tire wear를 더 의미 있게 만들기 위한 1차 실수 모델이 반영되어 있습니다.

조건:

- low consistency
- high tire wear
- ATTACK mode
- dirty air
- heavy braking segment

현재 결과:

- `lockup` 이벤트 생성
- 공격자 lap-time penalty
- 공격자 tire usage multiplier 증가

현재 효과:

- `lockup` event feed 표시
- 짧은 lap-time penalty
- tire usage multiplier 증가

드문 효과:

- spin
- pit entry miss
- retirement

초기에는 retirement까지 가지 말고 time loss 이벤트만 추천합니다.

### 5차 구현 완료: AI pace mode

AI 드라이버가 race context에 따라 `CONSERVE`, `STANDARD`, `ATTACK`을 자동 선택합니다.

현재 반영:

- 판단 주기는 8-12초 cooldown
- 앞차와 1초 이내이면 타이어 상태가 허용하는 한 `ATTACK`
- 뒤차와 0.85초 이내이면 방어 스탯/타이어 상태에 따라 `ATTACK` 또는 `STANDARD`
- 타이어 수명 25% 이하에서는 대체로 `CONSERVE`
- 타이어 수명 12% 이하에서는 `CONSERVE`
- pit in-lap push, pit out fresh tire, race end phase는 더 공격적
- player drivers는 AI pace mode 대상에서 제외

아직 하지 않은 것:

- traffic-aware pit window와 pace mode 연동
- stint target 기반 장기 tire saving
- 팀/드라이버별 성향 차이

### 6차 구현 추천: vehicle attribute split

현재 팀 성능은 `car_performance` 하나가 중심입니다. 장기적으로 아래처럼 나누는 것이 좋습니다.

- power
- aero
- mechanical_grip
- tire_wear
- reliability

트랙 segment와 연결:

- straight: power
- high_speed: aero
- technical/traction: mechanical_grip
- long race: tire_wear
- failure events: reliability

이 작업은 밸런스 영향이 크므로, segment model이 자리 잡은 뒤 진행하십시오.

## 추천 구현 순서

### 완료됨
1. 서킷 검증 도구 강화
2. 트랙 segment schema 추가
3. segment별 pace modifier
4. 명시적 추월/방어 이벤트
5. 실수/락업 이벤트
6. AI pace mode
7. 거리기반 그리드 동시 출발
8. 예선 Q1/Q2/Q3 녹아웃 + track evolution(세션 단위)
9. 사고 분류(원인×심각도) + SC/VSC 상태머신 + 필드 번칭업
10. SC/VSC 후속(랩 기준 SC 해제 + SC 명시적 피트 기회 + 백마커 언랩)
11. 피트 스톱 자연화(in/stop/out 3단계 + 핏레인 주행 + 시간 증가형 표시)

### 다음 (우선순위)
12. **피트 entry/exit progress 연동** — `pit_lane.entry_progress`/`exit_progress` 기준 피트인·아웃, 트랙 복귀 위치 (현재 결승선 랩 완료 트리거)
13. SC/VSC·피트 잔여: 적색기, 더블 스태킹/피트 출구 정체, SC 재출발(restart), undercut/overcut
14. Race Setup 서킷 미리보기
15. 예선 추가 고도화: 타이어 세트/소모, traffic/impeding, out/cool-down lap
16. 거리 기반 track model
17. 서킷 디자이너 UX
18. 차량 성능 세분화

## 테스트 원칙

- 서킷 데이터 변경 시 `test_circuit_track_coords_do_not_self_intersect` 류의 데이터 테스트를 반드시 통과해야 합니다.
- 드라이빙 모델 변경 시 동일 seed에서 상대적 결과가 의도대로 변하는 테스트를 추가하십시오.
- 예: straight segment에서 DRS가 없는 경우보다 DRS가 있는 경우 progress가 더 빠르다.
- 예: technical segment에서 worn tire가 fresh tire보다 더 큰 penalty를 받는다.
- 예: heavy braking segment에서 attack mode는 추월 이벤트 확률을 높이지만 lock-up 위험도 높인다.

## 당장 맡기기 좋은 작업 단위

### 작업 A: Pit Entry/Exit Progress 연동 (P1 추천)

파일:

- `backend/simulation/race_engine.py` (`_complete_lap`, `_tick_in_pit`, pit trigger)
- `backend/models/schemas.py` (`DriverPositionInfo`)
- `frontend/src/components/TrackView/TrackCanvas.jsx`

목표:

- 피트 요청 시 결승선이 아닌 `pit_lane.entry_progress` 근처에서 pit-in 시작
- pit-out 후 `exit_progress` 위치로 트랙 progress 복귀
- 핏레인 애니메이션과 트랙 위치 전환을 자연스럽게 연결

### 작업 B: Circuit Preview

파일:

- `frontend/src/components/RaceSetup.jsx`
- 신규 `frontend/src/components/TrackView/CircuitPreview.jsx`

목표:

- setup 화면에 선택 서킷 preview 추가
- driver marker 없이 track/pit/DRS/start-finish만 표시

### 작업 C: Red Flag / SC Restart

파일:

- `backend/simulation/race_engine.py`
- `backend/simulation/incidents.py` (필요 시)
- `frontend/src/App.jsx`, `EventFeed.css`

목표:

- red flag 상태 추가, 전원 pit 또는 grid 재정렬
- SC 해제 후 restart 가속 구간·리더 컨트롤

### (완료) 작업 D: Driving Segment Prototype

- optional `segments` schema, segment pace delta, 테스트 — 모두 완료
