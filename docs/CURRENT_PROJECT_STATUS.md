# F1 2D 레이스 시뮬레이션 현재 상태와 로드맵

상태: **현재 프로젝트 요약**
기준일: **2026-08-03**
우선 기준: 제품·물리 결정은 [`SIMULATION_FOUNDATION.md`](SIMULATION_FOUNDATION.md)를 따른다.

이 문서는 지금까지 확정하고 구현한 내용을 빠르게 파악하기 위한 단일 현황 문서다. 완료된 작업의 과거 지시서와 단계별 계획서는 유지하지 않으며, 앞으로 상태가 바뀌면 이 문서와 기준 설계를 함께 갱신한다.

## 1. 제품 목표

- 실제 차량 상태를 기반으로 움직이는 **2D F1 레이스 시뮬레이션**을 만든다.
- 물리엔진이 차량의 위치·속도·헤딩·요·타이어·접촉 결과를 확정하고, 텔레메트리·그래픽·순위·이벤트는 그 상태에서 파생한다.
- AI는 추월 성공이나 사고 결과를 확률로 직접 선택하지 않는다. 공격·방어·철회, 목표 경로와 제어 입력을 선택하고 물리가 결과를 결정한다.
- 첫 완성 대상은 Bahrain International Circuit이다. 다른 트랙은 바레인에서 검증한 데이터·물리·AI 승인 절차로 확장한다.
- 현재 런타임은 모든 트랙을 평면으로 취급하는 `planar_2d`다. 고도·경사·뱅킹은 사용하지 않는다.

## 2. 확정된 차량 기준

| 항목 | 현재 기준 |
|---|---|
| 동력원 | 모든 팀 동일 750kW 단일 내연기관 기준 |
| 2026 하이브리드 | 배터리, 회생, 전기 부스트와 액티브 에어로는 후속 ruleset |
| 기준 질량 | 768kg, 연료 소모에 따른 질량 변화 적용 |
| 차체 폭 | 1.900m |
| 휠베이스 | 3.400m |
| 임시 전체 길이 | 충돌·렌더링용 5.000m |
| 물리 차원 | XY 평면, 고도 0m, 경사 0, 뱅킹 0° |

팀별 차이는 엔진 출력을 임의로 바꾸는 대신 공력, 구동계 효율, 브레이크, 기계적 그립, 트랙션과 드라이버 능력에서 만든다.

## 3. 런타임 구조

| 계층 | 주기 | 역할 |
|---|---:|---|
| 차량 물리 | 50Hz, 0.02초 고정 스텝 | 종·횡 힘, 위치, 속도, 헤딩, 요, 타이어, 접촉 |
| AI 전술 | 기본 10Hz | 공격·방어·철회, 피트와 페이스 판단 |
| 로컬 플래너 | 상황별 5/10/20/50Hz | 일반·교통·배틀·긴급 회피 경로 검증 |
| 차량 pose 전송 | 실시간 30Hz | 직전 전송 뒤의 50Hz pose 열을 바이너리로 전송 |
| 대시보드 전송 | 실시간 4Hz | 순위·간격·전략·런타임 상태 |
| 미니섹터 전송 | 실시간 1Hz | 27개 조각의 시간·비교 상태 |
| 랩 기록·이벤트 | 변경 시 | 완주 랩 기록과 공개 이벤트를 별도 전송 |
| 브라우저 렌더링 | 최대 60FPS | 120ms 재생 버퍼와 물리 pose 보간 |

지원 배속은 `1x/2x`다. 배속은 한 프레임에 더 큰 물리 간격을 사용하는 방식이 아니라 같은 0.02초 물리 스텝을 더 많이 계산하는 방식이다. 같은 시뮬레이션 시간과 seed에서는 배속·외부 tick 분할과 무관하게 같은 결과를 목표로 한다.

백엔드 `RaceSession`은 시뮬레이션 credit과 화면 전송 deadline을 분리한다. 차량 pose는 30Hz 바이너리 패킷으로 보내고, `race_state`는 4Hz, `race_timing`은 1Hz로 분리한다. 누적되는 `lap_history`는 매 대시보드 패킷에서 제외하며, 최초 접속에는 전체 snapshot, 이후에는 새로 완주한 랩만 `race_history` 증분으로 보낸다. 공개 이벤트도 `race_events`로 분리한다. 프런트는 `world_x_m`, `world_y_m`, `heading_rad`를 권위 pose로 사용하며, 일시정지 중 pose와 대시보드 전송은 각각 1Hz로 낮춘다.

2배속은 물리 정밀도를 낮추지 않고 같은 50Hz 스텝을 벽시계 1초에 100회 처리한다. 공격 라인 평가의 280m 물리 horizon은 유지하되 중복 경계 샘플을 제거하고, 같은 차량 쌍의 선택 corridor는 0.2초 동안 보존해 물리·AI 결과를 유지하면서 계산량을 줄였다. 기존 3배속은 20대 배틀 구간에서 단일 Python 코어 포화로 실효 배속을 안정적으로 유지하지 못해 제품과 서버 허용값에서 제거했다.

## 4. 현재 구현된 물리

### 4.1 차량 동역학

- 750kW 출력, 구동계 효율, 질량, 구름저항, 속도별 항력과 다운포스
- 구동력·제동력·횡력에 공통으로 적용되는 결합 마찰 한계
- 전·후축 슬립각과 횡력, 조향각, 요 관성, 헤딩과 요레이트를 적분하는 평면 dynamic bicycle model
- 타이어 컴파운드, 마모, 전·후축별 표면·코어 온도에 따른 종·횡 그립 변화. 기존 차량 전체 대표 온도 필드는 두 축 평균으로 호환 유지
- 연료 소모에 따른 차량 질량 변화
- 실제 종가속도, 0.30m 무게중심 높이와 3.40m 휠베이스를 사용한 준정적 전·후축 하중 이동
- 전·후축 평균 회전속도와 제동 락업·후륜 구동 슬립률. 축별 슬립은 실제 힘 초과량에서 계산하며 독립된 네 바퀴 관성 모델은 아니다.
- 전·후축 평균 브레이크 온도, 주행풍 냉각과 고온·저온 제동력 저하. Bahrain 반복 제동 입력의 52랩 평형값은 프런트 약 649°C·리어 약 480°C이며 정상 효율 구간을 유지
- 물리 입력 한계 초과로 발생하는 락업, 휠스핀, 언더스티어, 오버스티어와 run-wide
- 차체 회전 사각형 SAT와 swept collision을 이용한 접촉·고속 관통 방지
- 접촉 심각도, 정지 차량 위험물, 회피 또는 정지 선택
- 정적 차체 겹침은 위치만 분리하고 닫히는 속도가 없는 접촉에는 최소 횡충격·요충격을 만들지 않음

최근 코너에서 차체 뒤가 과도하게 드리프트하는 것처럼 보이던 문제는 실제 차체 헤딩과 그래픽 회전을 같은 pose에서 사용하도록 정리하고, rear slip/yaw 응답을 F1 차량에 맞게 안정화하는 방향으로 보정했다. 그래픽은 별도의 slip-angle 회전 필터로 물리 헤딩보다 늦게 따라가지 않는다.

### 4.1.1 타이어 열 모델 기반 수정 1~5단계

2026-07-29 기준으로 타이어 열 모델의 구조적 1~5단계를 구현했다. 이 단계는 C1~C5 물성, 동적 날씨, 네 바퀴 독립 모델을 포함하지 않는다.

- 차량 물리 결과는 후륜 구동 작업량, 전·후축 제동 작업량, 전·후축 슬라이드 에너지를 발생 지점에서 직접 제공한다. 기존 `tire_slide_energy_j`는 두 축 합계 파생 호환 필드로 유지하며 축별 합계 불변식을 테스트한다.
- `advance_tire_thermal_state()`는 세션이 전달한 `TrackConditions(ambient_temperature_c, track_temperature_c)`를 필수 입력으로 받고, baseline/lateral/braking/traction/slide 발열과 표면 외부 냉각·표면→코어 전달·코어 냉각을 `TireThermalBudget`로 반환한다.
- 160/140°C 수치 guard는 `TIRE_SURFACE_NUMERIC_GUARD_C`/`TIRE_CORE_NUMERIC_GUARD_C`로 유지한다. 적용 전 온도, hit, overshoot, 누적 횟수, 연속 시간, 최초 발생과 조건을 bounded 진단에 남긴다.
- 진단 snapshot은 schema version 2이며 현재·최고·누적 열수지와 clamp evidence만 보존한다. 물리 이력 배열은 추가하지 않는다.
- `RaceSetupRequest`의 환경 생략 시 회로별 `default_preset`을 사용하고, 명시값은 엔진·대시보드·데스크톱 진단에 동일하게 노출된다. Bahrain 기본값은 호환 기준 `30/40°C`다.
- 회귀 보호는 `backend/tests/test_tire_thermal_foundation.py`에 있다. 순수 57랩 상당 surrogate 평형, 과열 후 회복, Cool/Normal/Hot, 0.02/0.05/0.10초 스텝별 누적 열수지 안정성, clamp 진단, Bahrain 20대 smoke와 1,700초 실제 엔진 통합을 포함한다.
- 2026-07-29 Bahrain 30/40°C, seed 42의 20대 1,700초 직접 엔진 재현에서 lateral 발열 변환계수를 `18.0 → 6.0`으로 한 계수군만 보정했다. 보정 전 수동 표본은 후륜 표면/코어 `149.705/131.218°C`, 20대 과열, 최대 연속 `1,392.48초`였고, 보정 후 앱 로그 표본은 `116.681/103.853°C`였다. 전역 controller-scale cache 충돌 제거 뒤 2026-07-30 기준은 `119.248/105.247°C`, load-sensitive stiffness 적용 뒤 `117.321/107.975°C`였으며, 2026-08-03 Bahrain lateral-slope 연속성 보정 뒤 현재 직접 엔진 기준은 `110.834/97.999°C`다. 새 경로도 130°C 초과 차량 0대, 연속 과열 0초, surface/core guard hit 0회를 유지한다.
- 보정 후 후륜 누적 열원은 lateral `37.6%`, slide `27.4%`, traction `19.2%`, baseline `13.7%`, braking `2.1%`이며, 표면 순열량과 코어 순열량은 각각 양수로 남아 있어 장기 입력·피트·환경별 실주행은 계속 승인해야 한다.
- 축별 실제 제동 작업량은 물리 단계에서 이미 bias 분리된 권위값이므로 열 모델에서 brake bias를 재적용하지 않는다. 실제 `applied_brake_work_energy_j`는 변환계수만 적용하고, fallback 입력에서만 `brake_heat_share`를 사용하며, 축별 합계 회귀를 추가했다.
- 리뷰 보강 전 전체 backend `472개` 테스트와 1,700초 실제 엔진 장기 회귀가 통과했다. desktop 진단 테스트 `12개`도 통과했다. 이후 장기 기준·누적 에너지·Race Setup 회귀를 보강했다.

2026-07-30 새 macOS 패키지에서 Bahrain `HOT 36/52°C`, 57랩·20대 수동
스트레스 표본을 완주했다. `RACING` 시작부터 `RESULTS`까지 벽시계 약 56분 41초였고
결과 차량은 20대였다. 후륜 표면/코어 최고는 `132.867/121.450°C`, 최대 동시
130°C 초과는 3대, 최대 연속 초과는 186.9초였으며 surface/core guard hit는 모두
0회였다. 종료 시 과열 차량은 0대였고 후륜 표면/코어는 `106.014/91.438°C`까지
회복했다. 이 결과는 Hot 내구 표본으로는 통과지만, 초기 제품 승인 계약의
Bahrain `NORMAL 30/40°C`, 57랩·20대 표본을 대체하지 않는다.

결과 화면에서 `New Race`를 실행한 뒤 즉시·10초·30초·60초 checkpoint 모두
active session false, client 0, loop task false, WebSocket/canvas/geometry/texture/
pose 0을 유지했다. 경기 중 앱 합산 메모리 최고는 약 781.6MB, Electron working
set 최고는 약 656.2MB, Python RSS 최고는 약 125.4MB였다. Electron CPU는 계속
관측 불가이므로 전체 CPU 승인은 별도 과제로 남는다.

### 4.1.2 회로 정적 열환경 1단계

2026-07-29 기준으로 등록된 9개 회로에 `COOL/NORMAL/HOT` 정적 열환경 프로필을
추가했다. 지형 샘플인 `Circuit.track_conditions`와 열환경인
`Circuit.thermal_profile`은 별도 필드로 유지한다. 앱 시작 시
`backend/data/circuit_thermal_profiles.json`을 회로 ID로 한 번 결합하고, 누락·중복·
미등록 ID·프리셋 누락·온도 순서 오류를 즉시 실패시킨다.

- Race Setup은 API가 제공한 회로 프로필에서 프리셋과 대기·노면 온도를 표시하고,
  예선과 레이스 요청에 같은 `thermal_preset`을 보낸다. 회로 또는 프리셋 변경 시
  기존 예선 결과를 폐기한다.
- 세션 생성 시 우선순위는 명시적 `track_conditions` override, 요청 프리셋,
  회로 기본 프리셋 순이며, override와 preset을 동시에 보낸 요청은 거부한다.
  해석된 `TrackConditions`, `thermal_preset`, `track_conditions_source`는 세션 동안
  불변이다.
- 해석값은 Race Setup 응답, WebSocket `race_info`, `RaceTickState`, dashboard,
  타이어 열 진단과 데스크톱 진단에 동일하게 노출된다. 브레이크 열 모델도 고정
  30°C 대신 세션 대기 온도를 냉각 기준으로 사용한다.
- Bahrain `NORMAL`은 기존 회귀 기준인 대기 30°C·노면 40°C를 유지한다. Bahrain
  `COOL 24/32`, `HOT 36/52`와 나머지 회로의 값은 후속 공식 세션 자료 대조 전까지
  정적 시나리오 `provisional`로 기록한다. 근거표는
  [`CIRCUIT_THERMAL_PRESET_SOURCES.md`](CIRCUIT_THERMAL_PRESET_SOURCES.md)다.
- 회로 프로필 무결성, 세션 해석·전파, 타이어 환경 순서, 브레이크 냉각 환경 민감도
  회귀는 `backend/tests/test_circuit_thermal_profiles.py`에 있다.

### 4.1.3 C1~C5 물리 컴파운드 2단계 현재 상태

2026-07-30부터 물리 컴파운드와 주말 역할을 분리하는 migration을 적용했다.
`PhysicalTireCompound(C1~C5/INTER/WET)`와 `DryTireRole(HARD/MEDIUM/SOFT)`는
서로 다른 타입이며, 차량 물리·마모·열 모델은 session snapshot의 물리 C 코드를
사용한다. 기존 `TireCompound`는 API·전략 호환 adapter로만 남긴다.

- `backend/data/tire_compound_nominations.json`에 9개 회로의 세 건식 지명을
  등록하고 중복·누락·미등록 ID·역순 C 코드를 loader에서 즉시 거부한다. 모든
  값은 현재 `provisional`이며 공식 시즌별 지명을 확정한 값이 아니다.
- Bahrain은 `HARD=C1`, `MEDIUM=C2`, `SOFT=C3`로 session resolver가 한 번
  해석한다. `race_info`, setup/qualifying 응답, dashboard/history, desktop
  diagnostics에 역할·물리 C 코드·ruleset을 함께 보낸다.
- C1~C5의 초기 grip, degradation, cliff, 축별 grip, 최적 표면 온도, 작동창,
  hot 진단값은 지시서의 `game_calibration_provisional` 표를 사용한다. C4/C5는
  초기 seed이며 제품 승인 전 보정값으로 취급하지 않는다.
- 예선은 해당 회로의 Soft 지명을 사용하고, 출발·피트·AI 전략은 주말 역할을
  같은 resolver로 물리 C 코드에 변환한다. UI는 `HARD · C1` 같은 역할·C 코드
  표기를 사용하며 회로 변경 시 예선·출발 선택·온도 preset을 함께 무효화한다.
- 피트 랩 이력은 교체 전 컴파운드와 교체 후 상태를 분리해 기록하고, blanket
  초기화는 새 물리 C 코드의 사양을 사용한다. 진단은 C1~C5별 bounded aggregate와
  기존 `legacy_130` 지표를 동시에 유지한다.

현재 C1~C5 구현은 데이터·세션·물리·예선·전략·피트·UI migration까지 완료했고,
Bahrain `NORMAL 30/40°C` 57랩·20대 수동 완주도 확인했다. 다만 C4/C5 대표 회로
단축 제품 승인은 아직 남아 있으므로 전체 컴파운드 물성을 최종 calibrated 값으로
간주하지 않는다.

직접 `RaceEngine`을 생성하는 도구·테스트도 회로 nomination을 우회하지 않는다.
nomination이 있는 회로는 항상 회로 resolver를 사용하고, `physical_tire_compound`가
존재하는 차량 상태에서는 legacy `tire_compound`/`tire_role`이 물리값을 덮어쓰지
않는다. 타이어 교체는 role·physical·legacy 필드를 하나의 helper로 갱신한다.
컴파운드 진단은 현재값과 별도로 C1~C5의 peak surface/core, hot threshold 기준
연속 과열 시간과 최대 드라이버를 bounded하게 보존한다.

### 4.1.4 Red Bull Ring 최적화 1차 상태

2026-07-31 Red Bull Ring `circuit_id=4`의 0~2단계 기준선 작업을 시작했다.
변경 전 입력의 해시와 실행 결과는
`backend/data/calibration/red_bull_ring_runtime_baseline_v1.json`에 보존하고,
sector timing 보강 후 결과는 `red_bull_ring_runtime_baseline_v2.json`에 저장했다.

- FIA 2025 Austrian Grand Prix 공식 회로도의 섹터 거리 `1,215/1,697/1,414m`와
  중심선 `4,326m`를 출처로 기록하고, 현재 컴파일 길이 `4,318m`에 진행률로
  정규화했다. Red Bull Ring의 대섹터는 `0.0–0.280860`,
  `0.280860–0.673139`, `0.673139–1.0`이며 `9+12+6=27` timing loop를 사용한다.
- `audit_tier_a --circuit-id 4`의 `sectors_have_timing_ranges`가 기존 `false`에서
  `true`로 바뀌었고, 형상·피트·DRS 검사는 무회귀다.
- 정적 telemetry 비교는 평균 `MAE 5.257km/h`, `RMSE 7.529km/h`,
  `Bias -3.038km/h`, 최대 절대 오차 `32.719km/h`로 기존 기준 안에 있다.
  sector 범위 변경은 이 정적 지표를 바꾸지 않았다.
- 실제 `RaceEngine` 0.02초 고정 스텝 단독 5랩은 5랩 완료·유효 2~5랩 4개를
  달성했지만, fastest valid lap `70.149초`가 예선 median `64.472초`보다
  `8.805%` 느리고 `T4` 4회·`T10` 7회의 run-wide 표본이 있어 동적 5랩 승인은
  아직 실패 상태다. contact/off-track/numeric guard는 0이다.
- 입력 조건을 예선 비교에 맞춰 `SOFT=C4 + ATTACK`으로 별도 실행한
  `red_bull_ring_runtime_baseline_v3_qualifying_input.json`도 fastest valid lap
  `69.568초`로 median 대비 `7.904%` 느렸고 T4/T10 run-wide가 각각 16/10회였다.
  따라서 단순히 Medium/Standard 입력을 예선 기준과 섞은 문제만으로 설명되지
  않으며, controller/실행 경로의 후속 보정 대상으로 분류한다.
- 재사용 진단 CLI는
  `backend/tools/run_circuit_baseline.py`이며, 단독 동적 결과는 traffic·pit·SC
  제품 승인을 대신하지 않는다. 현재 결과만으로 전역 물리·타이어 계수는 변경하지
  않고, 다음 단계에서 T4/T10 controller 병목을 한 계수군씩 분석한다.

### 4.2 트랙과 레이싱라인

- OSM 중심선·피트레인을 로컬 미터 좌표로 투영하고 공식 길이에 맞춰 보정
- 트랙 곡률·폭·표면 경계를 사용한 차량별 전역 궤적 최적화
- 레이싱라인은 강제 경로가 아니라 로컬 플래너의 기준 비용과 prior
- 교통이 있으면 현재 corridor를 중심으로 후보 경로를 만들고 2/3/4-wide 공간을 평가
- 경계, 연석, 아스팔트 런오프, 잔디와 자갈 표면 비용 및 트랙 리밋 판정
- 네 바퀴가 모두 흰 선 밖일 때만 트랙 리밋을 활성화하고, 두 바퀴의 낮은 연석 사용은 허용
- 기본 최적선의 연석 허용은 0.60m로 유지하되 교통·추월 경로의 차체 검사는 낮은 연석 전체 폭을 사용
- Bahrain 가변 폭 14–22m와 T1 22m 적용. 공개 좌우 측량값이 없어 T1 안쪽은 7m로 유지하고 확장분은 바깥쪽에 배분
- 현재 Bahrain의 기준 차량·새 medium 결과에서 전역 `racing_line`은 100.527초, 전 랩을 계속 사용하는 tactical `inside`는 +0.180초, `outside`는 +1.782초, `defensive_line`은 +0.120초다. 차량·타이어가 바뀌면 다시 생성되며, tactical 전 랩 시간은 특정 코너 공격의 손익이 아니므로 T1 선택은 20m 간격·280m 물리 horizon과 상대 차량 점유로 다시 평가한다.
- 현재 생성된 Bahrain 전역·tactical 라인의 표면 평가는 경계 위반 0, 낮은 연석 접촉 0이다. 즉 연석 사용은 물리적으로 허용돼 있지만 현재 전역 후보가 실제 F1처럼 출구 연석을 적극 사용한다고 보기는 어렵다. 코너별 실측 횡위치 또는 더 강한 다중 초기값 최적화가 다음 검증 항목이다.
- Red Bull Ring TUM 기반 좌·우 폭과 2025 예선 텔레메트리 prior 적용
- Silverstone·Spa·Hungaroring은 실제 전체 경계가 없어 일부 fallback 사용

새 트랙에서도 중심선·폭·표면·차량 사양으로 빠른 선을 생성할 수 있다. 현재 방식은 초기 휴리스틱에서 시작하는 결정론적 좌표 탐색이므로 수학적 전역 최적해를 증명하지 않으며, 실제 경계와 연석 정보가 부족한 결과는 “현재 모델 안의 검증된 빠른 후보”로 부른다. 선수 텔레메트리는 강제 경로가 아니라 속도·제동점·횡 위치 검증 자료로 사용한다.

### 4.3 피트, 출발과 경기 운영

- 서버가 `grid → lights → lights_out → racing` 상태를 단일 기준으로 관리
- 라이트가 꺼지기 전 차량을 실제 그리드 박스 pose에 배치
- 피트 진입 분기, 제한 시작선, 박스, 제한 해제선, 출구 도로와 측면 합류를 하나의 연속 경로로 계산
- 제한 시작선에 서킷 이벤트 설정값인 80km/h로 맞추도록 남은 거리 기반 감속
- 바레인 피트 제한 구간에서는 80km/h를 유지
- 단일 기준 `box_progress`를 중심으로 10개 팀에 16m 간격의 고유 정차 위치를 배정한다. 같은 팀 두 차량은 같은 박스를 사용한다.
- 정차 22m 전부터 개러지 방향으로 smoothstep 횡이동하고 팀 박스 앞에서 정차한 뒤, 출발하면서 다시 피트 주행로 중심으로 복귀한다.
- 2025 FIA 바레인 피트 도면처럼 개러지는 독립 건물 10개가 아니라 하나의 연속된 피트 빌딩으로 표시한다. 각 팀 구획은 셔터형 베이 3개, 팀 색상 헤더·지붕 띠, 전면 피트박스로 구성한다.
- 피트 빠른 주행로의 개러지 쪽 경계부터 개러지 문턱까지 연속 아스팔트 작업면을 깔고, 첫·마지막 박스의 횡이동 구간은 양 끝으로 24m 연장한다. 따라서 차량이 주행로에서 팀 박스로 이동할 때 기본 잔디 지면을 통과하지 않는다.
- 팀 pit crew skill에 따른 정지 타이어 교체 시간은 피트 주행 시간과 별도로 계산
- 제한 해제 뒤 출구 도로에서 가속하고 본선 차량·ManeuverGroup 점유를 확인해 `yield → hold → merge`
- 피트와 트랙 전환 이벤트도 최종 물리 pose를 사용해 순간이동 방지
- SC/VSC, 피트 출동, 대열 수집, 언랩, 재시작과 피트 복귀 지원
- SC 중에는 본선 차량의 대열 순서를 별도 상태로 고정한다. 피트 차량은 대열에서 빠지고, 피트아웃 뒤 바레인 `SC2=0.102`를 통과하기 전까지 실제 주행 거리 기반 임시 순위를 사용한 뒤 SC2에서 새 대열 위치를 한 번만 확정한다. 이후에는 수치상 위치가 교차해도 본선 추월로 처리하지 않는다.
- SC catch-up의 `0.85` 랩타임 요청은 명시적인 `STRAIGHT` 구간에서만 일반 목표속도보다 빠르게 적용한다. Heavy braking·technical·sweeping·traction 구간은 그린 플래그의 보정된 물리 속도 한계를 넘지 않으며, 대열 수집 중 SC 자체는 `2.5` 랩타임 계수로 속도를 낮춰 후미가 코너 과속 없이 합류하게 한다.
- SC/VSC의 계획된 목표속도 감속은 비상 제동과 분리한다. 낮은 race-control 속도 계수만으로 제동 modulation reserve를 줄이지 않으며, 실제 `emergency_braking` 또는 물리 입력 오류가 있을 때만 락업 한계를 넘을 수 있다. 이로써 SC 투입·복귀 때 전 차량에 락업 이벤트가 연쇄 발생하던 현상을 제거했다.
- SC/VSC 전환 시 모든 차량의 DRS·tow·dirty-air와 wake 기반 드래그·다운포스·제동·횡그립 배율을 즉시 초기화한다. 중립화 구간에서는 이전 그린 플래그의 `AIR` 상태가 물리에 남지 않는다.
- SC 대열 접근 시 최대 약 800m 전부터 앞차 속도·닫힘 속도·남은 거리를 사용한 `5m/s²` 감속 속도-envelope를 적용. 이 감속 영향권을 대열 꼬리에서 누적 600m까지 중·후미 차량 순서로 전파해 후방 아코디언 병목을 줄이되, 아직 먼 차량의 정상 catch-up을 막지 않는다.
- 대열 합류 완료는 50m 이내 진입만으로 확정하지 않고 앞차와의 상대속도가 `24km/h` 이하일 때 확정
- 합류 뒤에는 7 car-length 중심 간격을 목표로 앞차 속도를 상한으로 사용해 재가속·급제동 반복을 억제

2026 FIA Sporting Regulations의 기본 피트레인 제한은 80km/h이며 Race Director가 변경할 수 있다. 프로젝트도 `PitLaneConfig.speed_limit_kph`로 분리해 바레인을 80km/h로 설정한다.

바레인 물리 피트 경로는 약 778.9m다. 기존 60km/h에서는 진입 20.36초·정차 2.38초·출구 21.32초로 총 44.06초였고, 80km/h와 팀별 박스 적용 뒤 기준 단독 주행은 진입 12.84초·정차 2.38초·출구 20.40초로 총 35.62초다. 이 값은 피트 진입 anchor부터 본선 측면 합류까지의 전체 경과다. `pit_loss_time=23초`는 같은 구간을 본선으로 달리는 시간과 비교한 목표 순손실값이므로 전체 피트 경과와 동일한 숫자가 아니다.

바레인 제한 시작·종료 landmark는 `0.30 → 0.70`으로 적용했다. 제한 구간은 약 311.6m이고 기준 차량의 피트 진입 anchor부터 본선 측면 합류까지 전체 경과는 약 28.78초다. 첫 팀 박스 전에는 약 83.8m의 제동거리, 마지막 팀 박스 뒤에는 약 83.8m의 재가속거리가 남아 10개 팀의 16m 간격 고유 박스와 연속적인 감속·재가속을 여유 있게 유지한다. 비교했던 `0.40 → 0.60`은 양 끝 여유가 약 5.9m뿐이라 팀 박스가 8개 고유 위치로 겹치고 마지막 차량의 제한 해제 속도가 불연속이 되어 사용하지 않는다.

### 4.3.1 콜드 피트 아웃랩 설계 상태

건식 출발 타이어는 포메이션 랩을 생략한 현재 제품의 추상화로 90°C를 유지하고,
피트에서 교체한 건식 타이어만 70°C로 분리하는 방향을 확정했다. 다만 현재
C1~C5 `blanket_temperature_c`와 출발·피트 초기화는 모두 90°C를 사용하므로 아직
구현된 기능이 아니다.

피트 70°C 적용 전에 실제 thermal grip 즉시 초기화, 콜드 상태의
`fresh_after_stop → ATTACK` 차단, 목표속도·스로틀·제동의 연속적 회복, 피트 합류
gap 확대가 함께 구현돼야 한다. 물리 그립은 이미 온도에 따라 감소하므로 콜드
제어에서 그립을 다시 곱하지 않는다. 자세한 계약과 테스트 행렬은
[`COLD_OUTLAP_CONTROL_DESIGN.md`](COLD_OUTLAP_CONTROL_DESIGN.md)에 기록했다.

이 기능은 다른 회로의 현재 90°C 기준선을 먼저 확보한 뒤 구현한다. 회로 보정과
콜드 아웃랩을 동시에 바꾸지 않아 사고·열·랩타임 변화의 원인을 분리한다.

## 5. AI와 레이스크래프트

현재 AI는 다음 계층으로 나뉜다.

1. 전역 최적선과 속도 프로파일
2. 점유·충돌·경계를 보는 온라인 로컬 궤적 플래너
3. 공격·방어·철회와 위험을 판단하는 전술 계층
4. 경로를 조향·스로틀·브레이크로 추종하는 저수준 제어기
5. 모든 결과를 확정하는 공통 물리엔진

구현된 항목:

- 현재 직선 길이, 다음 제동구간, 접근속도, DRS, 웨이크와 필요 추월거리 기반 공격 판단
- `attack`, `hold`, `abort`와 설명 가능한 `reason_code`
- 두 차량 pair가 아니라 최대 4대의 `ManeuverGroup`으로 corridor와 코너 우선권 관리
- front-axle 위치와 inside corridor를 사용한 코너 진입 우선순위
- 공격·방어 차량은 다음 제동 코너 1.55초 또는 최소 70m 전까지 한 번의 corridor를 정한다. 이후에는 저장한 레이싱라인 상대 횡위치를 따라 코너 곡률만 추종하며, 제동 중 상대 움직임에 반응해 반대편으로 다시 이동하지 않는다.
- 급제동 구간의 안쪽 보너스와 명령 단계의 `inside` 하드코딩을 제거했다. 안쪽·바깥쪽을 같은 280m 물리 horizon으로 계산하고 차체 clearance가 확보되는 후보 중 빠른 쪽을 선택한다. Bahrain T1에서 본선 racing line이 점유된 회귀는 `outside`를 선택한다.
- 실제 차체 clearance가 생겨야 추월 완료
- 코너 진입 권한이 거절되면 뒤 차량을 즉시 레이싱라인으로 당기지 않고 `yield` 상태에서 현재 corridor를 유지한다. 차체가 완전히 뒤로 빠진 뒤에만 1.25초 smoothstep 목표로 레이싱라인에 복귀한다.
- 폭이 부족하거나 비용이 높으면 접촉 없이 철회
- 120m 내 여러 차량 중 가장 강하게 정렬된 후류 원천 선택
- 직선에서는 tow, 곡률이 충분한 코너에서는 dirty-air의 다운포스·제동·횡그립 손실
- 타이어와 연료 자원을 고려한 AI 페이스·피트 전략
- 같은 timing 순위가 아니라 300m 안의 실제 종·횡 차체 배치에서 가장 가까운 선행 차량을 추종 대상으로 찾는다. 제동 가능 거리 기반 속도 상한을 50Hz 물리 입력으로 사용하며, 물리 계산 뒤 앞차 속도를 복사하던 비접촉 보정은 제거했다.
- 저수준 물리의 최소 간격 위치·속도 하드 클램프도 제거했다. 20대 Bahrain 30초 진단에서 비접촉 차량의 20ms 속도 감소는 타이어 제동 한계인 약 4.0km/h 이하였고, 남은 더 큰 변화는 실제 swept-body 접촉 상태에서만 발생했다.

바레인 v1 판단 벤치마크는 [`AI_RACECRAFT_BENCHMARK.md`](AI_RACECRAFT_BENCHMARK.md)에 기록한다. 아직 실제 F1 리플레이와 추월 시도·성공·철회·접촉 빈도 분포가 일치한다고 보지는 않는다.

## 6. 타이밍과 텔레메트리

- 트랙을 공식 3개 대섹터로 구성하고 바레인은 S1 9개·S2 12개·S3 6개, 총 27개 타이밍 루프로 세분화
- 50Hz 프레임 안의 타이밍 선 통과 시각을 보간
- GAP은 선두, INT는 바로 앞 차량과의 차이이며 최신 공통 타이밍 루프를 공식 기준점으로 사용한다. 루프 사이에서는 현재 트랙 진행 거리·속도 추정을 기준점과 혼합해 매 상태 갱신마다 자연스럽게 움직인다.
- 공통 기록이 아직 없거나 추월 직후 최신 루프 통과 순서가 현재 순위와 반대이면 해당 기준점을 무효화하고 진행률 기반 `~`·`EST`로 전환한다. 음수 기록을 `0.000`으로 잘라 전체 열이 0이 되는 현상을 방지한다.
- 랩 기록에 3개 sector time과 27개 mini-sector time을 보존하고 각 조각을 세션 최고·개인 최고·그 외 기록으로 비교
- 좌측 Live Timing의 모든 드라이버 이름은 데이터 센터 모달을 열며, 선택한 차량의 27개 조각·GAP/INT·랩별 S1/S2/S3 기록과 타이어 마모·전후축 표면/코어 온도·열그립, 연료, 차량 질량, 전후 브레이크 온도·효율, 축 하중·슬립을 표시한다. 상시 섹터 패널은 제거해 트랙 캔버스 높이를 유지한다.
- Three.js 트랙에는 물리 타이밍과 같은 대·미니섹터 경계선을 그린다.
- 라이브 전송은 5개 pose 값과 화면에서 실제 사용하는 순위·속도·타이어·피트·배틀 상태만 포함한다. 상세 물리 텔레메트리는 엔진 내부 권위 상태와 테스트용 전체 tick에서 유지한다.
- 내부 AI 상태용 maneuver-group 생성·해제 이벤트는 화면 피드에 노출하지 않는다. 락업·트랙션·접촉·일상 배틀 메시지는 차량별 cooldown을 적용하고, 공격·방어·철회 메시지는 전체 화면 기준 최소 1.5초 간격을 둔다. 이는 물리나 AI 판단을 삭제하지 않고 공개 피드만 정리한다.
- 화면 이벤트는 안정적인 `event_id`를 사용하고 최근 50개만 보존한다. 이벤트 증가 자체는 GB 단위 메모리의 원인이 아니지만 React 갱신과 DOM 스크롤을 늘릴 수 있어 pose·대시보드와 분리했다.

## 7. 현재 그래픽 구조

현재 트랙 화면은 Three.js WebGL 렌더러만 활성화한다. PixiJS 소스와 의존성은 롤백을 위해 남겨 두었지만 앱에서 import하지 않으며 프로덕션 번들에도 포함되지 않는다.

- Three.js animation loop 최대 60FPS, device pixel ratio 최대 1.5
- 레이스 피드의 `DIAG` 버튼은 우측 상단의 소형 Runtime Diagnostics 모달을 연다. 모달에는 1초 단위 FPS, 프레임타임 P95, 30FPS보다 느린 프레임 수, draw call·triangle, WebGL geometry·texture, pose 버퍼, 장면·개러지·차량 모델 생성 횟수, 지원 브라우저의 JS heap과 DOM 노드 수를 표시한다. JS heap은 최대 300개·최근 5분의 고정 길이 표본만 사용해 MB/분 추세와 최고값을 계산하며 사용자가 계측 창을 초기화할 수 있다. 트랙 헤더의 긴 인라인 성능 문자열은 공간 확보를 위해 숨긴다.
- 50Hz pose 열을 약 120ms 늦춰 재생해 패킷 사이를 cubic Hermite 곡선으로 보간
- 차량별 pose는 JS 객체 배열과 `shift()`가 아니라 160칸 typed-array 링 버퍼에 기록한다. 보간 결과 객체도 차량별 1개를 재사용해 30Hz 수신·60FPS 보간의 단기 객체 생성을 억제한다.
- 최대 120ms·45m의 제한적 dead reckoning은 패킷이 늦을 때만 사용
- 메인 확대 배율 `700/1000/1500%`, 기본 `1000%`
- 100% 전체 트랙 미니맵은 별도 overview로 유지
- 미니맵에 현재 카메라 영역을 청록색 사각형으로 표시하고, 클릭한 위치로 자유 카메라를 이동
- 리타이어 차량은 사고 위험이 활성화된 동안에만 정지 위치를 빨간 마커로 유지한다. 위험 해제 뒤에는 Three.js 차량·화면 라벨·미니맵 SVG 마커·pose 재생 버퍼를 함께 제거하며, 일반 리타이어 흔적은 남기지 않는다.
- 가변 폭 아스팔트 폴리곤, 실제 표면 구역, 피트 도로, 그리드, 대·미니섹터 표시
- 피트 제한 구간 안에 10개 팀 색상 기준의 정적 피트 박스·개러지 베이를 배치한다. 박스 라인, 팀 컬러 중심선, 개러지 전면 헤더·기둥·지붕·장비를 저비용 geometry로 구성하며, 팀 목록이 바뀔 때만 장면을 재구성한다.
- 피트 진입 차량의 월드 pose 라벨에는 `PIT IN`, 정차 중 `STOP / PIT`, 출구의 `PIT OUT / YIELD / HOLD / MERGE`와 서버 누적 시간을 표시한다. 표시는 기존 `pit_elapsed`, `pit_stop_elapsed`, `pit_phase`, `pit_merge_state`만 사용하므로 그래픽을 위해 별도 타이머나 물리 상태를 만들지 않는다.
- 백엔드 SC 진행률을 사용하는 노란 3D 세이프티카, 점멸등, 화면 라벨과 미니맵 마커. 피트 출동·본선 주행·피트 복귀 경로를 같은 서버 상태로 전환
- 차량 pose와 카메라 추적을 같은 60FPS 재생 상태에서 계산
- 정적 트랙과 차량 pose 갱신을 분리하고 `ThreeTrackCanvas`를 lazy chunk로 로드
- 과거 Pixi와 분리한 공용 pose 재생 모듈을 Three 렌더러가 그대로 사용
- 패킷 수신 시 재생 시각을 즉시 되감지 않고 연속 시계로 완만하게 보정해 네트워크 jitter에 의한 pose 점프를 방지
- 동일 객체인 트랙·연석·레이싱라인은 React 갱신 때 다시 직렬화하지 않아 메인 스레드 프레임 스파이크를 줄임
- 그리드 박스의 `EdgesGeometry` 생성에 사용한 임시 `BoxGeometry`는 즉시 해제해 레이스 재구성 때 GPU geometry가 누적되지 않게 함
- `Exit`/`New Race`는 서버 세션 종료를 기다린 뒤 setup으로 돌아간다. 종료 시 50Hz task와 WebSocket, trajectory 버퍼, React pose/event 상태, Three.js geometry/material/render list/WebGL context를 명시적으로 해제한다.
- 기본 트랙 프로필 캐시는 다음 레이스 준비 시간 단축을 위해 유지하지만, 메모리 비중이 큰 차량별 최적선 프로필 캐시는 레이스 종료 시 비운다. 런타임이 확보한 힙 영역 때문에 OS의 RSS가 즉시 같은 폭으로 감소하지 않을 수 있으나 이전 RaceEngine과 차량별 세션 객체는 유지하지 않는다.
- 종전에는 약 88~175KB의 전체 차량 상태를 30Hz로 반복 전송했고, 랩 수가 늘수록 모든 차량의 전체 `lap_history`도 함께 커졌다. 현재는 바이너리 pose 30Hz·대시보드 4Hz·미니섹터 1Hz·변경 시 증분 랩 기록으로 분리하고 기본값을 생략한다. 포즈는 Pydantic 객체를 만들지 않고 50Hz 스텝에서 20바이트 표본을 바로 보존한다.

PixiJS도 WebGL이었으므로 이번 변경은 GPU 가속을 새로 켠 것이 아니라 2D 장면을 Three.js 2.5D 장면으로 교체한 것이다.

### Three.js 실험 렌더러

바레인에서 다음 구조의 기능 플래그 프로토타입을 구현했다.

- 물리·AI·WebSocket과 pose 재생 버퍼는 유지한다.
- Three.js `OrthographicCamera`를 사용하는 `ThreeTrackCanvas`를 lazy chunk로 로드한다.
- 물리 `world_x_m/world_y_m`는 Three.js `X/+Z` 평면에 매핑하고 Y는 시각적 높이에만 사용한다. `+Z`는 기존 캔버스의 아래 방향과 같아 메인 화면과 미니맵이 상하 반전되지 않는다.
- 트랙·백엔드 `surface_zones` 기반 연석·런오프는 시작 시 한 번 생성하는 정적 Mesh로 만든다.
- 코너 연석·런오프의 시작과 끝은 폭을 점진적으로 열고 닫고, 작은 반경에서 접히는 표시용 도로 경계는 원형 이웃 보간해 T1 안쪽의 직각 메시를 제거한다. 이 보간은 물리 중심선과 텔레메트리를 바꾸지 않는다.
- 피트 경로는 물리와 같은 진입·출구 anchor를 사용하고 별도 아스팔트, 양쪽 흰 경계선, 80km/h 시작·종료선을 표시한다.
- 팀별 개러지는 서버가 각 드라이버에 전송하는 `pit_box_progress`로 배치한다. 따라서 그래픽 개러지와 물리 정차 위치가 같은 종방향 좌표를 사용하고, 차량은 `box_offset`만큼 개러지 방향으로 이동해 박스 중심에 선다. 인접 팀 중심 간 화면상 거리를 모듈 폭으로 사용해 지붕·후면 벽·캐노피가 끊기지 않는 하나의 건물처럼 연결된다.
- 표시용 레이싱라인은 최적화 결과점을 단순 장거리 chord로 잇지 않고, 물리 pose 계산과 같은 폐곡선 Catmull–Rom을 0.75m 간격으로 재샘플링한다.
- 20대 차량은 단순 low-poly geometry와 소수 재질을 사용한다.
- React 대시보드와 SVG 미니맵은 유지한다.
- 전환 기간 외에는 PixiJS와 Three.js가 같은 트랙 장면을 장기적으로 이중 렌더링하지 않는다.
- 동적 고해상도 그림자·모션 블러·과도한 후처리보다 baked/fake shadow와 정적 배치를 우선한다.

Three.js 렌더러는 30Hz pose와 50Hz pose 열을 사용하며 패킷 주기를 올리지 않는다. 정적 트랙 데이터는 내용이 변경될 때만 WebGL 장면을 재구성하므로 pose 갱신이 트랙 mesh를 다시 만들지 않는다. 2026-07-24 실제 브라우저에서 바레인 20대로 약 30분의 시뮬레이션 시간을 주행했다. 화면은 대부분 60FPS, 프레임타임 P95 16.8~18.6ms, 최근 120프레임 기준 33.3ms 초과 0개를 유지했고, 데이터 센터 모달을 열고 닫아도 5~10초 정지는 재현되지 않았다. JS heap은 약 26~54MB 사이에서 회수됐으며 canvas는 1개, texture는 13개로 유지됐다. geometry는 동적 시설·상태가 처음 카메라에 들어오며 GPU에 업로드되는 동안 증가한 뒤 안정됐다.

프로덕션 빌드에서 피트 시설과 성능 계측을 포함한 Three 렌더러 코드는 약 32.8KB, 지연 로드되는 `three-vendor` chunk는 약 512.7KB(압축 약 127.9KB)다. 비활성 Pixi 렌더러 chunk는 생성되지 않는다.

## 8. 바레인 검증 상태

현재 저장된 주요 검증 결과:

- 1x/2x 각 200초 런타임 soak 통과
- 3배속 제거 후 바레인 20대 2배속 1분 단독 측정에서 실효 1.999x, backlog p95 13.3ms, broadcast jitter p95 13.3ms를 확인
- trajectory 최대 50Hz 이동 1.77m, frame gap 1
- 바레인 2/3/4-wide 고정 스텝 분할 결정성 golden 통과
- 피트 진입부터 측면 합류까지 단일 50Hz 종단 golden 통과
- 직선 tow와 T1 dirty-air 수치 golden 통과
- 2026-07-27 과거 회귀 기록은 Backend 433개 테스트와 27개 subtest, frontend lint/build 통과로 별도 보존한다.
- 2026-07-30 최종 Backend discovery는 장기 열 기준, 동시 과열 peak, 누적 에너지, Race Setup 계약과 controller-scale cache 회귀를 포함해 474개 테스트와 30개 subtest, 1,089.495초, `OK`로 통과했다.
- 2026-07-30 최종 strict 1,700초 직접 엔진 열 회귀도 `OK`다.
- 같은 날 새 macOS 패키지의 Bahrain `HOT 36/52°C` 57랩·20대 수동 스트레스
  표본도 완주했다. 결과 차량 20대, 후륜 표면/코어 최고
  `132.867/121.450°C`, 최대 동시 130°C 초과 3대, 최대 연속 186.9초,
  surface/core guard hit 0회였고 종료 시 과열 차량은 0대였다.
- 위 Hot 표본은 수치 포화와 장기 열 폭주가 재발하지 않았음을 확인했다.
- 이후 Bahrain `NORMAL 30/40°C`, C1/C2/C3, 57랩·20대 수동 완주도 완료했다.
  다음 회로와 직접 비교할 수 있도록 해당 실행의 최종 진단 JSON과 패키지 버전은
  calibration 산출물로 계속 보존해야 한다.
- T1 비대칭 폭·연석 허용·접촉 응답 변경 후 관련 Backend 290개 테스트를 867.509초 동안 재실행해 통과
- 10개 seed·70개 시나리오의 2초 바레인 racecraft benchmark 통과: 판단·이유 일치율 100%, 접촉 0, off-track 0
- Three.js T1 경계 보간 변경 후 frontend lint와 production build 통과
- 27개 타이밍 루프·미니섹터 비교 관련 Backend 단위 테스트 5개, frontend lint/build 통과
- 별도 브라우저에서 바레인 20대 레이스를 실행해 S1/S2/S3와 타이밍 출처 전환을 확인. 현재 표기는 유효 루프 기준점이 없는 `~ EST`와 기준점을 따라 루프 사이에서도 갱신되는 `LIVE`로 구분한다.
- Live Timing 전체 20명 데이터 센터, 선택 차량 27개 타이밍 셀, 복원된 448px 캔버스 높이를 브라우저에서 확인
- 강제 SC 투입 후 피트 출동부터 본선 `IN THIS LAP`까지 3D 세이프티카·라벨·미니맵 마커를 확인했으며 브라우저 콘솔 오류는 0건
- SC 선제 감속·안전 상대속도 합류·목표 간격 유지·20대 대열 형성·전체 수명주기 관련 Backend 테스트 12개 통과
- SC 감속 영향의 중·후미 전파와 누적 600m 범위 제한, 20대 3랩 이내 대열 형성, 물리 수명주기 회귀를 추가 확인
- SC2 앞·뒤 피트아웃, 두 대 동시 피트아웃, 합류 뒤 순서 고정, 피트 전체 사이클, 후미 대열 제어·언랩·SC 수명주기 관련 Backend 표적 회귀 10개를 통과했다.
- 바레인 단독 SC catch-up 110초 진단에서 변경 전 `run-wide 27 / off-track 6` 샘플이 변경 후 모두 0이 됐다. 코너 목표속도 상한·wake 초기화·물리 주행과 기존 SC 대열·피트아웃·언랩·수명주기 표적 회귀 10개를 통과했다.
- 리타이어 pose 버퍼는 사고 위험 활성 중 유지되고 위험 해제 시 삭제되는 Node 검증을 통과했으며, 해당 Three.js·미니맵 정리 변경 뒤 frontend lint와 production build를 통과했다.
- 추월 뒤 뒤집힌 최신 루프 기록이 GAP/INT를 `0.000`으로 만들지 않는 회귀와 루프 사이 live 갱신 단위 테스트 통과
- Bahrain T1 코너 진입 거절 시 `overlap → yield → abort` 연속 전환, 종방향 선행 분리와 횡방향 smooth rejoin 직접 회귀 9개 및 코너·추월 영향 범위 34개 통과
- 비접촉 close-gap 사후 속도 복사 방지, Bahrain T1 바깥 공격 선택, 제동 전 one-move corridor 고정 회귀를 추가했다.
- 이번 추종·one-move·라인 선택 변경 뒤 `test_engine` 261개(607.523초)와 차량·트랙·표면·foundation 72개(45.971초), frontend lint/build를 재실행해 통과했다.
- 팀별 피트 박스·개러지와 차량 위 피트 시간 배지를 추가한 뒤 frontend lint/build를 통과했다. 브라우저 실주행에서 `PIT IN → STOP 2.4s / PIT 23.1s → PIT OUT` 전환, 60FPS와 콘솔 오류 0건을 확인했다.
- 세션 종료 수명주기 단위 테스트와 세션 생성·교체 회귀 46개를 통과했다. 브라우저에서 레이스를 연속 두 번 열고 닫아 각 레이스 중 WebGL canvas 1개·차량/SC 라벨 21개, 종료 뒤 canvas 0개·라벨 0개와 서버 `active_session=false`를 확인했다.
- Live Timing의 리타이어 차량은 흐린 이름 뒤 작은 가상 문구 대신 STS와 GAP 양쪽에 명시적인 빨간 `RET`를 표시하고, 추정 기호가 붙은 `~—`가 나오지 않도록 분리했다.
- 바레인 피트 제한을 80km/h로 변경하고 팀별 박스 10개를 16m 간격으로 배정했다. 새 회귀에서 같은 팀은 같은 박스, 서로 다른 팀은 고유 박스, 정차 pose는 개러지 방향 `box_offset=10m`, 전체 피트 경과는 35.62초였다. 기존 피트 연속 pose·프레임 분할 결정성 golden 4개와 신규/관련 피트 회귀 6개를 통과했으며, 브라우저에서 PER 차량이 팀 개러지 앞에 정차하는 동안 60FPS와 콘솔 오류 0건을 확인했다.
- 실제 브라우저 20대 장시간 검증에서 약 30분의 시뮬레이션 시간과 19랩을 확인했다. 60FPS·P95 16.8~18.6ms, JS heap 26~54MB 회수, canvas 1개 유지였으며 데이터 센터 모달 반복 개폐 뒤에도 장시간 정지는 없었다.
- 개발 전용 `dev_retire_driver`는 확률 이벤트를 우회하지 않고 실제 기계 결함 `Incident → CAR_STOPPED → hazard → SC` 수명주기를 사용한다. 실제 브라우저에서 위험 활성 중 빨간 미니맵 마커·정지 차량 라벨·RET 행을 확인했고, 후미 통과 뒤 차량·라벨·빨간 마커가 제거되며 RET 행만 유지되는 것을 확인했다.
- 실제 브라우저에서 `SC 전개 → pit window → VER 피트 호출 → 정차/타이어 교체 → 본선 차량 양보 → SC2 순위 확정 → 피트아웃 → SC IN → 재시작` 전체 흐름을 확인했다. 화면은 60FPS·P95 18.7ms·느린 프레임 0개였고, 수정 뒤 SC 전개와 복귀 동안 락업·트랙션 손실 이벤트는 없었다.
- 공격 라인 horizon의 중복 물리 샘플과 매 tick 반복 선택을 줄인 뒤 관련 레이스크래프트 회귀 11개를 통과했다.
- 2026-07-25 문제 상태를 실측했을 때 `race_state` 한 건은 50.8KB였고 10Hz로 반복됐으며, 한 차량이 랩을 마칠 때마다 20대 전체 누적 기록도 다시 전송했다. 이는 약 25분에 760MB 이상의 dashboard JSON 파싱을 만들 수 있어 Chrome renderer 1GB대 증가와 규모가 일치했다.
- 최종 스트림은 20대 기준 pose 약 1.7KB, dashboard 약 8.5KB, timing 약 8.0KB이며 기본 계산량은 약 90.5KiB/s다. 실제 브라우저에서는 2배속에서 약 51~56KB/s를 확인했다. 랩 기록은 드라이버별 `start_index` 이후 새 항목만 전송한다.
- 격리된 새 renderer의 20대 2배속 검증에서 10랩 동안 60FPS, P95 16.7~17.9ms, pose 버퍼 최대 2,540개, JS heap 26~50MB를 유지했다. renderer RSS는 초기 장면 로딩 뒤 약 366.6MB에서 474.1MB까지 증가했지만 종전 8랩 1.9GB와 같은 증가율은 재현되지 않았다. typed-array 링 버퍼 적용 뒤 별도 5랩은 약 326.5→395.5MB였으므로 남은 수치는 live JS pose 개수보다 Chromium/V8·WebGL 네이티브 예약량의 영향이 크다.
- 2026-07-27 재검증에서는 진단창을 `document.body` portal로 옮겨 Race Feed의 `backdrop-filter` containing block 때문에 하단이 잘리던 문제를 제거하고, 폭 540px의 우측 상단 패널로 축소했다. 20대 2배속 8랩 표본에서 JS heap은 24.7→41.5→29.6MB처럼 GC 회수됐고, 첫 바퀴 뒤 WebGL geometry 491·texture 13, pose 2,540, scene/garage/car build 2/1/20이 고정됐다. 개발 모드의 scene 2회는 React StrictMode 초기 검증이며 레이스 중 재생성은 없었다.
- typed-array pose 버퍼에 20대·10만 패킷을 연속 입력한 스트레스 테스트에서 차량당 127개, 총 2,540개가 유지됐고 최신 시각 보간도 정상 완료됐다.
- 백엔드의 매 50Hz 프레임 ID·step 간격 진단 기록과 모든 과거 timing crossing도 무한히 커지던 결함을 발견했다. 진단은 최근 256개, 원본 timing anchor는 최근 4개 완료 랩 범위로 제한하고 완료 랩 요약은 별도로 유지한다. `vmmap`에서 Python 프로세스의 시스템 malloc 실할당은 3~10랩 표본에서 약 20.5~25.7MB였고 추가 RSS 상당수는 재사용을 위해 보존된 `VM_ALLOCATE` 영역이었다.
- 차량별 예측 속도 캐시는 거리·연료 질량·타이어·브레이크 상태 조합을 키로 사용해 랩마다 늘어날 수 있었으므로 물리 인스턴스당 최근 256개 LRU 항목으로 제한했다. 고정 트랙 기하 표본 캐시는 트랙의 10m 구간 수만 유지하고, 전역 controller scale 캐시는 최근 128개 설정으로 제한한다.
- 20대 Bahrain 300초 격리 대조에서 speed 캐시 상한 2,048는 독립 항목 24,768개·RSS 증가 20.9MB·계산 61.36초였고, 상한 256은 8,237개·RSS 증가 11.6MB·계산 61.74초였다. RSS 증가를 9.3MB(약 45%) 줄이는 동안 계산 시간 증가는 약 0.6%였다.

성능과 보정 상세 자료는 `backend/data/calibration/`에 저장한다. 과거 기준선은 Git
기록에 남기고 현재 승인값은 최신 전체 실행만 사용한다. 2026-08-03 Backend 전체
discovery는 525개 테스트를 1,254.129초에 `OK`로 통과했다. Frontend는 12개 테스트,
lint와 production build, Desktop은 12개 테스트를 통과했다. Bahrain Normal 수동
완주는 완료했고 C4/C5 대표 회로 수동 검증은 별도 완료 조건으로 남아 있다.

## 8.1 레이싱라인 보정 Phase 1 상태 — 부분 완료 (2026-08-02)

[`FULL_MODE_CIRCUIT_CALIBRATION_STATUS.md`](FULL_MODE_CIRCUIT_CALIBRATION_STATUS.md)의
Bahrain(circuit 3)·Red Bull Ring(circuit 4) race/NORMAL/Medium/Standard/50Hz 범위를 기준으로 단독 baseline,
좌표 계측, 제한된 controller A/B와 회귀 검증을 진행했다. 기존 사용자 변경은
보존했고, qualifying line·전역 타이어/차량 물리·pit/SC/render 폭은 이번 작업에서
승인 대상으로 포함하지 않았다.

- `backend/tools/run_circuit_baseline.py`에 source/hash·branch, compiled-centerline와
  active-line path progress, telemetry progress 보정, centerline-relative/line-relative
  offset, pose·curvature·lateral speed·slip/lock 계측과 코너별 signed error 보고를
  추가했다. 반복 baseline은 동일 입력 3회 결과와 compact trace를 함께 보존한다.
- 기본 5랩 baseline 3회는 결정적이었다. Bahrain은 fastest `92.907s`, lateral
  p95/max `4.785/6.141m`, heading p95 `0.105rad`, over-2m `53.46s`; RBR은
  `66.990s`, `5.759/7.918m`, `0.105rad`, `54.40s`였다. 두 트랙 모두
  contact/off-track/track-limit/run-wide/planner-fallback/numeric guard는 0이었다.
- reference smoothing, max lateral slope, reference lateral-speed feed-forward와
  clean-line lateral response cap 후보를 A/B했다. 가장 나은 단독 수치도 Bahrain
  `p95 4.169m`, RBR `p95 4.374m`으로 승인 기준 `p95 ≤ 2m`, `max ≤ 4m`을 넘었고,
  RBR slope 후보에는 planner fallback 150~200샘플이 생겼다. feed-forward는
  Bahrain correction reversal 1, RBR 1~3으로 늘었고, 2.5m/s response cap은
  Bahrain p95 `6.632m`, RBR p95 `6.560m` 및 off-track 15샘플로 거부했다.
- 기본값은 유지했다. `CircuitPhysicsCalibration.racing_line_max_lateral_slope`와
  RaceEngine의 optional diagnostic knobs는 기본 동작을 바꾸지 않으며,
  `data/circuits.json`에 승인된 per-circuit 보정값은 아직 기록하지 않았다.
- 15랩 단독 stint는 두 트랙 모두 완주했지만 RBR에서 planner-fallback 300샘플과
  correction reversal 8이 발생해 승인하지 않았다. Bahrain peak rear surface/core는
  `112.266/102.414°C`, RBR은 `102.702/94.295°C`; 두 결과 모두 clamp와 연속 과열은
  0이었다.
- 기존 20-car session harness를 circuit 3/4로 일반화했다. 1x/2x는 Bahrain
  effective `0.987/1.955x`, RBR `0.983/1.919x`로 PASS했고, 각 실행의 contact,
  track-limit, planner-fallback은 0이었다. 최소 10랩 direct traffic harness도
  추가했지만 계산 비용 때문에 이번 실행에서는 중단했으므로 10랩 traffic approval은
  미완료로 남긴다.
- coordinate contract 단위 테스트 8개와 지시문 지정 회귀 86개가 통과했다.
  track validation/audit는 두 트랙 모두 통과했고, `git diff --check`는 최종 보고
  전에 다시 실행한다.

현재 승인·실패 근거는 [`FULL_MODE_CIRCUIT_CALIBRATION_STATUS.md`](FULL_MODE_CIRCUIT_CALIBRATION_STATUS.md)에 통합했고,
원본 JSON은 `backend/data/calibration/`의 `*_race_line_baseline_v1.json`,
`*_race_line_stint15_v1.json`, `*_race_line_traffic_1x2_v1.json`에 보존한다.
현재 결론은 “계측·baseline·부분 회귀 완료, 승인 가능한 레이싱라인 보정값 없음”이며,
다음 단계는 reference path/pose 정렬을 더 좁은 fixture로 분리한 뒤에만 회로별
calibration을 재시도하는 것이다.

2026-08-02 실패 리뷰 뒤 foundation 보정을 추가했다. 첫 A/B가 reference 횡속도만
전달하고 target 횡속도를 0으로 둬 상대 프레임에서 의도한 transport를 상쇄하던
계약을 수정하고, 진단에서는 두 속도에 같은 active-line offset 미분값을 전달한다.
다만 이 후보를 20대 제품 기본값으로 활성화했을 때 기존 SC 대열 회귀 두 건이
발생했으므로 제품 기본값은 `0`으로 되돌리고 진단 CLI에서만 `1`을 사용한다.

진단 랩 수와 연료 기준도 분리했다. 새 5랩 3회 결과는 Bahrain 57랩 기준
`108.031kg`, RBR 71랩 기준 `107.364kg`으로 시작하며, 각각 lateral p95/max
`4.410/6.057m`, `5.652/7.942m`다. Bahrain은 이전 저연료 기준 p95 `4.785m`보다
개선됐지만 두 회로 모두 승인 기준에 미달하고, heading 0.105rad 상한 누적은
각각 `64.46s`, `57.54s`다. RBR은 실행당 planner fallback 250샘플도 남았다.
따라서 코드 계약·연료·reversal·heading/fallback 진단은 보완됐지만 제품
레이싱라인은 여전히 미승인이다. 새 산출물은
`bahrain_race_line_foundation_v2.json`과
`red_bull_ring_race_line_foundation_v2.json`이다. 관련 회귀 90개와 새 계약 때문에
영향받았던 SC 테스트 2개는 통과했다.

### 8.2 동적 레이싱라인 추종 기반 보정 — 부분 승인 (2026-08-02)

일정 곡률 fixture에서 고속·고다운포스 상태의 총 횡력 여유와 무관하게 전·후축
cornering stiffness가 정적 하중값으로 고정되는 원인을 재현했다. 이 때문에 실제
그립 상한에 도달하기 전에도 큰 slip/body angle이 필요했고 heading 0.105rad guard에
걸려 경로 복귀가 제한됐다.

- `vehicle_dynamics.py`에 수직하중 용량 대비 정적하중 용량의 0.65승(최대 2.5배)을
  사용하는 load-sensitive cornering stiffness를 추가했다. 총 타이어 횡력 상한은
  변경하지 않았다.
- 제동·구동 후 남은 combined-slip 횡력과 명목 타이어 하중 용량을 분리해, 제동
  순간에 수직하중 기반 stiffness까지 사라지지 않도록 했다.
- 55m/s·곡률 0.014/m·10초 일정 곡률 회귀를 추가했다.
- 새 하중 응답을 적용한 뒤 기존 SC 대열 형성과 6대 물리 순위 역전 복구를 포함한
  계약 테스트 14개가 통과해 paired target/reference line transport 기본값을 `1`로
  승인했다. 명시적 `0` legacy replay는 유지한다.
- 5랩×3회 full-race-fuel 산출물을 갱신했다. Bahrain은 p95/max
  `4.410/6.057 → 3.733/4.883m`, RBR은 `5.652/7.942 → 5.488/7.582m`다.
  두 회로 모두 contact/off-track/track-limit/run-wide/numeric guard/열 clamp는 0이고
  반복 결과는 동일하다. rear surface/core peak는 Bahrain `107.917/94.423°C`,
  RBR `103.170/91.666°C`다.

전역 차량 모델 수정은 승인하지만 회로별 레이싱라인 승인은 아직 아니다. Bahrain도
`p95 ≤ 2m, max ≤ 4m`에 미달하고, RBR은 T6–T7·T9–T10의 급격한 측정 기준선과
실행당 planner fallback 250샘플이 남는다. RBR reference 25% 축소, clean-line 횡속도
상한 증가, 전역 downforce grip 증가, 전체 목표속도 감속, yaw relaxation 감소 후보는
오프 트랙·fallback·전역 물리 왜곡·랩타임 손실 중 하나가 있어 적용하지 않았다.
후속 교통 안전 작업에서 Bahrain/RBR의 급격한 기준선 전환을 제한하는
`racing_line_max_lateral_slope`와 RBR `nominal_line_edge_buffer_m`는 채택했다.
이는 차선 연속성과 차체 경계 보정이며 엄격한 단독 레이싱라인 승인을 뜻하지 않는다.

최종 검증은 경로/트랙/trajectory/vehicle/telemetry와 SC 순위 복구 2건을 포함한
96개 테스트 `OK`(105.836초), Bahrain 20대·1,700초 장기 열 통합 `OK`
(513.766초), 회로별 5랩×3 결정성 `pass`, Python compile과 `git diff --check`
`OK`다. 장기 열 golden은 이후 Bahrain lateral-slope 연속성 보정까지 반영해
후륜 표면/코어 `110.834/97.999°C`로 갱신했으며 과열·연속 과열·clamp는 모두 0이다.

## 9. 남아 있는 핵심 한계

- 실제 F1 리플레이 분포에 맞춘 공격 시도·성공·철회·접촉·forced-wide 빈도 보정이 필요하다.
- Bahrain T1/T2 코너 최저속도 오차와 세션별 노면·기온·바람 차이가 남아 있다.
- 타이어 압력, 네 바퀴별 수직하중·회전 관성·독립 열/마모, 서스펜션·롤·피치는 아직 없다. 타이어 열·하중·슬립·브레이크 열은 전·후축 평균 모델이며 네 바퀴 독립 모델은 아니다.
- 등록된 9개 서킷은 모두 수작업 `surface_zones`가 비어 있다. 곡률로 생성한 양쪽 연석만 전부 `low`로 사용하므로 실제 높은 연석 위치 검증과 명시적 표면 데이터 입력이 필요하다.
- Silverstone·Spa·Hungaroring은 실제 좌우 경계·연석·표면 데이터가 부족하다.
- 팀별 직선·저속·고속·제동 특성과 컴파운드별 스틴트 길이 벤치마크가 부족하다.
- 사용자가 확인한 기존 전체 snapshot 스트림의 Chrome renderer private footprint 약 1.9GB와 이후 52랩 탭 표시 2.7GB는 정상 목표보다 높다. 반복 대형 JSON·전체 랩 기록·고빈도 포즈 객체 생성을 제거한 5~10랩 검증에서는 같은 증가율이 재현되지 않았고, 52랩 직후 별도 renderer 표본은 physical footprint 354MB·peak 473MB였으므로 단일 숫자만으로 지속 누수를 확정하지 않는다. 새 5분 JS heap 추세·DOM·WebGL 자원 계측으로 전체 레이스 증가 기울기를 다시 승인해야 한다.
- 화면에 표시하는 JS heap은 페이지의 JavaScript 객체만 포함한다. Chrome 탭 hover나 Chrome 작업 관리자의 memory footprint는 DOM, 렌더러 네이티브 메모리와 일부 그래픽 자원까지 포함하는 현재 OS 점유량이며 누적 사용량이 아니다. 두 숫자는 직접 비교하지 않는다.
- Three.js 차량과 피트 개러지는 기능 검증용 low-poly 모델이며 장벽·관중석·피트 크루·고품질 재질은 아직 추가 구현이 필요하다.
- 2026 하이브리드 파워유닛, 에너지 관리, 액티브 에어로, 날씨와 젖은 노면은 범위 밖이다.

## 10. 다음 권장 순서

1. 현재 `FULL` 변경과 문서·전체 회귀를 하나의 기준 commit으로 고정한다.
2. [`ABSTRACT_RACE_SIMULATION_DESIGN.md`](ABSTRACT_RACE_SIMULATION_DESIGN.md)의 단계 A를
   별도 브랜치와 `backend/simulation/abstract/` 패키지에서 시작한다. 기존
   `RaceEngine` private 메서드를 재사용하지 않는다.
3. `FULL` 모드는 비교·회귀 기준으로 유지한다. 남은 Bahrain/RBR 단독 레이싱라인은
   [`FULL_MODE_CIRCUIT_CALIBRATION_STATUS.md`](FULL_MODE_CIRCUIT_CALIBRATION_STATUS.md)의
   오차 게이트를 완화하지 않고 별도 보정한다.
4. C4/C5와 출발 90°C/피트 70°C는 각각
   [`TIRE_COMPOUND_SPEC.md`](TIRE_COMPOUND_SPEC.md),
   [`COLD_OUTLAP_CONTROL_DESIGN.md`](COLD_OUTLAP_CONTROL_DESIGN.md)에 남은 `FULL` 작업이다.
5. 새 회로는 `형상 무결성 → 단독 안정 랩 → 실제 랩/속도 비교 → 20대 교통 →
   피트/SC → 타이어·컴파운드 → 메모리/결정성` 순서와
   [`ADDING_REAL_CIRCUITS.md`](ADDING_REAL_CIRCUITS.md)를 따른다.
6. 장벽·관중석·고품질 그래픽과 2026 파워유닛·에너지·액티브 에어로는 두 결과
   엔진의 권위 계약과 분리된 후속 작업으로 유지한다.

## 11. 주요 코드 위치

### Backend

- `backend/session.py`: 고정 스텝 세션 루프, 바이너리 pose 30Hz·dashboard 4Hz·timing 1Hz·증분 history/event 전송
- `backend/simulation/race_engine.py`: 레이스 상태, AI·피트·타이밍·규칙 통합
- `backend/simulation/vehicle_physics.py`: 종방향 힘과 저수준 차량 제어
- `backend/simulation/vehicle_dynamics.py`: dynamic bicycle model
- `backend/simulation/global_trajectory_optimizer.py`: 전역 궤적 최적화
- `backend/simulation/local_trajectory_planner.py`: 교통·추월·회피 후보 경로
- `backend/simulation/planner_scheduler.py`: 상황별 플래너 주기와 캐시
- `backend/simulation/collision.py`: OBB/SAT와 swept collision
- `backend/simulation/wake_model.py`: tow와 dirty air
- `backend/simulation/track_surface.py`: 경계·연석·런오프 표면
- `backend/simulation/track_data_validation.py`: 트랙 데이터 계보와 승인 검사

### Frontend

- `frontend/src/components/TrackView/TrackCanvas.jsx`: 비활성 PixiJS 롤백용 렌더러
- `frontend/src/components/TrackView/ThreeTrackCanvas.jsx`: 현재 활성 Three.js 트랙과 차량 렌더링
- `frontend/src/components/TrackView/posePlayback.js`: Pixi/Three 공용 50Hz pose 재생·보간
- `frontend/src/hooks/useRaceWebSocket.js`: pose·dashboard·history·event 스트림 병합과 React 상태 수명주기
- `frontend/src/components/Dashboard/TimingBoard.jsx`: GAP/INT와 섹터 타이밍
- `frontend/src/components/Dashboard/SectorTimingPanel.jsx`: 선택 차량의 3개 대섹터와 27개 실시간 타이밍 조각
- `frontend/src/components/Dashboard/DriverDataCenter.jsx`: 모든 드라이버의 섹터·랩·실시간 상태 모달
- `frontend/src/components/Dashboard/StrategyPanel.jsx`: 타이어·페이스·피트 제어

## 12. 실행과 검증

Backend 전체 테스트:

```bash
cd backend
.venv/bin/python -m unittest discover -s tests
```

바레인 레이스크래프트 벤치마크:

```bash
cd backend
.venv/bin/python tools/run_bahrain_racecraft_benchmark.py --seeds 10 --seconds 2
```

바레인 런타임 soak:

```bash
cd backend
.venv/bin/python tools/run_bahrain_runtime_soak.py --help
```

Frontend 검증:

```bash
cd frontend
npm run lint
npm run build
```

새 실제 서킷 추가 절차는 [`ADDING_REAL_CIRCUITS.md`](ADDING_REAL_CIRCUITS.md)를 따른다.

## 13. Bahrain·Red Bull Ring 레이스 교통 재승인 (2026-08-03)

두 회로의 20대 교통에서 순위와 실제 차체 위치가 달라진 뒤 잘못된 차량을 추종하거나,
안전한 로컬 경로가 없는데도 `inside/outside` 전역 라인을 임시 명령으로 만들어 차선을
가로지르는 원인을 수정했다.

- 안전한 로컬 후보가 없으면 합성 tactical line을 만들지 않고 50Hz 종방향 추종 제어로
  대기한다. 이 정상 상태는 `planner_mode=traffic_hold`로 기록하며 실제 planner failure와
  분리한다.
- 녹색기에는 타이밍 순위가 아니라 현재·1.5초 예측·목표 횡좌표가 같은 물리적 차로로
  수렴하는 차량들을 검색한다. 후보가 여러 대면 가장 가까운 차가 아니라 요구 안전속도가
  가장 낮은 차를 추종한다.
- faster rear car 책임 예외는 레이싱 라인을 그대로 유지하는 후보에만 적용한다. 횡이동
  후보는 뒤차를 포함한 충돌 검사를 반드시 통과해야 한다.
- 이미 승인된 추월 쌍 주변 35m 안에 제3의 차량이 있으면 새 추월을 예약하지 않는다.
  서로 다른 두 차로가 종방향으로 겹친 상태에서 하나의 목표 차로로 합류하려 하면 기존
  물리적 측면을 유지한다.
- 새 추월은 직선에서 로컬 궤적으로 승인해야 한다. 제동이 시작된 뒤 새 tactical lane은
  생성하지 않으며, 기존에 직선에서 성립한 side-by-side만 코너로 이어간다. 압박에 따른
  제동 실수는 별도 입력 오류로 계속 발생할 수 있다.
- 출발 그리드 두 열은 고정 35m 뒤에 동시에 합류하지 않는다. 회로 데이터의 첫 코너
  종료점까지 열을 유지하고, 이후 120m 동안 합류한다. RBR의 계산값은 hold `483.39m`,
  merge complete `603.39m`다.
- 차량 yaw를 반영한 종·횡 차체 투영 폭을 추종·추월 안전 간격에 사용하고, 합법 간격에서
  시작한 추종 스텝은 최소 간격 경계를 넘지 않도록 한다. 속도는 선행차 값으로 순간 복사하지
  않고 타이어 제동 한계에 따라 연속적으로 감소한다.
- SC/VSC, DRS 재개, 피트 출구 progress 분리, stale local-plan 제거, 충돌 이벤트 진단
  payload도 같은 변경 묶음에 포함된다.

결정론적 10랩·20대·50Hz 제품 스타트 검증 결과는 다음과 같다.

| 항목 | Bahrain | Red Bull Ring |
|---|---:|---:|
| 판정 | pass | pass |
| 완주/은퇴 | 20/0 | 20/0 |
| contact/off-track/track-limit | 0/0/0 | 0/0/0 |
| planner fallback | 0 | 0 |
| 정상 traffic hold samples | 145 | 169 |
| 최대 횡이동/step | 0.08m | 0.08m |
| 최대 tactical rejoin | 17.12s | 14.94s |
| 후륜 surface/core peak | 111.137/98.782°C | 99.985/92.214°C |
| 과열/thermal clamp | 0/0 | 0/0 |

일회성 원본 산출물은 로컬 임시 경로에만 생성하고 저장소에는 커밋하지 않는다.
최종 Backend 전체 discovery는 525개 테스트, 1,254.129초, `OK`다. Bahrain
racecraft v2는 10 seeds·70 scenarios에서 결정·이유 일치율 100%, 접촉·이탈 0,
피트 합류 순서 100%를 확인했다. Frontend 12개·lint·production build와 Desktop
12개 테스트도 통과했다.

이 승인은 20대 교통 안전성과 열 안정성에 대한 것이다. 단독 기준선에서 남아 있는 엄격한
레이싱라인 오차 목표(Bahrain/RBR 일부 구간의 p95·max·heading)는 별도 회로 형상/기준선
보정 항목이며, 이번 교통 안전 수정으로 완료 처리하지 않는다.
