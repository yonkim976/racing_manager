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
2. [`ABSTRACT_RACE_SIMULATION_DESIGN.md`](ABSTRACT_RACE_SIMULATION_DESIGN.md)의 단계 A~C를
   별도 브랜치의 `backend/simulation/abstract/` 패키지에서 구현 완료했다. 기존
   `RaceEngine` private 메서드를 재사용하지 않으며, API·WebSocket·frontend에는 아직
   연결하지 않는다. FULL은 계속 비교·회귀 기준으로 유지한다.
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
- `backend/simulation/abstract/`: FULL과 분리된 단계 A~C 결과 엔진, snapshot, RNG, 성능·퀄리파잉·pose·교통

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

## 14. ABSTRACT 단계 A — 결과 엔진 상태 — 완료 (2026-08-03)

`ABSTRACT_RACE_SIMULATION_DESIGN.md`의 단계 A 범위인 결과 전용 엔진 기반을 별도
패키지로 구현했다. 이 단계는 `FULL` `RaceEngine`을 수정·상속·호출하지 않으며,
현재 session factory, API, WebSocket, frontend와도 연결하지 않는다. 모든 성능 수치는
공식 F1 물성이나 실측값이 아닌 `game_calibration_provisional` 게임 보정값이다.

### 공개 인터페이스와 입력 snapshot

- `AbstractSessionSnapshot.from_content(...)`가 검증된 `Circuit`·`Driver`·`Team`·타이어
  지명·`TrackConditions`를 실행 시작 시 불변 snapshot으로 고정한다. 참가자와 차량은
  stable ID로 정렬하고 FULL 런타임 mutable state를 공유하지 않는다.
- `AbstractRaceEngine(snapshot).run_qualifying()`가 단계 A의 유일한 실행 진입점이다.
  `QualifyingEngine`과 `run_abstract_qualifying(snapshot)`도 테스트·도구용 공개 진입점으로
  제공한다.
- `AbstractQualifyingResult`는 session/version identity, 드라이버별 best lap·S1/S2/S3,
  모든 run 기록, Q1/Q2/Q3 진출·탈락 요약, 최종 grid, 논리 이벤트와
  `canonical_result_hash`를 제공한다. `canonical_json()`/`from_dict()` 왕복 후 hash가
  유지된다. hash는 자기 자신의 필드를 제외한 canonical payload에 SHA-256을 적용한다.

### RNG stream과 성능 모델

- `IndependentRNG`는 `master_seed + stream_name`을 SHA-256 domain separator로 파생한다.
  Python 전역 `random` 상태와 `hash()`는 사용하지 않는다. `session:weather`,
  `session:track_evolution`, `driver:{id}:pace`, `driver:{id}:mistake`,
  `team:{id}:pit`, `vehicle:{id}:reliability`, `presentation:{event_id}`를 별도
  stream으로 유지한다. presentation 호출은 논리 결과 stream을 소비하지 않는다.
- 차량 축은 `power`, `drag_efficiency`, `high_speed_aero`, `medium_speed_aero`,
  `low_speed_grip`, `braking`, `traction`, `tire_management`, `cooling`,
  `reliability`를 독립 유지한다. `overall_rating`으로 합치지 않으며,
  reliability는 정상 랩타임 계산에서 제외하고 후속 결함 계층용으로 예약했다.
- 기본 segment 가중치는 straight=`power/drag_efficiency`, heavy braking=`braking`,
  traction=`traction/low_speed_grip`, sweeping=`high_speed_aero`,
  technical=`medium_speed_aero/low_speed_grip` 중심이다. tire·환경·트랙 에볼루션과
  driver pace는 별도 factor로 적용하고 seed는 정상 sector 편차·구체적인 실수에만
  사용한다.

### 퀄리파잉 계산과 검증

Q1 20→15, Q2 15→10, Q3 10명의 순서로 각 run의 `run_departed → out_lap_completed →
S1/S2/S3 → flying_lap_completed`를 논리 clock에 기록한다. 각 flying lap은 차량·driver·
tire·track evolution·교통 penalty를 먼저 계산하고, pace/mistake stream으로 정상
sector 편차와 구체적인 실수만 추가한 뒤 session ranking과 grid를 확정한다. 한 번의
무작위 최종 순위 추첨은 사용하지 않는다.

검증 결과:

- 새 `backend/tests/test_abstract_race_simulation.py` 12개: `OK`. 동일 입력·seed 100회
  결과/hash 완전 동일, 입력 순서 독립, presentation/pace stream 격리, Q1/Q2/Q3
  탈락·grid 무결성, canonical JSON 재로딩을 확인했다.
- Bahrain seed 42 표본: 20 drivers, 90 run records, 578 logical events.
- 업그레이드 A/B: power `+0.08`은 Start/Finish Straight를 `6.152943s → 6.136693s`,
  braking `+0.08`은 T1 Braking을 `8.211196s → 8.182781s`로 개선했다. Red Bull Ring
  Rindt Sweep의 high-speed aero `+0.10` 개선은 `0.008622s`이고 Main Straight 영향은
  `0.000000s`였다.
- 기존 관련 FULL 회귀: Qualifying/RaceSetup 59개, foundation·tire nomination 42개,
  모두 `OK`.
- Backend 전체 discovery: **537개, 1,223.888초, `OK`**. 기존 525개 기준에 단계 A
  테스트 12개가 추가된 수치다.
- 당시 단계 A 기준의 `git diff --check`, Python compile과 문서 링크 확인은 `OK`였고,
  단계 C 추가 후에도 동일 검사를 다시 `OK`로 통과했다.

### 알려진 한계와 단계 B 판정

현재 단계는 결과 전용 퀄리파잉 수직 실험이다. 레이스 tick, 추월·사고·피트·SC,
reliability 사건 처리, pose synthesis, renderer 연결, 실제 제품 mode 선택은 구현하지
않았다. 차량·트랙·타이어 balance는 provisional이며 FULL 물리값의 저정밀 대체가 아니다.

단계 A의 범위·결정성·A/B 승인 기준은 충족했으므로 **단계 B(단일 차량 pose 합성)로
넘어가도 된다**. 단, 단계 B도 별도 모듈·별도 테스트로 시작하고 API/frontend 기본
모드 변경은 단계 E 판정 전까지 보류한다.

## 15. ABSTRACT 단계 B — 단일 차량 pose 합성 상태 — 완료 (2026-08-03)

단계 A 결과와 독립적인 단일 차량 표시 pose 합성을
[`backend/simulation/abstract/pose.py`](../backend/simulation/abstract/pose.py)에
추가했다. 결과 엔진이나 FULL 물리 상태를 변경하지 않으며, API·WebSocket·frontend
연결도 아직 하지 않는다.

- `AbstractTrackSnapshot`이 검증된 `Circuit.track_coords`를 immutable centerline
  snapshot으로 보존한다. `TrackSpline`은 폐곡선 Catmull–Rom과 arc-length table을
  사용해 normalized progress를 centerline 위치·tangent·normal·heading으로 변환한다.
- `SingleVehiclePoseSynthesizer.pose_at()`은
  `world_position = center + normal × lateral_offset` 계약으로 위치를 만들고,
  `AbstractPoseFrame`에는 timestamp, driver ID, progress, lap, track distance,
  lateral offset, world position, heading, visual state와 `source_mode=abstract`만
  제공한다. 슬립각·힘·열·가짜 물리값은 넣지 않는다.
- `sequence()`는 `LogicalClock`의 0.10초 tick을 사용해 wall time과 game time을
  분리한다. 1x/2x/5x는 같은 logical time에서 같은 pose를 만들며, renderer 없이도
  deterministic tuple을 반환한다.
- Stage B 테스트 4개를 포함한 추상 테스트 **16개 `OK`**. 폐곡선 progress wrap,
  횡 오프셋 normal 적용, 1x/5x logical pose 동일성, 5x 연속성과 재현성을 확인했다.
- 관련 FULL 회귀는 Qualifying/RaceSetup **59개**, foundation·tire nomination
  **42개**가 모두 `OK`다.

현재 Stage B는 단일 차량 pose 계약만 승인했다. 20대 간격·2-wide corridor·추월,
사건 연출·renderer 연결은 포함하지 않는다. 따라서 **단계 C(20대 교통·추월)로
진행해도 되지만**, 제품 mode/API 연결은 계속 단계 E 판정까지 보류한다.

## 16. ABSTRACT 단계 C — 20대 교통·추월 상태 — 완료 (2026-08-03)

단계 B의 단일 차량 pose 위에 결과 전용 20대 traffic tick과 논리 추월 계층을
[`backend/simulation/abstract/race.py`](../backend/simulation/abstract/race.py) 및
[`backend/simulation/abstract/racecraft.py`](../backend/simulation/abstract/racecraft.py)에
추가했다. FULL `RaceEngine`의 private 메서드·mutable state를 호출하거나 상속하지 않으며,
API·WebSocket·frontend에도 연결하지 않았다.

### 공개 인터페이스와 race 상태

- `AbstractRaceEngine(snapshot).run_race(...)`와 `run_abstract_race(snapshot, ...)`가
  `grid_order`, lap 수, 논리 tick 간격을 입력받는다. 기본 tick은 0.10초이며 wall-clock과
  무관한 `LogicalClock`으로만 진행한다.
- `AbstractRaceResult`는 session/version identity, immutable grid/finish order, 매 tick의
  `RaceVehicleState`, attack/defend/overtake 논리 이벤트와 canonical SHA-256 hash를
  반환한다. `canonical_json()`/`from_dict()` 왕복 뒤 hash와 JSON이 같다.
- `RaceVehicleState`에는 순위·total progress·sector·gap/interval·ahead ID·최소 pose·
  `visual_state`/`maneuver`/corridor만 포함한다. slip angle, force, 축별 열수지 등 정밀
  물리값은 생성하지 않는다.

### 교통·추월 모델

- 일반 차량은 최소 5.0m longitudinal gap으로 제한하고, 추월 시에는 트랙 폭이
  `2 × 1.9m vehicle width + 0.75m margin = 4.55m` 이상인 2-wide corridor만 허용한다.
- attack window는 24m, closing speed는 0.15m/s 초과이며 straight/heavy braking/traction
  구간만 공격 후보로 본다. 공격·방어 성능은 해당 segment의 차량 축과 driver overtaking/
  defending 축을 사용하고, 결과는 `attack_started → overtake_completed/defense_hold`
  (또는 abort) timeline으로 기록한다.
- 한 차량은 동시에 하나의 corridor만 점유한다. 정상 pace는 차량·driver·타이어·segment
  요구 가중치가 결정하고, seed는 독립 pace/mistake stream의 정상 편차와 공격 판정에만
  사용한다. 사고·lockup·contact·pit·SC는 다음 단계 범위다.

검증 결과:

- 새 추상 테스트 파일 **22개, 42.077초, `OK`**. 동일 입력·seed 100회 race 결과와
  canonical hash 완전 동일, 입력 순서 독립, presentation stream 격리, 최소 gap·2-wide
  corridor·rank/display sync, attack/defend timeline, finish 중복·누락 방지,
  canonical JSON 재로딩을 확인했다.
- Bahrain seed 42, reverse grid, 1 lap/0.5초 tick 표본: 172 frames, attack 65회,
  defense hold 35회, overtake completed 30회, 20명 finish, 결과 hash
  `4fb550bb9cf3e8d0dbea547fe70b05a91d0abea89918fffd8417afa653a18c4c`.
- 업그레이드 A/B는 provisional segment 모델 방향과 일치했다. power `+0.08`의 straight
  `6.152942576s → 6.136693007s`, braking `+0.08`의 heavy-braking
  `8.211196225s → 8.182781475s` 개선을 확인했다. reliability 변경은 정상 segment time을
  바꾸지 않는다.
- 기존 FULL 관련 회귀: **118개, 556.019초, `OK`**.
- Backend 전체 unittest discovery: **547개, 1,277.015초, `OK`**.
- `git diff --check`, Python compile과 문서 링크 대상 파일 확인은 모두 `OK`다.

### 알려진 한계와 단계 D 판정

현재 단계는 결과 전용 traffic/추월 수직 실험이다. 날씨 변화·타이어 wear·reliability 사건,
사고·lockup·contact·피트·SC, renderer 합성 고도화와 API/product mode 선택은 구현하지 않았다.
공격 확률·차량 축 balance·corridor margin은 모두 첫 단계의 provisional 게임 보정값이며
공식 F1 물성이나 실측값이 아니다.

단계 C의 독립 상태·결정성·2-wide/추월 timeline 기준은 충족했다. 따라서 **단계 D
(lockup/contact/pit 계층)로 넘어가도 된다**. 단, API/frontend 연결과 FULL 기본 모드 변경은
단계 E의 제품 판정 전까지 계속 보류한다.

## 17. ABSTRACT 단계 D — 락업·접촉·피트 상태 — 완료 (2026-08-03)

단계 C race tick 위에 결과 전용 락업·경미한 접촉·피트 계층을 추가했다. 구현은
[`backend/simulation/abstract/incidents.py`](../backend/simulation/abstract/incidents.py)와
[`backend/simulation/abstract/pit.py`](../backend/simulation/abstract/pit.py)에 분리했으며,
race orchestration은 [`backend/simulation/abstract/race.py`](../backend/simulation/abstract/race.py)에
남겼다. `FULL` `RaceEngine`·API·WebSocket·frontend는 변경하지 않았다.

### 공개 인터페이스와 상태

- `AbstractRaceEngine(snapshot).run_race(...)`와 `run_abstract_race(...)`는 기존 입력에
  `pit_strategy: Mapping[driver_id, Sequence[stop_lap]]`를 선택적으로 받는다. 명시 전략이
  없으면 5랩 이상 경기에서 결정적인 1회 pit plan을 만든다.
- `AbstractRaceResult`의 `RaceVehicleState`는 physical compound/role, stint lap, wear와
  temperature band, pit phase/progress/count, damage level, incident state를 직렬화한다.
  이는 게임 상태·표시 계약이며 실제 슬립각·힘·축별 열수지·정밀 타이어 물리값이 아니다.
- `LockupAssessment`와 `ContactAssessment`는 `heavy_braking` 및 이미 승인된 2-wide
  corridor에서만 논리 사건을 생성한다. contact payload에는 `minor_side_contact` template
  ID를 기록하지만 renderer 연결은 아직 하지 않는다.
- pit timeline은 `pit_requested → pit_lane_entry → pit_stop_started → pit_stop_completed
  → pit_exit`로 고정하고, stop 중에는 logical speed factor만 적용한다. pit service time과
  replacement compound는 팀 pit stream과 snapshot 타이어에서 파생한다.

### 결정성·품질 검증

- 단계 D를 포함한 추상 테스트 **26개, 85.492초, `OK`**. 단계 D 테스트는 10랩 완주 및
  동일 hash 재실행, pit timeline/타이어 상태, lockup/contact logical payload와 damage
  범위, canonical JSON 재로딩을 확인했다.
- Bahrain seed 42의 10랩/0.5초 tick 표본은 **20대 finish, 1,233 logical events**이며
  `race_started`/`race_finished`, 20회 pit sequence, lockup·contact sequence를 모두
  확인했다. 제한 stress seeds 0~4도 10랩에서 각각 20대 완주했다.
- 기존 관련 FULL 회귀 **101개**(Qualifying 3, RaceSetup 56, foundation/tire nomination
  42)가 `OK`다. Backend 전체 unittest discovery는 **551개, 1,287.550초, `OK`**다.
- 단계 C에서 드러난 segment decimal boundary와 race-finish 직전 pending corridor 문제를
  수정했다. boundary는 epsilon 기준으로 다음 segment를 선택하고, 종료 tick의 미완료
  attack는 `attack_aborted(reason_code=race_finished)`로 닫는다. 기존 단계 C 테스트는
  tolerance를 완화하지 않고 다시 `OK`다.
- `git diff --check`, abstract package Python compile, 문서 링크 대상 확인은 모두 `OK`다.

### 알려진 한계와 단계 E 판정

현재 단계 D는 10랩 고정 건조 prototype의 논리 사건 계층이다. SC, 복합 사고, retirement와
reliability 결함, renderer/template 실제 연결, Instant 50랩 성능 승인, API/frontend 제품
mode 선택은 구현하지 않았다. 모든 사건 확률·damage·pit service time·성능 balance는
공식 F1 물성이나 실측값이 아닌 provisional 게임 보정값이다.

단계 D의 10랩 결정성·완주·사건 timeline·품질 기준과 전체 Backend 회귀는 충족했다.
따라서 코드 기준으로는 **단계 E(제품 결정) 검토로 넘어가도 된다**. 다만 단계 E에서
`ABSTRACT` 기본 채택 또는 두 모드 제공을 결정하기 전까지 API/frontend 연결과 `FULL` 기본
모드 변경은 계속 보류한다.

## 18. ABSTRACT 단계 E — 제품 mode 선택과 결과 연결 — 완료 (2026-08-03)

첫 랜딩의 race setup에서 엔진을 선택할 수 있도록 연결했다. 기존 `FULL`은 기본값이자
기존 경로이며, [`backend/simulation/abstract/`](../backend/simulation/abstract/)의 결과
엔진은 `ABSTRACT`를 명시적으로 선택했을 때만 실행된다. API 계약에는 `simulation_mode`와
`session_seed`를 추가했지만, 현재 API 기본 seed는 재현 가능한 데모를 위한 `42`이며 향후
세션 생성 정책에서 주입할 수 있다.

### 연결 범위

- [`backend/main.py`](../backend/main.py)는 `QualifyingRequest`와 `RaceSetupRequest`의
  `ABSTRACT`를 immutable snapshot으로 변환하고, abstract qualifying → grid → 결과 전용
  race 순서로 계산한다. 응답에는 version identity, seed, grid/finish order, 이벤트 요약,
  canonical result hash가 포함된다.
- `FULL` setup은 기존 `session_manager.create_session`과 WebSocket/live dashboard를
  그대로 사용한다. `RaceEngine`의 private 메서드나 mutable state를 abstract 경로와
  공유하지 않으며, 기존 FULL 기본 동작을 바꾸지 않았다.
- [`frontend/src/components/RaceSetup.jsx`](../frontend/src/components/RaceSetup.jsx)는
  `FULL · Physics`와 `ABSTRACT · Result`를 명시적인 선택지로 제공한다. ABSTRACT는
  [`frontend/src/components/AbstractRaceResult.jsx`](../frontend/src/components/AbstractRaceResult.jsx)
  에서 최종 분류·논리 이벤트·hash를 보여주며, 아직 WebSocket/renderer/라이브 조작에는
  연결하지 않는다.

### 검증 결과

- frontend contract/node tests **14개 `OK`**, lint `OK`, Vite production build `OK`.
- API mode routing tests **5개 `OK`**. ABSTRACT qualifying/setup 응답, grid 중복·누락,
  결과 hash와 FULL 경로 기본값을 확인했다.
- 현재 통합 코드 기준 FULL 관련 회귀: `RaceSetupTests` **56개**, `QualifyingTests`
  **3개**, 모두 `OK`.
- 현재 통합 코드 기준 ABSTRACT 전체 테스트 **26개, 87.080초, `OK`**. 동일 입력·seed
  100회 결정성, 입력 순서 독립, stream 격리, 성능 upgrade 방향, Q1/Q2/Q3, canonical
  JSON 왕복을 확인했다.
- Backend 전체 unittest discovery **553개, 1,329.968초, `OK`**. 단계 E 신규 API 테스트
  2개를 포함해 FULL 물리엔진 기준값과 기존 회귀를 통합 후 기준으로 확인했다.
- `git diff --check`, Python compile, 문서 링크 대상 확인도 단계 E 최종 검증에서
  `OK`로 확인했다.

### 알려진 한계와 다음 단계 판정

이 절은 2026-08-03 단계 E 완료 시점의 상태를 기록한다. 당시 ABSTRACT는 즉시 계산되는
결과 요약 화면이었고 live WebSocket race stream, 3D pose renderer, race control,
추월·사고의 시각 합성은 연결하지 않았다. 이후 2026-08-04 단계 E 후속 작업에서
`ABSTRACT_BROADCAST` 결정적 live replay를 별도 경로로 추가했다. `FULL`은 계속 기존 물리
엔진이고, ABSTRACT의 성능·사건·pit 수치는 공식 F1 물성이나 실측값이 아닌 provisional
게임 보정값이다.

## 19. ABSTRACT_BROADCAST — 결정적 live replay 연결 — 완료 (2026-08-04)

단계 E 후속으로 `ABSTRACT_BROADCAST`를 실제 관전 경로에 연결했다. 설계 문서의
`ABSTRACT_BROADCAST`와 `ABSTRACT_INSTANT` 구분을 API와 첫 화면에 반영했으며, 기존
`ABSTRACT` 값은 Stage E 결과 전용 클라이언트 호환을 위해 유지한다.

### Backend broadcast 경로

- [`backend/simulation/abstract/broadcast.py`](../backend/simulation/abstract/broadcast.py)의
  `AbstractBroadcastSession`은 먼저 `AbstractRaceResult`를 완전히 계산한 뒤 immutable
  frame을 logical time 순서로 재생한다. `RaceEngine`, `RaceSession`, FULL mutable state는
  호출하거나 공유하지 않는다.
- `/ws/race`의 기존 low-frequency 의미 계약(`race_info`, `pose_tick`, `race_state`,
  `race_events`, `race_end`)을 사용하되 `source_mode=abstract`와
  `simulation_mode=ABSTRACT_BROADCAST`를 표시한다. pose는 abstract spline 결과이며 힘,
  slip angle, 축별 열수지 같은 물리 텔레메트리를 채우지 않는다.
- `set_speed`는 ABSTRACT에서 `1x/2x/5x`, `pause`/`resume`을 지원한다. pit call, pace mode,
  개발용 race control은 결과가 이미 확정된 broadcast에서 허용하지 않는다.
- `ABSTRACT_INSTANT`와 `ABSTRACT_BROADCAST`는 동일한 `0.10s` logical tick으로 결과를
  만들고, 방송 모드는 같은 결과 frame을 전송하는 차이만 가진다. 화면 효과나 presentation
  호출이 결과 hash를 바꾸지 않는다.
- 초기 `race_info` 전송 중 브라우저가 재연결해도 끊긴 WebSocket을 session에 보존하지 않도록
  정리했다. 이로써 stale client가 이후 broadcast의 dead-client timeout을 누적시키거나 API
  응답을 막지 않는다.

### Frontend 관전 경로

- [`frontend/src/components/raceSetupContract.js`](../frontend/src/components/raceSetupContract.js)의
  mode 선택은 `FULL`, `ABSTRACT_BROADCAST`, `ABSTRACT_INSTANT`를 노출한다. `FULL`은
  여전히 기본 선택이다.
- [`frontend/src/App.jsx`](../frontend/src/App.jsx)는 broadcast를 기존
  [`frontend/src/components/TrackView/ThreeTrackCanvas.jsx`](../frontend/src/components/TrackView/ThreeTrackCanvas.jsx),
  timing board, event feed와 연결한다. 이 재사용은 메시지 의미만 공유하며 FULL 엔진 상태
  객체를 공유하지 않는다.
- ABSTRACT broadcast에서는 기존 renderer pose interpolation을 사용하고, race control에는
  `1x/2x/5x`와 pause/resume만 제공한다. 결과 전용 `ABSTRACT_INSTANT`는 기존 요약 화면을
  계속 사용한다.

### 검증 결과와 한계

- frontend contract/node tests **15개 `OK`**, lint와 Vite production build `OK`.
- API tests **6개 `OK`**. 별도 abstract session 생성, 초기 `race_info`/`pose_tick`/
  `race_state` 순서, `source_mode`, canonical hash 일치, 5x command를 확인했다.
- FULL 관련 회귀는 RaceSetup **56개**, Qualifying **3개**, 모두 `OK`.
- ABSTRACT 전체 테스트 **26개 `OK`**. 100회 결정성·canonical hash, pose 연속성, stream
  격리, Q1/Q2/Q3, Stage C/D 사건을 유지했다.
- 현재 broadcast는 precomputed replay이므로 사용자 전략 명령으로 결과를 바꾸지 않는다.
  setup 시 결과 frame을 먼저 계산하므로 랩 수가 많을수록 첫 race 화면 진입 전 대기 시간이
  늘어나는 한계가 남아 있다. 실제 live strategy input, camera director, 사운드·파티클 cue,
  자동 사건 감속은 후속 presentation 고도화 범위다. 모든 abstract 성능·사건·연출 수치는
  provisional 게임 보정값이며 공식 F1 물성이나 실측값이 아니다.
- 2026-08-04 로컬 5랩 수동 검증에서 `LIVE`, `LAP 2/5`, 20대 timing board, track pose,
  event feed와 1x/2x/5x·pause 컨트롤을 확인했다.
- Backend 전체 unittest discovery **554개, 1,921.844초, `OK`**. ABSTRACT broadcast
  연결 후 FULL 물리엔진과 기존 Backend 회귀를 통합 상태에서 다시 확인했다.
- `git diff --check`, Backend `compileall`, 문서 상대 링크 확인은 모두 `OK`다.

## 20. ABSTRACT 제품 품질 재검토 — 재설계 필요 (2026-08-04)

19절까지의 `완료` 표기는 각 구현 단계의 기능 연결과 당시 자동 테스트 통과 이력이다.
실제 FULL 비교 관전과 자원 benchmark를 포함한 후속 검토 결과,
`ABSTRACT_BROADCAST`는 현재 **제품 승인 실패·실험 prototype**으로 판정한다. FULL은 계속
기본 모드이자 그래픽·주행·회귀 기준이다.

주요 근거:

- abstract pose가 render coordinate를 metric `world_x_m/world_y_m`으로 사용한다. FULL의
  coordinate frame과 비교한 누락 배율은 Bahrain 약 `2.309`, Red Bull Ring 약 `1.744`다.
- abstract `race_info`가 grid, racing-line/width profile, driving line과 pit-exit geometry를
  빈 값으로 전송하고 별도 centerline spline을 사용한다. FULL racing line과 abstract 기본
  pose의 차이는 Bahrain `p95 2.85m/max 3.24m`, RBR `p95 2.32m/max 2.82m`였다.
- Bahrain 1랩·seed 42·0.10초 tick에서 attack start `158`, pass `58`, contact `9`가 발생했고,
  progress 재배치가 포함된 pose 진행량은 순간 `597.6km/h`, 종가속도 `986.9m/s²`, 횡속도
  `44m/s`에 해당했다. 기존 0.50초 중심 테스트는 제품 0.10초 tick의 cooldown 빈도를 대표하지
  못한다.
- pit phase가 진행돼도 pose는 본 트랙 spline에서 생성되어 실제 pit entry/lane/box/exit와
  일치하지 않는다.
- Bahrain 10랩 결과 생성은 `7.322초`, hash 한 번은 추가 `4.525초`였고 hash materialization을
  포함한 process peak RSS는 약 `720.1MB`였다. 20랩은 hash 전에도 `15.066초`, 약 `243.9MB`였다.
  전체 tick×20대 frame 보존과 반복 canonical payload 생성 때문에 57랩 제품 승인을 할 수 없다.

자동 테스트는 abstract/API 32개, frontend 15개, lint/build와 `git diff --check`가 통과했지만,
metric 좌표·주행 연속성·pit route·renderer 품질·57랩 자원 상한을 검사하지 않아 위 결함을
차단하지 못했다.

후속 작업은 [`ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md)의
단계 1~7을 순서대로 수행하고 매 단계 독립 검증을 받아야 한다. 먼저 Stage A의 snapshot/RNG/
성능·퀄리파잉 경계를 보존한 채 결과 저장·hash·setup 자원 구조를 수정한다. 좌표·pose·교통·피트·
사건과 제품 패키징을 한 번에 진행하지 않는다.

## 21. ABSTRACT 재설계 단계 1 — 결과 저장·hash·setup 자원 구조 (2026-08-04)

19절의 `ABSTRACT_BROADCAST` full-frame replay 서술은 단계 1 재설계 전의 구현 이력이다.
재설계 지시서에 따라 단계 1 범위에서 결과 권위와 presentation 저장을 분리했으며, FULL은
기본 모드·물리 권위·기존 회귀 기준으로 유지했다. 단계 2 이후의 좌표·차량 운동·교통/추월·
피트·사건/SC·제품 패키징은 수행하지 않았다.

### 결과 계약과 자원 구조

- [`backend/simulation/abstract/state.py`](../backend/simulation/abstract/state.py)의
  `AbstractRaceResult`는 `snapshot_hash`, 버전 identity, grid/finish classification,
  command log, logical events, `logical_tick_count`와 `AbstractTimingCheckpoint`만 권위 저장한다.
  전체 `0.10초 × 차량` pose frame과 `RaceVehicleState`는 결과에 보관하지 않는다.
- `AbstractTimingCheckpoint`는 랩 경계와 경기 시작/종료의 논리 시간·순위·progress만 가지며
  world 좌표, heading, 힘, slip, 열수지 같은 정밀 물리값을 만들지 않는다.
- `canonical_hash_sections()`가 identity/grid/command/events/timing/finish를 정해진 순서로
  incremental SHA-256 계산하고, `AbstractRaceResult` 생성 시 hash와 canonical JSON을 캐시한다.
  `canonical_result_hash`의 반복 접근은 payload를 다시 생성하지 않는다.
- [`backend/simulation/abstract/replay.py`](../backend/simulation/abstract/replay.py)의
  `RollingFrameBuffer`는 bounded presentation window이며 결과 hash와 독립적이다. Stage 1
  `ABSTRACT_BROADCAST`는 full pose replay를 권위로 사용하지 않고 logical-checkpoint 계약을
  표시한다. 이는 실험 모드이며 고품질 pose는 후속 단계 범위다.
- [`backend/main.py`](../backend/main.py)의 abstract setup CPU 계산은
  `asyncio.to_thread()`로 FastAPI event loop 밖에서 수행하며, summary는 한 번 계산해 재사용한다.
- [`frontend/src/components/raceSetupContract.js`](../frontend/src/components/raceSetupContract.js)와
  [`frontend/src/components/RaceSetup.jsx`](../frontend/src/components/RaceSetup.jsx)는
  `ABSTRACT_BROADCAST`가 Stage 1 experimental logical-result 모드임을 표시한다.

### 자동 benchmark 및 검증 결과

기존 구조를 변경하기 전 동일 benchmark로 재현한 값은 다음과 같다. 10랩은 8,622 frame/172,440
vehicle-state record, race 7.458초, peak Python RSS 721.094MB, hash 첫/두 번째 접근
4.540초/4.445초였다. 20랩은 17,498 frame/349,960 record, race 15.239초, peak RSS
1,414.516MB, hash 9.362초/9.024초였다. 57랩은 51,497 frame/1,029,940 record, race
45.146초, peak RSS 4,071.5MB, hash 27.659초/27.427초였다.

변경 후 [`backend/tools/benchmark_abstract_stage1.py`](../backend/tools/benchmark_abstract_stage1.py)
동일 실행 결과:

- `--laps 10`: 1.246초, peak 50.000MB, 증가 4.125MB, frame 0, vehicle-state 0,
  logical tick 8,622, checkpoint 195, retained collection 493,744B,
  hash 2µs/0µs, hash `6c8b8b23af3d28f03e11f2dee55462f72f1f10f8111ee099b0bfa6c49097ef22`.
- `--laps 20`: 2.533초, peak 56.938MB, 증가 11.016MB, frame 0, vehicle-state 0,
  logical tick 17,498, checkpoint 391, retained collection 857,536B,
  hash 1µs/0µs, hash `8b89c33c82b571d949204664a312044126327f37cef9b9e8be1dbd5bb7528545`.
- `--laps 57`: 7.465초, peak 78.875MB, 증가 32.844MB, frame 0, vehicle-state 0,
  logical tick 51,497, checkpoint 1,127, retained collection 1,774,760B,
  hash 1µs/0µs, hash `befd6041a9a99d9b4dede4cb13c401c4130583e953249f12a2109056222ad277`.

seed 42/Bahrain에서 변경 전후의 10/20/57랩 grid, finish order와 logical event count/type이
같았다. `ABSTRACT_INSTANT`와 `ABSTRACT_BROADCAST` API setup도 같은 grid, finish order,
event summary와 canonical hash를 반환한다.

대표 결과 identity는 1랩 `ba96b548b55f5a96b64c46609e47a5b4302489228db3e9a6e080c0984ed266f2`,
10랩 `6c8b8b23af3d28f03e11f2dee55462f72f1f10f8111ee099b0bfa6c49097ef22`, 57랩
`befd6041a9a99d9b4dede4cb13c401c4130583e953249f12a2109056222ad277`이며, 각 실행의 grid/finish
order는 benchmark JSON 원문에 기록된다.

실행한 검증:

- Abstract/API unittest: **34개, OK**.
- FULL 핵심 회귀 `test_engine`, `test_session`, `test_simulation_foundation`: **349개,
  704.458초, OK**.
- Frontend contract test: **15개, OK**. Frontend lint: **OK**.
- `git diff --check`: **OK**.
- Backend 전체 unittest discovery와 frontend build는 이 단계에서 미실행이다.

단계 1은 권장 57랩 시간/메모리 상한과 관련 결정성 검증을 통과했지만, 전체 Backend discovery와
frontend build 및 단계 2 이후의 관전 품질 승인은 아직 남아 있다.

## 22. ABSTRACT 단계 1 조건부 판정 보완 — logical replay/hash/clock contract (2026-08-04)

단계 2는 시작하지 않고, 조건부 판정에서 요청된 단계 1 보완만 적용했다. 이 절의 hash와
benchmark 값이 command/logical replay contract 보완 후 최신 기준이다.

- [`backend/simulation/abstract/replay.py`](../backend/simulation/abstract/replay.py)에
  `LogicalReplayCursor`를 추가했다. `race_start` checkpoint에서 시작해 checkpoint logical time
  순서로 진행하며, `race_finish` checkpoint를 보낸 뒤 모든 eligible event가 소비된 경우에만
  `race_end`를 보낸다. frame buffer가 비어 있어도 마지막 checkpoint를 초기 상태로 사용하거나
  즉시 종료하지 않는다.
- `InterruptibleReplayClock`는 checkpoint까지의 남은 logical time을 보유하고 wall-clock 경과 ×
  현재 배속만 소비한다. `set_speed`는 현재 wait를 깨워 새 배속으로 남은 시간을 재계산하고,
  `pause/resume`은 잔여 시간을 보존한다. close/cancel 뒤에는 대기 상태와 Event 참조를 정리한다.
  Stage 2 전까지 pose 배열은 비어 있을 수 있다.
- WebSocket event batch는 최대 32개 또는 UTF-8 JSON 16,384 bytes로 분할한다. cursor의 event
  index가 전진하므로 순서 보존·누락 방지·중복 방지를 테스트했다.
- [`AbstractCommandRecord`](../backend/simulation/abstract/state.py)는 sequence와
  logical time을 가진다. command log tuple 순서를 canonical payload와 hash에 그대로 보존하며,
  `logical_tick_count`도 hash identity section에 포함한다.
- benchmark는 `backend/.venv/bin/python backend/tools/benchmark_abstract_stage1.py --laps 1`
  형태로 `PYTHONPATH` 없이 직접 실행할 수 있다.

최신 benchmark:

- 10랩: 1.251초, peak RSS 50.188MB, 증가 4.141MB, frame/state 0/0, checkpoint 195,
  retained collection 493,744B, hash 1µs/0µs,
  hash `1327725d84da488c4daf61f90a9450f347b3461e53f4e1638678d631e02783b9`.
- 57랩: 7.308초, peak RSS 78.641MB, 증가 32.766MB, frame/state 0/0, checkpoint 1,127,
  retained collection 1,774,760B, hash 2µs/0µs,
  hash `da300d6bd71dc1d13e5233f40be07cdf35873110708ca8c17c0aeb9e0faca04d`.

최신 검증:

- Abstract/API unittest: **45개, OK**.
- frontend test: **15개, OK**.
- frontend lint/build: **OK**.
- direct benchmark smoke, backend `compileall`, 문서 링크 확인, `git diff --check`: **OK**.
- 단계 2 좌표·그래픽 구현, commit/push/package 생성: **수행하지 않음**.

## 23. ABSTRACT 단계 2 — FULL 공통 트랙·그래픽 좌표 계약 (2026-08-04)

단계 2 범위만 구현했다. 단계 1의 logical result/hash/replay 계약과 FULL 물리 기본값은
유지했으며, 단계 3의 차량 운동·교통/추월·피트 상태·사건/SC·제품 패키징은 시작하지 않았다.

### 공통 geometry 계약

- [`backend/simulation/track_display.py`](../backend/simulation/track_display.py)의 frozen
  `TrackDisplayGeometry`와 `build_track_display_geometry()`가 public immutable display DTO와
  공통 helper다. `LocalMetricCoordinateFrame`, compiled `TrackPhysicsProfile`, racing/driving
  line, track-width profile, 차량 치수, grid, pit lane/pit-exit/wall geometry와 운영 설정을 함께
  제공한다. DTO의 serialized 값에는 mutable FULL engine state를 넣지 않는다.
- [`backend/simulation/race_engine.py`](../backend/simulation/race_engine.py)의 public
  `track_physics_profile` property를 통해 [`backend/session.py`](../backend/session.py)의 FULL
  `RaceInfo`와 [`backend/simulation/abstract/broadcast.py`](../backend/simulation/abstract/broadcast.py)의
  ABSTRACT `RaceInfo`가 같은 compiled profile/geometry serializer를 소비한다. 기존 pit/grid
  수치와 circuit calibration 값은 변경하지 않았다.
- [`backend/simulation/abstract/pose.py`](../backend/simulation/abstract/pose.py)의 기본
  `TrackSpline` 경로는 독립 Catmull–Rom을 제거하고
  `TrackPhysicsProfile.line_pose_at_progress_m(DRIVING_LINE_RACING, progress)`를 사용한다.
  `world_x_m/world_y_m`는 `coordinate_frame.to_local_m()` 기준의 local metre 값이다.
- [`frontend/src/components/TrackView/coordinateContract.js`](../frontend/src/components/TrackView/coordinateContract.js)는
  Pixi/Three 양쪽에서 `origin_*_render`와 `meters_per_render_unit` 역변환을 공유한다.
- 재현 가능한 start/finish·상단 코너·pit entry/exit 좌표 진단은
  [`ABSTRACT_STAGE2_GEOMETRY_DIAGNOSTIC.json`](ABSTRACT_STAGE2_GEOMETRY_DIAGNOSTIC.json)에 남겼다.

### Bahrain/RBR 승인 기준 측정

1000개 progress 표본에서 ABSTRACT 기본 pose와 FULL compiled racing-line sampler를 비교했다.
두 회로 모두 metric track length는 공식 `circuit.track_length_m`과 정확히 일치했고, position과
heading 오차는 같은 public sampler 사용에 따른 부동소수점 수준이었다.

| circuit | metric length error | position p95 / max | heading p95 | geometry parity |
| --- | ---: | ---: | ---: | --- |
| Bahrain (id 3) | 0.0000% | 0.0000m / 0.0000m | 0.0000° | FULL/ABSTRACT exact |
| Red Bull Ring (id 4) | 0.0000% | 0.0000m / 0.0000m | 0.0000° | FULL/ABSTRACT exact |

FULL/ABSTRACT `race_info`의 origin/scale, racing/driving line, track width, grid 20개, pit
lane/pit-exit와 차량·트랙 metric ratio는 모두 비어 있지 않고 동일했다. racing-line path
length는 기존 FULL 보정값을 그대로 보존했다(Bahrain 5405.2443m, RBR 4318.8293m); 승인된
metric track length의 대상은 coordinate frame의 compiled track length다.

### 단계 1 비회귀·자원 측정

- 10랩: 1.246489초, peak RSS 50.922MB, 증가 4.094MB, frame/state 0/0, logical tick
  8,622, checkpoint 195, retained collection 493,744B, hash 첫/두 번째 1µs/0µs,
  hash `1327725d84da488c4daf61f90a9450f347b3461e53f4e1638678d631e02783b9`.
- 57랩: 7.235890초, peak RSS 79.516MB, 증가 32.797MB, frame/state 0/0, logical tick
  51,497, checkpoint 1,127, retained collection 1,774,760B, hash 첫/두 번째 2µs/0µs,
  hash `da300d6bd71dc1d13e5233f40be07cdf35873110708ca8c17c0aeb9e0faca04d`.
- 같은 seed의 grid/finish order, logical events와 canonical hash는 단계 2 변경 후에도
  유지됐다. benchmark direct-execution smoke도 통과했다.

### 검증 결과

- Stage 2 geometry test: **5개, OK**.
- Abstract/API 및 Stage 1 회귀: **50개, OK**.
- 관련 FULL `track_physics`, `RaceSetup`(grid/pit 포함), `session`, `simulation_foundation`:
  **112개, 56.776초, OK**.
- Frontend test: **16개, OK**. frontend lint/build: **OK**.
- Backend `compileall`: **OK**. 문서 relative link 55개 검사, 누락 0. `git diff --check`:
  **OK**.
- Backend 전체 unittest discovery: **실행하지 않음**. 따라서 전체 backend 통과로 기록하지
  않는다.

### 판정과 한계

단계 2의 공통 geometry·metric coordinate 계약과 자동 검증 기준은 **조건 충족**이다. 다만
독립 검증 전에는 단계 3으로 넘어가지 않는다. ABSTRACT broadcast는 여전히 단계 1의
logical-checkpoint presentation 계약이며 full pose/차량 운동을 만들지 않는다. 이번 단계는
실제 화면 screenshot 대신 좌표 진단 JSON을 남겼고, 전체 backend discovery는 미실행이다.

## 24. ABSTRACT 단계 3 — 단일 차량 종·횡 키네마틱 pose (2026-08-04)

단계 3 범위만 구현했다. 단계 1~2의 immutable result/hash/replay와 공통 FULL geometry는
유지했으며, 단계 4의 20대 교통·추월, 단계 5 pit 주행, 단계 6 incident/VSC/SC/restart와
제품 패키징은 시작하지 않았다.

### 구현 계약

- [`backend/simulation/abstract/kinematics.py`](../backend/simulation/abstract/kinematics.py)의
  frozen `CompiledSplineArcLengthAdapter`와 `RacingLineDistanceContract`가 단계 2 public
  `TrackPhysicsProfile.line_pose_at_progress_m()`와 동일한 compiled spline을 조밀한 periodic
  LUT로 샘플링한다. total progress↔spline arc distance, 누적 lap wrap, arc distance→local
  world pose/heading을 같은 거리 권위로 제공한다. `progress × circuit length`는 호환용
  `track_distance_m`에만 남기고 새 kinematics에는 사용하지 않는다.
- `SpeedKnot`/`LongitudinalKinematicPlan`은 segment base-time과 선택 차량 성능으로 periodic
  target-speed knot, quintic transition, braking anticipation을 만들고 0.10초마다
  `v_next`와 line distance를 전진 적분한다. 표시값은 provisional presentation diagnostic이며
  force, throttle, brake pressure, slip, tire temperature를 만들지 않는다.
- `LateralTrajectory`는 양 끝 속도·가속도가 0인 quintic easing으로 시작/목표 logical time과
  duration을 보유한다. 최소 duration을 계산해 자동 연장하고, 목표가 envelope 밖이면 clamp
  사유를 기록하며, 실행 중 차량 반폭+edge margin 경계를 벗어나면 `ValueError`로 거절한다.
- `SingleVehiclePoseSynthesizer.logical_sequence()`가 canonical logical interval API이고,
  `grid_sequence()`는 단계 2 shared grid slot에서 grid hold와 lights-out launch를 만든다.
  1x/2x/5x는 frame index, logical time, local-metre pose, heading, speed, acceleration과
  presentation pose hash가 같고 배속은 wall-clock scheduling 입력으로만 남는다.
- 매 tick world pose/heading은 위의 동일 compiled arc-length sampler에서 다시 계산한다. 이전의
  distance-only chord 이분법 보정은 제거했고, accepted distance/displacement/previous·next
  speed/acceleration/target을 immutable `KinematicStep` 하나로 원자 확정한다. 정상 tick은
  `ds=(v_prev+v_next)/2×0.10`, `a=(v_next-v_prev)/0.10` identity를 만족하며,
  `BoundedKinematicPoseBuffer`는 선택 probe의 제한된 presentation window만 보관한다.

### 기준 실패 재현과 회로 진단

구현 전에 기존 constant `progress_rate_per_s=0.02` sequence를 Bahrain/RBR에서 측정했다.
`progress × circuit length`와 compiled racing-line distance의 progress 0.5 차이는 각각
`2.1638865m`와 `3.1491842m`였고, 기존 chord 기준 max/min 가속도는 Bahrain
`+257.3486/-265.7765m/s²`, RBR `+165.2304/-193.6523m/s²`였다. 기존 Bahrain 표시속도는
`454.0601km/h`까지 올라갔다. 이 기준 실패는 새 kinematics의 회귀 기준이 아니라 Stage 3
작업 전 결함 재현값이다.

재현 가능한 전체 진단은
[`docs/ABSTRACT_STAGE3_KINEMATICS_DIAGNOSTIC.json`](ABSTRACT_STAGE3_KINEMATICS_DIAGNOSTIC.json)과
[`backend/tools/diagnose_abstract_stage3_kinematics.py`](../backend/tools/diagnose_abstract_stage3_kinematics.py)에
남겼다.

| circuit | max speed | max accel / braking | lateral speed / accel | min boundary clearance | tick / world-chord violations | lap wrap |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Bahrain (id 3) | 313.484926 km/h | +12.017028 / 29.312029 m/s² | 0.468750 / 0.720000 | 4.542577 m | 0 / 0 | 82.5s, 1→2 |
| Red Bull Ring (id 4) | 303.018112 km/h | +10.745800 / 30.949170 m/s² | 0.468750 / 0.720000 | 4.061022 m | 0 / 0 | 69.0s, 1→2 |

두 회로 모두 speed `<=370km/h`, longitudinal `<=+18m/s²`, braking `<=50m/s²`, 음수 speed
0회, distance reversal 0회, line-distance tick violation 0회, world-chord violation 0회였다.
grid hold는 0~1초에 고정되고 1초 lights-out 뒤 속도가 증가했으며(2초 표본 `18m/s`), signed `0→+2m`와
`0→-2m` lateral transition, grid column→racing line 경계 검사를 통과했다. 1x/2x/5x
pose hash는 Bahrain `876b2984128cb669ebba4192beae03f6fddc2546368621d23c04cee32bfc3ff2`,
RBR `3ec4a383ce86dd827bb37a97db1b8cfc4d3ba67a0598e20f1ec65bead41c3465`로 각각 동일했다.

### 검증 결과

- Stage 3 kinematics: **11개, OK**.
- Abstract/result/replay/API와 Stage 2 geometry: **55개, OK**.
- 관련 FULL `track_physics`, `session`, `simulation_foundation`, `engine`/grid/RaceInfo:
  **366개, 693.032초, OK**.
- Stage 1 benchmark 재실행: 10랩 `1.255056초`, peak RSS `51.344MB`, retained
  `493,744B`, frame/state `0/0`, hash 첫/두 번째 `1µs/0µs`, hash
  `1327725d84da488c4daf61f90a9450f347b3461e53f4e1638678d631e02783b9`; 57랩 `7.388848초`,
  peak RSS `79.922MB`, retained `1,774,760B`, frame/state `0/0`, hash 첫/두 번째
  `2µs/0µs`, hash `da300d6bd71dc1d13e5233f40be07cdf35873110708ca8c17c0aeb9e0faca04d`.
- Frontend test **16개, OK**, lint **OK**, production build **OK**. Backend `compileall`
  **OK**, 문서 상대 링크 **67개/누락 0**, `git diff --check` **OK**.

### 판정과 한계

단계 3의 단일 차량 presentation kinematics 기준은 **완료 후보**다. FULL 물리 결과·차량/타이어
모델·회로 보정값은 변경하지 않았다. 이 구현은 결과 엔진이나 logical result hash의 권위가
아니며, 선택 probe의 bounded pose만 제공한다. `world_x_m/world_y_m`는 local metric frame이고
`racing_line_length_m`(Bahrain `5405.2443m`, RBR `4318.8293m`)를 pose 거리로 사용하므로
공식 F1 물성이나 실측 주행 모델로 해석하면 안 된다. 전체 Backend unittest discovery는 이번
단계에서 **실행하지 않았다**.

단계 4는 시작하지 않았으며, 다음 단계로 진행할지는 독립 검증 후 판정한다.

## 25. ABSTRACT 단계 3 보완 — arc-length atomic pose integrator와 single probe (2026-08-04)

독립 검증에서 확인된 Stage 3의 distance-only post correction 결함만 수정했다. 단계 4의
20대 교통·추월·순위 crossing, 단계 5 pit 주행, 단계 6 incident/VSC/SC/restart와 제품
패키징은 시작하지 않았다. 기존 Stage 1~2 result/hash/replay/geometry 계약과 FULL 물리 기본값은
변경하지 않았다.

### 실패 고정과 수정 내용

수정 전 실패는 pose 차분 기준으로 고정했다. Bahrain은 distance-derived acceleration
`-57.2411~+27.3526m/s²`, 저장 acceleration `-29.3120~+12.0346m/s²`, 최대 적분 거리
오차 `0.409814m`, 1mm 초과 tick `44개`였다. RBR은 최대 적분 거리 오차 `0.319818m`,
1mm 초과 tick `8개`였다. `test_distance_derived_metrics_and_reported_identity`와
`test_atomic_integrator_step_exposes_one_accepted_identity`가 이 실패와 수정 후 identity를
회귀 고정한다.

- [`backend/simulation/abstract/kinematics.py`](../backend/simulation/abstract/kinematics.py)의
  public frozen `CompiledSplineArcLengthAdapter`가 `TrackPhysicsProfile.line_pose_at_progress_m()`
  결과를 16,384구간 periodic LUT로 샘플링한다. total progress→spline arc distance,
  spline arc distance→total progress, arc distance→local world x/y/heading, lap wrap을 한
  거리 계약으로 제공한다. FULL의 물리 calibration/path length를 변경하지 않는다.
- `LongitudinalKinematicPlan.integrate()`는 accepted distance, displacement, 이전/다음
  speed, acceleration, target speed를 frozen `KinematicStep` 하나로 확정한다. pose.py의
  거리 사후 이분법 축소와 speed/acceleration 불일치를 제거했다. Bahrain/RBR 전체 0.10초
  tick에서 trapezoid distance identity와 reported acceleration identity violation이 모두
  `0`이다.
- [`backend/tools/diagnose_abstract_stage3_kinematics.py`](../backend/tools/diagnose_abstract_stage3_kinematics.py)는
  저장 필드만 읽지 않고 실제 pose line-distance 차분으로 derived speed/acceleration,
  integrator error, reported identity, post-correction count, world-chord speed jump와
  `approval_passed`를 계산한다. 승인 실패 시 exit code `1`을 반환한다. 최신 원문은
  [`ABSTRACT_STAGE3_KINEMATICS_DIAGNOSTIC.json`](ABSTRACT_STAGE3_KINEMATICS_DIAGNOSTIC.json)이다.

### 회로 승인 측정

| circuit | derived speed min/max | derived accel min/max | integrator distance error | identity violations | max world-chord speed jump | approval |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Bahrain (id 3) | 43.125981 / 87.079171 m/s | -28.518051 / +11.981813 m/s² | 0.000000m | 0 / 0 | 2.927539m/s | **true** |
| Red Bull Ring (id 4) | 41.530361 / 84.171768 m/s | -30.298490 / +10.683153 m/s² | 0.000000m | 0 / 0 | 3.792575m/s | **true** |

두 회로의 max 표시속도는 각각 `313.485017km/h`, `303.018364km/h`이고, braking magnitude는
각각 `28.757175m/s²`, `30.598965m/s²`이다. speed/acceleration/reversal/tick/world-chord/
boundary violation은 모두 `0`이다. Bahrain progress `0.42444`, `0.13158`와 RBR
`0.29789~0.29960`을 회귀 표본으로 포함했다. grid hold는 0~1초 정지, 1초 뒤 launch이며
lap wrap은 Bahrain 82.5초, RBR 69.0초에 관측됐다. 1x/2x/5x pose hash는 Bahrain에서
`80f3a2d0e2c7c8453efb1f0add78256755f3c61a13d4778a686e7e1ae82563e7`, RBR에서
`66d1f2b2cca4e98b7367c6e46d1a94008962f2bbeefa01160d6d00ffc51411e0`로 각각 동일했다.

`reported_vs_derived_speed_error_max_mps`는 한 tick line-average speed와
`(v_prev+v_next)/2`의 차이를 기록하며, Bahrain/RBR 모두 `0.0m/s`이다. 승인 identity는
trapezoid distance와 speed-difference acceleration으로 별도 판정했고 두 회로 모두 `0`
violation이다.

### 실제 broadcast probe 계약

- [`backend/simulation/abstract/broadcast.py`](../backend/simulation/abstract/broadcast.py)의
  `STAGE3_SINGLE_PROBE_CONTRACT`(`stage3-single-probe-kinematic`)는 안정적으로 선택한 한 명의
  player driver만 `SingleVehiclePoseSynthesizer`로 on-demand 계산한다. `pose_tick`은 항상
  `pose_count=1`, `poses` 1개, `probe_only=true`이며 rank authority는
  `logical-checkpoint`이다. 나머지 19대 pose와 57랩 전체 pose tuple은 생성·보관하지 않는다.
- probe는 `BoundedKinematicPoseBuffer(max_frames=128)`에 checkpoint별 최신 표본만 보관한다.
  race start에서는 Stage 2 shared grid slot의 grid hold/lights-out launch를 사용하고, 이후
  checkpoint에서만 한 차량 pose를 재계산한다. `pause` 동안 checkpoint/probe cursor가 멈추고,
  resume 및 1x→5x 배속 변경 후 순서가 유지되며, close 뒤 loop task, wait Event, client와 probe
  buffer를 정리한다.

### 단계 1 비회귀 및 검증

10랩/57랩 benchmark 실측은 다음과 같다. 두 hash는 단계 1 기준과 동일하다.

- 10랩: elapsed `1.287158s`, peak Python RSS `52.188MB`(증가 `5.266MB`), retained
  collection `493,744B`, frame/state `0/0`, hash 첫/두 번째 `1µs/0µs`, logical tick
  `8,622`, checkpoint `195`, canonical hash
  `1327725d84da488c4daf61f90a9450f347b3461e53f4e1638678d631e02783b9`.
- 57랩: elapsed `7.527983s`, peak Python RSS `82.250MB`(증가 `35.078MB`), retained
  collection `1,774,760B`, frame/state `0/0`, hash 첫/두 번째 `2µs/0µs`, logical tick
  `51,497`, checkpoint `1,127`, canonical hash
  `da300d6bd71dc1d13e5233f40be07cdf35873110708ca8c17c0aeb9e0faca04d`.

실행 결과:

- Stage 3 kinematics + Abstract/result/replay/API + Stage 2 geometry: **64개, OK**.
- 관련 FULL track physics/session/foundation/engine/grid/RaceInfo/pit: **366개, 798.963초, OK**
  (`track_physics` 17, `session` 13, `simulation_foundation` 26, `engine` 310).
- frontend test **16개, OK**, lint **OK**, production build **OK**.
- backend `compileall` **OK**, diagnostic approval **exit 0**, 문서 상대 링크 **68개/누락 0**,
  `git diff --check` **OK**.
- Backend 전체 unittest discovery는 실행하지 않았다. commit, push, package 생성과 단계 4
  구현도 실행하지 않았다.

### 판정과 알려진 한계

단계 3 보완의 자동 승인 기준은 **충족**했다. 다만 이것은 독립 검증을 기다리는 완료 후보이며,
현재 요청의 범위상 단계 4로 진행하지 않는다. arc-length presentation 계약의 spline arc
length는 Bahrain `5422.916891m`, RBR `4325.753412m`로, 기존 FULL calibration path length
`5405.2443m`/`4318.8293m`와 다르다. 이는 같은 world pose spline의 kinematic distance
권위로 추가한 값이며 FULL 수치·회로 calibration·공식 F1 물성값을 의미하지 않는다. 또한
현재 broadcast pose는 의도적으로 single-car preview일 뿐 실제 순위/leader나 full-field
physical replay가 아니다.

단계 4는 시작하지 않았고, 독립 검증 전에는 시작하지 않는다.

## 26. ABSTRACT 단계 3 독립 재검증 — broadcast 연속성 미승인 (2026-08-04)

단계 3 보완 구현을 독립 재검증한 결과, same-spline arc-length와 atomic integrator의 실제 차분
운동학은 승인 범위를 만족했다. Bahrain derived acceleration은
`-28.518051~+11.981813m/s²`, RBR은 `-30.298490~+10.683153m/s²`이고 두 회로의 integrator 및
reported-acceleration identity violation은 `0/0`이다.

그러나 실제 `AbstractBroadcastSession`은 3초 이후 accepted kinematic step을 이어서 재생하지
않고 checkpoint progress를 직접 pose로 sample한다. Bahrain 5랩 seed 44에서 pose는 97개,
첫 갱신은 logical `84.2s`, 최대 frame gap은 `842 tick`, 0.10초 초과 interval은 `72/96`, 최대
world jump는 `428.519940m`였다. 위치 차분 최대 속도도 `116.469208m/s`(`419.289km/h`)로 표시
상한을 넘었다. 따라서 실제 single-probe broadcast의 pose 누락·cursor jump 승인 기준은
충족하지 못했고 단계 3은 **재보완 필요**로 판정한다.

추가로 진단 도구의 `post_integrator_distance_correction_count`가 실제 계측이 아니라 상수 `0`으로
기록되는 문제를 확인했다. 남은 수정 범위와 승인 기준은
[`ABSTRACT_RACE_SIMULATION_STAGE3_BROADCAST_CORRECTION_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE3_BROADCAST_CORRECTION_DIRECTIVE.md)에 고정했다. 단계 4는 시작하지 않는다.

## 27. ABSTRACT 단계 3 재보완 — 연속 single-probe broadcast (2026-08-04)

단계 3 독립 재검증에서 실패한 실제 broadcast 경로만 보완했다. same-spline arc-length,
atomic integrator, Stage 1 result/hash와 Stage 2 geometry는 유지했으며, 단계 4의 20대
교통·추월·순위 crossing, 단계 5 pit, 단계 6 incident/VSC/SC/restart와 제품 패키징은
시작하지 않았다.

### 수정 계약

- [`backend/simulation/abstract/pose.py`](../backend/simulation/abstract/pose.py)의 frozen
  `SingleProbeKinematicCursor`가 선택 driver 한 명의 logical tick/time, accepted line
  distance, speed, 마지막 `KinematicStep`, grid hold/launch 상태와 현재 pose만 보유한다.
  `advance_one_tick()`은 0.10초 logical tick을 정확히 한 번 적분하고,
  `current_pose()`는 cursor를 전진시키지 않는다.
- [`backend/simulation/abstract/broadcast.py`](../backend/simulation/abstract/broadcast.py)의
  game loop는 매 logical tick마다 replay clock을 기다린 뒤 probe를 정확히 한 번 전진시키고
  `pose_tick`을 전송한다. checkpoint가 없는 tick도 전송하며, checkpoint progress를
  presentation pose에 덮어쓰거나 snap하지 않는다. checkpoint는 순위/lap/event/result의
  권위로만 남고 pose에는 `probe_only=true`, `rank_authority=logical-checkpoint`를 표시한다.
- `BoundedKinematicPoseBuffer(128)`는 최신 single-car window만 유지한다. 57랩 controlled
  replay에서도 full-race pose tuple이나 20대 pose를 보관하지 않는다. reconnect/current-state
  전송은 cursor와 buffer를 전진시키지 않고, pause는 cursor와 잔여 logical time을 보존하며,
  resume과 1x/2x/5x 변경은 wall-clock scheduling만 바꾼다. close 후 loop task, replay wait
  Event, client, cursor와 probe buffer를 해제한다.
- broadcast 계측은 `frame gap`, logical-time gap, 누락·중복, interval violation, world jump,
  위치 차분 파생속도와 370km/h 초과 횟수를 retained pose sequence 없이 집계한다.
  [`backend/tools/diagnose_abstract_stage3_kinematics.py`](../backend/tools/diagnose_abstract_stage3_kinematics.py)는
  accepted step와 실제 pose의 distance/speed/acceleration/ds를 비교하며, mismatch 또는
  integrator adjustment가 있으면 approval을 거부한다. 최신 결과는
  [`ABSTRACT_STAGE3_KINEMATICS_DIAGNOSTIC.json`](ABSTRACT_STAGE3_KINEMATICS_DIAGNOSTIC.json)에
  기록했다.

### 실제 controlled broadcast 결과

Bahrain 5랩 seed 44를 실제 `AbstractBroadcastSession` loop와 WebSocket double로 실행했다.
초기 frame 0 뒤 종료 tick 4420까지 pose 4421개가 연속 전송됐고, frame/time 간격과 race end는
다음과 같았다.

| 항목 | 결과 |
| --- | ---: |
| 첫/마지막 pose frame | 0 / 4420 |
| logical time | 0.0s → 442.0s, 매 interval 0.10s |
| missing / duplicate / interval violation | 0 / 0 / 0 |
| max frame gap | 1 |
| max world jump | 8.577948m |
| max 위치 차분 파생속도 | 85.779481m/s (308.806km/h) |
| 370km/h 초과 | 0 |
| probe buffer count / peak | 128 / 128 |
| race_end | final checkpoint/event drain 뒤 1회 |

event ID는 원래 logical event 순서와 정확히 같고 중복이 없었다. 1x/2x/5x controlled replay의
result hash, broadcast pose hash, pose count와 race-end count도 모두 동일했다. replay 중
`1x→5x→2x→1x` 변경을 주입한 별도 WebSocket 테스트에서도 frame gap 1, logical-time gap
0.10초, missing/duplicate 0을 유지했다. pause 0.20초 동안 checkpoint/probe cursor가
정지했고, resume 후 잔여 logical time부터 진행했다.

### 장거리 bounded 검증

Bahrain 57랩 controlled replay는 pose 53,593개를 전송했지만 retained probe는 최대·종료 시
128개였고, event ID 5,224개는 원래 순서·중복 없이 drain됐다. race_end는 1회였고 close 뒤
client, cursor, Event, task와 buffer는 모두 정리됐다.

### 검증 결과

- Stage 1 deterministic/result/replay, Stage 2 geometry, Stage 3 kinematics 및 API/WebSocket:
  **69개, OK**. 같은 입력·seed 100회 결과와 canonical hash도 동일했다.
- Stage 1 benchmark 재실행: 10랩 elapsed `1.311304s`, peak RSS `52.703MB`(증가 `5.469MB`),
  retained `493,744B`, frame/state `0/0`, logical tick `8,622`, hash 첫/두 번째 `1µs/0µs`,
  canonical hash `1327725d84da488c4daf61f90a9450f347b3461e53f4e1638678d631e02783b9`; 57랩
  elapsed `7.439791s`, peak RSS `81.781MB`(증가 `34.781MB`), retained `1,774,760B`,
  frame/state `0/0`, logical tick `51,497`, hash 첫/두 번째 `2µs/0µs`, canonical hash
  `da300d6bd71dc1d13e5233f40be07cdf35873110708ca8c17c0aeb9e0faca04d`.
- 관련 FULL track physics/session/foundation/engine/grid/RaceInfo/pit 회귀: **366개,
  671.076초, OK**. FULL 기본 모드 결과와 기존 기준값은 변경하지 않았다.
- frontend test **16개, OK**, lint **OK**, production build **OK**. backend `compileall`
  **OK**, Stage 3 diagnostic approval **exit 0**, 문서 relative link **80개/누락 0**,
  `git diff --check` **OK**.
- Backend 전체 unittest discovery는 실행하지 않았다. commit, push, package 생성과 단계 4
  구현도 실행하지 않았다.

### 판정과 알려진 한계

실제 ABSTRACT broadcast의 Stage 3 single-probe 연속성 보완 기준은 **충족**했다. 다만
presentation은 선택 차량 한 대의 bounded preview이고, logical checkpoint/result가 실제 순위와
finish의 유일한 권위다. 20대 pose replay, 교통·추월, pit 주행, incident/VSC/SC/restart는
아직 구현하지 않았다. pose의 provisional kinematic 수치는 공식 F1 물성이나 실측값이 아니며,
FULL 물리·차량·타이어·회로 calibration을 복제하거나 변경하지 않는다.

단계 4는 시작하지 않았고, 독립 검증을 위해 여기서 멈춘다.

## 31. ABSTRACT 단계 4 — 20대 교통·순위 동기화·full-field broadcast (2026-08-05)

단계 4 작업 지시서의 Gate 4A→4B→4C→4D 범위만 구현했다. 단계 1~3의 snapshot, canonical
result/hash, same-spline arc distance, atomic kinematics, replay clock과 race-start timeline은
보존했다. 단계 5 pit route/box/merge, 단계 6 contact/spin/retirement/VSC/SC/restart, 단계 7
제품 패키징은 시작하지 않았다.

### 구현 계약

- [`backend/simulation/abstract/race.py`](../backend/simulation/abstract/race.py)의
  `AbstractTrafficSimulationCursor`와 `AbstractTrafficTick`을 Instant/Broadcast 공용의
  resumable bounded traffic cursor로 분리했다. 각 차량은 Stage 3 compiled spline arc distance를
  기준으로 accepted distance/speed/acceleration을 한 tick에 원자적으로 확정한다.
- car-following은 tick 시작 시 앞차의 거리·속도, body gap, closing speed, braking distance와
  reaction buffer를 사용해 다음 속도를 계획한다. 정상 가속/제동 guard는 `+18/-50m/s²`, 최고
  표시속도는 `98m/s`(352.8km/h)이며, 수치는 모두 provisional 게임 표시 보정값이지 공식 F1
  물성·실측값이 아니다. 적분 뒤 progress/distance clamp나 post-integrator correction은 없다.
- grid hold 1초, lights-out launch, lap wrap, Stage 3의 world chord pre-commit 재계획을 공용
  cursor에 연결했다. 초과 시 x/y를 사후 clamp하지 않고 같은 sampler로 속도·가속도·거리를
  함께 다시 계산한다.
- [`backend/simulation/abstract/racecraft.py`](../backend/simulation/abstract/racecraft.py)의
  `LocalCorridorAssessment`는 local left/right width, racing-line offset, segment/corner phase,
  gap·closing speed, 차량 크기, 제3 차량과 기존 time/space reservation을 함께 평가한다.
  maneuver는 approach→pull_out→overlap→crossing_confirmed→clearance_confirmed→rejoin
  흐름을 사용하고, 실제 crossing과 양쪽 insertion clearance가 확인된 경우에만
  `rank_swap="atomic"`으로 순위를 바꾼다. abort는 현재 lateral offset에서 safe fallback/rejoin하며
  logical pair cooldown은 8초 이상이다.
- [`backend/simulation/abstract/broadcast.py`](../backend/simulation/abstract/broadcast.py)는
  공용 cursor를 `pose_count=20` full-field broadcast에 연결했다. 20대의 현재 상태와 최대
  128개 rolling pose만 유지하며 57랩 전체 pose tuple은 저장하지 않는다. Broadcast와 Instant의
  frame sink 유무는 RNG, event, finish order, result hash를 바꾸지 않는다.

### 교통 매트릭스 및 진단

`backend/tools/diagnose_abstract_stage4_traffic.py`로 Bahrain(id 3)·Red Bull Ring(id 4),
10 seeds×10랩을 실행해 [`ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json`](ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json)에
20회 결과를 남겼다. `contract=abstract-stage4-traffic-v1`, `approval_passed=true`이다.

| 회로 | attack start 총계 | driver/lap 평균 범위 | driver/lap max | pass | defense | abort | logical contact | 최소 cooldown |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Bahrain | 205 | 0.075–0.130 | 4 | 89 | 75 | 41 | 42 | 9.6s |
| Red Bull Ring | 186 | 0.075–0.125 | 4 | 90 | 59 | 37 | 40 | 8.0s |

두 회로 모두 body overlap, 실제 corridor conflict, crossing 전 rank swap, clearance 없는 pass,
post-integrator correction, integrator/reported-acceleration identity, tick displacement,
boundary violation, finish 누락·중복이 0이었다. 표의 driver/lap max 4는 평균 승인 기준(≤3)이 아닌
별도 관찰값으로 기록한 것이다. 제3 차량 rejection은 Bahrain 25,326건/RBR 33,118건,
reservation rejection은 357건/618건으로, 승인된 충돌이 아니라 후보 거절 횟수이며 실제
corridor conflict는 0이다.

hard clamp 제거와 강제 순위 swap 제거로 `abstract_engine_version`을 기존 prototype에서
`abstract-stage4-traffic-v1`로 올렸다. Stage 1 benchmark의 새 결과는 10랩 logical ticks
9,504, retained collection 149,648B, hash
`069e094f9e015b284815f4092e9c0e7938ee4a907ae672c3c03fba2accb50393`, 57랩 ticks 48,693,
retained 748,632B, hash
`e8b29ba218d123d9fd1d945fdb7232d31bc52c80073c3c160dec97e5f7c4655d`이다. 이는 이전 Stage 3
기준 10랩 `1327725d84da488c4daf61f90a9450f347b3461e53f4e1638678d631e02783b9`, 57랩
`da300d6bd71dc1d13e5233f40be07cdf35873110708ca8c17c0aeb9e0faca04d`와 달라졌으며, 의도된
engine version 변경에 따른 결과 변경이다. 새 cursor/100회 결정성 및 frame-sink parity 테스트와
1x/2x/5x broadcast 회귀에서 grid, finish order, logical event 의미와 결과 hash의 안정성을
확인했다.

이전 Stage 1 benchmark artifact에는 old finish_order/event 배열을 저장하지 않고 tick/hash만
저장했으므로, 이 작업에서 old finish/event 배열 자체를 직접 재비교할 수는 없다. 현재 Stage 4
20회 결과는 매 실행 20명 unique finish와 ordered logical event를 확인했고, 해당 baseline
artifact의 미보존 부분은 독립 검증 시 재측정 대상으로 남긴다.

### 검증 결과

- Stage 4 traffic tests: **8개, OK**; Abstract 기존+Stage 4: **47개, OK**; API/WebSocket:
  **13개, OK** (57랩 controlled full-field replay 포함).
- Stage 2 geometry + Stage 3 kinematics: **19개, OK**. Stage 3 diagnostic Bahrain/RBR
  `approval_passed=true`, accepted-step/pose identity와 post-integrator correction은 0이다.
- 관련 FULL track physics/session/foundation/engine/grid/RaceInfo/pit 회귀: **366개,
  743.765초, OK**. 이는 전체 Backend discovery가 아니다.
- frontend test **16개, OK**, lint **OK**, production build **OK**; backend `compileall` **OK**,
  문서 relative link **95개/누락 0**, `git diff --check` **OK**.
- Stage 1 benchmark 실제 재측정: 10랩 elapsed 8.631013s, peak RSS 증가 4.703MB,
  retained 149,648B, frame/state 0/0; 57랩 elapsed 46.096156s, peak RSS 증가 23.781MB,
  retained 748,632B, frame/state 0/0. 양쪽 hash 첫/두 번째 접근은 약 1µs/0µs였다.

### 한계와 판정

실제 20대 traffic cursor와 bounded full-field broadcast까지 단계 4 범위를 충족했지만,
단일 driver/lap의 attack start 최대 관찰값은 4로 평균 guard보다 높으므로 후속 balance 검토
대상이다. 이는 진단에 숨기지 않고 기록했으며 tolerance 확대로 통과시키지 않았다. 단계 5의
pit route/box/merge 주행, 단계 6 사건·SC/restart, 단계 7 제품 패키징은 미구현이다.

단계 4 판정: **조건부 완료 — 독립 검증 대기**. 단계 5는 시작하지 않는다.

## 28. ABSTRACT 단계 3 재보완 독립 검증 — race-start event 지연 (2026-08-04)

실제 0.10초 single-probe broadcast는 독립 controlled replay에서도 frame `0~4420`, time
`0.0~442.0s`, frame/time gap `1/0.10s`, missing/duplicate `0/0`, 최대 world jump
`8.577948m`, 위치 차분 최고속도 `85.779481m/s`로 승인 기준을 만족했다. accepted-step/pose
distance·speed·acceleration mismatch와 post-integrator correction도 모두 `0`이다.

다만 `race_started(payload logical_time=0.0s)`가 실제로는 첫 다음 checkpoint인 pose time
`84.2s`에 전송되는 event timeline 회귀를 확인했다. game loop가 다음 checkpoint를 advance할 때만
event를 drain하고 현재 `race:start` checkpoint의 event를 첫 wait 전에 처리하지 않는 것이
원인이다. 수정 범위와 회귀 기준은
[`ABSTRACT_RACE_SIMULATION_STAGE3_EVENT_TIMELINE_CORRECTION_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE3_EVENT_TIMELINE_CORRECTION_DIRECTIVE.md)에 고정했다. 이 한 항목을 독립 재검증하기 전까지 단계 3은 조건부이며 단계 4를 시작하지 않는다.

## 29. ABSTRACT 단계 3 최종 보완 — race-start event timeline (2026-08-04)

독립 검증에서 확인된 마지막 단계 3 결함인 `race_started` 최초 전송 시점만 수정했다. 연속
single-probe, same-spline arc-length, atomic integrator, bounded buffer/cleanup, logical result와
FULL 계약은 재설계하지 않았으며, 단계 4~6 기능은 시작하지 않았다.

### 초기 timeline 수정

- [`backend/simulation/abstract/broadcast.py`](../backend/simulation/abstract/broadcast.py)는 첫
  정상 client가 activity gate를 통과한 뒤 첫 `replay_clock.wait(0.10)` 전에 현재 `race_start`
  checkpoint의 event를 global cursor에서 한 번 drain한다. `add_client()`는 race_info, pose 0,
  race_state만 전송하고 global event cursor를 소비하지 않는다.
- 초기 snapshot 전송 실패 client는 client set에 등록되지 않으므로 event cursor를 변경하지 않는다.
  event 전송 중 모든 client가 끊긴 경우에도 batch cursor를 복원해 다음 정상 client가
  `race_started`를 받을 수 있게 했다.
- [`backend/simulation/abstract/replay.py`](../backend/simulation/abstract/replay.py)에 undelivered
  batch rollback을 위한 `LogicalReplayCursor.restore_event_index()`를 추가했다. 정상 event
  ID와 payload, batch 상한 `32개/16,384 bytes`, result/hash 의미는 변경하지 않았다.

### 실제 controlled-clock timeline 결과

Bahrain 5랩 seed 44의 실제 `AbstractBroadcastSession`와 WebSocket double에서 다음 순서를
확인했다.

| 항목 | 결과 |
| --- | ---: |
| `race_started` count | 1 |
| `race_started.payload.logical_time_s` | 0.0s |
| event 전 마지막 pose | physics frame 0 / logical time 0.0s |
| frame 1보다 event 선행 | PASS |
| `race_finished` count/time | 1 / final logical time 442.0s |
| `race_finished` 전 마지막 pose | final frame 4420 / 442.0s |
| `race_end` | 모든 event 뒤 마지막 메시지, 1회 |

기존 지연 상태에서는 `race_started`가 pose time 84.2초에 전송됐지만, 수정 후 초기 state 뒤,
frame 1 전에 전송된다. 실패 snapshot client 뒤 정상 client 연결에서도 `race_started`는
global session에서 정확히 한 번만 전송됐다. reconnect 전후 probe cursor와 buffer count는
변하지 않았다.

### 비회귀 검증

- 초기 timeline, 실패 client, reconnect, 종료 순서 테스트를 포함한 Abstract/Stage 2/Stage 3/API
  통합 테스트: **71개, OK**.
- Stage 1 benchmark 재실행: 10랩 elapsed `1.315319s`, peak RSS `52.578MB`(증가 `5.469MB`),
  retained `493,744B`, frame/state `0/0`, logical tick `8,622`, hash 첫/두 번째 `1µs/0µs`,
  canonical hash `1327725d84da488c4daf61f90a9450f347b3461e53f4e1638678d631e02783b9`; 57랩
  elapsed `7.441969s`, peak RSS `81.891MB`(증가 `34.734MB`), retained `1,774,760B`,
  frame/state `0/0`, logical tick `51,497`, hash 첫/두 번째 `2µs/0µs`, canonical hash
  `da300d6bd71dc1d13e5233f40be07cdf35873110708ca8c17c0aeb9e0faca04d`.
- 기존 single-probe 연속성: frame/time gap `1/0.10s`, missing/duplicate `0/0`, 1x/2x/5x
  pose hash 동일, buffer capacity 128, close cleanup 통과.
- Stage 3 kinematics diagnostic: Bahrain/RBR approval `true`, post-integrator correction 및
  accepted-step pose mismatch `0`.
- 관련 FULL track physics/session/foundation/engine/grid/RaceInfo/pit 회귀: **366개,
  676.746초, OK**. frontend test **16개, OK**, lint **OK**, production build **OK**.
- backend `compileall` **OK**, 문서 relative link **86개/누락 0**, `git diff --check` **OK**.
  전체 Backend unittest discovery는 실행하지 않았다.

### 판정과 한계

race-start event timeline 보완 기준은 **완료 후보**다. ABSTRACT broadcast는 여전히 선택 차량
한 대의 bounded presentation preview이고 logical checkpoint/result가 순위·finish 권위다.
20대 교통·추월, pit 주행, incident/VSC/SC/restart와 제품 패키징은 구현하지 않았다.

단계 4는 시작하지 않았고, 독립 검증을 위해 여기서 멈춘다.

## 30. ABSTRACT 단계 3 독립 승인과 단계 4 작업 기준 (2026-08-04)

단계 3 최종 보완을 독립 재검증했다. Bahrain 5랩 seed 44에서 `race_started`는 pose frame
`0`/logical `0.0s` 뒤, frame 1 전에 정확히 한 번 전송됐고 `race_finished`는 final frame
`4420`/`442.0s`, `race_end`는 모든 event 뒤 마지막에 한 번 전송됐다. event 전송 중 모든 client가
끊긴 경우 event cursor `0`, client `0`, initial-drained `false`로 복원되고 다음 정상 client가
frame 0에서 `race_started`를 한 번 받는 것도 확인했다.

연속 pose는 frame/time gap `1/0.10s`, missing/duplicate `0/0`, 최대 world jump `8.577948m`, 위치
차분 최고속도 `85.779481m/s`를 유지했다. 관련 Abstract/Stage 2/Stage 3/API 71개, 관련 FULL 56개,
frontend 16개, diagnostic, compileall, lint/build, 문서 링크와 `git diff --check`가 통과했고 10/57랩
canonical hash도 기존 기준과 같았다. 전체 Backend discovery와 FULL 366개 묶음은 독립 검토에서
재실행하지 않았다. 단계 3은 **승인**한다.

다음 작업은
[`ABSTRACT_RACE_SIMULATION_STAGE4_WORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE4_WORK_DIRECTIVE.md)의 단계 4로 제한한다. 현재 prototype의 progress hard clamp, 고정 5-tick 강제 pass, 즉시 lateral offset, tick 기반 cooldown과 global-width corridor를 shared bounded traffic stepper, anticipatory car-following, local/third-car corridor, crossing+clearance 기반 rank swap과 20대 broadcast로 교체한다. 단계 5 이후는 시작하지 않는다.

## 32. ABSTRACT 단계 4 독립 검증과 최종 보완 기준 (2026-08-05)

단계 4 구현을 독립 재검증했다. shared traffic cursor, 20대 bounded broadcast, accepted-step 원자성,
crossing/clearance 기반 rank swap, pair cooldown과 replay hash divergence guard는 실제 코드와 테스트에서
확인됐다. 관련 ABSTRACT/Stage 2~4/API 79개, 관련 FULL 선별 회귀 47개, frontend 16개와
lint/build, compileall, 문서 링크 및 `git diff --check`가 통과했다.

Bahrain/RBR seeds 0..9×10랩 진단은 20회 모두 승인됐고 저장된
[`ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json`](ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json)과 재실행 파일의
SHA-256이 일치했다. 20회에서 attack/pass/defense/abort/contact는 각각
`391/179/134/78/82`, 완주 20대, body overlap·boundary·reverse·post-integrator correction·crossing 전
swap·cooldown violation은 0이었다. 10/57랩 benchmark hash도 현재 문서 기준과 일치했다.

다만 `corridor_conflict_count`가 실제 reservation 교차를 독립 계측하지 않고 초기값 0에 의존하며,
reservation에 arc/time/lateral 점유 계약이 없다. maneuver는 `approach` 없이 `pull_out`으로 시작하고
driver별 logical cooldown이 없으며, 결정적 10개 상황과 10 seeds×2 circuits×Instant/1x/2x/5x 전체
매트릭스도 아직 충족되지 않았다. `AbstractRaceEngine.run()` 아래에는 도달 불가능한 구형 clamp/swap
loop가 남아 있다.

따라서 단계 4는 **조건부 완료 — 최종 보완 필요**로 판정한다. 다음 작업은
[`ABSTRACT_RACE_SIMULATION_STAGE4_CORRECTION_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE4_CORRECTION_DIRECTIVE.md)에
한정하며 단계 5 이후는 시작하지 않는다.

## 33. ABSTRACT 단계 4 최종 보완 — Gate 4R (2026-08-05)

단계 4 최종 보완 지시에 따라 Gate 4R-A→4R-B→4R-C→4R-D→4R-E 순서로 reservation,
local corridor, maneuver 상태기, logical cooldown, shared full-field cursor와 진단을 보완했다.
FULL 물리·타이어·회로 calibration과 단계 5 이후 기능은 변경하거나 시작하지 않았다.

### 구현 및 진단

- `TrafficCorridorReservation`은 frozen/serializable DTO로 arc interval, logical time window,
  lateral bounds, required/available width, side, segment/corner phase, vehicle dimensions와
  anticipated third-vehicle IDs를 보존한다. accepted frame마다 time+arc+lateral/body envelope와
  third-vehicle occupancy를 독립 감사한다.
- maneuver는 `approach → pull_out → overlap → crossing_confirmed → clearance_confirmed →
  overtake_completed → rejoin → rejoin_complete` 또는
  `yield_or_abort → fall_back_to_safe_gap → rejoin → rejoin_complete` 순서만 허용한다.
  rank swap은 clearance 확인과 같은 accepted step에서만 수행하며, pair/driver eligibility는
  logical seconds로 기록한다.
- [`ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json`](ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json)은
  Bahrain(id 3)·Red Bull Ring(id 4), seeds `0..9`, 10랩, 20대, Instant/controlled Broadcast
  `1x/2x/5x` 총 80회 결과를 저장한다. `approval_passed=true`, parity failure `0`, zero-guard
  failure `0`, close cleanup 전부 통과다.

| 항목 | 실제 결과 |
|---|---:|
| 승인 run | 80/80 |
| Instant/Broadcast logical result·event·checkpoint·pose hash parity | 0 mismatch |
| max speed / acceleration / braking | 352.8 km/h / +18.0 / -50.0 m/s² |
| max attack start average / max per driver-lap | 0.08 / 1 |
| minimum same-pair cooldown | 67.6s |
| max logical contact | 2/race (0.2/race-lap) |
| body overlap / boundary / corridor conflict / crossing 전 swap | 0 |
| post-integrator correction / integrator identity violation | 0 / 0 |
| max retained rolling frame / pose | 128 / 2,560 |
| max broadcast runtime / peak RSS delta | 34.075s / 243,695,616B |
| close 후 frame/pose/cursor/client/task | 0 / 0 / false / 0 / false |

결정적 Gate 4R 상황 fixture 10개는 모두 통과했다. local-width 부족, corner phase, active
reservation, 제3 차량 occupancy와 실패 fallback을 실제 rejection/terminal event로 확인했으며,
terminal event 누락·중복은 0이다.

### 결정성·버전·benchmark

구형 post-integrator clamp, fixed-tick swap와 legacy-only 정의를 제거했다. reservation과 maneuver
의미 변경 때문에 `abstract_engine_version`을 `abstract-stage4-traffic-v1`에서
`abstract-stage4-traffic-v2`로 올렸다. 동일 session/circuit/seed의 version-only control은
1랩 Bahrain/RBR fixture에서 grid, finish order와 logical event sequence를 유지하고 hash만
version field에 따라 달라졌다(Bahrain `16506fd3…` → `79bae82…`, circuit id 4 fixture).
Stage 1 benchmark의 이번 실제 측정은 다음과 같다.

| 랩 | elapsed | logical ticks | checkpoints | events | retained collection | frame/state |
|---:|---:|---:|---:|---:|---:|---:|
| 10 | 15.998s | 7,873 | 202 | 209 | 139,888B | 0/0 |
| 57 | 94.162s | 46,924 | 1,142 | 874 | 739,136B | 0/0 |

두 benchmark 모두 첫/두 번째 hash 접근이 동일했고 `hash_equal=true`다. 57랩에는 기존 logical
incident 경로의 contact event가 포함될 수 있으나, 이번 단계에서 contact/spin/retirement를
확장하거나 balance하지 않았다.

### 비회귀 검증과 미실행

- Stage 4 correction/traffic: **21개 OK**; Stage 2 geometry + Stage 3 kinematics + API/WebSocket:
  **32개 OK**; FULL 관련 targeted track/session/foundation/tire 회귀: **72개 OK**; engine
  geometry/grid/RaceInfo/pit targeted 회귀: **27개 OK**.
- Stage 3 kinematics diagnostic Bahrain/RBR: **approval true**, post-integrator correction,
  integrator identity, reported acceleration identity 모두 `0`; frontend test **16개 OK**,
  lint/build **OK**, backend compileall **OK**, 문서 relative link **99개/누락 0**,
  `git diff --check` **OK**.
- Abstract 전체 기존 모듈은 **39개 중 3개 실패**했다. 실패는 구형 Stage C reversed-grid
  fixture의 corridor 존재 기대 1건과 Stage D의 기존 contact/damage 기대 2건이며, 모두 이번
  Stage 4의 post-integrator/fixed-swap 제거 또는 Stage 6 범위 contact 의미에 의존한다. 해당
  의미를 되살리거나 tolerance를 완화하지 않았다.
- 처음 시도한 여러 모듈 합본은 기존 FULL `test_p1_is_always_leader` 장기 실행에서 중단했다.
  **전체 Backend unittest discovery와 FULL 366개 전체 묶음은 이번 실행에서 통과로 기록하지
  않는다.** 수동 앱 관전도 실행하지 않았다.

### 판정

단계 4 최종 보완 판정은 **조건부 — 독립 검증 대기**다. Gate 4R-A~D와 80-run matrix는
통과했지만 위의 기존 Abstract Stage C/D 3개 비회귀 실패 및 전체 discovery/FULL 묶음 미실행이
남아 있다. 단계 5 pit route/box/merge, 단계 6 contact/spin/retirement/VSC/SC/restart와 단계 7
제품 패키징은 시작하지 않았다.

## 34. ABSTRACT 단계 4 조건부 검토 보완 — 전수 제3 차량 감사와 artifact 재현 경로 (2026-08-05)

독립 검토에서 지적된 Gate 4R 조건부 항목만 보완했다. 단계 5 pit route/box/merge, 단계 6
contact/spin/retirement/VSC/SC/restart와 단계 7 제품 패키징은 시작하지 않았다. FULL 물리·타이어·
회로 calibration은 변경하지 않았다.

### 보완 내용

- `audit_accepted_frame_reservations()`는 `third_vehicle_ids` 설명 목록에 의존하지 않고 accepted
  frame의 모든 비참여 차량을 stable ID 순서로 검사한다. admission 시점의 reservation arc 중간에
  이미 들어온 차량과 이후 swept occupancy를 같은 time+arc+lateral/body envelope으로 계측한다.
- reservation은 `approach`부터 deadline, fallback, rejoin 종료까지의 `8.0s` provisional horizon과
  accepted target-speed 기반 swept arc를 immutable DTO에 보존한다. `third_vehicle_ids`는 admission
  설명값으로만 유지하며 독립 accepted-frame 감사의 검사 universe가 아니다.
- Stage C의 구형 maneuver 허용 목록은 새 `approach`, `crossing_confirmed`, `clearance_confirmed`,
  `yield_or_abort`, `fall_back_to_safe_gap`, `rejoin` 상태를 검증하도록 갱신했다. 생산 seed에
  contact 발생을 요구하던 Stage D 테스트는 `force_contact`를 명시한 geometry-backed deterministic
  fixture로 바꿨다. 이 hook은 production RNG 경로에서 호출하지 않는다.
- `diagnose_abstract_stage4_traffic.py`가 `matrix`, `parity_fields`, `quality`, `resources`,
  `run_count` 집계를 직접 생성한다. `--matrix`는 circuit/seed case를 격리 child process로 실행하고
  stable order로 합치며, per-run broadcast runtime/RSS/rolling cleanup 측정은 유지한다. 따라서
  README의 단일 명령으로 권위 artifact를 재생성할 수 있다.
- 반복되는 immutable speed-knot distance axis는 plan 생성 시 한 번만 확정해 장거리 accepted
  forecast lookup 비용을 줄였다. 이는 수치·hash 계약을 변경하지 않는 구조적 최적화다.

### 80-run artifact 재생성 결과

통합 명령으로 Bahrain(id 3)·Red Bull Ring(id 4), seeds `0..9`, 10랩, 20대,
Instant/controlled Broadcast `1x/2x/5x`를 실행했다. artifact SHA-256은
`59874e16f5b2a95aefdffe763d8af25086ea138b05f303cec7e7b6cc5383bdd0`이다.

| 항목 | 실제 결과 |
|---|---:|
| 승인 run / 전체 run | 80 / 80 |
| Instant/Broadcast parity failure | 0 |
| zero-guard failure | 0 |
| close cleanup | 80/80 |
| max speed / acceleration / braking | 313.7355 km/h / +18.0 / -50.0 m/s² |
| actual admitted corridor conflict / accepted-frame third-car conflict | 0 / 0 |
| max corridor rejection reason `corridor_conflict` | 248 (rejection count이며 admitted conflict가 아님) |
| max retained rolling frame / pose | 128 / 2,560 |
| Broadcast runtime min / mean / max | 51.545s / 80.430s / 118.152s |
| Broadcast peak RSS delta max / mean | 243,744,768B / 75,169,519B |

이번 보수적 전수 감사 조건에서는 생산 RNG 경로의 accepted attack/pass/contact가 `0`이었다. 이는
제3 차량이 전체 reservation envelope에 들어올 가능성을 admission에서 거절한 결과다. crossing과
clearance 기반 실제 rank swap, 성공·실패 terminal event, cooldown 의미는 deterministic geometry
fixture에서 계속 검증한다. `min_cooldown_s`는 생산 accepted attack가 없어 `null`이며, fixture의
동일 pair logical cooldown 검증은 통과했다.

### Stage 1 benchmark 재검증

| 랩 | elapsed | logical ticks | checkpoints | events | retained collection | frame/state | canonical hash |
|---:|---:|---:|---:|---:|---:|---:|---|
| 10 | 30.451410s | 7,871 | 202 | 146 | 127,216B | 0/0 | `3ec9ea0122e73aef06da5905f780b186f49c63adc8d40d7992961e08e87d4e2c` |
| 57 | 201.822313s | 46,893 | 1,142 | 787 | 721,664B | 0/0 | `88c95cd30967514e073a40b3b57e0b48c86db20774adad189bb0915804ca7607` |

두 benchmark 모두 `hash_equal=true`; 첫/두 번째 hash 접근은 10랩 `1µs/0µs`, 57랩 `2µs/0µs`였다.
peak Python RSS는 각각 52.062MB(증가 4.641MB), 71.328MB(증가 23.938MB)였다.

### 검증 결과와 미실행

- Abstract 전체 discovery: **82개, OK**.
- Stage 4 correction/traffic: **24개, OK**. 여기에는 unlisted third-car audit, full reservation
  horizon, deterministic Stage C/D fixture와 diagnostic aggregate smoke test가 포함된다.
- frontend test **16개, OK**, lint **OK**, production build **OK**; backend `compileall` **OK**;
  문서 relative link **100개/누락 0**; `git diff --check` **OK**.
- Abstract API 회귀 **10개, OK**(57랩 controlled-clock 포함)를 분리 실행했다. 다만 관련 FULL
  표적 합본은 기존 장기 테스트에서 완료되지 않았다. `test_tire_thermal_foundation.py`의 17-lap
  equivalent 테스트와 `test_simulation_foundation.py`의 direct-file validator 테스트가 각각
  장시간 실행되어 수동 interrupt했다. 따라서 **관련 FULL 표적 합본 전체를 통과했다고 기록하지
  않는다.** 수동 앱 관전, 전체 Backend unittest discovery와 FULL 366개 전체 묶음도 미실행이다.

### 판정

조건부 검토의 제3 차량 universe 누락, reservation 범위 누락, artifact 재현 불가, 구형 test fixture
실패는 보완했다. 다만 생산 경로가 보수적 admission으로 attack/pass를 모두 거절하는 점과 장기
API/FULL 합본이 미완료인 점 때문에 단계 4는 **최종 승인 보류 — 조건부 유지**다. 독립 검증을 기다리며
단계 5는 시작하지 않는다.

## 35. ABSTRACT 단계 4 생산 교통 활성도 보완 — 시간 정렬 점유 모델 (2026-08-06)

독립 검증에서 확인된 생산 경로 `attack/pass=0` 원인을 수정했다. 기존 `v2`는 공격자·방어자가
8초 동안 지나갈 arc 전체를 같은 순간에 점유한 것으로 간주해, 실제로는 수백 m 떨어져 같은 속도로
이동하는 제3 차량까지 corridor conflict로 거절했다. 이 방식은 충돌에는 보수적이지만 정상적인
레이스 교통을 모두 제거하므로 승인 기준으로 사용할 수 없다.

### 구현 변경

- 생산 admission은 공격자·방어자·가장 가까운 앞/뒤 차량의 차체 위치를 동일한 미래 logical time에서
  비교한다. 0.5초 결정적 표본과 accepted 상대가속도 기반 interpolation guard를 사용하며, 8초
  deadline/fallback/rejoin horizon 자체는 유지한다.
- 순위상 더 먼 차량은 가까운 차량을 먼저 통과하지 않고 해당 corridor에 진입할 수 없으므로 admission
  forecast는 양쪽 인접 차량만 계산한다. accepted-frame 독립 감사는 기존대로 모든 비참여 차량을
  매 tick 검사한다.
- accepted-frame 감사는 매 tick의 실제 현재 점유를 검사한다. admission의 8초 예측을 매 tick 다시
  미래 충돌로 누적하지 않아, 변경된 target plan을 이미 발생한 충돌로 잘못 집계하지 않는다.
- reservation DTO에 `occupancy_model`을 추가했다. 기존 독립 fixture와 participant 상태가 없는 감사는
  `swept_envelope` fallback을 쓰며 생산 예약은 `time_aligned`를 명시한다.
- 전체 10-seed·10-lap 승인 매트릭스에서는 circuit별 Instant 합계에 attack과 completed pass가 각각
  최소 1회 있어야 한다. 따라서 안전 지표가 0이어도 생산 활동이 전무한 artifact는 승인되지 않는다.
- 결과 의미 변경을 반영해 `abstract_engine_version`과 진단 contract를
  `abstract-stage4-traffic-v3`로 올렸다.

### 실제 생산 회귀

20대, 10랩, 0.10초 tick의 실제 engine 경로를 fixture 주입 없이 실행했다.

| 회로 / seed | attack | completed pass | corridor/third/body conflict | 판정 |
|---|---:|---:|---:|---|
| Bahrain(id 3) / 0 | 6 | 1 | 0 / 0 / 0 | 통과 |
| Red Bull Ring(id 4) / 1 | 6 | 1 | 0 / 0 / 0 | 통과 |

두 실행 모두 admitted reservation conflict, reservation body violation, same-lane overlap,
crossing 전 rank swap과 post-integrator distance correction이 0이었다. Stage 4 correction 단위 계약
16개와 생산/aggregate 추가 회귀 2개는 함께 **18개 OK**, Stage 4 traffic 전체는 **11개 OK**다.
전체 Abstract discovery는 **85개, 165.223초, OK**, Abstract API/WebSocket 묶음은 57랩
controlled-clock 경로를 포함해 **10개, 523.108초, OK**다.
frontend test **16개 OK**, lint/build **OK**, backend compileall **OK**, 문서 상대 링크
**101개/누락 0**, 수정 파일 trailing whitespace 검사와 `git diff --check`도 **OK**다.

### v3 전체 매트릭스 승인

[`ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json`](ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json)을
`abstract-stage4-traffic-v3`으로 재생성했다. SHA-256은
`4946e797b0cf1e14b3cef0ffd33c5fc0c2164745d986bbc83ed252441213c519`이다.

| 항목 | v3 결과 |
|---|---:|
| 승인 run / 기대 run | 80 / 80 |
| Instant/Broadcast 1x·2x·5x parity mismatch | 0 |
| zero-guard / production activity failure | 0 / 0 |
| Bahrain Instant attack / completed pass | 63 / 7 |
| Red Bull Ring Instant attack / completed pass | 35 / 6 |
| Instant defense / abort / contact 합계 | 34 / 51 / 6 |
| max contact / race | 1 |
| minimum same-pair cooldown | 34.7s |
| max speed / acceleration / braking | 352.8km/h / +18.0 / -50.0m/s² |
| max retained rolling frame / pose | 128 / 2,560 |
| Broadcast runtime min / mean / max | 33.870s / 40.459s / 50.575s |
| Broadcast peak RSS delta max / mean | 243,417,088B / 75,132,109B |
| close cleanup | 80/80 통과 |

모든 run에서 corridor conflict, admitted reservation conflict, accepted-frame third-car conflict,
body/boundary/same-lane overlap, crossing 전 rank swap, post-integrator correction, cooldown violation,
frame/event 누락·중복이 0이다. 따라서 단계 4의 코드·자동 회귀·80-run artifact 판정은
**승인**으로 변경한다.

artifact 승격 후 JSON schema/승인 gate/SHA를 다시 읽어 검증했고, 문서 상대 링크
101개/누락 0, trailing whitespace와 `git diff --check`가 통과했다. Electron desktop 수명주기·진단
테스트도 **12개 OK**다.

전체 Backend/FULL discovery와 수동 패키지 앱 관전은 이번 `v3` 매트릭스와 별개의 제품 승인으로
남아 있다. 단계 5 pit route/box/merge는 아직 시작하지 않았다.

## 36. ABSTRACT_BROADCAST 단일 권위 cursor·선행 버퍼 전환 (2026-08-06)

제품 방송 모드의 긴 시작 대기와 이중 계산 원인을 제거했다. 기존 경로는 setup 요청 안에서 57랩
전체 `AbstractRaceResult`를 먼저 계산한 뒤, 방송을 시작할 때 별도의 traffic cursor로 같은 경기를
다시 진행했다. 그래서 사용자는 첫 화면을 보기 전에 전체 계산 시간을 기다렸고, CPU·메모리도
결과 계산과 재생 경로에서 중복 사용됐다.

### 제품 구조

- `ABSTRACT_BROADCAST` setup은 예선과 그리드만 확정한다. 전체 finish order·event·canonical hash는
  시작 전에 만들지 않으며, 방송이 끝난 뒤 단일 cursor의 결과로 확정한다.
- 하나의 `AbstractTrafficSimulationCursor`만 경기 상태의 권위다. producer는 별도 thread batch에서
  0.10초 logical tick을 생산하고 consumer는 `InterruptibleReplayClock`에 맞춰 동일 tick을 화면에
  공개한다. 별도 precompute 결과와 replay cursor는 존재하지 않는다.
- 시작 기준은 20 tick(2 logical seconds), 선행 버퍼 상한은 300 tick(30 logical seconds), producer
  batch는 10 tick이다. bounded queue가 가득 차면 producer가 대기하므로 무제한 메모리 증가가 없다.
- pause와 배속 변경은 공개 clock만 제어하고 생산된 tick의 의미·순서·최종 결과를 바꾸지 않는다.
  마지막 tick과 미전송 event를 모두 전달한 뒤에만 `race_end`를 보낸다.
- `ABSTRACT_INSTANT`는 결과를 즉시 요구하는 별도 제품 계약이므로 기존 전체 계산 경로를 유지한다.

### 전송·프런트·진단 계약

- 20대 pose는 FULL과 같은 `F1P1` binary packet으로 전송한다. 한 tick은 491B이며, JSON pose 배열을
  매 tick 생성하던 제품 경로를 제거했다. dashboard state는 5Hz, pose는 10Hz다.
- 프런트엔드는 기존 typed-array ring buffer와 보간 재생 경로를 그대로 사용한다. 버퍼 보존 범위는
  약 2.5초이고 장기 frame/state를 축적하지 않는다.
- setup 응답은 진행 중임을 나타내는 `buffered_broadcast_running`과 grid/total laps를 제공한다.
  finish order·event count·canonical hash는 완료 전 권위값이 아니므로 빈 값 또는 `null`이다.
- desktop diagnostics는 ABSTRACT 세션의 client, WebSocket, producer task, queue 크기·상한, 표시 tick과
  생산 tick을 포함한다. 종료 시 producer/consumer task, queue, pending event와 cursor를 정리한다.

### 측정 결과

fresh Bahrain 57랩 setup에서 전체 레이스를 선계산하지 않는 것을 확인했다.

| 항목 | 결과 |
|---|---:|
| setup 응답 및 20-tick 시작 버퍼 준비 | 0.278529초 |
| client 연결 직후 최초 binary pose | 0.294ms |
| 첫 이동 frame(tick 1) | 112.224ms |
| 시작/최대 선행 버퍼 | 2.0초 / 30.0초 |
| 300-tick queue 충전 전/후 Python RSS | 58.828MB / 67.734MB |
| queue 충전 RSS 증가 | 8.906MB |

57랩 setup 테스트는 기존 전체 계산 함수를 호출하면 즉시 실패하도록 patch한 상태에서도 3초 이내
통과했다. binary packet은 magic/version/tick/car count와 20대 pose를 해석해 491B 계약을 검증했다.

### 검증 및 남은 승인

- Abstract/API: **15개, 282.247초, OK**. 57랩 controlled broadcast, binary frame, 동적
  checkpoint/event, race end, 종료 cleanup과 desktop diagnostics를 포함한다.
- Abstract 전체 discovery: **85개, 164.679초, OK**.
- FULL session 표적 회귀: **13개, OK**.
- frontend test **16개 OK**, lint **OK**, production build **OK**; backend compileall과
  `git diff --check`도 **OK**다.

전체 Backend/FULL discovery는 이번 변경에서 실행하지 않았다. 2026-08-07 macOS arm64 앱을 새로
패키징했고 packaged Python의 새 broadcast module import, desktop test 12개와 전체 bundle의 엄격
ad-hoc 서명 검증을 통과했다. 실제 앱 1x 관전·완주 승인은 아직 수행하지 않았다. 이 절에서 남겼던
pit 명령의 선행 tick 무효화·checkpoint 재생산 계약은 37절에서 구현했다. SC·날씨처럼 추가로 결과를
바꾸는 명령도 같은 권위 경계를 따라야 한다. 단계 5 pit route/box/merge는 아직 시작하지 않았다.

## 37. ABSTRACT_BROADCAST 사용자 전략 개입 1차 — 결정적 buffer rewind (2026-08-08)

방송 모드를 배속·일시정지만 가능한 관전 경로에서 player team의 기본 전략 개입이 가능한 경로로
확장했다. 범위는 pace, pit call, pit cancel이며 SC/VSC·날씨·레이스 디렉터 명령은 포함하지 않는다.
전송 계약은 `abstract-interactive-buffered-binary-v2`다.

### 명령과 결과 권위

- 기존 FULL과 같은 `set_pace_mode(CONSERVE/STANDARD/ATTACK)`와
  `pit_call(SOFT/MEDIUM/HARD)` 명령을 사용한다. 명령 대상은 setup에서 선택한 player team 소속
  driver로 제한하며 다른 팀 명령은 거절한다.
- pace는 차량별 결정적 baseline에 `0.985/1.000/1.015`를 적용한다. accepted longitudinal
  acceleration/deceleration guard는 그대로 사용하므로 속도가 순간 이동하지 않는다. 거리당 wear에는
  `0.97/1.00/1.04`를 적용해 SAVE와 ATTACK의 기본 trade-off를 둔다.
- pit call은 현재 표시 시점에서 다음 가능한 lap의 기존 자동 stop을 교체하고, 회로 weekend
  nomination으로 역할을 실제 C1~C5 compound에 변환한다. pit sequence 시작 전에는 `pit_cancel`로
  원래 자동 plan을 복원할 수 있다.
- 각 명령은 sequence·표시 logical time·payload를 `AbstractCommandRecord`에 기록한다. strategy
  event와 command log는 최종 canonical result hash에 포함되므로 같은 seed와 같은 명령 순서는 같은
  결과를 만들고, 명령이나 순서가 달라지면 별도 결과 권위가 된다.

### 미공개 버퍼 무효화 계약

producer는 화면보다 최대 300 tick(30초) 앞에 있을 수 있으므로 미래 cursor에 값을 직접 쓰지 않는다.
명령 처리 순서는 다음과 같다.

1. consumer와 producer의 새 진행을 잠시 막고 현재 표시 tick을 고정한다.
2. 가장 가까운 100-tick(10초) runtime checkpoint로 cursor를 복원한다.
3. 같은 RNG state로 표시 tick까지 최대 99 tick을 다시 계산한다.
4. 명령을 기록하고 아직 client에 공개하지 않은 queue를 전부 폐기한다.
5. generation을 올려 동시에 끝난 이전 producer batch를 거절하고 다음 tick부터 새 branch를 생산한다.

checkpoint는 차량·순위·pit·maneuver·cooldown·metric·RNG state와 result collection 길이만 보존하며,
불변 track geometry·kinematic plan은 공유한다. 프런트에는 미래 server queue를 미리 보내지 않으므로
이미 표시한 pose를 되감거나 stream epoch를 바꿀 필요가 없다. 다음 binary tick은 기존 표시 frame에
연속해 도착한다.

### UI와 측정

- 방송 모드에서도 player driver `STRATEGY` 패널을 표시한다. `SAVE/STD/ATK`, 회로 지명 타이어를
  선택한 `BOX BOX`, pit 진입 전 `CANCEL BOX`를 제공한다.
- 57랩 Bahrain에서 server queue 300 tick과 runtime checkpoint를 채운 fresh process의 peak RSS
  증가는 `17.750MB`였다. 초기 deep-copy 구현의 `86.281MB` 증가는 불변 plan 공유와 checkpoint 간격
  보정으로 폐기했다.
- 표시 tick 92, producer tick 391에서 299개 미래 tick을 폐기한 표본의 명령 처리 시간은
  `152.846ms`였다. 이후 pose missing/duplicate는 `0/0`, max frame gap은 `1`이다.

### 검증과 남은 범위

- Abstract 전체 discovery: **87개, 171.295초, OK**. runtime checkpoint 동일 재생과 동일
  command sequence canonical hash 결정성 테스트를 포함한다.
- Abstract/API/WebSocket: **17개, 288.182초, OK**. 57랩 controlled broadcast, player 권한,
  pace/pit/cancel, unpublished buffer 폐기와 live pose 연속성 테스트를 포함한다.
- FULL session 표적 회귀 **13개 OK**, frontend test **16개 OK**, lint/build **OK**, desktop test
  **12개 OK**, backend compileall과 `git diff --check` **OK**다.

현재 pit 동작은 기존 추상 logical `entry/lane/stop/exit` 단계에 명령을 연결한 것이다. 실제 pit
entry route, box 위치와 본선 merge 교통을 갖춘 단계 5 구현을 대신하지 않는다. 전체 Backend/FULL
discovery와 SC/VSC·날씨 개입은 별도 승인으로 남아 있다. 2026-08-08 새 macOS arm64 패키지에
`abstract-interactive-buffered-binary-v2`, 100-tick checkpoint와 `CANCEL BOX` UI가 포함된 것을
패키지 내부 Python import·frontend asset 검사로 확인했고 전체 bundle 엄격 ad-hoc 서명도 통과했다.
실제 앱 1x 전략 개입·완주 관전은 아직 수행하지 않았다.

## 38. 제품 엔진 방향 확정과 ABSTRACT standing-start 보완 (2026-08-09)

### 제품 방향

레이싱 매니지먼트 제품의 기본 엔진은 장기적으로 물리 제약형 확률·논리 모델인
`ABSTRACT`를 목표로 한다. `FULL`은 삭제하거나 즉시 대체하지 않고 차량·서킷·타이어 보정,
비교 benchmark와 회귀 기준 엔진으로 유지한다. 단계 5 pit route/box/merge, 단계 6 사건·SC·restart,
단계 7 패키지 수동 관전·개입 승인이 끝나기 전에는 현재 기본 모드와 FULL 계약을 변경하지 않는다.

### 출발 이상 동작의 원인

기존 ABSTRACT standing start는 두 계약이 충돌했다.

- 그리드 중심 간격은 8m이고 차량 길이를 제외한 종방향 body gap은 약 3m인데, lights-out 직후부터
  일반 주행의 5m body gap과 reaction buffer를 적용했다. 그 결과 P1만 먼저 움직이고 후속 차량은
  앞차가 멀어질 때까지 한 대씩 정지하는 직렬 출발이 됐다.
- 모든 차량의 ±2.35m 그리드 횡오프셋은 종방향 속도와 무관하게 2초 동안 중앙으로 수렴했다.
  정지 차량에도 횡속도가 생겼고 pose heading이 그 횡속도를 따라 ±90°까지 돌아가 차량이 옆으로
  미끄러지는 것처럼 보였다.

### 수정 계약

- 1.0초 grid hold 뒤 첫 accepted tick에 20대가 함께 종방향 이동한다.
- lights-out 뒤 1.5초는 그리드 횡위치를 유지하고, 이후 주행 중 2.5초 quintic trajectory로 레이싱
  라인에 합류한다. 출발 전용 최소 간격·reaction envelope는 8초 동안 일반 레이스 값으로 smoothstep
  전환한다.
- 출발 전환 중 accepted speed가 0이면 lateral trajectory의 시간축도 같은 tick만큼 정지한다. 따라서
  정지 차량의 횡속도와 횡가속도는 0이고, 재출발 시 기존 위치에서 연속적으로 합류를 재개한다.
- 초기 차량군이 촘촘한 상태에서는 전환이 끝날 때까지 일반 추월 예약을 시작하지 않는다.
- 추월 중에는 단순 순위 인접차가 아니라 두 대 묶음의 실제 교통 경계를 사용한다. attacker는 defender
  앞 차량을, 후속 차량은 두 participant 중 뒤쪽 차량을 추종한다. lateral body clearance가 생기기
  전에는 defender가 attacker의 pre-commit 경계로 남는다.
- 결과 의미 변경을 반영해 `abstract_engine_version`을 `abstract-stage4-traffic-v4`로 올렸다. FULL
  엔진과 물리 기본값은 변경하지 않았다.

### 검증

- Bahrain·Red Bull Ring standing-start 계약: hold 중 20대 정지·횡속도 0, 첫 lights-out tick에
  20대 전부 전진, 정지 중 횡이동 0, 5초까지 body overlap 0.
- Stage 3 kinematics + Stage 4 traffic/correction: **44개, 79.268초, OK**.
- 단계 1~4 Abstract 전체 5개 모듈: **88개, 171.076초, OK**.
- Abstract setup/API 표적 회귀 **3개, 13.307초, OK**; frontend test **16개**, lint와
  production build **OK**; Abstract compileall과 `git diff --check` **OK**.
- Bahrain seed 0·Red Bull Ring seed 1의 10랩 생산 경로에서 attack와 completed overtake가 발생했고,
  corridor/admitted reservation/third-car/body/same-lane overlap, crossing 전 rank swap,
  post-integrator correction은 모두 0이다.
- 0.05/0.10/0.50초 3랩 표본은 모두 완주했고 body overlap 0이다.

기존 `ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json`은 v3 구현의 승인 artifact이므로 과거 근거로 유지한다.
v4 전체 80-run matrix 재생성, 전체 Backend/FULL discovery와 실제 패키지 1x 출발 관전은 아직
실행하지 않았다.

2026-08-10 macOS arm64 패키지를 `desktop/out/F1 Race Manager-darwin-arm64/`에 재생성했다.
앱과 내장 Python 실행 파일은 arm64이고 전체 bundle의 엄격 ad-hoc 서명 검증을 통과했다. 패키지
내부 Python에서 `abstract-stage4-traffic-v4`와 `1.0/1.5/2.5초` 출발 계약을 import했으며, Bahrain
스모크에서 grid hold 정지 20대, 첫 launch tick 이동 20대, launch 횡이동 0대, body overlap 0을
확인했다. 실제 Electron 창에서의 1x 육안 관전·완주 승인은 여전히 남아 있다.

## 39. ABSTRACT 진행률 권위 커널 1단계 (2026-08-11)

기존 `abstract-stage4-traffic-v4`를 직접 축소하지 않고 별도 opt-in 경로
`abstract-progress-race-v1`을 추가했다. 제품 기본 `run_race`, broadcast, API와 FULL은 변경하지
않았다.

### 권위와 경계

- 입력 권위는 기존 immutable `AbstractSessionSnapshot`, timing segment, 차량·드라이버 성능,
  타이어와 환경값을 재사용한다.
- 출력 권위는 `race_distance_m`, total progress, lap/segment, 논리 속도, 순위, 앞차 간격과
  exact finish time뿐이다. world x/y, heading, 횡오프셋, 가속도, force와 slip은 상태에 없다.
- 1초 standing hold 뒤 20대가 같은 logical instant부터 이동한다. grid는 P1 -8m부터 차량마다
  8m 간격의 unwrapped distance로 표현한다.
- 한 tick 안에서 남은 시간을 segment boundary까지 소비하고 다음 segment로 넘기는 event-driven
  적분을 사용한다. finish time은 tick 끝으로 양자화하지 않고 실제 boundary crossing time으로
  기록한다.
- `integration_tick_count`는 진단값일 뿐 canonical hash에서 제외한다. 따라서 계산 cadence가 달라도
  동일한 논리 결과를 표현한다.
- `AbstractRaceEngine.create_progress_cursor()`와 `run_progress_race()`로만 명시적으로 실행한다.
  기존 Stage 4 결과 버전과 canonical hash 계약은 그대로 유지한다.

### 검증 결과

- 신규 progress 테스트 **7개, OK**: 20대 동시 출발, 거리 단조성, 순위 연속성, pose/force 필드
  부재, 차량 성능 pace 반영, 57랩 bounded state, 기존 v4 기본 경로 보존.
- 0.05/0.10/0.50초 3랩에서 finish order, exact classification/finish time과 canonical hash가
  완전히 동일하다. 통합 Abstract 실행은 기존 88개를 포함해 **94개, 173.756초, OK**였고 이후
  차량 성능 회귀를 추가한 progress 모듈 **7개, 1.153초, OK**다.
- Abstract setup/API 표적 **3개, 13.332초, OK**; FULL session/grid/pit 표적 **4개,
  7.524초, OK**.

동일 Bahrain seed 42·10랩 비교에서 progress v1은 **0.097340초/1,680 tick**, Stage 4 v4는
**22.804289초/7,892 tick**으로 이 표본에서는 약 **234.3배** 빨랐다. 단, v1에는 교통·추월·피트·사고가
없으므로 속도 비교는 진행률 기반의 비용 상한을 확인하는 실험이며 기능 동등성 비교가 아니다.

Bahrain 57랩·1초 cadence는 wall **1.710368초**, logical duration **4,779.982250초**, 4,780 tick으로
완료했다. `tracemalloc` peak/retained는 **535,543B/435,389B**이며 cursor는 현재 20대 상태만 보존한다.

### 판정과 다음 단계

1단계 판정은 **승인**이다. 진행률 권위, 성능 입력, standing start, tick 불변 결과와 bounded state가
성립했다. 다만 완주 순서는 Stage 4와 크게 다르며 이는 아직 traffic cost·추월·피트·사건이 없기
때문이다. 다음 단계에서는 이 커널 위에 다음 순서로 논리 기능을 추가한다.

1. gap/traffic group과 `FOLLOW → ATTACK → SIDE_BY_SIDE → CLEARANCE` 상태기
2. 시간·거리 불변 hazard와 lockup/run-wide/contact/spin/retirement 결과
3. pit progress/merge와 타이어 stint 갱신
4. 순위·target gap 기반 SC/VSC/restart
5. 기존 binary pose broadcast로 변환하는 비권위 presentation adapter

이 다섯 항목과 seed/명령 rewind matrix가 승인되기 전에는 progress v1을 제품 기본 모드로 전환하거나
Stage 4/FULL 코드를 삭제하지 않는다.

## 40. ABSTRACT 진행률 권위 커널 2단계 — 논리 교통·사고 (2026-08-11)

`abstract-progress-race-v2`는 v1의 진행률 권위 위에 좌표·횡운동·충돌 형상을 되살리지 않고 논리
교통과 확률 사고를 추가했다. 이 경로는 계속 opt-in이며 기존 `abstract-stage4-traffic-v4`, binary
broadcast, API 기본값과 FULL은 변경하지 않았다.

### 논리 계약

- 외부 integration tick과 별개인 고정 **0.50초 decision boundary**에서만 교통·사고 RNG를 소비한다.
  따라서 0.05/0.10/0.50초 호출 cadence가 달라도 완주 결과·이벤트·지표·canonical hash가 같다.
- 공개 진행 거리의 역행을 막는 간격 제한은 고정 **0.05초 motion boundary**에서 계산하며 RNG를
  소비하지 않는다. 외부 tick을 0.05초로 읽는 3랩 회귀에서도 차량별 거리 역행은 0이다.
- 순위 변경은 `FOLLOW → ATTACK → SIDE_BY_SIDE → CLEARANCE` 상태를 거친 성공 maneuver의 atomic
  rank swap으로만 발생한다. 실패는 cooldown으로 끝나며 이벤트 없는 순위 교환은 허용하지 않는다.
- 차량 간격은 logical race distance로 제한하고, 진행 거리는 직전 decision boundary보다 뒤로 가지
  않는다. world pose, 차체 overlap, lateral corridor와 force는 결과 권위에 없다.
- 사고 hazard는 segment 유형, 드라이버 consistency, 날씨, 타이어 온도 band와 traffic state를
  사용한다. target 차량과의 gap이 가까울수록 hazard가 증가하며 결과는 lockup, run-wide, spin,
  contact와 경미한 누적 damage로 표현한다.
- `enable_traffic=False`, `enable_incidents=False`로 v1 pace 기준을 재생할 수 있다. retirements,
  pit/stint, SC/VSC/restart와 presentation adapter는 아직 이 커널에 없다.

### 검증과 성능

- 신규 progress 계약 **11개, 5.554초, OK**: standing start, 0.05초 공개 거리 단조성,
  pose/force 부재,
  57랩 bounded state, 차량 성능 pace, explicit maneuver chain, 사고 context, 기능 비활성 기준과
  tick 불변 결과를 확인했다.
- 기존 ABSTRACT 전체 **99개, 174.125초, OK**, API 전체 **17개, 335.667초, OK**, FULL
  session/grid/pit 표적 **4개, 9.613초, OK**다. abstract compileall과 `git diff --check`도 통과했다.
- Bahrain seed 42·10랩은 attack 79회, completed overtake 19회, incident 7회였고 hidden rank swap과
  distance reversal은 모두 0이다.
- Bahrain seed 42·57랩은 attack 273회, completed overtake 96회, incident 20회, logical duration
  4,796.515468초이며 hidden rank swap과 distance reversal은 모두 0이다. 사건은 lockup 7,
  run-wide 9, spin 4였고 이 seed에서는 contact가 없었다.
- 57랩 일반 실행은 v1 기준 **1.707815초**, v2 **2.365759초**였다. `tracemalloc` 계측 실행의 v2
  peak/retained는 **905,936B/853,379B**이며 840개 logical event와 현재 20대 tick만 보존했다.

### 판정과 다음 단계

2단계의 핵심 구조는 **승인**이다. 다만 수치는 게임 보정값이며 실제 시즌/서킷별 overtake·incident
분포와 아직 대조하지 않았다. `spacing_constraint_count`는 57랩에서 61,903회로, 좌표 충돌 비용을
제거한 대신 0.5초 논리 경계에서 follower 상태로 판정된 차량-판단 수다. 후속 단계에서는 이를 단순 횟수가
아닌 누적 traffic time/cost로 바꾸고 다음 순서로 확장한다.

1. pit entry/stop/exit progress와 타이어 stint·compound 갱신
2. retirement 및 contact pair 결과
3. 순위·target gap 기반 SC/VSC와 restart
4. 사용자 명령 checkpoint/rewind 및 비권위 presentation adapter

이 항목과 실제 제품 관전 승인이 끝나기 전에는 progress v2를 기본 엔진으로 선택하거나 기존
Stage 4/FULL을 삭제하지 않는다.

## 41. ABSTRACT 진행률 권위 커널 3단계 — 피트·타이어 stint (2026-08-11)

`abstract-progress-race-v3`는 v2의 논리 교통·사고 위에 진행률 기반 pit route와 타이어 stint를
추가했다. 기존 `abstract-stage4-traffic-v4`, broadcast/API 기본 경로와 FULL은 변경하지 않았고,
progress 경로에서도 `enable_pit=False`로 v2 교통·사고 기준을 재생할 수 있다.

### 피트·타이어 계약

- 기본 전략은 5랩 이상 경기에서 전 차량 1회 정차이며 팀별 pit RNG로 1.9~2.3초 service를 만든다.
  호출자는 `pit_strategy={driver_id: (lap, ...)}`로 0회·1회·다중 정차를 명시할 수 있다.
- 피트 상태는 `entry → lane → stop → exit → none`이며 `pit_requested`, `pit_lane_entry`,
  `pit_stop_started`, `pit_stop_completed`, `pit_exit` 이벤트를 순서대로 한 번씩 생성한다.
- snapshot canonical 계약에는 아직 실제 회로별 pit-loss/entry/exit anchor가 없다. 기존 snapshot과
  Stage 4 hash를 바꾸지 않기 위해 v3는 base lap time에서 provisional pit transit을 산출하고,
  진입 위치에서 0.12랩 길이의 별도 logical route progress를 사용한다. 이는 실제 pit geometry가 아니다.
- 첫 정차는 현재 C1~C5에서 한 단계 단단한 compound와 `MEDIUM`, 두 번째 정차는 다시 한 단계
  단단한 compound와 `HARD`를 적용한다. service 시 wear를 0으로 만들고 새 stint를 원자적으로
  시작하며 이후 race distance로 wear/stint lap을 갱신한다.
- wear는 1랩 이후 랩당 `0.002 / tire_management` pace 비용을 적용하고 최대 10%로 제한한다.
  이 수치는 공식 F1 값이 아닌 게임 보정 초안이다.
- pit 또는 incident 때문에 차량이 실제 진행 거리상 앞서면 `pit_order_changed` 또는
  `incident_order_changed` 이벤트와 함께 순위를 교환한다. active maneuver 사이에 pit 차량을
  삽입하지 않으며 이벤트 없는 rank swap과 거리 역행은 허용하지 않는다.

### 검증과 진단

- progress 계약 **14개, 8.914초, OK**: 20대 완전한 pit timeline, C3→C2 및
  C3→C2→C1 다중 stint, service 후 wear 재시작, 잘못된 전략 거부, 0.05/0.10/0.50초 pit 결과
  parity와 공개 거리 단조성을 확인했다.
- 기존 ABSTRACT 전체 **102개, 194.101초, OK**. Abstract/API 표적 3개와 FULL
  session/grid/pit 표적 4개는 합계 **7개, 24.002초, OK**다. 최종 progress 모듈, compileall과
  `git diff --check`도 다시 통과했다.
- Bahrain·Red Bull Ring, seed 0~4, 10랩 총 10회에서 매 경기 pit 완료 20, unique finish 20,
  hidden rank swap 0, distance reversal 0이었다.
- Bahrain seed 42·10랩은 pit 완료 20, pit rank change 148, logical duration 878.402336초였다.
- Bahrain seed 42·57랩은 pit 완료 20, pit rank change 147, incident rank change 2,
  attack/completed overtake/incident `206/65/33`, logical duration 4,984.428723초, logical event
  901개이며 hidden rank swap과 distance reversal은 0이다.

### 성능과 전략 효과

동일 Bahrain seed 42의 일반 실행 결과는 다음과 같다.

| 경로 | 10랩 | 57랩 |
|---|---:|---:|
| v2 기준 (`enable_pit=False`) | 0.468522초 | 2.591616초 |
| v3 자동 pit·stint | 0.518654초 | 2.893557초 |

57랩 `tracemalloc` 실행의 peak/retained는 **1,020,661B/972,428B**이며 현재 20대 tick과
901개 logical event를 보존했다. 같은 57랩에서 v2 무마모 기준 logical duration은
4,796.515468초, v3 마모 활성·무정차는 5,075.160625초, 자동 1회 정차는 4,984.428723초였다.
따라서 현재 초안에서도 정차가 무정차보다 약 90.7초 빠르며 전략 선택이 결과에 영향을 준다.

### 판정과 다음 단계

3단계 핵심 구조는 **승인**이다. 다만 pit route 0.12랩, pit-loss, wear와 compound 선택은
provisional이며 회로 nomination과 실제 pit profile을 snapshot의 별도 canonical 입력으로 설계해야
한다. 다음 단계는 다음 범위로 제한한다.

1. contact pair 결과, damage escalation과 retirement
2. 사건 심각도·정차 차량·순위 gap 기반 VSC/SC 발동
3. pit 중 SC queue와 pit-exit merge 순위 계약
4. restart ordering 및 사용자 명령 checkpoint/rewind

SC/restart와 제품 presentation 승인이 끝나기 전에는 progress v3를 기본 엔진으로 전환하거나 기존
Stage 4/FULL을 삭제하지 않는다.

## 42. ABSTRACT 진행률 권위 커널 4단계 — 접촉·손상·퇴역 (2026-08-11)

`abstract-progress-race-v4`는 v3의 진행률·교통·피트 권위 위에 차량 쌍 접촉, 누적 손상과
퇴역 분류를 추가했다. 좌표 충돌체나 힘 기반 충돌 해석은 되살리지 않았으며 기존
`abstract-stage4-traffic-v4`, broadcast/API 기본 경로와 FULL은 삭제하거나 교체하지 않았다.
`enable_incidents=False`이면 일반 사건뿐 아니라 차량 쌍 접촉 hazard도 차단되어 이전 무사고
기준 실행을 보존한다.

### 접촉·퇴역 계약

- 접촉 후보는 `side_by_side`인 논리 maneuver 쌍으로 한정한다. 0.5초 결정 경계마다 트랙 추월
  난이도와 두 드라이버 consistency로 hazard를 계산하고, 독립 contact RNG stream으로 결과와
  심각도를 결정한다. 같은 입력·seed에서 tick 크기와 무관하게 같은 사건을 만든다.
- 접촉은 두 차량 모두에 손상을 누적하고 `contact_started`에 두 driver id, 각 손상값, 심각도와
  지속 시간을 기록한다. 손상은 이후 pace multiplier를 낮추며, 누적 손상·심각도·reliability로
  퇴역 확률을 계산한다. 현재 계수는 공식 사고율이 아닌 게임 보정 초안이다.
- 퇴역 차량은 그 시점의 `race_distance_m`에서 정지하고 traffic·pit target을 해제한다.
  `driver_retired`는 이유, 거리와 손상을 기록한다. 최종 classification은 모든 완주자를 먼저
  finish time 순으로, DNF를 그 뒤에 완주 거리·퇴역 시간·grid index 순으로 배치한다.
- 결과 계약은 각 차량의 `status`, `race_distance_m`, 선택적 `finish_time_s`,
  `retirement_time_s`, `retirement_reason`을 포함한다. 20대 전원이 정확히 한 번 분류되어야 하며
  완주자 뒤에만 DNF가 올 수 있다.
- 피트 출구에는 짧은 논리 merge grace를 두어 별도 pit route에서 본선으로 복귀할 때 이벤트 없는
  순위 역전이나 spacing correction이 생기지 않도록 했다. incident로 이미 명시적 순위 변경이
  발생한 경우 이어지는 maneuver 종료가 같은 교환을 중복 집계하지 않는다.

### 검증과 진단

- progress 계약 **17개, 11.898초, OK**: 강제 접촉에서 양쪽 손상·pace 저하·정지 거리·DNF
  분류를 확인했고, 0.10/0.50초에서 classification/event/metric/hash가 완전히 같았다.
  `enable_incidents=False`의 pair hazard 차단도 별도 회귀로 고정했다.
- 기존 ABSTRACT 전체 **105개, 188.744초, OK**. Abstract/API 표적 3개와 FULL
  session/grid/pit 표적 4개는 합계 **7개, 19.067초, OK**다.
- Bahrain·Red Bull Ring, seed 0~9, 10랩 총 20회에서 명시적 pair contact는 32건이었다.
  모든 실행에서 classification 20/unique 20, hidden rank swap 0, distance reversal 0이었다.
  이 짧은 표본에서는 DNF가 없었으며 이는 강제 퇴역 회귀와 별개로 확률 보정이 더 필요함을 뜻한다.
- Bahrain seed 42·57랩에서는 pair contact 3, retirement 1(driver 3), classification 20,
  hidden rank swap 0, distance reversal 0이었다. pit 전 퇴역 때문에 pit 요청·완료는 19였고
  logical duration은 5,353.959150초, event는 839개였다.

### 성능과 메모리

Bahrain seed 42의 일반 실행은 10랩 **0.658648초/1,808 tick**, 57랩
**3.807126초/10,708 tick**이었다. 57랩 `tracemalloc` 실행의 peak/retained는
**1,358,477B/510,611B**이며 종료 후 현재 20대 tick과 839개 logical event만 결과에 남는다.
접촉·손상으로 경주 논리 시간이 늘어날 수 있으므로 v3와의 wall-time 수치만으로 기능 비용과 경기
길이 변화를 분리해 해석해서는 안 된다.

### 판정과 다음 단계

4단계 접촉·손상·퇴역의 구조와 결정론 계약은 **승인**이다. 다만 자연 표본의 접촉·DNF 빈도와
손상 pace 비용은 provisional이며 더 큰 seed/서킷/랩 매트릭스로 게임 밸런스를 승인해야 한다.
다음 단계는 사건 심각도·정지 위치·순위 gap 기반 VSC/SC 발동, pit 중 queue와 pit-exit merge,
restart ordering 및 사용자 명령 checkpoint/rewind다.

SC/restart와 제품 presentation 승인이 끝나기 전에는 progress v4를 기본 엔진으로 전환하거나 기존
Stage 4/FULL을 삭제하지 않는다.

## 43. ABSTRACT 진행률 권위 커널 5단계 — local yellow·VSC·SC·restart (2026-08-11)

`abstract-progress-race-v5`는 v4 사건 결과를 경기 통제 상태기로 연결했다. 기존 Stage 4/FULL과
현재 제품의 ABSTRACT instant/broadcast 기본 경로는 바꾸지 않았다. 이 단계의 권위는 진행 거리,
순위, gap, pit 상태와 logical event이며 차량 pose나 물리 속도는 결과를 결정하지 않는다.

### 경기 통제 계약

- 낮은 심각도의 접촉과 일부 spin/run-wide는 `local_yellow_started`와 hazard duration을 남기되
  전체 race state는 green으로 유지한다. 접촉 심각도 0.65 이상 또는 고심각도 spin/run-wide는
  VSC, 접촉 심각도 0.92 이상이나 실제 퇴역은 SC를 요청한다. 임계값은 게임 보정 초안이다.
- VSC는 15초 동안 전 차량의 정상 pace 상한을 65%로 제한하고 정상 추월 생성을 금지한다.
  발동 당시 진행 중인 공격은 `attack_failed(reason=race_control_caution)`로 명시적으로 닫고,
  종료 뒤 3초간 새 공격을 억제한다.
- SC는 `deploying → catch_up → queue_formed → restart_ready → in_this_lap → green` 상태를
  분리한다. 각 전환은 별도 logical event이며 queue가 실제 tolerance에 들어오기 전에는 다음
  단계로 넘어가지 않는다.
- SC 대열은 현재 classification order를 권위로 사용한다. leader는 제한 pace로 진행하고 후미는
  목표 간격까지 더 빠른 제한 pace로 따라오므로 위치 clamp로 순간 재배열하거나 정지시키지 않는다.
  기존 랩 차이는 driver별 `lap_deficit`으로 고정해 lapped car를 자동 언랩하지 않는다.
- pit 차량은 대열 계산에서 잠시 제외되고 기존 `pit_order_changed` 이벤트로 손익을 확정한 후
  pit exit에서 다시 SC queue에 들어간다. 정상 추월·incident pass는 caution 중 금지된다.
- `ProgressRuntimeCheckpoint`는 snapshot hash, 랩 수, tick 간격, 차량·순위·경기 통제·RNG·event
  경계를 함께 저장한다. 같은 cursor의 미공개 미래를 폐기하고 복원할 수 있으며 다른 snapshot이나
  tick 계약에는 적용할 수 없다.
- 진행률 tick은 `race_control_state`와 `race_control_phase`를 노출한다. 당시에는 presentation
  adapter가 없었으며 이후 44절에서 제품 방송과 화면 계약에 연결했다.

### 결정적 시나리오

- 강제 VSC는 logical time 20초에 발동해 35초에 green으로 복귀했고, 전 구간에서 순위가
  고정되고 주행 차량 표시 속도가 0.1m/s 아래로 떨어지지 않았다.
- 강제 SC는 20초 발동 후 `deploying/catch_up/queue_formed/restart_ready/in_this_lap`을 거쳐
  58초에 재시작했다. 0.10초와 0.50초 tick의 classification, event, metric과 canonical hash가
  완전히 같았다.
- SC catch-up 도중 checkpoint를 저장해 완주한 뒤 복원·재실행한 결과 classification, event,
  metric, tick count와 hash가 동일했다. 다른 snapshot의 checkpoint 적용은 거부했다.
- pit 진행 중 SC를 강제한 20대 2랩 fixture에서 pit 완료 20, SC/restart 1/1, 명시적 pit 순위
  변경 발생, hidden rank swap 0, distance reversal 0이었다.
- 1랩 뒤진 후미 차량 fixture는 SC 전후 `lap_deficit=1`을 유지하고 deadlock 없이 재시작했다.

### 자연 표본과 성능

Bahrain·Red Bull Ring, seed 0~9, 10랩 총 20회에서 pair contact 34, local/VSC/SC 판단 중
VSC 19회, SC 6회, restart 6회, DNF 1대였다. 20회 모두 green으로 종료했고 classification
20/unique 20, hidden rank swap 0, distance reversal 0이었다. 짧은 경기 표본 대비 caution 빈도는
아직 높은 편이므로 제품 밸런스 승인이 아니라 구조 검증 결과로만 사용한다.

Bahrain seed 42·57랩은 **4.242544초/10,789 tick**, logical duration 5,394.226890초,
event 1,293개였다. local yellow 12, VSC 3, SC/queue/restart 1/1/1, DNF 1(driver 3), hidden rank
swap과 distance reversal 0, 종료 state green이었다. 57랩 `tracemalloc` peak/retained는
**1,239,591B/1,189,255B**이며 현재 tick 20대와 canonical event timeline을 보존했다.

### 검증과 판정

- progress 계약 **22개, 13.892초, OK**.
- 기존 ABSTRACT 전체 **110개, 190.581초, OK**.
- Abstract/API 표적 3개와 FULL session/grid/pit 표적 4개는 합계 **7개, 19.171초, OK**다.

진행률 v5의 경기 통제·queue·restart 권위 구조는 **승인**이다. 당시 남아 있던 progress broadcast와
사용자 전략 명령 adapter는 44절에서 구현했다. 사고·SC 빈도 보정과 1x/2x/5x 수동 관전 승인은
여전히 남아 있으므로 기존 Stage 4/FULL을 삭제하지 않는다.

## 44. ABSTRACT 진행률 v5 제품 방송 연결 — adapter·명령 rewind·결과 UI (2026-08-11)

진행률 v5를 기존 bounded producer/consumer 방송 계약에 opt-in으로 연결했다. `RaceSetupRequest`의
`abstract_engine=STAGE4|PROGRESS_V5`가 권위를 명시하며, 기존 API 호출은 Stage 4를 유지하고 앱의
ABSTRACT race setup만 `PROGRESS_V5`를 보낸다. 전체 앱의 초기 모드는 계속 `FULL`이다.

### 구현 계약

- `ProgressBroadcastCursor`가 진행 거리·순위·pit·incident·race control을 결과 권위로 사용하고,
  track/pit geometry에서 20대 `RaceVehicleState`를 파생한다. pose·횡 오프셋·표시 가속도는 결과와
  RNG에 피드백하지 않는다.
- 진행률 커널의 0.5초 결정 경계 때문에 pose가 한 번에 이동하지 않도록 0.10초마다 보간한다.
  pit entry/exit 전환도 같은 경계를 거치며 world 이동은 presentation 계약상 370km/h를 넘지 않는다.
- producer는 기존과 같이 start 20 tick, 최대 300 tick만 미리 만든다. 사용자 `set_pace_mode`,
  `pit_call`, `pit_cancel`은 표시 tick을 덮는 runtime checkpoint로 복원한 뒤 speculative queue를
  폐기·재생성한다. 같은 checkpoint와 명령은 같은 event와 pose 미래를 만든다.
- progress runtime checkpoint에는 차량·순위·RNG·race control뿐 아니라 명령 sequence와 이미
  생성한 finish event 경계가 포함된다. presentation checkpoint에는 lateral/world pose 보간 상태도
  포함되어 rewind 뒤 화면 sequence가 달라지지 않는다.
- 방송 race state는 local yellow, VSC, SC phase·남은 시간, 추월 허용 여부와 pit/pace command 상태를
  기존 UI 계약으로 전달한다. instant 결과에는 완주/DNF classification과 retirement reason을
  노출하며 방송 결과도 같은 분류를 쓴다.
- 실제 pit 진입 판단은 회로 `pit_entry_progress`, 논리 route 길이는 entry→exit wrapped progress를
  사용한다. 임시 lap-boundary 진입과 고정 0.12랩 route 가정은 제거했다.

### 자동 검증과 판정

- progress·신규 제품 경로 **26개, 22.910초, OK**: Bahrain/Red Bull Ring 20대 pose, instant와
  broadcast result hash parity, 370km/h world 이동 상한, runtime checkpoint 명령 재생, progress
  instant/API 선택과 5랩 실제 bounded broadcast 완주를 확인했다. 완주 방송의 event id 중복,
  pose 누락·중복과 position speed 위반은 모두 0이었다.
- 기존 ABSTRACT 전체 **112개, 186.129초, OK**, API 전체 **19개, 302.309초, OK**다. API 묶음은
  기존 Stage 4 57랩 bounded replay와 신규 progress v5 제품 경로를 함께 포함한다.
- FULL RaceSetup/session 표적 **69개, 14.026초, OK**로 기존 물리 세션·grid·pit geometry와 cleanup
  계약을 보존했다.
- Frontend **16개**, lint, production build, backend compileall, 문서 상대 링크 **103개/누락 0**,
  `git diff --check`가 모두 통과했다.

따라서 진행률 v5의 제품 방송·사용자 명령·결과 UI 연결은 **자동 검증 승인**이다. 남은 제품 gate는
자연 사건·VSC·SC 빈도 보정, 1x/2x/5x 앱 수동 관전, Bahrain·Red Bull Ring pit transition 시각 확인,
패키징과 cleanup 자원 진단이다. 그 전까지 Stage 4/FULL은 삭제하지 않고 전체 제품 기본 모드를
`FULL`로 유지한다.

## 45. ABSTRACT Timing Authority v1 — 시간 GAP/INT·속도 UI 분리 (2026-08-11)

진행률 v5에 `abstract-progress-timing-v1`을 추가했다. 결과·순위 권위는 그대로 진행 거리와 논리
event가 소유하며, 사용자에게 보여 주는 GAP/INT만 동일 타이밍 라인을 통과한 논리 시각으로
측정한다. ABSTRACT 화면에서 표시되던 `speed_kph`는 pose 재생용 파생값이므로 레이스 판단 정보처럼
보이지 않도록 timing board, driver data center와 추적 HUD에서 제거했다. FULL 속도 표시는 유지한다.

### 구현 계약

- 회로의 sector/mini-sector 정의로 공통 timing loop를 만들고, 각 차량이 루프를 통과한 logical
  time을 진행률 커널 내부 권위 상태에 기록한다. 외부 broadcast tick이 아니라 고정 motion boundary
  위에서 보간하므로 0.05/0.10/0.50초 실행이 같은 시각 값을 만든다.
- GAP은 leader와 차량, INT는 바로 앞 차량과 follower가 마지막으로 함께 통과한 루프의 시각 차이를
  기준으로 한다. 루프 사이에서는 현재 논리 진행 거리와 segment pace를 혼합해 갱신하며, 공통 루프가
  아직 없으면 `estimated`, 있으면 `live`로 명시한다.
- crossing은 최근 4랩만 보존한다. 차량별 crossing 상태는 `_ProgressCar`에 포함되므로 기존 runtime
  checkpoint의 deep copy/restore와 사용자 명령 rewind가 같은 timing 미래를 재생한다.
- 방송 payload의 `gap`, `interval`, `gap_seconds`, `interval_seconds`는 모두 초 단위다. 기존의
  ABSTRACT `+123.4m` GAP은 제거했다. 내부 `logical_speed_mps`와 presentation `speed_kph`는 차량 pose
  보간에만 남고 화면 telemetry로 노출하지 않는다.

### 검증과 판정

- progress 전체 **26개, 59.362초, OK**. 공통 루프 유효성, 비음수 GAP/INT, 30초 동일 시점의
  0.05/0.10/0.50초 완전 일치, pit·SC·restart, runtime checkpoint 명령 재생과 완주 계약을 포함한다.
- progress/API 표적 **4개, 9.836초, OK**. 방송 payload가 leader 이외 모든 GAP을 초 단위로
  보내고 `source_mode=abstract`를 유지함을 확인했다.
- 기존 Stage 1~4를 포함한 ABSTRACT 전체 **114개, 230.643초, OK**.
- Frontend **16개**, lint, production build, backend compileall과 `git diff --check`: **OK**.

Timing Authority v1은 **자동 검증 승인**이다. 실제 패키지의 1x/2x/5x 수동 관전에서 숫자 갱신감과
pit/SC 중 표시를 확인하는 제품 승인은 남아 있다. 다음 고도화는 DRS·dirty air·추월 전술을
진행률/확률 권위로 확장하는 단계이며, 표시 속도를 다시 결과 권위로 사용해서는 안 된다.

## 46. ABSTRACT Racecraft Authority v1 — DRS·dirty air·추월 연출 v2 (2026-08-11)

`abstract-progress-racecraft-v1`은 Progress V5의 단순 상대속도 추월 확률을 서킷 DRS zone,
검지선 시각 차이, tow, dirty air와 공격 전술을 포함하는 논리 권위로 확장한다. FULL wake 물리를
복제하지 않으며, 차량 world pose나 표시 속도는 여전히 결과 판정에 피드백하지 않는다.

### 논리 권위

- 회로의 DRS name/detection/start/end를 immutable `AbstractTrackSnapshot`과 snapshot hash에
  포함한다. detection이 없는 기존 콘텐츠는 activation start 200m 전을 결정적 fallback으로 쓴다.
- DRS는 선두가 1랩을 완료한 뒤 green 상태에서만 작동한다. 차량이 detection line을 통과할 때
  바로 앞 차량과의 Timing Authority 간격이 1.000초 이내면 해당 zone 자격을 latch한다.
- VSC/SC 발동 즉시 active/eligibility를 모두 지운다. SC restart 뒤에는 당시 선두 거리부터
  추가 1랩 동안 DRS를 잠근 뒤 새 detection crossing에서만 자격을 다시 얻는다.
- 120m 이내 추종은 세그먼트 종류에 따라 분리한다. straight는 tow 이득, sweeping/technical은
  dirty-air 손실이 중심이며 heavy-braking/traction은 두 효과를 제한적으로 혼합한다. 최대 game
  pace 효과는 tow +1.0%, dirty air -1.4%, DRS +2.0%인 provisional 보정값이다.
- 추월 readiness/success는 기존 closing potential·driver skill·track difficulty에 tow/DRS 이득과
  dirty-air 손실을 더한다. `attack_mode=drs|tow|braking`과 선택한 maneuver side를 event 및 현재
  차량 상태에 보존한다.
- 순위 변경 직후 간격을 맞추기 위해 뒤 차량을 되감던 기존 보정을 제거했다. 필요한 clearance는
  앞쪽으로 전파해 모든 공개 race distance를 단조 증가시키며, caution 직전 이미 crossing한 차량은
  명시적 `overtake_completed(rank_swap=crossing_before_caution)`로 순위를 확정한다.

### 추월 애니메이션 v2

- `attack` 1.5초: 선택한 안/바깥쪽 라인으로 smoothstep 이동
- `side_by_side` 2.0초: 공격·방어 차량을 반대편 횡 위치로 분리
- `clearance` 1.5초: 성공 차량이 앞 공간을 확보하면서 레이싱 라인으로 복귀
- 실패·caution 중단: 순간 snap 없이 presentation tick당 최대 0.1m로 복귀

연출은 논리 상태를 표현할 뿐 crossing이나 성공 확률을 결정하지 않는다. ABSTRACT Driver Data
Center도 물리 온도·브레이크·슬립 대신 DRS, slipstream, dirty air, traffic/attack mode, pace, tire,
pit와 damage를 표시한다.

### 검증과 현재 보정 판정

- Progress **29개, OK**, Progress V5 방송 API 표적 **1개, 10.896초, OK**: 0.05/0.10/0.50초 결과 결정성, detection/zone, 최초 1랩
  금지, SC clear·restart 1랩 lockout, checkpoint rewind와 animation phase를 포함한다.
- 기존 Stage 1~4를 포함한 ABSTRACT 전체 **117개, 235.700초, OK**.
- Bahrain·Red Bull Ring, seed 0~4, 10랩 10회에서 hidden rank swap과 distance reversal은 모두 0.
  Bahrain은 경기당 attack 72~107회·완료 23~35회·DRS activation 320~611회, Red Bull Ring은
  attack 70~103회·완료 23~35회·DRS activation 332~429회였다. activation은 각 차량·각 zone 진입을
  세므로 추월 횟수와 동일한 지표가 아니다.
- Frontend **16개**, lint와 production build: **OK**.

Racecraft Authority v1의 구조와 결정성은 **자동 검증 승인**이다. 공격·완료 빈도, DRS 보너스와
dirty-air 손실은 게임 밸런스 provisional이며 패키지 1x/2x/5x 수동 관전과 더 큰 seed matrix를
통과한 뒤 확정한다.

## 47. macOS 재패키징·ABSTRACT 단기 smoke (2026-08-11)

arm64 앱을 새로 패키징해 Progress V5와 Timing/Racecraft Authority v1이 실제 번들에 포함된 것을
확인했다. 첫 2x 단기 실행은 Bahrain 57랩 세션을 정상 생성했고 20대 pose, bounded buffer
300 tick, rolling frame/pose 128/2,560을 유지했다. producer error, pose 누락·중복, pose interval 및
position speed 위반은 모두 0이었다. 약 83초 표본의 앱 합산 working set은 약 667~711MB였으며
Electron 합산 peak는 약 629MB, Python RSS는 약 95~102MB였다. 이 표본은 장기 누수 판정이나
57랩 완주 승인을 대신하지 않는다.

첫 실행 후 번들 안에 Python `.pyc`가 생성되어 ad-hoc 서명의 sealed resource 검증이 깨지는 결함을
발견했다. sidecar 실행 환경에 `PYTHONDONTWRITEBYTECODE=1`을 강제하고 회귀 테스트를 추가한 뒤
재패키징했다. 수정 패키지는 빌드 직후와 실제 실행·정상 종료 후 모두 `codesign --verify --deep
--strict`를 통과했고 backend resource 아래 새 `.pyc`가 0개였다. 두 번째 smoke에서 backend ready는
실행 후 약 1.0초, IDLE은 약 1.15초였고 종료 요청 뒤 sidecar가 SIGTERM으로 정상 정리됐다.

Desktop 테스트 **13개**, frontend production build와 `git diff --check`는 통과했다. 다만 화면을
독립적으로 조작·관찰할 로컬 UI 자동화 runtime을 사용할 수 없어 추월 애니메이션, GAP/INT 갱신감,
pit/SC 전환의 시각 승인은 수행하지 못했다. 따라서 다음 제품 gate인 **1x/2x/5x 수동 관전과 57랩
완주·New Race cleanup**은 계속 남아 있다.

## 48. ABSTRACT Racecraft Authority v2 — DRS train·3대 전투 그룹 (2026-08-11)

`abstract-progress-racecraft-v2`는 기존 pair 추월 권위를 유지하면서 DRS train과 최대 3대의
maneuver group 문맥을 추가한다. 인접 INT 1.2초 이내의 연속 차량은 DRS 활성 시점 이후 같은
train으로 분류한다. 폭 9m 이상인 직선·강제동 구간에서 기존 pair가 side-by-side이고 바로 뒤
차량이 12m 이내에서 DRS 또는 강한 tow를 가지면 세 차량을 corridor 0/1/2로 분리한다.

세 번째 차량은 전술 압박과 화면 corridor만 얻는다. 한 결정 경계의 순위 변경은 계속 인접
attacker/defender pair 한 쌍만 명시적 `overtake_completed`로 확정한다. 따라서 3-wide 화면을 이유로
숨은 2자리 추월을 만들지 않으며, opportunist는 이후 별도 공격 이벤트를 거쳐야 순위를 얻는다.
group은 green에서만 생성되고 pit·incident·코너성 구간과 caution에서는 생성하지 않으며
`three_wide → merge` 뒤 해산한다. presentation 계약은 `progress-v5-derived-pose-v2`로 올렸다.

방송 payload와 ABSTRACT Driver Data Center에는 train 크기·순번, group ID·구성원·phase·corridor를
추가했다. Timing Board는 3대 그룹을 `3W`, 일반 DRS train을 `Tn`으로 표시한다. 이 정보와 횡 pose는
논리 결과에 피드백하지 않으며 runtime checkpoint에 포함돼 사용자 명령 rewind 후 동일하게 재생된다.

### 자연 표본과 판정

Bahrain·Red Bull Ring, seed 0~4, 10랩 총 10회에서 경기당 attack 79~113회, 추월 완료 18~42회,
3대 group 형성 7~39회였다. 모든 경기의 최대 group은 3대였고 hidden rank swap과 distance reversal은
0이었다. 최대 DRS train은 15~20대로 길게 나타났다. 이는 현재 진행률 모델이 초중반 필드를 매우
촘촘하게 유지하는 특성이 반영된 값이므로 구조 오류로 보지는 않지만, 실제 제품 관전에서 `T15~T20`
표시가 과도한지는 별도 밸런스 gate로 남긴다.

### 자동 검증

- Progress 전체 **31개, 76.717초, OK**.
- 기존 Stage 1~4를 포함한 ABSTRACT 전체 **119개, 248.151초, OK**.
- Progress V5 방송 API 표적 **1개, 12.071초, OK**.
- FULL session·RaceSetup·grid/pit 표적 **77개, 25.508초, OK**.
- Frontend **16개**, lint와 production build, Desktop **13개**: **OK**.

Racecraft Authority v2의 논리 구조·결정성·기존 모드 회귀는 **자동 검증 승인**이다. 남은 제품 gate는
1x/2x/5x에서 3-wide corridor의 겹침·snap 여부, 긴 DRS train 표시 가독성, 사건·추월 빈도와
57랩 완주·New Race cleanup이다.

## 49. Racecraft Authority v2 macOS 패키징 (2026-08-12)

macOS arm64 앱을 다시 생성했다. 번들 backend에서 `abstract-progress-racecraft-v2`와
`progress-v5-derived-pose-v2`, frontend production asset에서 DRS train·maneuver group UI 포함을
확인했다. 앱 크기는 약 587MB이며 build 직후와 패키지 Python import 뒤 모두
`codesign --verify --deep --strict`를 통과했다. backend resource의 `.pyc`는 0개로 코드 서명
불변성 계약도 유지했다.

이번 단계는 패키지 생성·정적 검증까지만 수행했으며 앱 실행과 1x/2x/5x 관전은 수행하지 않았다.
따라서 Racecraft v2의 논리 자동 승인은 유지하지만 3-wide·긴 DRS train의 시각 제품 승인은 남아 있다.

## 50. FULL·ABSTRACT 듀얼 엔진 1차 분리 (2026-08-12)

FULL 물리와 ABSTRACT 매니지먼트 로직을 독립 개발하기 위한 실행 경계를 추가했다.
`backend/engines/contracts.py`가 family·capability·adapter protocol을, `factory.py`가 모든
`SimulationMode`의 단일 선택 권위를 가진다. API qualifying/race setup은 factory가 반환한 adapter만
호출하며 직접 `RaceEngine` 또는 `AbstractRaceEngine`을 생성하지 않는다.

FULL adapter는 qualifying과 기존 50Hz `RaceEngine` 조립을 소유하고 `SessionManager`는 build 결과를
세션 수명주기에 연결한다. ABSTRACT adapter는 immutable snapshot, qualifying, Progress/Stage4 instant,
bounded broadcast 생성과 응답 변환을 소유한다. adapter 간 상대 엔진 내부 import를 금지하는 AST
회귀 테스트 4개를 추가했다.

frontend는 `frontend/src/engines/full|abstract` runtime profile로 분리했다. FULL은 physics telemetry와
physics pose, ABSTRACT는 logical event와 derived strategy-map pose를 명시한다. 기존 renderer 호환은
유지하지만 헤더와 control policy는 profile을 통해 선택하므로 이후 ABSTRACT 지도 UI를 FULL과 별개로
교체할 수 있다.

검증 결과는 엔진 경계+Progress **35개, 85.187초**, API 전체 **19개, 356.579초**, FULL
RaceSetup/session **69개, 15.479초**, frontend **18개**와 lint/build가 모두 통과했다.

판정은 **1차 실행 경계 분리 승인**이다. 기존 FULL 실제 파일은 아직 `backend/simulation/`에 있고
ABSTRACT runtime도 `backend/simulation/abstract/`에 있으므로 물리적 이동 완료로 표현하지 않는다.
다음 단계는 현재 상태 커밋 후 결과 공식을 바꾸지 않는 기계적 모듈 이동이며 기준은
[`DUAL_ENGINE_ARCHITECTURE.md`](DUAL_ENGINE_ARCHITECTURE.md)를 따른다. 현재 macOS 패키지는 이 분리
직전에 생성됐으므로 새 frontend profile/factory를 포함하려면 다시 패키징해야 한다.

## 51. FULL runtime 물리 이동 1차 — qualifying·RaceEngine ownership (2026-08-12)

듀얼 엔진 이동 전 기준점을 `2d0236a`로 커밋한 뒤 FULL 예선 구현을
`backend/engines/full/runtime/qualifying.py`로 실제 이동했다. `FullEngineAdapter`는 새 경로를
직접 호출하며 `backend/simulation/qualifying.py`에는 기존 테스트와 외부 호출자를 위한 공개 symbol
재수출만 남겼다. 예선 공식, RNG, compound mapping과 응답 schema는 변경하지 않았다.

그 다음 중앙 50Hz 조정자도 `backend/engines/full/runtime/race_engine.py`로 이동했다.
`FullEngineAdapter`와 `SessionManager` 타입 계약은 새 경로를 직접 사용한다. 기존
`backend/simulation/race_engine.py`는 runtime 모듈 객체 자체를 등록하는 alias shim이므로 기존
도구·테스트의 import뿐 아니라 `patch("simulation.race_engine.*")`도 실제 엔진 전역에 그대로 적용된다.

경계 검사는 FULL·ABSTRACT 하위 디렉터리 전체를 재귀 탐색하도록 보강했고, legacy import와 새 runtime
import가 동일 `run_qualifying` 함수 객체를 반환하는 회귀를 추가했다. API 진입점의 사용하지 않는
legacy 예선 import도 제거했다.

예선 이동 직후 엔진 경계 **5개**, FULL 예선·타이어·레이스 엔진 **326개, 795.263초, OK**, API
전체 **19개, 345.373초, OK**였다. 중앙 엔진 이동 뒤에는 경계 **6개**, 동일 FULL 묶음
**326개, 647.657초, OK**, 별도 session·simulation foundation **39개, 35.171초, OK**로 다시
검증했다. 문서 상대 링크 누락은 0개이며 `git diff --check`도 통과했다.

이번 이동은 FULL 물리 분리의 첫 단위이며 차량 물리 하위 모듈과 ABSTRACT runtime의 실제 이동은
아직 시작하지 않았다. 다음 단위는 `RaceEngine`의 `simulation.*` 하위 의존성을 기능 묶음별로
옮기는 작업이다.

## 52. FULL runtime 물리 이동 2차 — 핵심 차량 물리 묶음 (2026-08-12)

ABSTRACT가 직접 또는 track presentation을 통해 사용하는 `track_physics`, `track_display`,
`start_grid_geometry`, `vehicle_dimensions`는 공용 계약으로 남겼다. 직접 import 조사 뒤 전이 의존성도
검사해 `track_physics → trajectory_physics`가 사용하는 `vehicle_dynamics` 역시 공용으로 유지했다.
FULL 전용으로 확인된 다음 6개
모듈은 `backend/engines/full/runtime/`으로 이동했다.

- `fixed_step`, `speed_profile`
- `vehicle_physics`
- `brake_model`, `collision`, `wake_model`

`RaceEngine`과 `vehicle_physics` 내부 연결은 package-relative import로 바꿨다. 기존
`simulation.<module>` 경로는 새 runtime 모듈 객체를 직접 가리키므로 import와 module-level patch
호환을 모두 유지한다. 6개 legacy/runtime 모듈 identity를 자동 검사하는 경계 회귀도 추가했다.

초기 경계 실행에서는 ABSTRACT 공용 track geometry 초기화 중 compatibility alias가 상위
`engines` package의 eager factory import를 유발하는 순환 의존을 검출했다. `engines`,
`engines.full`, `engines.abstract` package initializer를 지연 로딩으로 바꿔 runtime leaf import가
상대 엔진 adapter를 초기화하지 않도록 수정했다. `vehicle_dynamics` 공용 분류를 바로잡은 최종
상태에서 경계 **8개, 0.061초, OK**, 경계·충돌·차량 동역학·vehicle physics·wake·trajectory 표적
**62개, 3.857초, OK**다. FULL 예선·타이어·레이스 엔진 **326개, 658.712초, OK**, ABSTRACT
Stage 1~4·Progress 전체 **119개, 251.976초, OK**로 두 엔진의 장기 회귀를 모두 통과했다. API 전체
**19개, 336.871초, OK**로 lazy factory의 실제 FastAPI 진입점과 57랩 controlled broadcast도
통과했다. 문서 상대 링크 누락은 0개이며 `git diff --check`도 통과했다.

## 53. FULL runtime 물리 이동 3차 — 경기 운영 계층 (2026-08-12)

`RaceEngine`이 조합하는 경기 운영 mixin과 그 FULL 전용 보조 계약을 한 묶음으로
`backend/engines/full/runtime/`에 이동했다. 이동 대상은 다음 13개다.

- 운영 mixin: `incident_ops`, `racecraft_ops`, `pit_ops`, `strategy_ops`, `safety_car`, `start_ops`,
  `timing_ops`
- 보조 계약: `ai_strategy`, `events`, `incidents`, `pit_stop`, `runtime_constants`, `state_contract`

runtime 내부에서 이 모듈들 및 앞서 이동한 핵심 차량 물리를 참조할 때는 package-relative import를
사용한다. 기존 `simulation.<module>` 경로는 동일 runtime 모듈 객체를 가리키는 patch-safe alias로
유지한다. `SessionManager`의 pit command parsing도 새 runtime 경로를 직접 사용한다.

경계 검사는 총 19개 이동 모듈의 legacy/runtime identity와 공용 geometry의 FULL alias 비의존을
확인한다. 경계 8개와 기존 `simulation.race_engine.roll_solo_incident` patch 회귀 1개는
**9개, 2.815초, OK**다. FULL 예선·타이어·레이스 엔진 **326개, 667.509초, OK**, 별도
session·simulation foundation **39개, 36.854초, OK**, ABSTRACT Stage 1~4·Progress 전체
**119개, 259.666초, OK**다. 이번 단계는 API·factory 계약을 변경하지 않았으므로 API 전체 묶음은
재실행하지 않았으며, 직전 52절의 **19개, 336.871초, OK** 기준을 유지한다.

## 54. 공용 트랙 계약 분리 — ABSTRACT solver 직접 의존 제거 (2026-08-12)

`track_physics.py`는 공용 display profile과 FULL trajectory 최적화 구현이 섞여 있어 파일 전체를
FULL runtime으로 이동할 수 없었다. 엔진 중립 `simulation/track_contracts.py`를 추가하고 다음
계약을 옮겼다.

- `DRIVING_LINE_RACING|INSIDE|OUTSIDE|DEFENSIVE`
- `TrackPhysicsSample`, `LocalMetricCoordinateFrame`
- 최소 geometry 표면을 정의하는 `TrackGeometryProfile` Protocol

`TrackPhysicsProfile`은 이 Protocol을 구조적으로 구현하며 기존 symbol도 재수출하므로 FULL 및 외부
호출 호환을 유지한다. ABSTRACT pose·kinematics와 공용 start-grid는 더 이상 `track_physics` solver
모듈을 직접 import하지 않는다. `track_display`도 type 계약은 Protocol을 사용하고 실제 profile 생성
함수만 기존 compiler에서 호출한다.

경계·track physics·ABSTRACT geometry·kinematics 표적 **45개, 21.691초, OK**다. 별도 경계 테스트는 ABSTRACT
presentation 소비자가 `track_contracts`를 사용하고 `track_physics`를 직접 import하지 않는지
검사한다. 다음 이동 대상은 이 경계 밖의 FULL 전용 local planner·planner scheduler·lap physics다.

## 55. FULL runtime 물리 이동 4차 — local planner·lap physics (2026-08-12)

54절의 공용 `TrackGeometryProfile` 경계 밖에 있는 FULL 전용 모듈 3개를
`backend/engines/full/runtime/`으로 이동했다.

- `local_trajectory_planner`: short-horizon lateral lattice와 traffic validation
- `planner_scheduler`: 5/10/20/50Hz 검증 cadence와 progress spatial index
- `physics`: FULL 예선·랩 시간·progress 계산

local planner는 공용 profile의 구현 클래스 대신 `TrackGeometryProfile` Protocol만 타입 계약으로
사용한다. 충돌·차량 integrator는 FULL runtime 상대 import를 사용하고, track surface와
`vehicle_dynamics`는 공용 solver 계약으로 유지한다. `RaceEngine`, qualifying, racecraft, strategy,
start operation의 내부 참조도 package-relative 경로로 전환했다. 기존 `simulation.<module>` 경로는
동일 runtime 모듈을 가리키는 alias다.

경계·local planner·scheduler 표적 **30개, 0.767초, OK**, FULL 예선·타이어·레이스 엔진
**326개, 637.234초, OK**, 별도 session·simulation foundation **39개, 34.663초, OK**다.
ABSTRACT Stage 1~4·Progress 전체 **119개, 245.763초, OK**다. API·factory 계약은 변경하지 않았으므로
API 전체 묶음은 재실행하지 않고 52절의 **19개, 336.871초, OK** 기준을 유지한다.

## 56. FULL runtime 물리 이동 5차 — tire·trajectory solver (2026-08-12)

FULL 전용 solver 5개를 `backend/engines/full/runtime/`으로 이동했다.

- `tire_model`: compound physics, wear, blanket, thermal budget
- `vehicle_dynamics`: force·bicycle model primitives
- `trajectory_physics`: vehicle/tire whole-lap speed profile
- `track_surface`: four-wheel contact, kerb·runoff assessment
- `car_performance`: constructor spec의 FULL 성능 factor 변환

공용 `track_physics`의 기본 geometry compiler가 import만으로 이 solver 체인을 초기화하지 않도록
차량별 최적화 구간의 surface·trajectory import를 함수 내부로 지연했다. 공용 kerb allowance는
`track_contracts`로 옮겼다. 별도 subprocess 경계 테스트는 `import simulation.track_physics` 직후
위 FULL solver 5개가 `sys.modules`에 존재하지 않는지 확인한다. 기존 `simulation.<module>`은 동일
runtime 모듈을 가리키는 patch-safe alias이며 FULL runtime 내부는 package-relative import를 사용한다.

경계·trajectory·surface·vehicle dynamics·compound 표적 **42개, 9.443초, OK**, track physics·타이어
장기 열·서킷 열 profile **42개, 531.988초, OK**, FULL 예선·타이어·레이스 엔진
**326개, 658.400초, OK**, ABSTRACT Stage 1~4·Progress 전체 **119개, 250.224초, OK**다.
API·factory 계약은 변경하지 않았으므로 API 전체 묶음은 재실행하지 않고 52절의
**19개, 336.871초, OK** 기준을 유지한다.

## 57. FULL runtime 물리 이동 6차 — vehicle track solver (2026-08-12)

공용 `simulation/track_physics.py`에 남아 있던 차량·타이어별 전역 주행선 최적화와 대용량 profile
캐시를 `backend/engines/full/runtime/vehicle_track_solver.py`로 분리했다. 새 runtime 모듈은
`PhysicalGlobalTrajectoryCostModel`, vehicle trajectory 최적화, corner-adaptive center 선택,
차량별 `TrackPhysicsProfile` 생성과 캐시 수명주기를 소유한다. `RaceEngine`은 새 runtime 경로를 직접
호출한다.

공용 모듈에는 엔진 중립 geometry·기본 주행선 compiler와 profile 조회만 남겼다. 기존 테스트·도구의
`simulation.track_physics` 차량 solver import는 module `__getattr__`을 통해 동일 runtime symbol로
지연 연결되며 캐시 객체 identity도 보존한다. 따라서 공용 compiler를 단순 import할 때는
`vehicle_track_solver`, tire, surface, trajectory, vehicle dynamics 등 FULL solver가 초기화되지 않는다.
별도 subprocess 및 symbol identity 회귀가 이 경계를 검사한다.

경계·track physics·trajectory 표적 **33개, 13.122초, OK**다. Backend 전체 discovery는 장기 열·API·
FULL·ABSTRACT를 포함해 **671개, 1,841.926초, OK**다. 이번 변경은 물리 상수, 최적화 공식, profile
cache key와 결과 schema를 변경하지 않았다.

## 58. ABSTRACT runtime 물리 이동 (2026-08-12)

기존 `backend/simulation/abstract/`의 결과·방송·진행률·교통·pose 구현 16개를
`backend/engines/abstract/runtime/`으로 이동했다. `AbstractEngineAdapter`, API 진입점, ABSTRACT 진단
도구와 전용 테스트는 새 runtime을 직접 import한다. runtime 내부 결합은 package-relative import를
유지하므로 FULL 내부 구현을 참조하지 않는다.

기존 `simulation.abstract` package public symbol은 새 runtime symbol을 재수출하고, 각
`simulation.abstract.<module>`은 실제 runtime 모듈 객체를 등록하는 patch-safe alias로 남겼다.
따라서 기존 외부 호출과 monkey-patch 호환은 유지하면서 동일 구현이 서로 다른 module name으로
중복 로드되는 것을 막는다. 경계 테스트는 16개 모듈 identity, canonical hash 함수 patch 전달,
ABSTRACT의 FULL import 0과 API의 legacy import 0을 검사한다.

경계·ABSTRACT Stage 1~4·Progress 회귀는 **132개, 248.928초, OK**다. 변경된 API와 반대편 FULL
session·foundation 회귀는 **58개, 372.103초, OK**다. 결과 공식, RNG stream, engine/version 문자열,
canonical hash payload와 방송 계약은 변경하지 않았다. 다음 구조 작업은 FULL 테스트·도구에 남은
legacy import 사용처 감사이며, compatibility shim 삭제는 사용처 0과 외부 호환 정책을 별도로 승인한
뒤 결정한다. Backend 전체 discovery도 **673개, 1,827.339초, OK**로 최종 통과했다.

## 59. FULL legacy import 권위 경로 정리 (2026-08-12)

FULL runtime으로 이동 완료된 모듈을 계속 `simulation.*`으로 호출하던 제품 benchmark, 진단 도구와
테스트를 `engines.full.runtime.*` 권위 경로로 전환했다. 사고·SC monkey-patch도 실제 runtime 경로를
직접 대상으로 삼는다. 공용 track geometry·compiler·display·data validation과 vehicle dimension은
엔진 중립 `simulation.*` 계약이므로 이동 대상에서 제외했다.

`test_engine_boundaries.py`는 backend 저장소 전체 AST를 검사해 FULL compatibility module import가
권위 코드에 다시 들어오면 실패한다. 기존 `simulation.<moved-module>` 참조는 이 호환성 테스트와 shim
자체에만 남았고 제품·도구·일반 테스트의 권위 사용처는 0건이다. 호환 module identity와
`simulation.race_engine` patch 전달 회귀는 계속 유지한다.

경계·물리 하위 모듈·planner·session 표적은 **141개, 47.481초, OK**다. 변경된 네 진단 도구의
`--help` 진입점도 모두 정상이다. Backend 전체 discovery는 **674개, 1,828.368초, OK**다. 이번 단계는
import와 patch 대상 경로만 바꿨으며 물리·확률·결과 schema를 변경하지 않았다. 저장소 내부 전환은
완료됐지만 외부 도구·패키지 사용자를 위해 compatibility shim은 아직 삭제하지 않는다.

## 60. 엔진 compatibility 정책 확정 (2026-08-12)

`backend/engines/compatibility.py`를 `engine-runtime-compat-v1` 단일 코드 계약으로 추가했다. FULL
patch-safe alias 28개, ABSTRACT alias 16개와 FULL qualifying·ABSTRACT package facade를 명시한다.
manifest는 immutable mapping이며 경계 테스트가 각 legacy/runtime 파일 존재, 동일 module 객체,
facade 공개 symbol identity와 patch 전달을 검사한다.

정책 문서 [`ENGINE_COMPATIBILITY_POLICY.md`](ENGINE_COMPATIBILITY_POLICY.md)는 shim을 현재 유지하는
이유와 네 가지 삭제 gate를 정의한다. desktop packaging은 backend 전체를 복사하므로 runtime과 shim을
함께 포함한다. shim은 runtime module 객체를 재사용해 별도 물리 계산, 상태 복제나 tick당 메모리 비용을
추가하지 않는다.

경계·API는 **34개, 342.707초, OK**, 추가 manifest/facade 경계는 **16개, 1.271초, OK**다. Frontend
**18개**, lint와 production build, Desktop **13개**도 모두 통과했다. 직전 59절의 Backend 전체
**674개, 1,828.368초, OK** 이후 계산 코드는 변경하지 않았다. 새 macOS 앱 패키지는 이번 단계에서
생성하지 않았다.

## 61. 듀얼 엔진 분리 후 macOS 패키징·실행 smoke (2026-08-12)

브랜치 `codex/abstract-race-simulation`을 origin에 푸시하고 Electron 37.2.6 macOS arm64 앱을
재생성했다. 최종 앱은 `desktop/out/F1 Race Manager-darwin-arm64/F1 Race Manager.app`이며 약
587MB다. 번들에는 `engine-runtime-compat-v1`, FULL runtime과 ABSTRACT runtime이 모두 포함됐다.

패키징 직후와 실제 앱 실행·종료 뒤 모두 `codesign --verify --deep --strict`를 통과했다. backend
resource의 `.pyc`는 실행 전후 0개였고 F1 Race Manager/Electron/backend 잔류 프로세스도 0이다. 앱
설정 화면은 Bahrain 57랩, FULL Physics, ABSTRACT Broadcast, ABSTRACT Instant와 팀·타이어 데이터를
정상 표시했다.

검증 중 서명된 번들 Python을 직접 실행하면 `.pyc`가 생성되어 sealed resource를 변경한다는 점을
재확인했다. 해당 중간 산출물은 폐기하고 앱을 깨끗하게 다시 패키징했으며, 최종 검증은 번들 파일을
실행하지 않는 정적 검사와 실제 Electron 실행 경로만 사용했다. 이번 smoke에서는 레이스 계산이나
FULL·ABSTRACT 장기 관전은 시작하지 않았다.
