# Red Bull Ring 최적화 1차 작업 지시서

- 문서 상태: **구현 전 작업 기준**
- 작성일: **2026-07-31**
- 대상: 다음 구현 에이전트
- 회로: `circuit_id=4`, Red Bull Ring
- 1차 환경: `NORMAL`, 대기 18°C / 노면 30°C
- 1차 지명: `HARD=C2`, `MEDIUM=C3`, `SOFT=C4`
- 최종 목표: Bahrain에서 검증한 물리·열·AI 구조를 유지하면서 Red Bull Ring의
  단독 주행, 20대 교통, 피트, SC와 C2~C4 동작을 재현 가능한 기준선으로 승인

선행 문서:

1. [`SIMULATION_FOUNDATION.md`](SIMULATION_FOUNDATION.md)
2. [`CURRENT_PROJECT_STATUS.md`](CURRENT_PROJECT_STATUS.md)
3. [`ADDING_REAL_CIRCUITS.md`](ADDING_REAL_CIRCUITS.md)
4. [`TIRE_COMPOUND_C1_C5_PHASE_2_DIRECTIVE.md`](TIRE_COMPOUND_C1_C5_PHASE_2_DIRECTIVE.md)
5. [`CIRCUIT_THERMAL_PRESET_SOURCES.md`](CIRCUIT_THERMAL_PRESET_SOURCES.md)
6. [`COLD_OUTLAP_CONTROL_DESIGN.md`](COLD_OUTLAP_CONTROL_DESIGN.md)

---

## 1. 작업 목적

이번 작업은 여러 회로를 한꺼번에 보정하는 작업이 아니다. Red Bull Ring 하나를
대상으로 다음 수직 경로를 끝까지 검증한다.

```text
원본·형상 무결성
  → 정적 텔레메트리 비교
  → 단독 동적 5랩
  → 20대 단축 레이스
  → 피트 1회
  → 강제 SC 1회
  → C2/C3/C4 열·마모
  → 1x/2x·cleanup 승인
```

이 회로에서 재사용 가능한 진단과 승인 절차를 만든 뒤 Silverstone, Hungaroring,
Monza에 같은 절차를 적용한다.

이번 단계에서는 Bahrain에서 승인한 전역 차량 물리와 타이어 열 계수를 기준선으로
고정한다. Red Bull Ring의 문제를 전역 계수로 덮지 않고 다음 순서로 원인을
분리한다.

1. 원본 정렬·랜드마크·구간·대섹터 데이터
2. 레이싱라인과 폭 프로필
3. 회로별 속도·제동 controller profile
4. 로컬 플래너와 AI 실행
5. 타이어·브레이크 열 결과

---

## 2. 현재 기준선

### 2.1 데이터

| 항목 | 현재 값 |
|---|---|
| 공식 모델 길이 | 4,318m |
| 레이스 랩 | 71 |
| 기준 랩타임 | 65.0초 |
| 중심선 | OSM, `osm:red-bull-ring-gp`, ODbL |
| OSM 원본 길이 | 4,308.606m |
| 컴파일 길이 | 4,318.0m |
| 중심선 표본 | 157 |
| 피트 표본 | 43 |
| 최대 점 간격 | 15.932 |
| 폭 프로필 | TUMFTM Spielberg, 64개 표본 |
| 폭 범위 | 총 10.181~13.191m, 평균 10.993m |
| 피트 제한 | 80km/h |
| 피트 진입/출구 | 약 0.887 / 0.119 |
| 대기/노면 | NORMAL 18/30°C, provisional |
| 타이어 지명 | C2/C3/C4, provisional |

현재 `surface_zones`, 측량 기반 `track_boundaries`, 고도와 뱅킹 프로필은 없다.
런타임은 의도적으로 `planar_2d`다. 이 한계를 회로 속도계수로 숨기지 않는다.

### 2.2 텔레메트리 기준

`backend/data/calibration/red_bull_ring_2025_qualifying.json`에는 2025 Austrian
Grand Prix 예선의 다음 다섯 랩 median 기준이 있다.

```text
NOR 17 1:03.971
LEC 17 1:04.492
PIA 16 1:04.554
HAM 20 1:04.582
RUS 17 1:04.763
```

현재 `pace=1.04` 정적 목표속도 비교:

| 지표 | 현재 값 |
|---|---:|
| MAE | 5.314km/h |
| RMSE | 7.634km/h |
| Bias | -3.085km/h |
| 최대 과속 오차 | +32.879km/h |
| 최대 저속 오차 | -32.425km/h |

평균 형상은 기존 테스트 허용 범위에 들어오지만, 제동 시작·apex 전환의 국부
오차가 크다. 정적 target profile만으로 실제 차량이 트랙 안에서 같은 속도를
실행하는지는 아직 증명되지 않았다.

### 2.3 2026-07-31 작업 전 검증

다음 명령은 통과했다.

```bash
cd /Users/kimyongjin/Desktop/f1/backend

.venv/bin/python tools/validate_track_data.py --circuit-id 4
.venv/bin/python tools/audit_tier_a.py --circuit-id 4
.venv/bin/python -m unittest \
  tests.test_telemetry_calibration.RedBullRingTelemetryCalibrationTests
```

결과:

- `validate_track_data`: 오류 없음
- Red Bull Ring 텔레메트리 회귀 2개: 통과
- 중심선·피트·DRS·랜드마크·구간 coverage: 통과
- Tier A 감사에서 대섹터 `start/end`가 명시되지 않아
  `sectors_have_timing_ranges=false`

다음 Red Bull Ring 전용 `RaceSetupTests` 5개도 통과했다.

```bash
.venv/bin/python -m unittest \
  tests.test_engine.RaceSetupTests.test_red_bull_ring_pit_lane_is_anchored_to_track \
  tests.test_engine.RaceSetupTests.test_red_bull_ring_segment_lookup_matches_expected_driving_sections \
  tests.test_engine.RaceSetupTests.test_red_bull_ring_segments_have_corner_specific_speed_factors \
  tests.test_engine.RaceSetupTests.test_red_bull_ring_geo_source_matches_official_layout_anchors \
  tests.test_engine.RaceSetupTests.test_red_bull_ring_drs_zones_stay_on_straights
```

---

## 3. 범위

### 3.1 반드시 수행

- 현재 데이터와 실행 결과를 변경 전 기준선으로 저장
- Red Bull Ring 대섹터 timing range의 출처 확인과 명시
- 정적 텔레메트리 비교와 실제 동적 5랩 비교를 분리
- 코너별 속도·제동·스로틀 오차 보고
- 20대 단축 레이스에서 출발·교통·배틀 안정성 검증
- 피트 진입부터 합류까지 전체 사이클 검증
- 강제 SC의 출동·대열 수집·재시작·피트 복귀 검증
- NORMAL 18/30°C에서 C2/C3/C4의 열·마모·그립 순서 확인
- 1x/2x 결정성 및 세션 cleanup 확인
- 변경 전후 JSON 산출물과 적용한 계수군 기록

### 3.2 이번 단계에서 제외

- 콜드 아웃랩과 피트 건식 타이어 70°C
- 출발 포메이션 랩
- 전역 C1~C5 물성 변경
- 전역 타이어 발열·냉각 계수 변경
- 전역 차량 출력·공력·브레이크·마찰계수 변경
- 동적 날씨, 바람, 습도, 비와 젖은 노면
- 고도·경사·뱅킹 물리
- 근거 없는 연석·런오프·surface zone 제작
- 장벽·관중석 등 그래픽 확장
- COOL/HOT과 NORMAL의 동시 보정

위 항목이 필요해 보이면 구현하지 말고 근거와 재현 결과를 최종 보고의 후속
과제로 남긴다.

---

## 4. 데이터 권위와 변경 규칙

### 4.1 권위 순서

```text
공식 FIA/F1 회로 자료
  → OSM 중심선·피트
  → TUMFTM 좌·우 폭
  → TracingInsights 예선 속도·스로틀·제동 prior
  → 게임 회로별 controller calibration
```

- 실제 텔레메트리는 물리 차량을 강제로 움직이는 경로가 아니다.
- TUM 라인은 최종 레이싱라인이 아니라 폭과 형상 검증 자료다.
- `base_lap_time` 하나만 바꿔 코너 오차를 숨기지 않는다.
- `speed_factor`는 실측 차이가 특정 구간에 반복적으로 나타날 때만 변경한다.

### 4.2 계수 변경 순서

한 실험에서 한 계수군만 변경한다.

1. progress 정렬, sector/corner window와 데이터 오류
2. racing-line reference 정렬과 가중치
3. `controller_sample_distance_m`
4. Red Bull Ring 전용 `planner_braking_utilization`
5. telemetry braking 관련 회로별 설정
6. 근거가 남을 때만 segment별 `speed_factor`

다음은 1차 해결 수단으로 사용하지 않는다.

- 전역 출력 증가 또는 감소
- 전역 그립·다운포스·브레이크 변경
- C2/C3/C4 작동창 또는 degradation 변경
- numeric guard와 진단 threshold 완화
- 충돌 또는 run-wide 이벤트 억제

### 4.3 실험 기록

각 변경 후보는 다음을 기록한다.

```json
{
  "change_id": "rbr-p1-001",
  "parameter_group": "controller_sample_distance",
  "before": {},
  "after": {},
  "reason": "",
  "single_car_metrics": {},
  "traffic_metrics": {},
  "thermal_metrics": {},
  "accepted": false
}
```

실패 후보도 최종 JSON에 전부 누적할 필요는 없지만, 최소한 최종 보고에는 어떤
방향이 왜 기각됐는지 남긴다.

---

## 5. 단계별 작업

## 5.1 0단계 — 변경 전 기준선 고정

### 작업

1. 현재 작업 트리의 기존 사용자 변경을 보존한다.
2. 아래 파일의 변경 전 상태와 해시 또는 git blob 정보를 기록한다.
   - `backend/data/circuits.json`
   - `backend/data/circuit_sources/red_bull_ring_osm_geo.json`
   - `backend/data/circuit_sources/red_bull_ring_track_profile_v2.json`
   - `backend/data/circuit_thermal_profiles.json`
   - `backend/data/tire_compound_nominations.json`
3. 현재 정적 텔레메트리 비교값과 Tier A 감사 결과를 기준 산출물에 저장한다.
4. 테스트를 위해 원본 텔레메트리를 다시 내려받아야 한다면 기존 고정 URL과
   reference lap 목록을 사용한다. 다운로드 결과를 곧바로 프로젝트 데이터에
   덮어쓰지 않는다.

### 산출물

권장 신규 파일:

```text
backend/data/calibration/red_bull_ring_runtime_baseline_v1.json
```

최소 필드:

```text
schema_version
circuit_id
generated_at
git_commit 또는 working_tree_note
environment
nomination
source_identity
static_telemetry_metrics
tier_a_audit
single_car_dynamic
traffic
pit
safety_car
thermal
determinism
cleanup
accepted
```

`generated_at`은 정보용이다. 결정성 비교는 시간 문자열이 아니라 seed와 입력
조건으로 수행한다.

### 완료 조건

- 변경 전 기준 결과가 JSON 한 파일에서 재현 가능하다.
- 기존 calibration 파일을 결과 없이 덮어쓰지 않았다.
- 다른 회로 데이터는 변경하지 않았다.

---

## 5.2 1단계 — 대섹터와 트랙 계약 보강

### 작업

1. 공식 세션 또는 FIA 회로 자료에서 Sector 1/2 timing line의 위치를 확인한다.
2. `circuits.json`의 Red Bull Ring 세 대섹터에 명시적 `start/end`를 추가한다.
3. 다음 불변식을 검사한다.

```text
Sector 1 start = 0.0
Sector 1 end = Sector 2 start
Sector 2 end = Sector 3 start
Sector 3 end = 1.0
빈 구간과 중첩 없음
각 sector 안의 mini-sector 합계 = 27
```

4. 정확한 공식 위치를 확인하지 못하면 추정값을 넣지 않는다.
   `sectors_have_timing_ranges=false`를 알려진 차단 항목으로 유지하고 출처 확보를
   먼저 수행한다.
5. 피트 anchor, SC2, DRS와 segment 경계가 sector 변경으로 움직이지 않는지
   확인한다.

### 테스트

- Red Bull Ring sector range·연속성 전용 테스트 추가
- 27 mini-sector 생성과 랩 경계 wrap 테스트
- 기존 트랙 검증과 Tier A 감사

### 완료 조건

- `audit_tier_a --circuit-id 4`의 `sectors_have_timing_ranges=true`
- 출처 URL과 취득일이 문서 또는 데이터 계보에 남음
- 기존 형상·피트·DRS 검사 무회귀

---

## 5.3 2단계 — 재사용 가능한 동적 단독 랩 진단

### 작업

Bahrain 전용 도구를 복사해 숫자만 바꾸지 않는다. 다른 회로에도 재사용 가능한
진단 진입점을 추가한다.

권장 도구:

```text
backend/tools/run_circuit_baseline.py
```

권장 CLI:

```bash
cd /Users/kimyongjin/Desktop/f1/backend

.venv/bin/python tools/run_circuit_baseline.py \
  --circuit-id 4 \
  --thermal-preset NORMAL \
  --mode single-car \
  --laps 5 \
  --seed 42 \
  --output data/calibration/red_bull_ring_runtime_baseline_v1.json
```

도구는 `RaceEngine`의 실제 0.02초 고정 스텝을 사용한다. 정적
`target_speed_mps()`만 반복해 동적 테스트라고 부르지 않는다.

### 수집 항목

- 랩별 lap time과 sector time
- progress 표본별 실제 속도, target speed, throttle, brake
- 텔레메트리 median 대비 속도 오차
- 코너별 최저속도와 오차
- 실제/기준 제동 시작 progress와 거리 차이
- 실제/기준 throttle 재개 progress와 거리 차이
- 최대·P95 lateral error와 heading error
- lockup, wheelspin, understeer, oversteer, run-wide
- track limit, off-track, contact
- 전·후륜 표면/코어 온도와 thermal grip
- 전·후 브레이크 최고온도와 최소 효율
- numeric guard와 비정상 상태

첫 랩은 출발·가속 영향을 받으므로 정속 비교에서 제외하고 2~5랩을 평가한다.
다만 첫 랩의 off-track·contact·정지 상태는 안전성에서 제외하지 않는다.

### 정적 비교 회귀

기존 기준을 최소 보존한다.

```text
MAE < 6.0km/h
RMSE < 9.0km/h
|Bias| < 4.0km/h
최대 절대 오차 < 35.0km/h
```

1차 개선 목표는 평균 지표를 악화시키지 않으면서 국부 최대 절대 오차를
`25km/h 이하`로 낮추는 것이다. 동적 결과와 원본 표본 정렬상 25km/h가
비현실적임이 증명되면 threshold를 조용히 올리지 말고 코너별 근거를 보고한다.

### 동적 5랩 승인 조건

- 5랩 완료
- 2~5랩 유효 랩 4개
- contact, off-track과 numeric guard hit 0
- 차량 정지 또는 진행 불능 0
- 동일 조건 유효 랩 변동폭 1% 이내
- fastest valid lap이 예선 median 기준에서 ±3% 이내
- 반복되는 동일 코너 run-wide 0
- body heading P95와 횡오차가 기존 Bahrain 안전 경계를 넘지 않음

예선 랩타임을 맞추기 위해 연료·타이어·pace 조건을 불명확하게 섞지 않는다.
보고서에 연료, C 코드, 마모, 온도와 pace mode를 반드시 기록한다.

---

## 5.4 3단계 — 회로별 컨트롤러 보정

### 작업

2단계 보고서에서 반복적으로 큰 오차가 있는 구간만 보정한다.

Red Bull Ring 우선 관찰 구간:

- T1 진입과 출구
- T3 Remus hairpin 진입·가속
- T4 제동
- Rauch downhill sweep
- Wurth complex
- Rindt와 최종 T9/T10

각 후보 변경마다 다음 순서로 실행한다.

```text
정적 128표본
  → 실제 단독 5랩
  → 코너별 결과 비교
  → 채택 또는 원복
```

### 금지

- 정적 RMSE만 좋아졌다는 이유로 채택
- apex 속도는 맞지만 실제 차량이 트랙 밖으로 나가는 후보 채택
- T1을 고치기 위해 모든 코너의 전역 제동력을 변경
- `controller_speed_scale_floor` 또는 segment speed factor를 큰 폭으로 한 번에 변경
- 같은 커밋에서 트랙 폭, 라인, 제동계수와 타이어 계수를 모두 변경

### 완료 조건

- 단독 5랩 승인 조건 통과
- 변경 전보다 코너별 국부 오차 감소
- 평균 텔레메트리 지표 무회귀
- 선택한 계수군과 기각한 후보가 calibration 보고서에 기록됨
- Bahrain 관련 표적 회귀 통과

---

## 5.5 4단계 — 20대 단축 레이스

### 권장 실행 행렬

```text
회로: Red Bull Ring
환경: NORMAL 18/30°C
랩: 12
차량: 20
seed: 7, 42, 2026
배속: 우선 1x, 통과 뒤 2x
```

각 seed에서 최소 다음 구간을 별도로 집계한다.

- lights out부터 T1 탈출
- 첫 랩 T3/T4
- DRS 활성 뒤 배틀
- 중반 깨끗한 교통
- 마지막 두 랩

### 수집 항목

- contact와 충돌 심각도
- 리타이어와 SC/VSC 자연 발생
- track limit, run-wide, off-track
- 정지 또는 30초 이상 비정상 저속 차량
- 추월 시도·철회·완료
- 2/3/4-wide와 forced-wide
- planner fallback 및 emergency plan
- 랩타임 분포와 차량 간 spread
- pose 최대 이동거리와 순간이동
- CPU 처리시간, effective speed, Python RSS

### 승인 조건

- 세 seed 모두 레이스가 종료되거나 지정 12랩에 도달
- 출발·T3에서 반복되는 집단 pile-up이 없음
- 트랙 밖 정지 또는 속도 제한 고착 0
- 비접촉 순간이동 0
- 동일 위치에서 전 차량이 반복 run-wide하지 않음
- numeric guard hit 0
- planner fallback이 지속 상태로 고착되지 않음
- 2x에서 물리 스텝과 결과 계약이 1x와 일치

자연 사고 1건만으로 즉시 실패시키지 않지만 같은 seed·같은 코너에서 반복되는
접촉은 시스템 결함으로 분류한다. 사고를 없애기 위해 collision 판정을 약화하지
않는다.

---

## 5.6 5단계 — 피트와 강제 SC

### 피트 시나리오

최소 다음을 포함한다.

1. 단독 피트인·정차·피트아웃
2. 본선 차량이 있는 상태의 피트 합류
3. 같은 팀 두 차량의 인접 피트 호출
4. 피트 직전 또는 직후 배틀 상태

검사:

- 80km/h 제한선 진입 감속과 제한 유지
- 팀별 박스 정차 위치
- 정차 시간과 타이어 전환 시점
- 피트 경로의 순간이동·역주행·잔디 횡단
- `yield → hold → merge` 상태
- 합류 뒤 순위와 timing
- C2/C3/C4 role·physical code·history 일치

이번 단계는 피트 타이어 90°C 기준으로 수행한다. 70°C 콜드 아웃랩을 섞지 않는다.

### 강제 SC 시나리오

```text
정상 레이스
  → 실제 hazard 생성
  → SC 출동
  → 대열 수집
  → 최소 한 차량 피트
  → SC2에서 순서 확정
  → IN THIS LAP
  → 재시작
  → SC 피트 복귀
```

검사:

- 코너 catch-up 과속과 run-wide 0
- 후미 차량 속도 제한 고착 0
- 피트 차량의 임시 순위와 SC2 확정이 한 번만 발생
- 대열 중 추월 또는 순위 진동 0
- 재시작 뒤 정상 목표속도 복구
- SC 객체·상태·이벤트 cleanup

### 완료 조건

- 피트 전체 사이클과 SC 전체 수명주기 표적 테스트 추가
- 위 네 피트 시나리오 및 강제 SC 1회 통과
- 기존 Bahrain SC/피트 회귀 무회귀

---

## 5.7 6단계 — C2/C3/C4 열·마모 검증

NORMAL 18/30°C에서 먼저 수행한다.

### 단독 비교

동일 차량·연료·pace·seed로 C2, C3, C4를 각각 실행한다.

검사:

- fresh pace: 일반적으로 C4 > C3 > C2
- degradation/durability: C2 > C3 > C4
- warm-up: C4가 C2보다 빠르거나 같음
- 각 컴파운드의 작동창과 thermal grip 변화
- 전·후륜 표면/코어 peak
- 브레이크 온도·효율
- 열원별 누적 budget

### 20대 결과

- start role과 physical C 코드 일치
- 피트 전후 role·C 코드·마모·온도 history 일치
- 컴파운드별 hot diagnostic threshold 사용
- legacy 130°C 지표와 compound hot 지표 혼동 없음
- surface/core numeric guard hit 0
- 완화 구간 또는 피트 뒤 과열 회복

### 보정 원칙

Red Bull Ring 하나에서 나온 결과만으로 전역 C2/C3/C4 사양을 바꾸지 않는다.

다음 순서로 판단한다.

1. 회로 데이터와 주행 명령 오류
2. 특정 코너의 과도한 slip/lockup
3. 환경값과 실제 세션 근거
4. 여러 회로에서 동일하게 재현되는 컴파운드 문제

전역 컴파운드 변경은 최소 Bahrain과 Red Bull Ring 양쪽의 증거가 있을 때 별도
작업으로 제안한다.

### 완료 조건

- 세 컴파운드 상대 순서가 설명 가능
- 장기 열 폭주와 guard hit 0
- 특정 축의 지속 과열이 있으면 열원을 식별
- threshold 상향으로 결과를 숨기지 않음

---

## 5.8 7단계 — COOL/HOT 민감도와 최종 승인

NORMAL 전체 조건이 통과한 뒤에만 실행한다.

| 프리셋 | 대기 | 노면 | 목적 |
|---|---:|---:|---|
| COOL | 10°C | 18°C | C2 워밍업과 저온 그립 |
| NORMAL | 18°C | 30°C | 1차 승인 기준 |
| HOT | 28°C | 48°C | C4 과열·마모와 냉각 여유 |

COOL/HOT 값은 현재 `provisional` 시나리오다. 실제 공식 측정값이라고 보고하지
않는다.

### 승인 조건

- 온도 순서가 모델 결과에 방향성 있게 반영
- COOL에서 영구적인 미가열·저속 고착 없음
- HOT에서 guard hit와 회복 불가능한 열 폭주 없음
- 환경만 바꾼 비교에서 발열 입력 계약이 불필요하게 달라지지 않음
- NORMAL 기준선은 환경 보정 뒤에도 동일하게 재현

---

## 6. 자동 테스트 요구사항

가능하면 다음 테스트 파일에 추가하거나 책임이 커지면 Red Bull Ring 전용 파일을
만든다.

| 영역 | 권장 위치 |
|---|---|
| 원본·대섹터·DRS·피트 | `backend/tests/test_engine.py` 또는 `test_track_data_validation.py` |
| 정적 텔레메트리 | `backend/tests/test_telemetry_calibration.py` |
| 동적 단독 랩 | 신규 `backend/tests/test_circuit_runtime_baseline.py` |
| 20대 교통·피트·SC | `backend/tests/test_engine.py` |
| C2/C3/C4 | `backend/tests/test_tire_compound_nominations.py` 및 열 테스트 |
| 환경 민감도 | `backend/tests/test_circuit_thermal_profiles.py` |

긴 20대 12랩·다중 seed 시험을 전체 단위 테스트 discovery마다 실행해 개발 속도를
망가뜨리지 않는다.

- 빠른 결정론적 smoke는 기본 테스트
- 긴 매트릭스는 명시적 integration 명령
- 최종 패키지 수동 시험은 제품 승인

긴 시험을 단축 surrogate로 대체했다면 무엇을 실제 엔진으로 검증하지 못했는지
명시한다.

---

## 7. 필수 검증 명령

### 7.1 형상과 데이터

```bash
cd /Users/kimyongjin/Desktop/f1/backend

.venv/bin/python tools/validate_track_data.py --circuit-id 4
.venv/bin/python tools/audit_tier_a.py --circuit-id 4
```

### 7.2 Red Bull Ring 표적 회귀

```bash
.venv/bin/python -m unittest \
  tests.test_telemetry_calibration.RedBullRingTelemetryCalibrationTests

.venv/bin/python -m unittest \
  tests.test_engine.RaceSetupTests.test_red_bull_ring_pit_lane_is_anchored_to_track \
  tests.test_engine.RaceSetupTests.test_red_bull_ring_segment_lookup_matches_expected_driving_sections \
  tests.test_engine.RaceSetupTests.test_red_bull_ring_segments_have_corner_specific_speed_factors \
  tests.test_engine.RaceSetupTests.test_red_bull_ring_geo_source_matches_official_layout_anchors \
  tests.test_engine.RaceSetupTests.test_red_bull_ring_drs_zones_stay_on_straights
```

### 7.3 관련 물리·타이어 회귀

```bash
.venv/bin/python -m unittest \
  tests.test_vehicle_physics \
  tests.test_track_physics \
  tests.test_track_surface \
  tests.test_local_trajectory_planner \
  tests.test_tire_compound_nominations \
  tests.test_circuit_thermal_profiles
```

### 7.4 전체 회귀

```bash
.venv/bin/python -m unittest discover -s tests
```

### 7.5 프런트엔드

```bash
cd /Users/kimyongjin/Desktop/f1/frontend
npm test
npm run lint
npm run build
```

### 7.6 문서·diff

```bash
cd /Users/kimyongjin/Desktop/f1
git diff --check
```

새 `run_circuit_baseline.py`를 구현하면 실제 최종 CLI와 출력 예를 이 문서에
반영한다. 현재 구현된 명령과 출력 산출물은 다음과 같다.

```bash
cd /Users/kimyongjin/Desktop/f1/backend

.venv/bin/python tools/run_circuit_baseline.py \
  --circuit-id 4 \
  --thermal-preset NORMAL \
  --mode single-car \
  --laps 5 \
  --seed 42 \
  --tire-role MEDIUM \
  --pace-mode STANDARD \
  --output data/calibration/red_bull_ring_runtime_baseline_v2.json
```

출력 JSON에는 `static_telemetry_metrics`, `tier_a_audit`,
`single_car_dynamic.corners`, `single_car_dynamic.safety`, `thermal`,
`brakes`, `nomination`, `sector_timing_source`가 포함된다. 2026-07-31 기준
Red Bull Ring 단독 결과는 5랩 완료와 contact/off-track/numeric guard 0이지만,
fastest valid lap `70.149s` 대 예선 median `64.472s`와 T4/T10 run-wide가 남아
동적 5랩 승인에는 미달한다. 예선 비교 입력을 분리하려면 다음을 사용한다.

```bash
.venv/bin/python tools/run_circuit_baseline.py \
  --circuit-id 4 --thermal-preset NORMAL --mode single-car \
  --laps 5 --seed 42 --tire-role SOFT --pace-mode ATTACK \
  --output data/calibration/red_bull_ring_runtime_baseline_v3_qualifying_input.json
```

이 실행도 `69.568s`, 예선 median 대비 `7.904%`, T4/T10 run-wide `16/10회`로
동적 승인에는 미달했다. 존재하지 않는 명령을 실행했다고 보고하지 않는다.

---

## 8. 수동 앱 승인

새 macOS 패키지에서 다음 순서로 확인한다.

1. Red Bull Ring / NORMAL / 12랩 선택
2. 예선 실행 후 Soft가 `C4`로 표시되는지 확인
3. 20대 출발과 T1/T3 통과 관찰
4. DRS 세 구간과 순위·랩타임 확인
5. 한 차량을 C2 또는 C3로 피트 호출
6. 80km/h 제한, 팀 박스, 타이어 변경, 본선 합류 확인
7. 실제 hazard로 SC 강제
8. SC 대열 수집, 피트아웃 순위, 재시작 확인
9. 타이어·브레이크 진단에서 축별 온도와 grip 확인
10. 결과 또는 중간 종료 뒤 `New Race`
11. 60초 cleanup checkpoint 확인

수동 승인 기록:

- 앱/패키지 버전
- seed와 환경
- 출발 컴파운드와 피트 컴파운드
- 완주 차량 수
- 접촉·리타이어·SC
- 최고/최저 랩타임
- 전·후륜 surface/core peak
- 컴파운드 hot 진단과 guard hit
- 전·후 브레이크 peak와 최소 효율
- Electron/Python/전체 메모리 peak
- FPS/P95/slow frame
- cleanup 시 client, loop, WebSocket, canvas, geometry, texture, pose

---

## 9. 완료 체크리스트

- [ ] 변경 전 기준 JSON이 생성됐다.
- [ ] Red Bull Ring sector timing range가 출처와 함께 명시됐다.
- [ ] Tier A 감사에서 모든 필수 항목이 통과했다.
- [ ] 정적 텔레메트리 지표가 기존 기준보다 악화되지 않았다.
- [ ] 실제 0.02초 엔진의 단독 5랩이 통과했다.
- [ ] 코너별 속도·제동·스로틀 오차 보고가 생성됐다.
- [ ] 채택한 회로별 계수와 기각한 후보가 기록됐다.
- [ ] 20대·12랩·3 seed 단축 검증이 통과했다.
- [ ] 피트 전체 사이클이 통과했다.
- [ ] 강제 SC 전체 수명주기가 통과했다.
- [ ] C2/C3/C4 상대 성능·마모·워밍업이 설명 가능하다.
- [ ] NORMAL에서 열 폭주와 numeric guard hit가 없다.
- [ ] NORMAL 통과 뒤 COOL/HOT 민감도를 확인했다.
- [ ] 1x/2x 결정성과 cleanup을 확인했다.
- [ ] Bahrain 표적 회귀와 전체 Backend/Frontend 검증이 통과했다.
- [ ] calibration JSON과 `CURRENT_PROJECT_STATUS.md`가 갱신됐다.
- [ ] 콜드 아웃랩·70°C 피트 타이어를 이번 변경에 섞지 않았다.

모든 항목을 통과하기 전에는 Red Bull Ring을 `calibrated`로 표시하지 않는다.
부분 완료라면 정확히 어느 단계까지 완료했는지 기록한다.

---

## 10. 최종 보고 형식

```text
구현 단계:
변경 파일:
사용한 원본과 출처:

변경 전:
- 형상/Tier A:
- 정적 텔레메트리:
- 동적 단독 랩:
- 20대:
- 피트/SC:
- 타이어/브레이크:

변경 후:
- 형상/Tier A:
- 정적 텔레메트리:
- 동적 단독 랩:
- 20대:
- 피트/SC:
- 타이어/브레이크:

채택한 계수군:
기각한 후보와 이유:
테스트 명령과 결과:
수동 앱 결과:
남은 위험:
다음 단계 진입 가능 여부:
```

수치 없이 “자연스러워졌다”, “최적화됐다”라고만 보고하지 않는다.

---

## 11. 다음 에이전트용 시작 프롬프트

```text
/Users/kimyongjin/Desktop/f1 프로젝트에서 Red Bull Ring 최적화 1차 작업을 진행해라.

반드시 먼저 다음 문서를 순서대로 읽어라.
1. /Users/kimyongjin/Desktop/f1/docs/SIMULATION_FOUNDATION.md
2. /Users/kimyongjin/Desktop/f1/docs/CURRENT_PROJECT_STATUS.md
3. /Users/kimyongjin/Desktop/f1/docs/ADDING_REAL_CIRCUITS.md
4. /Users/kimyongjin/Desktop/f1/docs/RED_BULL_RING_OPTIMIZATION_PHASE_1_DIRECTIVE.md
5. /Users/kimyongjin/Desktop/f1/docs/COLD_OUTLAP_CONTROL_DESIGN.md

RED_BULL_RING_OPTIMIZATION_PHASE_1_DIRECTIVE.md의 단계, 불변식, 산출물과 승인 조건을
따라라. 사용자의 기존 작업 트리 변경을 보존하고, 관련 없는 파일을 되돌리거나
삭제하지 마라.

핵심 원칙:
- circuit_id=4, NORMAL 18/30°C를 첫 기준으로 사용한다.
- Bahrain에서 승인한 전역 물리·타이어 계수를 먼저 고정한다.
- 변경 전 기준 JSON을 먼저 남긴다.
- 정적 target profile과 실제 0.02초 엔진 동적 5랩을 분리해 검증한다.
- 한 실험에서 한 계수군만 변경한다.
- 전체 랩타임만 맞추기 위해 base_lap_time이나 speed_factor를 임의 조정하지 마라.
- 실제 근거 없는 sector, 경계, 연석 또는 surface zone을 만들지 마라.
- 피트 타이어 70°C와 콜드 아웃랩은 이번 범위에서 구현하지 마라.
- numeric guard 또는 진단 threshold를 올려 실패를 숨기지 마라.

우선 0~2단계를 구현하고 검증하라:
1. 현재 기준선과 Tier A 결과를 calibration JSON에 저장
2. 공식 근거를 확인해 Red Bull Ring sector timing range 보강
3. 재사용 가능한 run_circuit_baseline.py를 만들어 단독 동적 5랩과 코너별
   속도·제동·스로틀·안전·열 진단을 출력

그 결과로 실제 병목을 특정한 뒤에만 3단계 회로별 컨트롤러 보정을 수행하고,
이후 20대·피트·SC·C2/C3/C4 검증으로 진행하라.

각 단계가 끝날 때 변경 전후 수치, 실행한 테스트, 실패 또는 보류 항목을 보고하라.
최종적으로 전체 테스트를 실행하고 calibration JSON과
CURRENT_PROJECT_STATUS.md를 갱신하라.
```
