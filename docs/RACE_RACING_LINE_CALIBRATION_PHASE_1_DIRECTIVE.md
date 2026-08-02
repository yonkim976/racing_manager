# 레이스 기준 레이싱라인 보정 1차 작업 지시서

- 문서 상태: **구현 전 작업 기준**
- 작성일: **2026-08-02**
- 대상 회로: Bahrain International Circuit (`circuit_id=3`), Red Bull Ring (`circuit_id=4`)
- 대상 세션: **레이스**
- 제외 세션: **퀄리파잉** — 후속 단계에서 별도 보정
- 구현 상태: **미구현** — 이 문서는 코드나 회로 데이터를 변경하지 않는다
- 최종 목표: 레이스 연료·지정 Medium·Standard 페이스에서 실제 차량이 기준선을
  늦게 쫓거나 반대편으로 튀지 않고, 코너의 바깥–안쪽–바깥 흐름을 안정적으로
  반복하도록 두 회로의 기본 레이싱라인과 추종 계약을 승인한다.

선행 문서:

1. [`SIMULATION_FOUNDATION.md`](SIMULATION_FOUNDATION.md)
2. [`CURRENT_PROJECT_STATUS.md`](CURRENT_PROJECT_STATUS.md)
3. [`ADDING_REAL_CIRCUITS.md`](ADDING_REAL_CIRCUITS.md)
4. [`RED_BULL_RING_OPTIMIZATION_PHASE_1_DIRECTIVE.md`](RED_BULL_RING_OPTIMIZATION_PHASE_1_DIRECTIVE.md)
5. [`AI_RACECRAFT_BENCHMARK.md`](AI_RACECRAFT_BENCHMARK.md)

---

## 1. 이번 단계의 결정

### 1.1 레이스 기준을 먼저 승인한다

이번 작업에서 `racing_line`은 레이스의 기본선이다. 다음 조건에서 빠르면서도
반복 가능한 선을 우선한다.

| 항목 | Bahrain | Red Bull Ring |
|---|---:|---:|
| 환경 프리셋 | `NORMAL` | `NORMAL` |
| 대기 / 노면 | 30 / 40°C | 18 / 30°C |
| 주말 Medium의 물리 컴파운드 | C2 | C3 |
| 페이스 | `STANDARD` | `STANDARD` |
| 물리 스텝 | 0.02초 / 50Hz | 0.02초 / 50Hz |
| 기본 검증 | 단독 5랩 + 15랩 stint | 단독 5랩 + 15랩 stint |

단독 5랩은 궤적과 코너 진입을 빠르게 회귀 검증하는 용도이고, 15랩 stint는
연료 감소·타이어 온도·마모가 변해도 같은 선이 안정적인지 확인하는 용도다.
최종 승인은 20대 교통 표본까지 포함한다.

### 1.2 퀄리파잉은 이번 단계에서 만들지 않는다

이번 단계에서 다음을 구현하지 않는다.

- 퀄리파잉 전용 `racing_line` 또는 별도 회로 좌표
- C4/C5·`ATTACK`을 기준으로 한 선 보정
- 예선 한 랩 기록만 줄이기 위한 공격적 연석 사용
- `line_aggression=qualifying` 같은 세션별 가중치
- 레이스와 퀄리파잉 사이의 타이어 준비랩 제어

퀄리파잉은 레이스 기본선 승인 뒤 같은 기하를 공유하면서 연석·폭·열·위험 비용만
다르게 주는 2차 작업으로 진행한다. 이번 1차 구현에서 퀄리파잉을 미리 가정한
분기나 숫자를 넣지 않는다.

### 1.3 슬립은 제거 대상이 아니다

현재 기본 주행의 `traction_slip_ratio` 최대치는 두 회로 모두 약 1.7%이며,
`wheelspin`, `oversteer`, `traction_loss` 이벤트는 발생하지 않았다. 이 범위의 미세
종방향 슬립은 구동력을 만드는 정상 물리 범위로 취급한다.

따라서 레이싱라인 문제를 해결하기 위해 다음을 바꾸지 않는다.

- 전역 타이어 그립
- 전역 슬립 곡선 또는 traction-loss 임계값
- 엔진 출력과 다운포스
- C1~C5 열창·마모·수명
- 타이어 열·냉각 계수

가시적인 휠스핀이나 차체 슬라이드가 새로 발생하면 해당 레이싱라인 후보는
실패다. 반대로 1~2% 미세 슬립을 0으로 만드는 것은 성공 조건이 아니다.

---

## 2. 현재 확인된 기준선

아래 값은 2026-08-02 단독 5랩 진단 표본이다. 구현자는 작업 시작 시 같은
조건으로 새 기준 JSON을 저장하고, 숫자가 달라졌다면 원인을 먼저 기록한다.

### 2.1 전체 결과

| 지표 | Bahrain | Red Bull Ring |
|---|---:|---:|
| fastest valid lap | 92.907초 | 66.990초 |
| 예선 reference median | 90.091초 | 64.472초 |
| 차이 | +2.816초 | +2.518초 |
| lateral error p95 | 4.785m | 5.759m |
| lateral error max | 6.141m | 7.918m |
| heading error p95 | 0.105rad | 0.105rad |
| contact / off-track / track-limit | 0 | 0 |

예선 reference는 진행률·속도·제동·횡위치의 비교 자료다. Medium·Standard 레이스
표본을 예선 랩타임과 같게 만드는 목표값이 아니다. 레이스 승인에서는 기존
레이스 기준 대비 회귀 여부와 궤적·열·마모 안정성을 우선한다.

### 2.2 Bahrain 주요 추종 오차

| 우선순위 | 구간 | 현재 최대 오차 | 관찰 |
|---|---|---:|---|
| P0 | 코너 landmark / progress | 최대 약 228m 정렬 의심 | 코너별 보고의 귀속부터 불안정 |
| P1 | T4–T5–T6 | T5 5.62m | 실제 차량과 reference가 반대쪽에 있는 위상 지연 |
| P1 | T11–T12 | T11 6.14m, T12 4.92m | 빠른 연속 코너에서 늦은 보정과 되돌림 |
| P2 | T7 | 2.32m | 연결 구간 보정 뒤 재평가 필요 |
| P2 | T9–T10 | T9 2.54m | 코너 라벨 정렬 뒤 판단 필요 |

Bahrain의 기존 코너 `progress`는 일부 지점에서 곡률·최저속도·제동 신호와 크게
어긋날 가능성이 있다. 예를 들어 현재 표본에서는 T1 약 +68m, T4 약 +110m,
T13 약 -228m 수준의 차이가 관찰됐다. 이 값은 자동 수정값이 아니라 **재검증이
필요하다는 진단값**이다.

### 2.3 Red Bull Ring 주요 추종 오차

| 우선순위 | 구간 | 현재 최대 오차 | 관찰 |
|---|---|---:|---|
| P1 | T5–T6–T7 | T6 7.92m, T7 5.70m | 좌측 상단 연속 코너에서 늦은 이동·큰 되돌림 |
| P1 | T9–T10 | T9 6.89m, T10 4.89m | 마지막 연속 코너의 반대편 보정 |
| P2 | T8 | 2.38m | T7과 T9 수정 뒤 재평가 |
| 유지 | T1 / T3 / T4 | 2.19 / 1.24 / 1.50m | 현재 선을 우선 보존 |

Red Bull Ring의 코너 landmark는 Bahrain보다 잘 정렬돼 있다. T1·T3·T4·T10은
곡률 특징과 대략 10~13m 범위이며, T5~T9는 넓고 이어지는 코너의 특성 때문에
30~75m의 해석 여지가 있다. 이를 개별 apex 한 점의 문제로 단순화하지 않고
`T5–T7`, `T9–T10` 연속 경로로 평가한다.

---

## 3. 레이스 기본선의 목적 함수

레이스 기본선의 우선순위는 다음과 같다.

```text
트랙 내부와 낮은 연석 허용 범위 준수
  → 연속 코너에서 단일하고 부드러운 궤적
  → 출구 가속과 다음 코너 준비
  → 반복 랩의 열·마모 안정성
  → 교통이 없을 때 조향 되돌림과 planner 개입 최소화
  → 레이스 랩타임
```

목적 함수에 최소한 다음 비용을 분리해 기록한다.

```text
J = lap_time
  + boundary_cost
  + track_limit_cost
  + path_curvature_rate_cost
  + lateral_acceleration_rate_cost
  + correction_reversal_cost
  + exit_traction_cost
  + tire_heat_cost
  + tire_wear_cost
```

### 3.1 레이스 기준의 해석

- 한 랩에서 한 번 빠르지만 랩마다 선이 흔들리는 후보는 실패다.
- 코너 중앙의 순간 최대속도보다 출구 속도와 다음 직선의 시간을 우선한다.
- 연석은 실제 표면 데이터가 있는 낮은 연석만 허용한다.
- `surface_zones`가 없는 구간을 아스팔트나 낮은 연석으로 추정하지 않는다.
- 실제 횡위치 reference는 prior다. 차량을 강제로 순간이동시키는 경로가 아니다.
- 단독 주행에서 tactical `inside/outside/defensive_line`이 활성화되면 실패다.
- 교통에서는 tactical line을 쓸 수 있지만, 위험이 해소되면 기본선으로 부드럽게
  복귀해야 한다.

### 3.2 레이스와 퀄리파잉의 경계

1차 작업은 하나의 승인된 base path를 만든다. 2차 퀄리파잉 작업은 이 base path의
형상을 다시 처음부터 만들기보다 다음 비용만 조절하는 방식으로 시작한다.

```text
race:       thermal/wear/exit stability 비용을 높게
qualifying: lap time/kerb/width 활용을 높게, 단 safety 경계는 동일
```

이 구분은 이번 단계의 설계 메모일 뿐이며 아직 코드 계약이 아니다.

---

## 4. 데이터 권위와 좌표 계약

### 4.1 권위 순서

```text
FIA/F1 공식 회로도와 코너 순서
  → OSM 중심선·피트 형상
  → TUM 좌우 폭 및 사용 가능한 경계 자료
  → 고정된 실제 텔레메트리의 속도·제동·횡위치 prior
  → 게임 안의 결정론적 최적선과 회로별 controller 보정
```

실측값, 추출값, 추정값과 게임 밸런스값을 같은 필드에 섞지 않는다. 모든 progress
변환에는 원본 기준점, 방향, start/finish offset과 컴파일 뒤 변환식을 기록한다.

### 4.2 반드시 명시할 좌표계

구현 전 다음 값을 한 표본에 함께 출력하는 진단을 먼저 만든다.

- compiled centerline progress
- active racing-line path progress와 path distance
- telemetry source progress와 정렬된 progress
- 중심선 기준 reference lateral offset
- active line pose 기준 차량 lateral offset
- target lateral offset
- 차량 lateral speed
- reference line의 거리 미분으로 계산한 lateral speed 후보
- heading error와 곡률

서로 다른 프레임의 두 `lateral_offset_m`을 직접 빼지 않는다. 현재 오차가 실제
차량의 경로 이탈인지, 중심선과 racing-line pose 사이의 좌표 변환 차이인지 먼저
증명해야 한다.

### 4.3 controller 변경 조건

현재 clean-car 경로는 active line pose가 미래 경로를 이미 휘어간다는 전제에서
추가 lateral-speed feed-forward를 0으로 둔다. 이 계약은 임의로 뒤집지 않는다.

다음 A/B 재생으로 증거가 있을 때만 변경한다.

1. 현재 line-relative 제어: clean target lateral speed `0`
2. 동일 좌표계에서 계산한 reference offset derivative feed-forward
3. 각 후보에서 궤적 오차, 조향 반전, 경계 여유, 랩타임을 비교
4. active line pose에 이미 포함된 횡이동을 두 번 적용하지 않았음을 단위 테스트

단순히 clean-car 횡속도 상한을 올리거나 응답 시간을 줄이는 방식은 1차 수정으로
금지한다. 최대 오차만 줄고 조향 반전·타이어 열·경계 접근이 증가할 수 있기
때문이다.

---

## 5. 단계별 구현 순서

## 5.1 0단계 — 변경 전 기준선 고정

### 작업

1. 기존 사용자 변경을 보존하고 관련 파일의 SHA-256과 git identity를 기록한다.
2. 두 회로를 동일한 레이스 조건으로 각각 3회 실행한다.
3. seed, driver, 연료, Medium role, physical compound, 환경과 스텝을 JSON에 남긴다.
4. 전체·코너별 궤적 오차와 슬립·열·마모를 함께 저장한다.

권장 산출물:

```text
backend/data/calibration/bahrain_race_line_baseline_v1.json
backend/data/calibration/red_bull_ring_race_line_baseline_v1.json
```

3회 결과의 fastest lap만 고르지 않는다. lap time median, 코너별 p95, 최악값과
반복성을 모두 사용한다.

### 통과 조건

- 3회 모두 5랩 완료
- 입력 계약이 JSON에 완전하게 기록됨
- 각 지표가 같은 좌표계인지 보고서에 표시됨
- 현재 기준과 차이가 있으면 원인 설명 전까지 보정 작업을 시작하지 않음

## 5.2 1단계 — 진단 보강과 오류 분류

`backend/tools/run_circuit_baseline.py`의 기존 단독 주행 진단을 확장하거나 별도
도구를 만든다. 코너별로 다음을 보고한다.

- reference와 actual의 signed lateral error, MAE, p95, max
- `abs(error) > 2m` 누적 시간과 최대 연속 시간
- error zero-crossing과 correction reversal 횟수
- entry/apex/exit의 속도·스로틀·브레이크
- 최저속도 위치와 출구 50m/100m 속도
- understeer/oversteer/wheelspin/traction-loss 표본
- planner fallback, run-wide, track-limit과 접촉
- 앞/뒤 타이어 surface/core peak와 slide heat

각 문제를 아래 네 종류 중 하나로 분류한다.

| 분류 | 의미 | 우선 해결 위치 |
|---|---|---|
| `ALIGNMENT` | landmark·telemetry·compiled progress 불일치 | 데이터/변환 |
| `REFERENCE_PATH` | prior 자체가 경계·연속성·레이스 목적에 부적합 | 회로 calibration |
| `TRACKING_CONTROL` | reference는 적절하나 차량이 늦게 따라감 | 좌표/제어 계약 |
| `SPEED_PROFILE` | 선은 맞지만 진입·apex·출구 속도가 부적절 | 회로별 controller |

한 문제에 여러 분류를 동시에 붙일 수 있지만, 한 실험에서는 한 계수군만 바꾼다.

## 5.3 2단계 — Bahrain progress와 코너 귀속 수정

### 작업

1. start/finish 기준과 주행 방향을 먼저 검증한다.
2. 각 T1~T15에 대해 공식 코너 순서, 중심선 곡률 peak, 텔레메트리 브레이크 시작,
   최저속도와 횡위치 극값을 나란히 표시한다.
3. 넓은 코너는 단일 점 외에 `entry/apex/exit` window를 정의한다.
4. landmark를 변경할 때는 출처와 변환식을 `circuits.json` 근처의 source metadata
   또는 calibration 산출물에 남긴다.
5. DRS, sector, pit, surface와 기존 테스트가 같은 progress 계약을 쓰는지 확인한다.

### 금지

- runtime 차량의 최저속도 위치에 landmark를 맞춰 순환 논리를 만드는 것
- 인접 코너의 곡률 peak 하나를 두 코너가 공유하도록 자동 배정하는 것
- 코너 오차 수치가 작아지도록 label만 이동하는 것

### 통과 조건

- T1~T15가 순서대로 중복 없이 고유 window를 가짐
- corner window가 실제 곡률/제동 특징을 포함함
- DRS·sector·pit 관련 기존 테스트 무회귀
- 새 코너별 보고서에서 `unattributed` 비율이 기준보다 증가하지 않음

## 5.4 3단계 — Bahrain 레이스 라인 보정

다음 순서로 한 묶음씩 보정한다.

1. **T4–T5–T6**: T4 출구가 T5 진입 준비로 이어지고, T5에서 반대편을 뒤늦게
   쫓지 않도록 전체 transition을 최적화한다.
2. **T11–T12**: T11의 방향 전환과 T12의 고속 통과를 하나의 연속 곡선으로
   평가한다. T11의 순간 apex 오차만 줄이지 않는다.
3. **T7**, **T9–T10**: 앞 두 묶음 수정 뒤 남은 오차만 처리한다.
4. T1·T2·T8·T13·T14·T15는 명확한 회귀가 없으면 유지한다.

각 후보에서 코너 진입 150m부터 출구 150m까지 actual/reference overlay와
속도·브레이크·스로틀을 저장한다. 시각적으로만 판단하지 않고 같은 구간의 수치를
JSON으로 남긴다.

## 5.5 4단계 — Red Bull Ring 레이스 라인 보정

다음 순서로 진행한다.

1. **T5–T6–T7**: 좌측 상단 구간을 세 개의 독립 apex가 아니라 한 경로로 다룬다.
   T6 최대 7.92m와 T7 최대 5.70m 오차, 늦은 되돌림을 우선 제거한다.
2. **T9–T10**: T9 진입에서 T10 출구까지 방향 반전 횟수와 출구 속도를 함께
   최적화한다.
3. T8은 양쪽 묶음 변경 뒤 다시 측정한다.
4. T1·T3·T4는 현재 기준보다 나빠지면 후보 전체를 기각한다.

트랙 폭이나 렌더링 폭을 레이싱라인 오차에 맞춰 넓히지 않는다. 물리 폭과 렌더링
폭의 일치 여부는 별도 검증하되, 경로 추종 문제의 해결 수단으로 사용하지 않는다.

## 5.6 5단계 — 추종 제어 계약 보정

회로 reference를 수정한 뒤에도 actual이 같은 형태로 늦게 움직이는 경우에만
controller를 보정한다.

우선순위:

1. active line pose와 차량 lateral state의 좌표 변환
2. path distance/progress lookahead 정렬
3. clean line에서 offset error 제거 응답
4. 증명된 경우에만 lateral-speed feed-forward
5. 마지막으로 회로별 sample distance 또는 planner 설정

전역 상수를 바꿔야 한다면 Bahrain과 Red Bull Ring 외 최소 한 회로의 기존 테스트를
함께 실행한다. 회로별 calibration으로 충분한 문제를 전역 상수로 해결하지 않는다.

필수 단위 테스트:

- 직선 reference에서 불필요한 lateral speed가 생기지 않음
- 일정한 offset 변화에서 응답 방향이 reference와 같음
- line-relative pose와 centerline-relative offset을 혼합하지 않음
- feed-forward가 경로 곡률에 이미 포함된 횡이동을 두 번 적용하지 않음
- step `0.01/0.02/0.05초`에서 최종 경로와 랩타임이 허용 범위 안에 있음

## 5.7 6단계 — 레이스 stint와 교통 승인

### 단독 15랩

- 레이스 연료, Medium, Standard, NORMAL 환경
- 첫 2랩과 마지막 3랩의 경로 오차·랩타임·온도·마모 비교
- 연료 감소나 타이어 열로 correction reversal이 증가하지 않는지 확인
- Bahrain 후륜 surface/core가 기존 승인 범위를 벗어나지 않는지 확인
- 두 회로 모두 130°C 초과, clamp와 연속 과열이 0인지 확인

### 20대 교통

- race start부터 최소 10랩
- 기본선과 tactical line 전환 횟수 및 복귀 시간을 기록
- side-by-side가 끝난 뒤 기본선으로 복귀하면서 급격히 횡이동하지 않는지 확인
- 접촉을 없애려고 교통 판단을 비활성화하지 않음
- SC는 이번 레이싱라인 승인 조건이 아니지만 기존 SC 회귀 테스트는 통과해야 함

### 결정성

- 같은 seed·입력의 1x 반복 실행 결과가 허용 범위 안에서 같음
- 1x/2x는 같은 0.02초 물리 스텝을 소비하고 결과가 기존 허용오차 안에 있음

---

## 6. 정량 승인 기준

### 6.1 필수 안전 기준

두 회로 모두 다음을 만족해야 한다.

```text
contact = 0
off_track = 0
track_limit = 0
repeated_run_wide = 0
numeric_guard = 0
clean-car planner fallback = 0
wheelspin / oversteer / traction_loss event = 0
surface/core clamp = 0
130°C continuous overheat = 0
```

실제 낮은 연석 접촉은 surface 데이터와 네 바퀴 판정이 올바른 경우에만 허용한다.

### 6.2 궤적 추종 기준

| 지표 | 승인 목표 |
|---|---:|
| 전체 lateral error p95 | 2.0m 이하 |
| 전체 lateral error max | 4.0m 이하 |
| heading error p95 | 0.08rad 이하 |
| 우선 보정 구간 `abs(error)>2m` 시간 | 기준 대비 70% 이상 감소 |
| 우선 보정 구간 correction reversal | 기준 대비 증가 금지 |
| 유지 코너 max error | 기준 대비 0.5m 초과 악화 금지 |

단, 이 기준은 4.2의 동일 좌표계 진단이 증명된 뒤 적용한다. 현재 보고서의 offset이
서로 다른 프레임을 비교한 값이면 먼저 지표를 바로잡고 새 기준선을 고정한다.

### 6.3 레이스 성능 기준

- 동일 레이스 입력의 변경 전 3회 median보다 fastest valid lap이 0.5% 이상
  느려지지 않는다.
- 한 코너의 순간속도를 높이기 위해 다음 직선 100m의 시간을 악화시키지 않는다.
- 15랩의 마지막 3랩에서 correction reversal·run-wide가 증가하지 않는다.
- 기존 미세 traction slip 최대 약 1.7%를 억지로 0으로 만들지 않는다.
- 새 후보에서 traction slip 2.5% 이상이 반복되거나 이벤트가 생기면 기각한다.

예선 reference와의 차이는 보고하되, Medium·Standard 레이스 후보를 예선 median
3% 이내로 강제하는 승인 기준으로 사용하지 않는다.

### 6.4 열·마모 기준

- Bahrain 15랩 rear surface/core 결과가 기존 장기 열 승인 방향과 일치한다.
- Red Bull Ring 15랩 rear surface/core가 기존 기준보다 5°C 이상 악화되지 않는다.
- line 변경 전후 동일 구간 slide heat와 wear delta를 비교한다.
- C2/C3의 상대 수명이나 열창을 레이싱라인 작업 중 변경하지 않는다.

---

## 7. 변경 허용 파일과 금지 영역

### 7.1 예상 변경 파일

```text
backend/data/circuits.json
backend/data/calibration/*race_line*.json
backend/simulation/track_physics.py
backend/simulation/race_engine.py
backend/tools/run_circuit_baseline.py
backend/tests/test_circuit_runtime_baseline.py
backend/tests/test_track_physics.py
backend/tests/test_vehicle_physics.py
docs/CURRENT_PROJECT_STATUS.md
```

실제 원인이 더 좁으면 더 적은 파일만 변경한다. 생성된 calibration JSON에는
입력 파일 hash와 생성 명령을 기록한다.

### 7.2 이번 작업에서 변경 금지

```text
backend/simulation/tire_model.py의 C1~C5 물성
backend/simulation/vehicle_physics.py의 전역 그립·출력·공력 계수
backend/data/circuit_thermal_profiles.json
backend/data/circuit_tire_wear_profiles.json
backend/data/tire_compound_nominations.json
피트, SC와 경기 순위 계약
렌더링 폭을 이용한 물리 경계 보정
```

진단 결과 이 영역이 원인처럼 보여도 이번 구현에 포함하지 말고 별도 후속 이슈로
보고한다.

---

## 8. 검증 명령

구현자는 실제 CLI 옵션을 확인한 뒤 아래와 동등한 명령을 사용한다.

```bash
cd /Users/kimyongjin/Desktop/f1/backend

.venv/bin/python tools/validate_track_data.py --circuit-id 3
.venv/bin/python tools/validate_track_data.py --circuit-id 4

.venv/bin/python tools/audit_tier_a.py --circuit-id 3
.venv/bin/python tools/audit_tier_a.py --circuit-id 4

.venv/bin/python tools/run_circuit_baseline.py \
  --circuit-id 3 --mode single-car --laps 5 --seed 42 \
  --pace-mode STANDARD --tire-role MEDIUM \
  --output data/calibration/bahrain_race_line_candidate.json

.venv/bin/python tools/run_circuit_baseline.py \
  --circuit-id 4 --mode single-car --laps 5 --seed 42 \
  --pace-mode STANDARD --tire-role MEDIUM \
  --output data/calibration/red_bull_ring_race_line_candidate.json

.venv/bin/python -m unittest \
  tests.test_circuit_runtime_baseline \
  tests.test_track_physics \
  tests.test_trajectory_physics \
  tests.test_local_trajectory_planner \
  tests.test_vehicle_physics \
  tests.test_telemetry_calibration

git diff --check
```

현재 `run_circuit_baseline.py`에 `--pace-mode` 또는 `--tire-role`이 없으면 입력을
암묵적으로 가정하지 말고 옵션과 출력 계약부터 추가한다. 장기 15랩·20대 검증은
별도 결정론적 도구나 테스트로 자동화하고 실행 명령을 최종 보고에 기록한다.

전체 백엔드와 프런트엔드 회귀는 후보 승인 뒤 마지막에 실행한다. 중간 실험마다
전체 장기 묶음을 반복하지 않는다.

---

## 9. 실험과 변경 관리

한 실험에서 한 계수군만 바꾸고 다음 형식으로 기록한다.

```json
{
  "change_id": "race-line-p1-001",
  "circuit_id": 3,
  "session_basis": "race",
  "parameter_group": "landmark_alignment",
  "before": {},
  "after": {},
  "reason": "",
  "coordinate_contract": "",
  "single_car_5_lap": {},
  "single_car_15_lap": {},
  "traffic_20_car": {},
  "thermal": {},
  "accepted": false
}
```

실패한 후보는 코드에 남기지 않되, 어떤 방향이 왜 기각됐는지 최종 보고에 한 줄씩
남긴다. 기존 사용자의 dirty worktree를 정리하거나 되돌리지 않는다.

---

## 10. 완료 보고 형식

최종 보고에는 다음 표를 반드시 포함한다.

### 10.1 회로별 변경 전후

| 회로 | 항목 | 변경 전 | 변경 후 | 판정 |
|---|---|---:|---:|---|
| Bahrain | lateral p95 / max |  |  |  |
| Bahrain | T4–T6, T11–T12 `>2m` 시간 |  |  |  |
| Bahrain | lap / slip / rear temp / wear |  |  |  |
| RBR | lateral p95 / max |  |  |  |
| RBR | T5–T7, T9–T10 `>2m` 시간 |  |  |  |
| RBR | lap / slip / rear temp / wear |  |  |  |

### 10.2 필수 설명

- Bahrain landmark 중 실제로 변경한 항목과 근거
- reference-path 문제와 tracking-controller 문제를 구분한 증거
- clean-car lateral-speed 계약을 유지 또는 변경한 이유
- 유지하기로 한 코너에서 발생한 회귀 여부
- 15랩 열·마모 결과와 20대 교통 복귀 동작
- 실패하거나 기각한 후보
- 퀄리파잉 단계로 넘긴 미구현 항목

---

## 11. 완료 정의

다음 조건을 모두 만족해야 1차 완료다.

1. Bahrain 코너 progress와 window가 출처·변환식과 함께 검증됐다.
2. Bahrain T4–T6와 T11–T12의 늦은 반대편 보정이 정량적으로 감소했다.
3. Red Bull Ring T5–T7과 T9–T10이 하나의 부드러운 연속 경로로 실행된다.
4. 유지 코너, 피트·SC·타이어·전역 물리에 회귀가 없다.
5. 단독 5랩, 15랩 stint, 20대 교통과 1x/2x 검증이 통과했다.
6. 레이스 기준 결과가 calibration JSON과 `CURRENT_PROJECT_STATUS.md`에 기록됐다.
7. 퀄리파잉용 보정은 구현되지 않았으며 후속 단계로 명시됐다.

하나라도 남으면 `부분 완료`로 보고한다. 랩타임만 빨라진 상태는 완료가 아니다.

---

## 12. 후속 2차 — 퀄리파잉 보정 메모

레이스 기본선 승인 뒤 별도 지시서를 작성한다. 2차에서 검토할 항목은 다음뿐이다.

- Soft physical compound와 `ATTACK` 입력 계약
- 낮은 연석·트랙 폭 활용을 늘리는 `line_aggression`
- 한 랩 타이어 준비와 surface/core 온도
- 예선 실제 telemetry median에 대한 속도·제동·횡위치 비교
- race base path와 공유할 데이터, 세션별로 분리할 비용

퀄리파잉 구현은 이번 문서의 완료 조건이 아니다.

---

## 13. 다음 구현 에이전트용 프롬프트

```text
/Users/kimyongjin/Desktop/f1 프로젝트에서
docs/RACE_RACING_LINE_CALIBRATION_PHASE_1_DIRECTIVE.md를 처음부터 끝까지 읽고,
레이스 기준 레이싱라인 보정 1차 작업을 단계 순서대로 수행하라.

핵심 범위는 Bahrain(circuit_id=3)과 Red Bull Ring(circuit_id=4)의
NORMAL 환경, 주말 Medium, STANDARD 페이스다. 퀄리파잉 전용 라인이나
line_aggression은 구현하지 않는다.

먼저 dirty worktree와 사용자 변경을 보존하고 변경 전 3회 기준선을 고정하라.
Bahrain은 landmark/progress 정렬을 먼저 증명한 뒤 T4–T6, T11–T12 순서로,
Red Bull Ring은 T5–T7, T9–T10 순서로 보정하라. reference와 actual의 lateral
offset이 같은 좌표계인지 증명하기 전에는 controller 숫자를 바꾸지 말라.

현재 약 1.7%의 정상 미세 traction slip은 제거 대상이 아니다. 전역 타이어 그립,
C1~C5 물성, 열·마모 계수, 출력·공력, 피트·SC 계약과 렌더링 폭은 변경하지 말라.
한 실험에서는 한 계수군만 변경하고 변경 전후 JSON을 남겨라.

단독 5랩, 15랩 stint, 20대 교통, 1x/2x와 관련 회귀 테스트를 실행하라.
완료 보고에는 회로별 lateral p95/max, 우선 코너의 >2m 시간, correction reversal,
랩타임, slip, 온도, 마모, 안전 이벤트, 실패 후보와 남은 퀄리파잉 작업을 포함하라.
모든 완료 조건을 만족하지 않으면 부분 완료로 보고하라.
```
