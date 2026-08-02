# 콜드 피트 아웃랩 제어 설계

- 문서 상태: **설계 확정, 미구현**
- 작성일: **2026-07-31**
- 적용 예정 범위: 건식 C1~C5 피트 타이어
- 우선순위: 다중 서킷 기준선 확보 뒤 구현
- 선행 기준:
  - [`SIMULATION_FOUNDATION.md`](SIMULATION_FOUNDATION.md)
  - [`CURRENT_PROJECT_STATUS.md`](CURRENT_PROJECT_STATUS.md)
  - [`TIRE_COMPOUND_SPEC.md`](TIRE_COMPOUND_SPEC.md)

## 1. 결정

출발 그리드와 피트 타이어의 초기 상태를 분리한다.

| 상황 | 목표 초기값 | 의미 |
|---|---:|---|
| 레이스 출발 | 건식 90°C 유지 | 포메이션 랩이 생략된 현재 제품에서 포메이션 랩 완료 상태를 추상화 |
| 피트 타이어 교체 | 건식 70°C 적용 예정 | 피트 블랭킷에서 나온 차가운 새 타이어 |
| Intermediate | 현재 별도 사양 유지 | 건식 콜드 아웃랩 보정과 동시에 변경하지 않음 |
| Wet | 현재 별도 사양 유지 | 건식 콜드 아웃랩 보정과 동시에 변경하지 않음 |

현재 코드의 C1~C5 `blanket_temperature_c`는 모두 90°C이며, 출발과 피트 교체가
같은 값을 사용한다. 따라서 위 표의 70°C 피트 타이어와 콜드 아웃랩 제어는 아직
구현된 동작이 아니다.

70°C를 레이스 출발에도 바로 적용한 시험에서는 Bahrain 다중 차량 출발의 접촉,
리타이어와 SC 위험이 90°C 기준보다 증가했다. 현재 포메이션 랩이 없으므로 출발
온도만 낮추지 않는다.

## 2. 목표와 비목표

목표:

1. 피트에서 교체한 70°C 타이어의 실제 낮은 그립을 물리에 반영한다.
2. AI가 그 그립으로 수행할 수 없는 공격적 목표속도·스로틀·제동을 요구하지 않게
   한다.
3. 피트 출구 합류 차량의 낮은 가속력을 본선 차량이 예측할 수 있게 한다.
4. 온도와 그립 회복에 따라 제어 제한을 연속적으로 해제한다.
5. C1~C5의 서로 다른 워밍업 특성이 결과에 나타나게 한다.

비목표:

- 시간을 기준으로 타이어 온도를 강제로 올리는 기능
- 정확히 한 랩 뒤 정상 그립을 강제로 부여하는 기능
- 라이브 레이스 아웃랩의 과도한 지그재그 워밍업
- 포메이션 랩 전체 구현
- C1~C5 열용량·냉각계수의 동시 재보정

## 3. 핵심 원칙

### 3.1 실제 물리 그립이 단일 권위값이다

현재 차량 물리는 컴파운드, 온도와 마모로 계산한 제동·트랙션·횡그립을 이미
사용한다. 콜드 아웃랩 제어가 물리 그립을 다시 곱하면 이중 페널티가 된다.

```text
타이어 모델             → 실제 가능한 힘
콜드 아웃랩 제어        → AI가 요구하는 속도와 조작의 상한
차량 물리               → 실제 운동 결과
```

콜드 제어는 목표속도, 스로틀 요청, 제동 요청, 배틀 판단과 피트 합류 여유만
조정한다.

### 3.2 고정 랩이 아니라 실제 회복 상태로 종료한다

절대 온도 하나가 아니라 전·후륜 `thermal_grip`을 사용한다. 컴파운드마다 최적
온도와 작동창이 다르므로 같은 85°C라도 회복 정도가 다를 수 있다.

```python
minimum_thermal_grip = min(
    state.front_tire_thermal_grip,
    state.rear_tire_thermal_grip,
)
```

초기 제안값:

- 활성 유지: `minimum_thermal_grip < 0.970`
- 정상 복귀 후보: 전·후륜 모두 `thermal_grip >= 0.985`
- 정상 복귀 확정: 후보 상태를 연속 3초 유지
- 장시간 미복귀: 강제 정상화하지 않고 bounded 진단 기록

수치는 `game_calibration_provisional`이며 다중 서킷 시험으로 보정한다.

### 3.3 상태 전이는 연속적이어야 한다

```text
PIT_STOP
  → PIT_OUT_COLD
  → TRACK_OUTLAP_COLD
  → RECOVERING
  → NORMAL
```

- `PIT_OUT_COLD`: 피트박스 출발부터 합류 전까지 가속과 합류 판단을 제한한다.
- `TRACK_OUTLAP_COLD`: 본선 합류 뒤 배틀을 시작하지 않고 가능한 속도로 주행한다.
- `RECOVERING`: 온도 회복도에 따라 제한을 연속 보간한다.
- `NORMAL`: 새 타이어의 일반 전략과 공격 모드를 허용한다.

한 랩을 돌았다는 이유만으로 `NORMAL`로 바꾸지 않는다. 반대로 그립이 충분히
회복됐다면 출발선을 다시 통과할 때까지 불필요하게 제한하지 않는다.

## 4. 상태 계약

`DriverRaceState` 또는 같은 권위의 런타임 상태에 최소 다음 의미를 추가한다.
정확한 이름은 기존 schema 스타일에 맞출 수 있다.

```python
cold_outlap_active: bool
cold_outlap_phase: str
cold_outlap_elapsed_s: float
cold_outlap_recovery: float
cold_outlap_min_thermal_grip: float
cold_outlap_ready_elapsed_s: float
```

불변식:

- 새 건식 타이어 장착 직후 실제 컴파운드와 70°C로 thermal grip을 즉시 계산한다.
- 온도만 70°C로 바꾸고 `tire_thermal_grip=1.0`을 한 틱 동안 유지하지 않는다.
- 전·후륜 그립의 낮은 값이 회복 판정의 권위값이다.
- 세션 종료와 새 레이스 생성 때 모든 콜드 상태와 진단을 제거한다.
- 피트스톱을 하지 않은 차량에 `pit_count`만으로 콜드 상태를 추정하지 않는다.

## 5. 제어 계약

### 5.1 회복도

다음은 구조 예시이며 최종 계수가 아니다.

```python
recovery = clamp(
    (minimum_thermal_grip - cold_reference_grip)
    / (normal_reference_grip - cold_reference_grip),
    0.0,
    1.0,
)
```

불연속 상수 전환 대신 `recovery`로 모든 제어 제한을 보간한다.

### 5.2 목표속도

로컬 플래너는 이미 현재 타이어의 횡그립·제동·트랙션을 입력받는다. 콜드 제어는
전역 기준선이 최적 온도 차량을 전제로 과도한 목표를 내지 않도록 작은 속도
envelope를 추가한다.

```python
cold_speed_factor = lerp(0.94, 1.00, recovery)
```

동일한 계수를 물리 그립에 다시 적용하지 않는다. 직선 최고속도를 불필요하게
오래 제한하기보다 코너 진입과 가속 구간의 실행 가능성을 우선한다.

### 5.3 스로틀과 제동

후륜 회복도는 스로틀, 전륜 회복도는 제동 요구에 더 크게 연결한다.

```python
throttle_request_cap = lerp(0.75, 1.00, rear_recovery)
brake_request_cap = lerp(0.80, 1.00, front_recovery)
```

브레이크의 물리 효율을 다시 낮추지 않는다. 목표속도를 일찍 낮추고 요구 감속을
완만하게 만들어 실제 전륜 그립 안에서 제동을 끝내게 한다.

### 5.4 배틀과 페이스

콜드 상태가 다음 판단보다 우선한다.

- 새 타이어라는 이유만으로 즉시 `ATTACK` 선택 금지
- 신규 추월·방어 maneuver 시작 금지
- 이미 존재하는 maneuver는 안전하게 철회
- 충돌 회피와 race-control 명령은 항상 콜드 제한보다 우선
- 회복 뒤에는 기존 `fresh_after_stop` 공격 판단을 다시 허용

현재 `strategy_ops.py`의 `fresh_after_stop`은 연료 조건이 맞으면 `ATTACK`을
선택할 수 있으므로 70°C 피트 타이어 적용 전에 반드시 우선순위를 수정한다.

### 5.5 피트 합류

콜드 타이어 차량은 피트 출구에서 정상 차량보다 가속이 느리다. 합류 판단은 현재
속도뿐 아니라 콜드 회복도를 포함한다.

초기 제안:

```python
required_merge_gap_factor = lerp(1.35, 1.00, recovery)
```

- 뒤 차량과의 최소 gap/TTC 확대
- 합류 뒤 예상 가속도를 콜드 트랙션으로 계산
- 위험하면 강제 합류하지 않고 `yield` 또는 `hold`
- 합류 직후 레이싱라인으로 돌아가는 횡이동 속도 제한
- SC 중에는 SC 대열 순서와 SC2 확정 규칙을 우선

## 6. 구현 위치

| 파일 | 책임 |
|---|---|
| `backend/simulation/tire_model.py` | 출발과 피트용 초기 온도 계약 분리, 컴파운드별 thermal grip 계산 |
| `backend/simulation/pit_ops.py` | 교체 즉시 70°C 상태·실제 그립 설정, PIT_OUT_COLD 활성화, 합류 여유 |
| `backend/simulation/strategy_ops.py` | 콜드 상태에서 fresh tyre ATTACK 차단, 회복 후 허용 |
| `backend/simulation/race_engine.py` | 목표속도·스로틀·제동 명령의 회복도 보간 |
| `backend/simulation/racecraft_ops.py` | 신규 공격·방어 억제와 안전한 maneuver 철회 |
| `backend/models/schemas.py` | 필요한 권위 상태와 bounded 진단 필드 |

출발 코드와 피트 코드는 같은 `blanket_temperature_c` 함수를 그대로 공유하지
않도록 호출 의미를 분리한다. 예:

```python
grid_start_temperature_c(compound)
pit_blanket_temperature_c(compound)
```

## 7. 진단

차량별 무한 이력을 저장하지 않는다. 세션 진단에는 다음 bounded 집계만 남긴다.

- 콜드 아웃랩 진입 횟수
- 컴파운드별 평균·최대 회복 시간
- 가장 낮은 전륜/후륜 thermal grip
- 콜드 상태의 lockup, wheelspin, run-wide, contact 횟수
- `yield/hold/merge` 시간
- 회복 전 ATTACK 또는 신규 maneuver 요청이 차단된 횟수
- 제한 시간 안에 회복하지 못한 차량과 조건

UI에는 필요하면 `COLD TYRES`와 회복도만 표시한다. UI가 별도의 온도나 타이머를
계산하지 않는다.

## 8. 테스트

### 8.1 단위 테스트

- 출발은 90°C, 피트 교체는 70°C라는 서로 다른 초기화 경로
- 교체 프레임부터 실제 C1~C5 thermal grip이 설정됨
- 전·후륜 중 낮은 그립이 회복도를 결정
- 0.985 이상 연속 3초 전에는 해제되지 않음
- threshold 부근에서 상태가 흔들리지 않는 히스테리시스
- 콜드 상태에서 `fresh_after_stop → ATTACK` 차단
- 회복 뒤 기존 fresh tyre 공격 판단 복구
- 물리 그립이 한 번만 적용되는 불변식
- 세션 종료 뒤 상태·진단 cleanup

### 8.2 통합 테스트

최소 행렬:

| 회로 | 환경 | 지명 목적 |
|---|---|---|
| Bahrain | NORMAL/HOT | 기존 C1/C2/C3 기준 보존 |
| Red Bull Ring | COOL/NORMAL | C2/C3/C4와 짧은 랩 반복 |
| Monza | NORMAL | C3/C4/C5와 강한 제동·가속 |

각 시나리오는 단독 피트아웃, 20대 교통 피트아웃, SC 중 피트아웃과 여러 seed를
포함한다.

### 8.3 승인 조건

- 90°C 기존 기준보다 피트 합류 contact/SC/retirement가 악화되지 않는다.
- 콜드 상태의 lockup, wheelspin과 run-wide가 지속적으로 누적되지 않는다.
- 모든 건식 컴파운드가 green-flag 주행에서 정상 상태로 회복한다.
- C5의 회복이 C1보다 일반적으로 빠르거나 같다는 순서가 유지된다.
- surface/core numeric guard hit는 0이다.
- 1x/2x와 외부 tick 분할에서 상태 전이와 결과가 결정적이다.
- 온도 강제 상승이나 고정 랩 강제 해제 없이 통과한다.

## 9. 작업 순서와 보류 조건

1. 다른 서킷의 현재 90°C 기준선을 먼저 수집한다.
2. Red Bull Ring C2/C3/C4 단축 레이스를 첫 다중 서킷 승인 대상으로 삼는다.
3. 전역 물리 계수 변경 없이 회로 데이터와 controller profile 문제를 먼저
   분리한다.
4. 다중 서킷 기준선이 확보되면 출발/피트 초기 온도 계약을 분리한다.
5. 콜드 상태, 제어 보간, 합류 판단, 배틀 차단 순으로 구현한다.
6. Bahrain 회귀 뒤 Red Bull Ring과 Monza에서 승인한다.

다중 서킷 기준선 전에 콜드 아웃랩과 회로별 속도 보정을 동시에 적용하면 사고가
타이어 때문인지 트랙·컨트롤러 때문인지 구분하기 어렵다. 따라서 이 설계는
확정하되 즉시 구현하지 않고, 다음 작업은 Red Bull Ring 기준선 확보로 진행한다.
