# ABSTRACT 단계 3 보완 작업 지시서

작성일: 2026-08-04
대상 브랜치: `codex/abstract-race-simulation`
선행 문서: [`ABSTRACT_RACE_SIMULATION_STAGE3_WORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE3_WORK_DIRECTIVE.md)

## 1. 판정과 작업 범위

단계 3 최초 구현은 자동 테스트는 통과했지만 독립 검증에서 **실패**했다. 이번 작업은 아래 세
결함만 수정하는 **단계 3 보완**이다.

1. chord 상한 처리 후 line distance, speed, acceleration이 서로 불일치한다.
2. 테스트와 진단이 저장된 acceleration 필드만 검사해 실제 pose 이동의 상한 위반을 놓친다.
3. 단일 차량 bounded kinematic pose가 실제 ABSTRACT broadcast에 연결되지 않았다.

단계 4의 20대 교통·추월·순위 crossing, 단계 5 pit 주행, 단계 6 incident/VSC/SC/restart는
구현하지 않는다. FULL 물리와 논리 결과도 변경하지 않는다.

## 2. 확인된 실패 수치

0.10초 logical tick, 180초 단일 차량 표본에서 `line_distance_m` 차분으로 계산한 실제 운동은
다음과 같았다.

| 회로 | distance-derived acceleration | 저장된 acceleration 필드 | 최대 적분 거리 오차 | 1mm 초과 tick |
| --- | ---: | ---: | ---: | ---: |
| Bahrain | `-57.2411 ~ +27.3526m/s²` | `-29.3120 ~ +12.0346m/s²` | `0.409814m` | 44 |
| Red Bull Ring | `-49.3974 ~ +10.9704m/s²` | `-30.9492 ~ +10.7458m/s²` | `0.319818m` | 8 |

Bahrain은 실제 거리 이동 기준으로 가속 `+18m/s²`, 제동 `50m/s²` 승인 상한을 모두 넘었다.
따라서 기존 `ABSTRACT_STAGE3_KINEMATICS_DIAGNOSTIC.json`의 acceleration 수치는 승인 증거로
사용할 수 없다.

## 3. 원인

`SingleVehiclePoseSynthesizer.sequence()`는 먼저 `LongitudinalKinematicPlan.integrate()`가
계산한 distance/speed/acceleration을 받는다. 이후 sampled world chord가 이동 상한을 넘으면
`candidate_distance`만 이분법으로 줄이지만, `current_speed`와 acceleration은 원래 step 값을
유지한다.

```text
integrator step
  distance = D1
  speed = V1
  acceleration = A1

chord correction
  distance = D2  # D2 < D1
  speed = V1     # 불일치
  acceleration = A1  # 불일치
```

근본적으로 `TrackPhysicsProfile`의 line-distance 축과 실제 Catmull–Rom world sampler의 호 길이가
완전히 같은 parameterization이 아니어서 world chord가 line-distance increment보다 커질 수 있다.

## 4. 보완 대원칙

1. distance, speed, acceleration은 하나의 accepted step에서 원자적으로 확정한다.
2. 거리만 사후 축소하고 기존 속도·가속도를 유지하지 않는다.
3. 가능하면 world pose에 사용하는 동일 spline의 arc-length를 거리 권위로 만든다.
4. 검증은 저장 필드를 신뢰하지 않고 pose 차분으로 다시 계산한다.
5. 앱 연결은 차량 한 대만 대상으로 하며 20대 교통 의미를 만들지 않는다.
6. presentation pose와 pose hash를 logical result canonical hash에 넣지 않는다.

## 5. 필수 수정 1 — 동일 spline arc-length 계약

### 5.1 권장 해결 방식

단계 2의 compiled racing line world sampler와 동일한 곡선을 충분히 조밀하게 sample한 periodic
arc-length lookup table을 만든다.

필수 기능:

- spline progress별 누적 실제 local-metric arc length
- `total_progress -> spline_arc_distance_m`
- `spline_arc_distance_m -> total_progress`
- `arc_distance_m -> world_x_m/world_y_m/heading`
- lap wrap을 포함한 누적 거리 변환

LUT 해상도는 Bahrain/RBR에서 distance/world chord 오차를 승인 tolerance 안에 넣는 근거로
결정한다. 임의로 회로별 보정 상수를 추가하지 않는다.

기존 `TrackPhysicsProfile.line_distance_at_total_progress()`의 path distance가 world sampler와
정확히 일치하도록 공통 public 구현을 개선할 수 있다면 그 방식을 우선한다. 단, FULL 물리 결과나
racing-line calibration을 바꿔서는 안 된다. 위험이 있으면 ABSTRACT presentation 전용 immutable
arc-length adapter를 둔다.

### 5.2 금지되는 해결

- x/y 위치를 tick 후 사후 clamp
- progress를 임의 비율로 축소
- 회로별 magic scale 적용
- speed/acceleration 필드만 실제 이동에 맞는 것처럼 덮어쓰기
- 테스트 tolerance만 확대

## 6. 필수 수정 2 — 원자적인 kinematic step

최종 accepted step은 적어도 다음을 함께 가진 immutable 값이어야 한다.

- previous/next arc distance
- displacement
- previous/next speed
- acceleration
- target speed
- correction/substep 여부와 사유

모든 정상 tick에서 다음 관계가 tolerance 내에서 성립해야 한다.

```text
ds = next_distance - previous_distance
ds ≈ (previous_speed + next_speed) / 2 × dt
acceleration ≈ (next_speed - previous_speed) / dt
```

상한:

- `next_speed >= 0`
- `next_speed <= 370km/h`
- `acceleration <= +18m/s²`
- `acceleration >= -50m/s²`
- `ds >= 0`
- world movement `<= previous_speed × dt + 0.5m`

### 6.1 world chord가 여전히 상한을 넘는 경우

거리만 사후 줄이지 않는다. 다음 방법 중 하나를 사용한다.

- 동일 arc-length sampler로 원인을 제거한다.
- deterministic substep으로 적분과 sampling을 함께 다시 수행한다.
- 해당 tick에 허용되는 next speed/acceleration을 먼저 계산한 뒤 accepted displacement를 다시
  적분한다.

어떤 경우에도 accepted distance에서 speed와 acceleration을 재유도하고 전체 상한을 다시
검사한다. 상한을 동시에 만족하지 못하면 더 앞선 tick의 target-speed planning/braking
anticipation을 조정해야 한다.

## 7. 필수 수정 3 — 실제 차분 회귀 테스트

기존 Stage 3 테스트에 다음 검증을 추가한다.

### 7.1 integrator identity

Bahrain/RBR 각 180초 이상에서 모든 tick을 검사한다.

```text
abs(ds - (v_prev + v_next) / 2 × dt) <= tolerance
abs(frame_accel - (v_next - v_prev) / dt) <= tolerance
```

권장 distance tolerance는 `1e-6m` 수준이다. LUT 근사 때문에 더 큰 값이 필요하면 실제 최대
오차를 보고하고 근거 없이 `1mm` 이상으로 완화하지 않는다.

### 7.2 distance-derived metrics

연속 `line_distance_m`에서 interval speed를 계산하고, 연속 interval speed에서 derived
acceleration을 계산한다.

- derived max acceleration `<= +18m/s²`
- derived braking magnitude `<= 50m/s²`
- reported/derived speed 최대 오차 기록
- integrator identity mismatch count `0`
- distance reversal `0`

world chord speed 변화는 곡률 영향을 받으므로 longitudinal acceleration과 구분해 기록하되,
시각적인 jump/jerk 진단으로 별도 보고한다.

### 7.3 실패 지점 고정

최소 다음 기존 실패 지점 주변을 회귀 표본에 포함한다.

- Bahrain 약 `34.8s`, progress `0.42444`
- Bahrain 약 `90.9s`, progress `0.13158`
- RBR 약 `156.9~157.0s`, progress `0.29789~0.29960`

시간 자체를 golden으로 고정하기보다 해당 corner/progress window에서 identity와 상한을 검사한다.

## 8. 필수 수정 4 — 진단 도구 갱신

`backend/tools/diagnose_abstract_stage3_kinematics.py`와
`docs/ABSTRACT_STAGE3_KINEMATICS_DIAGNOSTIC.json`을 갱신한다.

반드시 추가할 필드:

- `derived_line_speed_min/max_mps`
- `derived_line_acceleration_min/max_mps2`
- `reported_vs_derived_speed_error_max_mps`
- `integrator_distance_error_max_m`
- `integrator_identity_violation_count`
- `reported_acceleration_identity_violation_count`
- `post_integrator_distance_correction_count`
- `world_chord_speed_jump_max_mps`

기존 stored-field max acceleration만으로 통과 판정하지 않는다. 진단 script는 목표 수치를 넘으면
non-zero exit 또는 명확한 `approval_passed: false`를 출력해야 한다.

## 9. 필수 수정 5 — 단일 probe broadcast 연결

`AbstractBroadcastSession`에 단일 선택 차량용 kinematic presenter를 연결한다.

### 9.1 허용 범위

- 기본 probe는 player driver 한 명 또는 명시된 진단 driver 한 명
- `BoundedKinematicPoseBuffer` 또는 현재 logical time on-demand 합성
- pose payload에는 해당 차량 한 대만 포함
- `presentation_contract`에 Stage 3 single-probe임을 명확히 표시
- grid hold, lights-out launch, 정상 한 랩 sequence 확인
- pause 동안 pose cursor 정지
- resume 후 잔여 logical time에서 재개
- 1×/2×/5×에서 logical pose hash 동일
- close 후 buffer/task/Event 정리

### 9.2 금지 범위

- 나머지 19대 pose 생성
- probe를 실제 leader나 순위 차량으로 취급
- logical checkpoint 순위를 probe pose로 변경
- 추월·접촉·car-following 구현
- 57랩 pose 전체 tuple 저장

UI에는 필요하면 `Stage 3 single-car preview`임을 표시해 실제 20대 경기 표현으로 오인되지 않게
한다.

## 10. 승인 기준

다음 조건을 모두 만족해야 단계 3 완료 후보로 다시 제출할 수 있다.

### 운동학

- Bahrain/RBR derived acceleration `<= +18m/s²`
- Bahrain/RBR derived braking magnitude `<= 50m/s²`
- integrator identity violation `0`
- reported acceleration identity violation `0`
- post-integrator distance-only correction `0`
- distance reversal `0`
- max speed `<= 370km/h`
- tick/world movement 위반 `0`
- lateral speed/acceleration/boundary 위반 `0`
- lap wrap 불연속 `0`

### broadcast

- 실제 WebSocket pose payload에 단일 probe 1대가 존재
- logical checkpoint/event/race_end 순서 불변
- pause/resume/speed 변경 중 pose 누락·중복·cursor jump 없음
- buffer capacity 초과 없음
- close cleanup 후 pose buffer/task/Event/client `0`

### 비회귀

- Stage 1 10/57랩 canonical hash 동일
- frame/state 권위 저장 `0/0`
- Stage 2 Bahrain/RBR geometry parity 유지
- FULL 기본 모드, 물리 결과와 관련 테스트 유지
- frontend test/lint/build 통과

## 11. 실행할 검증

- 수정된 Stage 3 kinematics tests
- 실제 broadcast single-probe API/WebSocket tests
- 기존 abstract/result/replay/API tests
- Stage 2 geometry tests
- 관련 FULL track/grid/RaceInfo/pit 회귀
- 10/57랩 Stage 1 benchmark
- frontend test/lint/production build
- backend `compileall`
- 문서 상대 링크 검사
- `git diff --check`

전체 backend discovery를 실행하지 않으면 미실행이라고 정확히 기록한다.

## 12. 완료 보고 형식

```text
단계: 3 보완 — arc-length/atomic step/derived diagnostics/single-probe broadcast
판정: 완료 후보 / 조건부 / 실패
단계 4 시작 여부: 시작하지 않음

원인 수정:
- 동일 spline arc-length 계약
- accepted step의 distance/speed/acceleration 원자성
- distance-only correction 제거 여부

Bahrain derived 결과:
- speed min/max
- acceleration min/max
- integrator distance error max
- identity violation counts
- tick/chord/boundary/wrap

RBR derived 결과:
- 동일 항목

Broadcast:
- probe driver와 payload count
- 1x/2x/5x hash
- pause/resume/close cleanup

비회귀:
- Stage 1 benchmark/hash
- Stage 2 geometry
- FULL/frontend 테스트

미실행/한계:
- 전체 backend discovery 여부
- Stage 4 이후 범위

변경 파일:
- 파일 목록
```

보고 후 단계 4를 시작하지 말고 독립 검증을 기다린다.

## 13. 다른 에이전트 전달용 프롬프트

```text
작업 저장소는 /Users/kimyongjin/Desktop/f1 입니다.
현재 브랜치는 codex/abstract-race-simulation 입니다.

먼저 다음 문서를 처음부터 끝까지 읽으세요.
1. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE3_CORRECTION_DIRECTIVE.md
2. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE3_WORK_DIRECTIVE.md
3. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md
4. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE2_WORK_DIRECTIVE.md
5. /Users/kimyongjin/Desktop/f1/docs/CURRENT_PROJECT_STATUS.md의 ABSTRACT 단계 1~3 최신 절

이번 작업은 독립 검증에서 실패한 "단계 3 보완"만 수행합니다. 단계 4의 20대 교통·추월·순위
crossing, pit 주행, incident/VSC/SC/restart와 제품 패키징은 구현하지 마세요.

현재 작업 트리에는 단계 1~3 미커밋 변경이 있습니다. 전부 사용자 작업으로 간주하고 보존하세요.
git reset, checkout, clean, stash, 대량 되돌리기를 하지 마세요. 사용자 요청 없이 commit, push,
branch 변경, package 생성도 하지 마세요.

확인된 핵심 결함은 pose.py에서 world chord 상한 초과 시 candidate_distance만 이분법으로 줄이고
speed/acceleration은 원래 KinematicStep 값을 유지하는 것입니다. Bahrain의 실제 line-distance
차분 가속도는 -57.24~+27.35m/s²인데 저장 필드는 -29.31~+12.03m/s²로 보고했습니다. 최대 적분
거리 오차는 0.410m이고 1mm 초과 불일치 tick은 44개였습니다.

반드시 다음 순서로 진행하세요.
1. 현재 실패를 distance-derived speed/acceleration과 integrator identity 테스트로 먼저 고정하세요.
2. world pose에 사용하는 동일 compiled spline을 조밀하게 sample한 periodic arc-length LUT 또는
   동등한 public adapter를 구현해 distance와 world sampler parameterization을 일치시키세요.
3. distance만 사후 축소하는 보정을 제거하세요. accepted next distance, displacement, speed,
   acceleration을 하나의 immutable step에서 원자적으로 확정하세요.
4. 모든 tick에서 ds≈(v_prev+v_next)/2×0.10과 a≈(v_next-v_prev)/0.10을 검증하세요.
5. Bahrain/RBR 180초 표본에서 derived acceleration <=+18m/s², braking magnitude <=50m/s²,
   integrator identity 위반 0, distance reversal 0, speed <=370km/h를 달성하세요.
6. diagnose_abstract_stage3_kinematics.py가 저장 필드가 아닌 pose 차분을 계산하도록 바꾸고,
   derived speed/acceleration, reported-vs-derived 오차, integrator distance error, violation count와
   post-correction count를 JSON에 기록하세요. 실패 시 approval_passed=false 또는 non-zero exit를
   내게 하세요.
7. AbstractBroadcastSession에 player driver 한 명만 bounded/on-demand kinematic probe로 연결하세요.
   pose payload에는 1대만 넣고 Stage 3 preview임을 표시하세요. 나머지 19대와 교통/순위 의미는
   만들지 마세요.
8. probe가 pause 중 정지하고 resume 후 잔여 logical time에서 재개되며 1x/2x/5x logical pose hash가
   같고 close 뒤 buffer/task/Event/client가 정리되는 API/WebSocket 테스트를 추가하세요.
9. 기존 logical checkpoint/event/race_end 순서, AbstractRaceResult hash, grid/finish/event 의미,
   Stage 1 frame/state 0/0과 Stage 2 geometry parity를 유지하세요.
10. 수정된 Stage 3, broadcast/API, 기존 abstract/replay, Stage 2 geometry, 관련 FULL 회귀,
    10/57랩 benchmark, frontend test/lint/build, compileall, 문서 링크와 git diff --check를 실행하세요.
11. 작업 지시서의 완료 보고 형식으로 보고하고 즉시 멈추세요. 단계 4는 시작하지 마세요.

FULL 물리·차량·타이어·회로 calibration을 수정하지 말고, 가짜 force/throttle/brake/slip/열 데이터를
만들지 마세요. x/y hard clamp, 회로별 magic scale, 속도 필드만 덮어쓰기, tolerance 완화로 문제를
숨기지 마세요. 전체 backend discovery를 실행하지 않았다면 미실행이라고 정확히 기록하고 독립
검증을 기다리세요.
```
