# ABSTRACT 레이스 시뮬레이션 단계 3 작업 지시서

작성일: 2026-08-04
대상 브랜치: `codex/abstract-race-simulation`
상위 문서: [`ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md)
선행 단계: [`ABSTRACT_RACE_SIMULATION_STAGE2_WORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE2_WORK_DIRECTIVE.md)

## 1. 단계와 전제

단계 1의 결과/hash/bounded replay와 단계 2의 FULL 공통 geometry/local metric 좌표 계약은
독립 검증을 통과했다. 이번 작업은 **단계 3 — 단일 차량 종·횡 키네마틱 pose**만 수행한다.

목표는 한 차량이 정지 grid, lights-out 출발, 직선 가속, 제동, 코너, 탈출과 하나의 안전한
line transition을 순간이동 없이 표현하는 것이다. 이는 presentation kinematics이며 FULL 강체
물리나 타이어 힘을 복제하는 작업이 아니다.

단계 4의 20대 car-following·추월·순위 crossing, 단계 5의 pit state/route 주행, 단계 6의
incident/VSC/SC/restart는 구현하지 않는다. 논리 grid, finish order, logical event와 canonical
result hash도 변경하지 않는다.

## 2. 현재 문제

현재 `SingleVehiclePoseSynthesizer.pose_at()`은 compiled racing line의 한 progress 지점을 정확히
sample하지만, `sequence()`는 다음 한계가 있다.

- 하나의 `progress_rate_per_s`로 일정 속도를 만든다.
- segment 경계의 가속·제동 전이가 없다.
- `track_distance_m = progress × circuit.track_length_m`를 사용해 racing-line distance와 섞인다.
- lateral offset이 프레임마다 즉시 바뀌며 duration·횡속도·횡가속도 계약이 없다.
- grid hold와 launch가 없다.
- `real_duration × speed_multiplier`로 logical sequence를 만들기 때문에 1×/2×/5×의 pose 열이
  동일하다는 계약을 표현하지 못한다.
- stage 1 broadcast는 logical checkpoint 중심이므로 전체 57랩 pose를 미리 만들어 저장해서는
  안 된다.

## 3. 대원칙

1. **거리 권위는 compiled racing line distance 하나다.**
2. **logical pose와 wall-clock 재생을 분리한다.**
3. **가속·제동·횡이동은 명시적인 presentation limit 안에서 적분한다.**
4. **매 pose는 공통 compiled sampler에서 다시 계산한다.**
5. **한 랩 계획 또는 작은 rolling window만 유지한다.** 57랩 전체 pose 배열을 결과에 넣지 않는다.
6. **presentation pose는 logical result hash의 권위가 아니다.** 별도 pose-sequence hash로 결정성만
   검증한다.
7. **단계 3에서는 차량 한 대만 승인한다.** 동일 pose 모델을 20대 교통으로 확장하지 않는다.

## 4. 권장 구조

이름은 구현자가 조정할 수 있지만 역할은 분리한다.

```text
TrackDisplayGeometry / TrackPhysicsProfile
  -> RacingLineDistanceContract
     - progress <-> line distance
     - distance -> local metric pose
  -> LongitudinalKinematicPlan
     - periodic target-speed knots
     - acceleration/braking-limited integration
     - grid hold and launch state
  -> LateralTrajectory
     - start/end offset, duration, easing
     - lateral velocity/acceleration
     - boundary-safe envelope
  -> SingleVehiclePoseSynthesizer
     - fixed logical tick pose
     - position, heading, speed, acceleration diagnostics
  -> bounded replay/preview sink
```

`TrackPhysicsProfile`의 public 함수를 우선 사용한다.

- `line_distance_at_total_progress()`
- `total_progress_at_line_distance()`
- `line_pose_at_progress_m()`
- `length_for_line()`

ABSTRACT가 FULL `RaceEngine` 인스턴스나 private 필드에 의존하게 만들지 않는다.

## 5. 필수 구현 작업

### 5.1 수정 전 실패 진단

Bahrain(id `3`)과 Red Bull Ring(id `4`)에서 기존 `sequence()`로 다음 실패를 수치화한다.

- 모든 구간이 사실상 같은 속도인 표본
- segment 경계 speed/acceleration discontinuity
- 즉시 lateral offset 변경 시 횡속도·횡가속도 무한대 또는 상한 초과
- `track_distance_m`과 compiled racing-line distance 차이
- 1×/2×/5× 호출의 logical frame 수/hash 차이
- grid hold/launch 부재

실패 재현 테스트를 먼저 작성하고 수정 뒤 같은 테스트가 통과하게 한다.

### 5.2 racing-line 거리 계약

- `progress × circuit.track_length_m`를 pose 거리 권위로 사용하지 않는다.
- racing-line distance와 total progress 변환은 단계 2의 compiled profile public API 한 곳에서만
  수행한다.
- 한 랩 길이는 `racing_line_length_m`를 사용한다.
- lap wrap을 포함하는 누적 거리와 lap-local 거리를 명확히 분리한다.
- pose의 `track_distance_m` 필드 의미를 `line_distance_m`으로 바꾸거나, 호환 필드를 유지한다면
  정확한 의미와 단위를 문서화한다.
- `distance -> progress -> world pose` 왕복 오차와 wrap을 테스트한다.

### 5.3 target-speed knot와 연속 속도 계획

구간마다 일정 속도를 직접 지정하지 말고, racing-line 거리 축에 periodic target-speed knot를
만든다.

- knot 입력은 abstract `SegmentRequirement`, base segment time, segment 특성과 compiled line
  distance를 사용한다.
- knot 사이 목표 속도는 선형 점프가 아닌 monotone cubic, Hermite, smoothstep 등 연속 transition
  curve로 보간한다.
- 코너 진입 전에 감속이 시작되도록 거리 기반 backward braking pass 또는 동등한 anticipation을
  둔다.
- 코너 탈출은 acceleration-limited forward pass로 연결한다.
- 정상 주행 target/actual speed는 `370km/h`를 넘지 않는다.
- 이 값은 presentation guard다. FULL의 엔진 출력·타이어 grip·제동 계수를 가져오거나 변경하지
  않는다.

권장 별도 immutable 결과:

- knot distance/progress
- target speed
- transition 종류/구간
- plan version과 tick size

### 5.4 longitudinal 적분

고정 logical tick을 기준으로 이전 상태에서 다음 상태를 적분한다.

- 기준 logical tick은 현재 계약의 `0.10초`를 유지한다. 내부 정확도를 위해 substep을 사용해도
  외부 pose tick과 결정성은 유지해야 한다.
- acceleration 상한: `+18m/s²`
- braking magnitude 상한: `50m/s²`
- `v_next`를 먼저 제한하고 trapezoidal 또는 동등한 방식으로 `distance_next`를 계산한다.
- 한 tick의 실제 위치 이동은 `v × dt + 0.5m`를 넘지 않아야 한다. 여기서 어떤 시점의 `v`를
  사용했는지 테스트와 문서에 명시한다.
- speed/acceleration은 presentation diagnostics로만 제공하고 force, throttle, brake pressure,
  slip, tire temperature를 가짜로 만들지 않는다.
- 음수 속도, 역주행, distance 감소가 없어야 한다.

### 5.5 lateral trajectory

offset을 즉시 대입하지 말고 시간 기반 trajectory로 표현한다.

- 입력: 시작 offset, 목표 offset, 시작 logical time, duration
- 최소 quintic smoothstep 또는 위치·속도·가속도가 양 끝에서 연속인 easing 사용
- 최대 횡속도: `8m/s`
- 최대 횡가속도: `20m/s²`
- 요청 duration이 상한을 만족하지 못하면 안전한 최소 duration으로 늘리거나 명시적으로 거절한다.
- 각 progress의 left/right width, racing-line offset, 차량 반폭과 edge margin으로 허용 offset
  envelope를 계산한다.
- 허용 범위를 벗어난 목표는 silent hard snap하지 말고 clamp/reject 사유를 diagnostic에 남긴다.
- heading은 compiled line tangent를 기본으로 하고, lateral velocity가 있으면 연속적인 작은 yaw
  보정을 사용할 수 있다. heading을 임의 상수로 점프시키지 않는다.
- 단계 3에서는 중앙선↔하나의 안전한 offset transition만 검증한다. 공격·방어 corridor 의미를
  붙이지 않는다.

### 5.6 grid hold와 lights-out launch

단계 2의 shared `grid_slots`를 사용해 단일 차량의 시작 pose를 만든다.

- grid 단계에서는 speed/acceleration이 `0`이고 위치와 heading이 고정된다.
- 두 grid column의 lateral offset과 position별 종방향 간격을 그대로 사용한다.
- lights-out logical time 이후 speed가 0에서 연속적으로 증가한다.
- launch도 `+18m/s²` presentation 상한을 지킨다.
- grid slot에서 racing line으로의 횡이동은 lateral trajectory 계약을 사용한다.
- launch는 표시 pose만 바꾸며 logical race start, grid order, finish order를 바꾸지 않는다.

### 5.7 logical sequence와 배속 분리

현재 `real_duration_s × speed_multiplier` 기반 sequence API는 제품 기본 경로에서 제거하거나
호환 wrapper로 격리한다.

- canonical API는 logical start/end time 또는 logical tick 범위를 입력받는다.
- 같은 seed/plan/logical interval이면 1×/2×/5× 모두 같은 pose frame index와 값이 나와야 한다.
- speed multiplier는 wall-clock scheduling에만 적용한다.
- 별도 presentation pose hash를 canonical JSON/고정 precision으로 계산해 동일성을 검증한다.
- pose hash를 `AbstractRaceResult.canonical_result_hash`에 추가하지 않는다.

### 5.8 bounded broadcast/preview 연결

- 단계 3에서는 단일 probe/선택 차량의 pose를 bounded rolling buffer 또는 현재 logical time에서
  on-demand로 합성할 수 있는 인터페이스만 연결한다.
- 20대 전체 pose replay, 57랩 full-frame tuple을 만들지 않는다.
- client가 없거나 pause 상태면 logical cursor와 pose cursor가 진행하지 않는다.
- speed change/pause/resume 뒤에도 같은 logical pose sequence의 잔여 구간을 사용한다.
- 기존 stage 1 checkpoint/event 순서와 cleanup 계약을 유지한다.

## 6. 자동 승인 테스트

### 6.1 속도·가속·이동 상한

Bahrain/RBR 각각 최소 한 랩, start launch 포함 표본에서 다음을 측정한다.

- max speed `<= 370km/h`
- max longitudinal acceleration `<= +18m/s²`
- max braking magnitude `<= 50m/s²`
- 음수 speed `0회`
- distance reversal `0회`
- tick displacement 위반 `0회`

평균 속도만 확인하지 말고 모든 logical tick을 검사한다.

### 6.2 lateral 상한과 경계

두 회로에서 최소 다음 transition을 검사한다.

- racing line `0m -> +2m -> 0m`
- racing line `0m -> -2m -> 0m`
- grid column offset -> racing line

모든 tick에서:

- lateral speed `<= 8m/s`
- lateral acceleration `<= 20m/s²`
- 차량 외곽+margin이 track boundary 안에 있음
- offset/heading/position jump 없음

### 6.3 lap wrap과 corner chord

- progress `0.99 -> 1.01`을 통과하는 sequence에서 position과 wrapped heading이 연속이어야 한다.
- 정확히 한 랩 차이인 동일 local distance의 world pose가 일치해야 한다.
- Bahrain/RBR 전체 코너에서 연속 pose 사이 chord를 여러 subpoint로 검사해 차체 중심 또는
  지정 margin이 track corridor를 가로질러 이탈하지 않아야 한다.
- 매 tick pose가 compiled sampler 결과와 일치하는지 확인한다.

### 6.4 배속 결정성

같은 logical interval에 대해 1×/2×/5×를 재생하고 다음을 확인한다.

- logical frame count 동일
- frame index/logical time 동일
- position/heading/speed/acceleration/lateral 값 동일
- pose sequence hash 동일
- wall-clock 완료 시간만 배속에 따라 달라짐

### 6.5 grid/launch

- lights-out 전 pose 이동 `0`
- 두 grid column과 longitudinal spacing이 shared grid geometry와 일치
- lights-out 첫 tick 위치/speed/acceleration 연속
- launch 후 racing line 합류까지 boundary 이탈과 lateral 상한 위반 `0`
- result grid/finish/hash 불변

### 6.6 단계 1·2 비회귀

- Bahrain/RBR geometry parity 기준 유지
- 10/57랩 grid, finish order, event 의미와 canonical hash 유지
- frame/state 권위 저장 `0/0`
- 57랩 elapsed/RSS/retained collection 상한 유지
- replay clock speed/pause/resume/close 유지
- FULL 기본 모드와 기존 RaceInfo/pit/grid 수치 유지

## 7. 필수 진단 산출물

재현 가능한 JSON 또는 CSV를 남긴다.

- 회로/plan version/logical tick
- speed min/max, acceleration/braking max
- tick displacement max와 위반 수
- lateral speed/acceleration max
- boundary clearance min과 위반 수
- lap-wrap position/heading delta
- corner-chord 위반 수
- 1×/2×/5× pose hash
- grid hold/launch 최초 3초의 핵심 표본

파일 이름 예시:

`docs/ABSTRACT_STAGE3_KINEMATICS_DIAGNOSTIC.json`

수치 생성용 tool/script는 직접 실행 가능하고 결정적이어야 한다.

## 8. 수동 확인

Bahrain과 RBR에서 한 차량만 표시해 다음을 확인한다.

- 두 줄 grid에서 정지
- lights-out launch와 racing line 합류
- 긴 직선 가속
- 주요 제동 구간
- 좌측/우측 상단 코너
- lap start/finish wrap
- 한 번의 좌우 line transition

속도가 코너마다 갑자기 변하거나 차량이 chord로 코너 안쪽을 가로지르거나 heading이 튀지 않는지
확인한다. 가능하면 같은 viewport screenshot 또는 짧은 녹화본을 남긴다. GUI 확인이 불가능하면
해당 지점의 연속 pose 진단을 제출한다.

## 9. 수정 대상 예상 파일

- 새 `backend/simulation/abstract/kinematics.py` 또는 동등 역할 파일
- `backend/simulation/abstract/pose.py`
- `backend/simulation/abstract/clock.py`
- 필요 시 `backend/simulation/abstract/broadcast.py`, `replay.py`
- `backend/simulation/abstract/state.py`는 presentation DTO 추가만 허용
- `backend/tests/test_abstract_stage3_kinematics.py`
- 기존 abstract/stage 1/stage 2/API 테스트
- 단일 차량 renderer 연결이 필요한 최소 frontend 파일과 테스트
- 진단 script/tool과 JSON
- `docs/CURRENT_PROJECT_STATUS.md`

`backend/simulation/abstract/race.py`를 수정한다면 logical result를 바꾸지 않는 presentation sink
경계인지 증명해야 한다. `race_engine.py`, FULL 물리, vehicle/tire model과 circuit JSON은 수정하지
않는 것이 원칙이다.

## 10. 금지 사항

- 20대 car-following·간격 보정·추월·방어·순위 crossing 구현
- existing attack/contact/pit/incident 확률이나 결과 balance 변경
- pit route 주행·pit state machine 구현
- VSC/SC/restart 구현
- FULL 속도·가속·타이어·차량 계수 수정
- 가짜 force, throttle, brake pressure, slip, tire temperature 생성
- 결과 순위를 맞추기 위한 progress/position 강제 이동
- 57랩 전체 pose 배열을 result/snapshot에 저장
- pose sequence를 logical result canonical hash에 포함
- 테스트 상한 완화 또는 synthetic track만으로 완료 선언
- 사용자 요청 없는 commit, push, branch 변경, package 생성
- 기존 미커밋 변경 reset/checkout/clean/stash

## 11. 실행할 검증

최소 다음을 실행한다.

- Stage 3 kinematics tests
- 기존 abstract/result/replay/API tests
- Stage 2 Bahrain/RBR geometry tests
- 관련 FULL track/grid/RaceInfo 회귀
- 10/57랩 stage 1 benchmark
- frontend test/lint/production build(연결 변경이 없더라도 비회귀 확인)
- backend `compileall`
- 문서 상대 링크 검사
- `git diff --check`

전체 backend discovery를 실행하지 않았다면 반드시 미실행으로 기록한다.

## 12. 완료 보고 형식

```text
단계: 3 — 단일 차량 종·횡 키네마틱 pose
판정: 완료 후보 / 조건부 / 실패
단계 4 시작 여부: 시작하지 않음

구현 요약:
- 거리/속도/lateral/grid-launch 계약
- logical pose와 wall-clock 분리
- bounded presentation 연결

Bahrain:
- max speed, acceleration, braking
- lateral speed/acceleration
- min boundary clearance, violation count
- tick displacement/chord/wrap 결과
- 1x/2x/5x pose hash

Red Bull Ring:
- 동일 항목

비회귀:
- Stage 1 hash/자원
- Stage 2 geometry
- FULL 관련 테스트
- frontend 결과

수동 확인/진단:
- screenshot·video 또는 진단 파일 경로

미실행/알려진 한계:
- 전체 backend discovery 여부
- Stage 4 이후로 남긴 항목

변경 파일:
- 파일 목록
```

보고 후 코드를 더 수정하지 말고 독립 검증을 기다린다.

## 13. 다른 에이전트 전달용 프롬프트

```text
작업 저장소는 /Users/kimyongjin/Desktop/f1 입니다.
현재 브랜치는 codex/abstract-race-simulation 입니다.

먼저 다음 문서를 처음부터 끝까지 읽으세요.
1. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE3_WORK_DIRECTIVE.md
2. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md
3. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE2_WORK_DIRECTIVE.md
4. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_DESIGN.md
5. /Users/kimyongjin/Desktop/f1/docs/SIMULATION_FOUNDATION.md
6. /Users/kimyongjin/Desktop/f1/docs/CURRENT_PROJECT_STATUS.md의 ABSTRACT 단계 1~2 및 최신 절

이번 작업은 "단계 3 — 단일 차량 종·횡 키네마틱 pose"만 수행합니다. 단계 1과 2는 독립
검증을 통과했습니다. 단계 4의 20대 교통·추월·순위 crossing, 단계 5의 pit 주행, 단계 6의
incident/VSC/SC/restart와 제품 패키징은 구현하지 마세요.

현재 작업 트리에는 검증을 통과한 단계 1~2 미커밋 변경이 있습니다. 전부 사용자 작업으로
간주하고 보존하세요. git reset, checkout, clean, stash 또는 대량 되돌리기를 하지 마세요.
사용자 요청 없이 commit, push, branch 변경, package 생성도 하지 마세요.

핵심 목표는 단계 2의 compiled racing-line/local metric geometry 위에서 한 차량이 grid hold,
lights-out launch, 직선 가속, 제동, 코너, 탈출과 하나의 lateral transition을 연속적인 logical
pose로 표현하게 만드는 것입니다. 이는 presentation kinematics이며 FULL 물리나 타이어 힘을
복제하는 작업이 아닙니다.

반드시 다음 순서로 진행하세요.
1. Bahrain(id 3)과 Red Bull Ring(id 4)에서 현재 constant progress-rate, distance 불일치,
   즉시 lateral offset, 배속별 logical sequence 차이와 grid/launch 부재를 수치화하고 실패
   테스트를 먼저 만드세요.
2. compiled racing line의 progress↔line distance public API를 유일한 거리 권위로 사용하세요.
   progress×circuit.track_length_m를 pose 거리 권위로 사용하지 마세요.
3. segment/base-time 기반 periodic target-speed knot와 연속 transition curve를 만들고 거리 기반
   braking anticipation 및 acceleration-limited forward transition을 구현하세요.
4. 고정 0.10초 logical tick에서 speed/distance를 적분하고 max speed 370km/h,
   acceleration +18m/s², braking magnitude 50m/s²와 tick displacement 한계를 지키세요.
5. start/end offset, logical duration과 easing을 가진 lateral trajectory를 구현하세요. 횡속도
   8m/s, 횡가속도 20m/s², local track boundary와 차량 반폭/margin을 모두 검사하세요.
6. 단계 2 shared grid slot으로 정지 pose와 lights-out launch를 구현하세요. grid/finish/result
   hash에는 영향을 주지 마세요.
7. canonical pose API를 logical interval 기준으로 만들고 1×/2×/5×는 같은 pose sequence/hash를
   사용하며 wall-clock scheduling만 달라지게 하세요.
8. 매 tick마다 compiled sampler에서 position/heading을 다시 계산하고 Bahrain/RBR lap wrap,
   corner chord, boundary 이탈과 순간이동이 없는지 전 구간 검사하세요.
9. 단일 probe/선택 차량만 bounded rolling buffer 또는 on-demand interface에 연결하세요.
   20대 전체 또는 57랩 full-frame replay를 저장하지 마세요.
10. Stage 3 tests, 기존 abstract/API/replay, Stage 2 geometry, 관련 FULL 회귀, 10/57랩 benchmark,
    frontend test/lint/build, compileall, 문서 링크와 git diff --check를 실행하세요.
11. docs/ABSTRACT_STAGE3_KINEMATICS_DIAGNOSTIC.json에 회로별 max speed/accel/braking,
    lateral 상한, boundary clearance, displacement/chord/wrap 위반, 1×/2×/5× pose hash와 launch
    표본을 남기세요.
12. 작업 지시서의 완료 보고 형식으로 결과를 보고하고 즉시 멈추세요. 단계 4는 시작하지 마세요.

논리 grid, finish order, logical event 의미와 AbstractRaceResult canonical hash를 변경하지 마세요.
가짜 force/throttle/brake/slip/열 데이터를 만들지 말고 FULL 물리·차량·타이어·회로 보정값을
수정하지 마세요. 테스트 기준을 완화하거나 synthetic track만으로 완료 선언하지 마세요.
전체 backend discovery를 실행하지 않았다면 미실행이라고 정확히 기록하고, 목표 미달 항목은
조건부 또는 실패로 보고한 뒤 독립 검증을 기다리세요.
```
