# ABSTRACT 단계 3 재보완 작업 지시서 — 연속 single-probe broadcast

작성일: 2026-08-04
대상 브랜치: `codex/abstract-race-simulation`
선행 문서: [`ABSTRACT_RACE_SIMULATION_STAGE3_CORRECTION_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE3_CORRECTION_DIRECTIVE.md)
상위 문서: [`ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md)

## 1. 판정과 이번 작업 범위

단계 3의 spline arc-length, 원자적 accepted step과 실제 차분 운동학은 독립 검증을 통과했다.
그러나 `AbstractBroadcastSession`이 0.10초 kinematic pose stream을 재생하지 않고 logical
checkpoint 위치를 간헐적으로 직접 sample하므로 **단계 3 전체는 아직 승인하지 않는다**.

이번 작업은 다음 두 결함만 고치는 단계 3 재보완이다.

1. 실제 WebSocket 방송에서 한 차량의 pose를 0.10초 logical tick마다 연속 생성한다.
2. `post_integrator_distance_correction_count`를 상수가 아닌 실제 accepted-step/pose 비교로 계측한다.

단계 4의 20대 교통, car-following, 추월, 순위 crossing은 구현하지 않는다. 단계 5 pit 주행과
단계 6 incident/VSC/SC/restart도 구현하지 않는다. FULL 물리, logical result, checkpoint, event,
finish order와 canonical hash를 변경하지 않는다.

## 2. 독립 검증에서 확인된 실패

Bahrain 5랩, seed `44`의 실제 `AbstractBroadcastSession` checkpoint를 끝까지 재생해 single-probe
payload의 좌표를 비교한 결과는 다음과 같다.

| 항목 | 측정값 |
| --- | ---: |
| pose 표본 수 | 97 |
| 첫 pose 갱신 시점 | logical `84.2s` |
| 최대 physics-frame 간격 | 842 tick |
| 0.10초 초과 interval | 72 / 96 |
| 최대 world chord | `428.519940m` |
| 최대 line-distance 변화 | `5,205.562656m` |
| 위치 차분 최대 속도 | `116.469208m/s` (`419.289km/h`) |

`backend/simulation/abstract/broadcast.py::_probe_pose()`는 3초 이후 다음과 같이 동작한다.

```text
checkpoint progress
  -> pose_at(progress)
  -> checkpoint마다 pose 한 개 전송
```

따라서 `LongitudinalKinematicPlan.integrate()`의 accepted step은 실제 broadcast에서 이어지지 않는다.
payload의 `speed_mps`는 plan target speed지만, 좌표는 checkpoint progress이므로 같은 운동 상태를
나타내지도 않는다. 화면에서는 장시간 외삽 후 큰 위치 보정이나 순간 이동이 발생할 수 있다.

현재 API 테스트는 `physics_frame == sorted(set(physics_frames))`만 검사한다. 이 조건은
`0 -> 842`처럼 중복은 없지만 841개 tick이 누락된 경우도 통과시킨다.

## 3. 보존해야 하는 통과 항목

다음 구현과 결과는 변경하거나 다시 설계하지 않는다.

- `CompiledSplineArcLengthAdapter`의 동일 spline arc-length 계약
- `LongitudinalKinematicPlan.integrate()`의 원자적 distance/speed/acceleration
- Bahrain derived acceleration `-28.518051 ~ +11.981813m/s²`
- RBR derived acceleration `-30.298490 ~ +10.683153m/s²`
- 두 회로 integrator/reported acceleration identity violation `0/0`
- 두 회로 speed, tick displacement, chord, boundary, reversal, wrap guard
- Stage 1 10/57랩 canonical hash
- Stage 2 geometry/local metric 좌표 계약
- logical checkpoint/event/race_end의 권위와 순서
- FULL 기본 모드의 물리·타이어·회로 calibration

## 4. 대원칙

1. pose cursor와 checkpoint cursor는 서로 다른 책임을 가진다.
2. pose는 고정 `0.10s` logical tick에서 accepted kinematic step으로만 전진한다.
3. checkpoint는 순위, lap, logical event와 race result의 권위로만 사용한다.
4. checkpoint progress로 presentation pose를 덮어쓰거나 snap하지 않는다.
5. 배속은 wall-clock 소비 속도만 바꾸며 logical pose tick을 생략하지 않는다.
6. 한 차량과 최대 128개 rolling pose만 유지한다.
7. presentation pose는 canonical result hash에 포함하지 않는다.

## 5. 필수 수정 1 — 상태를 가진 단일 probe cursor

한 tick씩 전진할 수 있는 명시적인 cursor/presenter를 추가한다. 이름은 조정할 수 있지만 다음
상태와 기능은 필요하다.

```text
SingleProbeKinematicCursor
  driver_id
  logical_tick_index / logical_time_s
  accepted line_distance_m
  speed_mps
  last KinematicStep
  lateral trajectory / grid-launch state
  advance_one_tick() -> pose + accepted step
  current_pose() -> 전진 없이 현재 pose
  dispose()
```

필수 계약:

- 외부 pose tick은 정확히 `0.10s`다.
- 시작 pose는 Stage 2 shared grid slot을 사용한다.
- lights-out 이전에는 distance/speed/acceleration이 고정된다.
- lights-out 이후 매 tick `LongitudinalKinematicPlan.integrate()`를 정확히 한 번 호출한다.
- pose의 line distance, speed, acceleration은 같은 accepted `KinematicStep`에서 나온다.
- `current_pose()`와 client reconnect는 cursor를 전진시키지 않는다.
- cursor는 57랩 pose tuple을 보관하지 않는다.
- 진단 또는 테스트가 accepted step과 생성된 pose를 직접 비교할 수 있어야 한다.

기존 `SingleVehiclePoseSynthesizer.logical_sequence()`를 전부 미리 만드는 방식으로 실제 57랩
broadcast를 구현하지 않는다. sequence API는 단위 테스트와 짧은 진단용으로 유지할 수 있다.

## 6. 필수 수정 2 — 0.10초 logical broadcast scheduler

현재처럼 checkpoint 사이 전체 delta를 한 번 기다리지 않는다. game loop의 기본 scheduling 단위를
`BROADCAST_LOGICAL_TICK_SECONDS == 0.10`으로 바꾼다.

권장 흐름:

```text
send initial race_info/current pose/current state

while not finished:
  await replay_clock.wait(0.10 logical seconds)
  advance single probe exactly one accepted tick
  send one pose_tick
  drain every checkpoint whose tick_index <= current pose tick
    send authoritative race_state/checkpoint update in original order
    drain events up to that checkpoint time in original order
  when final checkpoint and final events are consumed:
    send race_end once
```

구현 세부사항:

- checkpoint가 없는 tick에도 pose를 전송한다.
- 한 pose tick 안에 여러 checkpoint가 있으면 원래 순서대로 모두 처리한다.
- checkpoint tick과 pose tick이 같은 경우에도 pose를 중복 생성하지 않는다.
- event batch `32개/16,384 bytes` 제한을 유지한다.
- `race_end`는 final checkpoint와 그 시점까지의 event 전송 후 정확히 한 번 보낸다.
- `race_state.positions`와 순위는 checkpoint 권위를 계속 사용한다.
- single probe의 `rank_authority="logical-checkpoint"`, `probe_only=true` 의미를 유지한다.
- probe pose로 logical checkpoint의 progress, position, gap 또는 finish order를 수정하지 않는다.

checkpoint 사이 state payload가 필요하지 않다면 pose만 0.10초마다 보내고 state/event는 checkpoint
도달 시에만 보내도 된다. 다만 기존 프런트엔드 계약에 필요한 heartbeat가 있으면 동일 logical tick의
값으로 bounded하게 전송한다.

## 7. 필수 수정 3 — pause/resume/배속 계약

`InterruptibleReplayClock`의 잔여 logical time 계약을 그대로 사용한다.

- pause 명령은 진행 중인 0.10초 wait를 즉시 중단해 남은 logical time을 보존한다.
- pause 동안 pose tick, checkpoint index, event index와 buffer count가 모두 증가하지 않는다.
- resume은 pause 직전 tick 다음 pose부터 재개한다.
- `set_speed(1|2|5)`는 현재 wait를 즉시 재계산하지만 pose frame을 건너뛰지 않는다.
- 1x/2x/5x에서 같은 seed와 logical 종료 tick의 실제 broadcast pose hash가 동일해야 한다.
- 명령 ack나 current-state 재전송은 pose cursor를 전진시키거나 buffer에 중복 append하지 않는다.
- 새 client 연결은 최신 pose를 읽기만 하며 새로운 tick을 만들지 않는다.

`asyncio.sleep(0.10)`을 별도 pause loop로 사용해 replay clock과 두 개의 시간 권위를 만들지 않는다.

## 8. 필수 수정 4 — 실제 post-correction 계측

`backend/tools/diagnose_abstract_stage3_kinematics.py`의 다음 상수 대입을 제거한다.

```python
post_integrator_distance_correction_count = 0
```

권장 계측:

1. cursor의 `advance_one_tick()`이 `KinematicStep`과 그 step으로 만든 pose를 함께 노출한다.
2. 매 tick 다음을 비교한다.

```text
pose.line_distance_m == accepted_step.distance_m
pose.speed_mps == accepted_step.speed_mps
pose.acceleration == accepted_step.acceleration
actual ds == accepted_step.displacement_m
```

3. accepted step 확정 후 pose distance가 달라진 tick을
   `post_integrator_distance_correction_count`로 집계한다.
4. 이 값이 `0`이 아니면 `approval_passed=false`와 non-zero exit를 반환한다.

`integrator_adjusted=true`는 accepted step 내부의 사전 guard이며 post-integrator correction과
구분한다. 필요하면 다음 필드를 별도로 기록한다.

- `accepted_step_adjustment_count`
- `accepted_step_adjustment_reasons`
- `accepted_step_pose_distance_mismatch_count`
- `accepted_step_pose_speed_mismatch_count`
- `accepted_step_pose_acceleration_mismatch_count`

상수를 다른 파일이나 항상 false인 필드로 옮기는 방식은 계측으로 인정하지 않는다.

## 9. 필수 회귀 테스트

### 9.1 cursor 단위 테스트

Bahrain과 RBR에서 최소 한 랩 이상 다음을 전 tick 검사한다.

- `physics_frame[n] == physics_frame[n-1] + 1`
- `simulation_time[n] == simulation_time[n-1] + 0.10`
- `ds == (v_prev + v_next) / 2 * 0.10`
- `acceleration == (v_next - v_prev) / 0.10`
- accepted step과 pose의 distance/speed/acceleration 일치
- distance reversal `0`
- speed `<=370km/h`, acceleration `<=+18m/s²`, braking `<=50m/s²`
- tick/world chord/boundary violation `0`
- grid hold 및 lights-out 첫 tick 연속성
- lap wrap에서 distance와 frame 연속성

### 9.2 실제 WebSocket broadcast 테스트

실제 `AbstractBroadcastSession._game_loop()` 경로를 fake/controlled clock으로 실행한다.

- 연속 pose의 frame delta가 전부 `1`
- 연속 pose의 logical-time delta가 전부 `0.10s`
- 동일 `physics_frame` pose 중복 `0`
- 종료 logical tick까지 pose 누락 `0`
- 첫 checkpoint가 84.2초여도 그 전 pose tick이 계속 생성됨
- checkpoint/event ID는 기존 순서와 정확히 동일하고 중복·누락 `0`
- `race_end`는 마지막 메시지이며 한 번만 전송
- 모든 pose payload의 `pose_count=1`, `len(poses)=1`
- buffer count `<=128`

### 9.3 pause/resume/speed/reconnect

- wait 도중 pause 후 0.20초 이상 관찰해 pose/checkpoint/event/buffer 변화 `0`
- resume 첫 frame이 pause 직전 frame `+1`
- 1x→5x 및 5x→2x 변경 전후 frame gap `1`
- 새 client 연결 전후 cursor와 buffer count 불변
- 1x/2x/5x actual broadcast pose hash 동일
- close 뒤 loop task, replay wait Event, client, pose buffer와 cursor 참조 정리

현재처럼 `sorted(set(physics_frames))`만 비교하지 말고 정확한 인접 delta를 검사한다.

## 10. 성능과 보관 상한

- pose buffer capacity: `128` 이하 유지
- logical result의 frame/state 권위 저장: `0/0` 유지
- 57랩 전체 pose tuple/list 생성 금지
- client가 없을 때 무한히 pose를 쌓지 않는다.
- 느린 client는 기존 timeout/dead-client 정리 계약을 유지한다.
- 57랩 controlled-clock broadcast 테스트에서 메모리가 logical tick 수에 비례해 증가하지 않아야 한다.

진단에 다음을 기록한다.

- total logical pose tick
- emitted pose count
- missing/duplicate pose count
- max frame gap/logical-time gap
- probe buffer peak/capacity
- retained pose count after close

## 11. 금지 사항

- checkpoint progress로 probe pose hard snap
- x/y 좌표 사후 clamp 또는 interpolation으로 결함 숨기기
- 누락 tick을 프런트엔드 extrapolation에 맡기기
- 20대 pose, 추월, 교통, 접촉 또는 순위 crossing 추가
- logical result hash에 pose hash 추가
- FULL `RaceEngine`, 차량 물리, 타이어, 트랙 calibration 변경
- 테스트 tolerance 확대
- 전체 57랩 pose 사전 생성·보관
- 사용자 요청 없는 commit, push, branch 변경 또는 package 생성

## 12. 승인 기준

다음을 모두 만족해야 단계 3을 승인한다.

### 실제 broadcast

- pose frame gap 전부 `1`
- pose logical-time gap 전부 `0.10s`
- pose missing/duplicate `0/0`
- post-integrator distance correction `0`
- accepted step/pose distance·speed·acceleration mismatch `0/0/0`
- 위치 차분 speed `<=370km/h`
- checkpoint/event/race_end 순서와 의미 불변
- pause/resume/speed/reconnect cursor jump `0`
- single probe 및 buffer `<=128`
- close cleanup 뒤 retained pose/client/task/Event `0`

### 비회귀

- Bahrain/RBR 기존 Stage 3 derived 운동학 승인 유지
- Stage 1 10/57랩 canonical hash 동일
- Stage 2 Bahrain/RBR geometry parity 유지
- 관련 FULL 회귀 통과
- frontend test/lint/build 통과
- backend compileall, 문서 링크와 `git diff --check` 통과

## 13. 실행할 검증

- 수정된 Stage 3 cursor/kinematics tests
- 실제 single-probe API/WebSocket tests
- 기존 Abstract/result/replay/API tests
- Stage 2 geometry tests
- 관련 FULL track physics/session/foundation/engine/grid/RaceInfo/pit 회귀
- 10/57랩 Stage 1 benchmark
- 57랩 controlled-clock bounded broadcast benchmark
- frontend test/lint/production build
- backend `compileall`
- 진단 JSON 재생성과 non-zero failure 경로 테스트
- 문서 상대 링크 검사
- `git diff --check`

전체 backend discovery 또는 FULL 366개 묶음을 실행하지 않으면 완료 보고에 정확히 미실행으로
기록한다.

## 14. 완료 보고 형식

```text
단계: 3 재보완 — continuous single-probe broadcast
판정: 완료 후보 / 조건부 / 실패
단계 4 시작 여부: 시작하지 않음

연속 probe:
- cursor/scheduler 구조
- emitted tick / missing / duplicate
- frame/time gap
- 위치 차분 max speed/chord

pause/resume/speed/reconnect:
- cursor 연속성
- 1x/2x/5x actual broadcast hash

accepted-step 진단:
- post-correction count
- pose distance/speed/acceleration mismatch

순서와 cleanup:
- checkpoint/event/race_end
- buffer peak/capacity
- close 후 retained 상태

비회귀:
- Stage 1 benchmark/hash
- Stage 2/Stage 3 운동학
- FULL/frontend 테스트

미실행/한계:
- 전체 backend discovery/FULL 366 여부
- Stage 4 이후 미구현 범위

변경 파일:
- 파일 목록
```

보고 후 단계 4를 시작하지 말고 독립 검증을 기다린다.

## 15. 다른 에이전트 전달용 프롬프트

```text
작업 저장소는 /Users/kimyongjin/Desktop/f1 입니다.
현재 브랜치는 codex/abstract-race-simulation 입니다.

먼저 다음 문서를 처음부터 끝까지 읽으세요.
1. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE3_BROADCAST_CORRECTION_DIRECTIVE.md
2. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE3_CORRECTION_DIRECTIVE.md
3. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE3_WORK_DIRECTIVE.md
4. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md
5. /Users/kimyongjin/Desktop/f1/docs/CURRENT_PROJECT_STATUS.md의 ABSTRACT 단계 1~3 최신 절

이번 작업은 단계 3 재보완만 수행합니다. 이미 통과한 same-spline arc-length와 atomic integrator는
보존하고, 실제 ABSTRACT broadcast의 연속 single-probe와 진단 계측만 수정하세요. 단계 4의 20대
교통·추월·순위 crossing, 단계 5 pit, 단계 6 incident/VSC/SC/restart는 시작하지 마세요.

현재 작업 트리는 단계 1~3 변경을 포함합니다. 모두 사용자 작업으로 간주하고 보존하세요. git
reset/checkout/clean/stash나 대량 되돌리기를 하지 마세요. 사용자 요청 없이 commit, push, branch
변경, package 생성도 하지 마세요.

독립 검증에서 Bahrain 5랩 seed 44의 실제 broadcast pose는 97개뿐이었고 첫 갱신이 84.2초 뒤,
최대 frame gap 842 tick, 0.10초 초과 interval 72/96, 최대 world jump 428.52m였습니다. 위치 차분
속도는 최대 116.469m/s(419.3km/h)였습니다. 원인은 broadcast._probe_pose()가 3초 이후 accepted
kinematic step을 이어가지 않고 checkpoint progress를 pose_at()으로 직접 sample하기 때문입니다.

반드시 다음 순서로 작업하세요.
1. 현재 실패를 실제 AbstractBroadcastSession/WebSocket 테스트로 고정하세요. physics_frame이
   sorted/unique인지만 보지 말고 인접 delta=1, logical-time delta=0.10, missing/duplicate=0을
   검증하세요.
2. 한 차량의 현재 logical tick, accepted line distance, speed, last KinematicStep과 grid/launch
   상태를 가진 bounded SingleProbeKinematicCursor 또는 동등 구조를 구현하세요.
3. game loop를 0.10초 logical scheduling으로 바꾸세요. 매 tick probe를 정확히 한 번 적분·전송하고,
   도달한 checkpoint와 event를 원래 순서로 drain하세요. checkpoint가 없는 tick도 pose를 보내세요.
4. checkpoint progress로 pose를 덮어쓰거나 snap하지 마세요. checkpoint는 순위/lap/event/result
   권위로만 유지하고 probe는 probe_only single-car preview로 유지하세요.
5. pause는 진행 중인 wait의 잔여 logical time을 보존하고 모든 cursor를 정지해야 합니다. resume은
   다음 frame부터, 1x/2x/5x 변경은 logical frame 누락 없이 wall-clock 속도만 바꿔야 합니다.
   reconnect와 current-state 전송도 pose cursor나 buffer를 전진시키면 안 됩니다.
6. diagnose_abstract_stage3_kinematics.py의 post_integrator_distance_correction_count=0 상수를
   제거하세요. accepted KinematicStep과 실제 pose의 distance/speed/acceleration 및 ds를 비교해
   count를 실제 집계하고 mismatch가 있으면 approval false/non-zero exit를 반환하세요.
7. 한 차량과 최대 128개 rolling pose만 유지하세요. 57랩 전체 pose tuple이나 20대 pose를
   생성하지 마세요. close 후 buffer/cursor/task/Event/client를 모두 정리하세요.
8. 실제 controlled-clock broadcast에서 frame/time gap, 누락/중복, 위치 차분 속도, checkpoint/event/
   race_end 순서, pause/resume/speed/reconnect, buffer 상한과 cleanup을 검증하세요.
9. Bahrain/RBR 기존 derived 운동학, Stage 1 10/57랩 hash, Stage 2 geometry, FULL 기본 모드와
   frontend 계약을 유지하세요.
10. 지시서 13절의 검증을 실행하고 14절 형식으로 보고한 뒤 멈추세요. 단계 4를 시작하지 마세요.

FULL 물리·타이어·회로 calibration과 logical result/hash를 수정하지 마세요. checkpoint snap, x/y
사후 clamp, 프런트엔드 extrapolation, tolerance 확대 또는 전체 pose 사전 생성으로 문제를 숨기지
마세요. 전체 backend discovery나 FULL 366개 묶음을 실행하지 않았다면 정확히 미실행으로
기록하고 독립 검증을 기다리세요.
```
