# ABSTRACT 단계 3 최종 보완 지시서 — race-start event timeline

작성일: 2026-08-04
대상 브랜치: `codex/abstract-race-simulation`
선행 문서: [`ABSTRACT_RACE_SIMULATION_STAGE3_BROADCAST_CORRECTION_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE3_BROADCAST_CORRECTION_DIRECTIVE.md)
상위 문서: [`ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md)

## 1. 판정과 범위

단계 3의 same-spline arc-length, atomic accepted step, 실제 0.10초 single-probe broadcast,
pause/resume/배속, bounded buffer와 cleanup은 독립 검증을 통과했다. 남은 결함은
`race_started(logical_time=0.0s)` 이벤트가 첫 다음 checkpoint인 84.2초까지 지연되는 한 가지다.

이번 작업은 **race-start event timeline만 수정하는 단계 3 최종 보완**이다.

- 연속 pose cursor와 운동학을 다시 설계하지 않는다.
- checkpoint/event/result의 내용과 canonical hash를 변경하지 않는다.
- 단계 4의 20대 교통·추월·순위 crossing을 시작하지 않는다.
- 단계 5 pit, 단계 6 incident/VSC/SC/restart를 시작하지 않는다.
- FULL 물리·타이어·회로 calibration을 변경하지 않는다.

## 2. 독립 검증 실패

Bahrain 5랩, seed `44`, controlled clock의 실제 WebSocket 메시지를 시간순으로 검사했다.

```text
race_started.payload.logical_time_s = 0.0
race_started가 실제 전송될 때 마지막 pose time = 84.2
첫 다음 checkpoint tick = 842
```

연속 pose 자체는 frame `0~4420`, time `0.0~442.0s`, frame gap `1`, time gap `0.10s`,
missing/duplicate `0/0`으로 정상이다.

## 3. 원인

`AbstractBroadcastSession._game_loop()`는 매 pose tick 후 `_drain_checkpoints_until()`을 호출한다.
이 함수는 **다음 checkpoint로 advance할 때만** `_send_events_until()`을 호출한다.

현재 checkpoint인 `race:start(tick=0)`은 이미 replay cursor가 가리키고 있으므로 advance 대상이
아니다. 따라서 logical time 0의 `race_started` event는 첫 다음 checkpoint가 도달한 84.2초에
`_send_events_until(84.2)`이 실행될 때까지 남는다.

## 4. 보완 대원칙

1. 현재 `race_start` checkpoint의 event를 첫 logical wait 전에 정확히 한 번 drain한다.
2. event cursor가 전역 권위이며 client reconnect가 event를 다시 소비하지 않는다.
3. 초기 snapshot 전송과 global event 소비를 혼합하지 않는다.
4. pose frame 0을 중복 생성하거나 cursor를 전진시키지 않는다.
5. 기존 event ID 순서, batch 상한과 final race-end 순서를 유지한다.

## 5. 필수 구현

### 5.1 game loop의 초기 event drain

첫 client가 준비되어 activity gate를 통과한 후, 첫 `replay_clock.wait(0.10)` 전에 현재 checkpoint
시점의 event를 broadcast한다.

권장 순서:

```text
add_client:
  race_info
  current pose frame 0
  current race_state at race_start
  register client and wake game loop

game_loop first activation:
  drain events through current race_start logical time (0.0s)
  wait 0.10 logical seconds
  advance/send pose frame 1
  continue normal checkpoint/event drain
```

필요하면 다음과 같은 명시적 상태를 둘 수 있다.

```text
_initial_checkpoint_events_drained: bool
```

단, `LogicalReplayCursor.event_index`가 이미 소비 여부를 보장하므로 이중 권위를 만들지 않는다.
flag를 사용한다면 event cursor와 모순되지 않아야 하고 진단용 상태로만 사용한다.

### 5.2 reconnect와 다중 client

- `add_client()` 안에서 global `_send_events_until()`을 직접 호출하지 않는다.
  - 아직 client 등록 전이라 기존/신규 client 사이의 전송 범위가 달라질 수 있다.
  - 초기 snapshot 실패가 global event cursor를 소비해서는 안 된다.
- reconnect는 최신 race_info/pose/state만 받고 이미 소비된 `race_started`를 다시 전송하지 않는다.
- 첫 client가 event drain 전에 끊기면 event를 소비하지 않고 다음 정상 client까지 보존한다.
- `race_started` event는 session 전체에서 정확히 한 번 broadcast한다.

### 5.3 기존 순서 보존

- `race_started`는 pose frame 0과 race-start state가 준비된 뒤 전송한다.
- `race_started`는 pose frame 1 전에 전송한다.
- 이후 logical events는 기존 checkpoint 기반 순서로 유지한다.
- `race_finished`는 final pose/checkpoint state 뒤에 전송한다.
- `race_end`는 모든 final event 뒤 마지막 메시지로 정확히 한 번 전송한다.
- event batch 상한 `32개/16,384 bytes`를 유지한다.

## 6. 필수 테스트

### 6.1 초기 timeline

실제 `AbstractBroadcastSession._game_loop()`와 WebSocket double을 controlled clock으로 실행한다.

반드시 다음을 검사한다.

```text
race_started count == 1
race_started.payload.logical_time_s == 0.0
race_started 전 마지막 pose physics_frame == 0
race_started 전 마지막 pose simulation_time_s == 0.0
race_started message index < pose frame 1 message index
```

event ID 배열이 기대 배열과 같은지만 검사해서는 안 된다. 각 event batch가 전송된 시점의 마지막
pose frame/time도 함께 기록한다.

### 6.2 reconnect와 초기 실패

- 정상 첫 client 이후 reconnect해도 `race_started` global count가 증가하지 않는다.
- 초기 snapshot 전송에 실패한 client는 global event cursor를 소비하지 않는다.
- 실패 client 뒤 정상 client가 연결되면 `race_started`가 logical 0에서 한 번 전송된다.
- reconnect/current-state 전송 전후 pose cursor와 buffer count가 변하지 않는다.

### 6.3 종료 timeline

```text
race_finished count == 1
race_finished.payload.logical_time_s == final logical time
race_finished 전 마지막 pose frame == final tick
race_end count == 1
race_end is final message
```

### 6.4 기존 연속성 비회귀

- pose frame gap 전부 `1`
- pose logical-time gap 전부 `0.10s`
- missing/duplicate/interval violation `0/0/0`
- 1x/2x/5x actual broadcast pose hash 동일
- checkpoint/event ID 순서와 중복·누락 `0`
- probe buffer `<=128`
- close 후 cursor/buffer/task/Event/client `0`

## 7. 금지 사항

- `race_started`의 payload time을 84.2초로 덮어써 현상을 숨기기
- event를 pose frame 1 이후에 두고 테스트 tolerance만 완화하기
- `add_client()`의 초기 snapshot 도중 global event cursor 소비
- reconnect마다 `race_started` 재전송
- checkpoint/event 배열을 재정렬하거나 result hash 변경
- 연속 probe cursor 또는 accepted integrator 재설계
- 단계 4 이후 기능 구현
- 사용자 요청 없는 commit, push, branch 변경 또는 package 생성

## 8. 승인 기준

다음을 모두 만족하면 단계 3 승인 후보로 제출한다.

- `race_started` logical 0초, 실제 pose frame 0 시점 전송, 정확히 1회
- pose frame 1보다 먼저 전송
- 실패 client/reconnect에 의한 누락·중복 `0`
- `race_finished` final logical time, final pose 뒤 정확히 1회
- `race_end` 모든 event 뒤 마지막에 정확히 1회
- 전체 event ID 순서·누락·중복 불변
- 기존 연속 pose 및 운동학 승인 수치 불변
- Stage 1 10/57랩 canonical hash 불변
- Stage 2 geometry와 관련 FULL 회귀 통과
- frontend test/lint/build, backend compileall, 문서 링크와 `git diff --check` 통과

## 9. 실행할 검증

- 수정된 API/WebSocket event timeline tests
- Stage 3 cursor/kinematics tests
- 기존 Abstract result/replay/API tests
- Stage 2 geometry tests
- 관련 FULL track physics/session/foundation 회귀
- 10/57랩 Stage 1 benchmark
- frontend test/lint/production build
- backend compileall
- 문서 상대 링크 검사
- `git diff --check`

전체 Backend discovery와 기존 FULL 366개 묶음을 실행하지 않았다면 정확히 미실행으로 기록한다.

## 10. 완료 보고 형식

```text
단계: 3 최종 보완 — race-start event timeline
판정: 완료 후보 / 조건부 / 실패
단계 4 시작 여부: 시작하지 않음

초기 timeline:
- race_started payload time
- 전송 당시 pose frame/time
- frame 1과의 메시지 순서
- event count

reconnect/failure:
- global event cursor 보존
- race_started 누락/중복
- pose cursor/buffer 변화

종료 timeline:
- race_finished frame/time/count
- race_end 순서/count

비회귀:
- pose frame/time gap, missing/duplicate
- event ID 순서
- pose/result hash
- buffer/cleanup
- Stage 1~3/FULL/frontend 검증

미실행:
- 전체 Backend discovery/FULL 366 여부

변경 파일:
- 파일 목록
```

보고 후 단계 4를 시작하지 말고 독립 검증을 기다린다.

## 11. 다른 에이전트 전달용 프롬프트

```text
작업 저장소는 /Users/kimyongjin/Desktop/f1 입니다.
현재 브랜치는 codex/abstract-race-simulation 입니다.

먼저 다음 문서를 처음부터 끝까지 읽으세요.
1. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE3_EVENT_TIMELINE_CORRECTION_DIRECTIVE.md
2. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE3_BROADCAST_CORRECTION_DIRECTIVE.md
3. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE3_CORRECTION_DIRECTIVE.md
4. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md
5. /Users/kimyongjin/Desktop/f1/docs/CURRENT_PROJECT_STATUS.md의 ABSTRACT 단계 3 최신 절

이번 작업은 단계 3의 마지막 race-start event timeline 보완만 수행합니다. 연속 0.10초
single-probe, same-spline arc-length, atomic integrator, bounded buffer와 cleanup은 독립 검증을
통과했으므로 다시 설계하지 마세요. 단계 4의 20대 교통·추월·순위 crossing, 단계 5 pit,
단계 6 incident/VSC/SC/restart는 시작하지 마세요.

현재 작업 트리의 단계 1~3 변경은 모두 사용자 작업입니다. git reset/checkout/clean/stash나 대량
되돌리기를 하지 마세요. 사용자 요청 없이 commit, push, branch 변경, package 생성도 하지 마세요.

독립 검증에서 race_started의 payload logical time은 0.0초였지만 실제 전송 당시 마지막 pose는
84.2초였습니다. 원인은 game loop가 다음 checkpoint를 advance할 때만 event를 drain하여 현재
race_start checkpoint의 event를 첫 logical wait 전에 소비하지 않기 때문입니다.

반드시 다음 순서로 작업하세요.
1. 실제 AbstractBroadcastSession/WebSocket controlled-clock 테스트로 실패를 먼저 고정하세요.
   race_started가 전송될 때 마지막 pose frame/time이 0/0.0이고 pose frame 1보다 앞서는지
   검증하세요. event ID 배열만 비교해서는 안 됩니다.
2. 첫 정상 client가 activity gate를 통과한 뒤 첫 replay_clock.wait(0.10) 전에 현재 race_start
   checkpoint 시점의 event를 정확히 한 번 drain하세요.
3. add_client 초기 snapshot 안에서 global event cursor를 소비하지 마세요. snapshot 실패 client가
   event를 잃게 하거나 reconnect가 race_started를 다시 전송해서는 안 됩니다.
4. race_started는 frame 0/state 이후 frame 1 전에 한 번, race_finished는 final pose/checkpoint 뒤
   한 번, race_end는 모든 event 뒤 마지막에 한 번 전송되게 하세요.
5. 기존 checkpoint 기반 event ID 순서, batch 32개/16,384 bytes, logical result/hash를 유지하세요.
6. pose frame/time gap 1/0.10초, missing/duplicate 0/0, 1x/2x/5x pose hash, buffer 128와 close
   cleanup이 그대로 유지되는지 재검증하세요.
7. Stage 1 10/57랩 hash, Stage 2 geometry, Stage 3 운동학, 관련 FULL과 frontend 회귀를 실행하세요.
8. 지시서 10절 형식으로 보고하고 멈추세요. 단계 4는 시작하지 마세요.

payload time 덮어쓰기, event tolerance 완화, reconnect 재전송 또는 add_client 중 global cursor 소비로
문제를 숨기지 마세요. 전체 Backend discovery나 FULL 366개 묶음을 실행하지 않았다면 정확히
미실행으로 기록하고 독립 검증을 기다리세요.
```
