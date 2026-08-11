# ABSTRACT 레이스 시뮬레이션 단계 4 작업 지시서

작성일: 2026-08-04
대상 브랜치: `codex/abstract-race-simulation`
상위 문서: [`ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md)
선행 승인: [`ABSTRACT_RACE_SIMULATION_STAGE3_EVENT_TIMELINE_CORRECTION_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE3_EVENT_TIMELINE_CORRECTION_DIRECTIVE.md)

## 1. 단계와 판정 전제

단계 1의 immutable result/hash, 단계 2의 공통 geometry/local metric 좌표, 단계 3의 same-spline
arc-length·atomic kinematics·0.10초 single-probe broadcast와 event timeline은 독립 검증을 통과했다.

이번 작업은 **단계 4 — 20대 교통, car-following, 공격·방어 corridor, 실제 crossing과 순위
동기화, full-field bounded broadcast**만 수행한다.

단계 5의 pit entry/lane/box/exit pose, 단계 6의 contact/spin/retirement/VSC/SC/restart와 단계 7의
제품 패키징은 시작하지 않는다. 현재 존재하는 논리 pit/lockup/contact 결과는 삭제하지 않되 이번
단계에서 확장하거나 balance하지 않는다. FULL 모드의 물리, 타이어, 회로 calibration과 승인값은
변경하지 않는다.

## 2. 현재 prototype의 확인된 구조적 결함

현재 `backend/simulation/abstract/race.py`의 Stage C prototype에는 다음 임시 구현이 있다.

1. `_enforce_minimum_intervals()`가 적분 후 뒤차의 `total_progress`를 앞차 뒤로 직접 clamp한다.
2. pending attack 동안에도 progress를 앞차 주변으로 직접 제한한다.
3. 성공 후보는 고정 `5 tick` 뒤 order를 swap하고 attacker/defender progress를 강제로 다시 쓴다.
4. 공격 시작 tick에 횡 offset을 attacker `+2.2m`, defender `-2.2m`로 즉시 변경한다.
5. 공격 종료 tick에 두 offset을 즉시 `0m`로 되돌린다.
6. cooldown이 `30 tick`, 즉 현재 0.10초에서 약 3초로 고정돼 있고 tick size에 종속된다.
7. corridor 평가는 local width가 아니라 회로 전역 `track_width_m`만 사용한다.
8. 제3 차량, 기존 corridor의 시간·공간 점유와 corner phase를 예약 충돌에 사용하지 않는다.
9. 결과 계산 중 `frame_sink`가 없으면 20대 frame은 버려지며, 실제 broadcast는 Stage 3 probe 한
   대만 재생한다.

이 prototype의 기존 Bahrain seed 42 1랩 `158 attacks / 58 passes / 9 contacts`는 품질 기준이
아니다. 기존 결과를 억지로 보존하기 위해 clamp·swap을 남기지 않는다.

## 3. 권위와 불변 대원칙

### 3.1 결과 권위

- `AbstractRaceResult`, logical event와 deterministic traffic stepper가 경기 결과 권위다.
- presentation pose는 결과, 순위, 사건 또는 난수 호출 순서를 바꾸지 않는다.
- `ABSTRACT_INSTANT`와 `ABSTRACT_BROADCAST`는 같은 snapshot, seed, command log에서 동일한
  logical result를 만들어야 한다.
- broadcast 재생은 결과 계산에 사용한 결정적 traffic 계약을 재사용하되 렌더 여부가 결과를
  변경하면 안 된다.

### 3.2 거리와 시간 권위

- 종방향 거리와 gap은 Stage 3의 `RacingLineDistanceContract` spline arc distance를 사용한다.
- 외부 logical tick은 `0.10s`를 유지한다.
- cooldown, maneuver duration, corridor reservation은 tick 수가 아니라 logical seconds로 정의한다.
- 차량의 accepted distance/speed/acceleration은 한 step에서 원자적으로 확정한다.
- 적분 후 progress만 줄이거나 순위를 맞추기 위해 위치를 다시 쓰지 않는다.

### 3.3 순위 권위

- 단순히 계획된 attack 결과가 success라는 이유로 순위를 swap하지 않는다.
- crossing과 안전 clearance가 실제 accepted longitudinal state에서 성립한 뒤에만
  `overtake_completed`와 order swap을 같은 tick에 원자적으로 확정한다.
- rejoin은 별도 완료 상태이며 이미 확정한 순위를 다시 바꾸지 않는다.
- 실패 공격은 뒤차 위치로 순간 복귀하지 않고 실제 감속과 횡 trajectory로 종료한다.

## 4. 단계 4 내부 게이트

단계 4는 아래 순서로 수행한다. 앞 게이트의 테스트가 통과하기 전 다음 게이트로 넘어가지 않는다.

```text
4A 현재 실패 고정 + shared traffic stepper
  -> 4B car-following + corridor/maneuver lifecycle
  -> 4C 20-car bounded broadcast 연결
  -> 4D multi-seed 품질·결정성·자원 승인
```

## 5. Gate 4A — 실패 진단과 shared traffic stepper

### 5.1 수정 전 실패를 먼저 고정

Bahrain(id 3), Red Bull Ring(id 4)에서 다음을 진단 JSON과 회귀 테스트로 기록한다.

- post-integrator progress correction count와 최대 correction distance
- order swap 당시 실제 attacker/defender longitudinal gap
- crossing 전 order swap count
- 한 tick lateral offset jump 최대값
- pair별 attack 재시도 최소 간격
- local width 부족인데 global width로 승인된 corridor count
- 제3 차량 또는 corridor reservation 충돌 count
- overlap/body-intersection count
- attack start/pass/defense/abort/contact 빈도

진단 필드는 저장된 maneuver 문자열만 믿지 않고 연속 frame/accepted state 차분으로 계산한다.

### 5.2 resumable shared stepper

현재 `AbstractRaceEngine.run()`의 거대한 loop를 테스트와 broadcast에서 한 tick씩 재사용할 수 있는
bounded stepper/cursor로 분리한다. 이름은 조정할 수 있지만 역할은 다음과 같다.

```text
AbstractTrafficSimulationCursor
  immutable snapshot/seed/rules
  current logical tick/time
  20 current vehicle states
  current order
  active maneuvers/corridor reservations
  cooldown eligibility times
  advance_one_tick() -> bounded AbstractRaceFrame + new logical events/checkpoints
  finalize() -> AbstractRaceResult
  dispose()
```

필수 계약:

- Instant result 계산과 broadcast replay가 같은 tick transition 함수를 사용한다.
- 한 번의 run이 전체 20대×57랩 frame tuple을 보관하지 않는다.
- result에는 immutable event/checkpoint/finish 의미만 남긴다.
- frame sink의 존재 여부가 RNG 소비, event, finish order 또는 canonical hash를 바꾸지 않는다.
- driver 처리 순서는 명시적으로 안정 정렬하고 set/dict 순회에 의존하지 않는다.
- current state와 bounded diagnostic window 외 과거 mutable state를 보관하지 않는다.

setup에서 result를 먼저 계산한 뒤 broadcast가 필요하면 동일 snapshot/seed/command log로 새 cursor를
재생할 수 있다. 이 경우 replay cursor가 만드는 event/checkpoint/result hash가 사전 계산 결과와
일치하는지 자동 검증한다. 불일치하면 화면을 계속 진행하지 말고 명시적인 오류로 종료한다.

## 6. Gate 4B — 자연스러운 car-following

### 6.1 accepted speed를 먼저 제한

앞차와의 간격을 적분 후 progress clamp로 맞추지 않는다. 각 tick 시작 시 다음 입력으로 뒤차의
허용 next speed/acceleration을 계산한다.

- 현재 spline arc distance와 speed
- 선행 차량의 accepted distance/speed
- 차량 길이, 최소 안전 gap과 closing speed
- 현재 segment/corner phase
- active two-wide corridor 여부
- 앞쪽 제3 차량과 queue 상태
- bounded normal/emergency deceleration

IDM, braking-distance safe-speed 또는 동등한 deterministic controller를 사용할 수 있다. 선택한
공식과 계수, 단위와 상한은 문서화한다.

필수 관계:

```text
accepted next speed >= 0
accepted distance >= previous distance
ds ~= (v_prev + v_next) / 2 * dt
acceleration ~= (v_next - v_prev) / dt
post-integrator distance correction == 0
```

정상 following에서는 앞차 속도를 복사하거나 뒤차 위치를 직접 수정하지 않는다. 안전 간격을
만족하기 어려우면 더 앞선 tick에서 braking anticipation을 시작한다. emergency guard가 필요하면
accepted speed/acceleration을 먼저 다시 계산하고 그 이유를 진단에 남긴다.

### 6.2 차량 형상과 overlap

최소한 다음 표시 형상을 사용한다.

- vehicle length/width
- longitudinal body clearance
- lateral separation
- local tangent/normal frame

같은 lane 차량은 longitudinal body clearance를 유지한다. two-wide 상태에서는 종방향 overlap이
가능하지만 lateral body clearance와 corridor boundary를 만족해야 한다. 단순 center-point gap만으로
overlap을 판정하지 않는다.

### 6.3 grid launch와 lap wrap

- Stage 2 shared grid slot의 실제 종·횡 위치로 20대를 배치한다.
- lights-out 이전 모든 차량은 정지한다.
- launch queue도 car-following controller로 처리한다.
- lap wrap 전후 arc distance, order, gap과 body clearance가 연속이어야 한다.

## 7. Gate 4B — corridor와 maneuver lifecycle

### 7.1 local corridor reservation

공격 승인에는 다음을 모두 사용한다.

- attacker/defender의 local left/right usable width
- racing-line offset, 차량 반폭과 edge margin
- segment type과 corner entry/apex/exit phase
- gap, closing speed, 남은 직선/제동 거리
- 현재 두 차량의 lane/offset
- 앞뒤 제3 차량의 body envelope와 future occupancy
- 같은 track interval/time window의 active corridor reservation
- 규칙상 공격 가능 여부와 cooldown

회로 전역 평균 width만으로 승인하지 않는다. corridor reservation은 최소 다음을 가진 immutable
값이어야 한다.

- stable corridor ID
- attacker/defender IDs
- reserved arc-distance interval
- logical start/expiry time
- inside/outside side assignment
- local lateral bounds
- required width와 실제 available width
- phase와 abort reason

### 7.2 maneuver state machine

고정 5 tick 뒤 결과를 확정하지 않는다. 최소 다음 상태를 명시적으로 둔다.

```text
approach
  -> pull_out
  -> overlap
  -> crossing_confirmed
  -> clearance_confirmed / overtake_completed + atomic rank swap
  -> rejoin
  -> rejoin_complete

실패 분기:
pull_out/overlap
  -> yield_or_abort
  -> fall_back_to_safe_gap
  -> rejoin
  -> rejoin_complete
```

각 상태는 start/end logical time, longitudinal target, lateral trajectory, corridor ownership과
completion condition을 가진다.

### 7.3 연속 횡 trajectory

- attacker/defender offset을 한 tick에 ±2.2m로 대입하지 않는다.
- Stage 3 `LateralTrajectory` 또는 같은 연속성 계약을 재사용한다.
- 횡속도 `<=8m/s`, 횡가속도 `<=20m/s²` guard를 유지한다.
- local boundary와 차량 간 lateral clearance를 매 tick 검증한다.
- duration이 부족하면 trajectory를 늘리거나 attack을 거절한다.
- abort/rejoin도 현재 offset에서 시작하는 새 continuous trajectory로 처리한다.

### 7.4 crossing, clearance, rank swap

다음을 별도 조건으로 계산한다.

- `crossing_confirmed`: attacker reference point가 defender보다 실제 앞섬
- `clearance_confirmed`: attacker rear/body와 defender front/body 사이에 안전 종방향 clearance 확보
- `rejoin_complete`: attacker가 허용 lane/racing line으로 횡 복귀하고 corridor 해제

`overtake_completed` event와 logical order swap은 `clearance_confirmed` tick에 한 번만 발생한다.
그 전에는 assessment가 success candidate여도 순위를 바꾸지 않는다. rejoin 중에는 새 공격을
예약하지 않는다.

### 7.5 cooldown

- 동일 pair cooldown 기본값은 logical `8.0s 이상`이다.
- attacker와 defender 각각의 짧은 driver cooldown도 logical seconds로 관리한다.
- `next_attack_eligible_time_s` 형태를 우선하고 `tick - last_tick` 비교를 제거한다.
- 0.05/0.10/0.50초 진단 tick에서 실제 최소 재시도 시간이 허용 오차 한 tick 안에 일치해야 한다.

### 7.6 실패 공격의 시간 손실

- 실패 attacker는 defender 앞으로 이동하지 않는다.
- 안전 trailing gap을 만들 때까지 bounded 감속한다.
- pull-out과 rejoin에 사용한 거리·시간으로 실제 progress/time loss가 발생한다.
- defender도 방어 line 사용에 따른 작은 설명 가능한 time cost를 가질 수 있다.
- 손실을 finish order에 맞추기 위한 임의 progress subtraction으로 만들지 않는다.

## 8. Gate 4C — 20대 bounded broadcast

### 8.1 full-field presentation contract

Stage 3 single-probe 계약을 단계 4 full-field 계약으로 명시적으로 전환한다.

- `pose_tick.pose_count == active vehicle count`이며 정상 시작은 20대다.
- 모든 pose는 같은 logical tick/frame을 사용한다.
- 순위·gap·event는 logical traffic state/checkpoint가 권위다.
- presentation pose는 logical result hash에 포함하지 않는다.
- UI/race_info에서 Stage 4 full-field kinematic preview임을 표시한다.

20개의 독립 `SingleProbeKinematicCursor`를 단순 복제하지 않는다. 공유 traffic state, following과
corridor reservation을 사용하는 하나의 full-field cursor가 pose를 생성해야 한다.

### 8.2 bounded storage

- 전체 57랩×20대 pose tuple/list를 생성하거나 result에 넣지 않는다.
- backend는 current 20 states와 작은 rolling frame window만 유지한다.
- rolling capacity를 frame 기준으로 명시하고 총 retained pose 상한을 진단한다.
- frontend의 driver별 typed-array ring buffer 상한과 기존 cleanup 계약을 유지한다.
- client가 없을 때 logical cursor가 진행하거나 pose를 축적하지 않는다.

### 8.3 배속·pause·reconnect

- 1x/2x/5x는 wall-clock scheduling만 변경한다.
- 각 모드의 logical frame/event/result hash는 동일하다.
- pause 동안 20대 pose, traffic state, corridor, checkpoint와 event cursor가 모두 정지한다.
- resume은 다음 logical tick부터 재개한다.
- reconnect는 current full-field snapshot만 읽으며 simulation cursor를 전진시키지 않는다.
- close/New Race 뒤 cursor, reservation, buffer, client, task와 Event를 정리한다.

## 9. Gate 4D — 진단과 승인 매트릭스

### 9.1 결정적 단위 시나리오

Bahrain/RBR에서 최소 다음 시나리오를 고정한다.

1. 직선에서 빠른 뒤차가 접근하지만 공격 공간이 없어 자연스럽게 following
2. 넓은 직선 corridor에서 성공 공격
3. heavy-braking corridor에서 crossing 후 clearance
4. 좁은 구간에서 corridor 거절
5. corner entry/apex에서 위험한 공격 거절
6. 제3 차량 때문에 corridor 예약 거절 또는 지연
7. 방어 성공 후 attacker fall-back/rejoin
8. pull-out 후 abort와 실제 time loss
9. lap wrap 부근 side-by-side
10. finish 직전 미완료 attack의 안전 종료

각 시나리오에서 frame-by-frame distance, speed, acceleration, offset, body clearance, maneuver phase,
corridor owner, logical order와 event를 검사한다.

### 9.2 multi-seed 품질 매트릭스

다음을 자동 실행한다.

```text
circuits: Bahrain + Red Bull Ring
seeds: 최소 10개 고정 seed
distance: 각 10랩
drivers: 20
modes: Instant + controlled Broadcast 1x/2x/5x
```

반드시 보고할 지표:

- driver당 lap당 attack start 평균/최대/P95
- race lap당 pass/defense/abort/contact 평균
- attack→terminal event 누락/중복
- corridor rejection reason 분포
- same-pair 최소 cooldown
- crossing 전 rank swap
- rank swap without clearance
- post-integrator correction count/max distance
- longitudinal/lateral body overlap count
- corridor conflict/third-car conflict count
- max longitudinal/lateral speed·acceleration
- boundary/lap-wrap/reverse movement violation
- finish 중복·누락과 position contiguous violation
- mode별 result/event/pose hash
- runtime, peak RSS와 retained frame/pose count

품질 guard:

- driver당 평균 attack start `<=3회/lap`
- logical contact `<=2회/race lap`을 목표 guard로 보고
- same pair cooldown `>=8.0s`
- crossing 전 rank swap `0`
- clearance 없는 completed pass `0`
- post-integrator progress correction `0`
- 일반 traffic body overlap `0`
- corridor/third-car reservation 충돌 `0`
- 중복·누락 finish `0/0`

빈도 guard를 넘으면 단순 확률 하향만 하지 말고 attack window, corridor availability, cooldown,
closing 조건과 segment eligibility 중 원인을 분해해 보고한다.

## 10. canonical hash와 버전 정책

Stage 4는 기존 hard clamp와 강제 swap을 제거하므로 기존 Stage 1 benchmark의 finish/event/hash가
의도적으로 바뀔 수 있다.

- old/new result를 같은 snapshot/seed에서 비교한다.
- finish order, event count/type과 hash 변경 원인을 문서화한다.
- 교통 의미가 바뀌면 `abstract_engine_version` 또는 ruleset version을 명시적으로 올린다.
- 단순 serialization 순서나 부동소수점 비결정성 때문에 hash가 바뀌면 승인하지 않는다.
- 새 버전 안에서는 Instant/1x/2x/5x/반복 100회 hash가 모두 동일해야 한다.
- 기존 hash를 유지하기 위해 progress/order를 사후 조작하지 않는다.

변경이 없는 Stage 1~3 구조 계약은 계속 유지한다.

- result canonical JSON round-trip
- frame/state 권위 저장 `0/0`
- result hash와 pose hash 분리
- event batch 상한과 race-start/race-finish timeline
- Stage 2 geometry와 Stage 3 arc-length/kinematic identity

## 11. 금지 범위

- 적분 뒤 progress/distance clamp
- success candidate만으로 order swap
- attacker를 defender 앞에 순간 배치
- 즉시 lateral offset 대입 또는 rejoin snap
- 회로 전역 width만으로 corridor 승인
- third-car/corridor reservation 충돌 무시
- tick count 기반 cooldown
- 20개의 독립 single-probe를 traffic 없이 배치
- 57랩 full pose 사전 생성·보관
- result에 presentation pose/hash 포함
- 접촉·사고·SC 결과 확장
- pit route/box/merge pose 구현
- FULL 파일·물리 계수·타이어·회로 calibration 변경
- 테스트 tolerance 확대나 확률만 낮춰 결함 숨기기
- 사용자 요청 없는 commit, push, branch 변경 또는 package 생성

## 12. 승인 기준

다음을 모두 만족해야 단계 4 완료 후보로 제출한다.

### 교통·추월

- car-following post-integrator correction `0`
- 정상 traffic overlap/reverse/teleport `0`
- attack phases와 lateral trajectory 연속
- local/third-car corridor conflict `0`
- crossing+clearance 전 rank swap `0`
- completed pass와 order swap 일대일 대응
- 실패 공격의 safe fall-back/rejoin과 time loss 확인
- pair cooldown `>=8s`, tick-size 독립

### full-field broadcast

- 20대 pose가 동일 frame/time으로 연속 전송
- missing/duplicate frame과 driver pose `0`
- 1x/2x/5x pose/event/result hash 동일
- bounded backend/frontend pose storage
- pause/resume/reconnect cursor jump `0`
- close cleanup 뒤 cursor/reservation/buffer/client/task/Event `0`

### 품질·비회귀

- 10 seeds×2 circuits×10랩 품질 guard 충족
- Instant/Broadcast 결과 동일
- 의도된 hash/version 변경 문서화
- Stage 1~3 비변경 계약과 관련 회귀 통과
- FULL 기본 모드와 관련 테스트 통과
- frontend test/lint/build 통과
- backend compileall, 문서 링크와 `git diff --check` 통과

## 13. 실행할 검증

- 새 traffic/following/corridor/maneuver cursor 단위 테스트
- Bahrain/RBR 결정적 10개 시나리오
- 기존 Abstract result/replay/kinematics/API 테스트
- 20대 actual WebSocket controlled broadcast 테스트
- 10 seeds×2 circuits×10랩 품질 매트릭스
- Instant/1x/2x/5x/반복 100회 결정성
- 10/57랩 benchmark와 bounded resource 측정
- Stage 2 geometry와 Stage 3 diagnostic
- 관련 FULL track physics/session/foundation/engine/grid/RaceInfo 회귀
- frontend test/lint/production build
- backend compileall
- 문서 상대 링크 검사
- `git diff --check`

전체 Backend discovery, FULL 366개 묶음 또는 수동 앱 관전을 실행하지 않으면 완료 보고에 정확히
미실행으로 기록한다.

## 14. 완료 보고 형식

```text
단계: 4 — 20대 교통·추월·순위 동기화
판정 요청: 완료 후보 / 조건부 / 실패
단계 5 시작 여부: 시작하지 않음

구조:
- shared traffic stepper/cursor
- result와 presentation 권위 분리
- bounded storage

car-following:
- controller 공식/상한
- post-integrator correction/overlap/reverse
- grid launch/lap wrap

maneuver/corridor:
- phase state machine
- local width/third-car reservation
- crossing/clearance/rank swap
- abort/rejoin/cooldown

full-field broadcast:
- pose count/frame/time
- 1x/2x/5x/pause/reconnect
- buffer/resource/cleanup

품질 매트릭스:
- 10 seeds×Bahrain/RBR×10랩
- attack/pass/defense/abort/contact
- overlap/corridor/rank/finish violations

결정성/버전:
- old/new hash와 변경 사유
- Instant/Broadcast/반복 hash

비회귀:
- Stage 1~3/FULL/frontend

미실행/한계:
- 전체 discovery/FULL 366/수동 관전
- Stage 5~7 미구현 범위

변경 파일:
- 파일 목록
```

보고 후 단계 5를 시작하지 말고 독립 검증을 기다린다.

## 15. 다른 에이전트 전달용 프롬프트

```text
작업 저장소는 /Users/kimyongjin/Desktop/f1 입니다.
현재 브랜치는 codex/abstract-race-simulation 입니다.

먼저 다음 문서를 처음부터 끝까지 읽으세요.
1. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE4_WORK_DIRECTIVE.md
2. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md
3. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_DESIGN.md
4. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE3_EVENT_TIMELINE_CORRECTION_DIRECTIVE.md
5. /Users/kimyongjin/Desktop/f1/docs/CURRENT_PROJECT_STATUS.md의 ABSTRACT 단계 1~3 최신 절

이번 작업은 단계 4 — 20대 교통·car-following·공격/방어 corridor·실제 crossing/clearance 기반
순위 동기화와 bounded full-field broadcast만 수행합니다. 단계 5 pit route/box/merge, 단계 6
contact/spin/retirement/VSC/SC/restart, 단계 7 제품 패키징은 시작하지 마세요. 기존 논리 pit/incident
결과는 보존하되 확장하거나 balance하지 마세요. FULL 물리·타이어·회로 calibration을 수정하지 마세요.

현재 작업 트리에는 단계 1~3의 미커밋 변경이 있습니다. 전부 사용자 작업으로 간주하고 보존하세요.
git reset/checkout/clean/stash나 대량 되돌리기를 하지 마세요. 사용자 요청 없이 commit, push, branch
변경 또는 package 생성도 하지 마세요.

현재 prototype의 핵심 결함은 다음과 같습니다. _enforce_minimum_intervals()가 적분 뒤 progress를
직접 clamp하고, 성공 attack은 5 tick 뒤 order swap과 progress 강제 재배치를 수행합니다. 횡 offset은
시작/종료에 ±2.2m/0m로 즉시 바뀌고, cooldown은 30 tick(약 3초), corridor는 global track width만
보며 제3 차량을 고려하지 않습니다. 실제 broadcast는 아직 Stage 3 single probe 한 대뿐입니다.

지시서의 Gate 4A→4B→4C→4D 순서를 지키세요.

1. 수정 전 post-integrator correction, crossing 전 swap, lateral jump, cooldown, local-width 오류,
   overlap/corridor conflict와 attack/pass/contact 빈도를 테스트와 진단으로 먼저 고정하세요.
2. AbstractRaceEngine loop를 Instant와 Broadcast가 함께 쓰는 resumable bounded traffic cursor/stepper로
   분리하세요. frame sink나 렌더링 여부가 RNG, event, finish order, result hash를 바꾸면 안 됩니다.
3. gap을 적분 뒤 clamp하지 말고 tick 시작 시 앞차 distance/speed, body gap, closing speed와 braking
   distance로 accepted next speed/acceleration을 계산하는 deterministic car-following controller를
   구현하세요. post-integrator distance correction은 0이어야 합니다.
4. Stage 3 spline arc distance와 atomic ds/speed/acceleration identity를 20대 모두에 사용하세요.
   body clearance, grid launch와 lap wrap을 frame-by-frame 검증하세요.
5. corridor는 local left/right width, racing-line offset, 차량 크기, segment/corner phase, 남은 거리,
   제3 차량과 기존 time/space reservation을 사용해 승인하세요. global 평균 width만 사용하지 마세요.
6. approach→pull_out→overlap→crossing_confirmed→clearance_confirmed/overtake_completed+atomic rank
   swap→rejoin→rejoin_complete 상태기를 구현하세요. success candidate나 고정 5 tick만으로 순위를
   바꾸지 마세요.
7. lateral offset은 Stage 3 LateralTrajectory 계약으로 연속화하고 횡속도 8m/s, 횡가속도 20m/s²,
   track boundary와 body clearance를 지키세요. abort도 현재 offset에서 safe fall-back/rejoin하며 실제
   time loss를 가져야 합니다.
8. pair/driver cooldown을 logical seconds로 바꾸고 same-pair 기본 8초 이상을 0.05/0.10/0.50초
   tick에서 검증하세요.
9. 공유 traffic cursor를 실제 WebSocket full-field pose_count 20에 연결하세요. 20개 독립 single
   probe를 복제하지 말고, 전체 57랩 pose를 저장하지 않으며 current state와 bounded rolling window만
   유지하세요.
10. pause/resume/reconnect/1x/2x/5x에서 20대 logical frame, event, result와 pose hash가 동일하고
    cursor jump가 없으며 close 뒤 모든 cursor/reservation/buffer/client/task/Event가 정리되게 하세요.
11. Bahrain/RBR 결정적 시나리오와 10 seeds×2 circuits×10랩 매트릭스를 실행하세요. driver당 lap당
    attack start 평균 <=3, logical contact <=2/race lap, cooldown >=8s, crossing 전 rank swap 0,
    clearance 없는 pass 0, post-integrator correction 0, body overlap/corridor conflict/finish 누락 0을
    수치로 보고하세요.
12. hard clamp 제거로 기존 result hash가 의도적으로 바뀌면 old/new finish/event/hash를 비교하고
    engine/ruleset version을 명시적으로 올리세요. 기존 hash를 보존하려고 progress/order를 조작하지
    마세요. 새 버전에서는 Instant/1x/2x/5x/반복 100회가 동일해야 합니다.
13. 지시서 13절의 회귀를 실행하고 14절 형식으로 보고한 뒤 멈추세요. 단계 5는 시작하지 마세요.

적분 뒤 progress clamp, attacker 순간 배치, 즉시 lateral snap, tick 기반 cooldown, global width만의
corridor 승인, 20개의 독립 probe, 전체 pose 사전 저장, tolerance 확대 또는 확률만 낮추는 방식으로
문제를 숨기지 마세요. 전체 Backend discovery/FULL 366/수동 앱 관전을 실행하지 않았다면 정확히
미실행으로 기록하고 독립 검증을 기다리세요.
```
