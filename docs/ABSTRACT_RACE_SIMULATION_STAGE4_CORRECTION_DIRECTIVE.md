# ABSTRACT 레이스 시뮬레이션 단계 4 최종 보완 작업 지시서

작성일: 2026-08-05
대상 브랜치: `codex/abstract-race-simulation`
선행 문서: [`ABSTRACT_RACE_SIMULATION_STAGE4_WORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_STAGE4_WORK_DIRECTIVE.md)
현재 판정: **조건부 완료 — 본 지시서 통과 전 단계 4 최종 승인 금지**

## 1. 목적과 범위

단계 4의 shared 20-car traffic cursor, anticipatory car-following, crossing/clearance 기반 순위
교환과 bounded full-field broadcast는 실제 실행 및 주요 회귀를 통과했다. 그러나 독립 검증에서
다음 계약이 충분히 구현되거나 증명되지 않았다.

1. 실제 active corridor 교차를 독립 계측하지 않고 `corridor_conflict_count=0` 초기값을 승인에 사용한다.
2. corridor reservation에 arc interval, lateral bounds, time window와 판단 근거가 보존되지 않는다.
3. maneuver가 `approach` 없이 `pull_out`부터 시작하며 driver cooldown이 없다.
4. 지시서의 결정적 10개 상황과 10 seeds×2 circuits×Instant/1x/2x/5x 매트릭스가 완성되지 않았다.
5. `AbstractRaceEngine.run()`의 return 아래에 구형 clamp/swap loop 전체가 도달 불가능 코드로 남아 있다.

이번 작업은 위 항목만 보완한다. 단계 5의 pit route/box/merge, 단계 6의 contact/spin/retirement/
VSC/SC/restart, 단계 7의 제품 패키징은 시작하지 않는다. 기존 FULL 물리, 타이어, 회로 calibration과
승인값을 변경하지 않는다.

## 2. 변경 금지 불변조건

- `AbstractRaceResult`와 logical events/checkpoints가 경기 결과 권위다.
- presentation pose와 진단 수치는 canonical result hash에 포함하지 않는다.
- Stage 3 same-spline arc distance와 atomic distance/speed/acceleration 관계를 유지한다.
- 적분 뒤 distance/progress를 줄이거나 order에 맞춰 차량 위치를 다시 쓰지 않는다.
- crossing과 full insertion clearance 전에는 `overtake_completed` 또는 rank swap을 발생시키지 않는다.
- Instant와 Broadcast는 동일한 traffic transition을 사용한다.
- 전체 레이스 pose 배열을 사전 생성하거나 result에 저장하지 않는다.
- 난수 stream 이름, 호출 순서 또는 기존 pit/incident 의미를 이유 없이 변경하지 않는다.
- 기존 사용자 변경을 보존하며 reset/checkout/clean/stash를 하지 않는다.
- 사용자 요청 없이 commit, push, branch 변경 또는 package 생성을 하지 않는다.

## 3. 작업 순서와 게이트

```text
4R-A 실제 reservation·conflict 계측
  -> 4R-B maneuver lifecycle·cooldown 계약 완성
  -> 4R-C 결정적 10개 상황 회귀
  -> 4R-D multi-mode/multi-seed·자원 승인
  -> 4R-E 구형 코드 제거·전체 비회귀·문서화
```

앞 게이트가 테스트를 통과하기 전에 다음 게이트의 balance 조정을 하지 않는다.

## 4. Gate 4R-A — 명시적 corridor reservation과 실제 충돌 계측

### 4.1 reservation 데이터 계약

현재 `_TrafficManeuver`에 암묵적으로 섞인 corridor 정보를 명시적인 reservation 값 객체로
분리한다. 이름은 조정할 수 있지만 최소 다음 값을 가진다.

```text
TrafficCorridorReservation (immutable)
  corridor_id
  attacker_id / defender_id
  start_arc_distance_m / end_arc_distance_m
  start_time_s / expiry_time_s
  lateral_min_m / lateral_max_m
  side_assignment
  required_width_m / available_width_m
  segment_type / corner_phase
  third_vehicle_ids
  admission_reason
```

- arc interval은 lap wrap을 안전하게 처리하는 unwrapped distance 또는 명시적 wrap helper를 사용한다.
- time window는 maneuver의 예상 pull-out/overlap/clearance/rejoin 점유 시간을 포함한다.
- local lateral bounds는 예약 구간의 여러 geometry sample에서 모두 유효해야 한다.
- reservation은 생성 후 변경하지 않는다. phase, 완료 여부와 abort reason은 별도의 mutable maneuver
  runtime state가 소유한다.
- 진단과 logical event payload가 reservation의 핵심 판단 근거를 재구성할 수 있어야 한다.

### 4.2 충돌 판정

새 공격 승인 전에 모든 active reservation과 다음 교차를 검사한다.

```text
time windows overlap
AND arc intervals overlap (lap wrap 포함)
AND lateral envelopes/body envelopes conflict
```

제3 차량은 현재 center point만 보지 말고 bounded horizon 동안 accepted/anticipated longitudinal
occupancy와 body envelope를 사용한다. 과도한 예측 물리나 Stage 6 collision engine은 만들지 않는다.

### 4.3 실제 독립 계측

`corridor_conflict_count`는 초기값을 그대로 반환하는 카운터가 될 수 없다. 진단 도구에서 각 accepted
frame의 active reservation과 vehicle body envelope를 독립적으로 검사한다.

반드시 분리할 지표:

- admitted reservation끼리의 time/arc/lateral conflict count
- third vehicle가 admitted corridor를 실제 침범한 count
- reservation admission rejection count와 reason 분포
- longitudinal overlap이 허용된 two-wide 상태의 lateral body clearance violation count
- same-lane longitudinal body overlap count
- 최소 boundary/body/corridor clearance

승인 기준은 실제 충돌 `0`이다. 안전하게 거절된 후보 횟수는 실패가 아니며 별도 지표로 남긴다.

## 5. Gate 4R-B — maneuver lifecycle과 logical cooldown 완성

### 5.1 상태기

다음 상태를 코드와 event/diagnostic에서 실제로 구분한다.

```text
approach
  -> pull_out
  -> overlap
  -> crossing_confirmed
  -> clearance_confirmed + overtake_completed + atomic rank swap
  -> rejoin
  -> rejoin_complete

실패:
approach/pull_out/overlap
  -> yield_or_abort
  -> fall_back_to_safe_gap
  -> rejoin
  -> rejoin_complete
```

- `approach`는 공격 후보가 생긴 즉시 횡 이동을 시작하는 상태가 아니다. closing/gap, corridor 예약,
  next accepted speed와 pull-out 시작 조건을 확인하는 짧고 bounded한 준비 상태다.
- `clearance_confirmed`는 boolean만 세팅하지 말고 상태 전환과 event/metric으로 관찰 가능해야 한다.
- 성공/실패 모두 terminal event가 정확히 하나 있어야 한다.
- finish 직전 미완료 maneuver는 안전하게 종결하고 finish/order를 사후 수정하지 않는다.
- rejoin 완료 시 offset을 단순 대입하기 전에 trajectory의 종료 offset/velocity/acceleration 연속성을
  검증한다. 부동소수점 정규화는 trajectory 완료 뒤 허용하되 teleport로 계측되면 안 된다.

### 5.2 cooldown

- 동일 pair cooldown은 logical `>=8.0s`를 유지한다.
- attacker와 defender 각각에 짧은 driver-level `next_attack_eligible_time_s`를 둔다.
- tick count 비교를 사용하지 않는다.
- 0.05/0.10/0.50초 tick에서 허용 오차 한 tick 내에 같은 logical 의미를 가져야 한다.
- frame rendering, speed multiplier와 client 연결 여부가 eligibility나 RNG 순서를 바꾸면 안 된다.

### 5.3 실패 공격의 손실

- 실패 attacker가 defender 앞 순위로 이동하지 않는다.
- fall-back은 accepted speed/acceleration과 실제 이동거리로 trailing gap을 회복한다.
- 공격 시작 전후 동일 조건의 비공격 기준과 비교해 time/progress loss가 실제로 관찰돼야 한다.
- 임의 progress subtraction이나 finish order 보정으로 손실을 만들지 않는다.

## 6. Gate 4R-C — 결정적 10개 상황 회귀

Bahrain과 Red Bull Ring의 실제 snapshot/geometry를 사용하고, 필요하면 테스트 전용 driver pace 또는
초기 traffic fixture를 주입하되 production 확률을 억지로 반복 호출해 원하는 결과를 기다리지 않는다.

최소 다음 10개 테스트를 각각 독립된 이름과 frame-by-frame assertion으로 구현한다.

1. 빠른 뒤차가 corridor 부재로 자연스럽게 following
2. 넓은 직선에서 approach부터 성공 추월
3. heavy-braking 구간 crossing 후 clearance와 atomic swap
4. local width 부족으로 corridor 거절
5. corner entry/apex 위험 공격 거절
6. 제3 차량 anticipated occupancy 때문에 예약 거절/지연
7. active reservation time/arc/lateral 교차 때문에 두 번째 예약 거절
8. 방어 성공 후 fall-back, rejoin과 실제 time loss
9. lap wrap 인근 side-by-side에서 연속 distance/order/body clearance
10. finish 직전 미완료 공격의 안전 종료와 unique contiguous finish

각 테스트는 가능한 범위에서 다음을 검사한다.

- accepted distance/speed/acceleration identity
- maneuver phase 순서와 logical time 단조 증가
- lateral offset/velocity/acceleration 연속성과 상한
- body, boundary, corridor clearance
- order swap tick과 completed event 일치
- terminal event 누락/중복 0
- reservation 생성/해제와 cleanup

기존 8개 Stage 4 테스트를 삭제해 숫자만 맞추지 말고 부족한 상황을 보강한다.

## 7. Gate 4R-D — multi-seed, multi-mode와 자원 승인

### 7.1 필수 매트릭스

```text
circuits: Bahrain(id 3), Red Bull Ring(id 4)
seeds: 0..9
laps: 10
drivers: 20
modes: Instant, controlled Broadcast 1x, 2x, 5x
```

80개의 full-field replay를 모두 메모리에 동시에 보관하지 않는다. 각 run의 compact summary/hash만
남기고 run 종료 때 cursor/session/buffer를 정리한다. 실행 시간이 과도하면 CI용 fast layer와 수동
approval layer를 분리할 수 있으나 권위 diagnostic artifact는 위 전체 매트릭스를 실행해야 한다.

### 7.2 mode parity

동일 circuit/seed에서 다음 값이 Instant/1x/2x/5x 모두 동일해야 한다.

- grid/finish order
- canonical result hash
- ordered logical event ID/type/payload hash
- ordered checkpoint hash
- full-field pose hash
- logical tick/frame count

방송은 pose frame `0..final`, logical gap `0.10s`, 매 frame driver ID 20개 unique/complete를 검사한다.
missing/duplicate frame, driver pose, event와 race_end는 모두 `0/0` 또는 정확히 1회여야 한다.

### 7.3 품질과 자원 지표

기존 지표에 다음을 추가한다.

- attack마다 approach→terminal phase 완결성
- clearance event와 completed pass/order swap 일대일 대응
- reservation conflict를 독립 계산한 실제 수치
- mode별 runtime, peak RSS 증가와 종료 후 retained bytes
- frame buffer peak/capacity와 retained frame/pose count
- close 후 cursor, maneuver, reservation, client, task, Event, buffer count

기존 guard를 유지한다.

- driver당 평균 attack start `<=3/lap`
- logical contact `<=2/race lap`
- same pair cooldown `>=8.0s`
- crossing/clearance 전 rank swap `0`
- clearance 없는 completed pass `0`
- post-integrator correction, reverse, teleport, overlap, boundary/corridor conflict `0`
- finish 누락·중복 및 position 비연속 `0`

단일 driver/lap 최대 attack 관찰값 4는 별도 관찰치로 계속 보고한다. 이를 숨기기 위해 tolerance를
높이거나 확률만 낮추지 않는다.

## 8. Gate 4R-E — 구형 구현 제거와 정리

### 8.1 도달 불가능 코드 제거

`AbstractRaceEngine.run()`이 새 cursor 결과를 반환한 뒤 남아 있는 구형 loop를 제거한다. 함께
사용되지 않는 `_PendingOvertake`, legacy-only helper/import/constant가 있는지 `rg`로 확인한다.

- audit 목적의 구형 소스는 Git history가 보존하므로 production 파일에 남기지 않는다.
- 공용 helper 중 새 cursor가 사용하는 부분은 제거하지 않는다.
- 제거 전후 Stage 4 결과 hash가 같아야 한다. 달라지면 dead code가 아니므로 중단하고 원인을 보고한다.

### 8.2 명칭과 문서 정확성

- Stage 4 full-field 테스트/diagnostic의 `single_probe`, `probe_*` 잔여 명칭을 호환 API를 제외하고
  full-field/traffic 명칭으로 정리한다.
- `CURRENT_PROJECT_STATUS.md`의 approach, reservation, mode matrix 표현은 실제 구현과 검증 수치에
  맞춰 갱신한다.
- 기존 `ABSTRACT_STAGE4_TRAFFIC_DIAGNOSTIC.json`은 새 schema/version으로 교체하고 실행 명령,
  contract version과 생성일을 기록한다.
- 교통 의미 또는 event/hash 의미가 바뀌면 `abstract_engine_version`을 올리고 old/new hash 차이를
  문서화한다. 계측·테스트만 추가되고 결과가 같으면 불필요하게 version을 올리지 않는다.

## 9. 필수 비회귀

최소 다음을 실행한다.

1. Stage 4 결정적 상황 테스트와 전체 multi-mode diagnostic
2. 기존 Abstract result/replay/Stage 2 geometry/Stage 3 kinematics/Stage 4/API 테스트
3. Stage 3 diagnostic Bahrain/RBR와 저장 artifact 비교
4. Stage 1 10랩/57랩 benchmark와 canonical hash/retention 비교
5. 57랩 controlled full-field WebSocket replay, buffer/cleanup
6. 관련 FULL track physics/session/tire foundation/engine/grid/RaceInfo/pit 회귀
7. frontend test, lint, production build
8. backend compileall
9. 문서 상대 링크 검사와 `git diff --check`

전체 Backend discovery와 FULL 전체 묶음 또는 수동 앱 관전을 실행하지 않았다면 완료 보고에 정확히
미실행으로 적는다. 기존 성공 보고를 이번 실행 결과인 것처럼 복사하지 않는다.

## 10. 승인 판정

다음이 모두 충족돼야 단계 4를 **최종 승인 후보**로 제출한다.

- 명시적 immutable reservation과 실제 time/arc/lateral 교차 판정
- 실제 frame/state 기반 corridor/third-car/body conflict `0`
- approach와 clearance가 관찰 가능한 완전한 maneuver lifecycle
- pair `>=8s` 및 driver logical cooldown, tick-size 독립
- 결정적 10개 상황 전부 통과
- 10 seeds×2 circuits×Instant/1x/2x/5x 전체 parity 및 품질 guard 통과
- 10/57랩 bounded storage와 close cleanup 통과
- 구형 clamp/swap dead code와 legacy-only 정의 제거
- Stage 1~3, 관련 FULL, frontend와 문서 비회귀 통과
- 단계 5~7 미착수

## 11. 완료 보고 형식

```text
단계: 4 최종 보완
판정 요청: 최종 승인 후보 / 조건부 / 실패
단계 5 시작 여부: 시작하지 않음

reservation/conflict:
- 데이터 계약과 교차 공식
- 실제/거절 conflict 수치

maneuver/cooldown:
- 상태 전이와 terminal completeness
- pair/driver cooldown 및 tick-size 결과
- abort time-loss 결과

결정적 시나리오:
- 10개별 PASS/FAIL 및 핵심 수치

전체 매트릭스:
- 10 seeds×2 circuits×4 modes
- result/event/checkpoint/pose hash parity
- attack/pass/defense/abort/contact/conflict/finish 수치

자원/cleanup:
- runtime, peak RSS, retained frame/pose
- close 후 객체 수

dead-code/버전:
- 제거 목록
- old/new result hash와 version 판단

비회귀:
- Abstract/Stage 1~4/API/FULL/frontend/문서

미실행/한계:
- 전체 discovery/수동 관전 등

변경 파일:
- 파일 목록
```

보고 뒤 단계 5를 시작하지 말고 독립 검증을 기다린다.

## 12. 다른 에이전트 전달용 프롬프트

```text
작업 저장소는 /Users/kimyongjin/Desktop/f1 입니다.
현재 브랜치는 codex/abstract-race-simulation 입니다.

먼저 다음 문서를 처음부터 끝까지 읽으세요.
1. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE4_CORRECTION_DIRECTIVE.md
2. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE4_WORK_DIRECTIVE.md
3. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md
4. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_DESIGN.md
5. /Users/kimyongjin/Desktop/f1/docs/CURRENT_PROJECT_STATUS.md의 ABSTRACT 단계 1~4 최신 절

이번 작업은 단계 4 최종 보완만 수행합니다. 단계 5 pit route/box/merge, 단계 6
contact/spin/retirement/VSC/SC/restart, 단계 7 제품 패키징은 시작하지 마세요. FULL 물리, 타이어,
회로 calibration과 승인값을 변경하지 마세요.

현재 작업 트리에는 단계 1~4의 미커밋 변경이 있습니다. 모두 사용자 작업으로 간주하고 보존하세요.
git reset/checkout/clean/stash나 대량 되돌리기를 하지 마세요. 사용자 요청 없이 commit, push, branch
변경 또는 package 생성도 하지 마세요.

지시서의 Gate 4R-A→4R-B→4R-C→4R-D→4R-E 순서를 지키세요.

핵심 작업은 다음과 같습니다.
1. immutable TrafficCorridorReservation을 만들고 arc interval, logical time window, lateral bounds,
   required/available width, side, segment/phase와 제3 차량 정보를 보존하세요.
2. active reservation의 time+arc+body/lateral envelope 교차와 제3 차량 anticipated occupancy를 실제로
   판정하세요. corridor_conflict_count를 초기값 0으로 통과시키지 말고 연속 accepted frame/state에서
   독립 계측하세요.
3. maneuver를 approach→pull_out→overlap→crossing_confirmed→clearance_confirmed+completed/atomic
   swap→rejoin→rejoin_complete로 구현하세요. 실패는 yield_or_abort→fall_back_to_safe_gap→rejoin으로
   처리하고 terminal event 누락·중복을 0으로 만드세요.
4. pair cooldown >=8초와 attacker/defender driver-level next_attack_eligible_time_s를 logical seconds로
   구현하고 0.05/0.10/0.50초 tick에서 검증하세요.
5. Bahrain/RBR 실제 geometry를 사용하는 결정적 10개 상황 테스트를 지시서 6절 그대로 구현하세요.
   production 확률을 반복해 원하는 결과를 기다리지 말고 결정적 fixture/injection boundary를 사용하세요.
6. Bahrain/RBR, seeds 0..9, 10랩, Instant+controlled Broadcast 1x/2x/5x 전체 매트릭스를 실행하세요.
   result/event/checkpoint/pose hash parity, frame/driver/event 누락·중복, 품질 guard, runtime, peak RSS,
   retained pose와 cleanup을 compact artifact로 기록하세요.
7. AbstractRaceEngine.run() return 아래의 구형 clamp/swap loop와 legacy-only 정의를 제거하세요. 제거
   전후 hash가 달라지면 중단하고 실제 사용 경로를 찾으세요.
8. 지시서 9절의 비회귀를 실행하고 CURRENT_PROJECT_STATUS.md, docs/README.md와 새 Stage 4 diagnostic을
   실제 결과에 맞춰 갱신하세요.

적분 후 progress clamp, 고정 tick rank swap, 즉시 lateral snap, tick 기반 cooldown, global width만의
corridor, 상수 0 진단, 20개 독립 probe, full-race pose 저장, tolerance 확대나 확률만 낮추는 우회는
금지합니다. 테스트나 계측 때문에 logical result가 바뀌면 안 됩니다. 교통/event 의미를 의도적으로
바꾼 경우에만 engine version을 올리고 old/new hash 차이를 설명하세요.

완료 보고는 지시서 11절 형식을 사용하세요. 전체 Backend discovery, FULL 전체 묶음 또는 수동 앱
관전을 실행하지 않았다면 정확히 미실행으로 기록하세요. 완료 후 단계 5를 시작하지 말고 독립 검증을
기다리세요.
```
