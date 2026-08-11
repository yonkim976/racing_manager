# 추상 레이스 시뮬레이션 재설계 작업 지시서

상태: **작업 지시 · 단계별 독립 검증 필수**
기준일: **2026-08-04**
대상 브랜치: `codex/abstract-race-simulation`
기준 커밋: `652ca70` (`docs: consolidate project guidance and add abstract design`)

## 1. 목적과 현재 판정

현재 `backend/simulation/abstract/` 구현은 불변 snapshot, 독립 RNG, 성능 축,
퀄리파잉 결정성 등 결과 엔진의 기초는 재사용할 가치가 있다. 그러나 다음 결함 때문에
`ABSTRACT_BROADCAST`를 제품 완료 상태로 승인하지 않는다.

- 렌더 좌표를 미터 좌표로 사용해 Bahrain 약 `2.309배`, Red Bull Ring 약 `1.744배`의
  화면 비율 오류가 생긴다.
- FULL의 compiled racing line, track-width profile, grid와 pit-exit geometry를 사용하지
  않고 별도 centerline spline과 빈 renderer profile을 전송한다.
- 구간별 일정 속도, progress 직접 보정과 즉시 lateral offset 변경으로 가감속·추월이
  순간이동처럼 보인다.
- pit 상태만 바뀌고 차량 pose는 계속 본 트랙 위에 있다.
- 전체 경기의 `0.10초 × 20대` frame을 먼저 보관하고 hash 때 다시 materialize하여
  장거리 경기의 시작 시간과 메모리가 크게 증가한다.
- 테스트가 결정성과 형태는 확인하지만 metric 좌표, 자연스러운 운동, pit route,
  renderer 품질과 57랩 자원 상한을 승인하지 않는다.

본 지시서의 목표는 기존 코드를 무조건 폐기하는 것이 아니다. **단계 A 결과 권위는
보존하고, 저장·표시·운동·교통·피트·사건·제품 연결을 승인 게이트 순서로 다시 만든다.**

## 2. 절대 원칙

1. `FULL`은 기본 모드이자 회귀·그래픽 비교 기준으로 계속 유지한다.
2. 추상 결과 엔진과 presentation은 분리한다. pose가 순위, 사건, pit 결과를 결정하면 안 된다.
3. `world_x_m`, `world_y_m`, 차량 크기, 폭과 거리 필드는 반드시 같은 local metric frame을
   사용한다. render coordinate를 `_m` 필드에 넣지 않는다.
4. racing line, width, grid, pit entry/lane/exit 등 검증된 트랙 콘텐츠는 FULL과 공통 compiled
   contract를 사용한다. 별도 스플라인으로 같은 정보를 재발명하지 않는다.
5. 논리 시간은 `0.10초` fixed tick을 유지하되 cooldown, pit phase, maneuver duration은
   tick 개수가 아니라 초 단위 의미로 정의한다.
6. 순위 변경은 실제 longitudinal crossing과 완료 조건 뒤에만 확정한다. 순위를 맞추기 위해
   차량 progress를 앞차 앞으로 직접 이동시키지 않는다.
7. 전체 레이스 pose frame을 영구 결과에 저장하지 않는다. 결과 권위는 snapshot, command,
   logical event, timing/checkpoint와 최종 분류다.
8. hash는 경기 결과 의미만 포함하고 presentation sampling 빈도에 영향받지 않아야 한다.
9. 같은 입력·seed·버전은 `INSTANT`와 `BROADCAST`, 1x/2x/5x에서 같은 논리 결과를 낸다.
10. 정밀 물리값이 없는 추상 모드는 힘, slip, 열수지 등을 가짜 값으로 채우지 않는다.
11. 한 단계의 승인 기준을 통과하기 전에 다음 단계 코드를 구현하지 않는다.
12. 사용자 요청 없이 commit, push, branch 변경, package 생성과 기존 산출물 삭제를 하지 않는다.

## 3. 보존·재작업·보류 범위

### 3.1 우선 보존

- `AbstractSessionSnapshot`의 immutable input 경계
- stable ID 정렬과 콘텐츠·ruleset·engine version identity
- `IndependentRNG`와 named independent stream
- explicit vehicle performance axes
- qualifying의 Q1/Q2/Q3 결과와 canonical 결정성
- `source_mode=abstract` 구분
- FULL과 별도 패키지라는 모듈 경계

보존은 무수정 고정을 뜻하지 않는다. 테스트로 계약을 먼저 고정한 뒤 필요한 최소 변경만 한다.

### 3.2 재작업

- `AbstractRaceResult` frame 저장 및 hash 계약
- `pose.py`의 좌표·라인·sampling 계약
- `race.py`의 progress, 속도 전환, 최소 간격과 순위 변경
- `racecraft.py`의 local corridor, cooldown과 maneuver timeline
- pit route와 pit box pose
- incident/reliability/SC presentation 및 논리 상태
- `broadcast.py`의 renderer geometry와 rolling replay
- API setup의 동기 CPU 작업과 중복 hash
- Stage B 이후 품질 테스트

### 3.3 최종 승인 전 보류

- ABSTRACT를 기본 모드로 변경
- FULL 물리 파일·테스트 삭제 또는 기준 완화
- 10x를 제품 속도로 노출
- Godot 또는 별도 renderer 이식
- 공식 F1 실측값이라고 주장하는 balance 보정

## 4. 공통 작업 규칙

각 단계의 작업 에이전트는 다음 순서로 진행한다.

1. 본 문서 전체와 `ABSTRACT_RACE_SIMULATION_DESIGN.md`, `SIMULATION_FOUNDATION.md`,
   `CURRENT_PROJECT_STATUS.md`의 관련 절을 읽는다.
2. `git status --short --branch`, `git diff --check`, 기준 commit과 현재 diff를 기록한다.
3. 현재 실패를 자동 테스트 또는 재현 가능한 benchmark로 먼저 고정한다.
4. 해당 단계 범위만 구현한다.
5. 신규 테스트, 관련 회귀, lint/build 또는 benchmark를 실행한다.
6. 문서는 실제 통과한 수치만 기록하고 미실행 항목은 `미실행`이라고 쓴다.
7. 아래 종료 보고 형식으로 보고한 뒤 멈춘다.

### 공통 금지사항

- 테스트 통과를 위해 기준값·tolerance를 이유 없이 완화하지 않는다.
- 기존 사용자 변경을 reset, checkout, clean 또는 덮어쓰지 않는다.
- `RaceEngine` private 메서드에 추상 모드가 의존하게 만들지 않는다.
- 두 엔진이 사용할 계약은 public/shared compiler 또는 immutable DTO로 추출한다.
- 테스트용 mock track만 통과시키고 Bahrain/RBR 실제 콘텐츠 검증을 생략하지 않는다.
- 전체 backend 회귀를 실행하지 않았는데 실행했다고 기록하지 않는다.
- 다음 단계의 기능을 편의상 함께 구현하지 않는다.

## 5. 단계 1 — 결과 저장·hash·setup 자원 구조 수정

### 목표

장거리 레이스 전체 pose를 선계산·영구 보관하지 않고도 INSTANT 결과와 BROADCAST가 같은
논리 결과를 사용하도록 결과 권위와 재생 구조를 정리한다.

### 필수 작업

- 현재 10랩/20랩 benchmark를 자동화해 실행 시간, frame 수, Python RSS와 hash 비용을 기록한다.
- `AbstractRaceResult.frames`를 경기 전체 권위 원본에서 제거하거나 bounded checkpoint/keyframe
  구조로 교체한다.
- 최종 결과 hash는 initial snapshot identity, command log, logical events, timing/checkpoint,
  finish classification을 canonical 순서로 incremental 계산한다.
- hash를 매 property 접근마다 다시 만들지 않고 한 번 계산·캐시한다.
- `_abstract_race_summary()` 중복 호출과 broadcast의 반복 hash 계산을 제거한다.
- FastAPI event loop에서 장거리 CPU 계산을 직접 수행하지 않는다. 계산 구조를 충분히 줄인 뒤에도
  CPU-bound라면 background task/worker 또는 `asyncio.to_thread` 경계를 명시한다.
- BROADCAST는 전체 tuple replay 대신 bounded rolling buffer 또는 logical state에서 필요한 pose를
  그때 합성할 수 있는 인터페이스만 마련한다. 실제 고품질 pose는 단계 2~3에서 구현한다.
- 현재 `ABSTRACT_BROADCAST`가 승인 전 실험 모드임을 UI/문서에서 명확히 표시한다.

### 금지 범위

- 트랙 스플라인·레이싱라인 수정
- 추월 확률·속도·피트·사고 balance 수정
- 그래픽 품질을 고쳤다고 선언
- FULL session의 실행·메모리 계약 변경

### 승인 기준

- 결과 저장량이 `logical ticks × vehicles` 전체 pose 객체 수에 비례하지 않는다.
- 같은 snapshot/seed로 변경 전 논리 grid, finish order와 logical event 의미가 의도치 않게 바뀌지 않는다.
- hash를 두 번 이상 조회해도 두 번째 조회가 전체 payload를 재생성하지 않는다.
- 10랩과 57랩 benchmark를 동일 명령으로 재현하고 결과를 보고한다.
- 권장 목표: Bahrain 57랩 INSTANT 계산 `10초 이하`, Python peak RSS 증가 `250MB 이하`.
  장비 차이로 목표를 넘으면 수치를 숨기지 말고 원인과 다음 조치를 제시하며 단계 완료로 선언하지 않는다.
- API heartbeat 또는 동시 read 요청이 계산 때문에 장시간 멈추지 않는다.
- 기존 Stage A 결정성 테스트와 관련 API 테스트가 통과한다.

### 검증 요청 시 제출할 자료

- benchmark 명령과 원문 수치
- 결과 객체의 retained collection 크기
- hash 첫/두 번째 접근 시간과 peak RSS
- 1/10/57랩 결과 hash 및 finish order 비교
- 변경 파일 목록과 미해결 위험

## 6. 단계 2 — FULL 공통 트랙·그래픽 좌표 계약

### 목표

ABSTRACT와 FULL이 같은 서킷 형상, local metric frame, racing line, track width, grid와 pit
geometry를 사용하게 만든다.

### 필수 작업

- 검증된 `TrackPhysicsProfile` 또는 그 public immutable display DTO를 공통 contract로 사용한다.
- `world_x_m/world_y_m`를 `coordinate_frame.to_local_m()` 기준으로 생성한다.
- abstract `race_info`에 FULL과 같은 의미의 다음 필드를 제공한다.
  - world origin과 meters-per-render-unit
  - racing-line coordinates/profile/length
  - track-width profile
  - driving-line coordinates/lengths
  - grid slots
  - pit lane과 pit-exit lane
- `SingleVehiclePoseSynthesizer` 기본 경로를 compiled racing line으로 변경한다.
- renderer fallback을 기대하며 필드를 빈 배열로 보내지 않는다.
- Bahrain과 RBR에서 차량 길이·폭·트랙 폭의 화면상 비율이 FULL과 일치하는지 자동 계산한다.

### 금지 범위

- 20대 추월·교통 로직 수정
- 피트 상태 머신 수정
- 사건·SC 구현
- FULL racing-line 보정값 자체 변경

### 승인 기준

- Bahrain/RBR의 metric track length 오차 `0.1% 이하`.
- ABSTRACT 기본 pose와 FULL compiled racing line 비교: 위치 `p95 ≤ 0.10m`, `max ≤ 0.30m`,
  heading `p95 ≤ 0.5°`. 같은 public sampler를 사용한다면 부동소수 오차 수준이어야 한다.
- car width/length, track width가 FULL과 같은 metric ratio를 사용한다.
- `race_info`의 공통 geometry 필드가 비어 있지 않고 schema/renderer 테스트가 통과한다.
- Bahrain/RBR start/finish와 좌우 상단 코너를 1대 차량으로 수동 확인하고 screenshot 또는 진단 수치를 남긴다.

## 7. 단계 3 — 단일 차량 종·횡 키네마틱 pose

### 목표

한 차량이 출발, 직선, 제동, 코너, 탈출과 라인 전환을 속도·가속도·헤딩이 연속적인 pose로
표현하도록 한다.

### 필수 작업

- segment time만으로 일정 속도를 만들지 말고 구간 경계의 target-speed knot와 transition curve를 만든다.
- longitudinal 속도·가속·제동에 명시적인 presentation limit를 둔다.
- 렌더 프레임마다 compiled spline을 다시 sample해 코너 chord 횡단을 방지한다.
- lateral offset은 duration, easing, 최대 횡속도·횡가속도를 가진 trajectory로 만든다.
- progress, line distance와 pose distance 사이의 변환을 한 contract에서 관리한다.
- 정지 grid pose, 두 줄 grid와 lights-out launch curve를 추가한다. 결과 순위에는 영향을 주지 않는다.
- 1x/2x/5x가 동일 logical pose sequence를 사용하고 wall-clock sampling만 달라지게 한다.

### 금지 범위

- 20대 간격·추월 결과 수정
- pit route·incident·SC 구현
- 차량 힘·타이어 slip을 가짜로 계산

### 승인 기준

- 정상 주행 최고 표시속도 `370km/h 이하`.
- presentation longitudinal acceleration `+18m/s² 이하`, braking magnitude `50m/s² 이하`.
- lateral speed `8m/s 이하`, lateral acceleration `20m/s² 이하`.
- 한 tick의 이동이 `v × dt + 0.5m`를 넘지 않는다.
- pose position과 heading이 lap wrap에서 연속이다.
- Bahrain/RBR 전 구간에서 track boundary 이탈과 corner chord 횡단이 없다.
- 1x/2x/5x logical pose hash가 동일하다.

수치는 강체 물리 재현값이 아니라 명백한 순간이동을 막는 presentation guard다. 더 엄격하거나
완화된 값이 필요하면 benchmark 근거와 함께 검증 요청에 포함한다.

## 8. 단계 4 — 20대 교통·추월·순위 동기화

### 목표

차량이 앞차에 접근해 자연스럽게 감속하고, local 공간이 있을 때 연속적인 공격·방어 궤적으로
추월하며, 실제 crossing 뒤에만 순위가 바뀌게 한다.

### 필수 작업

- `_enforce_minimum_intervals()`의 progress 직접 clamp를 car-following target speed/deceleration으로 교체한다.
- 성공 추월 시 attacker를 defender 앞으로 직접 이동시키는 코드를 제거한다.
- local track width, 현재 line, segment/corner phase와 주변 제3 차량을 corridor 승인에 사용한다.
- attack/defend/rejoin을 초 단위 duration과 continuous lateral trajectory로 표현한다.
- pair/driver cooldown을 초 단위로 정의해 `tick_seconds` 변경에 따라 사건 빈도가 달라지지 않게 한다.
- crossing, clearance, rejoin 완료를 구분하고 논리 순위·표시 순서를 동기화한다.
- 실패 공격은 실제 시간 손실과 자연스러운 재합류로 끝낸다.

### 금지 범위

- pit route 구현
- 접촉·스핀·SC 결과 확장
- 성능 축 전체 rebalance

### 승인 기준

- 추월 완료 전에는 순위가 바뀌지 않는다.
- 일반 traffic에서 차량 겹침·역주행·progress 순간 증가가 없다.
- 순위 보정을 위한 1 tick `1m 초과` 위치 강제 이동이 없다.
- 동일 pair 공격 cooldown은 기본 `8초 이상`이며 tick 크기와 무관하다.
- Bahrain seed 42 1랩의 기존 `158 attacks/58 passes/9 contacts`를 품질 기준으로 사용하지 않는다.
  10 seeds에서 driver당 평균 attack start가 랩당 `3회 이하`, 논리 contact가 랩당 평균 `2회 이하`인지
  보고한다. 실패 시 원인 분석 후 완료 선언을 보류한다.
- 10 seeds × Bahrain/RBR × 10랩에서 중복·누락 finish, overlap, corridor 충돌이 없다.
- 1x/2x/5x/Instant의 논리 결과 hash가 동일하다.

## 9. 단계 5 — 피트 진입·박스·출구와 전략 상태

### 목표

pit 논리 상태와 표시 위치를 일치시키고 Bahrain/RBR의 분리된 pit entry/lane/exit 경로를 사용한다.

### 필수 작업

- pit request가 entry commit point 이전에 확정되고 entry route로 연속 전환되게 한다.
- speed-limit line, pit lane, team box, service stop과 exit/rejoin route를 각각 표시한다.
- service 중 차량은 실제 box pose에 정지하고 서비스 종료 후에만 출발한다.
- pit lane 차량의 main-track progress, 순위와 timing crossing 의미를 명시한다.
- pit entry/lane/exit duration과 service time은 초 단위이며 tick 크기에 독립적이어야 한다.
- 피트아웃 차량은 분리된 exit lane을 따라가다 merge point에서만 본선에 합류한다.
- 교통 차량과 pit-in/out 차량의 corridor 충돌을 방지한다.

### 금지 범위

- SC queue/restart 구현
- 복합 사고·reliability retirement 구현
- 타이어 열 모델을 FULL 수준으로 재현

### 승인 기준

- `pit_state != none`인 차량 pose가 해당 pit route/box 허용 폭 안에 있다.
- service 동안 표시속도 `0.1m/s 이하`이고 지정 service duration을 유지한다.
- pit limiter 구간에서 속도 제한을 넘지 않는다.
- entry와 exit에서 위치·heading·속도 불연속이 없다.
- Bahrain/RBR에서 20대 1회 pit을 수행해 차량 중첩, 본선 정지, 잘못된 조기 merge가 없다.
- 0.10초와 0.50초 진단 tick에서 pit event logical time이 허용 오차 `0.10초` 안에 일치한다.

## 10. 단계 6 — 사건·리타이어·SC와 presentation template

### 목표

락업, run-wide, 경미한 접촉, 스핀, 정지, 신뢰성 고장, 리타이어와 SC를 논리 결과와 연속 pose가
일치하는 template로 구현한다.

### 필수 작업

- 각 사건은 원인, logical start/end, time loss, damage/reliability 결과, presentation template와
  race-control cue를 가진다.
- lockup/run-wide/spin/contact가 단순 speed multiplier와 상태 문자열로 끝나지 않게 한다.
- stopped/retired 차량의 위치, hazard lifetime, 제거/잔류 정책을 명시한다.
- reliability stream이 정상 lap-time bonus가 아니라 고장 사건만 결정하게 유지한다.
- local yellow/VSC/SC 판단과 race state를 broadcast에 반영한다.
- SC에서는 원칙적으로 track position order를 queue authority로 사용하며 허용된 pit/회수 예외만
  별도 event로 처리한다.
- catch-up, queue formed, restart 준비, SC in, control line release를 단계 상태로 분리한다.
- restart에서 차량 위치와 순위가 다르더라도 progress clamp로 멈추거나 순간 재배열하지 않는다.

### 승인 기준

- 사건 전후 pose·순위·시간 손실이 logical event와 일치한다.
- retired 차량이 최종 결과에서 `retired=false`로 남지 않는다.
- yellow/SC 중 금지된 정상 추월이 없다.
- SC queue가 deadlock 없이 형성되고 1x/2x/5x restart가 완료된다.
- Bahrain/RBR의 결정적 시나리오로 pit 중 SC, lapped car, 순위/track-position 불일치,
  restart 직전 사건을 각각 테스트한다.
- 사고 template가 없어도 임의 순간이동으로 대체하지 않고 안전한 fallback 상태를 사용한다.

## 11. 단계 7 — 제품 통합·장기 승인·문서화

### 목표

앞 단계가 모두 승인된 뒤 ABSTRACT_INSTANT/BROADCAST를 제품 선택지로 유지할지 결정하고 장기
자원·그래픽·회귀·패키징 기준을 확정한다.

### 필수 작업

- 단계 1~6의 미해결 항목과 수동 승인 기록을 재검토한다.
- 전체 backend regression, frontend tests/lint/build, `git diff --check`, 문서 링크를 검증한다.
- Bahrain/RBR 10랩을 1x와 5x에서 수동 관전하고 57랩 INSTANT/BROADCAST 자원을 계측한다.
- Electron/Python RSS, Electron CPU, renderer FPS, JS heap, pose buffer, geometry/texture 수명주기를 기록한다.
- New Race/Exit 후 60초 cleanup을 검증한다.
- `CURRENT_PROJECT_STATUS.md`와 설계 문서의 상태를 실제 결과에 맞게 갱신한다.
- 모든 승인을 통과한 뒤에만 새 패키지를 만든다.

### 최종 승인 기준

- FULL 기본 모드와 기존 승인 수치가 유지된다.
- ABSTRACT 57랩이 설정한 시간·메모리 상한 안에서 시작·완주·정리된다.
- Bahrain/RBR에서 track scale, racing line, pit route, start, 추월, 사건과 SC가 수동 승인된다.
- 1x/2x/5x/Instant의 논리 결과 동일성이 유지된다.
- cleanup 뒤 active session/client/task/socket/canvas/geometry/texture/pose buffer가 기준대로 0이 된다.
- 문서에 `완료`로 표시된 항목은 자동·수동 근거를 모두 가진다.

## 12. 단계별 종료 보고 형식

작업 에이전트는 매 단계 종료 시 아래 형식을 그대로 사용한다.

```text
단계: N — 이름
판정 요청: 완료 후보 / 조건부 / 실패

구현 요약:
- ...

변경 파일:
- path: 변경 이유

계약 변화:
- 입력/출력/schema/hash/시간/좌표 의미

검증 명령과 결과:
- command
  - tests, elapsed, peak RSS, 핵심 수치

승인 기준 대조:
- PASS ...
- FAIL ...
- NOT RUN ...

FULL 영향:
- 없음 또는 구체적인 공용 변경과 회귀 결과

남은 위험:
- ...

git 상태:
- branch
- status --short
- diff --check

다음 단계 작업 여부:
- 진행하지 않음
```

보고 후 코드를 더 수정하지 말고 검증자의 답을 기다린다. 검증자가 실패 또는 조건부로 판정하면
동일 단계 보완만 수행한다.

## 13. 검증자가 단계마다 확인할 공통 항목

- 지시 범위를 넘은 파일·기능이 없는가
- 사용자 변경을 훼손하지 않았는가
- 실패 재현 테스트가 수정 전 실패하고 수정 후 통과하는가
- 수치가 실제 product tick과 실제 Bahrain/RBR 콘텐츠에서 측정되었는가
- 테스트가 구현을 그대로 복제해 거짓 양성이 되지 않는가
- 결정성 hash가 presentation sample을 포함해 불필요하게 커지지 않는가
- FULL의 기존 권위·기본 모드·회귀가 유지되는가
- 문서의 완료 표현이 증거보다 앞서지 않는가

## 14. 다른 작업 에이전트에게 전달할 시작 프롬프트

아래 프롬프트는 **단계 1만 시작**하도록 작성되어 있다. 다음 단계에서는 `단계 1`을 승인받은
단계 번호와 절 제목으로 바꾸되 나머지 통제 문장은 유지한다.

```text
작업 저장소는 /Users/kimyongjin/Desktop/f1 입니다.

먼저 다음 문서를 처음부터 끝까지 읽으세요.
1. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md
2. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_DESIGN.md
3. /Users/kimyongjin/Desktop/f1/docs/SIMULATION_FOUNDATION.md
4. /Users/kimyongjin/Desktop/f1/docs/CURRENT_PROJECT_STATUS.md 중 ABSTRACT 관련 절

이번 작업은 재설계 지시서의 "단계 1 — 결과 저장·hash·setup 자원 구조 수정"만 수행합니다.
단계 2 이후의 좌표/그래픽, 차량 운동, 교통/추월, 피트, 사건/SC와 제품 패키징은 구현하지 마세요.

현재 작업 트리에는 다른 에이전트가 만든 미커밋 변경이 있습니다. 이를 사용자 작업으로 간주하고
보존하세요. git reset, checkout, clean, stash, 대량 되돌리기를 하지 마세요. 먼저 기준 commit
652ca70과 현재 diff, git status를 확인해 현재 구현을 파악하세요.

핵심 목표는 Stage A의 immutable snapshot, independent RNG, performance/qualifying 결정성을
보존하면서 전체 경기의 0.10초 × 20대 pose frame 보관, 반복 canonical payload/hash 생성,
동기 API setup 계산으로 생기는 시간·메모리 문제를 수정하는 것입니다.

반드시 다음 순서로 진행하세요.
1. 기존 10/20랩과 hash 비용 문제를 자동 benchmark/test로 재현합니다.
2. 단계 1의 저장·hash·rolling result 계약을 설계하고 코드에 구현합니다.
3. 같은 seed의 grid, finish order, logical event 의미와 INSTANT/BROADCAST 동일성을 검증합니다.
4. 10랩과 57랩의 elapsed, peak Python RSS, retained collection 크기, hash 첫/두 번째 접근 시간을
   실제로 측정합니다.
5. 관련 backend/API 테스트와 git diff --check를 실행합니다.
6. 지시서의 "단계별 종료 보고 형식"으로 보고하고 즉시 멈춥니다.

FULL 모드의 물리 결과나 기본값을 바꾸지 마세요. 테스트 기준을 임의로 완화하지 말고, 실행하지
않은 전체 회귀를 통과했다고 쓰지 마세요. 사용자 요청 없이 commit, push, branch 변경, package
생성을 하지 마세요. 목표 수치를 넘지 못하면 숨기지 말고 실패/조건부로 보고하세요.

단계 1 완료 후 단계 2를 시작하지 말고 검증을 기다리세요.
```
