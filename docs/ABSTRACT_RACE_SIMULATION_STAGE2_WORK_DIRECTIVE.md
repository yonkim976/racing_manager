# ABSTRACT 레이스 시뮬레이션 단계 2 작업 지시서

작성일: 2026-08-04
대상 브랜치: `codex/abstract-race-simulation`
상위 문서: [`ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md`](ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md)

## 1. 단계와 판정 전제

단계 1의 결과 저장, canonical hash, bounded logical replay, interruptible replay clock은
검증을 통과했다. 이번 작업은 **단계 2 — FULL 공통 트랙·그래픽 좌표 계약**만 수행한다.

단계 3의 종·횡 속도 모델, 출발 가속, 제동, 코너 속도 전이, 20대 교통, 추월, 피트 상태
머신, 사건과 Safety Car는 구현하지 않는다. 단계 2의 단일 pose는 오직 동일 progress를 같은
공통 racing-line sampler에 넣었을 때 FULL과 같은 위치·heading이 나오는지를 검증하기 위한
geometry 표본이다.

## 2. 현재 문제

현재 FULL은 `build_track_physics_profile(circuit)`으로 만든 `TrackPhysicsProfile`과
`LocalMetricCoordinateFrame`을 사용한다. 반면 ABSTRACT의 `SingleVehiclePoseSynthesizer`는
`AbstractTrackSnapshot.centerline_points`로 별도의 Catmull–Rom `TrackSpline`을 만들고 있다.

이 구조에는 다음 문제가 있다.

- ABSTRACT 기본 pose가 compiled racing line이 아니라 centerline을 따른다.
- centerline render 좌표를 metre 좌표처럼 사용할 수 있어 track scale이 달라진다.
- `race_info`의 world origin, metre scale, racing line, driving lines, track width, grid와 pit-exit
  geometry가 비어 있거나 FULL과 다른 생성 경로를 사용한다.
- renderer fallback에 의해 화면은 나오더라도 FULL과 같은 형상·차량/트랙 비율이라는 보장이 없다.

## 3. 목표 구조

한 번 compiled된 circuit에서 다음 presentation geometry를 만드는 **public immutable contract**를
둔다.

```text
compiled Circuit
  -> public TrackPhysicsProfile / TrackDisplayGeometry builder
     -> FULL RaceInfo
     -> ABSTRACT Broadcast RaceInfo
     -> SingleVehiclePoseSynthesizer geometry sampler
     -> Bahrain/RBR parity diagnostics
```

권장 계약 이름은 `TrackDisplayGeometry`, `CompiledTrackGeometry` 등으로 정할 수 있다. 이름보다
다음 원칙이 중요하다.

1. FULL `RaceEngine` 인스턴스나 private 필드에 ABSTRACT가 의존하지 않는다.
2. 동일 circuit 입력에 대해 FULL과 ABSTRACT가 같은 public builder 결과를 사용한다.
3. contract는 생성 이후 수정되지 않는 DTO여야 한다.
4. 결과 hash의 논리 권위와 presentation geometry를 혼합하지 않는다.
5. 좌표 공간을 필드명과 타입으로 구분한다.
   - `*_render`: circuit/render 좌표
   - `*_m`: `coordinate_frame.to_local_m()`로 변환된 local metric 좌표
6. geometry 생성 때문에 매 WebSocket 연결마다 track profile을 다시 계산하지 않는다.

## 4. 필수 구현 작업

### 4.1 기준선과 실패 테스트 작성

수정 전에 Bahrain(id `3`)과 Red Bull Ring(id `4`) 실제 콘텐츠로 다음 값을 기록한다.

- 공식 `track_length_m`
- compiled centerline의 metric length
- racing-line length
- local coordinate frame origin과 `meters_per_render_unit`
- track/racing/driving/pit/pit-exit point count
- track-width min/max
- 차량 폭·길이 대 평균 track width 비율
- 동일 progress 표본에서 기존 ABSTRACT pose와 FULL public sampler의 position/heading 오차

다음 실패를 먼저 테스트로 고정한다.

- ABSTRACT `race_info` 공통 geometry 필드가 비어 있음
- ABSTRACT 기본 pose가 compiled racing line과 불일치함
- render 좌표가 local metre 좌표로 오인될 가능성

### 4.2 공통 불변 display geometry 계약 추출

`TrackPhysicsProfile`을 그대로 전달해도 되지만, API/renderer가 필요한 값만 가진 immutable DTO를
추출하는 방식을 권장한다. 최소 필드는 다음과 같다.

- `world_origin_x_render`, `world_origin_y_render`
- `world_meters_per_render_unit`
- `track_length_m`, `track_width_m`
- `car_width_m`, `car_length_m`, `wheelbase_m`
- `track_coords`
- `racing_line_coords`, `racing_line_profile`, `racing_line_length_m`
- `track_width_profile`
- `driving_line_coords`, `driving_line_lengths_m`, `predicted_line_lap_times`
- `grid_slots`
- `pit_lane_coords`, `pit_exit_lane_coords`, `pit_wall_coords`
- `start_finish_index`

다음 조건을 지킨다.

- 차량 치수는 `simulation.vehicle_dimensions`의 FULL 상수를 사용한다.
- racing/driving line은 `TrackPhysicsProfile`의 public 값과 sampler를 사용한다.
- track width는 profile의 좌우 폭을 그대로 전달한다.
- grid slot 계산이 현재 FULL 엔진 상태에 묶여 있다면 순수 public helper로 추출하고 FULL과
  ABSTRACT가 모두 그 helper를 사용하게 한다. 기존 FULL slot 수치가 바뀌면 안 된다.
- pit/pit-exit 표시 geometry가 private mixin 상태에 묶여 있다면 public helper/DTO로 추출한다.
  피트 상태 머신이나 피트 경로 보정값은 변경하지 않는다.
- cache가 필요하면 circuit geometry/version/관련 calibration을 포함한 안전한 key를 사용한다.

### 4.3 ABSTRACT setup과 broadcast 연결

- abstract setup 시 compiled circuit과 공통 display geometry를 한 번 생성한다.
- `AbstractBroadcastSession.race_info`가 FULL과 동일 의미의 geometry 필드를 실제 값으로 보낸다.
- 빈 배열이나 `{}`를 renderer fallback 용도로 보내지 않는다.
- `RaceInfoMessage` 또는 별도의 shared serializer를 사용해 FULL/ABSTRACT schema drift를 막는다.
- `ABSTRACT_INSTANT`의 논리 결과와 canonical hash에는 presentation geometry를 불필요하게 넣지 않는다.
- 단계 1의 57랩 frame/state `0/0`, hash cache와 event-loop `to_thread` 경계를 유지한다.

### 4.4 SingleVehiclePoseSynthesizer 기본 geometry 교체

- 기본 경로를 `AbstractTrackSnapshot.centerline_points -> TrackSpline`에서 compiled
  `TrackPhysicsProfile.line_pose_at_progress_m(DRIVING_LINE_RACING, progress)`로 변경한다.
- 위치는 반드시 local metric frame의 `world_x_m/world_y_m`여야 한다.
- heading은 동일 public sampler가 반환한 tangent heading을 사용한다.
- 단계 2의 parity 테스트를 위한 `pose_at(progress)`만 다룬다.
- 독립 Catmull–Rom `TrackSpline`은 기본 경로에서 제거한다. 호환 경로가 정말 필요하면 명시적
  opt-in으로 격리하고 제품 ABSTRACT 경로에서는 사용하지 않는다.
- lateral offset을 적용할 경우 같은 heading의 normal을 사용한다. lateral transition·횡속도·
  횡가속도는 단계 3에서 구현한다.

### 4.5 renderer 좌표와 비율 검증

- frontend가 `world_origin_*_render`와 `world_meters_per_render_unit`로 local metric pose를 render
  좌표에 올바르게 변환하는지 계약 테스트를 추가한다.
- track, racing line, pit lane을 render 좌표로 전달하고 차량 pose는 local metre로 전달하는 현재
  계약을 명시한다.
- car width/length와 local track width가 모두 metre 단위로 같은 ratio를 사용하는지 확인한다.
- FULL/ABSTRACT가 동일 `race_info` geometry를 받았을 때 canvas bounds, display rotation, vehicle
  scale 계산이 달라지지 않아야 한다.

## 5. 수정 대상 예상 파일

구현에 따라 이름은 달라질 수 있으나 우선 다음을 검토한다.

- `backend/simulation/track_physics.py`
- 새 public display DTO/helper 파일(필요한 경우)
- `backend/simulation/start_ops.py`
- `backend/simulation/pit_ops.py`
- `backend/session.py`
- `backend/simulation/abstract/pose.py`
- `backend/simulation/abstract/broadcast.py`
- `backend/main.py`
- `backend/models/schemas.py`
- `backend/tests/test_abstract_race_simulation.py`
- geometry/engine/API 관련 기존 테스트
- `frontend/src/App.jsx` 및 canvas coordinate contract 테스트
- `docs/CURRENT_PROJECT_STATUS.md`

`race_engine.py`, FULL 물리, circuit JSON을 수정해야 한다면 단순 공유 helper 추출인지 먼저
증명한다. racing-line 보정값, track width 데이터, pit route 데이터 자체는 변경하지 않는다.

## 6. 필수 자동 검증

### 6.1 Bahrain/RBR metric length

두 회로 모두 다음을 검증한다.

```text
abs(measured_metric_length - circuit.track_length_m) / circuit.track_length_m <= 0.001
```

측정 대상과 공식 길이의 의미가 다른 경우 임의의 scale 보정으로 숨기지 말고 원인을 보고한다.

### 6.2 FULL/ABSTRACT pose parity

Bahrain과 RBR 각각 최소 1,000개의 동일 progress 표본을 사용한다. start/finish wrap과 주요 코너
표본을 반드시 포함한다.

- position error: `p95 <= 0.10m`, `max <= 0.30m`
- heading error: `p95 <= 0.5°`

가능하면 양쪽에서 같은 public sampler를 직접 사용해 부동소수 오차 수준으로 맞춘다. 테스트가
같은 구현을 두 번 호출하는 것만으로 끝나지 않도록 race-info 직렬화/역변환 좌표도 별도로
검증한다.

### 6.3 race_info completeness/parity

FULL과 ABSTRACT_BROADCAST에 대해 다음을 확인한다.

- 공통 geometry 배열과 mapping이 비어 있지 않음
- world origin/scale이 유한하고 scale이 양수
- racing/driving line이 닫힌 경로이며 length가 양수
- track-width 좌우 값이 양수
- grid slot이 참가 차량 수와 같고 position이 유일함
- pit lane과 pit-exit lane point가 충분하고 FULL과 ABSTRACT 값이 동일함
- 차량 치수와 track-width ratio가 동일함
- Pydantic/API JSON round trip 후 단위와 좌표가 유지됨

### 6.4 단계 1 비회귀

- 동일 seed의 grid, finish order, logical events와 canonical result hash 유지
- 10/57랩 frame/state `0/0`
- hash 두 번째 접근이 캐시됨
- replay speed/pause/resume/close 테스트 유지
- ABSTRACT 계산은 event loop 밖에서 수행
- FULL 기본 모드와 기존 RaceInfo/engine 테스트 유지

### 6.5 실행할 테스트

최소한 다음 범위를 실행한다.

- abstract/result/replay/API 테스트
- track physics, engine RaceInfo, pit route, start grid 관련 테스트
- Bahrain/RBR 실제 데이터 geometry 테스트
- frontend contract test, lint, production build
- backend `compileall`
- 문서 상대 링크 검사
- `git diff --check`

전체 backend discovery는 시간이 허용되면 실행한다. 실행하지 않았다면 명확히 미실행으로
보고하며 통과했다고 쓰지 않는다.

## 7. 수동 확인

Bahrain과 RBR에서 각각 단일 차량을 다음 위치에 표시한다.

- start/finish 직전·직후
- 화면 좌측 상단 코너
- 화면 우측 상단 코너
- pit entry, pit lane, pit exit/rejoin

FULL과 ABSTRACT를 같은 viewport로 비교해 다음을 확인한다.

- 트랙 형상·회전·화면 점유율
- 차량 길이/폭 대 도로 폭 비율
- racing line 위 차량 중심과 heading
- lap wrap 시 위치/heading 불연속 부재

가능하면 같은 viewport의 screenshot을 남기고, 불가능하면 progress, world metre 좌표, render
역변환 좌표, position/heading 오차를 진단 JSON으로 남긴다.

## 8. 금지 사항

- 20대 교통, 추월, 방어, 접촉 balance 변경
- 종·횡 속도, 가속·제동, start launch 구현
- pit state machine, pit timing, pit strategy 변경
- incident, VSC, SC, restart 구현
- FULL racing-line/track-width/pit calibration 값 변경
- renderer fallback을 유지한 채 완료 선언
- 승인 기준 완화 또는 실제 콘텐츠 대신 synthetic track만 검증
- 사용자 요청 없는 commit, push, branch 변경, package 생성
- 기존 미커밋 변경 reset/checkout/clean/stash

## 9. 완료 보고 형식

```text
단계: 2 — FULL 공통 트랙·그래픽 좌표 계약
판정: 완료 후보 / 조건부 / 실패
단계 3 시작 여부: 시작하지 않음

구현 요약:
- 공통 geometry 계약과 소비 경로
- FULL private 의존 제거 내용
- ABSTRACT race_info와 pose sampler 변경

Bahrain 결과:
- metric length error
- pose position p95/max
- heading p95/max
- race_info point counts와 ratio

RBR 결과:
- metric length error
- pose position p95/max
- heading p95/max
- race_info point counts와 ratio

비회귀:
- 단계 1 hash/finish/events/자원 결과
- FULL 관련 테스트 결과
- frontend 결과

수동 확인:
- screenshot 또는 진단 파일 경로

미실행/알려진 한계:
- 실행하지 않은 전체 회귀
- 단계 3 이후로 남긴 항목

변경 파일:
- 파일 목록
```

보고 후 코드를 더 수정하지 말고 독립 검증을 기다린다.

## 10. 다른 에이전트 전달용 프롬프트

```text
작업 저장소는 /Users/kimyongjin/Desktop/f1 입니다.
현재 브랜치는 codex/abstract-race-simulation 입니다.

먼저 다음 문서를 처음부터 끝까지 읽으세요.
1. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_STAGE2_WORK_DIRECTIVE.md
2. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_REWORK_DIRECTIVE.md
3. /Users/kimyongjin/Desktop/f1/docs/ABSTRACT_RACE_SIMULATION_DESIGN.md
4. /Users/kimyongjin/Desktop/f1/docs/SIMULATION_FOUNDATION.md
5. /Users/kimyongjin/Desktop/f1/docs/CURRENT_PROJECT_STATUS.md의 ABSTRACT 단계 1 및 최신 절

이번 작업은 "단계 2 — FULL 공통 트랙·그래픽 좌표 계약"만 수행합니다. 단계 1은 승인됐습니다.
단계 3의 차량 종·횡 운동, 출발 가속, 제동/코너 속도 전이와 20대 교통·추월, 피트 상태 머신,
사건/VSC/SC/restart, 제품 패키징은 구현하지 마세요.

현재 작업 트리에는 검증을 통과한 단계 1 미커밋 변경이 있습니다. 전부 사용자 작업으로 간주해
보존하세요. git reset, checkout, clean, stash, 대량 되돌리기를 하지 마세요. 먼저 git status,
기준 commit과 현재 diff를 확인하고 단계 1 계약을 파악하세요. 사용자 요청 없이 commit, push,
branch 변경, package 생성도 하지 마세요.

핵심 목표는 ABSTRACT가 별도 centerline Catmull–Rom 좌표계를 사용하지 않고 FULL과 동일한
public TrackPhysicsProfile/local metric frame/racing-line sampler 및 grid/pit display geometry를
사용하게 만드는 것입니다. ABSTRACT가 FULL RaceEngine 인스턴스나 private 필드에 의존하게
만들지 말고, 필요한 값을 public immutable display DTO/helper로 추출해 두 경로가 공유하게 하세요.

반드시 다음 순서로 진행하세요.
1. Bahrain(id 3)과 Red Bull Ring(id 4)의 현재 metric length, coordinate frame, pose 오차와
   race_info 빈 필드를 진단하고 실패 테스트를 먼저 만드세요.
2. compiled Circuit에서 world frame, racing/driving line, track width, grid, pit/pit-exit 및 차량
   치수를 만드는 public immutable geometry 계약을 설계하세요.
3. FULL RaceInfo와 ABSTRACT Broadcast RaceInfo가 같은 계약을 소비하게 하되 FULL의 기존 수치와
   circuit calibration 값은 바꾸지 마세요.
4. SingleVehiclePoseSynthesizer 기본 pose를
   TrackPhysicsProfile.line_pose_at_progress_m(DRIVING_LINE_RACING, progress) 기준으로 바꾸세요.
   world_x_m/world_y_m는 coordinate_frame.to_local_m() 기준이어야 합니다.
5. Bahrain/RBR 각 최소 1,000 progress 표본에서 FULL 대비 position p95 <= 0.10m,
   max <= 0.30m, heading p95 <= 0.5도와 metric track length 오차 <= 0.1%를 검증하세요.
6. FULL/ABSTRACT race_info의 geometry가 비어 있지 않고 origin/scale, racing/driving lines,
   track-width, grid, pit/pit-exit, 차량/트랙 metric ratio가 동일한지 API/schema/frontend 테스트를
   추가하세요.
7. 단계 1의 grid, finish order, logical event 의미, canonical hash, frame/state 0/0,
   replay clock과 자원 상한이 유지되는지 확인하세요.
8. 관련 backend/API/track physics/engine/grid/pit 테스트, frontend test/lint/build, compileall,
   문서 링크와 git diff --check를 실행하세요.
9. 가능하면 Bahrain/RBR start/finish, 좌우 상단 코너, pit entry/exit 단일 차량 화면을 수동
   확인하고 screenshot을 남기세요. GUI 확인이 불가능하면 같은 지점의 world/render 좌표와
   position/heading 오차 진단 JSON을 제출하세요.
10. 작업 지시서의 완료 보고 형식으로 결과를 보고하고 즉시 멈추세요. 단계 3은 시작하지 마세요.

테스트 기준을 임의로 완화하거나 synthetic track만으로 완료 선언하지 마세요. 전체 backend
discovery를 실행하지 않았다면 미실행이라고 정확히 기록하세요. 목표를 못 맞춘 항목은 숨기지 말고
조건부 또는 실패로 보고한 뒤 독립 검증을 기다리세요.
```
