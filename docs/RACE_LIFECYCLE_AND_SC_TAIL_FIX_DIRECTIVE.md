# 레이스 종료 메모리 및 SC 후미 65km/h 고착 수정 지시문

## 1. 작업 목표

현재 확인된 아래 두 문제를 재현하고 원인을 계측한 뒤 수정한다.

1. 바레인 한 경기 종료 시 Chrome 탭 메모리가 약 2.2GB까지 증가하고, `New Race`로 새 경기를 시작해도 표시 메모리가 즉시 감소하지 않는다.
2. 세이프티카 상황에서 후미 두 차량이 약 65km/h에 계속 묶여 대열을 따라잡지 못한다.

두 현상을 임의 상수 조정이나 기능 비활성화로 숨기지 말고, 실제 제한 출처와 객체 소유자를 기록한 뒤 최소 범위로 수정한다.

## 2. 작업 원칙

- 현재 작업 트리의 기존 변경을 보존한다. 관련 없는 파일을 되돌리거나 정리하지 않는다.
- 물리 50Hz, pose 30Hz, 렌더 60FPS, 1배속·2배속 구조를 유지한다.
- 차량 위치를 순간이동시키거나 SC 순위를 강제로 재배치하지 않는다.
- 물리·AI·타이밍 기록을 삭제해 메모리 수치를 맞추지 않는다. 화면과 게임 진행에 필요한 최소 기록만 경계가 있는 버퍼로 유지한다.
- Chrome 탭 메모리, JavaScript heap, Python RSS를 서로 다른 값으로 분리해 측정한다.
- 수정 전 재현 로그와 수정 후 동일 조건 로그를 모두 남긴다.
- 커밋과 푸시는 별도 요청이 있기 전에는 하지 않는다.

## 3. 우선 재현 조건

### A. 메모리

가능하면 프로덕션 프런트엔드 빌드와 개발 서버를 각각 한 번씩 측정한다. DevTools의 Performance/Memory 녹화는 끈 상태에서 아래 순서로 실행한다.

1. 새 Chrome renderer에서 바레인 57랩, 20대를 시작한다.
2. 시작 직후, 5랩, 15랩, 30랩, 45랩, 종료 직후 값을 기록한다.
3. `New Race`를 누르고 setup 화면이 된 뒤 10초, 30초, 60초 시점 값을 기록한다.
4. 같은 탭에서 두 번째 바레인 레이스를 시작하고 같은 지점을 기록한다.
5. 두 번째 레이스 종료 후 다시 setup으로 돌아가 같은 값을 기록한다.

각 지점에서 반드시 기록할 값:

- Chrome 작업 관리자 탭 memory footprint
- macOS Activity Monitor의 해당 Chrome renderer RSS
- 화면 `DIAG`의 JavaScript heap 현재값·최고값·MB/분
- DOM node 수
- canvas 수
- WebGL geometry·texture 수
- scene/garage/car model build 수
- pose sample 수
- 열린 WebSocket 수 또는 연결 상태
- 백엔드 Python RSS
- `session_manager.session` 유무
- 실행 중인 레이스 loop task 유무
- 차량별 predictive speed cache 항목 합계
- vehicle track physics cache 항목 수
- timing crossing, trajectory, event/history 버퍼 크기

중요 판정:

- 종료 뒤 RSS가 바로 떨어지지 않는 것만으로 누수라고 판정하지 않는다. V8·Chromium·GPU allocator는 확보한 영역을 다음 사용을 위해 유지할 수 있다.
- 대신 이전 `RaceSession`, `RaceEngine`, WebSocket, animation loop, WebGL context가 살아 있는지와 두 번째 레이스가 첫 경기만큼 메모리를 다시 추가하는지를 판정한다.
- setup 화면에서 canvas 0, 성능 진단 노드 0, 서버 active session false가 되어야 한다.
- 새 레이스에서는 canvas와 WebGL context가 각각 하나만 생성되어야 한다.

### B. 후미 65km/h

SC를 투입한 뒤 피트인·피트아웃 차량이 섞이는 조건과 섞이지 않는 조건을 모두 실행한다. 후미 두 차량이 65km/h 부근에 3초 이상 머물면 매 0.5초마다 아래 필드를 기록한다.

- driver id, 순위, 물리 `total_progress`, 속도
- `race_phase`, `safety_car_stage`
- `_sc_caught_driver_ids` 포함 여부
- `_sc_unlap_driver_ids` 포함 여부
- `_sc_pit_exit_order_targets` 값
- `_sc_order_yield_targets` 값과 지정 predecessor
- `_sc_running_order`의 현재 인덱스
- predecessor id·속도·물리 간격
- `_sc_queue_control_distance_m`
- `_race_control_speed_cap_mps` 결과
- `speed_limit_factor`
- `maximum_speed_mps`
- `emergency_braking`, `avoidance_active`, hazard id와 TTC
- `in_pit`, `pit_phase`, `pit_merge_state`
- local yellow 여부
- finished·retired 여부

최종 로그에는 속도 제한의 출처를 문자열로 함께 남긴다. 최소한 아래 값을 구분한다.

- `none`
- `sc_caught`
- `sc_approach`
- `sc_order_yield`
- `sc_unlap`
- `vsc`
- `emergency_braking`
- `hazard_following`
- `pit_limit`
- `pit_merge_hold`

## 4. 현재 코드에서 우선 확인할 원인

### 4.1 SC 순서 복원 제한

`backend/simulation/safety_car.py`의 `_race_control_speed_cap_mps()`는 `_sc_order_yield_targets`에 든 차량을 predecessor보다 18km/h 느리게 제한하고 즉시 반환한다.

후미 차량이 잘못된 yield target을 보유하면, 대열 합류를 위한 양의 closing margin 로직까지 도달하지 못하고 predecessor의 저속을 계속 복사할 수 있다.

확인 사항:

- 후미 두 차량이 실제로 `_sc_order_yield_targets`에 남아 있는가
- predecessor가 피트인·피트아웃·리타이어·완주로 바뀌었는데 기존 target이 남는가
- 물리 순서는 이미 복원됐지만 signed gap 계산의 랩 경계 때문에 release 조건을 통과하지 못하는가
- SC2 전후의 provisional pit-exit order가 frozen order와 중복되거나 누락되는가
- 마지막 두 차량에서 `_sc_running_order`와 실제 물리 진행 순서가 엇갈리는가

수정 방향:

- yield target은 실제 순서 역전 복구에만 사용한다.
- target predecessor가 비활성·피트·완주·리타이어이면 즉시 재평가한다.
- 랩 경계를 포함한 signed physical gap을 공통 헬퍼로 계산한다.
- 이미 predecessor 뒤로 충분히 복귀한 차량은 같은 틱에 target을 해제한다.
- uncaught 차량에는 yield 제한보다 SC queue catch-up 제어가 우선되어야 하는지 테스트로 결정한다.

### 4.2 SC 후미 catch-up 전파

현재 rear-train 단위 테스트는 모든 후미 차량이 65km/h일 때 cap이 65km/h보다 커지는지만 확인한다. 다음 실제 상태는 추가로 덮어야 한다.

- 마지막 두 차량만 600m queue-control horizon 밖에 있는 경우
- 마지막 두 차량이 horizon 안으로 들어오는 경계
- 앞 차량은 caught, 후미는 uncaught인 상태
- 후미 차량이 pit-out provisional order에 있는 상태
- 후미 차량이 SC2를 통과하며 order가 확정되는 틱
- predecessor가 피트로 들어가거나 리타이어하는 틱
- SC `collecting → queued → in_this_lap → restart → green`

catch-up 차량은 실제 queue-join gap 밖에서는 predecessor보다 최소 8km/h 이상 빠르게 닫을 수 있어야 한다. 다만 코너에서는 green-flag 물리 한계와 정상 following constraint를 넘지 않아야 한다.

### 4.3 비상 제동 0.35배 제한

`backend/simulation/race_engine.py`는 `state.emergency_braking`일 때 `speed_limit_factor`에 0.35를 곱한다. 속도 결과가 약 65km/h로 보일 수 있으므로 SC 제한과 혼동하지 않는다.

현재 avoidance/emergency 상태는 매 물리 틱 시작에 초기화되지만 같은 틱에 hazard 경로가 다시 활성화될 수 있다. 후미 차량 앞에 제거되지 않은 hazard, retired pose 또는 잘못된 corridor leader가 남는지 확인한다.

필수 테스트:

- hazard 해제 다음 틱에 emergency braking이 false가 된다.
- retired 차량의 `hazard_active=false` 뒤 following constraint가 생성되지 않는다.
- SC 종료 뒤 race-control cap은 `None`이고 emergency 상태도 정상 해제된다.

## 5. 메모리 원인별 확인과 수정 방향

### 5.1 백엔드 세션 객체

현재 `RaceSession.close()`와 `SessionManager.clear_async()`는 loop task 취소, WebSocket 종료, trajectory/event/history 보조 버퍼 삭제, vehicle-specific track cache 삭제와 GC를 수행한다.

단위 확인에서는 이벤트 루프가 한 번 더 순환한 뒤 이전 `RaceSession`과 `RaceEngine` weak reference가 모두 수거됐다. 이 경로는 유지하되 실제 서버에서도 동일한지 계측한다.

필수 보강:

- 세션 close 전후에 session id, loop task 상태, client 수, 주요 버퍼 크기를 개발 로그로 한 번만 남긴다.
- old session/engine이 다음 이벤트 루프 뒤에도 살아 있으면 `gc.get_referrers()`로 소유자를 찾는다.
- `create_session()`의 동기 `clear()`와 API의 `clear_async()`가 중복되더라도 새 세션 생성 중 이전 비동기 close를 건너뛰지 않도록 한다.
- 자연스러운 race end 시점과 `New Race` 클릭 시점을 구분한다. 결과 모달이 떠 있는 동안은 기존 장면이 의도적으로 표시되지만, `New Race` 후에는 반드시 해제되어야 한다.

### 5.2 예측·트랙 캐시

- predictive speed cache는 인스턴스당 최근 256개 상한을 유지한다.
- 차량별 vehicle track physics cache는 레이스 종료 시 비운다.
- 공용 기본 트랙 프로필 캐시는 다음 레이스에서 재사용할 수 있으나 크기와 상한을 계측한다.
- 캐시를 무조건 제거해 계산량을 늘리지 말고, bounded LRU와 공유 가능한 불변 프로필을 사용한다.
- 레이스 A 종료 후 캐시 수, 레이스 B 시작·종료 후 캐시 수가 상한을 넘어 계속 증가하지 않아야 한다.

### 5.3 프런트엔드 상태

`useRaceWebSocket`의 아래 자료가 종료 후 모두 비워지는지 확인한다.

- `eventsRef`
- pose/dashboard refs
- `historyByDriverRef`
- `timingByDriverRef`
- transport window
- React `raceInfo`, `raceState`, `raceEnd`, events

WebSocket 이벤트 리스너는 익명 함수 대신 명명된 핸들러를 사용해 cleanup에서 `removeEventListener()`까지 수행하는 방식을 검토한다. `ReconnectingWebSocket`이 close 이후 재연결 타이머나 listener closure를 남기는지 heap snapshot에서 확인한 뒤 필요할 때만 수정한다.

랩 기록은 경기 결과에 필요한 큰 틀만 유지한다.

- 드라이버별 완료 랩 요약
- S1/S2/S3
- 필요한 미니섹터 비교 범위
- 최고·마지막 기록

화면에 다시 쓰지 않는 원본 tick, 전체 과거 pose, 반복 전체 snapshot은 보존하지 않는다.

### 5.4 Three.js와 Chromium native memory

현재 종료 cleanup은 animation loop 정지, geometry/material/texture 해제, render list 해제, renderer dispose, context loss와 canvas 제거를 수행한다.

추가 확인:

- 모든 material의 `map`, `normalMap`, `roughnessMap` 등 연결 texture가 해제되는가
- 개러지·차량 effect별 cleanup과 전체 scene cleanup이 중복 생성된 객체를 놓치지 않는가
- 첫 race와 두 번째 race에서 scene build 수가 레이스 중 계속 증가하지 않는가
- setup 전환 후 canvas와 WebGL context가 0개인가
- 개발 모드 React StrictMode의 초기 이중 생성과 실제 누수를 구분하는가

Chrome renderer RSS가 내려가지 않더라도 두 번째 경기에서 geometry·texture·canvas가 중복되지 않고 JavaScript heap이 GC 후 기준 범위로 복귀하면 allocator high-water일 가능성이 높다. 반대로 두 번째 경기마다 탭 footprint가 비슷한 폭으로 더 증가하면 native resource 또는 listener/ArrayBuffer 보존을 계속 추적한다.

## 6. 필수 회귀 테스트

### SC 및 65km/h

1. 20대 rear train에서 마지막 두 차량도 같은 틱에 65km/h보다 높은 catch-up cap을 받는다.
2. yield target 차량은 물리 순서 복원 후 즉시 target에서 제거된다.
3. pit-out provisional order와 SC2 확정 전후에 후미차가 고착되지 않는다.
4. predecessor pit/retire/finish 시 stale target이 남지 않는다.
5. SC 종료 후 모든 SC set/dict가 비고 정상 green target speed로 복귀한다.
6. 실제 바레인 SC 전체 수명주기에서 어떤 정상 주행 차량도 65±2km/h에 불필요하게 3초 이상 머물지 않는다.

### 세션 및 메모리

1. started loop를 가진 세션도 `clear_async()` 뒤 task 0, client 0이 된다.
2. 다음 이벤트 루프 뒤 이전 session/engine weak reference가 수거된다.
3. `Exit`와 `New Race` 모두 같은 cleanup 경로를 사용한다.
4. setup 화면에서 canvas 0, 성능 노드 0, active session false다.
5. 두 번째 레이스에서 canvas 1, WebGL context 1을 유지한다.
6. 두 경기 연속 soak에서 JS heap, geometry, texture, pose buffer와 서버 캐시가 설정된 상한을 넘지 않는다.

## 7. 승인 기준

- 65km/h 문제는 단순히 속도를 올리는 것으로 끝내지 않고 실제 cap source 로그로 잘못된 상태가 사라졌음을 증명한다.
- 바레인 20대 SC 수명주기를 피트인·피트아웃 포함 조건에서 완료한다.
- 레이스 종료 후 이전 세션과 엔진 객체가 유지되지 않는다.
- Chrome 탭 2.2GB는 현재 목표 범위를 벗어난 실패값으로 취급한다.
- 새 renderer 기준 1경기 종료 시 탭 footprint는 가능하면 400~650MB 수준, 일시 peak는 1GB 미만을 목표로 한다.
- allocator high-water 때문에 setup에서 숫자가 즉시 감소하지 않더라도, 두 번째 경기 종료 수치가 첫 경기 종료 대비 지속적으로 큰 폭 증가하지 않아야 한다.
- 기존 backend 전체 테스트와 subtest, frontend lint, production build, `git diff --check`를 모두 통과한다.

## 8. 최종 보고 형식

최종 보고에는 아래를 포함한다.

1. 두 문제의 재현 여부
2. 확정 원인과 코드 위치
3. 수정 파일 목록
4. 수정 전·후 메모리 표
5. 65km/h 차량의 수정 전·후 cap source 로그
6. 두 경기 연속 수명주기 결과
7. 테스트·빌드 결과
8. 남은 한계와 다음 권장 작업
