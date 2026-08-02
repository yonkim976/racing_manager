# macOS 데스크톱 진단 앱 구현 계획서

## 0. 문서 상태

- 문서 목적: 다른 구현 에이전트가 현재 웹앱을 macOS 진단 앱으로 포장하고, 레이스 간 메모리 증가를 프로세스별로 추적할 수 있도록 작업 범위와 승인 기준을 고정한다.
- 현재 상태: 계획 승인 대기. 데스크톱 셸은 아직 구현되지 않았다.
- 1차 런타임 결정: **Electron 기반 진단용 macOS 앱**
- 2차 후보: Electron에서 원인을 수정하고 동일 soak를 통과한 뒤에만 Tauri 2/WKWebView 비교 실험을 검토한다.
- 이 문서는 [`RACE_LIFECYCLE_AND_SC_TAIL_FIX_DIRECTIVE.md`](RACE_LIFECYCLE_AND_SC_TAIL_FIX_DIRECTIVE.md)를 대체하지 않는다. SC 65km/h 수정과 데스크톱 포장은 별도 작업 축이다.
- 별도 요청이 없으면 커밋과 푸시는 하지 않는다.

## 1. 배경과 핵심 판단

현재 앱은 다음 세 부분으로 구성되어 있다.

1. React UI와 Three.js 렌더러
2. FastAPI WebSocket 서버
3. Python `RaceSession`/`RaceEngine` 시뮬레이션

웹앱을 macOS 앱으로 포장한다고 JavaScript 참조, WebGL resource, WebSocket listener 또는 Python 세션 누수가 자동으로 해결되지는 않는다.

Electron을 1차로 선택하는 이유는 다음과 같다.

- 현재 Chrome과 같은 Chromium 계열에서 증상을 재현한다.
- Browser/Tab/GPU/Utility 프로세스별 메모리를 앱에서 수집할 수 있다.
- Chrome DevTools와 tracing을 계속 사용할 수 있다.
- 기존 React/Three.js 코드를 거의 그대로 사용한다.
- Python 백엔드를 별도 sidecar 프로세스로 두어 프로세스 수명주기를 명시적으로 통제할 수 있다.

Tauri를 바로 사용하지 않는 이유는 macOS에서 WKWebView를 사용해 렌더링 엔진이 바뀌기 때문이다. 문제가 보이지 않게 되더라도 Chrome/Chromium 자원 소유권 문제가 해결된 것인지, 단순히 다른 allocator 동작으로 가려진 것인지 구분하기 어렵다.

## 2. 작업 목표

### 2.1 1차 목표

현재 웹앱을 수정 범위를 최소화하여 다음 기능을 가진 macOS Electron 앱으로 실행한다.

- 앱이 Python/FastAPI sidecar를 직접 시작한다.
- 백엔드가 준비된 뒤에만 UI 창을 연다.
- 앱 종료 시 race session, WebSocket, Python sidecar를 순서대로 종료한다.
- Electron main/renderer/GPU와 Python RSS를 같은 시간축으로 기록한다.
- 경기 시작, 주요 랩, 경기 종료, `New Race` 후 10/30/60초 checkpoint를 기록한다.
- 같은 앱 프로세스에서 바레인 두 경기를 연속 실행해 per-race 증가를 판정할 수 있다.
- 진단 기능이 물리 50Hz, pose 30Hz, 렌더 60FPS, 1배속·2배속 구조를 바꾸지 않는다.

### 2.2 2차 목표

Electron 진단 앱에서 수집한 자료로 다음을 구분한다.

- JavaScript heap 증가
- Blink/DOM/ArrayBuffer 증가
- GPU/WebGL resource 증가
- Chromium allocator high-water
- Python 세션 또는 cache 증가
- 종료되지 않은 WebSocket, animation loop 또는 sidecar 프로세스

### 2.3 비목표

이번 작업에 포함하지 않는다.

- SwiftUI/Metal 기반 전체 재작성
- Python 시뮬레이션을 Rust, Swift 또는 Node.js로 이전
- SC 65km/h 물리 로직 자체 수정
- 물리 주기, pose 주기, 렌더 FPS 변경
- 기록 삭제 또는 기능 비활성화로 메모리 수치만 낮추기
- Tauri production 앱 동시 구현
- 앱 스토어 제출
- 자동 업데이트
- Windows/Linux 패키징

## 3. 변경 전 필수 확인

구현 에이전트는 작업 시작 전에 반드시 다음을 수행한다.

1. 현재 작업 트리가 dirty 상태임을 확인한다.
2. 기존 변경을 사용자 소유 변경으로 취급한다.
3. 관련 없는 파일을 되돌리거나 정리하지 않는다.
4. 아래 문서를 읽는다.

   - `docs/SIMULATION_FOUNDATION.md`
   - `docs/CURRENT_PROJECT_STATUS.md`
   - `docs/RACE_LIFECYCLE_AND_SC_TAIL_FIX_DIRECTIVE.md`
   - 이 문서

5. 다음 현재 구조를 확인한다.

   - `backend/main.py`는 `frontend/dist`가 존재하면 production frontend를 제공한다.
   - `frontend/src/hooks/useRaceWebSocket.js`는 현재 페이지 host의 `/ws/race`를 사용한다.
   - `DELETE /api/race/session`은 `SessionManager.clear_async()`를 호출한다.
   - `ThreeTrackCanvas` unmount는 animation loop와 WebGL context를 정리한다.

## 4. 목표 아키텍처

```text
┌─────────────────────────────────────────────────────────────┐
│ macOS F1 Race Manager.app                                   │
│                                                             │
│  Electron Main                                              │
│  ├─ BrowserWindow lifecycle                                 │
│  ├─ ephemeral localhost port 선택                           │
│  ├─ Python sidecar start/health/shutdown                    │
│  ├─ Electron process metrics sampler                        │
│  └─ bounded diagnostic log writer                           │
│         │                                      ▲            │
│         │ preload의 제한된 IPC                 │ metrics    │
│         ▼                                      │            │
│  Chromium Renderer ───── HTTP/WebSocket ─── FastAPI sidecar │
│  React + Three.js                         RaceSession/Engine │
└─────────────────────────────────────────────────────────────┘
```

### 4.1 로컬 origin

1차 구현에서는 Electron이 `file://`로 frontend를 직접 열지 않는다.

권장 방식:

1. `npm run build`로 `frontend/dist`를 만든다.
2. FastAPI sidecar가 해당 production frontend를 제공한다.
3. Electron은 `http://127.0.0.1:<ephemeral-port>/`를 연다.

장점:

- 현재 상대 `/api` 요청을 유지할 수 있다.
- 현재 `window.location.host` 기반 WebSocket URL을 유지할 수 있다.
- CORS 또는 별도 API base URL 변경을 최소화한다.

백엔드는 반드시 `127.0.0.1`에만 bind한다. `0.0.0.0`에 bind하지 않는다.

### 4.2 sidecar 인증

다른 로컬 프로세스가 desktop 전용 진단·종료 endpoint를 호출하지 못하도록 매 실행마다 임의 token을 만든다.

권장 환경 변수:

```text
F1_DESKTOP_MODE=1
F1_DESKTOP_TOKEN=<cryptographically-random-token>
F1_FRONTEND_DIST=<absolute-packaged-frontend-dist>
```

일반 race API를 모두 token화하는 것은 1차 범위가 아니다. 다음 desktop 전용 endpoint 또는 IPC만 token을 요구한다.

- runtime diagnostic snapshot
- graceful application shutdown
- diagnostic marker 기록

token은 URL query에 남기지 말고 header 또는 좁은 IPC 경로로 전달한다. 로그에는 token을 기록하지 않는다.

## 5. 디렉터리와 파일 구상

예상 신규 구조:

```text
desktop/
├── package.json
├── package-lock.json
├── forge.config.js
├── src/
│   ├── main.js
│   ├── preload.js
│   ├── backendProcess.js
│   ├── diagnostics.js
│   └── lifecycle.js
├── scripts/
│   ├── prepareFrontend.js
│   └── preparePythonSidecar.js
├── resources/
│   └── .gitkeep
└── test/
    ├── backendProcess.test.js
    ├── diagnostics.test.js
    └── lifecycle.test.js
```

예상 기존 파일 변경:

```text
backend/main.py
backend/session.py
backend/simulation/track_physics.py
backend/simulation/vehicle_physics.py
backend/pyproject.toml
backend/tests/test_session.py
frontend/src/App.jsx
frontend/src/hooks/useRaceWebSocket.js
frontend/src/components/TrackView/ThreeTrackCanvas.jsx
docs/CURRENT_PROJECT_STATUS.md
docs/README.md
.gitignore
```

실제 구현 중 더 작은 구조로 해결할 수 있으면 파일 수를 줄여도 된다. 단, lifecycle, sidecar process, diagnostics 책임을 하나의 거대한 `main.js`에 섞지 않는다.

## 6. 앱 수명주기

### 6.1 상태

Electron main process가 다음 상태를 권위 있게 관리한다.

```text
BOOTING
BACKEND_STARTING
BACKEND_READY
WINDOW_READY
RACING
RESULTS
DISPOSING_RACE
IDLE
APP_SHUTTING_DOWN
FAILED
```

React 화면 상태와 Electron lifecycle 상태를 동일 객체로 합치지 않는다. Renderer는 main lifecycle을 읽을 수 있지만 임의로 변경할 수 없다.

### 6.2 시작

1. app ready
2. production frontend 존재 확인
3. 사용 가능한 localhost port 선택
4. Python sidecar spawn
5. stdout/stderr 캡처 시작
6. 제한 시간 동안 `/api/health` polling
7. health 성공 시 BrowserWindow 생성 또는 표시
8. 지정 local origin 로드
9. 초기 metric checkpoint 기록

권장 제한:

- backend health timeout: 30초
- polling interval: 200~500ms
- 실패 시 빈 창이 아니라 오류 화면과 로그 위치를 표시
- 자동 무한 재시작 금지

### 6.3 `New Race`와 `Exit`

둘 다 같은 race disposal 경로를 사용한다.

1. UI 입력 잠금
2. lifecycle `DISPOSING_RACE`
3. `DELETE /api/race/session`
4. 응답 후 renderer state reset
5. canvas/WebGL cleanup 완료 marker 대기
6. setup 전환
7. 10/30/60초 checkpoint 예약
8. lifecycle `IDLE`

`DELETE` 실패 시에도 UI만 먼저 setup으로 바꾸지 않는다. 실패 원인을 표시하고 retry 또는 앱 전체 재시작을 선택할 수 있게 한다.

### 6.4 앱 종료

1. 중복 종료 방지 flag 설정
2. sampler 중지
3. active race session `clear_async()`
4. 로그 flush
5. Python에 graceful termination 요청
6. 정해진 시간 안에 종료되지 않으면 `SIGTERM`
7. 그래도 종료되지 않을 때만 마지막 수단으로 강제 종료
8. 자식 PID가 남지 않았음을 확인한 뒤 Electron 종료

강제 종료는 정상 경로가 아니며 diagnostic log에 원인을 남긴다.

## 7. Electron 보안 계약

BrowserWindow 기본값:

```js
{
  webPreferences: {
    nodeIntegration: false,
    contextIsolation: true,
    sandbox: true,
    preload: PRELOAD_PATH,
  },
}
```

추가 규칙:

- Renderer에 `ipcRenderer` 자체를 노출하지 않는다.
- preload는 다음처럼 좁은 API만 제공한다.

```text
desktopDiagnostics.recordRendererSnapshot(snapshot)
desktopDiagnostics.recordCheckpoint(name, details)
desktopDiagnostics.getLifecycleState()
desktopDiagnostics.getLogLocation()
```

- 모든 IPC payload는 main에서 schema와 크기를 검증한다.
- 외부 navigation을 막는다.
- 새 창 생성을 기본 차단한다.
- 개발 모드 외에는 DevTools 자동 실행을 막는다.
- `webSecurity`를 끄지 않는다.
- remote URL을 BrowserWindow 안에서 열지 않는다.

## 8. 진단 데이터 설계

### 8.1 로그 형식

newline-delimited JSON(JSONL)을 사용한다.

파일 위치:

```text
<Electron app userData>/diagnostics/
  YYYYMMDD-HHMMSS-<app-session-id>.jsonl
```

각 record 공통 필드:

```json
{
  "schema_version": 1,
  "timestamp": "ISO-8601",
  "monotonic_ms": 0,
  "app_session_id": "uuid",
  "race_session_id": "uuid-or-null",
  "race_index": 0,
  "record_type": "sample|checkpoint|lifecycle|error",
  "lifecycle": "RACING"
}
```

로그는 다음과 같이 제한한다.

- 기본 sampling interval: 5초
- raw pose, 전체 dashboard snapshot, 전체 event history 저장 금지
- 파일당 크기 상한 설정
- 파일 수 또는 총 용량 기준 rotation
- 권장 초기 상한: 최근 20개 파일 또는 총 200MB 중 먼저 도달하는 조건

### 8.2 Electron 프로세스 지표

`app.getAppMetrics()` 기반:

- pid
- creationTime
- process type
- service name
- CPU percent
- memory.private
- memory.shared 또는 제공되는 추가 memory field
- sandboxed

최소 분류:

- Browser/main
- Tab/renderer
- GPU
- Utility/network
- unknown

macOS에서는 단일 RSS 숫자만으로 판정하지 않고 `private` 메모리를 핵심 비교값으로 사용한다.

### 8.3 Renderer 지표

기존 DIAG를 확장하거나 재사용한다.

- JS heap current/peak/trend
- Blink allocated/total을 얻을 수 있으면 포함
- DOM node 수
- canvas 수
- active WebGL canvas/context 수
- renderer.info.memory.geometries
- renderer.info.memory.textures
- render calls/triangles
- scene build count
- garage build count
- car model build count
- pose sample count
- 열린 WebSocket 수 또는 상태
- React phase
- race end overlay 유무

Renderer snapshot을 main IPC로 보낼 때 5초에 한 번보다 자주 보내지 않는다.

### 8.4 Python 지표

가능하면 `psutil`을 명시적 dependency로 추가해 current RSS를 기록한다. `ru_maxrss`만 사용하면 peak만 알 수 있으므로 현재값과 구분한다.

최소 필드:

- pid
- current RSS
- peak RSS
- active session 존재
- session id
- engine id 또는 stable diagnostic id
- loop task 존재/done/cancelled
- client 수
- trajectory sample 합계
- event-feed auxiliary map 크기
- last-history-length map 크기
- predictive speed cache 항목 합계
- predictive track sample cache 항목 합계
- common track physics cache 항목 수
- vehicle track physics cache 항목 수
- timing crossing 항목 합계
- lap history 항목 합계
- local trajectory plan/cache 크기
- SC set/dict 크기

내부 dict를 endpoint에서 직접 조립하지 말고 각 도메인이 작은 `diagnostic_counts()` 함수를 제공하도록 한다.

### 8.5 checkpoint

자동 checkpoint 이름:

```text
app_started
backend_ready
setup_idle
race_started
lap_5
lap_15
lap_30
lap_45
race_finished
new_race_clicked
race_disposed
setup_after_10s
setup_after_30s
setup_after_60s
second_race_started
app_shutdown_requested
backend_exited
```

랩 수가 짧은 개발 경기는 존재하는 checkpoint만 기록한다.

## 9. 백엔드 변경 계획

### 9.1 FastAPI lifespan

현재 전역 `FastAPI()` 생성 방식을 lifespan context로 보강한다.

- startup에서 desktop mode와 token을 읽는다.
- shutdown에서 반드시 `await session_manager.clear_async()`를 호출한다.
- 테스트와 일반 웹 실행을 깨지 않는다.

### 9.2 health

`/api/health`는 최소한 다음 값을 반환한다.

```json
{
  "status": "ok",
  "active_session": false,
  "desktop_mode": true,
  "pid": 1234
}
```

민감한 token은 반환하지 않는다.

### 9.3 진단 endpoint

desktop mode에서만 활성화하는 read-only endpoint를 추가한다.

예:

```text
GET /api/desktop/diagnostics
```

요구 조건:

- desktop token 검증
- side effect 없음
- payload 크기 제한
- 객체 전체 serialization 금지
- GC 강제 실행 금지
- production web mode에서는 404 또는 비활성

### 9.4 세션 close 관측

`RaceSession.close()` 전후에 구조화된 diagnostic record를 한 번씩 만든다.

전:

- session id
- task 상태
- client 수
- 주요 buffer 크기

후:

- task 0
- client 0
- 주요 buffer 0
- vehicle-specific cache 0

일반 physics tick마다 close 로그를 남기지 않는다.

### 9.5 atomic session replacement

현재 API는 먼저 `clear_async()`를 호출하고 `create_session()` 내부에서 다시 동기 `clear()`를 호출한다.

1차 작업에서 최소한 다음 계약을 테스트한다.

- 새 session을 publish하기 전에 old session의 async close가 끝난다.
- 동시에 두 setup 요청이 들어와도 old session close를 건너뛰지 않는다.
- sync `stop_loop()` 경로가 production replacement의 주 경로가 되지 않는다.

필요하면 `asyncio.Lock`을 가진 `replace_session_async()`를 추가한다. 불필요하게 큰 리팩터링은 피한다.

## 10. 프런트엔드 변경 계획

### 10.1 desktop 환경 감지

`window.desktopDiagnostics` 존재 여부로 desktop 기능을 감지한다. 일반 브라우저에서는 현재 동작을 유지한다.

### 10.2 renderer metric 전달

현재 `ThreeTrackCanvas`의 performance stats를 5초 주기로 desktop preload에 전달한다.

규칙:

- React render마다 IPC를 보내지 않는다.
- 전달 객체는 숫자와 짧은 문자열만 포함한다.
- positions, track coordinate, history 원본을 전달하지 않는다.

### 10.3 WebSocket 수명주기

`useRaceWebSocket`의 `open`, `close`, `error`, `message` 핸들러를 명명된 함수로 만든다.

cleanup:

- 각 listener `removeEventListener`
- reconnecting socket close
- refs와 history/timing maps clear
- transport stats reset
- desktop checkpoint 기록

heap snapshot으로 실제 보존이 확인되지 않은 라이브러리 내부를 임의 monkey patch하지 않는다.

### 10.4 Three.js resource disposal

공통 helper를 둔다.

```text
disposeObject3D(root)
disposeMaterial(material, disposedTextures)
```

처리할 texture slot 예:

- map
- normalMap
- roughnessMap
- metalnessMap
- aoMap
- alphaMap
- emissiveMap
- bumpMap
- displacementMap
- environment map 계열

규칙:

- `Set`으로 shared texture 중복 dispose 방지
- geometry dispose
- material dispose
- renderLists dispose
- renderer dispose
- animation loop null
- context loss
- canvas remove
- ResizeObserver disconnect

cleanup이 여러 effect에서 실행되어도 idempotent해야 한다.

### 10.5 disposal acknowledgement

`New Race` 시 다음 조건을 만족한 뒤 renderer disposal 완료 marker를 main에 보낸다.

- animation loop stopped
- pose buffers cleared
- scene refs null
- renderer ref null
- canvas removed

WebGL context 수는 브라우저 API가 직접 제공하지 않으면 canvas count와 renderer lifecycle id로 대체 관측한다. 관측할 수 없는 값을 임의로 0이라고 기록하지 않는다.

## 11. Electron 구현 단계

### Phase A — 최소 실행 셸

완료 조건:

- Electron 창에서 기존 production UI가 열린다.
- backend가 자동 시작된다.
- setup과 race 시작이 동작한다.
- 앱 종료 후 Python child process가 남지 않는다.
- 개발 중에는 기존 브라우저/Vite 실행도 계속 가능하다.

### Phase B — 통합 진단

완료 조건:

- 5초마다 Electron/renderer/Python sample이 한 JSONL에 기록된다.
- lifecycle과 race checkpoint가 기록된다.
- 로그 경로를 UI 또는 개발 메뉴에서 확인할 수 있다.
- sampler 자체가 무제한 메모리를 사용하지 않는다.

### Phase C — race disposal barrier

완료 조건:

- `Exit`와 `New Race`가 같은 disposal 경로를 사용한다.
- setup 전환 시 active session false
- canvas 0
- renderer lifecycle disposed
- loop task/client/buffer 0
- old session/engine weak reference 수거 테스트 통과

### Phase D — 2경기 soak

조건:

- production frontend
- 새 Electron app launch
- 바레인 57랩, 20대
- 1배속 또는 2배속은 문서에 기록
- 결과 화면 후 `New Race`
- setup 60초 대기
- 같은 조건 두 번째 경기
- 두 번째 종료 후 setup 60초 대기

반드시 결과 표에 포함:

- Browser/main private memory
- Renderer private memory
- GPU private memory
- JS heap
- Python RSS
- canvas/context lifecycle
- geometry/textures
- pose samples
- active session/task/WebSocket
- 주요 cache/buffer

## 12. 테스트 계획

### 12.1 백엔드

추가 테스트:

1. started loop session도 clear 후 task/client/buffer 0
2. 이벤트 루프 한 번 뒤 old session/engine weakref dead
3. FastAPI shutdown lifespan이 active session 정리
4. diagnostic endpoint는 desktop mode/token이 없으면 비활성
5. diagnostic endpoint가 bounded count만 반환
6. concurrent session replacement가 old close를 await
7. child termination 중 close가 두 번 호출되어도 안전

### 12.2 Electron main

Node 내장 test runner 또는 최소 dependency 테스트를 우선한다.

1. 사용 가능한 port 선택
2. health timeout
3. child early exit
4. stdout/stderr bounded capture
5. token redaction
6. graceful shutdown
7. 강제 종료 fallback
8. metric record schema
9. log rotation
10. lifecycle illegal transition 거부

### 12.3 프런트엔드

1. 일반 브라우저에서 desktop API 없이 정상 실행
2. performance snapshot이 설정 주기보다 자주 전송되지 않음
3. WebSocket cleanup이 listener와 buffers 정리
4. Three.js disposal helper가 shared texture를 한 번만 dispose
5. unmount 후 canvas가 남지 않음

### 12.4 회귀 명령

프로젝트에 실제 존재하는 명령을 확인한 뒤 실행한다.

```bash
cd backend
.venv/bin/python -m unittest discover -s tests

cd ../frontend
npm run lint
npm run build

cd ../desktop
npm test
npm run package

cd ..
git diff --check
```

테스트 전체가 너무 오래 걸리면 먼저 관련 테스트를 실행하되 최종 승인 전에는 기존 전체 backend 테스트를 실행한다.

## 13. 메모리 판정 규칙

다음은 즉시 실패다.

- 이전 `RaceSession` 또는 `RaceEngine`이 disposal 뒤 살아 있음
- setup에서 active session true
- setup에서 canvas가 남음
- 새 경기마다 renderer/WebGL context가 하나씩 추가됨
- loop task 또는 WebSocket이 경기마다 누적
- geometry/texture/pose/cache가 설정된 상한을 넘어서 계속 증가
- 앱 종료 후 Python child process가 남음

다음은 단독으로 누수 확정 근거가 아니다.

- 경기 종료 직후 RSS가 떨어지지 않음
- GPU/renderer private memory가 첫 경기의 peak를 일부 유지
- JS heap capacity가 GC 뒤 즉시 OS에 반환되지 않음

allocator high-water 판정 조건:

- old 객체와 resource lifecycle은 모두 종료됨
- 두 번째 경기에서 첫 경기와 비슷한 최대 resource count
- 두 번째 cleanup checkpoint가 첫 번째 대비 지속적으로 큰 폭 증가하지 않음
- 세 번째 짧은 확인 또는 동일 시나리오 반복에서 증가 기울기가 둔화 또는 plateau

숫자 하나만으로 승인하지 않는다. resource count와 객체 생존 여부를 함께 본다.

## 14. Tauri 비교로 넘어가는 gate

다음을 모두 만족하기 전에는 Tauri 구현을 시작하지 않는다.

- Electron에서 old session/engine 수거 증명
- 두 경기 연속 WebGL/JS/Python resource count bounded
- Chromium에서 per-race 지속 증가가 제거되거나 원인이 allocator로 분류됨
- 현재 앱 기능 회귀 없음
- Electron 진단 로그가 최종 보고에 포함됨

그 뒤 별도 spike에서 비교한다.

- 동일 production frontend
- 동일 Python sidecar
- 동일 바레인 두 경기
- Electron Chromium 대 Tauri WKWebView
- 메모리뿐 아니라 Three.js 렌더 정확도와 프레임 시간 비교

Tauri가 더 낮은 메모리를 보여도 기능 또는 렌더 품질 차이가 있으면 자동 채택하지 않는다.

## 15. 패키징 전략

### 15.1 개발용

- Electron이 `backend/.venv/bin/python -m uvicorn`을 실행
- 현재 저장소의 `frontend/dist` 사용
- DevTools 허용
- code signing 불필요

### 15.2 진단 배포용 `.app`

초기 대상:

- Apple Silicon arm64
- 저장소 외부 실행 가능
- production frontend 포함
- Python runtime과 data 포함

Python은 우선 one-directory 형태의 sidecar를 검토한다. one-file self-extraction은 시작 시간, 임시 파일, 메모리 판정을 복잡하게 할 수 있다.

### 15.3 외부 배포

사용자 배포 전 별도 작업:

- Developer ID code signing
- hardened runtime/entitlements 검토
- Apple notarization
- sidecar 포함 전체 bundle 서명
- 서명된 clean machine smoke test

이번 1차 진단 MVP의 완료 조건에는 notarization을 넣지 않는다.

## 16. 위험과 대응

### 위험: Electron 자체의 기본 메모리가 높다

대응:

- 절대 총량만 보지 않고 race-to-race delta와 process별 private memory를 본다.
- Chrome 목표치 400~650MB를 Electron 전체 앱에 그대로 적용하지 않는다.

### 위험: desktop metric 수집이 결과를 오염한다

대응:

- 5초 sampling
- bounded JSONL
- raw state 저장 금지
- 진단 on/off 비교 한 번 수행

### 위험: Python sidecar가 orphan으로 남는다

대응:

- PID와 creation time 기록
- main 종료 경로에서 await
- child `exit`/`error` 감시
- 마지막 fallback만 강제 종료

### 위험: 기존 dirty 변경과 충돌한다

대응:

- desktop 폴더를 우선 신규 추가
- 기존 파일 변경 전 diff 확인
- SC/physics 변경과 desktop 변경을 섞지 않음
- 관련 없는 formatting 금지

### 위험: WKWebView에서만 메모리가 낮아 원인이 가려진다

대응:

- Electron 수정·검증이 먼저
- Tauri는 동일 로그 schema를 사용하는 비교 대상

## 17. 구현 완료 산출물

최종 보고에 반드시 포함한다.

1. 변경 파일 목록
2. 앱 실행 구조
3. backend sidecar 시작/종료 증거
4. Electron process metric 예시
5. Python diagnostic payload 예시
6. 한 경기와 두 경기 checkpoint 표
7. renderer/GPU/Python 중 증가 소유자 판정
8. old session/engine weakref 결과
9. canvas/WebGL/geometry/texture 결과
10. 테스트·lint·build·package 결과
11. 알려진 한계
12. Tauri 비교 진행 여부와 근거

## 18. 공식 참고 문서

- Electron process metrics: <https://www.electronjs.org/docs/latest/api/app>
- Electron process memory: <https://www.electronjs.org/docs/latest/api/process>
- Electron performance: <https://www.electronjs.org/docs/latest/tutorial/performance>
- Electron security: <https://www.electronjs.org/docs/latest/tutorial/security>
- Electron context isolation: <https://www.electronjs.org/docs/latest/tutorial/context-isolation>
- Electron packaging: <https://www.electronjs.org/docs/latest/tutorial/tutorial-packaging>
- Electron macOS signing: <https://www.electronjs.org/docs/latest/tutorial/code-signing>
- Tauri architecture: <https://v2.tauri.app/concept/architecture/>
- Tauri external sidecar: <https://v2.tauri.app/develop/sidecar/>
- Tauri platform WebView: <https://v2.tauri.app/reference/webview-versions/>

## 19. 다른 에이전트에 전달할 실행 프롬프트

아래 프롬프트를 새 작업에 그대로 전달할 수 있다.

```text
저장소 `/Users/kimyongjin/Desktop/f1`에서 macOS Electron 진단 앱을 구현해줘.

작업을 시작하기 전에 다음 문서를 끝까지 읽고 지시를 따라라.

1. `/Users/kimyongjin/Desktop/f1/docs/SIMULATION_FOUNDATION.md`
2. `/Users/kimyongjin/Desktop/f1/docs/CURRENT_PROJECT_STATUS.md`
3. `/Users/kimyongjin/Desktop/f1/docs/RACE_LIFECYCLE_AND_SC_TAIL_FIX_DIRECTIVE.md`
4. `/Users/kimyongjin/Desktop/f1/docs/MACOS_DESKTOP_DIAGNOSTIC_APP_PLAN.md`

핵심 목표:

- 기존 React/Three.js frontend와 FastAPI/Python 시뮬레이션을 유지한다.
- Electron main process가 Python sidecar를 시작하고 health 확인 후 production frontend를 연다.
- 앱 종료 시 active race session, WebSocket, diagnostic log, Python child process를 순서대로 정리한다.
- Electron Browser/Renderer/GPU process memory, renderer JS/Blink/WebGL 지표, Python RSS/session/cache 지표를 5초 주기로 같은 JSONL에 기록한다.
- `New Race`와 `Exit`가 동일한 disposal 경로를 사용하게 하고 setup에서 active session false, canvas 0, loop/client/buffer 0을 증명한다.
- 바레인 57랩 20대 두 경기 연속 soak를 실행할 수 있는 진단 기반을 만든다.

중요 제약:

- 현재 작업 트리는 dirty 상태이며 모든 기존 변경은 사용자 소유다. 관련 없는 파일을 되돌리거나 정리하지 마라.
- SC 65km/h 물리 로직은 이번 데스크톱 포장 작업 범위가 아니다.
- 물리 50Hz, pose 30Hz, 렌더 60FPS, 1배속·2배속을 유지하라.
- 메모리 수치를 낮추기 위해 기능이나 필요한 기록을 삭제하지 마라.
- Electron renderer에서 `nodeIntegration`을 켜지 마라. `contextIsolation`과 sandbox를 사용하고 preload에는 좁은 진단 API만 노출하라.
- backend는 `127.0.0.1`에만 bind하고 desktop 전용 endpoint는 실행별 임의 token으로 보호하라.
- 진단 log는 bounded JSONL로 만들고 raw pose나 전체 dashboard snapshot을 저장하지 마라.
- 별도 요청이 없으므로 커밋과 푸시는 하지 마라.

구현 순서:

1. 기존 빌드·실행·cleanup 경로와 git diff를 확인한다.
2. `desktop/`에 최소 Electron 셸과 sidecar lifecycle을 구현한다.
3. FastAPI lifespan, desktop diagnostic endpoint, session diagnostic counts를 최소 범위로 추가한다.
4. renderer performance stats와 WebSocket/Three.js disposal acknowledgement를 좁은 preload IPC로 연결한다.
5. process별 metric sampler와 log rotation을 구현한다.
6. backend, frontend, Electron 단위 테스트를 추가한다.
7. production frontend build, Electron package, smoke test를 실행한다.
8. 가능하면 바레인 두 경기 soak를 실행하고, 불가능하면 정확한 blocker와 재현 절차를 남긴다.

필수 검증:

- backend 전체 unittest
- frontend lint
- frontend production build
- desktop tests
- Electron package
- `git diff --check`
- 앱 종료 후 Python orphan process가 없는지 확인
- old session/engine weak reference 수거 확인

최종 보고 형식:

1. 구현 결과 요약
2. 변경 파일
3. 아키텍처와 lifecycle
4. 실제 metric/log 예시
5. 한 경기·두 경기 메모리 표
6. cleanup 및 객체 수거 결과
7. 테스트/build/package 결과
8. 남은 문제와 Tauri 비교 권장 여부

계획서와 실제 환경이 충돌하면 임의로 범위를 넓히지 말고, 안전한 read-only 확인을 먼저 수행한 뒤 충돌과 선택지를 명확히 보고하라.
```
