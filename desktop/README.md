# F1 Race Manager macOS 진단 앱

상태: **Electron arm64 진단 런타임 구현 완료**
기준일: **2026-08-03**

현재 React/Three.js frontend와 FastAPI backend를 하나의 macOS 앱 수명주기로 묶는다.
Electron 전환은 메모리 문제를 자동 해결하는 것이 아니라 Renderer/GPU/Python 소유자를
분리 계측하고 레이스 종료 시 자원 해제를 검증하기 위한 런타임 경계다.

## 실행과 검증

```bash
cd desktop
npm install
npm run build:frontend
npm start
```

```bash
cd desktop
npm test
npm run package
```

`npm run package`는 frontend production build, backend 코드·데이터와 one-directory
Python runtime을 `Contents/Resources` 아래에 복사해 macOS arm64 `.app`를 만든다.
추가 resource 복사 뒤 번들 전체에 로컬 ad-hoc 서명을 다시 적용하고 strict 검증한다.
현재 빌드는 진단·로컬 사용용이며 외부 배포용 Developer ID 서명/notarization은 별도다.

## 런타임 계약

- main process가 `127.0.0.1`의 임시 포트를 고르고 256-bit 세션 token을 만든다.
- Python sidecar는 같은 origin에서 정적 frontend와 API/WebSocket을 제공한다.
- health 응답의 PID가 생성한 sidecar와 일치한 뒤에만 창을 연다.
- sidecar는 `PYTHONDONTWRITEBYTECODE=1`로 실행해 서명된 앱 번들 내부에
  `__pycache__`/`.pyc`를 생성하지 않는다. 패키지는 실행 전후 strict code-sign 검증을 유지해야 한다.
- 세션 삭제 API는 desktop token을 요구하며 앱 종료 전에 현재 레이스를 해제한다.
- 정상 종료가 지연되면 `SIGTERM` 뒤 제한 시간 이후 해당 child PID에만 `SIGKILL`한다.
- 브라우저 창은 `nodeIntegration=false`, `contextIsolation=true`, `sandbox=true`이며
  외부 탐색과 새 창을 차단한다.

수명주기는 다음 상태만 허용한다.

```text
BOOTING → BACKEND_STARTING → BACKEND_READY → WINDOW_READY → IDLE
IDLE → RACING → RESULTS → DISPOSING_RACE → IDLE
any active state → APP_SHUTTING_DOWN
```

`New Race`는 renderer disposal 확인 뒤 `IDLE`로 돌아간다. 앱 종료는 checkpoint timer,
진단 recorder, 레이스 세션, sidecar 순서로 정리한다.

## 진단

실행별 bounded JSONL은 Electron `userData/diagnostics/`에 생성된다. 기록 대상은 다음과
같다.

- Electron Browser/Tab/GPU/Utility process별 working set과 합계
- Renderer JS heap, canvas, geometry, texture와 pose sample 수
- Python RSS/CPU, active session, client, loop task와 cache count
- Electron+Python 앱 합계와 레이스/정리 checkpoint

macOS Electron의 `privateBytes`가 지원되지 않으므로
`private_memory_supported=false`를 기록하고 working set 기준임을 명시한다. raw pose와
전체 dashboard snapshot은 로그에 보존하지 않는다.

57랩 soak는 `Exit` 또는 `New Race` 후 setup 상태에서 10/30/60초 checkpoint를 확인하고
같은 앱에서 두 번째 경기를 시작한다. `backend/tools/run_bahrain_runtime_soak.py`는 짧은
cadence 회귀이며 실제 두 경기 장시간 실행을 대신하지 않는다.

## 생성 산출물

다음 경로는 Git에서 제외되며 다시 만들 수 있다.

```text
desktop/out/
desktop/resources/
desktop/.electron-cache/
frontend/dist/
```

패키지 재생성이 필요할 때 `npm run package`를 실행한다. `node_modules`와
`backend/.venv`는 의존성 설치 결과이므로 일반 정리에서는 유지한다.
