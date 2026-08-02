# F1 Race Manager macOS diagnostic shell

개발 실행은 저장소의 production frontend와 `backend/.venv` sidecar를 사용한다.

```bash
cd desktop
npm install
npm run build:frontend
npm start
```

macOS arm64 진단용 `.app`는 다음 명령으로 만든다. frontend, backend/data, one-directory Python runtime은 앱의 `Contents/Resources` 아래에 외부 resource로 포함된다.

```bash
npm run package
```

실행별 bounded JSONL은 Electron `userData/diagnostics/`에 생성된다. 스키마 2부터 Electron `app.getAppMetrics()`의 KiB 값을 `working_set_bytes`로 변환해 Browser/Tab/GPU/Utility process별 및 합계로 기록한다. macOS에서는 Electron의 `privateBytes`가 지원되지 않으므로 `private_memory_supported=false`와 working-set 기준을 명시한다. Renderer snapshot, Python sidecar RSS/CPU/session/cache count와 Electron+Python 합계도 함께 기록되며 raw pose와 전체 dashboard snapshot은 기록하지 않는다.

진단 soak는 앱에서 Bahrain 57랩·20대를 시작하고 `Exit` 또는 결과 화면의 `New Race`를 누른 뒤 setup 상태에서 10/30/60초를 기다려 같은 앱에서 두 번째 경기를 시작하는 방식으로 실행한다. 현재 저장소의 `backend/tools/run_bahrain_runtime_soak.py`는 짧은 1x/2x cadence 검증용이며, 57랩 두 경기의 실제 시간은 별도 장시간 실행으로 기록해야 한다.
