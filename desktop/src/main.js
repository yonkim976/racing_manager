const { app, BrowserWindow, dialog, ipcMain } = require('electron');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

const { BackendProcess, findAvailablePort } = require('./backendProcess');
const {
  DiagnosticRecorder,
  buildSetupCheckpointDetails,
  rendererDisposedSnapshot,
  validateRendererSnapshot,
} = require('./diagnostics');
const { LifecycleController } = require('./lifecycle');

const PROJECT_ROOT = path.resolve(__dirname, '..', '..');
const PRELOAD_PATH = path.join(__dirname, 'preload.js');
let mainWindow = null;
let backend = null;
let diagnostics = null;
let lifecycle = new LifecycleController();
let shutdownPromise = null;
let allowWindowClose = false;
let setupCheckpointTimers = [];

function pathsForRuntime() {
  const packaged = app.isPackaged;
  return {
    packaged,
    frontendDist: packaged
      ? path.join(process.resourcesPath, 'frontend-dist')
      : path.join(PROJECT_ROOT, 'frontend', 'dist'),
  };
}

function assertFrontendDist(frontendDist) {
  if (!fs.existsSync(path.join(frontendDist, 'index.html'))) {
    throw new Error(`Production frontend not found: ${frontendDist}. Run npm --prefix frontend run build.`);
  }
}

function transitionLifecycle(nextState, details = {}) {
  const previous = lifecycle.state;
  if (previous === nextState) return;
  lifecycle.transition(nextState);
  diagnostics?.recordLifecycle(previous, nextState, details);
}

function clearSetupCheckpointTimers() {
  setupCheckpointTimers.forEach((timer) => clearTimeout(timer));
  setupCheckpointTimers = [];
}

function scheduleSetupCheckpoints() {
  clearSetupCheckpointTimers();
  [10, 30, 60].forEach((seconds) => {
    const timer = setTimeout(() => {
      void (async () => {
        const sample = await diagnostics?.sampleNow();
        diagnostics?.recordCheckpoint(
          `setup_after_${seconds}s`,
          buildSetupCheckpointDetails(sample),
        );
      })().catch((error) => diagnostics?.recordError(error, {
        source: `setup_after_${seconds}s`,
      }));
    }, seconds * 1000);
    timer.unref?.();
    setupCheckpointTimers.push(timer);
  });
}

function installIpcHandlers() {
  ipcMain.handle('desktop:renderer-snapshot', (_event, snapshot) => {
    const safe = validateRendererSnapshot(snapshot);
    if (!safe) throw new Error('Invalid renderer diagnostic snapshot');
    diagnostics?.setRendererSnapshot(safe);
    return { accepted: true };
  });
  ipcMain.handle('desktop:checkpoint', async (_event, payload) => {
    const name = String(payload?.name || '').slice(0, 80);
    if (!name || !diagnostics) return { accepted: false };
    const details = payload?.details || {};
    if (name === 'race_started') {
      clearSetupCheckpointTimers();
      diagnostics.setRaceContext({
        raceSessionId: details.session_id || null,
        raceIndex: Number(details.race_index || 0),
      });
      if (lifecycle.state === 'IDLE') transitionLifecycle('RACING', details);
    } else if (name === 'race_finished' && lifecycle.state === 'RACING') {
      transitionLifecycle('RESULTS', details);
    } else if (name === 'race_disposal_started' && lifecycle.state !== 'DISPOSING_RACE') {
      transitionLifecycle('DISPOSING_RACE', details);
    } else if (name === 'renderer_disposed') {
      const disposalBarrier = lifecycle.state === 'DISPOSING_RACE';
      if (disposalBarrier) transitionLifecycle('IDLE', details);
      diagnostics.setRendererSnapshot(rendererDisposedSnapshot(details));
    }
    diagnostics.recordCheckpoint(name, details);
    if (name === 'renderer_disposed' && lifecycle.state === 'IDLE') {
      diagnostics.setRaceContext({ raceSessionId: null });
      const sample = await diagnostics.sampleNow();
      diagnostics.recordCheckpoint('setup_idle', buildSetupCheckpointDetails(sample));
      scheduleSetupCheckpoints();
    }
    return { accepted: true };
  });
  ipcMain.handle('desktop:lifecycle-state', () => lifecycle.state);
  ipcMain.handle('desktop:log-location', () => diagnostics?.getLogLocation() || null);
}

function createWindow(url) {
  const window = new BrowserWindow({
    width: 1600,
    height: 1000,
    minWidth: 1100,
    minHeight: 700,
    show: false,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      sandbox: true,
      preload: PRELOAD_PATH,
    },
  });
  window.webContents.setWindowOpenHandler(() => ({ action: 'deny' }));
  window.webContents.on('will-navigate', (event, destination) => {
    if (!destination.startsWith(url)) event.preventDefault();
  });
  window.on('close', (event) => {
    if (allowWindowClose) return;
    event.preventDefault();
    void shutdown('window_close').then(() => {
      allowWindowClose = true;
      window.close();
      app.quit();
    });
  });
  mainWindow = window;
  return window;
}

async function shutdown(reason = 'app_shutdown') {
  if (shutdownPromise) return shutdownPromise;
  shutdownPromise = (async () => {
    const previous = lifecycle.state;
    if (previous !== 'APP_SHUTTING_DOWN') {
      try { transitionLifecycle('APP_SHUTTING_DOWN', { reason }); }
      catch (_) { lifecycle.state = 'APP_SHUTTING_DOWN'; }
    }
    diagnostics?.recordCheckpoint('app_shutdown_requested', { reason });
    clearSetupCheckpointTimers();
    diagnostics?.stop();
    if (backend) {
      const pid = backend.pid;
      const exitInfo = await backend.stop();
      diagnostics?.recordCheckpoint('backend_exited', { pid, exit: exitInfo || null });
      backend = null;
    }
    diagnostics?.close();
  })();
  return shutdownPromise;
}

async function start() {
  app.setName('F1 Race Manager');
  const runtime = pathsForRuntime();
  assertFrontendDist(runtime.frontendDist);
  diagnostics = new DiagnosticRecorder({
    userDataPath: app.getPath('userData'),
    appSessionId: crypto.randomUUID(),
    backend: { baseUrl: '', token: '' },
    app,
  });
  transitionLifecycle('BACKEND_STARTING');

  const port = await findAvailablePort('127.0.0.1');
  const token = crypto.randomBytes(32).toString('hex');
  backend = new BackendProcess({
    projectRoot: PROJECT_ROOT,
    frontendDist: runtime.frontendDist,
    token,
    port,
    packaged: runtime.packaged,
    onOutput: (stream, text) => {
      if (stream === 'stderr' || stream === 'shutdown-error') diagnostics?.write('sidecar', { stream, text });
    },
    onExit: (info) => diagnostics?.write('sidecar', { stream: 'exit', info }),
  });
  diagnostics.backend = backend;
  const health = await backend.start();
  transitionLifecycle('BACKEND_READY', { pid: health.pid, port });
  diagnostics.recordCheckpoint('backend_ready', { pid: health.pid, port });

  const origin = backend.baseUrl;
  const window = createWindow(origin);
  transitionLifecycle('WINDOW_READY');
  await window.loadURL(`${origin}/`);
  transitionLifecycle('IDLE', { origin });
  diagnostics.recordCheckpoint('app_started', { origin });
  await diagnostics.sampleNow();
  diagnostics.start();
  window.show();
}

app.whenReady().then(() => {
  installIpcHandlers();
  return start();
}).catch(async (error) => {
  diagnostics?.recordError(error, { source: 'startup' });
  try { await shutdown('startup_failure'); } catch (_) {}
  await dialog.showMessageBox({
    type: 'error',
    title: 'F1 Race Manager failed to start',
    message: error.message,
  });
  app.exit(1);
});

app.on('before-quit', (event) => {
  if (allowWindowClose || shutdownPromise) return;
  event.preventDefault();
  void shutdown('before_quit').then(() => {
    allowWindowClose = true;
    app.quit();
  });
});

app.on('window-all-closed', () => {
  if (!shutdownPromise) void shutdown('window_all_closed').then(() => app.quit());
});

module.exports = { assertFrontendDist, pathsForRuntime };
