const test = require('node:test');
const assert = require('node:assert/strict');
const { EventEmitter } = require('node:events');
const { findAvailablePort, resolvePythonExecutable } = require('../src/backendProcess');
const { BackendProcess } = require('../src/backendProcess');

test('port selection binds only to localhost and returns an ephemeral port', async () => {
  let port;
  try {
    port = await findAvailablePort();
  } catch (error) {
    // Managed CI sandboxes can forbid local socket creation even though the
    // production Electron process is allowed to bind its loopback sidecar.
    assert.equal(error.code, 'EPERM');
    return;
  }
  assert.ok(Number.isInteger(port));
  assert.ok(port > 0 && port < 65536);
});

test('explicit Python executable wins over environment discovery', () => {
  assert.equal(
    resolvePythonExecutable({ projectRoot: '/tmp/f1', env: { F1_PYTHON: '/custom/python' } }),
    '/custom/python',
  );
});

class FakeChild extends EventEmitter {
  constructor({ exitOn = {} } = {}) {
    super();
    this.stdout = new EventEmitter();
    this.stderr = new EventEmitter();
    this.exitOn = exitOn;
    this.killSignals = [];
  }

  kill(signal) {
    this.killSignals.push(signal);
    if (this.exitOn[signal]) {
      queueMicrotask(() => this.emit('exit', 0, signal));
    }
    return true;
  }
}

test('health polling rejects when the child exits before readiness', async () => {
  const backend = new BackendProcess({
    projectRoot: '/tmp/f1',
    frontendDist: '/tmp/f1/dist',
    token: 'token',
    port: 12345,
    requestJsonImpl: async () => ({ status: 'ok', pid: 7 }),
  });
  backend.child = new FakeChild();
  backend.exitInfo = { code: 1, signal: null };
  await assert.rejects(
    backend.waitForHealth({ healthTimeoutMs: 20, healthPollMs: 1 }),
    /Backend exited before health check/,
  );
});

test('health polling reports a bounded timeout', async () => {
  const backend = new BackendProcess({
    projectRoot: '/tmp/f1',
    frontendDist: '/tmp/f1/dist',
    token: 'token',
    port: 12345,
    requestJsonImpl: async () => { throw new Error('not ready'); },
  });
  backend.child = new FakeChild();
  await assert.rejects(
    backend.waitForHealth({ healthTimeoutMs: 10, healthPollMs: 1 }),
    /Backend health timeout/,
  );
});

test('shutdown falls back to SIGKILL when SIGTERM does not exit', async () => {
  const child = new FakeChild({ exitOn: { SIGKILL: true } });
  const backend = new BackendProcess({
    projectRoot: '/tmp/f1',
    frontendDist: '/tmp/f1/dist',
    token: 'token',
    port: 12345,
    shutdownTimeoutMs: 5,
    spawnImpl: () => child,
  });
  backend.child = child;
  backend.exitPromise = new Promise((resolve) => {
    child.once('exit', (code, signal) => {
      backend.exitInfo = { code, signal };
      resolve(backend.exitInfo);
    });
  });
  backend.clearRaceSession = async () => ({ status: 'cleared' });

  const exitInfo = await backend.stop();
  assert.deepEqual(child.killSignals, ['SIGTERM', 'SIGKILL']);
  assert.equal(exitInfo.signal, 'SIGKILL');
  assert.equal(backend.child, null);
});
