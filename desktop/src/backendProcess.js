const { spawn } = require('node:child_process');
const http = require('node:http');
const net = require('node:net');
const os = require('node:os');
const path = require('node:path');

const HEALTH_TIMEOUT_MS = 30_000;
const HEALTH_POLL_MS = 300;
const SHUTDOWN_TIMEOUT_MS = 5_000;
const OUTPUT_LIMIT_BYTES = 64 * 1024;

function findAvailablePort(host = '127.0.0.1') {
  return new Promise((resolve, reject) => {
    const server = net.createServer();
    server.once('error', reject);
    server.listen({ host, port: 0 }, () => {
      const address = server.address();
      const port = typeof address === 'object' && address ? address.port : null;
      server.close(() => {
        if (!port) reject(new Error('Could not select an ephemeral localhost port'));
        else resolve(port);
      });
    });
  });
}

function requestJson(baseUrl, route, { method = 'GET', token, timeoutMs = 2_000 } = {}) {
  const url = new URL(route, baseUrl);
  const headers = { accept: 'application/json' };
  if (token) headers['x-f1-desktop-token'] = token;
  return new Promise((resolve, reject) => {
    const request = http.request(url, { method, headers }, (response) => {
      let body = '';
      response.setEncoding('utf8');
      response.on('data', (chunk) => {
        body += chunk;
        if (body.length > 1024 * 1024) request.destroy(new Error('HTTP response exceeded 1 MiB'));
      });
      response.on('end', () => {
        let parsed = null;
        if (body) {
          try { parsed = JSON.parse(body); } catch (error) { reject(error); return; }
        }
        if (response.statusCode < 200 || response.statusCode >= 300) {
          const error = new Error(`HTTP ${response.statusCode} from ${route}`);
          error.statusCode = response.statusCode;
          error.body = parsed;
          reject(error);
          return;
        }
        resolve(parsed);
      });
    });
    request.setTimeout(timeoutMs, () => request.destroy(new Error(`HTTP timeout for ${route}`)));
    request.on('error', reject);
    request.end();
  });
}

function resolvePythonExecutable({ projectRoot, packaged = false, env = process.env } = {}) {
  if (env.F1_PYTHON) return env.F1_PYTHON;
  if (packaged) {
    const packagedPython = path.join(process.resourcesPath, 'python', 'bin', 'python');
    if (require('node:fs').existsSync(packagedPython)) return packagedPython;
  }
  const candidates = [
    path.join(projectRoot, 'backend', '.venv', 'bin', 'python'),
    path.join(projectRoot, 'backend', '.venv', 'bin', 'python3'),
  ];
  const fs = require('node:fs');
  const existing = candidates.find((candidate) => fs.existsSync(candidate));
  if (existing) return existing;
  return env.PYTHON || (os.platform() === 'win32' ? 'python' : 'python3');
}

function appendBounded(previous, chunk) {
  const next = `${previous}${chunk}`;
  return next.length > OUTPUT_LIMIT_BYTES ? next.slice(-OUTPUT_LIMIT_BYTES) : next;
}

class BackendProcess {
  constructor({
    projectRoot,
    frontendDist,
    token,
    port,
    packaged = false,
    spawnImpl = spawn,
    requestJsonImpl = requestJson,
    shutdownTimeoutMs = SHUTDOWN_TIMEOUT_MS,
    onOutput = () => {},
    onExit = () => {},
  }) {
    this.projectRoot = projectRoot;
    this.frontendDist = frontendDist;
    this.token = token;
    this.port = port;
    this.packaged = packaged;
    this.spawnImpl = spawnImpl;
    this.requestJsonImpl = requestJsonImpl;
    this.shutdownTimeoutMs = shutdownTimeoutMs;
    this.onOutput = onOutput;
    this.onExit = onExit;
    this.child = null;
    this.stdoutTail = '';
    this.stderrTail = '';
    this.exitPromise = null;
    this.exitInfo = null;
  }

  get baseUrl() { return `http://127.0.0.1:${this.port}`; }
  get pid() { return this.child?.pid || null; }

  start({ healthTimeoutMs = HEALTH_TIMEOUT_MS, healthPollMs = HEALTH_POLL_MS } = {}) {
    if (this.child) throw new Error('Backend process already started');
    const python = resolvePythonExecutable({ projectRoot: this.projectRoot, packaged: this.packaged });
    const backendRoot = this.packaged
      ? path.join(process.resourcesPath, 'backend')
      : path.join(this.projectRoot, 'backend');
    const env = {
      ...process.env,
      F1_DESKTOP_MODE: '1',
      F1_DESKTOP_TOKEN: this.token,
      F1_FRONTEND_DIST: this.frontendDist,
      PYTHONUNBUFFERED: '1',
    };
    const child = this.spawnImpl(
      python,
      ['-m', 'uvicorn', 'main:app', '--host', '127.0.0.1', '--port', String(this.port), '--log-level', 'warning'],
      { cwd: backendRoot, env, stdio: ['ignore', 'pipe', 'pipe'] },
    );
    this.child = child;
    this.exitPromise = new Promise((resolve) => {
      child.once('exit', (code, signal) => {
        this.exitInfo = { code, signal };
        this.onExit(this.exitInfo);
        resolve(this.exitInfo);
      });
    });
    child.once('error', (error) => this.onExit({ error: error.message }));
    child.stdout?.on('data', (chunk) => {
      this.stdoutTail = appendBounded(this.stdoutTail, chunk.toString());
      this.onOutput('stdout', chunk.toString().slice(-4096));
    });
    child.stderr?.on('data', (chunk) => {
      this.stderrTail = appendBounded(this.stderrTail, chunk.toString());
      this.onOutput('stderr', chunk.toString().slice(-4096));
    });
    return this.waitForHealth({ healthTimeoutMs, healthPollMs });
  }

  async waitForHealth({ healthTimeoutMs, healthPollMs }) {
    const deadline = Date.now() + healthTimeoutMs;
    let lastError = null;
    while (Date.now() < deadline) {
      if (this.exitInfo) throw new Error(`Backend exited before health check: ${JSON.stringify(this.exitInfo)}`);
      try {
        const health = await this.requestJsonImpl(this.baseUrl, '/api/health', { timeoutMs: 1_000 });
        if (health?.status === 'ok' && health?.pid === this.pid) return health;
        lastError = new Error('Backend health response was incomplete');
      } catch (error) {
        lastError = error;
      }
      await new Promise((resolve) => setTimeout(resolve, healthPollMs));
    }
    throw new Error(`Backend health timeout: ${lastError?.message || 'unknown error'}`);
  }

  async clearRaceSession() {
    if (!this.child) return null;
    return this.requestJsonImpl(this.baseUrl, '/api/race/session', {
      method: 'DELETE', token: this.token, timeoutMs: 3_000,
    });
  }

  async stop() {
    const child = this.child;
    if (!child) return { alreadyStopped: true };
    try { await this.clearRaceSession(); } catch (error) { this.onOutput('shutdown-error', error.message); }
    if (!this.exitInfo) child.kill('SIGTERM');
    const exitInfo = this.exitInfo || await Promise.race([
      this.exitPromise,
      new Promise((resolve) => setTimeout(() => resolve(null), this.shutdownTimeoutMs)),
    ]);
    if (!exitInfo && !this.exitInfo) {
      child.kill('SIGKILL');
      await this.exitPromise;
    }
    this.child = null;
    return this.exitInfo || exitInfo;
  }
}

module.exports = {
  BackendProcess,
  HEALTH_TIMEOUT_MS,
  findAvailablePort,
  requestJson,
  resolvePythonExecutable,
};
