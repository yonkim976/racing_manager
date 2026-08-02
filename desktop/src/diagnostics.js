const fs = require('node:fs');
const path = require('node:path');
const { requestJson } = require('./backendProcess');

const SCHEMA_VERSION = 2;
const DEFAULT_MAX_FILE_BYTES = 2 * 1024 * 1024;
const DEFAULT_MAX_FILES = 20;
const DEFAULT_MAX_TOTAL_BYTES = 200 * 1024 * 1024;

function clampString(value, maxLength = 160) { return String(value).slice(0, maxLength); }

function sanitizeValue(value, depth = 0) {
  // Electron process metrics are nested as record -> electron -> processes ->
  // process -> field. Keep enough depth for those scalar fields while still
  // bounding arbitrary diagnostic payloads.
  if (depth > 6) return '[truncated]';
  if (value === null || typeof value === 'boolean' || typeof value === 'number') return value;
  if (typeof value === 'string') return clampString(value);
  if (Array.isArray(value)) return value.slice(0, 64).map((item) => sanitizeValue(item, depth + 1));
  if (typeof value === 'object') {
    return Object.fromEntries(Object.entries(value).slice(0, 80).map(([key, item]) => [
      clampString(key, 80), sanitizeValue(item, depth + 1),
    ]));
  }
  return undefined;
}

function validateRendererSnapshot(snapshot) {
  if (!snapshot || typeof snapshot !== 'object' || Array.isArray(snapshot)) return null;
  const safe = sanitizeValue(snapshot);
  return safe && typeof safe === 'object' && !Array.isArray(safe) ? safe : null;
}

class BoundedJsonlWriter {
  constructor(directory, {
    appSessionId,
    maxFileBytes = DEFAULT_MAX_FILE_BYTES,
    maxFiles = DEFAULT_MAX_FILES,
    maxTotalBytes = DEFAULT_MAX_TOTAL_BYTES,
    now = () => new Date(),
  } = {}) {
    this.directory = directory;
    this.appSessionId = appSessionId;
    this.maxFileBytes = maxFileBytes;
    this.maxFiles = maxFiles;
    this.maxTotalBytes = maxTotalBytes;
    this.now = now;
    this.filePath = null;
    this.bytesWritten = 0;
    this.fileSequence = 0;
    fs.mkdirSync(directory, { recursive: true });
    this._openFile();
  }

  _openFile() {
    const stamp = this.now().toISOString().replace(/[:.]/g, '-');
    this.filePath = path.join(
      this.directory,
      `${stamp}-${this.appSessionId}-${this.fileSequence}.jsonl`,
    );
    this.fileSequence += 1;
    this.bytesWritten = 0;
  }

  _files() {
    return fs.readdirSync(this.directory)
      .filter((name) => name.endsWith('.jsonl'))
      .map((name) => {
        const filePath = path.join(this.directory, name);
        const stat = fs.statSync(filePath);
        return { filePath, size: stat.size, mtimeMs: stat.mtimeMs };
      })
      .sort((left, right) => left.mtimeMs - right.mtimeMs);
  }

  _rotateIfNeeded(nextBytes) {
    if (this.bytesWritten > 0 && this.bytesWritten + nextBytes > this.maxFileBytes) this._openFile();
    let files = this._files();
    let totalBytes = files.reduce((total, file) => total + file.size, 0);
    while (files.length > this.maxFiles || totalBytes > this.maxTotalBytes) {
      const oldest = files.shift();
      if (!oldest || oldest.filePath === this.filePath) break;
      fs.rmSync(oldest.filePath, { force: true });
      totalBytes -= oldest.size;
    }
  }

  write(record) {
    const line = `${JSON.stringify(sanitizeValue(record))}\n`;
    const bytes = Buffer.byteLength(line);
    this._rotateIfNeeded(bytes);
    fs.appendFileSync(this.filePath, line, 'utf8');
    this.bytesWritten += bytes;
    return this.filePath;
  }

  flush() {}
  close() {}
}

function kibToBytes(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? Math.round(numeric * 1024) : null;
}

function mapElectronMetric(metric) {
  const memory = metric.memory || {};
  const processType = metric.type || metric.processType || 'unknown';
  const classifiedType = processType === 'Browser'
    ? 'Browser/main'
    : processType === 'Tab'
      ? 'Tab/renderer'
      : processType === 'GPU'
        ? 'GPU'
        : processType === 'Utility' || metric.serviceName
          ? 'Utility/network'
          : 'unknown';
  return {
    pid: metric.pid,
    creation_time: metric.creationTime,
    process_type: processType,
    classified_type: classifiedType,
    service_name: metric.serviceName || null,
    cpu_percent: metric.cpu?.percent ?? metric.cpuPercent ?? null,
    memory_unit: 'bytes',
    memory_source: 'app.getAppMetrics.memory',
    private_memory_supported: process.platform === 'win32',
    private_kib: process.platform === 'win32' ? (memory.privateBytes ?? null) : null,
    private_bytes: process.platform === 'win32' ? kibToBytes(memory.privateBytes) : null,
    working_set_kib: memory.workingSetSize ?? null,
    working_set_bytes: kibToBytes(memory.workingSetSize),
    peak_working_set_kib: memory.peakWorkingSetSize ?? null,
    peak_working_set_bytes: kibToBytes(memory.peakWorkingSetSize),
    shared_kib: memory.sharedBytes ?? memory.shared ?? null,
    shared_bytes: kibToBytes(memory.sharedBytes ?? memory.shared),
    sandboxed: metric.sandboxed ?? null,
  };
}

function rendererDisposedSnapshot(details = {}) {
  return {
    renderer_lifecycle_id: details.renderer_lifecycle_id || null,
    renderer_active: false,
    react_phase: 'setup',
    websocket_state: 'closed',
    websocket_count: 0,
    js_heap_current_bytes: null,
    js_heap_peak_bytes: null,
    js_heap_trend_mb_per_min: null,
    blink_allocated_bytes: null,
    dom_nodes: null,
    canvas_count: 0,
    active_webgl_contexts: 'not_observable',
    geometries: 0,
    textures: 0,
    render_calls: 0,
    triangles: 0,
    scene_builds: null,
    garage_builds: null,
    car_models_built: null,
    pose_samples: 0,
    race_end_overlay: false,
    ...details,
    renderer_active: false,
    react_phase: 'setup',
    websocket_state: 'closed',
    websocket_count: 0,
    js_heap_current_bytes: null,
    js_heap_peak_bytes: null,
    js_heap_trend_mb_per_min: null,
    blink_allocated_bytes: null,
    dom_nodes: null,
    canvas_count: 0,
    active_webgl_contexts: 'not_observable',
    geometries: 0,
    textures: 0,
    render_calls: 0,
    triangles: 0,
    scene_builds: null,
    garage_builds: null,
    car_models_built: null,
    pose_samples: 0,
    race_end_overlay: false,
  };
}

function numericOrNull(value) {
  return typeof value === 'number' && Number.isFinite(value) ? value : null;
}

function sumNumbers(values) {
  const numbers = values.map(numericOrNull).filter((value) => value !== null);
  return numbers.length ? numbers.reduce((total, value) => total + value, 0) : null;
}

function summarizeElectronMetrics(processes) {
  return {
    process_count: processes.length,
    cpu_percent: sumNumbers(processes.map((processMetric) => processMetric.cpu_percent)),
    working_set_bytes: sumNumbers(
      processes.map((processMetric) => processMetric.working_set_bytes),
    ),
    peak_working_set_bytes: sumNumbers(
      processes.map((processMetric) => processMetric.peak_working_set_bytes),
    ),
    memory_basis: 'working_set_bytes',
  };
}

function buildSetupCheckpointDetails(sample) {
  const python = sample?.python;
  const renderer = sample?.renderer;
  const pythonEvidence = [
    python?.active_session,
    python?.client_count,
    python?.loop_task_exists,
    python?.trajectory_sample_count,
  ];
  const rendererEvidence = [
    renderer?.renderer_active,
    renderer?.canvas_count,
    renderer?.websocket_count,
    renderer?.pose_buffers_cleared,
    renderer?.scene_refs_cleared,
    renderer?.renderer_ref_cleared,
  ];
  const evidenceKnown = [...pythonEvidence, ...rendererEvidence]
    .every((value) => value !== undefined && value !== null);
  const rendererDisposalAcknowledged = evidenceKnown
    ? python.active_session === false
      && python.client_count === 0
      && python.loop_task_exists === false
      && python.trajectory_sample_count === 0
      && renderer.renderer_active === false
      && renderer.canvas_count === 0
      && renderer.websocket_count === 0
      && renderer.pose_buffers_cleared === true
      && renderer.scene_refs_cleared === true
      && renderer.renderer_ref_cleared === true
    : null;
  return {
    active_session: typeof python?.active_session === 'boolean'
      ? python.active_session : null,
    client_count: numericOrNull(python?.client_count),
    loop_task_exists: typeof python?.loop_task_exists === 'boolean'
      ? python.loop_task_exists : null,
    trajectory_sample_count: numericOrNull(python?.trajectory_sample_count),
    canvas_count: numericOrNull(renderer?.canvas_count),
    websocket_count: numericOrNull(renderer?.websocket_count),
    renderer_disposal_acknowledged: rendererDisposalAcknowledged,
  };
}

class DiagnosticRecorder {
  constructor({
    userDataPath,
    appSessionId,
    backend,
    app,
    intervalMs = 5_000,
    requestJsonImpl = requestJson,
  } = {}) {
    this.appSessionId = appSessionId;
    this.backend = backend;
    this.app = app;
    this.intervalMs = intervalMs;
    this.requestJsonImpl = requestJsonImpl;
    this.lifecycle = 'BOOTING';
    this.raceSessionId = null;
    this.raceIndex = 0;
    this.rendererSnapshot = null;
    this.samplerTimer = null;
    this.writer = new BoundedJsonlWriter(path.join(userDataPath, 'diagnostics'), { appSessionId });
  }

  setLifecycle(lifecycle) { this.lifecycle = lifecycle; }
  setRaceContext({ raceSessionId = null, raceIndex = this.raceIndex } = {}) {
    this.raceSessionId = raceSessionId;
    this.raceIndex = raceIndex;
  }

  _common(recordType) {
    return {
      schema_version: SCHEMA_VERSION,
      timestamp: new Date().toISOString(),
      monotonic_ms: Math.round(process.uptime() * 1000),
      app_session_id: this.appSessionId,
      race_session_id: this.raceSessionId,
      race_index: this.raceIndex,
      record_type: recordType,
      lifecycle: this.lifecycle,
    };
  }

  write(recordType, fields = {}) { return this.writer.write({ ...this._common(recordType), ...fields }); }
  recordCheckpoint(name, details = {}) { return this.write('checkpoint', { name, details }); }
  recordLifecycle(from, to, details = {}) {
    this.setLifecycle(to);
    return this.write('lifecycle', { from, to, details });
  }
  recordError(error, details = {}) {
    return this.write('error', { error: clampString(error?.message || error), details });
  }
  setRendererSnapshot(snapshot) { this.rendererSnapshot = validateRendererSnapshot(snapshot); }

  async sampleNow() {
    const electronProcesses = this.app?.getAppMetrics?.().map(mapElectronMetric) || [];
    const electron = {
      processes: electronProcesses,
      aggregate: summarizeElectronMetrics(electronProcesses),
    };
    let python = null;
    try {
      python = await this.requestJsonImpl(this.backend.baseUrl, '/api/desktop/diagnostics', {
        token: this.backend.token,
        timeoutMs: 1_500,
      });
    } catch (error) {
      python = { unavailable: true, error: clampString(error.message) };
    }
    const sample = {
      electron,
      renderer: this.rendererSnapshot,
      python,
      app_total: {
        cpu_percent: sumNumbers([
          electron.aggregate.cpu_percent,
          python?.process?.cpu_percent,
        ]),
        memory_bytes: sumNumbers([
          electron.aggregate.working_set_bytes,
          python?.process?.rss_bytes,
        ]),
        memory_basis: 'electron_working_set_plus_python_rss',
      },
    };
    this.write('sample', sample);
    return sample;
  }

  start() {
    if (this.samplerTimer) return;
    this.samplerTimer = setInterval(() => {
      this.sampleNow().catch((error) => this.recordError(error, { source: 'sampler' }));
    }, this.intervalMs);
    this.samplerTimer.unref?.();
  }
  stop() { if (this.samplerTimer) clearInterval(this.samplerTimer); this.samplerTimer = null; }
  getLogLocation() { return this.writer.filePath; }
  close() { this.stop(); this.writer.flush(); this.writer.close(); }
}

module.exports = {
  BoundedJsonlWriter,
  DiagnosticRecorder,
  SCHEMA_VERSION,
  buildSetupCheckpointDetails,
  kibToBytes,
  mapElectronMetric,
  rendererDisposedSnapshot,
  sanitizeValue,
  summarizeElectronMetrics,
  validateRendererSnapshot,
};
