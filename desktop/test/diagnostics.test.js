const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const {
  BoundedJsonlWriter,
  buildSetupCheckpointDetails,
  DiagnosticRecorder,
  kibToBytes,
  mapElectronMetric,
  rendererDisposedSnapshot,
  sanitizeValue,
  summarizeElectronMetrics,
  validateRendererSnapshot,
} = require('../src/diagnostics');

test('renderer snapshot validation keeps only bounded scalar/object data', () => {
  const snapshot = validateRendererSnapshot({ fps: 60, poseSamples: 12, rawPose: 'x'.repeat(1000) });
  assert.equal(snapshot.fps, 60);
  assert.equal(snapshot.rawPose.length, 160);
  assert.equal(validateRendererSnapshot(null), null);
});

test('nested Electron process metrics survive sanitization and KiB become bytes', () => {
  const safe = sanitizeValue({
    electron: {
      processes: [{ pid: 42, cpu_percent: 12.5, working_set_bytes: 1024 }],
    },
  });
  assert.equal(safe.electron.processes[0].pid, 42);
  assert.equal(safe.electron.processes[0].cpu_percent, 12.5);
  assert.equal(safe.electron.processes[0].working_set_bytes, 1024);
  assert.equal(kibToBytes(1024), 1024 * 1024);

  const metric = mapElectronMetric({
    pid: 42,
    type: 'Tab',
    cpu: { percent: 12.5 },
    memory: { workingSetSize: 1024, peakWorkingSetSize: 2048, privateBytes: 99 },
  });
  assert.equal(metric.working_set_kib, 1024);
  assert.equal(metric.working_set_bytes, 1024 * 1024);
  assert.equal(metric.peak_working_set_bytes, 2048 * 1024);
  assert.equal(metric.cpu_percent, 12.5);
  if (process.platform === 'darwin') assert.equal(metric.private_memory_supported, false);

  const aggregate = summarizeElectronMetrics([
    { cpu_percent: 1.25, working_set_bytes: 1024, peak_working_set_bytes: 2048 },
    { cpu_percent: 2.75, working_set_bytes: 2048, peak_working_set_bytes: 4096 },
  ]);
  assert.equal(aggregate.cpu_percent, 4);
  assert.equal(aggregate.working_set_bytes, 3072);
  assert.equal(aggregate.peak_working_set_bytes, 6144);
});

test('renderer disposal snapshot clears live values instead of retaining the race snapshot', () => {
  const snapshot = rendererDisposedSnapshot({
    renderer_lifecycle_id: 'renderer-test',
    animation_loop_stopped: true,
    pose_buffers_cleared: true,
    scene_refs_cleared: true,
    renderer_ref_cleared: true,
  });
  assert.equal(snapshot.renderer_active, false);
  assert.equal(snapshot.websocket_count, 0);
  assert.equal(snapshot.canvas_count, 0);
  assert.equal(snapshot.geometries, 0);
  assert.equal(snapshot.textures, 0);
  assert.equal(snapshot.pose_samples, 0);
  assert.equal(snapshot.js_heap_current_bytes, null);
  assert.equal(snapshot.renderer_lifecycle_id, 'renderer-test');
});

test('setup checkpoint evidence is measured from sampler values', () => {
  const details = buildSetupCheckpointDetails({
    python: {
      active_session: false,
      client_count: 0,
      loop_task_exists: false,
      trajectory_sample_count: 0,
    },
    renderer: {
      renderer_active: false,
      canvas_count: 0,
      websocket_count: 0,
      pose_buffers_cleared: true,
      scene_refs_cleared: true,
      renderer_ref_cleared: true,
    },
  });
  assert.deepEqual(details, {
    active_session: false,
    client_count: 0,
    loop_task_exists: false,
    trajectory_sample_count: 0,
    canvas_count: 0,
    websocket_count: 0,
    renderer_disposal_acknowledged: true,
  });
});

test('sample JSONL preserves numeric Electron metrics and app totals', async () => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'f1-recorder-'));
  const recorder = new DiagnosticRecorder({
    userDataPath: directory,
    appSessionId: 'sample-test',
    backend: { baseUrl: 'http://127.0.0.1:1', token: 'token' },
    app: {
      getAppMetrics: () => [{
        pid: 42,
        type: 'Tab',
        cpu: { percent: 3.5 },
        memory: { workingSetSize: 100, peakWorkingSetSize: 120 },
      }],
    },
    requestJsonImpl: async () => ({
      process: { rss_bytes: 200 * 1024, cpu_percent: 4.5 },
      active_session: false,
    }),
  });
  const sample = await recorder.sampleNow();
  recorder.close();

  const line = fs.readFileSync(recorder.getLogLocation(), 'utf8').trim();
  const record = JSON.parse(line);
  assert.equal(record.schema_version, 2);
  assert.equal(record.electron.processes[0].working_set_bytes, 100 * 1024);
  assert.equal(record.electron.aggregate.cpu_percent, 3.5);
  assert.equal(record.app_total.cpu_percent, 8);
  assert.equal(record.app_total.memory_bytes, 300 * 1024);
  assert.equal(sample.app_total.memory_basis, 'electron_working_set_plus_python_rss');
});

test('JSONL writer rotates files and retains a bounded set', () => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'f1-diag-'));
  const writer = new BoundedJsonlWriter(directory, {
    appSessionId: 'test', maxFileBytes: 80, maxFiles: 2, maxTotalBytes: 160,
  });
  for (let index = 0; index < 12; index += 1) writer.write({ index, payload: 'bounded' });
  const files = fs.readdirSync(directory).filter((name) => name.endsWith('.jsonl'));
  assert.ok(files.length <= 2);
  assert.ok(files.every((name) => fs.statSync(path.join(directory, name)).size <= 160));
});
