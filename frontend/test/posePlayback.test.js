import test from 'node:test';
import assert from 'node:assert/strict';
import {
  appendPoseTickToBuffers,
  bufferedWorldPoseAtTime,
} from '../src/components/TrackView/posePlayback.js';
import {
  advanceSafetyCarRenderProgress,
  safetyCarProgressAtRenderTime,
} from '../src/components/TrackView/safetyCarPlayback.js';

function binaryTick(samples) {
  const buffer = new ArrayBuffer(11 + 4 + samples.length * 20);
  const view = new DataView(buffer);
  view.setUint8(0, 0x46);
  view.setUint8(1, 0x31);
  view.setUint8(2, 0x50);
  view.setUint8(3, 0x31);
  view.setUint32(4, samples.at(-1)[1], true);
  view.setUint8(8, 1);
  view.setUint8(9, 0);
  view.setUint8(10, 1);
  view.setUint16(11, 7, true);
  view.setUint8(13, 0);
  view.setUint8(14, samples.length);
  samples.forEach(([time, frame, x, y, heading], index) => {
    const offset = 15 + index * 20;
    view.setFloat32(offset, time, true);
    view.setUint32(offset + 4, frame, true);
    view.setFloat32(offset + 8, x, true);
    view.setFloat32(offset + 12, y, true);
    view.setFloat32(offset + 16, heading, true);
  });
  return { physics_frame: samples.at(-1)[1], pose_buffer: buffer };
}

test('binary pose packets use bounded typed-array buffers and interpolate', () => {
  const buffers = new Map();
  const latest = appendPoseTickToBuffers(binaryTick([
    [0, 1, 0, 0, 0],
    [0.02, 2, 1, 0, 0],
  ]), buffers);
  assert.ok(Math.abs(latest - 0.02) < 1e-5);
  const pose = bufferedWorldPoseAtTime(buffers.get(7), 0.01);
  assert.ok(Math.abs(pose.xM - 0.5) < 0.01);

  for (let frame = 3; frame <= 220; frame += 1) {
    appendPoseTickToBuffers(binaryTick([[frame * 0.02, frame, frame, 0, 0]]), buffers);
  }
  assert.ok(buffers.get(7).count >= 125);
  assert.ok(buffers.get(7).count <= 160);
  assert.equal(buffers.get(7).capacity, 160);
});

test('retired vehicles without active hazards release their pose buffer', () => {
  const buffers = new Map();
  appendPoseTickToBuffers(binaryTick([[0, 1, 1, 2, 0]]), buffers);
  const retired = new ArrayBuffer(15 + 20);
  const view = new DataView(retired);
  view.setUint8(10, 1);
  view.setUint16(11, 7, true);
  view.setUint8(13, 1);
  view.setUint8(14, 1);
  view.setFloat32(15, 0, true);
  view.setUint32(19, 2, true);
  appendPoseTickToBuffers({ pose_buffer: retired }, buffers);
  assert.equal(buffers.has(7), false);
});

test('Safety Car track extrapolation follows simulation speed at 2x', () => {
  const oneX = safetyCarProgressAtRenderTime({
    route: 'track',
    progress: 0.25,
    progressRate: 0.02,
    elapsedWallSeconds: 0.25,
    speedMultiplier: 1,
  });
  const twoX = safetyCarProgressAtRenderTime({
    route: 'track',
    progress: 0.25,
    progressRate: 0.02,
    elapsedWallSeconds: 0.25,
    speedMultiplier: 2,
  });

  assert.ok(Math.abs(oneX - 0.255) < 1e-9);
  assert.ok(Math.abs(twoX - 0.26) < 1e-9);
});

test('Safety Car pit extrapolation does not double-scale measured wall rate', () => {
  const progress = safetyCarProgressAtRenderTime({
    route: 'pit',
    pitLaneProgress: 0.4,
    pitLaneProgressRate: 0.2,
    elapsedWallSeconds: 0.25,
    speedMultiplier: 2,
  });

  assert.ok(Math.abs(progress - 0.45) < 1e-9);
});

test('Safety Car render correction cannot jump across a delayed packet', () => {
  const progress = advanceSafetyCarRenderProgress({
    currentProgress: 0.25,
    desiredProgress: 0.30,
    route: 'track',
    progressRate: 0.02,
    elapsedWallSeconds: 1 / 60,
    speedMultiplier: 2,
  });

  assert.ok(progress > 0.25);
  assert.ok(progress < 0.30);
});
