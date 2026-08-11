import test from 'node:test';
import assert from 'node:assert/strict';
import {
  localMetricToRenderPoint,
  renderToLocalMetricPoint,
} from '../src/components/TrackView/coordinateContract.js';

test('metric pose and render track coordinates round-trip through the RaceInfo frame', () => {
  const frame = {
    originXRender: 239.61,
    originYRender: 360.46,
    metersPerRenderUnit: 2.3090140782992954,
  };
  const metric = [123.456, -78.9];
  const render = localMetricToRenderPoint(...metric, frame);
  const restored = renderToLocalMetricPoint(render, frame);

  assert.ok(Math.abs(restored[0] - metric[0]) < 1e-9);
  assert.ok(Math.abs(restored[1] - metric[1]) < 1e-9);
  assert.deepEqual(localMetricToRenderPoint(0, 0, frame), [
    frame.originXRender,
    frame.originYRender,
  ]);
});
