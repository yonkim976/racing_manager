const test = require('node:test');
const assert = require('node:assert/strict');
const { LifecycleController } = require('../src/lifecycle');

test('lifecycle accepts the normal boot path and rejects illegal transitions', () => {
  const lifecycle = new LifecycleController();
  for (const state of ['BACKEND_STARTING', 'BACKEND_READY', 'WINDOW_READY', 'IDLE', 'RACING', 'RESULTS', 'DISPOSING_RACE', 'IDLE']) {
    lifecycle.transition(state);
  }
  assert.equal(lifecycle.state, 'IDLE');
  assert.throws(() => lifecycle.transition('BACKEND_READY'), /Illegal lifecycle transition/);
});
