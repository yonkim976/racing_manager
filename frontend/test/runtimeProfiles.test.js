import test from 'node:test';
import assert from 'node:assert/strict';

import { runtimeProfileForMode } from '../src/engines/runtimeProfiles.js';

test('runtime profiles keep FULL and ABSTRACT presentation authority separate', () => {
  const full = runtimeProfileForMode('FULL');
  const broadcast = runtimeProfileForMode('ABSTRACT_BROADCAST');
  const instant = runtimeProfileForMode('ABSTRACT_INSTANT');

  assert.equal(full.family, 'full');
  assert.equal(full.telemetryAuthority, 'physics');
  assert.equal(full.physicsDevControls, true);
  assert.equal(broadcast.family, 'abstract');
  assert.equal(broadcast.telemetryAuthority, 'logical-events');
  assert.equal(broadcast.physicsDevControls, false);
  assert.deepEqual(broadcast.allowedSpeeds, [1, 2, 5]);
  assert.equal(instant.resultOnly, true);
  assert.notEqual(full.presentationAuthority, broadcast.presentationAuthority);
});

test('runtime profile selection rejects unowned modes', () => {
  assert.throws(() => runtimeProfileForMode('HYBRID'), /Unsupported simulation mode/);
});
