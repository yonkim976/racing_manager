import test from 'node:test';
import assert from 'node:assert/strict';
import {
  THERMAL_PRESET_OPTIONS,
  beginQualifyingRequest,
  buildQualifyingPayload,
  buildRaceSetupPayload,
  defaultStartingTiresForDrivers,
  defaultThermalPresetForCircuit,
  invalidateQualifyingRequest,
  isCurrentQualifyingRequest,
  weekendTireOptionsForCircuit,
  thermalConditionsForPreset,
} from '../src/components/raceSetupContract.js';

const bahrain = {
  id: 3,
  total_laps: 57,
  thermal_profile: {
    default_preset: 'NORMAL',
    presets: {
      COOL: { ambient_temperature_c: 24, track_temperature_c: 32 },
      NORMAL: { ambient_temperature_c: 30, track_temperature_c: 40 },
      HOT: { ambient_temperature_c: 36, track_temperature_c: 52 },
    },
  },
  tire_compound_nomination: {
    ruleset: '2026_C1_C5',
    hard: 'C1',
    medium: 'C2',
    soft: 'C3',
  },
};

test('thermal setup exposes the three presets and circuit default', () => {
  assert.deepEqual(THERMAL_PRESET_OPTIONS, ['COOL', 'NORMAL', 'HOT']);
  assert.equal(defaultThermalPresetForCircuit(bahrain), 'NORMAL');
  assert.deepEqual(thermalConditionsForPreset(bahrain, 'HOT'), {
    ambient_temperature_c: 36,
    track_temperature_c: 52,
  });
});

test('missing circuit profile uses the compatibility default without inventing preview values', () => {
  assert.equal(defaultThermalPresetForCircuit({ id: 99 }), 'NORMAL');
  assert.equal(thermalConditionsForPreset({ id: 99 }, 'NORMAL'), null);
});

test('weekend tire roles resolve to the circuit physical nomination', () => {
  assert.deepEqual(weekendTireOptionsForCircuit(bahrain), [
    { role: 'HARD', physicalCompound: 'C1', label: 'HARD · C1' },
    { role: 'MEDIUM', physicalCompound: 'C2', label: 'MEDIUM · C2' },
    { role: 'SOFT', physicalCompound: 'C3', label: 'SOFT · C3' },
  ]);
  assert.equal(weekendTireOptionsForCircuit({ id: 99 })[1].physicalCompound, null);
});

test('circuit changes reset starting roles to the neutral medium choice', () => {
  assert.deepEqual(
    defaultStartingTiresForDrivers([{ id: 7 }, { id: 8 }]),
    { 7: 'MEDIUM', 8: 'MEDIUM' },
  );
});

test('qualifying payload carries the selected thermal preset', () => {
  assert.deepEqual(
    buildQualifyingPayload({
      circuitId: 3,
      playerTeamId: 1,
      thermalPreset: 'COOL',
    }),
    {
      circuit_id: 3,
      player_team_id: 1,
      attempt_laps: 3,
      thermal_preset: 'COOL',
    },
  );
});

test('race setup payload reuses the same preset and preserves tire/grid choices', () => {
  assert.deepEqual(
    buildRaceSetupPayload({
      circuitId: 3,
      playerTeamId: 1,
      thermalPreset: 'HOT',
      totalLaps: 57,
      selectedTeamDrivers: [{ id: 7 }, { id: 8 }],
      startingTires: { 7: 'SOFT' },
      gridOrder: [8, 7],
    }),
    {
      circuit_id: 3,
      player_team_id: 1,
      thermal_preset: 'HOT',
      total_laps: 57,
      starting_tires: { 7: 'SOFT', 8: 'MEDIUM' },
      grid_order: [8, 7],
    },
  );
});

test('invalidating an in-flight qualifying request clears loading and rejects stale completion', async () => {
  const requestRef = { current: 0 };
  let loading = false;
  const setLoading = (value) => {
    loading = value;
  };

  const firstRequest = beginQualifyingRequest(requestRef);
  loading = true;
  const invalidatedRequest = invalidateQualifyingRequest(requestRef, setLoading);
  assert.equal(loading, false);
  assert.notEqual(invalidatedRequest, firstRequest);
  assert.equal(isCurrentQualifyingRequest(requestRef, firstRequest), false);

  const secondRequest = beginQualifyingRequest(requestRef);
  loading = true;
  await Promise.resolve();
  assert.equal(isCurrentQualifyingRequest(requestRef, firstRequest), false);
  assert.equal(isCurrentQualifyingRequest(requestRef, secondRequest), true);
  assert.equal(loading, true);
});
