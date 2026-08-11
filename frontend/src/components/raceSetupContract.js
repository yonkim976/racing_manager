export const THERMAL_PRESET_OPTIONS = ['COOL', 'NORMAL', 'HOT'];
export const WEEKEND_TIRE_ROLES = ['HARD', 'MEDIUM', 'SOFT'];
export const SIMULATION_MODE_OPTIONS = [
  {
    value: 'FULL',
    label: 'FULL · Physics',
    description: 'Existing high-fidelity physical engine',
  },
  {
    value: 'ABSTRACT_BROADCAST',
    label: 'ABSTRACT · Broadcast (Experimental)',
    description: 'Progress-authoritative race with bounded live presentation',
  },
  {
    value: 'ABSTRACT_INSTANT',
    label: 'ABSTRACT · Instant',
    description: 'Progress-authoritative deterministic result without live replay',
  },
];

export function tireNominationForCircuit(circuit) {
  return circuit?.tire_compound_nomination || null;
}

export function weekendTireOptionsForNomination(nomination) {
  return WEEKEND_TIRE_ROLES.map((role) => ({
    role,
    physicalCompound: nomination?.[role.toLowerCase()] || null,
    label: nomination?.[role.toLowerCase()]
      ? `${role} · ${nomination[role.toLowerCase()]}`
      : role,
  }));
}

export function weekendTireOptionsForCircuit(circuit) {
  return weekendTireOptionsForNomination(tireNominationForCircuit(circuit));
}

export function defaultStartingTiresForDrivers(drivers) {
  return Object.fromEntries((drivers || []).map((driver) => [driver.id, 'MEDIUM']));
}

export function beginQualifyingRequest(requestRef) {
  const requestId = requestRef.current + 1;
  requestRef.current = requestId;
  return requestId;
}

export function invalidateQualifyingRequest(requestRef, setLoading) {
  const requestId = beginQualifyingRequest(requestRef);
  setLoading(false);
  return requestId;
}

export function isCurrentQualifyingRequest(requestRef, requestId) {
  return requestRef.current === requestId;
}

export function defaultThermalPresetForCircuit(circuit) {
  return circuit?.thermal_profile?.default_preset || 'NORMAL';
}

export function thermalConditionsForPreset(circuit, thermalPreset) {
  return circuit?.thermal_profile?.presets?.[thermalPreset] || null;
}

export function buildQualifyingPayload({
  circuitId,
  playerTeamId,
  thermalPreset,
  simulationMode = 'FULL',
}) {
  return {
    circuit_id: circuitId,
    player_team_id: playerTeamId,
    attempt_laps: 3,
    simulation_mode: simulationMode,
    thermal_preset: thermalPreset,
  };
}

export function buildRaceSetupPayload({
  circuitId,
  playerTeamId,
  thermalPreset,
  totalLaps,
  selectedTeamDrivers,
  startingTires,
  gridOrder,
  simulationMode = 'FULL',
}) {
  return {
    circuit_id: circuitId,
    player_team_id: playerTeamId,
    simulation_mode: simulationMode,
    abstract_engine: simulationMode.startsWith('ABSTRACT') ? 'PROGRESS_V5' : 'STAGE4',
    thermal_preset: thermalPreset,
    total_laps: totalLaps,
    starting_tires: Object.fromEntries(
      selectedTeamDrivers.map((driver) => [
        driver.id,
        startingTires[driver.id] || 'MEDIUM',
      ]),
    ),
    grid_order: gridOrder || [],
  };
}
