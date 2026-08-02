export const THERMAL_PRESET_OPTIONS = ['COOL', 'NORMAL', 'HOT'];
export const WEEKEND_TIRE_ROLES = ['HARD', 'MEDIUM', 'SOFT'];

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

export function buildQualifyingPayload({ circuitId, playerTeamId, thermalPreset }) {
  return {
    circuit_id: circuitId,
    player_team_id: playerTeamId,
    attempt_laps: 3,
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
}) {
  return {
    circuit_id: circuitId,
    player_team_id: playerTeamId,
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
