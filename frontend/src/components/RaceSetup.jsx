import { useState, useEffect, useMemo, useRef } from 'react';
import CircuitDesigner from './CircuitDesigner';
import {
  SIMULATION_MODE_OPTIONS,
  THERMAL_PRESET_OPTIONS,
  beginQualifyingRequest,
  buildQualifyingPayload,
  buildRaceSetupPayload,
  defaultStartingTiresForDrivers,
  defaultThermalPresetForCircuit,
  invalidateQualifyingRequest,
  isCurrentQualifyingRequest,
  thermalConditionsForPreset,
  weekendTireOptionsForCircuit,
} from './raceSetupContract';
import './RaceSetup.css';

const MIN_LAPS = 5;
const MAX_LAPS = 100;

function clampLapCount(value) {
  const numericValue = Number(value);
  if (Number.isNaN(numericValue)) return MIN_LAPS;
  return Math.min(MAX_LAPS, Math.max(MIN_LAPS, Math.round(numericValue)));
}

function getTireClass(compound) {
  switch (compound) {
    case 'SOFT': return 'race-setup__tire-option--soft';
    case 'MEDIUM': return 'race-setup__tire-option--medium';
    case 'HARD': return 'race-setup__tire-option--hard';
    default: return '';
  }
}

function formatLapTime(seconds) {
  if (!seconds || seconds <= 0) return '--';
  const minutes = Math.floor(seconds / 60);
  const remaining = seconds - minutes * 60;
  return `${minutes}:${remaining.toFixed(3).padStart(6, '0')}`;
}

export default function RaceSetup({ onStart }) {
  const [circuits, setCircuits] = useState([]);
  const [teams, setTeams] = useState([]);
  const [drivers, setDrivers] = useState([]);
  const [circuitId, setCircuitId] = useState(1);
  const [thermalPreset, setThermalPreset] = useState('NORMAL');
  const [simulationMode, setSimulationMode] = useState('FULL');
  const [teamId, setTeamId] = useState(1);
  const [lapCount, setLapCount] = useState(30);
  const [startingTires, setStartingTires] = useState({});
  const [qualifying, setQualifying] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [designerOpen, setDesignerOpen] = useState(false);
  const qualifyingRequestIdRef = useRef(0);

  useEffect(() => {
    Promise.all([
      fetch('/api/circuits').then((r) => r.json()),
      fetch('/api/teams').then((r) => r.json()),
      fetch('/api/drivers').then((r) => r.json()),
    ])
      .then(([c, t, d]) => {
        setCircuits(c);
        setTeams(t);
        setDrivers(d);
        if (c.length) {
          setCircuitId(c[0].id);
          setLapCount(clampLapCount(c[0].total_laps));
          setThermalPreset(defaultThermalPresetForCircuit(c[0]));
        }
        if (t.length) setTeamId(t[0].id);
      })
      .catch(() => setError('Failed to load race data. Is the backend running?'));
  }, []);

  const selectedCircuit = circuits.find((c) => c.id === circuitId);
  const startingTireOptions = weekendTireOptionsForCircuit(selectedCircuit);
  const thermalProfile = selectedCircuit?.thermal_profile;
  const selectedThermalConditions = thermalConditionsForPreset(selectedCircuit, thermalPreset);
  const selectedTeam = teams.find((t) => t.id === teamId);
  const selectedTeamDrivers = useMemo(
    () => drivers.filter((d) => d.team_id === teamId),
    [drivers, teamId],
  );
  const selectedTeamDriverIds = useMemo(
    () => new Set(selectedTeamDrivers.map((driver) => driver.id)),
    [selectedTeamDrivers],
  );

  useEffect(() => {
    setStartingTires((current) => {
      const next = {};
      for (const driver of selectedTeamDrivers) {
        next[driver.id] = current[driver.id] || 'MEDIUM';
      }
      return next;
    });
  }, [selectedTeamDrivers]);

  const handleStartingTireChange = (driverId, compound) => {
    setStartingTires((current) => ({
      ...current,
      [driverId]: compound,
    }));
  };

  const handleCircuitChange = (nextCircuitId) => {
    invalidateQualifyingRequest(qualifyingRequestIdRef, setLoading);
    setCircuitId(nextCircuitId);
    setStartingTires(defaultStartingTiresForDrivers(selectedTeamDrivers));
    setQualifying(null);
    const nextCircuit = circuits.find((c) => c.id === nextCircuitId);
    if (nextCircuit) {
      setLapCount(clampLapCount(nextCircuit.total_laps));
      setThermalPreset(defaultThermalPresetForCircuit(nextCircuit));
    }
  };

  const handleThermalPresetChange = (nextPreset) => {
    invalidateQualifyingRequest(qualifyingRequestIdRef, setLoading);
    setThermalPreset(nextPreset);
    setQualifying(null);
  };

  const handleSimulationModeChange = (nextMode) => {
    invalidateQualifyingRequest(qualifyingRequestIdRef, setLoading);
    setSimulationMode(nextMode);
    setQualifying(null);
  };

  const handleLapCountChange = (value) => {
    setLapCount(clampLapCount(value));
  };

  const handleRunQualifying = async () => {
    const requestId = beginQualifyingRequest(qualifyingRequestIdRef);
    setLoading(true);
    setError(null);
    setQualifying(null);
    try {
      const res = await fetch('/api/qualifying/run', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(buildQualifyingPayload({
          circuitId,
          playerTeamId: teamId,
          thermalPreset,
          simulationMode,
        })),
      });
      if (!res.ok) {
        const body = await res.json().catch(() => ({}));
        throw new Error(body.detail || 'Qualifying failed');
      }
      const data = await res.json();
      if (isCurrentQualifyingRequest(qualifyingRequestIdRef, requestId)) {
        setQualifying(data);
      }
    } catch (err) {
      if (isCurrentQualifyingRequest(qualifyingRequestIdRef, requestId)) {
        setError(err.message);
      }
    } finally {
      if (isCurrentQualifyingRequest(qualifyingRequestIdRef, requestId)) {
        setLoading(false);
      }
    }
  };

  const handleStart = async () => {
    setLoading(true);
    setError(null);
    const selectedLapCount = clampLapCount(lapCount);
    setLapCount(selectedLapCount);
    try {
      const res = await fetch('/api/race/setup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(buildRaceSetupPayload({
          circuitId,
          playerTeamId: teamId,
          thermalPreset,
          totalLaps: selectedLapCount,
          selectedTeamDrivers,
          startingTires,
          gridOrder: qualifying?.grid_order || [],
          simulationMode,
        })),
      });
      if (!res.ok) {
        const body = await res.json().catch(() => ({}));
        throw new Error(body.detail || 'Setup failed');
      }
      const data = await res.json();
      onStart(data);
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  if (designerOpen) {
    return (
      <CircuitDesigner
        circuits={circuits}
        initialCircuitId={circuitId}
        onClose={() => setDesignerOpen(false)}
      />
    );
  }

  return (
    <div className="race-setup carbon-bg">
      <div className="race-setup__panel glass-panel">
        <div className="race-setup__header">
          <div>
            <h1 className="race-setup__title">F1 RACE MANAGER</h1>
            <p className="race-setup__subtitle">Select your engine, circuit and team to begin</p>
          </div>
          <button
            type="button"
            className="race-setup__designer"
            onClick={() => setDesignerOpen(true)}
            disabled={!circuits.length}
          >
            Maker
          </button>
        </div>

        {error && <div className="race-setup__error">{error}</div>}

        <div className="race-setup__field">
          <label>Simulation Mode</label>
          <div className="race-setup__mode-options" role="radiogroup" aria-label="Simulation mode">
            {SIMULATION_MODE_OPTIONS.map((mode) => {
              const selected = simulationMode === mode.value;
              return (
                <button
                  key={mode.value}
                  type="button"
                  role="radio"
                  aria-checked={selected}
                  className={`race-setup__mode-option ${selected ? 'race-setup__mode-option--selected' : ''}`}
                  onClick={() => handleSimulationModeChange(mode.value)}
                >
                  <span className="race-setup__mode-label">{mode.label}</span>
                  <span className="race-setup__mode-description">{mode.description}</span>
                </button>
              );
            })}
          </div>
          {simulationMode === 'ABSTRACT_BROADCAST' && (
            <span className="race-setup__hint">
              Experimental Progress v5: progress, rank, pit stops and race control are authoritative; vehicle poses are presentation-only.
            </span>
          )}
          {simulationMode === 'ABSTRACT_INSTANT' && (
            <span className="race-setup__hint">
              Deterministic result mode. No live replay is started.
            </span>
          )}
        </div>

        <div className="race-setup__field">
          <label htmlFor="circuit">Circuit</label>
          <select
            id="circuit"
            value={circuitId}
            onChange={(e) => handleCircuitChange(Number(e.target.value))}
          >
            {circuits.map((c) => (
              <option key={c.id} value={c.id}>
                {c.name}
              </option>
            ))}
          </select>
          {selectedCircuit && (
            <span className="race-setup__hint">
              {selectedCircuit.physics_calibration?.reference_lap_time_seconds
                ? `Telemetry reference: ${selectedCircuit.physics_calibration.reference_lap_time_seconds}s`
                : `Base lap: ${selectedCircuit.base_lap_time}s`}
            </span>
          )}
        </div>

        <div className="race-setup__field">
          <label htmlFor="lap-count">Race Laps</label>
          <div className="race-setup__lap-control">
            <button
              type="button"
              className="race-setup__lap-step"
              onClick={() => handleLapCountChange(lapCount - 1)}
              disabled={lapCount <= MIN_LAPS}
            >
              -
            </button>
            <input
              id="lap-count"
              type="number"
              min={MIN_LAPS}
              max={MAX_LAPS}
              value={lapCount}
              onChange={(e) => handleLapCountChange(e.target.value)}
            />
            <button
              type="button"
              className="race-setup__lap-step"
              onClick={() => handleLapCountChange(lapCount + 1)}
              disabled={lapCount >= MAX_LAPS}
            >
              +
            </button>
          </div>
          <span className="race-setup__hint">Min {MIN_LAPS} laps / Max {MAX_LAPS} laps</span>
        </div>

        <div className="race-setup__field">
          <label htmlFor="thermal-preset">Track Temperature</label>
          <select
            id="thermal-preset"
            value={thermalPreset}
            onChange={(e) => handleThermalPresetChange(e.target.value)}
            disabled={!thermalProfile}
          >
            {THERMAL_PRESET_OPTIONS.map((preset) => (
              <option key={preset} value={preset}>
                {preset[0] + preset.slice(1).toLowerCase()}
                {preset === thermalProfile?.default_preset ? ' (Default)' : ''}
              </option>
            ))}
          </select>
          {selectedThermalConditions && (
            <span className="race-setup__thermal-preview">
              <span>Ambient {selectedThermalConditions.ambient_temperature_c}°C</span>
              <span>Track {selectedThermalConditions.track_temperature_c}°C</span>
            </span>
          )}
        </div>

        <div className="race-setup__field">
          <label htmlFor="team">Your Team</label>
          <select
            id="team"
            value={teamId}
            onChange={(e) => setTeamId(Number(e.target.value))}
          >
            {teams.map((t) => (
              <option key={t.id} value={t.id}>
                {t.name}
              </option>
            ))}
          </select>
          {selectedTeam && (
            <span
              className="race-setup__team-color"
              style={{ backgroundColor: selectedTeam.color }}
            />
          )}
        </div>

        {selectedTeamDrivers.length > 0 && (
          <div className="race-setup__field">
            <label>Starting Tires</label>
            <div className="race-setup__tires">
              {selectedTeamDrivers.map((driver) => (
                <div className="race-setup__driver-tires" key={driver.id}>
                  <div className="race-setup__driver">
                    <span>{driver.abbreviation}</span>
                    <span>{driver.name}</span>
                  </div>
                  <div className="race-setup__tire-options">
                    {startingTireOptions.map((option) => {
                      const selected = (startingTires[driver.id] || 'MEDIUM') === option.role;
                      return (
                        <button
                          key={option.role}
                          type="button"
                          className={`race-setup__tire-option ${getTireClass(option.role)} ${selected ? 'race-setup__tire-option--selected' : ''}`}
                          onClick={() => handleStartingTireChange(driver.id, option.role)}
                          title={option.label}
                        >
                          {option.physicalCompound
                            ? `${option.role[0]} · ${option.physicalCompound}`
                            : option.role[0]}
                        </button>
                      );
                    })}
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}

        {qualifying?.results?.length > 0 && (
          <div className="race-setup__field">
            <label>Qualifying Result</label>
            <div className="race-setup__qualifying">
              <div className="race-setup__qualifying-head">
                <span>Pos</span>
                <span>Driver</span>
                <span>Q1</span>
                <span>Q2</span>
                <span>Q3</span>
              </div>
              <div className="race-setup__qualifying-list">
                {qualifying.results.map((result, index) => {
                  const isPlayer = selectedTeamDriverIds.has(result.driver_id);
                  const prev = qualifying.results[index - 1];
                  const isCutLine = prev && prev.knockout !== result.knockout;
                  const koClass = `race-setup__qualifying-row--ko-${result.knockout.toLowerCase()}`;
                  return (
                    <div
                      key={result.driver_id}
                      className={[
                        'race-setup__qualifying-row',
                        koClass,
                        isPlayer ? 'race-setup__qualifying-row--player' : '',
                        isCutLine ? 'race-setup__qualifying-row--cut' : '',
                      ].filter(Boolean).join(' ')}
                    >
                      <span className="race-setup__qualifying-pos">P{result.position}</span>
                      <span className="race-setup__qualifying-driver">
                        <i style={{ backgroundColor: result.team_color }} />
                        {result.name}
                      </span>
                      <span className="race-setup__qualifying-time">{formatLapTime(result.q1_time)}</span>
                      <span className="race-setup__qualifying-time">{formatLapTime(result.q2_time)}</span>
                      <span className="race-setup__qualifying-time race-setup__qualifying-time--final">{formatLapTime(result.q3_time)}</span>
                    </div>
                  );
                })}
              </div>
            </div>
          </div>
        )}

        <button
          type="button"
          className="race-setup__start"
          onClick={qualifying ? handleStart : handleRunQualifying}
          disabled={loading || !circuits.length}
        >
          {loading
            ? (qualifying ? 'Starting...' : 'Qualifying...')
            : (qualifying ? 'START RACE' : 'RUN QUALIFYING')}
        </button>
      </div>
    </div>
  );
}
