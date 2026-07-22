import { lazy, Suspense, useEffect, useState } from 'react';
import TimingBoard from './components/Dashboard/TimingBoard';
import StrategyPanel from './components/Dashboard/StrategyPanel';
import SpeedControl from './components/Controls/SpeedControl';
import EventFeed from './components/Controls/EventFeed';
import DevRaceControl from './components/DevRaceControl/DevRaceControl';
import RaceSetup from './components/RaceSetup';
import { useRaceWebSocket } from './hooks/useRaceWebSocket';
import './App.css';

const CIRCUIT_DISPLAY_ROTATION_OVERRIDES = {
  3: 270,
  6: 270,
};
const DEV_RACE_CONTROLS_ENABLED = import.meta.env.DEV
  || import.meta.env.VITE_ENABLE_RACE_DEV_CONTROLS === 'true';
const EMPTY_TRACK_DATA = Object.freeze([]);
const TrackCanvas = lazy(() => import('./components/TrackView/TrackCanvas'));

function formatRaceTime(seconds) {
  if (!seconds || seconds <= 0) return '—';
  const mins = Math.floor(seconds / 60);
  const secs = (seconds % 60).toFixed(3).padStart(6, '0');
  return `${mins}:${secs}`;
}

function formatSafetyCarStage(stage) {
  const labels = {
    deploying: 'DEPLOYED',
    collecting: 'CATCHING FIELD',
    queued: 'FIELD QUEUED',
    unlapping: 'LAPPED CARS OVERTAKING',
    in_this_lap: 'IN THIS LAP',
    restart: 'RESTART',
  };
  return labels[stage] || 'DEPLOYED';
}

function formatResultTime(result, winnerTime) {
  if (result.retired) return 'DNF';
  if (result.position === 1) return formatRaceTime(result.total_time);
  return `+${Math.max(0, result.total_time - winnerTime).toFixed(3)}`;
}

function StartLightsOverlay({ phase, lightCount, lightsOut }) {
  return (
    <div className="start-lights-overlay">
      <div className={`start-lights ${lightsOut ? 'start-lights--out' : ''}`}>
        <div className="start-lights__row">
          {[0, 1, 2, 3, 4].map((index) => (
            <span
              key={index}
              className={`start-lights__lamp ${index < lightCount ? 'start-lights__lamp--on' : ''}`}
            />
          ))}
        </div>
        <span className="start-lights__status">
          {lightsOut ? 'LIGHTS OUT' : phase === 'grid' ? 'CARS ON GRID' : 'GET READY'}
        </span>
      </div>
    </div>
  );
}

function getCircuitViewBounds(circuit) {
  const fit = circuit?.metric?.fit || circuit?.geo?.fit;
  const width = Number(fit?.width);
  const height = Number(fit?.height);
  if (!Number.isFinite(width) || !Number.isFinite(height) || width <= 0 || height <= 0) {
    return null;
  }

  return {
    x: Number(fit.x) || 0,
    y: Number(fit.y) || 0,
    width,
    height,
  };
}

function getCoordinateBounds(coords = []) {
  if (!coords.length) return null;

  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;

  for (const coord of coords) {
    const [x, y] = coord || [];
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    minX = Math.min(minX, x);
    minY = Math.min(minY, y);
    maxX = Math.max(maxX, x);
    maxY = Math.max(maxY, y);
  }

  if (!Number.isFinite(minX) || !Number.isFinite(minY)) return null;
  return { width: maxX - minX, height: maxY - minY };
}

function getCircuitDisplayRotationDeg(circuit) {
  if (!circuit?.geo) return 0;
  const rotationOverride = CIRCUIT_DISPLAY_ROTATION_OVERRIDES[Number(circuit.id)];
  if (rotationOverride !== undefined) return rotationOverride;

  const bounds = getCoordinateBounds(circuit.track_coords || []);
  if (!bounds || bounds.width <= 0 || bounds.height <= 0) return 0;
  return bounds.height > bounds.width * 1.05 ? 90 : 0;
}

export default function App() {
  const [phase, setPhase] = useState('setup');
  const [setupResult, setSetupResult] = useState(null);
  const [pendingPaused, setPendingPaused] = useState(null);

  const {
    raceInfo,
    raceState,
    poseTickRef,
    events,
    raceEnd,
    connected,
    connectionState,
    sendCommand,
    resetState,
  } = useRaceWebSocket(phase === 'race');

  const handleRaceStart = (result) => {
    resetState();
    setPendingPaused(null);
    setSetupResult(result);
    setPhase('race');
  };

  const handleBackToSetup = async () => {
    fetch('/api/race/session', { method: 'DELETE' }).catch(() => {});
    setPhase('setup');
    setSetupResult(null);
    setPendingPaused(null);
    resetState();
  };

  useEffect(() => {
    if (
      pendingPaused !== null
      && raceState?.paused === pendingPaused
    ) {
      setPendingPaused(null);
    }
  }, [pendingPaused, raceState?.paused]);

  useEffect(() => {
    if (!connected) setPendingPaused(null);
  }, [connected]);

  if (phase === 'setup') {
    return <RaceSetup onStart={handleRaceStart} />;
  }

  const playerDriverIds = raceInfo?.player_drivers || setupResult?.player_drivers?.map((d) => d.id) || [];
  const displayedPaused = pendingPaused ?? raceState?.paused ?? false;
  const setupPlayerDriverCodes = setupResult?.player_drivers?.map((driver) => driver.abbreviation) || [];
  const livePlayerDriverCodes = raceState?.positions
    ?.filter((position) => playerDriverIds.includes(position.driver_id))
    .map((position) => position.name) || [];
  const playerDriverCodes = [...new Set([...setupPlayerDriverCodes, ...livePlayerDriverCodes])];
  const playerPositions = raceState?.positions?.filter((p) => playerDriverIds.includes(p.driver_id)) || [];
  const finalResults = [...(raceEnd?.results || [])].sort((a, b) => a.position - b.position);
  const winnerTime = finalResults.find((r) => !r.retired)?.total_time || 0;
  const circuitName = raceInfo?.circuit_name || setupResult?.circuit?.name;
  const trackCoords = raceInfo?.track_coords || setupResult?.circuit?.track_coords;
  const trackLengthM = raceInfo?.track_length_m || setupResult?.circuit?.track_length_m || 5000;
  const worldCoordinateFrame = raceInfo ? {
    originXRender: Number(raceInfo.world_origin_x_render || 0),
    originYRender: Number(raceInfo.world_origin_y_render || 0),
    metersPerRenderUnit: Number(raceInfo.world_meters_per_render_unit || 1),
  } : null;
  const trackWidthM = raceInfo?.track_width_m || setupResult?.circuit?.track_width_m || 12;
  const carWidthM = raceInfo?.car_width_m || 1.9;
  const carLengthM = raceInfo?.car_length_m || 5.0;
  const gridSlots = raceInfo?.grid_slots || EMPTY_TRACK_DATA;
  const racingLineProfile = raceInfo?.racing_line_profile || EMPTY_TRACK_DATA;
  const trackWidthProfile = raceInfo?.track_width_profile || EMPTY_TRACK_DATA;
  const surfaceZones = raceInfo?.surface_zones || EMPTY_TRACK_DATA;
  const racingLineCoords = raceInfo?.racing_line_coords || EMPTY_TRACK_DATA;
  const pitLaneCoords = raceInfo?.pit_lane_coords || setupResult?.circuit?.pit_lane_coords;
  const pitBoxOffset = raceInfo?.pit_box_offset ?? setupResult?.circuit?.pit_lane?.box_offset ?? 11;
  const pitLaneWidthM = raceInfo?.pit_lane_width_m
    ?? setupResult?.circuit?.pit_lane?.lane_width_m
    ?? 4;
  const pitSideEntryProgress = raceInfo?.pit_side_entry_progress
    ?? setupResult?.circuit?.pit_lane?.side_entry_progress
    ?? 0.02;
  const pitSpeedLimitStart = raceInfo?.pit_speed_limit_start
    ?? setupResult?.circuit?.pit_lane?.speed_limit_start
    ?? 0.12;
  const pitBoxProgress = raceInfo?.pit_box_progress
    ?? setupResult?.circuit?.pit_lane?.box_progress
    ?? 0.5;
  const pitSpeedLimitEnd = raceInfo?.pit_speed_limit_end
    ?? setupResult?.circuit?.pit_lane?.speed_limit_end
    ?? 0.88;
  const pitSideRejoinProgress = raceInfo?.pit_side_rejoin_progress
    ?? setupResult?.circuit?.pit_lane?.side_rejoin_progress
    ?? 0.94;
  const drsZones = raceInfo?.drs_zones || setupResult?.circuit?.drs_zones || [];
  const sectors = raceInfo?.sectors || setupResult?.circuit?.sectors || [];
  const startFinishIndex = raceInfo?.start_finish_index ?? setupResult?.circuit?.start_finish_index ?? 0;
  const landmarks = raceInfo?.landmarks || setupResult?.circuit?.landmarks || [];
  const circuitViewBounds = getCircuitViewBounds(setupResult?.circuit);
  const circuitDisplayRotationDeg = getCircuitDisplayRotationDeg(setupResult?.circuit);
  const showCircuitBearing = Boolean(setupResult?.circuit?.geo);
  const startSequencePhase = raceState?.start_sequence_phase || 'connecting';
  const startLightCount = raceState?.start_light_count || 0;
  const lightsOut = startSequencePhase === 'lights_out';
  const startLightsActive = ['grid', 'lights', 'lights_out'].includes(startSequencePhase);
  const statusText = startLightsActive
    ? (lightsOut ? 'LIGHTS OUT' : 'ON THE GRID')
    : (connectionState === 'connected' ? '● LIVE' : connectionState.toUpperCase());
  const statusClass = startLightsActive ? 'connecting' : connectionState;

  return (
    <div className="race-app carbon-bg">
      <header className="race-header glass-panel">
        <div className="race-header__left">
          <span className="race-header__logo">F1 RACE MANAGER</span>
          {circuitName && (
            <span className="race-header__circuit">{circuitName}</span>
          )}
        </div>
        <div className="race-header__center">
          {startLightsActive ? (
            <span className="race-header__lap">STARTING GRID</span>
          ) : raceState && (
            <>
              <span className="race-header__lap">
                LAP {Math.min(raceState.lap, raceState.total_laps)}/{raceState.total_laps}
              </span>
              {raceState.race_phase === 'sc' && (
                <span className="race-header__sc">
                  SAFETY CAR
                  <span className="race-header__sc-time">
                    {formatSafetyCarStage(raceState.safety_car_stage)}
                  </span>
                </span>
              )}
              {raceState.race_phase === 'vsc' && (
                <span className="race-header__sc race-header__sc--vsc">
                  VSC
                  <span className="race-header__sc-time">
                    {Math.max(0, Math.ceil(raceState.race_phase_remaining_seconds || 0))}s
                  </span>
                </span>
              )}
              {raceState.pit_window_open && (
                <span className="race-header__pit-window">PIT WINDOW OPEN</span>
              )}
            </>
          )}
        </div>
        <div className="race-header__right">
          <span className={`race-header__status race-header__status--${statusClass}`}>
            {statusText}
          </span>
          <button type="button" className="race-header__back" onClick={handleBackToSetup}>
            Exit
          </button>
        </div>
      </header>

      {raceEnd && (
        <div className="race-end-overlay">
          <div className="race-end-panel glass-panel">
            <h2>Race Finished</h2>
            <div className="race-end-table">
              <div className="race-end-table__head">
                <span>POS</span>
                <span>DRIVER</span>
                <span>TEAM</span>
                <span>TIME</span>
              </div>
            <ol className="race-end-results">
              {finalResults.map((r) => (
                <li key={r.driver_id}>
                  <span className="race-end-pos">P{r.position}</span>
                  <span className="race-end-driver">
                    <span>{r.name}</span>
                    <span>{r.full_name}</span>
                  </span>
                  <span className="race-end-team">{r.team}</span>
                  <span className={r.retired ? 'race-end-time race-end-time--dnf' : 'race-end-time'}>
                    {formatResultTime(r, winnerTime)}
                  </span>
                </li>
              ))}
            </ol>
            </div>
            <button type="button" onClick={handleBackToSetup}>New Race</button>
          </div>
        </div>
      )}

      {startLightsActive && (
        <StartLightsOverlay
          phase={startSequencePhase}
          lightCount={startLightCount}
          lightsOut={lightsOut}
        />
      )}

      <main className="race-layout">
        <aside className="race-layout__sidebar">
          <TimingBoard
            positions={raceState?.positions}
            poseTickRef={poseTickRef}
            playerDriverIds={playerDriverIds}
          />
        </aside>

        <section className="race-layout__main">
          <Suspense fallback={<div className="track-canvas track-canvas--loading">Loading track…</div>}>
            <TrackCanvas
            trackCoords={trackCoords}
            trackLengthM={trackLengthM}
            worldCoordinateFrame={worldCoordinateFrame}
            trackWidthM={trackWidthM}
            carWidthM={carWidthM}
            carLengthM={carLengthM}
            gridSlots={gridSlots}
            racingLineProfile={racingLineProfile}
            trackWidthProfile={trackWidthProfile}
            surfaceZones={surfaceZones}
            racingLineCoords={racingLineCoords}
            pitLaneCoords={pitLaneCoords}
            pitBoxOffset={pitBoxOffset}
            pitLaneWidthM={pitLaneWidthM}
            pitSideEntryProgress={pitSideEntryProgress}
            pitSpeedLimitStart={pitSpeedLimitStart}
            pitBoxProgress={pitBoxProgress}
            pitSpeedLimitEnd={pitSpeedLimitEnd}
            pitSideRejoinProgress={pitSideRejoinProgress}
            drsZones={drsZones}
            sectors={sectors}
            startFinishIndex={startFinishIndex}
            landmarks={landmarks}
            viewBounds={circuitViewBounds}
            displayRotationDeg={circuitDisplayRotationDeg}
            showBearing={showCircuitBearing}
            positions={raceState?.positions}
            playerDriverIds={playerDriverIds}
            speedMultiplier={raceState?.speed_multiplier || 1}
            paused={displayedPaused}
            racePhase={raceState?.race_phase || 'green'}
            safetyCarStage={raceState?.safety_car_stage || 'inactive'}
            safetyCarVisible={raceState?.safety_car_visible || false}
            safetyCarRoute={raceState?.safety_car_route || 'track'}
            safetyCarProgress={raceState?.safety_car_progress}
            safetyCarProgressRate={raceState?.safety_car_progress_rate || 0}
            safetyCarPitLaneProgress={raceState?.safety_car_pit_lane_progress || 0}
            />
          </Suspense>
          <EventFeed events={events} playerDriverCodes={playerDriverCodes} />
        </section>

        <aside className="race-layout__controls">
          <SpeedControl
            connected={connected}
            speedMultiplier={raceState?.speed_multiplier ?? 1}
            paused={displayedPaused}
            onSpeed={(m) => sendCommand({ type: 'set_speed', multiplier: m })}
            onPause={() => {
              setPendingPaused(true);
              sendCommand({ type: 'pause' });
            }}
            onResume={() => sendCommand({ type: 'resume' })}
          />
          {DEV_RACE_CONTROLS_ENABLED && (
            <DevRaceControl
              connected={connected}
              racePhase={raceState?.race_phase || 'green'}
              safetyCarStage={raceState?.safety_car_stage || 'inactive'}
              remainingSeconds={raceState?.race_phase_remaining_seconds || 0}
              remainingLaps={raceState?.race_phase_remaining_laps || 0}
              physicsHz={raceState?.physics_hz || 50}
              broadcastHz={raceState?.broadcast_hz || 30}
              requestedSpeedMultiplier={raceState?.speed_multiplier || 1}
              effectiveSpeedMultiplier={raceState?.effective_speed_multiplier || 0}
              simulationBacklogSeconds={raceState?.simulation_backlog_seconds || 0}
              broadcastJitterMs={raceState?.broadcast_jitter_ms || 0}
              onSetPhase={(nextPhase) => sendCommand({
                type: 'dev_set_race_phase',
                phase: nextPhase,
              })}
            />
          )}
          <StrategyPanel
            drivers={playerPositions}
            pitWindowOpen={raceState?.pit_window_open || false}
            onPitCall={(driverId, tire) =>
              sendCommand({ type: 'pit_call', driver_id: driverId, tire_choice: tire })
            }
            onPaceModeChange={(driverId, paceMode) =>
              sendCommand({ type: 'set_pace_mode', driver_id: driverId, pace_mode: paceMode })
            }
          />
        </aside>
      </main>
    </div>
  );
}
