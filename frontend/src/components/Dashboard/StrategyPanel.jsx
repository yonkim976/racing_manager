import { useEffect, useMemo, useState } from 'react';
import { createPortal } from 'react-dom';
import './StrategyPanel.css';

const TIRE_OPTIONS = ['SOFT', 'MEDIUM', 'HARD'];
const PACE_MODES = [
  { value: 'CONSERVE', label: 'SAVE' },
  { value: 'STANDARD', label: 'STD' },
  { value: 'ATTACK', label: 'ATK' },
];

function wearColor(wear) {
  const life = tireLifePercent(wear);
  if (life <= 20) return 'var(--accent-red)';
  if (life <= 50) return 'var(--accent-yellow)';
  return 'var(--accent-green)';
}

function tireLifePercent(wear) {
  return Math.max(0, Math.round((1 - Math.min(wear || 0, 1)) * 100));
}

function tireColor(compound) {
  if (compound === 'SOFT') return '#e10600';
  if (compound === 'MEDIUM') return '#f1c40f';
  if (compound === 'HARD') return '#f5f5f5';
  if (compound === 'INTER') return '#2ecc71';
  if (compound === 'WET') return '#3498db';
  return 'var(--text-secondary)';
}

function formatLapTime(seconds) {
  if (!seconds || seconds <= 0) return '—';
  const minutes = Math.floor(seconds / 60);
  const remaining = (seconds - minutes * 60).toFixed(3).padStart(6, '0');
  return `${minutes}:${remaining}`;
}

function formatDelta(seconds) {
  if (!seconds || seconds <= 0.0005) return 'BEST';
  return `+${seconds.toFixed(3)}`;
}

function formatLapPointTitle(lap) {
  return `Lap ${lap.lap} · ${formatLapTime(lap.lap_time)} · ${lap.tire_compound}${lap.pit_stop ? ' · PIT' : ''}`;
}

function averageLapTime(laps) {
  if (!laps.length) return 0;
  return laps.reduce((sum, lap) => sum + lap.lap_time, 0) / laps.length;
}

function bestLap(laps) {
  if (!laps.length) return null;
  return laps.reduce((best, lap) => (lap.lap_time < best.lap_time ? lap : best), laps[0]);
}

function buildStints(laps) {
  const stints = [];
  laps.forEach((lap) => {
    let stint = stints.find((item) => item.stint === lap.stint);
    if (!stint) {
      stint = {
        stint: lap.stint,
        tire_compound: lap.tire_compound,
        laps: [],
      };
      stints.push(stint);
    }
    stint.laps.push(lap);
  });
  return stints;
}

function LapTimeChart({ laps }) {
  if (!laps.length) {
    return <div className="strategy-stats__chart-empty">NO LAPS</div>;
  }

  const width = 1120;
  const height = 300;
  const padX = 38;
  const padY = 34;
  const times = laps.map((lap) => lap.lap_time);
  const minTime = Math.min(...times);
  const maxTime = Math.max(...times);
  const range = Math.max(0.25, maxTime - minTime);
  const chartWidth = width - padX * 2;
  const chartHeight = height - padY * 2;
  const xForIndex = (index) =>
    padX + (laps.length === 1 ? chartWidth / 2 : (index / (laps.length - 1)) * chartWidth);
  const yForTime = (time) => padY + ((time - minTime) / range) * chartHeight;
  const points = laps.map((lap, index) => `${xForIndex(index)},${yForTime(lap.lap_time)}`).join(' ');

  return (
    <svg className="strategy-stats__chart" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Lap time trend">
      <line x1={padX} y1={padY} x2={padX} y2={height - padY} className="strategy-stats__axis" />
      <line x1={padX} y1={height - padY} x2={width - padX} y2={height - padY} className="strategy-stats__axis" />
      <line
        x1={padX}
        y1={yForTime(minTime)}
        x2={width - padX}
        y2={yForTime(minTime)}
        className="strategy-stats__best-line"
      />
      {laps.length > 1 && <polyline points={points} className="strategy-stats__line" />}
      {laps.map((lap, index) => (
        <g key={`${lap.lap}-${lap.stint}`} className="strategy-stats__point-hit">
          <title>{formatLapPointTitle(lap)}</title>
          <circle
          key={`${lap.lap}-${lap.stint}`}
          cx={xForIndex(index)}
          cy={yForTime(lap.lap_time)}
          r={lap.pit_stop ? 4.6 : 3.4}
          fill={tireColor(lap.tire_compound)}
          className={lap.pit_stop ? 'strategy-stats__point strategy-stats__point--pit' : 'strategy-stats__point'}
          />
        </g>
      ))}
      <text x={padX} y={10} className="strategy-stats__chart-label">{formatLapTime(minTime)}</text>
      <text x={padX} y={height - 3} className="strategy-stats__chart-label">{formatLapTime(maxTime)}</text>
    </svg>
  );
}

function DriverLapStats({ driver }) {
  const [view, setView] = useState('chart');
  const laps = driver.lap_history || [];
  const tableLaps = [...laps].reverse();
  const best = bestLap(laps);
  const latest = laps[laps.length - 1] || null;
  const average = averageLapTime(laps);
  const stints = buildStints(laps);

  return (
    <section className="strategy-stats">
      <div className="strategy-stats__header">
        <span>{driver.name} LAP STATS</span>
        <span>{laps.length} LAPS</span>
      </div>

      {laps.length === 0 ? (
        <div className="strategy-stats__empty">NO COMPLETED LAPS</div>
      ) : (
        <>
          <div className="strategy-stats__summary">
            <div>
              <span>BEST</span>
              <strong>{formatLapTime(best.lap_time)}</strong>
            </div>
            <div>
              <span>LAST</span>
              <strong>{formatLapTime(latest.lap_time)}</strong>
            </div>
            <div>
              <span>AVG</span>
              <strong>{formatLapTime(average)}</strong>
            </div>
          </div>

          <div className="strategy-stats__tabs" role="tablist" aria-label="Lap statistics view">
            <button
              type="button"
              className={`strategy-stats__tab ${view === 'chart' ? 'strategy-stats__tab--active' : ''}`}
              onClick={() => setView('chart')}
              role="tab"
              aria-selected={view === 'chart'}
            >
              GRAPH
            </button>
            <button
              type="button"
              className={`strategy-stats__tab ${view === 'table' ? 'strategy-stats__tab--active' : ''}`}
              onClick={() => setView('table')}
              role="tab"
              aria-selected={view === 'table'}
            >
              TABLE
            </button>
          </div>

          <div className={`strategy-stats__content strategy-stats__content--${view}`}>
            {view === 'chart' ? (
              <>
                <LapTimeChart laps={laps} />

                <div className="strategy-stats__stints">
                  {stints.map((stint) => {
                    const stintBest = bestLap(stint.laps);
                    const firstLap = stint.laps[0];
                    const lastLap = stint.laps[stint.laps.length - 1];
                    return (
                      <div key={stint.stint} className="strategy-stats__stint">
                        <span
                          className="strategy-stats__stint-swatch"
                          style={{ backgroundColor: tireColor(stint.tire_compound) }}
                        />
                        <span>ST{stint.stint}</span>
                        <span>{stint.tire_compound}</span>
                        <span>L{firstLap.lap}-{lastLap.lap}</span>
                        <strong>{formatLapTime(stintBest.lap_time)}</strong>
                      </div>
                    );
                  })}
                </div>
              </>
            ) : (
              <div className="strategy-stats__table">
                <div className="strategy-stats__table-head">
                  <span>LAP</span>
                  <span>TIRE</span>
                  <span>TIME</span>
                  <span>DELTA</span>
                </div>
                {tableLaps.map((lap) => (
                  <div
                    key={`${lap.lap}-${lap.stint}-${lap.lap_time}`}
                    className={lap.pit_stop ? 'strategy-stats__lap strategy-stats__lap--pit' : 'strategy-stats__lap'}
                  >
                    <span>{lap.lap}</span>
                    <span>
                      <i style={{ backgroundColor: tireColor(lap.tire_compound) }} />
                      {lap.tire_compound}
                      {lap.pit_stop && <b>PIT</b>}
                    </span>
                    <span>{formatLapTime(lap.lap_time)}</span>
                    <span>{formatDelta(lap.lap_time - best.lap_time)}</span>
                  </div>
                ))}
              </div>
            )}
          </div>
        </>
      )}
    </section>
  );
}

export default function StrategyPanel({ drivers, pitWindowOpen = false, onPitCall, onPaceModeChange }) {
  const [tireChoices, setTireChoices] = useState({});
  const [statsDriverId, setStatsDriverId] = useState(null);

  useEffect(() => {
    if (!drivers?.length) {
      setStatsDriverId(null);
      return;
    }
    if (statsDriverId !== null && !drivers.some((driver) => driver.driver_id === statsDriverId)) {
      setStatsDriverId(null);
    }
  }, [drivers, statsDriverId]);

  useEffect(() => {
    if (statsDriverId === null) return undefined;

    const handleKeyDown = (event) => {
      if (event.key === 'Escape') {
        setStatsDriverId(null);
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [statsDriverId]);

  const selectedDriver = useMemo(
    () => drivers?.find((driver) => driver.driver_id === statsDriverId) || null,
    [drivers, statsDriverId],
  );

  if (!drivers || drivers.length === 0) {
    return (
      <div className="strategy-panel glass-panel">
        <div className="strategy-panel__header">STRATEGY</div>
        <div className="strategy-panel__empty">Waiting for driver data...</div>
      </div>
    );
  }

  const handlePit = (driverId) => {
    const tire = tireChoices[driverId] || 'MEDIUM';
    onPitCall(driverId, tire);
  };

  const handleDriverKeyDown = (event, driverId) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      setStatsDriverId(driverId);
    }
  };

  const statsModal = selectedDriver
    ? createPortal(
      <div
        className="strategy-stats-modal"
        role="presentation"
        onMouseDown={() => setStatsDriverId(null)}
      >
        <div
          className="strategy-stats-dialog glass-panel"
          role="dialog"
          aria-modal="true"
          aria-labelledby="strategy-stats-title"
          onMouseDown={(event) => event.stopPropagation()}
        >
          <div className="strategy-stats-dialog__top">
            <div>
              <span className="strategy-stats-dialog__eyebrow">DRIVER ANALYTICS</span>
              <h2 id="strategy-stats-title">{selectedDriver.full_name || selectedDriver.name}</h2>
            </div>
            <button
              type="button"
              className="strategy-stats-dialog__close"
              onClick={() => setStatsDriverId(null)}
              aria-label="Close lap statistics"
            >
              X
            </button>
          </div>
          <DriverLapStats driver={selectedDriver} />
        </div>
      </div>,
      document.body,
    )
    : null;

  return (
    <div className="strategy-panel glass-panel">
      <div className="strategy-panel__header">STRATEGY</div>

      {pitWindowOpen && (
        <div className="strategy-panel__pit-window">PIT WINDOW OPEN — SAFETY CAR</div>
      )}

      {drivers.map((driver) => (
        <div
          key={driver.driver_id}
          className={`strategy-driver ${statsDriverId === driver.driver_id ? 'strategy-driver--selected' : ''}`}
          role="button"
          tabIndex={0}
          onClick={() => setStatsDriverId(driver.driver_id)}
          onKeyDown={(event) => handleDriverKeyDown(event, driver.driver_id)}
        >
          {(() => {
            const tireLife = tireLifePercent(driver.tire_wear);
            const paceTransitionProgress = Math.max(
              0,
              Math.min(1, Number(driver.pace_mode_transition_progress ?? 1)),
            );
            const paceTransitioning = paceTransitionProgress < 0.999;
            const paceFrom = PACE_MODES.find(
              (mode) => mode.value === driver.pace_mode_from,
            )?.label || 'STD';
            const paceTarget = PACE_MODES.find(
              (mode) => mode.value === driver.pace_mode,
            )?.label || 'STD';
            return (
              <>
          <div className="strategy-driver__info">
            <span
              className="strategy-driver__bar"
              style={{ backgroundColor: driver.team_color }}
            />
            <span className="strategy-driver__name">{driver.name}</span>
            <span className="strategy-driver__pos">P{driver.position}</span>
          </div>

          <div className="strategy-driver__tire-row">
            <span className="strategy-driver__compound">{driver.tire_compound}</span>
            <span className="strategy-driver__age">{driver.tire_age} laps</span>
            <span
              className="strategy-driver__life"
              style={{ color: wearColor(driver.tire_wear) }}
            >
              {tireLife}%
            </span>
          </div>

          <div className="strategy-driver__wear">
            <div
              className="strategy-driver__wear-bar"
              style={{
                width: `${tireLife}%`,
                backgroundColor: wearColor(driver.tire_wear),
              }}
            />
          </div>

          <div
            className="strategy-driver__pace-modes"
            aria-label={`${driver.name} pace mode`}
            onClick={(event) => event.stopPropagation()}
            onKeyDown={(event) => event.stopPropagation()}
          >
            {PACE_MODES.map((mode) => {
              const selected = (driver.pace_mode || 'STANDARD') === mode.value;
              return (
                <button
                  key={mode.value}
                  type="button"
                  className={`strategy-driver__pace-btn ${selected ? 'strategy-driver__pace-btn--selected' : ''}`}
                  onClick={() => onPaceModeChange(driver.driver_id, mode.value)}
                  disabled={driver.in_pit || driver.retired || driver.finished}
                >
                  {mode.label}
                </button>
              );
            })}
          </div>

          {paceTransitioning && (
            <div
              className="strategy-driver__pace-transition"
              aria-live="polite"
              aria-label={`Pace transition ${Math.round(paceTransitionProgress * 100)} percent`}
            >
              <span>{paceFrom} → {paceTarget}</span>
              <div className="strategy-driver__pace-transition-track">
                <div
                  className="strategy-driver__pace-transition-fill"
                  style={{ width: `${paceTransitionProgress * 100}%` }}
                />
              </div>
              <b>{Math.round(paceTransitionProgress * 100)}%</b>
            </div>
          )}

          <div
            className="strategy-driver__actions"
            onClick={(event) => event.stopPropagation()}
            onKeyDown={(event) => event.stopPropagation()}
          >
            <select
              value={tireChoices[driver.driver_id] || 'MEDIUM'}
              onChange={(e) =>
                setTireChoices((prev) => ({
                  ...prev,
                  [driver.driver_id]: e.target.value,
                }))
              }
              disabled={driver.in_pit || driver.retired}
            >
              {TIRE_OPTIONS.map((t) => (
                <option key={t} value={t}>{t}</option>
              ))}
            </select>

            <button
              type="button"
              className={`strategy-driver__box-btn ${pitWindowOpen ? 'strategy-driver__box-btn--window' : ''}`}
              onClick={() => handlePit(driver.driver_id)}
              disabled={driver.in_pit || driver.retired}
            >
              BOX BOX
            </button>
          </div>
              </>
            );
          })()}
        </div>
      ))}

      {statsModal}
    </div>
  );
}
