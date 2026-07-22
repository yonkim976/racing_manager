import React, { useMemo, useState } from 'react';
import './TimingBoard.css';

/**
 * Format lap time from seconds to M:SS.mmm format
 */
function formatLapTime(seconds) {
  if (!seconds || seconds <= 0) return '—';
  const mins = Math.floor(seconds / 60);
  const secs = (seconds % 60).toFixed(3);
  const paddedSecs = secs.padStart(6, '0');
  return mins > 0 ? `${mins}:${paddedSecs}` : paddedSecs;
}

/**
 * Get CSS class name for tire compound
 */
function getTireClass(compound) {
  switch (compound?.toUpperCase()) {
    case 'SOFT': return 'tire-soft';
    case 'MEDIUM': return 'tire-medium';
    case 'HARD': return 'tire-hard';
    case 'INTER':
    case 'INTERMEDIATE': return 'tire-inter';
    case 'WET': return 'tire-wet';
    default: return 'tire-medium';
  }
}

/**
 * Get short tire label
 */
function getTireLabel(compound) {
  switch (compound?.toUpperCase()) {
    case 'SOFT': return 'S';
    case 'MEDIUM': return 'M';
    case 'HARD': return 'H';
    case 'INTER':
    case 'INTERMEDIATE': return 'I';
    case 'WET': return 'W';
    default: return '?';
  }
}

function tireLifePercent(wear) {
  return Math.max(0, Math.round((1 - Math.min(wear || 0, 1)) * 100));
}

function getStatusBadge(driver, isFinished) {
  if (driver.in_pit) return 'PIT';
  if (driver.local_yellow_active && !isFinished) return 'YEL';
  if (driver.maneuver_group_size >= 4 && !isFinished) return '4W';
  if (driver.maneuver_group_size === 3 && !isFinished) return '3W';
  if (driver.side_by_side_active && !isFinished) return 'SBS';
  if (driver.drs_active && !isFinished) return 'DRS';
  if (driver.dirty_air_active && !driver.drs_active && !isFinished) return 'AIR';
  return '';
}

function formatPitCell(driver) {
  if (!driver.in_pit) return driver.pit_count || 0;

  const elapsed = (driver.pit_elapsed || 0).toFixed(1);
  if (driver.pit_phase === 'in') return `IN ${elapsed}s`;
  if (driver.pit_phase === 'stop') return `BOX ${elapsed}s`;
  if (driver.pit_phase === 'out') return `OUT ${elapsed}s`;
  return `${elapsed}s`;
}

function formatSpeed(driver) {
  if (driver.retired || driver.finished) return '—';
  const speed = Number(driver.speed_kph || 0);
  if (!Number.isFinite(speed) || speed <= 0) return '—';
  return Math.round(speed);
}

/**
 * F1 TV-style timing board showing all drivers in position order.
 */
export default function TimingBoard({ positions, playerDriverIds }) {
  const [gapMode, setGapMode] = useState('gap');
  const playerSet = useMemo(
    () => new Set(playerDriverIds || []),
    [playerDriverIds]
  );
  const isGapMode = gapMode === 'gap';

  if (!positions || positions.length === 0) {
    return (
      <div className="timing-board glass-panel">
        <div className="timing-board__header">
          <span className="timing-board__title">LIVE TIMING</span>
        </div>
        <div className="timing-board__empty">Waiting for race data...</div>
      </div>
    );
  }

  const sorted = [...positions].sort((a, b) => a.position - b.position);

  return (
    <div className="timing-board glass-panel">
      <div className="timing-board__header">
        <span className="timing-board__title">LIVE TIMING</span>
        <span className="timing-board__driver-count">{sorted.length} drivers</span>
      </div>

      <div className="timing-board__columns">
        <span className="col-pos">POS</span>
        <span className="col-driver">DRIVER</span>
        <span className="col-status">STS</span>
        <button
          className="timing-board__gap-toggle"
          type="button"
          onClick={() => setGapMode(isGapMode ? 'interval' : 'gap')}
          aria-label={`Show ${isGapMode ? 'interval to car ahead' : 'gap to leader'}`}
        >
          <span className={isGapMode ? 'is-active' : ''}>GAP</span>
          <span className={!isGapMode ? 'is-active' : ''}>INT</span>
        </button>
        <span className="col-last">LAST</span>
        <span className="col-speed">SPD</span>
        <span className="col-tire">TIRE</span>
        <span className="col-pit">PIT</span>
      </div>

      <div className="timing-board__list">
        {sorted.map((driver, index) => {
          const isPlayer = playerSet.has(driver.driver_id);
          const isLeader = index === 0;
          const isRetired = driver.retired;
          const isFinished = driver.finished;
          const tireLife = tireLifePercent(driver.tire_wear);
          const statusBadge = getStatusBadge(driver, isFinished);

          return (
            <div
              key={driver.driver_id}
              className={`timing-row ${isPlayer ? 'timing-row--player' : ''} ${isLeader ? 'timing-row--leader' : ''} ${isRetired ? 'timing-row--retired' : ''} ${isFinished ? 'timing-row--finished' : ''} ${driver.in_pit ? 'timing-row--pit' : ''}`}
              style={{
                '--team-color': driver.team_color || '#666',
                '--row-index': index,
              }}
            >
              <span className="timing-row__pos">
                <span className="timing-row__pos-num">{driver.position}</span>
              </span>

              <span className="timing-row__driver">
                <span
                  className="timing-row__team-bar"
                  style={{ backgroundColor: driver.team_color }}
                />
                <span className="timing-row__name">{driver.name}</span>
                <span
                  className="timing-row__sector"
                  title={`Sector ${driver.current_sector || 1}, mini-sector ${driver.current_mini_sector || 1}`}
                >
                  S{driver.current_sector || 1} M{driver.current_mini_sector || 1}
                </span>
              </span>

              <span className="timing-row__status">
                {statusBadge && (
                  <span className={`timing-row__status-badge timing-row__status-badge--${statusBadge.toLowerCase()}`}>
                    {statusBadge}
                  </span>
                )}
              </span>

              <span
                className="timing-row__gap"
                title={driver.timing_gap_valid ? 'Measured at timing loop' : 'Provisional until next common timing loop'}
              >
                {isFinished ? (
                  <span className="timing-row__gap-finished">FIN</span>
                ) : isGapMode && driver.gap === 'LEADER' ? (
                  <span className="timing-row__gap-leader">LEADER</span>
                ) : (
                  <span>{isGapMode ? driver.gap : driver.interval || '—'}</span>
                )}
              </span>

              <span className="timing-row__last">
                {formatLapTime(driver.last_lap_time)}
              </span>

              <span className={driver.drs_active ? 'timing-row__speed timing-row__speed--drs' : 'timing-row__speed'}>
                {formatSpeed(driver)}
              </span>

              <span className="timing-row__tire">
                <span className={`tire-badge ${getTireClass(driver.tire_compound)}`}>
                  {getTireLabel(driver.tire_compound)}
                </span>
                <span className={tireLife <= 20 ? 'tire-life tire-life--critical' : 'tire-life'}>
                  {tireLife}%
                </span>
              </span>

              <span className="timing-row__pit-count">
                {formatPitCell(driver)}
              </span>
            </div>
          );
        })}
      </div>
    </div>
  );
}
