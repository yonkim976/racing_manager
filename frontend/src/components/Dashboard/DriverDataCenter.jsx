import { useEffect, useMemo } from 'react';
import { createPortal } from 'react-dom';
import SectorTimingPanel from './SectorTimingPanel';
import './DriverDataCenter.css';

function formatLapTime(seconds) {
  const value = Number(seconds);
  if (!Number.isFinite(value) || value <= 0) return '—';
  const minutes = Math.floor(value / 60);
  return `${minutes}:${(value % 60).toFixed(3).padStart(6, '0')}`;
}

function formatSectorTime(seconds) {
  const value = Number(seconds);
  return Number.isFinite(value) && value > 0 ? value.toFixed(3) : '—';
}

function tireLifePercent(wear) {
  return Math.max(0, Math.round((1 - Math.min(Number(wear) || 0, 1)) * 100));
}

function SummaryItem({ label, value, tone = '' }) {
  return (
    <div className={`driver-data-center__summary-item ${tone ? `is-${tone}` : ''}`}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

export default function DriverDataCenter({ driver, sectors, onClose }) {
  useEffect(() => {
    const handleKeyDown = (event) => {
      if (event.key === 'Escape') onClose();
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [onClose]);

  const completedLaps = useMemo(
    () => [...(driver.lap_history || [])].reverse(),
    [driver.lap_history],
  );
  const gapSource = driver.position === 1 || driver.timing_gap_source === 'live'
    ? 'LIVE'
    : 'EST';
  const gap = driver.position === 1 ? 'LEADER' : (driver.gap || '—');

  return createPortal(
    <div
      className="driver-data-center-modal"
      role="presentation"
      onMouseDown={onClose}
    >
      <div
        className="driver-data-center glass-panel"
        role="dialog"
        aria-modal="true"
        aria-labelledby="driver-data-center-title"
        onMouseDown={(event) => event.stopPropagation()}
      >
        <header className="driver-data-center__top">
          <div className="driver-data-center__identity">
            <span
              className="driver-data-center__team-bar"
              style={{ backgroundColor: driver.team_color || '#777' }}
            />
            <div>
              <span className="driver-data-center__eyebrow">DRIVER DATA CENTER</span>
              <h2 id="driver-data-center-title">
                {driver.full_name || driver.name}
                <small>{driver.team}</small>
              </h2>
            </div>
          </div>
          <button
            type="button"
            className="driver-data-center__close"
            onClick={onClose}
            aria-label="Close driver data center"
          >
            X
          </button>
        </header>

        <div className="driver-data-center__scroll">
          <section className="driver-data-center__summary" aria-label="Driver live summary">
            <SummaryItem label="POSITION" value={`P${driver.position}`} />
            <SummaryItem label={`GAP · ${gapSource}`} value={gap} tone={gapSource === 'EST' ? 'estimated' : ''} />
            <SummaryItem label="INTERVAL" value={driver.position === 1 ? '—' : (driver.interval || '—')} />
            <SummaryItem label="SPEED" value={`${Math.round(Number(driver.speed_kph || 0))} km/h`} />
            <SummaryItem label="LAST LAP" value={formatLapTime(driver.last_lap_time)} />
            <SummaryItem label="BEST LAP" value={formatLapTime(driver.best_lap_time)} tone="best" />
            <SummaryItem label="TIRE" value={`${driver.tire_compound} · ${tireLifePercent(driver.tire_wear)}%`} />
            <SummaryItem label="CURRENT" value={`S${driver.current_sector || 1} · M${driver.current_mini_sector || 1}`} />
          </section>

          <SectorTimingPanel
            positions={[driver]}
            playerDriverIds={[driver.driver_id]}
            sectors={sectors}
          />

          <section className="driver-data-center__laps">
            <header>
              <span>COMPLETED LAPS</span>
              <span>{completedLaps.length} LAPS</span>
            </header>
            <div className="driver-data-center__lap-table">
              <div className="driver-data-center__lap-head">
                <span>LAP</span>
                <span>TIRE</span>
                <span>S1</span>
                <span>S2</span>
                <span>S3</span>
                <span>LAP TIME</span>
                <span>MINI</span>
              </div>
              {completedLaps.length === 0 ? (
                <div className="driver-data-center__empty">NO COMPLETED LAPS</div>
              ) : completedLaps.map((lap) => (
                <div className="driver-data-center__lap" key={`${lap.lap}-${lap.stint}-${lap.lap_time}`}>
                  <span>{lap.lap}</span>
                  <span>{lap.tire_compound}{lap.pit_stop ? ' · PIT' : ''}</span>
                  <span>{formatSectorTime(lap.sector_times?.[0])}</span>
                  <span>{formatSectorTime(lap.sector_times?.[1])}</span>
                  <span>{formatSectorTime(lap.sector_times?.[2])}</span>
                  <strong>{formatLapTime(lap.lap_time)}</strong>
                  <span>{lap.mini_sector_times?.length || 0}</span>
                </div>
              ))}
            </div>
          </section>
        </div>
      </div>
    </div>,
    document.body,
  );
}
