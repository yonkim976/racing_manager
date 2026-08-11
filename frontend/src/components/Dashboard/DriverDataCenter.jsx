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

function formatNumber(value, digits = 1, fallback = '—') {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : fallback;
}

function formatPercent(value, digits = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? `${(number * 100).toFixed(digits)}%` : '—';
}

function axleSlipLabel(value) {
  const slip = Number(value);
  if (!Number.isFinite(slip) || Math.abs(slip) < 0.0005) return 'ROLLING';
  return `${slip < 0 ? 'LOCK ' : 'SPIN +'}${Math.abs(slip * 100).toFixed(1)}%`;
}

function SummaryItem({ label, value, tone = '' }) {
  return (
    <div className={`driver-data-center__summary-item ${tone ? `is-${tone}` : ''}`}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function ConditionMetric({ label, value, detail = '', tone = '' }) {
  return (
    <div className={`driver-data-center__metric ${tone ? `is-${tone}` : ''}`}>
      <span>{label}</span>
      <strong>{value}</strong>
      {detail && <small>{detail}</small>}
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
  const isAbstractMode = driver.source_mode === 'abstract';

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
            {isAbstractMode ? (
              <SummaryItem label="PACE MODE" value={String(driver.pace_mode || 'STANDARD').toUpperCase()} />
            ) : (
              <SummaryItem label="SPEED" value={`${Math.round(Number(driver.speed_kph || 0))} km/h`} />
            )}
            <SummaryItem label="LAST LAP" value={formatLapTime(driver.last_lap_time)} />
            <SummaryItem label="BEST LAP" value={formatLapTime(driver.best_lap_time)} tone="best" />
            <SummaryItem label="TIRE" value={`${driver.tire_compound} · ${tireLifePercent(driver.tire_wear)}%`} />
            <SummaryItem label="CURRENT" value={`S${driver.current_sector || 1} · M${driver.current_mini_sector || 1}`} />
          </section>

          <section className="driver-data-center__condition" aria-label="Live car condition">
            <header>
              <div>
                <span>LIVE CAR CONDITION</span>
                <small>{isAbstractMode ? 'LOGICAL RACECRAFT STATE' : 'PHYSICS & RESOURCE STATE'}</small>
              </div>
              <strong>{String(
                isAbstractMode
                  ? driver.attack_mode || driver.maneuver || 'clear'
                  : driver.handling_state || 'stable'
              ).toUpperCase()}</strong>
            </header>

            {isAbstractMode ? (
              <div className="driver-data-center__condition-group">
                <h3>RACECRAFT &amp; STRATEGY</h3>
                <div className="driver-data-center__metric-grid">
                  <ConditionMetric
                    label="DRS"
                    value={driver.drs_active ? 'ACTIVE' : 'CLOSED'}
                    detail="detection-line eligibility"
                    tone={driver.drs_active ? 'best' : ''}
                  />
                  <ConditionMetric
                    label="SLIPSTREAM"
                    value={formatPercent(driver.tow_strength || 0)}
                    detail="logical straight-line tow"
                  />
                  <ConditionMetric
                    label="DIRTY AIR"
                    value={formatPercent(driver.dirty_air_strength || 0)}
                    detail={driver.dirty_air_active ? 'corner pace affected' : 'clear airflow'}
                    tone={driver.dirty_air_active ? 'warning' : ''}
                  />
                  <ConditionMetric
                    label="TRAFFIC"
                    value={String(driver.maneuver || 'clear').toUpperCase()}
                    detail={`attack ${String(driver.attack_mode || 'none').toUpperCase()}`}
                  />
                  <ConditionMetric
                    label="DRS TRAIN"
                    value={driver.drs_train_size >= 3 ? `${driver.drs_train_size} CARS` : 'NONE'}
                    detail={driver.drs_train_size >= 3
                      ? `logical order ${Number(driver.drs_train_position || 0) + 1}`
                      : 'no contiguous train'}
                    tone={driver.drs_train_size >= 3 ? 'warning' : ''}
                  />
                  <ConditionMetric
                    label="PACE MODE"
                    value={String(driver.pace_mode || 'STANDARD').toUpperCase()}
                    detail="manager command authority"
                  />
                  <ConditionMetric
                    label="TIRE LIFE"
                    value={`${tireLifePercent(driver.tire_wear)}%`}
                    detail={`${driver.tire_compound || '—'} · ${driver.tire_age || 0} laps`}
                  />
                  <ConditionMetric
                    label="PIT STATE"
                    value={driver.in_pit ? String(driver.pit_phase || 'PIT').toUpperCase() : 'TRACK'}
                    detail={`${driver.pit_count || 0} stops completed`}
                  />
                  <ConditionMetric
                    label="DAMAGE"
                    value={formatPercent(driver.damage_level || 0)}
                    detail={driver.retired ? 'retired' : 'logical incident state'}
                    tone={Number(driver.damage_level || 0) > 0.2 ? 'warning' : ''}
                  />
                </div>
              </div>
            ) : (<>
            <div className="driver-data-center__condition-group">
              <h3>TYRES &amp; FUEL</h3>
              <div className="driver-data-center__metric-grid">
                <ConditionMetric
                  label="TIRE LIFE"
                  value={`${tireLifePercent(driver.tire_wear)}%`}
                  detail={`${driver.tire_compound || '—'} · ${driver.tire_age || 0} laps`}
                />
                <ConditionMetric
                  label="FRONT TYRES"
                  value={`${formatNumber(
                    driver.front_tire_surface_temperature_c
                      ?? driver.tire_surface_temperature_c,
                  )}°C`}
                  detail={`core ${formatNumber(
                    driver.front_tire_core_temperature_c
                      ?? driver.tire_core_temperature_c,
                  )}°C · grip ${formatPercent(
                    driver.front_tire_thermal_grip
                      ?? driver.tire_thermal_grip
                      ?? 1,
                  )}`}
                />
                <ConditionMetric
                  label="REAR TYRES"
                  value={`${formatNumber(
                    driver.rear_tire_surface_temperature_c
                      ?? driver.tire_surface_temperature_c,
                  )}°C`}
                  detail={`core ${formatNumber(
                    driver.rear_tire_core_temperature_c
                      ?? driver.tire_core_temperature_c,
                  )}°C · grip ${formatPercent(
                    driver.rear_tire_thermal_grip
                      ?? driver.tire_thermal_grip
                      ?? 1,
                  )}`}
                />
                <ConditionMetric
                  label="GRIP INDEX"
                  value={formatPercent(
                    driver.tire_lateral_grip_index
                      ?? (Number(driver.tire_lateral_grip) || 0) / 1.025,
                  )}
                  detail={`C3=100 · traction ${formatPercent(
                    driver.tire_traction_grip_index
                      ?? (Number(driver.tire_traction_grip) || 0) / 1.025,
                  )} · brake ${formatPercent(
                    driver.tire_braking_grip_index
                      ?? (Number(driver.tire_braking_grip) || 0) / 1.015,
                  )}`}
                />
                <ConditionMetric
                  label="FUEL"
                  value={`${formatNumber(driver.fuel_mass_kg)} kg`}
                  detail={`${formatNumber(driver.fuel_laps_remaining, 1)} laps remaining`}
                />
                <ConditionMetric
                  label="FUEL USED"
                  value={`${formatNumber(driver.fuel_burned_kg)} kg`}
                  detail={`vehicle ${formatNumber(driver.vehicle_mass_kg, 0)} kg`}
                />
              </div>
            </div>

            <div className="driver-data-center__condition-group">
              <h3>BRAKES, LOAD &amp; SLIP</h3>
              <div className="driver-data-center__metric-grid">
                <ConditionMetric
                  label="FRONT BRAKES"
                  value={`${formatNumber(driver.front_brake_temperature_c, 0)}°C`}
                  detail={`${formatNumber((Number(driver.applied_brake_force_n) || 0) / 1000, 1)} kN applied`}
                />
                <ConditionMetric
                  label="REAR BRAKES"
                  value={`${formatNumber(driver.rear_brake_temperature_c, 0)}°C`}
                  detail={`effectiveness ${formatPercent(driver.brake_fade_factor ?? 1)}`}
                  tone={Number(driver.brake_fade_factor ?? 1) < 0.97 ? 'warning' : ''}
                />
                <ConditionMetric
                  label="FRONT LOAD"
                  value={`${formatNumber((Number(driver.front_normal_load_n) || 0) / 1000, 1)} kN`}
                  detail={`transfer ${formatNumber((Number(driver.longitudinal_load_transfer_n) || 0) / 1000, 1)} kN`}
                />
                <ConditionMetric
                  label="REAR LOAD"
                  value={`${formatNumber((Number(driver.rear_normal_load_n) || 0) / 1000, 1)} kN`}
                  detail={`grip use ${formatPercent(driver.grip_utilization)}`}
                />
                <ConditionMetric
                  label="FRONT AXLE"
                  value={axleSlipLabel(driver.front_axle_slip_ratio)}
                  detail={`${formatNumber(driver.front_wheel_speed_rad_s, 0)} rad/s`}
                  tone={Math.abs(Number(driver.front_axle_slip_ratio) || 0) >= 0.05 ? 'warning' : ''}
                />
                <ConditionMetric
                  label="REAR AXLE"
                  value={axleSlipLabel(driver.rear_axle_slip_ratio)}
                  detail={`${formatNumber(driver.rear_wheel_speed_rad_s, 0)} rad/s`}
                  tone={Math.abs(Number(driver.rear_axle_slip_ratio) || 0) >= 0.05 ? 'warning' : ''}
                />
              </div>
            </div>
            </>)}
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
