import { useMemo } from 'react';
import './SectorTimingPanel.css';

function buildMiniSectors(sectors = []) {
  const major = sectors.slice(0, 3).map((sector, sectorIndex) => ({
    name: sector.name || `Sector ${sectorIndex + 1}`,
    sector: sectorIndex + 1,
    count: Math.max(1, Number(sector.mini_sector_count || 6)),
  }));
  if (major.length === 3) return major;
  return [1, 2, 3].map((sector) => ({
    name: `Sector ${sector}`,
    sector,
    count: 6,
  }));
}

function formatSplit(seconds) {
  const value = Number(seconds);
  return Number.isFinite(value) && value > 0 ? value.toFixed(3) : '—';
}

function formatDelta(seconds) {
  if (seconds === null || seconds === undefined || seconds === '') return '';
  const value = Number(seconds);
  if (!Number.isFinite(value)) return '';
  return value <= 0.0005 ? 'BEST' : `+${value.toFixed(3)}`;
}

export default function SectorTimingPanel({ positions, playerDriverIds, sectors }) {
  const majorSectors = useMemo(() => buildMiniSectors(sectors), [sectors]);
  const miniSectors = useMemo(
    () => majorSectors.flatMap((sector) => (
      Array.from({ length: sector.count }, (_, miniIndex) => ({
        sector: sector.sector,
        mini: miniIndex + 1,
      }))
    )),
    [majorSectors],
  );
  const playerSet = useMemo(() => new Set(playerDriverIds || []), [playerDriverIds]);
  const sorted = useMemo(
    () => [...(positions || [])].sort((a, b) => a.position - b.position),
    [positions],
  );
  const playerRows = sorted.filter((driver) => playerSet.has(driver.driver_id));
  const rows = (playerRows.length ? playerRows : sorted.slice(0, 2)).slice(0, 2);
  const measured = rows.length > 0 && rows.every(
    (driver) => driver.position === 1 || (
      driver.timing_gap_valid && driver.interval_timing_gap_valid
    ),
  );
  const miniGridStyle = {
    gridTemplateColumns: `repeat(${Math.max(1, miniSectors.length)}, minmax(4px, 1fr))`,
  };

  return (
    <section className="sector-timing glass-panel" aria-label="Sector timing comparison">
      <header className="sector-timing__header">
        <div>
          <span className="sector-timing__title">SECTOR TIMING</span>
          <span className="sector-timing__subtitle">LIVE + TIMING LOOPS</span>
        </div>
        <span className={`sector-timing__source ${measured ? 'is-measured' : 'is-estimated'}`}>
          {measured ? 'LIVE' : 'EST'}
        </span>
      </header>

      <div className="sector-timing__body">
        <div className="sector-timing__major-row">
          <span className="sector-timing__driver-spacer" />
          <div className="sector-timing__major-grid" style={miniGridStyle}>
            {majorSectors.map((sector) => (
              <span
                key={sector.sector}
                className={`sector-timing__major sector-timing__major--${sector.sector}`}
                style={{ gridColumn: `span ${sector.count}` }}
              >
                S{sector.sector}
              </span>
            ))}
          </div>
        </div>

        {rows.map((driver) => {
          const splits = driver.mini_sector_splits || [];
          const statuses = driver.mini_sector_statuses || [];
          const activeIndex = Math.max(0, Number(driver.current_timing_loop || 1) - 1);
          return (
            <div className="sector-timing__driver-row" key={driver.driver_id}>
              <div className="sector-timing__driver">
                <span
                  className="sector-timing__team"
                  style={{ backgroundColor: driver.team_color || '#777' }}
                />
                <span className="sector-timing__code">{driver.name}</span>
                <span className="sector-timing__last">
                  {formatSplit(driver.last_mini_sector_time)}
                </span>
                <span className="sector-timing__delta">
                  {formatDelta(driver.last_mini_sector_delta_to_best)}
                </span>
              </div>
              <div className="sector-timing__mini-grid" style={miniGridStyle}>
                {miniSectors.map((mini, index) => {
                  const split = splits[index];
                  const status = statuses[index] || 'pending';
                  return (
                    <span
                      key={`${mini.sector}-${mini.mini}`}
                      className={`sector-timing__mini is-${status} ${index === activeIndex ? 'is-active' : ''}`}
                      title={`S${mini.sector} M${mini.mini}: ${formatSplit(split)}`}
                    />
                  );
                })}
              </div>
            </div>
          );
        })}
      </div>

      <footer className="sector-timing__footer">
        GAP/INT: live movement anchored to timing loops · <span>~</span> without an anchor
      </footer>
    </section>
  );
}
