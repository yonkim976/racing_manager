import './DevRaceControl.css';

function formatPhaseStatus(racePhase, safetyCarStage, remainingSeconds, remainingLaps) {
  if (racePhase === 'vsc') {
    return `VSC · ${Math.max(0, Math.ceil(remainingSeconds || 0))}s`;
  }
  if (racePhase === 'sc') {
    const labels = {
      deploying: 'DEPLOYING',
      collecting: 'CATCHING FIELD',
      queued: 'FIELD QUEUED',
      unlapping: 'LAPPED CARS OVERTAKING',
      in_this_lap: 'IN THIS LAP',
      restart: 'RESTART',
    };
    if (labels[safetyCarStage]) return `SAFETY CAR · ${labels[safetyCarStage]}`;
    const laps = Math.max(0, Number(remainingLaps) || 0);
    return `SAFETY CAR · ${laps} ${laps === 1 ? 'LAP' : 'LAPS'}`;
  }
  return 'GREEN FLAG';
}

export default function DevRaceControl({
  connected,
  racePhase = 'green',
  safetyCarStage = 'inactive',
  remainingSeconds = 0,
  remainingLaps = 0,
  physicsHz = 50,
  onSetPhase,
}) {
  const controls = [
    { phase: 'vsc', label: 'VSC', title: 'Force Virtual Safety Car' },
    { phase: 'sc', label: 'SC', title: 'Force full Safety Car' },
    {
      phase: 'green',
      label: racePhase === 'sc' ? 'SC IN' : 'GREEN',
      title: racePhase === 'sc'
        ? 'Call the Safety Car into the pits'
        : 'End the active race-control period',
    },
  ];

  return (
    <section className="dev-race-control glass-panel" data-testid="dev-race-control">
      <header className="dev-race-control__header">
        <span>DEV RACE CONTROL</span>
        <span className="dev-race-control__badge">LOCAL</span>
      </header>
      <div className={`dev-race-control__status dev-race-control__status--${racePhase}`}>
        <span className="dev-race-control__status-dot" aria-hidden="true" />
        <strong>{formatPhaseStatus(racePhase, safetyCarStage, remainingSeconds, remainingLaps)}</strong>
      </div>
      <div className="dev-race-control__buttons" role="group" aria-label="Development race phase">
        {controls.map((control) => (
          <button
            key={control.phase}
            type="button"
            className={racePhase === control.phase ? 'dev-race-control__button dev-race-control__button--active' : 'dev-race-control__button'}
            onClick={() => onSetPhase(control.phase)}
            disabled={!connected
              || racePhase === control.phase
              || (racePhase === 'sc' && control.phase === 'vsc')
              || (control.phase === 'green' && ['in_this_lap', 'restart'].includes(safetyCarStage))}
            title={control.title}
            data-testid={`dev-phase-${control.phase}`}
          >
            {control.label}
          </button>
        ))}
      </div>
      <div className="dev-race-control__physics">
        <span>PHYSICS V2 · {physicsHz}HZ</span>
      </div>
    </section>
  );
}
