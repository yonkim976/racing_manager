import './SpeedControl.css';

const SPEEDS = [1, 2, 3];

export default function SpeedControl({
  connected,
  speedMultiplier = 1,
  paused = false,
  onSpeed,
  onPause,
  onResume,
}) {
  return (
    <div className="speed-control glass-panel">
      <div className="speed-control__header">
        <span>RACE CONTROL</span>
        <span className={`speed-control__status ${paused ? 'speed-control__status--paused' : ''}`}>
          {paused ? 'PAUSED' : `${speedMultiplier}x`}
        </span>
      </div>

      <div className="speed-control__speeds">
        {SPEEDS.map((s) => (
          <button
            key={s}
            type="button"
            className={`speed-control__btn ${speedMultiplier === s && !paused ? 'speed-control__btn--active' : ''}`}
            onClick={() => onSpeed(s)}
            disabled={!connected}
          >
            {s}x
          </button>
        ))}
      </div>

      <div className="speed-control__pause-row">
        <button
          type="button"
          className={`speed-control__pause ${paused ? 'speed-control__pause--active' : ''}`}
          onClick={onPause}
          disabled={!connected || paused}
        >
          ⏸ Pause
        </button>
        <button
          type="button"
          className={`speed-control__resume ${!paused ? 'speed-control__resume--active' : ''}`}
          onClick={onResume}
          disabled={!connected || !paused}
        >
          ▶ Resume
        </button>
      </div>
    </div>
  );
}
