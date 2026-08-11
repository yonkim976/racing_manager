import './AbstractRaceResult.css';

function formatEventType(value) {
  return value.replaceAll('_', ' ');
}

export default function AbstractRaceResult({ result, onBack, isDisposing }) {
  const summary = result?.abstract_result_summary;
  const gridByDriver = new Map((summary?.grid || []).map((entry) => [entry.driver_id, entry]));
  const classification = summary?.classification
    || (summary?.finish_order || []).map((driverId, index) => ({
      position: index + 1,
      driver_id: driverId,
      status: 'finished',
    }));
  const eventTypes = Object.entries(summary?.event_types || {})
    .sort(([, left], [, right]) => right - left);

  return (
    <div className="abstract-result carbon-bg">
      <div className="abstract-result__panel glass-panel">
        <header className="abstract-result__header">
          <div>
            <span className="abstract-result__eyebrow">ABSTRACT RESULT</span>
            <h1>{result?.circuit?.name || 'Race Result'}</h1>
            <p>Deterministic result engine · {summary?.total_laps || result?.circuit?.total_laps || 0} laps</p>
          </div>
          <button type="button" onClick={onBack} disabled={isDisposing}>
            {isDisposing ? 'DISPOSING…' : 'NEW RACE'}
          </button>
        </header>

        <div className="abstract-result__meta">
          <span>Seed <strong>{String(result?.session_seed ?? '—')}</strong></span>
          <span>Events <strong>{summary?.event_count ?? 0}</strong></span>
          <span>Hash <code>{summary?.canonical_result_hash || '—'}</code></span>
        </div>

        <section className="abstract-result__section">
          <div className="abstract-result__section-title">FINAL CLASSIFICATION</div>
          <div className="abstract-result__table abstract-result__table--header">
            <span>POS</span>
            <span>DRIVER</span>
            <span>TEAM</span>
            <span>START</span>
            <span>STATUS</span>
          </div>
          <ol className="abstract-result__classification">
            {classification.map((resultEntry, index) => {
              const driverId = resultEntry.driver_id;
              const entry = gridByDriver.get(driverId) || {};
              const retired = resultEntry.status === 'retired';
              return (
                <li key={driverId} className="abstract-result__table">
                  <span className="abstract-result__position">P{resultEntry.position || index + 1}</span>
                  <span className="abstract-result__driver">
                    <i style={{ backgroundColor: entry.team_color || '#666' }} />
                    <strong>{entry.name || driverId}</strong>
                    <small>{entry.abbreviation || ''}</small>
                  </span>
                  <span>{entry.team || '—'}</span>
                  <span>P{entry.position || '—'}</span>
                  <span>{retired ? `DNF · ${resultEntry.retirement_reason || 'retired'}` : 'FINISHED'}</span>
                </li>
              );
            })}
          </ol>
        </section>

        <section className="abstract-result__section abstract-result__events">
          <div className="abstract-result__section-title">LOGICAL EVENTS</div>
          <div className="abstract-result__event-list">
            {eventTypes.map(([eventType, count]) => (
              <span key={eventType}>
                {formatEventType(eventType)} <strong>{count}</strong>
              </span>
            ))}
          </div>
        </section>
      </div>
    </div>
  );
}
