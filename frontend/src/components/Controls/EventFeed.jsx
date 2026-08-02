import { useEffect, useMemo, useRef, useState } from 'react';
import PerformanceDiagnosticsModal from './PerformanceDiagnosticsModal';
import './EventFeed.css';

const KEY_EVENT_TYPES = new Set([
  'lockup',
  'spin',
  'dnf',
  'minor_contact',
  'incident',
  'retirement',
  'hazard_cleared',
  'run_wide',
  'traction_loss',
  'wheelspin',
  'forced_wide',
  'side_by_side',
  'overtake_abort',
  'sc_start',
  'sc_end',
  'vsc_start',
  'vsc_end',
]);

const PLAYER_COMMAND_TYPES = new Set(['command_ack', 'command_error']);

function shouldShowFocusedEvent(evt, playerDriverCodes) {
  if (!evt) return false;
  if (PLAYER_COMMAND_TYPES.has(evt.type)) return true;
  if (evt.driver && playerDriverCodes.has(evt.driver)) return true;
  return KEY_EVENT_TYPES.has(evt.type);
}

export default function EventFeed({
  events,
  playerDriverCodes = [],
  performanceStats = null,
  onResetPerformanceWindow,
}) {
  const listRef = useRef(null);
  const [language, setLanguage] = useState('en');
  const [feedMode, setFeedMode] = useState('all');
  const [showPerformanceDiagnostics, setShowPerformanceDiagnostics] = useState(false);
  const playerDriverCodeSet = useMemo(
    () => new Set(playerDriverCodes || []),
    [playerDriverCodes],
  );
  const visibleEvents = useMemo(() => {
    if (feedMode === 'all') return events || [];
    return (events || []).filter((evt) => shouldShowFocusedEvent(evt, playerDriverCodeSet));
  }, [events, feedMode, playerDriverCodeSet]);

  useEffect(() => {
    if (listRef.current) {
      listRef.current.scrollTop = listRef.current.scrollHeight;
    }
  }, [visibleEvents]);

  return (
    <div className="event-feed glass-panel">
      <div className="event-feed__header">
        <span>RACE FEED</span>
        <div className="event-feed__controls">
          <button
            type="button"
            className="event-feed__diagnostics"
            onClick={() => setShowPerformanceDiagnostics(true)}
            aria-label="Open memory and rendering diagnostics"
          >
            <span
              className={`event-feed__diagnostics-dot ${
                Number(performanceStats?.jsHeapTrendMbPerMin) > 2
                  ? 'is-warning'
                  : ''
              }`}
            />
            DIAG
          </button>
          <div className="event-feed__mode" aria-label="Race feed mode">
            <button
              type="button"
              className={feedMode === 'all' ? 'is-active' : ''}
              onClick={() => setFeedMode('all')}
            >
              ALL
            </button>
            <button
              type="button"
              className={feedMode === 'focus' ? 'is-active' : ''}
              onClick={() => setFeedMode('focus')}
            >
              FOCUS
            </button>
          </div>
          <div className="event-feed__language" aria-label="Race feed language">
            <button
              type="button"
              className={language === 'en' ? 'is-active' : ''}
              onClick={() => setLanguage('en')}
            >
              EN
            </button>
            <button
              type="button"
              className={language === 'ko' ? 'is-active' : ''}
              onClick={() => setLanguage('ko')}
            >
              KO
            </button>
          </div>
        </div>
      </div>
      <div className="event-feed__list" ref={listRef}>
        {!visibleEvents.length && (
          <div className="event-feed__empty">
            {feedMode === 'focus' ? 'No focused events yet' : 'No events yet'}
          </div>
        )}
        {visibleEvents.map((evt, i) => {
          const message = language === 'ko' ? evt.message_ko || evt.message : evt.message;
          return (
            <div
              key={evt.event_id || `${evt.type}-${evt.driver}-${evt.message}-${i}`}
              className={`event-feed__item event-feed__item--${evt.type}`}
            >
              {evt.driver && <span className="event-feed__driver">{evt.driver}</span>}
              <span className="event-feed__message">{message}</span>
            </div>
          );
        })}
      </div>
      {showPerformanceDiagnostics && (
        <PerformanceDiagnosticsModal
          stats={performanceStats}
          onClose={() => setShowPerformanceDiagnostics(false)}
          onReset={onResetPerformanceWindow}
        />
      )}
    </div>
  );
}
