import { useEffect } from 'react';
import { createPortal } from 'react-dom';
import './PerformanceDiagnosticsModal.css';

function finiteNumber(value) {
  if (value === null || value === undefined || value === '') return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function formatNumber(value, digits = 0, fallback = '—') {
  const number = finiteNumber(value);
  return number === null ? fallback : number.toFixed(digits);
}

function formatSigned(value, digits = 1) {
  const number = finiteNumber(value);
  if (number === null) return 'COLLECTING';
  return `${number >= 0 ? '+' : ''}${number.toFixed(digits)} MB/MIN`;
}

function memoryAssessment(stats) {
  const heap = finiteNumber(stats?.jsHeapMb);
  const trend = finiteNumber(stats?.jsHeapTrendMbPerMin);
  if (heap === null) {
    return {
      tone: 'neutral',
      label: 'UNAVAILABLE',
      detail: '이 브라우저는 JavaScript heap 계측값을 제공하지 않습니다.',
    };
  }
  if (trend === null) {
    return {
      tone: 'neutral',
      label: 'COLLECTING',
      detail: '최소 30초 동안 표본을 모은 뒤 증가 속도를 표시합니다.',
    };
  }
  if (trend > 2) {
    return {
      tone: 'danger',
      label: 'GROWING',
      detail: '지속적인 증가입니다. 5분 이상 같은 기울기가 유지되는지 확인해야 합니다.',
    };
  }
  if (trend > 0.5) {
    return {
      tone: 'warning',
      label: 'WATCH',
      detail: '완만한 증가입니다. 가비지 컬렉션 뒤 다시 내려오는지 관찰해야 합니다.',
    };
  }
  return {
    tone: 'stable',
    label: 'STABLE',
    detail: '현재 JavaScript heap 증가 속도는 안정 범위입니다.',
  };
}

function Metric({ label, value, detail = '' }) {
  return (
    <div className="performance-modal__metric">
      <span>{label}</span>
      <strong>{value}</strong>
      {detail && <small>{detail}</small>}
    </div>
  );
}

export default function PerformanceDiagnosticsModal({
  stats,
  onClose,
  onReset,
}) {
  const assessment = memoryAssessment(stats);

  useEffect(() => {
    const handleKeyDown = (event) => {
      if (event.key === 'Escape') onClose();
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [onClose]);

  return createPortal((
    <div
      className="performance-modal__backdrop"
      role="presentation"
      onMouseDown={(event) => {
        if (event.target === event.currentTarget) onClose();
      }}
    >
      <section
        className="performance-modal glass-panel"
        role="dialog"
        aria-modal="true"
        aria-labelledby="performance-modal-title"
      >
        <header className="performance-modal__header">
          <div>
            <span>RUNTIME DIAGNOSTICS</span>
            <h2 id="performance-modal-title">Memory &amp; Rendering</h2>
          </div>
          <button type="button" onClick={onClose} aria-label="Close runtime diagnostics">
            X
          </button>
        </header>

        <div className={`performance-modal__assessment is-${assessment.tone}`}>
          <div>
            <span>JS HEAP STATUS</span>
            <strong>{assessment.label}</strong>
          </div>
          <p>{assessment.detail}</p>
        </div>

        <div className="performance-modal__section">
          <h3>MEMORY</h3>
          <div className="performance-modal__grid">
            <Metric
              label="CURRENT JS HEAP"
              value={`${formatNumber(stats?.jsHeapMb, 1)} MB`}
              detail="현재 페이지의 JavaScript 객체"
            />
            <Metric
              label="5 MIN PEAK"
              value={`${formatNumber(stats?.jsHeapPeakMb, 1)} MB`}
              detail="최근 계측 창의 최고값"
            />
            <Metric
              label="HEAP TREND"
              value={formatSigned(stats?.jsHeapTrendMbPerMin)}
              detail="최소 30초 뒤 표시"
            />
            <Metric
              label="DOM NODES"
              value={formatNumber(stats?.domNodes)}
              detail="화면에 유지 중인 요소 수"
            />
          </div>
        </div>

        <div className="performance-modal__section">
          <h3>RENDERING</h3>
          <div className="performance-modal__grid">
            <Metric
              label="FRAME RATE"
              value={`${formatNumber(stats?.fps)} FPS`}
              detail={`P95 ${formatNumber(stats?.frameP95Ms, 1)} ms`}
            />
            <Metric
              label="SLOW FRAMES"
              value={formatNumber(stats?.slowFrames)}
              detail="최근 표본 중 30FPS 미만"
            />
            <Metric
              label="DRAW LOAD"
              value={`${formatNumber(stats?.calls)} CALLS`}
              detail={`${formatNumber(stats?.triangles)} triangles`}
            />
            <Metric
              label="WEBGL RESOURCES"
              value={`${formatNumber(stats?.geometries)} / ${formatNumber(stats?.textures)}`}
              detail="geometry / texture"
            />
            <Metric
              label="POSE BUFFER"
              value={formatNumber(stats?.poseSamples)}
              detail="20대 전체 최근 pose 표본"
            />
            <Metric
              label="SCENE LIFECYCLE"
              value={`${formatNumber(stats?.sceneBuilds)} / ${formatNumber(stats?.garageBuilds)}`}
              detail={`scene / garage · cars ${formatNumber(stats?.carModelsBuilt)}`}
            />
          </div>
        </div>

        <div className="performance-modal__note">
          <strong>Chrome 탭 메모리와 JS heap은 다른 값입니다.</strong>
          <p>
            Chrome 작업 관리자의 탭 메모리에는 WebGL, 렌더러 네이티브 메모리와
            브라우저 예약 영역이 포함됩니다. 여기의 heap이 안정적인데 탭 메모리만
            계속 증가하면 JavaScript 배열보다 Chrome/WebGL 쪽을 우선 조사합니다.
          </p>
        </div>

        <footer className="performance-modal__footer">
          <button type="button" onClick={onReset}>
            RESET 5 MIN WINDOW
          </button>
          <button type="button" className="is-primary" onClick={onClose}>
            CLOSE
          </button>
        </footer>
      </section>
    </div>
  ), document.body);
}
