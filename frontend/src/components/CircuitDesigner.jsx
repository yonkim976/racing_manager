import { useMemo, useRef, useState } from 'react';
import {
  DEFAULT_VIEWBOX,
  clamp,
  compileCircuitDraft,
  normalizeCircuitForEditor,
  pointAtProgress,
  progressForIndex,
  toPoint,
} from '../utils/circuitGeometry';
import './CircuitDesigner.css';

const TOOLS = [
  { id: 'select', label: 'Select' },
  { id: 'move', label: 'Move' },
  { id: 'add_center', label: 'Add Centerline Point' },
  { id: 'delete', label: 'Delete Point' },
  { id: 'start', label: 'Set Start/Finish' },
  { id: 'pit', label: 'Edit Pit Lane' },
  { id: 'corner', label: 'Add Corner Marker' },
  { id: 'drs', label: 'Add DRS Zone' },
  { id: 'segment', label: 'Add Driving Segment' },
];

const DRIVING_SEGMENT_TYPES = ['straight', 'heavy_braking', 'technical', 'traction', 'sweeping'];

function pathFromCoords(coords = []) {
  return coords.map(([x, y]) => `${x},${y}`).join(' ');
}

function validationButtonLabel(status) {
  if (status === 'checking') return 'Checking';
  if (status === 'valid') return 'Valid';
  if (status === 'invalid') return 'Issues';
  return 'Validate';
}

function validationText(validation) {
  if (validation.status === 'checking') return 'Checking circuit geometry...';
  if (validation.status === 'valid') return 'Validation passed';
  if (validation.status === 'invalid') {
    return validation.errors[0] || validation.warnings[0] || 'Validation found issues';
  }
  return 'Draft ready';
}

function progressPolyline(trackCoords, start, end, samples = 28) {
  if (!trackCoords.length) return [];
  const startValue = Number(start);
  const endValue = Number(end);
  if (!Number.isFinite(startValue) || !Number.isFinite(endValue)) return [];
  const length = startValue <= endValue ? endValue - startValue : (1 - startValue) + endValue;
  return Array.from({ length: samples + 1 }, (_, index) => {
    const progress = (startValue + (length * index) / samples) % 1;
    const point = pointAtProgress(trackCoords, progress);
    return [Number(point.x.toFixed(3)), Number(point.y.toFixed(3))];
  });
}

function segmentColor(type) {
  switch (type) {
    case 'straight': return '#38f38d';
    case 'heavy_braking': return '#ff5d5d';
    case 'technical': return '#58a6ff';
    case 'traction': return '#f2a13b';
    case 'sweeping': return '#c084fc';
    default: return '#ffffff';
  }
}

function makeStarterCircuit() {
  return normalizeCircuitForEditor({
    id: `custom_${Date.now()}`,
    name: 'Custom Circuit',
    country: 'Custom',
    base_lap_time: 90,
    track_length_m: 5000,
    total_laps: 50,
    pit_loss_time: 20,
    overtaking_difficulty: 0.5,
    editor: {
      centerlineControlPoints: [
        { x: 180, y: 180 },
        { x: 430, y: 125 },
        { x: 640, y: 270 },
        { x: 560, y: 465 },
        { x: 250, y: 475 },
        { x: 120, y: 330 },
      ],
      pitLaneControlPoints: [],
      trackWidth: 18,
      sampleSpacing: 16,
    },
    drs_zones: [],
    landmarks: [],
    segments: [
      { name: 'Full Lap', type: 'straight', start: 0, end: 1 },
    ],
  });
}

function readJsonFile(file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => {
      try {
        resolve(JSON.parse(String(reader.result || '{}')));
      } catch (error) {
        reject(error);
      }
    };
    reader.onerror = () => reject(reader.error);
    reader.readAsText(file);
  });
}

function readImageFile(file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(String(reader.result || ''));
    reader.onerror = () => reject(reader.error);
    reader.readAsDataURL(file);
  });
}

export default function CircuitDesigner({ circuits, initialCircuitId, onClose }) {
  const svgRef = useRef(null);
  const importFileRef = useRef(null);
  const imageFileRef = useRef(null);
  const initialCircuit = circuits.find((circuit) => circuit.id === initialCircuitId) || circuits[0];
  const [sourceCircuitId, setSourceCircuitId] = useState(initialCircuit?.id || 'new');
  const [circuit, setCircuit] = useState(() => (
    initialCircuit ? normalizeCircuitForEditor(initialCircuit) : makeStarterCircuit()
  ));
  const [tool, setTool] = useState('select');
  const [selected, setSelected] = useState(null);
  const [dragTarget, setDragTarget] = useState(null);
  const [pendingRange, setPendingRange] = useState(null);
  const [importText, setImportText] = useState('');
  const [validation, setValidation] = useState({
    status: 'idle',
    errors: [],
    warnings: [],
    checkedAt: '',
  });

  const compiled = useMemo(() => compileCircuitDraft(circuit), [circuit]);
  const exportJson = useMemo(() => JSON.stringify(compiled.circuit, null, 2), [compiled.circuit]);
  const editor = circuit.editor;
  const view = editor.view || { offsetX: 0, offsetY: 0, scale: 1 };
  const viewBox = {
    x: view.offsetX,
    y: view.offsetY,
    width: DEFAULT_VIEWBOX.width / Math.max(0.2, view.scale),
    height: DEFAULT_VIEWBOX.height / Math.max(0.2, view.scale),
  };

  const updateCircuit = (updater) => {
    setCircuit((current) => {
      const draft = JSON.parse(JSON.stringify(current));
      updater(draft);
      return normalizeCircuitForEditor(draft);
    });
    setValidation((current) => ({ ...current, status: 'idle' }));
  };

  const setEditorPatch = (patch) => {
    updateCircuit((draft) => {
      draft.editor = {
        ...draft.editor,
        ...patch,
        view: {
          ...(draft.editor?.view || {}),
          ...(patch.view || {}),
        },
      };
    });
  };

  const screenToWorld = (event) => {
    const rect = svgRef.current.getBoundingClientRect();
    return {
      x: viewBox.x + ((event.clientX - rect.left) / rect.width) * viewBox.width,
      y: viewBox.y + ((event.clientY - rect.top) / rect.height) * viewBox.height,
    };
  };

  const setView = (nextView) => {
    setEditorPatch({ view: nextView });
  };

  const zoomBy = (factor, anchor = null) => {
    const nextScale = clamp(view.scale * factor, 0.4, 6);
    const currentCenter = anchor || {
      x: viewBox.x + viewBox.width / 2,
      y: viewBox.y + viewBox.height / 2,
    };
    const nextWidth = DEFAULT_VIEWBOX.width / nextScale;
    const nextHeight = DEFAULT_VIEWBOX.height / nextScale;
    setView({
      offsetX: currentCenter.x - nextWidth / 2,
      offsetY: currentCenter.y - nextHeight / 2,
      scale: nextScale,
    });
  };

  const fitView = () => {
    setView({ offsetX: 0, offsetY: 0, scale: 1 });
  };

  const resetFromSource = (nextCircuitId) => {
    if (nextCircuitId === 'new') {
      setSourceCircuitId('new');
      setCircuit(makeStarterCircuit());
      setSelected(null);
      setValidation({ status: 'idle', errors: [], warnings: [], checkedAt: '' });
      return;
    }
    const nextCircuit = circuits.find((item) => String(item.id) === String(nextCircuitId)) || circuits[0];
    setSourceCircuitId(nextCircuit.id);
    setCircuit(normalizeCircuitForEditor(nextCircuit));
    setSelected(null);
    setValidation({ status: 'idle', errors: [], warnings: [], checkedAt: '' });
  };

  const updateMeta = (key, value) => {
    updateCircuit((draft) => {
      draft[key] = value;
    });
  };

  const updateEditorNumber = (key, value) => {
    updateCircuit((draft) => {
      draft.editor[key] = Number(value);
    });
  };

  const updatePoint = (kind, index, point) => {
    updateCircuit((draft) => {
      const key = kind === 'pit' ? 'pitLaneControlPoints' : 'centerlineControlPoints';
      if (!draft.editor[key][index]) return;
      draft.editor[key][index] = {
        x: Number(point.x.toFixed(3)),
        y: Number(point.y.toFixed(3)),
      };
    });
  };

  const deletePoint = (kind, index) => {
    updateCircuit((draft) => {
      const key = kind === 'pit' ? 'pitLaneControlPoints' : 'centerlineControlPoints';
      draft.editor[key].splice(index, 1);
    });
    setSelected(null);
  };

  const addCenterPoint = (point) => {
    updateCircuit((draft) => {
      draft.editor.centerlineControlPoints.push({
        x: Number(point.x.toFixed(3)),
        y: Number(point.y.toFixed(3)),
      });
    });
  };

  const addPitPoint = (point) => {
    updateCircuit((draft) => {
      draft.editor.pitLaneControlPoints.push({
        x: Number(point.x.toFixed(3)),
        y: Number(point.y.toFixed(3)),
      });
    });
  };

  const setStartFinishFromPoint = (point) => {
    updateCircuit((draft) => {
      const index = compiled.trackCoords.length
        ? compiled.trackCoords.reduce((best, coord, coordIndex) => {
          const current = toPoint(coord);
          const bestPoint = toPoint(compiled.trackCoords[best]);
          const currentDistance = Math.hypot(current.x - point.x, current.y - point.y);
          const bestDistance = Math.hypot(bestPoint.x - point.x, bestPoint.y - point.y);
          return currentDistance < bestDistance ? coordIndex : best;
        }, 0)
        : 0;
      draft.start_finish_index = clamp(index, 0, Math.max(0, compiled.trackCoords.length - 2));
    });
  };

  const addLandmarkAtPoint = (point) => {
    if (!compiled.trackCoords.length) return;
    const index = compiled.trackCoords.reduce((best, coord, coordIndex) => {
      const current = toPoint(coord);
      const bestPoint = toPoint(compiled.trackCoords[best]);
      const currentDistance = Math.hypot(current.x - point.x, current.y - point.y);
      const bestDistance = Math.hypot(bestPoint.x - point.x, bestPoint.y - point.y);
      return currentDistance < bestDistance ? coordIndex : best;
    }, 0);
    const progress = progressForIndex(index, compiled.trackCoords);
    updateCircuit((draft) => {
      draft.landmarks.push({
        type: 'corner',
        label: `T${draft.landmarks.length + 1}`,
        progress: Number(progress.toFixed(6)),
        track_index: index,
      });
    });
  };

  const addRangeItem = (point, rangeType) => {
    if (!compiled.trackCoords.length) return;
    const index = compiled.trackCoords.reduce((best, coord, coordIndex) => {
      const current = toPoint(coord);
      const bestPoint = toPoint(compiled.trackCoords[best]);
      const currentDistance = Math.hypot(current.x - point.x, current.y - point.y);
      const bestDistance = Math.hypot(bestPoint.x - point.x, bestPoint.y - point.y);
      return currentDistance < bestDistance ? coordIndex : best;
    }, 0);
    const progress = progressForIndex(index, compiled.trackCoords);
    if (!pendingRange || pendingRange.type !== rangeType) {
      setPendingRange({ type: rangeType, start: progress });
      return;
    }

    const start = Number(Math.min(pendingRange.start, progress).toFixed(6));
    const end = Number(Math.max(pendingRange.start, progress).toFixed(6));
    if (end - start < 0.005) {
      setPendingRange(null);
      return;
    }

    updateCircuit((draft) => {
      if (rangeType === 'drs') {
        draft.drs_zones.push({
          name: `DRS ${draft.drs_zones.length + 1}`,
          detection: Number(Math.max(0, start - 0.03).toFixed(6)),
          start,
          end,
        });
      } else {
        draft.segments.push({
          name: `Segment ${draft.segments.length + 1}`,
          type: 'straight',
          start,
          end,
        });
      }
    });
    setPendingRange(null);
  };

  const handleStagePointerDown = (event) => {
    if (event.button !== 0) return;
    const point = screenToWorld(event);

    if (tool === 'add_center') {
      addCenterPoint(point);
      return;
    }
    if (tool === 'pit') {
      addPitPoint(point);
      return;
    }
    if (tool === 'start') {
      setStartFinishFromPoint(point);
      return;
    }
    if (tool === 'corner') {
      addLandmarkAtPoint(point);
      return;
    }
    if (tool === 'drs') {
      addRangeItem(point, 'drs');
      return;
    }
    if (tool === 'segment') {
      addRangeItem(point, 'segment');
      return;
    }
    if (tool === 'move') {
      setDragTarget({
        kind: 'pan',
        pointerId: event.pointerId,
        startX: event.clientX,
        startY: event.clientY,
        offsetX: view.offsetX,
        offsetY: view.offsetY,
      });
      event.currentTarget.setPointerCapture?.(event.pointerId);
    }
  };

  const handlePointPointerDown = (event, kind, index) => {
    event.stopPropagation();
    if (tool === 'delete') {
      deletePoint(kind, index);
      return;
    }
    setSelected({ kind, index });
    setDragTarget({
      kind,
      index,
      pointerId: event.pointerId,
    });
    event.currentTarget.setPointerCapture?.(event.pointerId);
  };

  const handlePointerMove = (event) => {
    if (!dragTarget || dragTarget.pointerId !== event.pointerId) return;
    if (dragTarget.kind === 'pan') {
      const rect = svgRef.current.getBoundingClientRect();
      const dx = ((event.clientX - dragTarget.startX) / rect.width) * viewBox.width;
      const dy = ((event.clientY - dragTarget.startY) / rect.height) * viewBox.height;
      setView({
        ...view,
        offsetX: dragTarget.offsetX - dx,
        offsetY: dragTarget.offsetY - dy,
      });
      return;
    }
    updatePoint(dragTarget.kind, dragTarget.index, screenToWorld(event));
  };

  const handlePointerUp = (event) => {
    if (dragTarget?.pointerId === event.pointerId) {
      event.currentTarget.releasePointerCapture?.(event.pointerId);
    }
    setDragTarget(null);
  };

  const handleWheel = (event) => {
    event.preventDefault();
    const anchor = screenToWorld(event);
    zoomBy(event.deltaY < 0 ? 1.12 : 0.88, anchor);
  };

  const validateCircuit = async () => {
    setValidation({ status: 'checking', errors: [], warnings: [], checkedAt: '' });
    try {
      const response = await fetch('/api/circuits/validate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(compiled.circuit),
      });
      const result = await response.json();
      const errors = result.errors || [];
      const warnings = result.warnings || [];
      setValidation({
        status: errors.length ? 'invalid' : 'valid',
        errors,
        warnings,
        checkedAt: new Date().toLocaleTimeString(),
      });
    } catch (error) {
      setValidation({
        status: 'invalid',
        errors: [error.message],
        warnings: [],
        checkedAt: new Date().toLocaleTimeString(),
      });
    }
  };

  const handleImportObject = (object) => {
    setCircuit(normalizeCircuitForEditor(object));
    setSelected(null);
    setPendingRange(null);
    setValidation({ status: 'idle', errors: [], warnings: [], checkedAt: '' });
  };

  const handleImportText = () => {
    try {
      handleImportObject(JSON.parse(importText));
      setImportText('');
    } catch (error) {
      setValidation({
        status: 'invalid',
        errors: [`Import failed: ${error.message}`],
        warnings: [],
        checkedAt: new Date().toLocaleTimeString(),
      });
    }
  };

  const handleImportFile = async (event) => {
    const file = event.target.files?.[0];
    if (!file) return;
    try {
      handleImportObject(await readJsonFile(file));
    } catch (error) {
      setValidation({
        status: 'invalid',
        errors: [`Import failed: ${error.message}`],
        warnings: [],
        checkedAt: new Date().toLocaleTimeString(),
      });
    } finally {
      event.target.value = '';
    }
  };

  const handleImageFile = async (event) => {
    const file = event.target.files?.[0];
    if (!file) return;
    try {
      const backgroundImage = await readImageFile(file);
      setEditorPatch({ backgroundImage });
    } finally {
      event.target.value = '';
    }
  };

  const copyExportJson = async () => {
    if (!navigator.clipboard) return;
    await navigator.clipboard.writeText(exportJson);
  };

  const startPoint = compiled.trackCoords[compiled.circuit.start_finish_index] || compiled.trackCoords[0];
  const statusClass = validation.status === 'idle' ? '' : `circuit-designer__status--${validation.status}`;

  return (
    <div className="circuit-designer carbon-bg">
      <header className="circuit-designer__header">
        <div>
          <span className="circuit-designer__eyebrow">Circuit Maker</span>
          <input
            className="circuit-designer__name"
            value={circuit.name}
            onChange={(event) => updateMeta('name', event.target.value)}
          />
        </div>
        <div className="circuit-designer__header-actions">
          <select value={sourceCircuitId} onChange={(event) => resetFromSource(event.target.value)}>
            <option value="new">New custom circuit</option>
            {circuits.map((item) => (
              <option key={item.id} value={item.id}>{item.name}</option>
            ))}
          </select>
          <button
            type="button"
            className={`circuit-designer__button circuit-designer__button--${validation.status}`}
            onClick={validateCircuit}
            disabled={validation.status === 'checking'}
          >
            {validationButtonLabel(validation.status)}
          </button>
          <button type="button" className="circuit-designer__button circuit-designer__button--ghost" onClick={onClose}>
            Back
          </button>
        </div>
      </header>

      <main className="circuit-designer__workspace">
        <section className="circuit-designer__stage glass-panel">
          <div className="circuit-designer__toolbar">
            {TOOLS.map((item) => (
              <button
                key={item.id}
                type="button"
                className={tool === item.id ? 'circuit-designer__tool circuit-designer__tool--active' : 'circuit-designer__tool'}
                onClick={() => {
                  setTool(item.id);
                  setPendingRange(null);
                }}
              >
                {item.label}
              </button>
            ))}
          </div>

          <svg
            ref={svgRef}
            className="circuit-designer__svg"
            viewBox={`${viewBox.x} ${viewBox.y} ${viewBox.width} ${viewBox.height}`}
            role="img"
            aria-label="Editable circuit layout"
            onPointerDown={handleStagePointerDown}
            onPointerMove={handlePointerMove}
            onPointerUp={handlePointerUp}
            onPointerLeave={handlePointerUp}
            onWheel={handleWheel}
          >
            <rect
              x={viewBox.x}
              y={viewBox.y}
              width={viewBox.width}
              height={viewBox.height}
              fill="rgba(2, 3, 7, 0.36)"
            />

            {editor.backgroundImage && (
              <image
                href={editor.backgroundImage}
                x="0"
                y="0"
                width={DEFAULT_VIEWBOX.width}
                height={DEFAULT_VIEWBOX.height}
                preserveAspectRatio="xMidYMid meet"
                opacity={editor.backgroundOpacity}
              />
            )}

            {compiled.boundaries.left.length > 1 && (
              <>
                <polyline points={pathFromCoords(compiled.boundaries.left)} className="circuit-designer__boundary" />
                <polyline points={pathFromCoords(compiled.boundaries.right)} className="circuit-designer__boundary" />
              </>
            )}

            {compiled.trackCoords.length > 1 && (
              <>
                <polyline points={pathFromCoords(compiled.trackCoords)} className="circuit-designer__track-border" />
                <polyline points={pathFromCoords(compiled.trackCoords)} className="circuit-designer__track" />
                <polyline points={pathFromCoords(compiled.trackCoords)} className="circuit-designer__track-center" />
              </>
            )}

            {circuit.segments.map((segment, index) => (
              <polyline
                key={`${segment.name}-${index}-${segment.start}-${segment.end}`}
                points={pathFromCoords(progressPolyline(compiled.trackCoords, segment.start, segment.end, 24))}
                className="circuit-designer__driving-segment"
                style={{ stroke: segmentColor(segment.type) }}
              />
            ))}

            {circuit.drs_zones.map((zone, index) => (
              <polyline
                key={`${zone.name}-${index}-${zone.start}-${zone.end}`}
                points={pathFromCoords(progressPolyline(compiled.trackCoords, zone.start, zone.end, 20))}
                className="circuit-designer__drs-zone"
              />
            ))}

            {compiled.pitLaneCoords.length > 1 && (
              <polyline points={pathFromCoords(compiled.pitLaneCoords)} className="circuit-designer__pit-lane" />
            )}

            {startPoint && (
              <circle
                cx={startPoint[0]}
                cy={startPoint[1]}
                r="9"
                className="circuit-designer__start-marker"
              />
            )}

            {compiled.pitEntryIndex !== null && compiled.trackCoords[compiled.pitEntryIndex] && (
              <circle
                cx={compiled.trackCoords[compiled.pitEntryIndex][0]}
                cy={compiled.trackCoords[compiled.pitEntryIndex][1]}
                r="7"
                className="circuit-designer__pit-marker"
              />
            )}
            {compiled.pitExitIndex !== null && compiled.trackCoords[compiled.pitExitIndex] && (
              <rect
                x={compiled.trackCoords[compiled.pitExitIndex][0] - 6}
                y={compiled.trackCoords[compiled.pitExitIndex][1] - 6}
                width="12"
                height="12"
                className="circuit-designer__pit-marker"
              />
            )}

            {compiled.circuit.landmarks.map((landmark, index) => {
              const point = pointAtProgress(compiled.trackCoords, landmark.progress || 0);
              return (
                <g key={`${landmark.label}-${index}`} transform={`translate(${point.x} ${point.y})`}>
                  <circle r="7" className="circuit-designer__landmark" />
                  <text x="10" y="-10" className="circuit-designer__landmark-label">{landmark.label}</text>
                </g>
              );
            })}

            {editor.centerlineControlPoints.map((point, index) => (
              <circle
                key={`center-${index}`}
                cx={point.x}
                cy={point.y}
                r={selected?.kind === 'center' && selected.index === index ? 7 : 5}
                className={[
                  'circuit-designer__handle',
                  selected?.kind === 'center' && selected.index === index ? 'circuit-designer__handle--selected' : '',
                ].join(' ')}
                onPointerDown={(event) => handlePointPointerDown(event, 'center', index)}
              />
            ))}

            {editor.pitLaneControlPoints.map((point, index) => (
              <rect
                key={`pit-${index}`}
                x={point.x - 5}
                y={point.y - 5}
                width="10"
                height="10"
                className={[
                  'circuit-designer__handle',
                  'circuit-designer__handle--control',
                  selected?.kind === 'pit' && selected.index === index ? 'circuit-designer__handle--selected' : '',
                ].join(' ')}
                onPointerDown={(event) => handlePointPointerDown(event, 'pit', index)}
              />
            ))}
          </svg>

          <div className="circuit-designer__viewport-controls">
            <button type="button" onClick={() => zoomBy(0.85)}>-</button>
            <span>{Math.round(view.scale * 100)}%</span>
            <button type="button" onClick={() => zoomBy(1.15)}>+</button>
            <button type="button" onClick={fitView}>Fit</button>
          </div>

          <div className={`circuit-designer__status ${statusClass}`}>
            {pendingRange ? `Click range end for ${pendingRange.type.toUpperCase()}` : validationText(validation)}
          </div>
        </section>

        <aside className="circuit-designer__tools glass-panel">
          <section className="circuit-designer__tool-section">
            <div className="circuit-designer__section-title">Inspector</div>
            <div className="circuit-designer__editor-grid">
              <label>
                Country
                <input value={circuit.country} onChange={(event) => updateMeta('country', event.target.value)} />
              </label>
              <label>
                Base Lap
                <input type="number" step="0.1" value={circuit.base_lap_time} onChange={(event) => updateMeta('base_lap_time', Number(event.target.value))} />
              </label>
              <label>
                Length M
                <input type="number" value={circuit.track_length_m} onChange={(event) => updateMeta('track_length_m', Number(event.target.value))} />
              </label>
              <label>
                Laps
                <input type="number" value={circuit.total_laps} onChange={(event) => updateMeta('total_laps', Number(event.target.value))} />
              </label>
              <label>
                Track Width
                <input type="number" value={editor.trackWidth} onChange={(event) => updateEditorNumber('trackWidth', event.target.value)} />
              </label>
              <label>
                Spacing
                <input type="number" value={editor.sampleSpacing} onChange={(event) => updateEditorNumber('sampleSpacing', event.target.value)} />
              </label>
            </div>
            <div className="circuit-designer__metrics">
              <span>Center points <strong>{editor.centerlineControlPoints.length}</strong></span>
              <span>Track points <strong>{compiled.trackCoords.length}</strong></span>
              <span>Pixel length <strong>{Math.round(compiled.pixelLength)}</strong></span>
              <span>S/F index <strong>{compiled.circuit.start_finish_index}</strong></span>
              <span>Pit entry <strong>{compiled.pitEntryIndex ?? '-'}</strong></span>
              <span>Pit exit <strong>{compiled.pitExitIndex ?? '-'}</strong></span>
            </div>
          </section>

          <section className="circuit-designer__tool-section">
            <div className="circuit-designer__section-title">Background</div>
            <input
              ref={imageFileRef}
              type="file"
              accept="image/png,image/jpeg"
              className="circuit-designer__hidden-input"
              onChange={handleImageFile}
            />
            <button type="button" className="circuit-designer__mini-button" onClick={() => imageFileRef.current?.click()}>
              Upload PNG/JPG
            </button>
            <label className="circuit-designer__range-label">
              Opacity
              <input
                type="range"
                min="0"
                max="1"
                step="0.05"
                value={editor.backgroundOpacity}
                onChange={(event) => setEditorPatch({ backgroundOpacity: Number(event.target.value) })}
              />
            </label>
          </section>

          <section className="circuit-designer__tool-section">
            <div className="circuit-designer__section-title">Validation</div>
            <div className={`circuit-designer__validation-card circuit-designer__validation-card--${validation.status}`}>
              <div className="circuit-designer__validation-head">
                <strong>{validation.status === 'idle' ? 'Not checked' : validation.status.toUpperCase()}</strong>
                {validation.checkedAt && <span>{validation.checkedAt}</span>}
              </div>
              <p>{validationText(validation)}</p>
              {(validation.errors.length > 0 || validation.warnings.length > 0) && (
                <ul>
                  {validation.errors.map((error) => <li key={error}>{error}</li>)}
                  {validation.warnings.map((warning) => <li key={warning}>Warning: {warning}</li>)}
                </ul>
              )}
              <button type="button" className="circuit-designer__mini-button" onClick={validateCircuit}>
                Validate
              </button>
            </div>
          </section>

          <section className="circuit-designer__tool-section">
            <div className="circuit-designer__section-title">Landmarks</div>
            <div className="circuit-designer__rows">
              {circuit.landmarks.map((landmark, index) => (
                <div className="circuit-designer__row circuit-designer__row--landmark" key={`${landmark.label}-${index}`}>
                  <input value={landmark.label} onChange={(event) => updateCircuit((draft) => { draft.landmarks[index].label = event.target.value; })} />
                  <input type="number" min="0" max="1" step="0.001" value={landmark.progress ?? 0} onChange={(event) => updateCircuit((draft) => { draft.landmarks[index].progress = Number(event.target.value); })} />
                </div>
              ))}
            </div>
          </section>

          <section className="circuit-designer__tool-section">
            <div className="circuit-designer__section-title">DRS Zones</div>
            <div className="circuit-designer__rows">
              {circuit.drs_zones.map((zone, index) => (
                <div className="circuit-designer__row" key={`${zone.name}-${index}`}>
                  <input value={zone.name} onChange={(event) => updateCircuit((draft) => { draft.drs_zones[index].name = event.target.value; })} />
                  <input type="number" min="0" max="1" step="0.001" value={zone.start} onChange={(event) => updateCircuit((draft) => { draft.drs_zones[index].start = Number(event.target.value); })} />
                  <input type="number" min="0" max="1" step="0.001" value={zone.end} onChange={(event) => updateCircuit((draft) => { draft.drs_zones[index].end = Number(event.target.value); })} />
                  <button type="button" onClick={() => updateCircuit((draft) => { draft.drs_zones.splice(index, 1); })}>x</button>
                </div>
              ))}
            </div>
          </section>

          <section className="circuit-designer__tool-section">
            <div className="circuit-designer__section-title">Driving Segments</div>
            <div className="circuit-designer__rows">
              {circuit.segments.map((segment, index) => (
                <div className="circuit-designer__row circuit-designer__row--driving" key={`${segment.name}-${index}`}>
                  <input value={segment.name} onChange={(event) => updateCircuit((draft) => { draft.segments[index].name = event.target.value; })} />
                  <input type="number" min="0" max="1" step="0.001" value={segment.start} onChange={(event) => updateCircuit((draft) => { draft.segments[index].start = Number(event.target.value); })} />
                  <input type="number" min="0" max="1" step="0.001" value={segment.end} onChange={(event) => updateCircuit((draft) => { draft.segments[index].end = Number(event.target.value); })} />
                  <select value={segment.type} onChange={(event) => updateCircuit((draft) => { draft.segments[index].type = event.target.value; })}>
                    {DRIVING_SEGMENT_TYPES.map((type) => <option key={type} value={type}>{type}</option>)}
                  </select>
                </div>
              ))}
            </div>
          </section>

          <section className="circuit-designer__tool-section">
            <div className="circuit-designer__section-title">Import JSON</div>
            <input
              ref={importFileRef}
              type="file"
              accept="application/json,.json"
              className="circuit-designer__hidden-input"
              onChange={handleImportFile}
            />
            <div className="circuit-designer__import-actions">
              <button type="button" className="circuit-designer__mini-button" onClick={() => importFileRef.current?.click()}>
                Import File
              </button>
              <button type="button" className="circuit-designer__mini-button" onClick={handleImportText}>
                Import Text
              </button>
            </div>
            <textarea
              value={importText}
              onChange={(event) => setImportText(event.target.value)}
              placeholder="Paste circuit JSON here"
            />
          </section>

          <section className="circuit-designer__tool-section">
            <div className="circuit-designer__section-title">Export JSON</div>
            <button type="button" className="circuit-designer__mini-button" onClick={copyExportJson}>
              Copy Export JSON
            </button>
            <textarea value={exportJson} readOnly />
            <p className="circuit-designer__export-hint">
              Use this object as one entry in backend/data/circuits.json, then restart the backend.
            </p>
          </section>
        </aside>
      </main>
    </div>
  );
}
