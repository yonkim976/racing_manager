export const DEFAULT_VIEWBOX = {
  x: 0,
  y: 0,
  width: 820,
  height: 620,
};

export const DEFAULT_EDITOR = {
  backgroundImage: null,
  backgroundOpacity: 0.5,
  view: {
    offsetX: 0,
    offsetY: 0,
    scale: 1,
  },
  centerlineControlPoints: [],
  pitLaneControlPoints: [],
  trackWidth: 18,
  sampleSpacing: 16,
};

export function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}
export function toPoint(coord) {
  if (!coord) return null;
  if (Array.isArray(coord)) return { x: Number(coord[0] || 0), y: Number(coord[1] || 0) };
  if (typeof coord === 'object') return { x: Number(coord.x || 0), y: Number(coord.y || 0) };
  return null;
}

export function toPointArray(point) {
  return [Number(point.x.toFixed(3)), Number(point.y.toFixed(3))];
}

export function normalizeCoordinatePath(coords = []) {
  return (coords || [])
    .map(toPoint)
    .filter(Boolean)
    .map(toPointArray);
}

export function distance(a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

export function lerp(a, b, t) {
  return {
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
  };
}

export function samePoint(a, b, epsilon = 0.001) {
  return a && b && Math.hypot(a.x - b.x, a.y - b.y) <= epsilon;
}

export function buildPathMetrics(pointsLike = [], closed = false) {
  const points = pointsLike.map(toPoint).filter(Boolean);
  const prepared = [...points];
  if (closed && prepared.length > 1 && !samePoint(prepared[0], prepared[prepared.length - 1])) {
    prepared.push({ ...prepared[0] });
  }

  const cumulative = [0];
  for (let index = 0; index < prepared.length - 1; index += 1) {
    cumulative.push(cumulative[cumulative.length - 1] + distance(prepared[index], prepared[index + 1]));
  }
  return {
    points: prepared,
    cumulative,
    totalLength: cumulative[cumulative.length - 1] || 0,
  };
}

export function pointAtDistance(metrics, targetDistance) {
  const { points, cumulative } = metrics;
  if (!points.length) return { x: 0, y: 0 };
  if (points.length === 1 || targetDistance <= 0) return points[0];
  if (targetDistance >= metrics.totalLength) return points[points.length - 1];

  let low = 0;
  let high = cumulative.length - 1;
  while (low < high) {
    const mid = Math.floor((low + high + 1) / 2);
    if (cumulative[mid] <= targetDistance) low = mid;
    else high = mid - 1;
  }

  const segmentLength = cumulative[low + 1] - cumulative[low];
  if (segmentLength <= 0) return points[low];
  return lerp(points[low], points[low + 1], (targetDistance - cumulative[low]) / segmentLength);
}

export function pointAtProgress(pointsLike, progress, closed = true) {
  const metrics = buildPathMetrics(pointsLike, closed);
  if (metrics.totalLength <= 0) return metrics.points[0] || { x: 0, y: 0 };
  const normalized = closed ? ((progress % 1) + 1) % 1 : clamp(progress, 0, 1);
  return pointAtDistance(metrics, normalized * metrics.totalLength);
}

export function resamplePoints(pointsLike, spacing = 16, closed = true) {
  const metrics = buildPathMetrics(pointsLike, closed);
  if (metrics.points.length < 2 || metrics.totalLength <= 0) {
    return metrics.points.map(toPointArray);
  }

  const steps = Math.max(1, Math.ceil(metrics.totalLength / Math.max(1, spacing)));
  const finalIndex = closed ? steps - 1 : steps;
  const sampled = [];
  for (let index = 0; index <= finalIndex; index += 1) {
    sampled.push(pointAtDistance(metrics, (metrics.totalLength * index) / steps));
  }
  if (closed && sampled.length) sampled.push({ ...sampled[0] });
  return sampled.map(toPointArray);
}

export function catmullRomPoint(p0, p1, p2, p3, t) {
  const t2 = t * t;
  const t3 = t2 * t;
  return {
    x: 0.5 * (
      (2 * p1.x)
      + (-p0.x + p2.x) * t
      + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2
      + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3
    ),
    y: 0.5 * (
      (2 * p1.y)
      + (-p0.y + p2.y) * t
      + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2
      + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3
    ),
  };
}

export function sampleCatmullRomClosed(controlPoints = [], spacing = 16) {
  const controls = controlPoints.map(toPoint).filter(Boolean);
  if (controls.length < 4) return [];

  const raw = [];
  for (let index = 0; index < controls.length; index += 1) {
    const p0 = controls[(index - 1 + controls.length) % controls.length];
    const p1 = controls[index];
    const p2 = controls[(index + 1) % controls.length];
    const p3 = controls[(index + 2) % controls.length];
    const samples = Math.max(10, Math.ceil(distance(p1, p2) / 10));
    for (let step = 0; step < samples; step += 1) {
      raw.push(catmullRomPoint(p0, p1, p2, p3, step / samples));
    }
  }
  raw.push({ ...raw[0] });
  return resamplePoints(raw, spacing, true);
}

export function sampleCatmullRomOpen(controlPoints = [], spacing = 14) {
  const controls = controlPoints.map(toPoint).filter(Boolean);
  if (controls.length < 2) return [];
  if (controls.length === 2) return resamplePoints(controls, spacing, false);

  const raw = [];
  for (let index = 0; index < controls.length - 1; index += 1) {
    const p0 = controls[Math.max(0, index - 1)];
    const p1 = controls[index];
    const p2 = controls[index + 1];
    const p3 = controls[Math.min(controls.length - 1, index + 2)];
    const samples = Math.max(10, Math.ceil(distance(p1, p2) / 10));
    for (let step = 0; step < samples; step += 1) {
      raw.push(catmullRomPoint(p0, p1, p2, p3, step / samples));
    }
  }
  raw.push({ ...controls[controls.length - 1] });
  return resamplePoints(raw, spacing, false);
}

export function tangentAt(pointsLike, index) {
  const points = pointsLike.map(toPoint).filter(Boolean);
  if (points.length < 2) return { x: 1, y: 0 };
  const safeIndex = clamp(index, 0, points.length - 1);
  const previous = points[Math.max(0, safeIndex - 1)];
  const next = points[Math.min(points.length - 1, safeIndex + 1)];
  const dx = next.x - previous.x;
  const dy = next.y - previous.y;
  const length = Math.max(0.0001, Math.hypot(dx, dy));
  return { x: dx / length, y: dy / length };
}

export function normalAt(pointsLike, index) {
  const tangent = tangentAt(pointsLike, index);
  return { x: -tangent.y, y: tangent.x };
}

export function buildBoundaries(trackCoords = [], trackWidth = 18) {
  const points = trackCoords.map(toPoint).filter(Boolean);
  const halfWidth = Math.max(0, Number(trackWidth || 0)) / 2;
  const left = [];
  const right = [];

  points.forEach((point, index) => {
    const normal = normalAt(points, index);
    left.push(toPointArray({ x: point.x + normal.x * halfWidth, y: point.y + normal.y * halfWidth }));
    right.push(toPointArray({ x: point.x - normal.x * halfWidth, y: point.y - normal.y * halfWidth }));
  });

  return { left, right };
}

export function nearestPointIndex(pointsLike = [], x, y) {
  const points = pointsLike.map(toPoint).filter(Boolean);
  if (!points.length) return 0;
  const target = { x: Number(x || 0), y: Number(y || 0) };
  let bestIndex = 0;
  let bestDistance = Infinity;
  points.forEach((point, index) => {
    const nextDistance = distance(point, target);
    if (nextDistance < bestDistance) {
      bestDistance = nextDistance;
      bestIndex = index;
    }
  });
  return bestIndex;
}

export function progressForIndex(index, trackCoords = []) {
  const denominator = Math.max(1, trackCoords.length - 1);
  return clamp(index / denominator, 0, 1);
}

export function polylineLength(pointsLike = [], closed = false) {
  return buildPathMetrics(pointsLike, closed).totalLength;
}

function sampleExistingCoordsAsControls(coords = []) {
  const points = coords.map(toPoint).filter(Boolean);
  if (!points.length) return [];
  const unique = samePoint(points[0], points[points.length - 1]) ? points.slice(0, -1) : points;
  const maxControls = 28;
  const step = Math.max(1, Math.floor(unique.length / maxControls));
  return unique.filter((_, index) => index % step === 0).map((point) => ({ x: point.x, y: point.y }));
}

function layoutSegmentsToControls(layoutSegments = []) {
  return layoutSegments
    .map((segment) => toPoint(segment.start))
    .filter(Boolean)
    .map((point) => ({ x: point.x, y: point.y }));
}

export function normalizeCircuitForEditor(source = {}) {
  const circuit = JSON.parse(JSON.stringify(source || {}));
  const editor = {
    ...DEFAULT_EDITOR,
    ...(circuit.editor || {}),
    view: {
      ...DEFAULT_EDITOR.view,
      ...(circuit.editor?.view || {}),
    },
  };

  if (!editor.centerlineControlPoints?.length) {
    editor.centerlineControlPoints = layoutSegmentsToControls(circuit.layout_segments);
  }
  if (!editor.centerlineControlPoints?.length) {
    editor.centerlineControlPoints = sampleExistingCoordsAsControls(circuit.track_coords);
  }
  if (!editor.pitLaneControlPoints?.length) {
    editor.pitLaneControlPoints = sampleExistingCoordsAsControls(circuit.pit_lane_coords).slice(0, 10);
  }

  return {
    id: circuit.id ?? Date.now(),
    name: circuit.name || 'Custom Circuit',
    country: circuit.country || 'Custom',
    base_lap_time: Number(circuit.base_lap_time || 90),
    track_length_m: Number(circuit.track_length_m || circuit.length_m || 5000),
    total_laps: Number(circuit.total_laps || circuit.laps || 50),
    pit_loss_time: Number(circuit.pit_loss_time || 20),
    overtaking_difficulty: Number(circuit.overtaking_difficulty ?? 0.5),
    sectors: circuit.sectors || [],
    layout_segments: circuit.layout_segments || [],
    track_coords: normalizeCoordinatePath(circuit.track_coords || []),
    start_finish_index: Number(circuit.start_finish_index || 0),
    pit_lane: circuit.pit_lane || { entry_progress: null, exit_progress: null },
    pit_lane_segments: circuit.pit_lane_segments || [],
    pit_lane_coords: normalizeCoordinatePath(circuit.pit_lane_coords || []),
    pit_wall_coords: normalizeCoordinatePath(circuit.pit_wall_coords || []),
    drs_zones: circuit.drs_zones || [],
    landmarks: (circuit.landmarks || []).map((landmark) => ({
      ...landmark,
      label: landmark.label || landmark.name || landmark.type || 'Marker',
    })),
    segments: circuit.segments || [],
    editor,
  };
}

export function compileCircuitDraft(circuit) {
  const editor = {
    ...DEFAULT_EDITOR,
    ...(circuit.editor || {}),
    view: {
      ...DEFAULT_EDITOR.view,
      ...(circuit.editor?.view || {}),
    },
  };
  const sampleSpacing = Number(editor.sampleSpacing || DEFAULT_EDITOR.sampleSpacing);
  const trackCoords = editor.centerlineControlPoints?.length >= 4
    ? sampleCatmullRomClosed(editor.centerlineControlPoints, sampleSpacing)
    : normalizeCoordinatePath(circuit.track_coords || []);
  const boundaries = buildBoundaries(trackCoords, editor.trackWidth);
  const startFinishIndex = clamp(
    Number(circuit.start_finish_index || 0),
    0,
    Math.max(0, trackCoords.length - 2),
  );

  let pitLaneCoords = [];
  let pitEntryIndex = null;
  let pitExitIndex = null;
  if (editor.pitLaneControlPoints?.length >= 2) {
    pitLaneCoords = sampleCatmullRomOpen(editor.pitLaneControlPoints, Math.max(2, sampleSpacing * 0.875));
    if (trackCoords.length && pitLaneCoords.length >= 2) {
      const first = toPoint(pitLaneCoords[0]);
      const last = toPoint(pitLaneCoords[pitLaneCoords.length - 1]);
      pitEntryIndex = nearestPointIndex(trackCoords, first.x, first.y);
      pitExitIndex = nearestPointIndex(trackCoords, last.x, last.y);
      const entryPoint = toPoint(trackCoords[pitEntryIndex]);
      const exitPoint = toPoint(trackCoords[pitExitIndex]);
      pitLaneCoords = [
        toPointArray(entryPoint),
        ...pitLaneCoords.slice(1, -1),
        toPointArray(exitPoint),
      ];
    }
  } else {
    pitLaneCoords = normalizeCoordinatePath(circuit.pit_lane_coords || []);
  }

  const pitWallCoords = pitLaneCoords.length >= 2
    ? offsetOpenPath(pitLaneCoords, -7)
    : normalizeCoordinatePath(circuit.pit_wall_coords || []);

  const pitLane = {
    ...(circuit.pit_lane || {}),
    entry_progress: pitEntryIndex === null
      ? circuit.pit_lane?.entry_progress ?? null
      : Number(progressForIndex(pitEntryIndex, trackCoords).toFixed(6)),
    exit_progress: pitExitIndex === null
      ? circuit.pit_lane?.exit_progress ?? null
      : Number(progressForIndex(pitExitIndex, trackCoords).toFixed(6)),
  };

  const landmarks = (circuit.landmarks || []).map((landmark) => {
    const progress = Number(landmark.progress ?? progressForIndex(landmark.track_index || 0, trackCoords));
    return {
      ...landmark,
      label: landmark.label || landmark.name || landmark.type || 'Marker',
      progress: Number(clamp(progress, 0, 1).toFixed(6)),
      track_index: nearestPointIndex(
        trackCoords,
        pointAtProgress(trackCoords, progress).x,
        pointAtProgress(trackCoords, progress).y,
      ),
    };
  });

  return {
    circuit: {
      ...circuit,
      track_length_m: Number(circuit.track_length_m || circuit.length_m || 5000),
      total_laps: Number(circuit.total_laps || circuit.laps || 50),
      editor,
      track_coords: trackCoords,
      track_boundaries: boundaries,
      start_finish_index: startFinishIndex,
      pit_lane: pitLane,
      pit_lane_coords: pitLaneCoords,
      pit_wall_coords: pitWallCoords,
      landmarks,
    },
    trackCoords,
    pitLaneCoords,
    pitWallCoords,
    boundaries,
    pixelLength: polylineLength(trackCoords, false),
    pitEntryIndex,
    pitExitIndex,
  };
}

export function offsetOpenPath(coords = [], offset = 0) {
  const points = coords.map(toPoint).filter(Boolean);
  return points.map((point, index) => {
    const normal = normalAt(points, index);
    return toPointArray({
      x: point.x + normal.x * offset,
      y: point.y + normal.y * offset,
    });
  });
}
