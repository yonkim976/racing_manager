import { useEffect, useRef, useState, useCallback, useMemo } from 'react';
import { Application, Container, Graphics, Text } from 'pixi.js';
import { normalizeCoordinatePath } from '../../utils/circuitGeometry';
import './TrackCanvas.css';

const PADDING = 48;
const LABEL_MARGIN = 6;
const LABEL_COLLISION_PADDING = 5;
const DETAIL_ZOOM_PERCENT = 2000;
const PHYSICAL_ZOOM_MIN_PERCENT = 1500;
const ZOOM_LEVELS = [100, 250, 500, PHYSICAL_ZOOM_MIN_PERCENT, DETAIL_ZOOM_PERCENT];
const MARKER_INTERPOLATION_MS = 140;
const MARKER_ROUTE_TRANSITION_MS = 240;
const MARKER_PREDICTION_MAX_MS = 450;
const MARKER_ROTATION_LERP = 0.18;
const MARKER_ROUTE_JUMP_THRESHOLD = 0.22;
const FOLLOW_DEADZONE_PX = 26;
const SMOOTH_PATH_SEGMENT_STEPS = 12;
const RACING_LINE_SAMPLE_SPACING_M = 2;
const SAFETY_CAR_MARKER_KEY = '__safety_car__';
const SAFETY_CAR_LEAD_PROGRESS = 0.012;

function computeBounds(allCoords) {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;

  for (const coords of allCoords) {
    if (!coords?.length) continue;
    for (const [x, y] of coords) {
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
    }
  }

  return { minX, minY, maxX, maxY, width: maxX - minX, height: maxY - minY };
}

function normalizeDegrees(degrees) {
  const numeric = Number(degrees);
  if (!Number.isFinite(numeric)) return 0;
  return ((numeric % 360) + 360) % 360;
}

function formatBearingDegrees(degrees) {
  return String(Math.round(normalizeDegrees(degrees))).padStart(3, '0');
}

function normalizeViewBounds(viewBounds) {
  const x = Number(viewBounds?.x);
  const y = Number(viewBounds?.y);
  const width = Number(viewBounds?.width);
  const height = Number(viewBounds?.height);

  if (
    !Number.isFinite(x)
    || !Number.isFinite(y)
    || !Number.isFinite(width)
    || !Number.isFinite(height)
    || width <= 0
    || height <= 0
  ) {
    return null;
  }

  return {
    minX: x,
    minY: y,
    maxX: x + width,
    maxY: y + height,
    width,
    height,
  };
}

function rotationCenterForView(allCoords, viewBounds) {
  const fixedBounds = normalizeViewBounds(viewBounds);
  if (fixedBounds) {
    return {
      x: fixedBounds.minX + fixedBounds.width / 2,
      y: fixedBounds.minY + fixedBounds.height / 2,
    };
  }

  const bounds = computeBounds(allCoords);
  if (!Number.isFinite(bounds.width) || !Number.isFinite(bounds.height)) return null;
  return {
    x: bounds.minX + bounds.width / 2,
    y: bounds.minY + bounds.height / 2,
  };
}

function rotatePoint(point, degrees, center) {
  if (!center || normalizeDegrees(degrees) === 0) return point;

  const radians = (Number(degrees) * Math.PI) / 180;
  const cos = Math.cos(radians);
  const sin = Math.sin(radians);
  const dx = point[0] - center.x;
  const dy = point[1] - center.y;

  return [
    center.x + dx * cos - dy * sin,
    center.y + dx * sin + dy * cos,
  ];
}

function rotateCoordinatePath(coords, degrees, center) {
  if (!coords?.length || !center || normalizeDegrees(degrees) === 0) return coords || [];
  return coords.map((point) => rotatePoint(point, degrees, center));
}

function computeRenderBounds(allCoords, viewBounds) {
  const actualBounds = computeBounds(allCoords);
  if (
    Number.isFinite(actualBounds.width)
    && Number.isFinite(actualBounds.height)
    && actualBounds.width > 0
    && actualBounds.height > 0
  ) {
    return actualBounds;
  }

  return normalizeViewBounds(viewBounds) || actualBounds;
}

function computeViewportTransform(bounds, viewWidth, viewHeight) {
  if (bounds.width <= 0 || bounds.height <= 0 || viewWidth <= 0 || viewHeight <= 0) {
    return { scale: 1, offsetX: 0, offsetY: 0 };
  }

  const scale = Math.min(
    (viewWidth - PADDING * 2) / bounds.width,
    (viewHeight - PADDING * 2) / bounds.height,
  );

  const offsetX = (viewWidth - bounds.width * scale) / 2 - bounds.minX * scale;
  const offsetY = (viewHeight - bounds.height * scale) / 2 - bounds.minY * scale;

  return { scale, offsetX, offsetY };
}

function applyZoomToTransform(transform, bounds, viewWidth, viewHeight, zoomPercent) {
  const zoom = Math.max(0.5, Number(zoomPercent) / 100 || 1);
  if (zoom === 1 || bounds.width <= 0 || bounds.height <= 0) return transform;

  const baseCenterX = bounds.minX * transform.scale + transform.offsetX + (bounds.width * transform.scale) / 2;
  const baseCenterY = bounds.minY * transform.scale + transform.offsetY + (bounds.height * transform.scale) / 2;
  const viewCenterX = viewWidth / 2;
  const viewCenterY = viewHeight / 2;
  const nextScale = transform.scale * zoom;

  return {
    scale: nextScale,
    offsetX: viewCenterX - (baseCenterX - transform.offsetX) * zoom,
    offsetY: viewCenterY - (baseCenterY - transform.offsetY) * zoom,
  };
}

function toCanvasPoint(x, y, transform) {
  return {
    x: x * transform.scale + transform.offsetX,
    y: y * transform.scale + transform.offsetY,
  };
}

function shortestAngleDelta(from, to) {
  return Math.atan2(Math.sin(to - from), Math.cos(to - from));
}

function smoothStep(value) {
  const t = Math.min(1, Math.max(0, value));
  return t * t * (3 - 2 * t);
}

function routeProgressDelta(from, to, closed) {
  if (!closed) return to - from;

  let delta = to - from;
  if (delta < -0.5) delta += 1;
  if (delta > 0.5) delta -= 1;
  return delta;
}

function interpolateRouteProgress(from, to, t, closed) {
  const next = from + routeProgressDelta(from, to, closed) * t;
  if (!closed) return Math.min(1, Math.max(0, next));
  return ((next % 1) + 1) % 1;
}

function progressAtTime(animation, now) {
  if (!animation) return 0;
  const elapsed = Math.max(0, now - animation.startTime);
  const blendElapsed = Math.min(animation.duration, elapsed);
  const t = animation.duration > 0 ? blendElapsed / animation.duration : 1;
  const interpolated = interpolateRouteProgress(
    animation.fromProgress,
    animation.toProgress,
    t,
    animation.closed,
  );
  const predictionElapsed = Math.min(
    MARKER_PREDICTION_MAX_MS,
    Math.max(0, elapsed - animation.duration),
  );
  const predicted = interpolated + animation.progressRate * (predictionElapsed / 1000);
  if (!animation.closed) return Math.min(1, Math.max(0, predicted));
  return ((predicted % 1) + 1) % 1;
}

function lateralOffsetAtTime(animation, now) {
  if (!animation) return 0;
  const elapsed = Math.max(0, now - animation.startTime);
  const t = animation.duration > 0
    ? Math.min(1, elapsed / animation.duration)
    : 1;
  const from = Number(animation.fromLateralOffsetM || 0);
  const to = Number(animation.toLateralOffsetM || 0);
  const lateralSpeed = Number(animation.lateralSpeedMps || 0);
  const interpolated = from + (to - from) * t;
  const predictionElapsed = Math.min(
    MARKER_PREDICTION_MAX_MS,
    Math.max(0, elapsed - animation.duration),
  );
  return interpolated + lateralSpeed * (predictionElapsed / 1000);
}

function markerPoseForRoute(
  routeType,
  progress,
  transform,
  coords,
  trackMetrics,
  pit,
  pitMetrics,
  lateralOffsetM = 0,
  trackLengthM = 5000,
) {
  if (routeType === 'pit' && pit?.length >= 2) {
    return pathPointAndAngle(pit, progress, transform, pitMetrics, false);
  }

  const pose = pathPointAndAngle(coords, progress, transform, trackMetrics);
  const pixelsPerMeter = (
    (trackMetrics?.totalLength || 0) / Math.max(1, trackLengthM)
  ) * transform.scale;
  return {
    ...pose,
    x: pose.x - Math.sin(pose.angle) * lateralOffsetM * pixelsPerMeter,
    y: pose.y + Math.cos(pose.angle) * lateralOffsetM * pixelsPerMeter,
  };
}

function buildPathMetrics(coords) {
  if (!coords || coords.length < 2) return { cumulative: [0], totalLength: 0 };

  const cumulative = [0];
  for (let i = 0; i < coords.length - 1; i += 1) {
    const [x1, y1] = coords[i];
    const [x2, y2] = coords[i + 1];
    cumulative.push(cumulative[cumulative.length - 1] + Math.hypot(x2 - x1, y2 - y1));
  }

  return {
    cumulative,
    totalLength: cumulative[cumulative.length - 1],
  };
}

function progressToDistance(progress, metrics, closed = true) {
  if (!metrics?.totalLength) return 0;

  const numeric = Number(progress);
  if (!Number.isFinite(numeric)) return 0;
  if (!closed) {
    return Math.min(1, Math.max(0, numeric)) * metrics.totalLength;
  }
  if (Math.abs(numeric - 1) < 1e-9) return metrics.totalLength;

  const wrapped = ((numeric % 1) + 1) % 1;
  return wrapped * metrics.totalLength;
}

function progressRangeLength(startProgress, endProgress) {
  const start = Number(startProgress);
  const end = Number(endProgress);
  if (!Number.isFinite(start) || !Number.isFinite(end)) return 0;
  if (start <= end) return end - start;
  return (1 - start) + end;
}

function midpointProgress(startProgress, endProgress) {
  const start = Number(startProgress);
  const range = progressRangeLength(startProgress, endProgress);
  return (start + range / 2) % 1;
}

function pushDistinct(points, point) {
  const previous = points[points.length - 1];
  if (!previous || Math.hypot(previous[0] - point[0], previous[1] - point[1]) > 0.001) {
    points.push(point);
  }
}

function catmullRomPoint(p0, p1, p2, p3, t) {
  const t2 = t * t;
  const t3 = t2 * t;
  return [
    0.5 * (
      (2 * p1[0])
      + (-p0[0] + p2[0]) * t
      + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
      + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
    ),
    0.5 * (
      (2 * p1[1])
      + (-p0[1] + p2[1]) * t
      + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
      + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
    ),
  ];
}

function buildSmoothedPath(coords, closed = true) {
  if (!coords || coords.length < 3) return coords || [];

  if (closed && areSameCoord(coords[0], coords[coords.length - 1])) {
    const path = coords.slice(0, -1);
    if (path.length < 3) return coords;

    const smoothed = [path[0]];
    for (let i = 0; i < path.length; i += 1) {
      const p0 = path[(i - 1 + path.length) % path.length];
      const p1 = path[i];
      const p2 = path[(i + 1) % path.length];
      const p3 = path[(i + 2) % path.length];
      for (let step = 1; step <= SMOOTH_PATH_SEGMENT_STEPS; step += 1) {
        pushDistinct(smoothed, catmullRomPoint(p0, p1, p2, p3, step / SMOOTH_PATH_SEGMENT_STEPS));
      }
    }
    pushDistinct(smoothed, path[0]);
    return smoothed;
  }

  const smoothed = [coords[0]];
  for (let i = 0; i < coords.length - 1; i += 1) {
    const p0 = coords[Math.max(0, i - 1)];
    const p1 = coords[i];
    const p2 = coords[i + 1];
    const p3 = coords[Math.min(coords.length - 1, i + 2)];
    for (let step = 1; step <= SMOOTH_PATH_SEGMENT_STEPS; step += 1) {
      pushDistinct(smoothed, catmullRomPoint(p0, p1, p2, p3, step / SMOOTH_PATH_SEGMENT_STEPS));
    }
  }
  return smoothed;
}

function pathPointAndAngle(coords, progress, transform, metrics, closed = true) {
  if (!coords || coords.length < 2) return { x: 0, y: 0, angle: 0 };

  const cumulative = metrics?.cumulative || buildPathMetrics(coords).cumulative;
  const target = progressToDistance(progress, metrics || buildPathMetrics(coords), closed);
  let idx = 0;
  while (idx < cumulative.length - 2 && cumulative[idx + 1] < target) {
    idx += 1;
  }

  const segmentLength = Math.max(0.001, cumulative[idx + 1] - cumulative[idx]);
  const t = Math.min(1, Math.max(0, (target - cumulative[idx]) / segmentLength));
  const [x1, y1] = coords[idx];
  const [x2, y2] = coords[idx + 1];
  const point = toCanvasPoint(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t, transform);
  return {
    ...point,
    angle: Math.atan2((y2 - y1) * transform.scale, (x2 - x1) * transform.scale),
  };
}

function interpolatePath(coords, progress, transform, metrics, closed = true) {
  const { x, y } = pathPointAndAngle(coords, progress, transform, metrics, closed);
  return { x, y };
}

function hexToNumber(hex) {
  if (!hex) return 0x666666;
  return parseInt(hex.replace('#', ''), 16);
}

function drawPath(graphics, coords, transform, strokeStyle) {
  drawPolylinePath(
    graphics,
    coords.map(([x, y]) => toCanvasPoint(x, y, transform)),
    strokeStyle,
  );
}

function areSameCoord(a, b) {
  if (!a || !b) return false;
  return Math.hypot(a[0] - b[0], a[1] - b[1]) < 0.001;
}

function drawPolylinePath(graphics, points, strokeStyle) {
  if (!points?.length) return;
  graphics.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i += 1) {
    graphics.lineTo(points[i].x, points[i].y);
  }
  graphics.stroke(strokeStyle);
}

function offsetCanvasPoint(point, offset) {
  const normal = point.angle + Math.PI / 2;
  return {
    x: point.x + Math.cos(normal) * offset,
    y: point.y + Math.sin(normal) * offset,
  };
}

function drawProgressPath(graphics, coords, startProgress, endProgress, transform, metrics, strokeStyle) {
  if (!coords || coords.length < 2) return;

  const drawSegment = (start, end) => {
    const distance = progressRangeLength(start, end) * (metrics?.totalLength || coords.length);
    const steps = Math.max(8, Math.ceil(distance / 12));
    const first = interpolatePath(coords, start, transform, metrics);
    graphics.moveTo(first.x, first.y);
    for (let i = 1; i <= steps; i += 1) {
      const progress = start + ((end - start) * i) / steps;
      const point = interpolatePath(coords, progress, transform, metrics);
      graphics.lineTo(point.x, point.y);
    }
  };

  if (startProgress <= endProgress) {
    drawSegment(startProgress, endProgress);
  } else {
    drawSegment(startProgress, 1);
    drawSegment(0, endProgress);
  }
  graphics.stroke(strokeStyle);
}

function drawMetricRacingLine(
  graphics,
  coords,
  transform,
  metrics,
  trackLengthM,
  racingLineProfile,
) {
  if (!racingLineProfile?.length || !metrics?.totalLength) return;
  const pixelsPerMeter = (metrics.totalLength / Math.max(1, trackLengthM)) * transform.scale;
  const profile = racingLineProfile
    .map(([progress, offsetM]) => [
      ((Number(progress) % 1) + 1) % 1,
      Number(offsetM || 0),
    ])
    .sort((a, b) => a[0] - b[0]);
  const offsetAtProgress = (progress) => {
    let index = profile.length - 1;
    for (let candidate = 0; candidate < profile.length; candidate += 1) {
      if (profile[candidate][0] > progress) break;
      index = candidate;
    }
    const nextIndex = (index + 1) % profile.length;
    const previousIndex = (index - 1 + profile.length) % profile.length;
    const followingIndex = (nextIndex + 1) % profile.length;
    const startProgress = profile[index][0];
    let endProgress = profile[nextIndex][0];
    if (nextIndex === 0) endProgress += 1;
    const targetProgress = progress < startProgress ? progress + 1 : progress;
    const t = Math.min(1, Math.max(
      0,
      (targetProgress - startProgress) / Math.max(1e-9, endProgress - startProgress),
    ));
    const curved = catmullRomPoint(
      [0, profile[previousIndex][1]],
      [0, profile[index][1]],
      [0, profile[nextIndex][1]],
      [0, profile[followingIndex][1]],
      t,
    )[1];
    return Math.min(
      Math.max(profile[index][1], profile[nextIndex][1]),
      Math.max(Math.min(profile[index][1], profile[nextIndex][1]), curved),
    );
  };
  const steps = Math.max(
    256,
    Math.min(5000, Math.ceil(trackLengthM / RACING_LINE_SAMPLE_SPACING_M)),
  );
  const points = [];
  for (let index = 0; index <= steps; index += 1) {
    const progress = index / steps;
    const pose = pathPointAndAngle(coords, progress, transform, metrics);
    points.push(offsetCanvasPoint(
      pose,
      offsetAtProgress(progress % 1) * pixelsPerMeter,
    ));
  }
  drawPolylinePath(graphics, points, {
    width: 1.15,
    color: 0xb2b2c0,
    alpha: 0.78,
    cap: 'round',
    join: 'round',
  });
}

function drawKerbs(graphics, coords, transform, metrics) {
  const kerbSegments = [
    [0.14, 0.24],
    [0.35, 0.43],
    [0.60, 0.72],
    [0.84, 0.94],
  ];

  kerbSegments.forEach(([start, end], index) => {
    drawProgressPath(graphics, coords, start, end, transform, metrics, {
      width: 14,
      color: index % 2 === 0 ? 0xfff4f4 : 0xe10600,
      alpha: 0.88,
      cap: 'butt',
      join: 'round',
    });
    drawProgressPath(graphics, coords, start + 0.012, Math.min(end, start + 0.038), transform, metrics, {
      width: 14,
      color: index % 2 === 0 ? 0xe10600 : 0xfff4f4,
      alpha: 0.9,
      cap: 'butt',
      join: 'round',
    });
  });
}

function drawStartFinishLine(graphics, coords, sfIndex, transform) {
  const i = Math.min(sfIndex, coords.length - 2);
  const p1 = toCanvasPoint(coords[i][0], coords[i][1], transform);
  const p2 = toCanvasPoint(coords[i + 1][0], coords[i + 1][1], transform);

  const mx = (p1.x + p2.x) / 2;
  const my = (p1.y + p2.y) / 2;
  const angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);
  const perp = angle + Math.PI / 2;
  const halfLen = 22;

  const x1 = mx + Math.cos(perp) * halfLen;
  const y1 = my + Math.sin(perp) * halfLen;
  const x2 = mx - Math.cos(perp) * halfLen;
  const y2 = my - Math.sin(perp) * halfLen;

  graphics.moveTo(x1, y1);
  graphics.lineTo(x2, y2);
  graphics.stroke({ width: 8, color: 0x050508, alpha: 0.75, cap: 'round' });

  graphics.moveTo(x1, y1);
  graphics.lineTo(x2, y2);
  graphics.stroke({ width: 4, color: 0xffffff, alpha: 1, cap: 'round' });
}

function createLabel(text, x, y, color = 0xffffff) {
  const label = new Text({
    text,
    style: {
      fontFamily: 'Inter, Arial, sans-serif',
      fontSize: 11,
      fontWeight: '700',
      fill: color,
      letterSpacing: 1,
    },
  });
  label.x = x;
  label.y = y;
  return label;
}

function rectsOverlap(a, b, padding = 0) {
  return !(
    a.x + a.width + padding < b.x
    || b.x + b.width + padding < a.x
    || a.y + a.height + padding < b.y
    || b.y + b.height + padding < a.y
  );
}

function overlapArea(a, b, padding = 0) {
  if (!rectsOverlap(a, b, padding)) return 0;
  const left = Math.max(a.x - padding, b.x);
  const right = Math.min(a.x + a.width + padding, b.x + b.width);
  const top = Math.max(a.y - padding, b.y);
  const bottom = Math.min(a.y + a.height + padding, b.y + b.height);
  return Math.max(0, right - left) * Math.max(0, bottom - top);
}

function clampLabelPosition(label, x, y, viewport) {
  if (!viewport) return { x, y };

  return {
    x: Math.min(
      Math.max(LABEL_MARGIN, x),
      Math.max(LABEL_MARGIN, viewport.width - label.width - LABEL_MARGIN),
    ),
    y: Math.min(
      Math.max(LABEL_MARGIN, y),
      Math.max(LABEL_MARGIN, viewport.height - label.height - LABEL_MARGIN),
    ),
  };
}

function labelRect(label, x, y) {
  return {
    x,
    y,
    width: label.width,
    height: label.height,
  };
}

function addPlacedLabel(
  markersContainer,
  placedLabels,
  text,
  anchor,
  color,
  offsets,
  viewport,
) {
  const label = createLabel(text, 0, 0, color);
  let bestPosition = null;
  let bestRect = null;
  let bestScore = Infinity;

  offsets.forEach(([offsetX, offsetY], index) => {
    const position = clampLabelPosition(
      label,
      anchor.x + offsetX,
      anchor.y + offsetY,
      viewport,
    );
    const rect = labelRect(label, position.x, position.y);
    const score = placedLabels.reduce(
      (total, existing) => total + overlapArea(rect, existing, LABEL_COLLISION_PADDING),
      index * 0.1,
    );

    if (score === 0 && bestPosition === null) {
      bestPosition = position;
      bestRect = rect;
      bestScore = score;
      return;
    }

    if (score < bestScore) {
      bestPosition = position;
      bestRect = rect;
      bestScore = score;
    }
  });

  label.x = bestPosition?.x ?? anchor.x;
  label.y = bestPosition?.y ?? anchor.y;
  markersContainer.addChild(label);
  if (bestRect) placedLabels.push(bestRect);
  return label;
}

function drawTrackScene(
  trackGfx,
  markersContainer,
  coords,
  routeCoords,
  pitCoords,
  pitRouteCoords,
  pitBoxOffset,
  drsZones,
  sfIndex,
  landmarks,
  transform,
  trackMetrics,
  pitMetrics,
  viewport,
  trackLengthM,
  trackWidthM,
  racingLineProfile,
  physicalScale,
) {
  trackGfx.clear();
  markersContainer.removeChildren();
  const placedLabels = [];
  const activeRouteCoords = routeCoords?.length >= 2 ? routeCoords : coords;
  const activePitRouteCoords = pitRouteCoords?.length >= 2 ? pitRouteCoords : pitCoords;

  const pixelsPerMeter = (
    (trackMetrics?.totalLength || 0) / Math.max(1, trackLengthM)
  ) * transform.scale;
  const physicalTrackWidth = Math.max(4, trackWidthM * pixelsPerMeter);
  const surfaceWidth = physicalScale ? physicalTrackWidth : 18;

  drawPath(trackGfx, activeRouteCoords, transform, {
    width: surfaceWidth + (physicalScale ? 3 : 6),
    color: 0x07070b,
    alpha: 0.95,
    cap: 'round',
    join: 'round',
  });

  if (!physicalScale) drawKerbs(trackGfx, activeRouteCoords, transform, trackMetrics);

  drawPath(trackGfx, activeRouteCoords, transform, {
    width: surfaceWidth,
    color: 0x1e1e2e,
    alpha: 0.95,
    cap: 'round',
    join: 'round',
  });

  drawPath(trackGfx, activeRouteCoords, transform, {
    width: physicalScale ? Math.max(2, surfaceWidth - 3) : 10,
    color: 0x2a2a38,
    alpha: 1,
    cap: 'round',
    join: 'round',
  });

  if (!physicalScale) {
    drawPath(trackGfx, activeRouteCoords, transform, {
      width: 2,
      color: 0x777790,
      alpha: 0.65,
      cap: 'round',
      join: 'round',
    });
  }

  if (physicalScale) {
    drawMetricRacingLine(
      trackGfx,
      activeRouteCoords,
      transform,
      trackMetrics,
      trackLengthM,
      racingLineProfile,
    );
  }

  (drsZones || []).forEach((zone) => {
    if (!physicalScale) {
      drawProgressPath(trackGfx, activeRouteCoords, Number(zone.start), Number(zone.end), transform, trackMetrics, {
        width: 5,
        color: 0x2ecc71,
        alpha: 0.82,
        cap: 'butt',
        join: 'round',
      });
    }
    const labelProgress = midpointProgress(zone.start, zone.end);
    const labelPoint = interpolatePath(activeRouteCoords, labelProgress, transform, trackMetrics);
    addPlacedLabel(markersContainer, placedLabels, 'DRS', labelPoint, 0x2ecc71, [
      [-10, -28],
      [12, -28],
      [-10, 14],
      [12, 14],
      [-34, -14],
      [18, -44],
    ], viewport);
  });

  if (pitCoords?.length >= 2) {
    drawPath(trackGfx, activePitRouteCoords, transform, {
      width: 9,
      color: 0x2a1d0b,
      alpha: 0.95,
      cap: 'round',
      join: 'round',
    });

    drawPath(trackGfx, activePitRouteCoords, transform, {
      width: 2.5,
      color: 0xf39c12,
      alpha: 0.85,
      cap: 'round',
      join: 'round',
    });

    const entry = toCanvasPoint(activePitRouteCoords[0][0], activePitRouteCoords[0][1], transform);
    const exit = toCanvasPoint(
      activePitRouteCoords[activePitRouteCoords.length - 1][0],
      activePitRouteCoords[activePitRouteCoords.length - 1][1],
      transform,
    );
    addPlacedLabel(markersContainer, placedLabels, 'PIT IN', entry, 0xf39c12, [
      [-48, 14],
      [-52, -26],
      [10, 14],
      [-48, 30],
      [10, -26],
    ], viewport);
    addPlacedLabel(markersContainer, placedLabels, 'PIT OUT', exit, 0xf39c12, [
      [10, 14],
      [-58, 14],
      [10, -26],
      [-58, -26],
      [10, 30],
    ], viewport);

    for (let i = 0; i < 10; i += 1) {
      const slotPose = pathPointAndAngle(
        activePitRouteCoords,
        0.22 + i * 0.055,
        transform,
        pitMetrics,
        false,
      );
      const slot = offsetCanvasPoint(slotPose, pitBoxOffset);
      trackGfx.roundRect(slot.x - 7, slot.y - 7, 14, 10, 2);
      trackGfx.fill({ color: 0x111118, alpha: 0.95 });
      trackGfx.stroke({ width: 1, color: 0xf39c12, alpha: 0.42 });
    }

    const pitPoint = interpolatePath(activePitRouteCoords, 0.5, transform, pitMetrics, false);
    addPlacedLabel(markersContainer, placedLabels, 'PIT LANE', pitPoint, 0xf39c12, [
      [-28, -24],
      [-28, 16],
      [12, -18],
      [-72, -18],
      [12, 16],
    ], viewport);
  }

  drawStartFinishLine(trackGfx, activeRouteCoords, sfIndex, transform);

  const sfPoint = toCanvasPoint(coords[sfIndex][0], coords[sfIndex][1], transform);
  addPlacedLabel(markersContainer, placedLabels, 'S/F', sfPoint, 0xffffff, [
    [14, 18],
    [-30, 18],
    [14, -28],
    [-30, -28],
    [28, -6],
    [-44, -6],
  ], viewport);

  (landmarks || []).forEach((lm) => {
    if (lm.type === 'start_finish' || lm.type === 'pit') return;
    let point;
    if (lm.progress !== null && lm.progress !== undefined) {
      point = interpolatePath(activeRouteCoords, Number(lm.progress), transform, trackMetrics);
    } else {
      const idx = lm.track_index ?? 0;
      if (!coords[idx]) return;
      point = toCanvasPoint(coords[idx][0], coords[idx][1], transform);
    }
    addPlacedLabel(markersContainer, placedLabels, lm.label, point, 0x888899, [
      [8, -18],
      [12, 8],
      [-30, -18],
      [-34, 8],
      [8, -34],
      [-30, -34],
    ], viewport);
  });

  return placedLabels;
}

function formatPitStatus(driver) {
  if (!driver.in_pit) return '';
  const pitElapsed = (driver.pit_elapsed || 0).toFixed(1);
  if (driver.pit_phase === 'stop') {
    const stopElapsed = (driver.pit_stop_elapsed || 0).toFixed(1);
    return `BOX ${stopElapsed}s / PIT ${pitElapsed}s`;
  }
  if (driver.pit_phase === 'in') return `PIT IN ${pitElapsed}s`;
  if (driver.pit_phase === 'out') return `PIT OUT ${pitElapsed}s`;
  return `PIT ${pitElapsed}s`;
}

function formatDriverSpeed(driver) {
  const speed = Number(driver?.speed_kph || 0);
  if (!Number.isFinite(speed) || speed <= 0) return '—';
  return `${Math.round(speed)} km/h`;
}

function createDriverMarker() {
  const marker = new Container();
  marker.sortableChildren = true;

  const halo = new Graphics();
  const dot = new Graphics();
  const labelBg = new Graphics();
  const nameLabel = new Text({
    text: '',
    style: {
      fontFamily: 'Inter, Arial, sans-serif',
      fontSize: 10,
      fontWeight: '800',
      fill: 0xffffff,
      letterSpacing: 0.5,
    },
  });
  const pitLabel = new Text({
    text: '',
    style: {
      fontFamily: 'Inter, Arial, sans-serif',
      fontSize: 9,
      fontWeight: '700',
      fill: 0xf39c12,
      letterSpacing: 0,
    },
  });

  marker.addChild(halo, labelBg, nameLabel, pitLabel, dot);
  marker.halo = halo;
  marker.dot = dot;
  marker.labelBg = labelBg;
  marker.nameLabel = nameLabel;
  marker.pitLabel = pitLabel;
  marker.routeAnimation = null;

  return marker;
}

function createSafetyCarMarker() {
  const marker = new Container();
  const halo = new Graphics();
  const vehicle = new Container();
  const body = new Graphics();
  const wheels = new Graphics();
  const lightBar = new Graphics();
  const labelBg = new Graphics();
  const label = new Text({
    text: 'SAFETY CAR',
    style: {
      fontFamily: 'Inter, Arial, sans-serif',
      fontSize: 9,
      fontWeight: '900',
      fill: 0xf7d038,
      letterSpacing: 0.4,
    },
  });

  halo.circle(0, 0, 10);
  halo.fill({ color: 0xf39c12, alpha: 0.14 });
  halo.stroke({ width: 1.5, color: 0xf7d038, alpha: 0.85 });

  body.roundRect(-9, -4.5, 18, 9, 2.5);
  body.fill({ color: 0xf7d038, alpha: 1 });
  body.stroke({ width: 1, color: 0xffffff, alpha: 0.8 });
  body.roundRect(-2.5, -3.5, 7, 7, 1.5);
  body.fill({ color: 0x20232a, alpha: 0.95 });

  wheels.roundRect(-6.5, -5.5, 4, 2, 0.6);
  wheels.roundRect(3, -5.5, 4, 2, 0.6);
  wheels.roundRect(-6.5, 3.5, 4, 2, 0.6);
  wheels.roundRect(3, 3.5, 4, 2, 0.6);
  wheels.fill({ color: 0x050508, alpha: 1 });

  lightBar.circle(0, -4.8, 1.5);
  lightBar.circle(0, 4.8, 1.5);
  lightBar.fill({ color: 0xffa600, alpha: 1 });

  vehicle.addChild(wheels, body, lightBar);
  labelBg.roundRect(10, -21, label.width + 8, 14, 3);
  labelBg.fill({ color: 0x050508, alpha: 0.82 });
  labelBg.stroke({ width: 1, color: 0xf7d038, alpha: 0.65 });
  label.x = 14;
  label.y = -19;

  marker.addChild(halo, labelBg, label, vehicle);
  marker.dot = vehicle;
  marker.lightBar = lightBar;
  marker.routeAnimation = null;
  marker.zIndex = 1000;
  return marker;
}

export default function TrackCanvas({
  trackCoords,
  trackLengthM = 5000,
  trackWidthM = 12,
  carWidthM = 1.9,
  carLengthM = 5.0,
  racingLineProfile = [],
  pitLaneCoords,
  pitBoxOffset = 11,
  drsZones = [],
  startFinishIndex = 0,
  landmarks = [],
  viewBounds = null,
  displayRotationDeg = 0,
  showBearing = false,
  positions,
  playerDriverIds,
  speedMultiplier = 1,
  paused = false,
  racePhase = 'green',
  safetyCarStage = 'inactive',
  safetyCarVisible = false,
  safetyCarRoute = 'track',
  safetyCarProgress = null,
  safetyCarProgressRate = 0,
  safetyCarPitLaneProgress = 0,
}) {
  const containerRef = useRef(null);
  const appRef = useRef(null);
  const worldRef = useRef(null);
  const trackGfxRef = useRef(null);
  const markersRef = useRef(null);
  const dotsRef = useRef(new Map());
  const transformRef = useRef({ scale: 1, offsetX: 0, offsetY: 0 });
  const sourceTrackCoords = useMemo(() => normalizeCoordinatePath(trackCoords), [trackCoords]);
  const sourcePitLaneCoords = useMemo(() => normalizeCoordinatePath(pitLaneCoords), [pitLaneCoords]);
  const displayRotation = normalizeDegrees(displayRotationDeg);
  const displayBearing = formatBearingDegrees(displayRotation);
  const displayCenter = useMemo(
    () => rotationCenterForView([sourceTrackCoords, sourcePitLaneCoords], viewBounds),
    [sourceTrackCoords, sourcePitLaneCoords, viewBounds],
  );
  const normalizedTrackCoords = useMemo(
    () => rotateCoordinatePath(sourceTrackCoords, displayRotation, displayCenter),
    [sourceTrackCoords, displayRotation, displayCenter],
  );
  const normalizedPitLaneCoords = useMemo(
    () => rotateCoordinatePath(sourcePitLaneCoords, displayRotation, displayCenter),
    [sourcePitLaneCoords, displayRotation, displayCenter],
  );
  const smoothedTrackCoords = useMemo(() => buildSmoothedPath(normalizedTrackCoords, true), [normalizedTrackCoords]);
  const smoothedPitLaneCoords = useMemo(() => buildSmoothedPath(normalizedPitLaneCoords, false), [normalizedPitLaneCoords]);
  const trackMetrics = useMemo(() => buildPathMetrics(smoothedTrackCoords), [smoothedTrackCoords]);
  const pitMetrics = useMemo(() => buildPathMetrics(smoothedPitLaneCoords), [smoothedPitLaneCoords]);
  const metaRef = useRef({
    trackCoords: normalizedTrackCoords,
    smoothedTrackCoords,
    pitLaneCoords: normalizedPitLaneCoords,
    smoothedPitLaneCoords,
    pitBoxOffset,
    drsZones,
    startFinishIndex,
    landmarks,
    viewBounds,
    trackMetrics,
    pitMetrics,
    trackLengthM,
    trackWidthM,
    racingLineProfile,
  });
  const [appReady, setAppReady] = useState(false);
  const [zoomPercent, setZoomPercent] = useState(100);
  const [panOffset, setPanOffset] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [followDriverId, setFollowDriverId] = useState(null);
  const zoomRef = useRef(100);
  const panRef = useRef({ x: 0, y: 0 });
  const followDriverIdRef = useRef(null);
  const pausedRef = useRef(false);
  const lastPausedStateRef = useRef(paused);
  const dragRef = useRef(null);
  const lastViewKeyRef = useRef('');
  const playerDrivers = useMemo(
    () => (positions || [])
      .filter((driver) => (playerDriverIds || []).includes(driver.driver_id))
      .sort((a, b) => a.position - b.position),
    [positions, playerDriverIds],
  );
  const followedDriver = useMemo(
    () => playerDrivers.find((driver) => driver.driver_id === followDriverId) || null,
    [followDriverId, playerDrivers],
  );

  metaRef.current = {
    trackCoords: normalizedTrackCoords,
    smoothedTrackCoords,
    pitLaneCoords: normalizedPitLaneCoords,
    smoothedPitLaneCoords,
    pitBoxOffset,
    drsZones,
    startFinishIndex,
    landmarks,
    viewBounds,
    trackMetrics,
    pitMetrics,
    trackLengthM,
    trackWidthM,
    racingLineProfile,
  };
  zoomRef.current = zoomPercent;
  followDriverIdRef.current = followDriverId;
  pausedRef.current = paused;

  useEffect(() => {
    if (followDriverIdRef.current === null) {
      panRef.current = panOffset;
    }
  }, [panOffset]);

  const applyCameraPan = useCallback((pan = panRef.current) => {
    const world = worldRef.current;
    if (!world) return;
    world.x = Number(pan?.x || 0);
    world.y = Number(pan?.y || 0);
  }, []);

  useEffect(() => {
    applyCameraPan(panOffset);
  }, [applyCameraPan, panOffset]);

  const applyViewport = useCallback(() => {
    const app = appRef.current;
    const {
      trackCoords: coords,
      smoothedTrackCoords: routeCoords,
      pitLaneCoords: pit,
      smoothedPitLaneCoords: pitRoute,
      pitBoxOffset: boxOffset,
      drsZones: drs,
      startFinishIndex: sf,
      landmarks: lms,
      viewBounds: fixedViewBounds,
      trackMetrics: mainMetrics,
      pitMetrics: laneMetrics,
    } =
      metaRef.current;

    if (!app || !coords?.length || !trackGfxRef.current || !markersRef.current) return null;

    const bounds = computeRenderBounds([coords, pit], fixedViewBounds);
    const baseTransform = computeViewportTransform(bounds, app.screen.width, app.screen.height);
    const zoomedTransform = applyZoomToTransform(
      baseTransform,
      bounds,
      app.screen.width,
      app.screen.height,
      zoomRef.current,
    );
    const transform = zoomedTransform;
    transformRef.current = transform;

    drawTrackScene(
      trackGfxRef.current,
      markersRef.current,
      coords,
      routeCoords,
      pit,
      pitRoute,
      boxOffset,
      drs,
      sf,
      lms,
      transform,
      mainMetrics,
      laneMetrics,
      { width: app.screen.width, height: app.screen.height },
      trackLengthM,
      trackWidthM,
      racingLineProfile,
      zoomRef.current >= PHYSICAL_ZOOM_MIN_PERCENT,
    );
    applyCameraPan();
    return transform;
  }, [applyCameraPan, racingLineProfile, trackLengthM, trackWidthM]);

  const handleZoomChange = useCallback((level) => {
    if (level >= PHYSICAL_ZOOM_MIN_PERCENT) {
      const targetDriverId = followDriverIdRef.current ?? playerDrivers[0]?.driver_id ?? null;
      if (targetDriverId === null) return;
      panRef.current = { x: 0, y: 0 };
      setPanOffset({ x: 0, y: 0 });
      setFollowDriverId(targetDriverId);
      setZoomPercent(level);
      return;
    }

    setFollowDriverId(null);
    setPanOffset(panRef.current);
    setZoomPercent(level);
    if (level === 100) {
      panRef.current = { x: 0, y: 0 };
      setPanOffset({ x: 0, y: 0 });
    }
  }, [playerDrivers]);

  const resetView = useCallback(() => {
    setFollowDriverId(null);
    panRef.current = { x: 0, y: 0 };
    setZoomPercent(100);
    setPanOffset({ x: 0, y: 0 });
  }, []);

  const handlePointerDown = useCallback((event) => {
    if (event.button !== 0) return;
    setPanOffset(panRef.current);
    setFollowDriverId(null);
    dragRef.current = {
      pointerId: event.pointerId,
      startX: event.clientX,
      startY: event.clientY,
      panX: panRef.current.x,
      panY: panRef.current.y,
    };
    event.currentTarget.setPointerCapture?.(event.pointerId);
    setIsDragging(true);
  }, []);

  const handlePointerMove = useCallback((event) => {
    const drag = dragRef.current;
    if (!drag || drag.pointerId !== event.pointerId) return;

    setPanOffset({
      x: drag.panX + event.clientX - drag.startX,
      y: drag.panY + event.clientY - drag.startY,
    });
  }, []);

  const finishPointerDrag = useCallback((event) => {
    const drag = dragRef.current;
    if (!drag || drag.pointerId !== event.pointerId) return;
    dragRef.current = null;
    event.currentTarget.releasePointerCapture?.(event.pointerId);
    setIsDragging(false);
  }, []);

  const toggleFollowDriver = useCallback((driverId) => {
    setFollowDriverId((current) => {
      if (current === driverId) {
        setPanOffset(panRef.current);
        return null;
      }
      setPanOffset(panRef.current);
      return driverId;
    });
  }, []);

  useEffect(() => {
    if (!containerRef.current) return undefined;

    let destroyed = false;
    const app = new Application();
    const dots = dotsRef.current;

    app.init({
      background: 0x0a0a0f,
      resizeTo: containerRef.current,
      antialias: true,
    }).then(() => {
      if (destroyed) {
        app.destroy(true, { children: true });
        return;
      }

      containerRef.current?.appendChild(app.canvas);
      appRef.current = app;

      const world = new Container();
      worldRef.current = world;
      app.stage.addChild(world);
      applyCameraPan();

      const trackGfx = new Graphics();
      trackGfxRef.current = trackGfx;
      world.addChild(trackGfx);

      const markers = new Container();
      markersRef.current = markers;
      world.addChild(markers);

      let previousFrameTime = performance.now();
      const animateMarkers = () => {
        const now = performance.now();
        const frameSeconds = Math.min(0.05, Math.max(0.001, (now - previousFrameTime) / 1000));
        previousFrameTime = now;
        const appInstance = appRef.current;
        const {
          smoothedTrackCoords: routeCoords,
          smoothedPitLaneCoords: pitRoute,
          trackMetrics: mainMetrics,
          pitMetrics: laneMetrics,
          trackLengthM: routeLengthM,
        } = metaRef.current;
        const transform = transformRef.current;
        let followedPose = null;

        dots.forEach((marker, driverId) => {
          const animation = marker.routeAnimation;
          if (!animation || !routeCoords?.length) return;

          const progress = progressAtTime(animation, now);
          const lateralOffsetM = lateralOffsetAtTime(animation, now);
          const targetPose = markerPoseForRoute(
            animation.routeType,
            progress,
            transform,
            routeCoords,
            mainMetrics,
            pitRoute,
            laneMetrics,
            lateralOffsetM,
            routeLengthM,
          );

          const transition = animation.routeTransition;
          const transitionProgress = transition
            ? (now - transition.startTime) / transition.duration
            : 1;
          const transitionBlend = smoothStep(transitionProgress);
          const x = transition
            ? transition.fromX + (targetPose.x - transition.fromX) * transitionBlend
            : targetPose.x;
          const y = transition
            ? transition.fromY + (targetPose.y - transition.fromY) * transitionBlend
            : targetPose.y;

          marker.x = x;
          marker.y = y;
          if (driverId === followDriverIdRef.current) {
            followedPose = {
              x: x + Number(worldRef.current?.x || 0),
              y: y + Number(worldRef.current?.y || 0),
            };
          }
          if (Number.isFinite(targetPose.angle)) {
            if (transition) {
              marker.dot.rotation = transition.fromAngle + shortestAngleDelta(
                transition.fromAngle,
                targetPose.angle,
              ) * transitionBlend;
            } else {
              marker.dot.rotation += shortestAngleDelta(
                marker.dot.rotation,
                targetPose.angle,
              ) * MARKER_ROTATION_LERP;
            }
          }
          if (transition && transitionProgress >= 1) {
            animation.routeTransition = null;
          }
          if (marker.lightBar) {
            marker.lightBar.alpha = Math.floor(now / 180) % 2 === 0 ? 1 : 0.3;
          }
        });

        if (followedPose && appInstance && !pausedRef.current) {
          const physicalScale = zoomRef.current >= PHYSICAL_ZOOM_MIN_PERCENT;
          const deadzonePx = physicalScale ? 0 : FOLLOW_DEADZONE_PX;
          const errorX = appInstance.screen.width / 2 - followedPose.x;
          const errorY = appInstance.screen.height / 2 - followedPose.y;
          const targetPan = {
            x: panRef.current.x + (
              Math.abs(errorX) > deadzonePx
                ? errorX - Math.sign(errorX) * deadzonePx
                : 0
            ),
            y: panRef.current.y + (
              Math.abs(errorY) > deadzonePx
                ? errorY - Math.sign(errorY) * deadzonePx
                : 0
            ),
          };
          const followStrength = physicalScale ? 14 : 4;
          const followBlend = 1 - Math.exp(-followStrength * frameSeconds);
          const nextPan = {
            x: panRef.current.x + (targetPan.x - panRef.current.x) * followBlend,
            y: panRef.current.y + (targetPan.y - panRef.current.y) * followBlend,
          };
          if (
            Math.abs(nextPan.x - panRef.current.x) > 0.1
            || Math.abs(nextPan.y - panRef.current.y) > 0.1
          ) {
            panRef.current = nextPan;
            applyCameraPan(nextPan);
          }
        }
      };
      app.ticker.add(animateMarkers);
      app.__animateMarkers = animateMarkers;

      setAppReady(true);
    });

    const handleResize = () => applyViewport();
    window.addEventListener('resize', handleResize);

    return () => {
      destroyed = true;
      setAppReady(false);
      window.removeEventListener('resize', handleResize);
      dots.clear();
      if (appRef.current?.__animateMarkers) {
        appRef.current.ticker.remove(appRef.current.__animateMarkers);
        appRef.current.__animateMarkers = null;
      }
      trackGfxRef.current = null;
      markersRef.current = null;
      worldRef.current = null;
      if (appRef.current) {
        appRef.current.destroy(true, { children: true });
        appRef.current = null;
      }
    };
  }, [applyCameraPan, applyViewport]);

  useEffect(() => {
    if (!appReady || !normalizedTrackCoords?.length) return;
    applyViewport();
  }, [
    appReady,
    normalizedTrackCoords,
    smoothedTrackCoords,
    normalizedPitLaneCoords,
    smoothedPitLaneCoords,
    pitBoxOffset,
    drsZones,
    startFinishIndex,
    landmarks,
    viewBounds,
    trackMetrics,
    pitMetrics,
    zoomPercent,
    applyViewport,
  ]);

  useEffect(() => {
    const world = worldRef.current;
    if (!appReady || !world || !normalizedTrackCoords?.length || !positions?.length) return;

    const transform = transformRef.current;
    const playerSet = new Set(playerDriverIds || []);
    const pitRoute = smoothedPitLaneCoords?.length >= 2 ? smoothedPitLaneCoords : null;
    const orderedPositions = [...positions].sort((a, b) => a.position - b.position);
    const viewKey = `${zoomPercent}:${transform.scale.toFixed(4)}`;
    const viewChanged = lastViewKeyRef.current !== viewKey;
    lastViewKeyRef.current = viewKey;
    const now = performance.now();
    const justResumed = lastPausedStateRef.current && !paused;

    orderedPositions.forEach((driver) => {
      if (driver.retired) {
        const old = dotsRef.current.get(driver.driver_id);
        if (old) {
          world.removeChild(old);
          old.destroy();
          dotsRef.current.delete(driver.driver_id);
        }
        return;
      }

      let marker = dotsRef.current.get(driver.driver_id);
      const isNewMarker = !marker;
      if (!marker) {
        marker = createDriverMarker();
        world.addChild(marker);
        dotsRef.current.set(driver.driver_id, marker);
      }

      const routeType = driver.in_pit && pitRoute ? 'pit' : 'track';
      const routeProgress = routeType === 'pit'
        ? Math.min(1, Math.max(0, driver.pit_lane_progress ?? 0))
        : ((driver.progress % 1) + 1) % 1;
      const serverProgressRate = routeType === 'pit'
        ? Number(driver.pit_lane_progress_rate || 0)
        : Number(driver.progress_rate || 0);
      const displayProgressRate = (Number.isFinite(serverProgressRate) ? serverProgressRate : 0)
        * (driver.finished || driver.retired ? 0 : 1)
        * (paused ? 0 : speedMultiplier);
      const closed = routeType !== 'pit';
      const previousAnimation = marker.routeAnimation;
      const previousProgress = previousAnimation
        ? progressAtTime(previousAnimation, now)
        : routeProgress;
      const routeLateralOffsetM = routeType === 'pit'
        ? 0
        : Number(driver.lateral_offset_m || 0);
      const previousLateralOffsetM = previousAnimation
        ? lateralOffsetAtTime(previousAnimation, now)
        : routeLateralOffsetM;
      const routeChanged = Boolean(previousAnimation && previousAnimation.routeType !== routeType);
      const progressJump = Math.abs(routeProgressDelta(previousProgress, routeProgress, closed));
      const progressDiscontinuous = !paused
        && !justResumed
        && !routeChanged
        && progressJump > MARKER_ROUTE_JUMP_THRESHOLD;
      const resetRouteProgress = isNewMarker || routeChanged || progressDiscontinuous;
      const previousTransition = previousAnimation?.routeTransition;
      const transitionStillActive = previousTransition
        && now < previousTransition.startTime + previousTransition.duration;
      let routeTransition = transitionStillActive && !viewChanged ? previousTransition : null;

      if (routeChanged && !viewChanged) {
        routeTransition = {
          fromX: marker.x,
          fromY: marker.y,
          fromAngle: marker.dot.rotation,
          startTime: now,
          duration: MARKER_ROUTE_TRANSITION_MS,
        };
      }

      const holdVisualProgress = paused || justResumed;
      const fromProgress = holdVisualProgress
        ? previousProgress
        : (resetRouteProgress ? routeProgress : previousProgress);

      marker.routeAnimation = {
        routeType,
        fromProgress,
        toProgress: holdVisualProgress ? previousProgress : routeProgress,
        startTime: now,
        duration: holdVisualProgress || resetRouteProgress ? 1 : MARKER_INTERPOLATION_MS,
        closed,
        progressRate: displayProgressRate,
        fromLateralOffsetM: previousLateralOffsetM,
        toLateralOffsetM: holdVisualProgress
          ? previousLateralOffsetM
          : routeLateralOffsetM,
        lateralSpeedMps: paused
          ? 0
          : Number(driver.lateral_speed_mps || 0) * speedMultiplier,
        routeTransition,
      };

      if (isNewMarker || progressDiscontinuous || viewChanged) {
        const { x, y, angle } = markerPoseForRoute(
          routeType,
          fromProgress,
          transform,
          smoothedTrackCoords,
          trackMetrics,
          pitRoute,
          pitMetrics,
          previousLateralOffsetM,
          trackLengthM,
        );
        marker.x = x;
        marker.y = y;
        marker.dot.rotation = angle;
      }

      const color = hexToNumber(driver.team_color);
      const radius = playerSet.has(driver.driver_id) ? 6 : 4;
      const pitStatus = formatPitStatus(driver);

      marker.zIndex = driver.position;

      marker.halo.clear();
      if (driver.in_pit) {
        marker.halo.circle(0, 0, radius + 3);
        marker.halo.stroke({ width: 1.8, color: 0xf39c12, alpha: 0.9 });
      } else if (playerSet.has(driver.driver_id)) {
        marker.halo.circle(0, 0, radius + 4);
        marker.halo.stroke({ width: 1.8, color: 0xff1801, alpha: 0.85 });
      }

      marker.dot.clear();
      const physicalScale = zoomPercent >= PHYSICAL_ZOOM_MIN_PERCENT && !driver.in_pit;
      const pixelsPerMeter = (
        (trackMetrics?.totalLength || 0) / Math.max(1, trackLengthM)
      ) * transform.scale;
      if (physicalScale) {
        const physicalLengthPx = Math.max(2, Number(driver.car_length_m || carLengthM) * pixelsPerMeter);
        const physicalWidthPx = Math.max(1, Number(driver.car_width_m || carWidthM) * pixelsPerMeter);
        marker.dot.roundRect(
          -physicalLengthPx / 2,
          -physicalWidthPx / 2,
          physicalLengthPx,
          physicalWidthPx,
          Math.min(2, physicalWidthPx * 0.3),
        );
      } else {
        marker.dot.moveTo(radius + 3, 0);
        marker.dot.lineTo(-radius, -radius * 0.72);
        marker.dot.lineTo(-radius * 0.55, 0);
        marker.dot.lineTo(-radius, radius * 0.72);
        marker.dot.closePath();
      }
      marker.dot.fill({ color, alpha: driver.in_pit ? 0.75 : 1 });
      marker.dot.stroke({ width: 1, color: 0xffffff, alpha: 0.45 });

      marker.nameLabel.text = `${driver.position} ${driver.name}`;
      marker.pitLabel.text = pitStatus;
      marker.nameLabel.visible = true;
      marker.pitLabel.visible = driver.in_pit && pitStatus.length > 0;

      const labelWidth = Math.max(marker.nameLabel.width, marker.pitLabel.visible ? marker.pitLabel.width : 0);
      const labelHeight = marker.pitLabel.visible ? 25 : 14;
      const labelOffsetX = 9;
      const labelOffsetY = driver.in_pit ? -28 : -22;
      marker.nameLabel.x = labelOffsetX;
      marker.nameLabel.y = labelOffsetY;

      marker.pitLabel.x = labelOffsetX;
      marker.pitLabel.y = labelOffsetY + 12;

      marker.labelBg.clear();
      marker.labelBg.visible = true;
      marker.labelBg.roundRect(labelOffsetX - 4, labelOffsetY - 2, labelWidth + 8, labelHeight, 4);
      marker.labelBg.fill({ color: 0x0a0a0f, alpha: driver.in_pit ? 0.86 : 0.7 });
      marker.labelBg.stroke({
        width: 1,
        color: driver.in_pit ? 0xf39c12 : color,
        alpha: driver.in_pit ? 0.9 : 0.55,
      });
    });

    const existingSafetyCar = dotsRef.current.get(SAFETY_CAR_MARKER_KEY);
    if (racePhase !== 'sc' || !safetyCarVisible) {
      if (existingSafetyCar) {
        world.removeChild(existingSafetyCar);
        existingSafetyCar.destroy({ children: true });
        dotsRef.current.delete(SAFETY_CAR_MARKER_KEY);
      }
    } else {
      const onTrackLeader = orderedPositions.find(
        (driver) => !driver.retired && !driver.finished && !driver.in_pit,
      );
      const serverSafetyCarProgress = Number(safetyCarProgress);
      const hasServerSafetyCarProgress = safetyCarProgress !== null
        && Number.isFinite(serverSafetyCarProgress);
      if (hasServerSafetyCarProgress || onTrackLeader || safetyCarRoute === 'pit') {
        let safetyCar = existingSafetyCar;
        const isNewSafetyCar = !safetyCar;
        if (!safetyCar) {
          safetyCar = createSafetyCarMarker();
          world.addChild(safetyCar);
          dotsRef.current.set(SAFETY_CAR_MARKER_KEY, safetyCar);
        }

        const routeType = safetyCarRoute === 'pit' && pitRoute ? 'pit' : 'track';
        const fallbackProgress = Number(onTrackLeader?.progress || 0) + SAFETY_CAR_LEAD_PROGRESS;
        const rawRouteProgress = routeType === 'pit'
          ? Number(safetyCarPitLaneProgress || 0)
          : (hasServerSafetyCarProgress ? serverSafetyCarProgress : fallbackProgress);
        const routeProgress = routeType === 'pit'
          ? Math.min(1, Math.max(0, rawRouteProgress))
          : ((rawRouteProgress % 1) + 1) % 1;
        const serverProgressRate = routeType === 'pit'
          ? 0
          : (hasServerSafetyCarProgress
            ? Number(safetyCarProgressRate || 0)
            : Number(onTrackLeader?.progress_rate || 0));
        const displayProgressRate = (Number.isFinite(serverProgressRate) ? serverProgressRate : 0)
          * (paused ? 0 : speedMultiplier);
        const previousAnimation = safetyCar.routeAnimation;
        const previousProgress = previousAnimation
          ? progressAtTime(previousAnimation, now)
          : routeProgress;
        const closed = routeType !== 'pit';
        const routeChanged = Boolean(previousAnimation && previousAnimation.routeType !== routeType);
        const progressJump = Math.abs(routeProgressDelta(previousProgress, routeProgress, closed));
        const progressDiscontinuous = !paused
          && !justResumed
          && !routeChanged
          && progressJump > MARKER_ROUTE_JUMP_THRESHOLD;
        const resetProgress = isNewSafetyCar || routeChanged || progressDiscontinuous;
        const previousTransition = previousAnimation?.routeTransition;
        const transitionStillActive = previousTransition
          && now < previousTransition.startTime + previousTransition.duration;
        let routeTransition = transitionStillActive && !viewChanged ? previousTransition : null;
        if (routeChanged && !viewChanged) {
          routeTransition = {
            fromX: safetyCar.x,
            fromY: safetyCar.y,
            fromAngle: safetyCar.dot.rotation,
            startTime: now,
            duration: MARKER_ROUTE_TRANSITION_MS,
          };
        }
        const holdVisualProgress = paused || justResumed;
        const fromProgress = holdVisualProgress
          ? previousProgress
          : (resetProgress ? routeProgress : previousProgress);

        safetyCar.routeAnimation = {
          routeType,
          fromProgress,
          toProgress: holdVisualProgress ? previousProgress : routeProgress,
          startTime: now,
          duration: holdVisualProgress || resetProgress ? 1 : MARKER_INTERPOLATION_MS,
          closed,
          progressRate: displayProgressRate,
          routeTransition,
        };

        if (isNewSafetyCar || progressDiscontinuous || viewChanged) {
          const { x, y, angle } = markerPoseForRoute(
            routeType,
            fromProgress,
            transform,
            smoothedTrackCoords,
            trackMetrics,
            pitRoute,
            pitMetrics,
          );
          safetyCar.x = x;
          safetyCar.y = y;
          safetyCar.dot.rotation = angle;
        }
      }
    }
    lastPausedStateRef.current = paused;
  }, [
    appReady,
    normalizedTrackCoords,
    smoothedTrackCoords,
    trackMetrics,
    normalizedPitLaneCoords,
    smoothedPitLaneCoords,
    pitMetrics,
    positions,
    playerDriverIds,
    zoomPercent,
    speedMultiplier,
    paused,
    racePhase,
    safetyCarStage,
    safetyCarVisible,
    safetyCarRoute,
    safetyCarProgress,
    safetyCarProgressRate,
    safetyCarPitLaneProgress,
    trackLengthM,
    carWidthM,
    carLengthM,
  ]);

  useEffect(() => {
    if (
      followDriverId !== null
      && !playerDrivers.some((driver) => driver.driver_id === followDriverId && !driver.retired)
    ) {
      setFollowDriverId(null);
    }
  }, [followDriverId, playerDrivers]);

  return (
    <div className="track-canvas glass-panel">
      <div className="track-canvas__header">
        <span>TRACK VIEW</span>
        <div className="track-canvas__tools">
          {playerDrivers.length > 0 && (
            <div className="track-canvas__follow" aria-label="Follow player driver">
              {playerDrivers.map((driver) => (
                <button
                  key={driver.driver_id}
                  type="button"
                  className={`track-canvas__follow-btn ${followDriverId === driver.driver_id ? 'track-canvas__follow-btn--active' : ''}`}
                  onClick={() => toggleFollowDriver(driver.driver_id)}
                  aria-pressed={followDriverId === driver.driver_id}
                  disabled={driver.retired}
                >
                  {driver.name}
                </button>
              ))}
            </div>
          )}
          {followedDriver && (
            <div className="track-canvas__speed-hud" aria-label={`${followedDriver.name} live speed`}>
              <span className="track-canvas__speed-driver">{followedDriver.name}</span>
              <span className="track-canvas__speed-value">{formatDriverSpeed(followedDriver)}</span>
              {followedDriver.drs_active && (
                <span className="track-canvas__speed-flag track-canvas__speed-flag--drs">DRS</span>
              )}
              {followedDriver.in_pit && (
                <span className="track-canvas__speed-flag track-canvas__speed-flag--pit">PIT</span>
              )}
            </div>
          )}
          <div className="track-canvas__zoom" aria-label="Track zoom">
            {ZOOM_LEVELS.map((level) => (
              <button
                key={level}
                type="button"
                className={`track-canvas__zoom-btn ${zoomPercent === level ? 'track-canvas__zoom-btn--active' : ''}`}
                onClick={() => handleZoomChange(level)}
                aria-pressed={zoomPercent === level}
                aria-label={level >= PHYSICAL_ZOOM_MIN_PERCENT
                  ? `${level}% physical 1.9 by 5.0 metre car view`
                  : `${level}% track zoom`}
                data-testid={level === DETAIL_ZOOM_PERCENT ? 'track-detail-view' : undefined}
                disabled={level >= PHYSICAL_ZOOM_MIN_PERCENT && playerDrivers.length === 0}
                title={level >= PHYSICAL_ZOOM_MIN_PERCENT ? 'Physical 1.9×5.0m car view' : undefined}
              >
                {level}%
              </button>
            ))}
            <button
              type="button"
              className="track-canvas__reset-btn"
              onClick={resetView}
            >
              RESET
            </button>
          </div>
          <span className="track-canvas__legend">
            <span className="track-canvas__legend-item track-canvas__legend-item--sf">S/F</span>
            <span className="track-canvas__legend-item track-canvas__legend-item--pit">PIT</span>
            <span className="track-canvas__legend-item track-canvas__legend-item--drs">DRS</span>
            {zoomPercent >= PHYSICAL_ZOOM_MIN_PERCENT && (
              <span className="track-canvas__legend-item">PHYSICAL SCALE</span>
            )}
          </span>
        </div>
      </div>
      <div
        className={`track-canvas__viewport ${isDragging ? 'track-canvas__viewport--dragging' : ''}`}
        ref={containerRef}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={finishPointerDrag}
        onPointerCancel={finishPointerDrag}
        onPointerLeave={finishPointerDrag}
      >
        {showBearing && (
          <div className="track-canvas__bearing" aria-label={`True north bearing ${displayBearing} degrees`}>
            <span
              className="track-canvas__bearing-arrow"
              style={{ '--bearing-rotation': `${displayRotation}deg` }}
              aria-hidden="true"
            >
              <span className="track-canvas__bearing-arrow-head" />
              <span className="track-canvas__bearing-arrow-line" />
            </span>
            <span className="track-canvas__bearing-text">
              <span className="track-canvas__bearing-label">N</span>
              <span className="track-canvas__bearing-value">{displayBearing}°</span>
            </span>
          </div>
        )}
      </div>
    </div>
  );
}
