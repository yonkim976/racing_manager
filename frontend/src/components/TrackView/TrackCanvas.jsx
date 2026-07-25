import { useEffect, useRef, useState, useCallback, useMemo } from 'react';
import { Application, Container, Graphics, Text } from 'pixi.js';
import {
  normalizeCoordinatePath,
  sampleCatmullRomClosed,
} from '../../utils/circuitGeometry';
import {
  appendPoseTickToBuffers,
  bufferedWorldPoseAtTime,
} from './posePlayback';
import './TrackCanvas.css';

const PADDING = 48;
const LABEL_MARGIN = 6;
const LABEL_COLLISION_PADDING = 5;
const PHYSICAL_ZOOM_MIN_PERCENT = 700;
const DEFAULT_ZOOM_PERCENT = 1000;
const DETAIL_ZOOM_PERCENT = 1500;
const ZOOM_LEVELS = [PHYSICAL_ZOOM_MIN_PERCENT, DEFAULT_ZOOM_PERCENT, DETAIL_ZOOM_PERCENT];
const MARKER_INTERPOLATION_MS = 50;
const MARKER_WORLD_INTERPOLATION_FALLBACK_MS = 55;
const MARKER_WORLD_INTERPOLATION_MIN_MS = 32;
const MARKER_WORLD_INTERPOLATION_MAX_MS = 140;
const MARKER_WORLD_PREDICTION_MAX_MS = 120;
const MARKER_WORLD_PREDICTION_MAX_DISTANCE_M = 45;
const POSE_PLAYBACK_DELAY_MS = 120;
const MARKER_ROUTE_TRANSITION_MS = 240;
const MARKER_PREDICTION_MAX_MS = 30;
const MARKER_PREDICTION_MAX_DISTANCE_M = 1.0;
const MARKER_ROUTE_JUMP_THRESHOLD = 0.22;
const FOLLOW_DEADZONE_PX = 26;
const PHYSICAL_FOLLOW_DEADZONE_PX = 1.5;
const PHYSICAL_FOLLOW_STRENGTH = 8;
const RACING_LINE_SAMPLE_SPACING_M = 2;
const TRACK_BOUNDARY_SAMPLE_SPACING_M = 1;
const TRACK_BOUNDARY_TANGENT_SAMPLE_M = 1.5;
const TRACK_RENDER_SAMPLE_SPACING = 0.75;
const SAFETY_CAR_MARKER_KEY = '__safety_car__';
const SAFETY_CAR_LEAD_PROGRESS = 0.012;
const MANEUVER_PHASE_COLORS = {
  approach: 0xf5a623,
  pull_out: 0xff7a00,
  overlap: 0x00d8ff,
  three_wide: 0xffb020,
  four_wide: 0xff5c35,
  clear: 0x34d058,
  merge: 0xb26bff,
  abort: 0x9aa0a6,
};

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

function closedPathControls(coords) {
  if (!coords?.length) return [];
  const controls = [...coords];
  const first = controls[0];
  const last = controls[controls.length - 1];
  if (
    controls.length > 1
    && Math.hypot(first[0] - last[0], first[1] - last[1]) <= 1e-6
  ) {
    controls.pop();
  }
  return controls;
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
  const rawPrediction = animation.progressRate * (predictionElapsed / 1000);
  const predictionLimit = Math.max(
    0,
    Number(animation.maximumPredictionProgress || 0),
  );
  const prediction = predictionLimit > 0
    ? Math.max(-predictionLimit, Math.min(predictionLimit, rawPrediction))
    : rawPrediction;
  const predicted = interpolated + prediction;
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

function worldPoseAtTime(animation, now) {
  if (!animation?.usesWorldPose) return null;
  const elapsed = Math.max(0, now - animation.startTime);
  const t = animation.duration > 0
    ? Math.min(1, elapsed / animation.duration)
    : 1;
  const path = Array.isArray(animation.worldPath) ? animation.worldPath : [];
  let fromSample = {
    xM: Number(animation.fromWorldXM || 0),
    yM: Number(animation.fromWorldYM || 0),
    headingRad: Number(animation.fromWorldHeadingRad || 0),
  };
  let toSample = {
    xM: Number(animation.toWorldXM || 0),
    yM: Number(animation.toWorldYM || 0),
    headingRad: Number(animation.toWorldHeadingRad || 0),
  };
  let segmentT = t;

  if (path.length >= 2) {
    const firstTime = Number(path[0].simulationTimeS);
    const lastTime = Number(path[path.length - 1].simulationTimeS);
    if (Number.isFinite(firstTime) && Number.isFinite(lastTime) && lastTime > firstTime) {
      const targetTime = firstTime + (lastTime - firstTime) * t;
      let segmentIndex = 0;
      while (
        segmentIndex < path.length - 2
        && Number(path[segmentIndex + 1].simulationTimeS) < targetTime
      ) {
        segmentIndex += 1;
      }
      fromSample = path[segmentIndex];
      toSample = path[segmentIndex + 1];
      const segmentStart = Number(fromSample.simulationTimeS);
      const segmentEnd = Number(toSample.simulationTimeS);
      segmentT = segmentEnd > segmentStart
        ? Math.min(1, Math.max(0, (targetTime - segmentStart) / (segmentEnd - segmentStart)))
        : 1;
    } else {
      const scaled = t * (path.length - 1);
      const segmentIndex = Math.min(path.length - 2, Math.floor(scaled));
      fromSample = path[segmentIndex];
      toSample = path[segmentIndex + 1];
      segmentT = Math.min(1, Math.max(0, scaled - segmentIndex));
    }
  }

  const fromX = Number(fromSample.xM || 0);
  const fromY = Number(fromSample.yM || 0);
  const toX = Number(toSample.xM || 0);
  const toY = Number(toSample.yM || 0);
  const deltaX = toX - fromX;
  const deltaY = toY - fromY;
  const chordLength = Math.hypot(deltaX, deltaY);
  const fromHeading = Number(fromSample.headingRad || 0);
  const toHeading = Number(toSample.headingRad || 0);
  let xM = fromX + deltaX * segmentT;
  let yM = fromY + deltaY * segmentT;

  if (chordLength > 1e-6) {
    // Cubic Hermite interpolation follows the physical heading at both
    // telemetry frames. Linear x/y interpolation cuts every corner into a
    // visible chord, especially at higher speeds where frames are farther
    // apart in simulation time.
    const chordHeading = Math.atan2(deltaY, deltaX);
    const fromAlignment = Math.max(0, Math.cos(shortestAngleDelta(fromHeading, chordHeading)));
    const toAlignment = Math.max(0, Math.cos(shortestAngleDelta(toHeading, chordHeading)));
    const tangentLength = chordLength * (
      0.55 + 0.45 * Math.min(fromAlignment, toAlignment)
    );
    const fromTangentX = Math.cos(fromHeading) * tangentLength;
    const fromTangentY = Math.sin(fromHeading) * tangentLength;
    const toTangentX = Math.cos(toHeading) * tangentLength;
    const toTangentY = Math.sin(toHeading) * tangentLength;
    const t2 = segmentT * segmentT;
    const t3 = t2 * segmentT;
    const h00 = 2 * t3 - 3 * t2 + 1;
    const h10 = t3 - 2 * t2 + segmentT;
    const h01 = -2 * t3 + 3 * t2;
    const h11 = t3 - t2;
    xM = h00 * fromX + h10 * fromTangentX + h01 * toX + h11 * toTangentX;
    yM = h00 * fromY + h10 * fromTangentY + h01 * toY + h11 * toTangentY;
  }
  let headingRad = fromHeading + shortestAngleDelta(
      fromHeading,
      toHeading,
    ) * segmentT;
  const predictionElapsedMs = Math.min(
    Number(animation.maximumWorldPredictionMs || 0),
    Math.max(0, elapsed - animation.duration),
  );
  const predictionScale = Math.max(
    0,
    Number(animation.worldPredictionTimeScale || 0),
  );
  if (predictionElapsedMs > 0 && predictionScale > 0) {
    const predictionSeconds = predictionElapsedMs / 1000 * predictionScale;
    const velocityX = Number(animation.toWorldVelocityXMps || 0);
    const velocityY = Number(animation.toWorldVelocityYMps || 0);
    const accelerationX = Number(animation.toWorldAccelerationXMps2 || 0);
    const accelerationY = Number(animation.toWorldAccelerationYMps2 || 0);
    const predictionOriginX = Number(animation.toWorldXM || 0);
    const predictionOriginY = Number(animation.toWorldYM || 0);
    const predictionHeading = Number(animation.toWorldHeadingRad || 0);
    const predictedX = predictionOriginX
      + velocityX * predictionSeconds
      + 0.5 * accelerationX * predictionSeconds ** 2;
    const predictedY = predictionOriginY
      + velocityY * predictionSeconds
      + 0.5 * accelerationY * predictionSeconds ** 2;
    const predictionDistance = Math.hypot(
      predictedX - predictionOriginX,
      predictedY - predictionOriginY,
    );
    const maximumDistance = Math.max(
      0,
      Number(animation.maximumWorldPredictionDistanceM || 0),
    );
    const distanceScale = predictionDistance > maximumDistance && predictionDistance > 1e-9
      ? maximumDistance / predictionDistance
      : 1;
    xM = predictionOriginX + (predictedX - predictionOriginX) * distanceScale;
    yM = predictionOriginY + (predictedY - predictionOriginY) * distanceScale;
    headingRad = predictionHeading + Number(animation.toWorldYawRateRadS || 0)
      * predictionSeconds * distanceScale;
  }
  return { xM, yM, headingRad };
}

function worldPoseToRenderPoint(worldPose, coordinateFrame, displayRotation, displayCenter) {
  const metersPerRenderUnit = Math.max(
    1e-9,
    Number(coordinateFrame?.metersPerRenderUnit || 1),
  );
  return rotatePoint([
    Number(coordinateFrame?.originXRender || 0) + worldPose.xM / metersPerRenderUnit,
    Number(coordinateFrame?.originYRender || 0) + worldPose.yM / metersPerRenderUnit,
  ], displayRotation, displayCenter);
}

function markerPoseForWorld(
  worldPose,
  coordinateFrame,
  displayRotation,
  displayCenter,
  transform,
) {
  const metersPerRenderUnit = Math.max(
    1e-9,
    Number(coordinateFrame?.metersPerRenderUnit || 1),
  );
  const renderPoint = [
    Number(coordinateFrame?.originXRender || 0) + worldPose.xM / metersPerRenderUnit,
    Number(coordinateFrame?.originYRender || 0) + worldPose.yM / metersPerRenderUnit,
  ];
  const [rotatedX, rotatedY] = rotatePoint(
    renderPoint,
    displayRotation,
    displayCenter,
  );
  return {
    ...toCanvasPoint(rotatedX, rotatedY, transform),
    angle: worldPose.headingRad + Number(displayRotation || 0) * Math.PI / 180,
  };
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

function smoothPathPointAndAngle(
  coords,
  progress,
  transform,
  metrics,
  tangentSampleM = TRACK_BOUNDARY_TANGENT_SAMPLE_M,
) {
  const center = interpolatePath(coords, progress, transform, metrics);
  const progressDelta = Math.min(
    0.001,
    Math.max(1e-6, tangentSampleM / Math.max(1, metrics?.totalLength || 1)),
  );
  const before = interpolatePath(
    coords,
    progress - progressDelta,
    transform,
    metrics,
  );
  const after = interpolatePath(
    coords,
    progress + progressDelta,
    transform,
    metrics,
  );
  return {
    ...center,
    angle: Math.atan2(after.y - before.y, after.x - before.x),
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

function drawOpenProgressPath(
  graphics,
  coords,
  startProgress,
  endProgress,
  transform,
  metrics,
  strokeStyle,
) {
  if (!coords || coords.length < 2 || !metrics?.totalLength) return;
  const start = Math.min(1, Math.max(0, Number(startProgress)));
  const end = Math.min(1, Math.max(start, Number(endProgress)));
  const distance = (end - start) * metrics.totalLength;
  const steps = Math.max(2, Math.ceil(distance / 8));
  const first = interpolatePath(coords, start, transform, metrics, false);
  graphics.moveTo(first.x, first.y);
  for (let index = 1; index <= steps; index += 1) {
    const progress = start + ((end - start) * index) / steps;
    const point = interpolatePath(coords, progress, transform, metrics, false);
    graphics.lineTo(point.x, point.y);
  }
  graphics.stroke(strokeStyle);
}

function drawPitGate(
  graphics,
  coords,
  progress,
  transform,
  metrics,
  halfWidthPx,
  color,
  dotted = false,
  strokeWidth = 2.2,
  alpha = 1,
) {
  const pose = pathPointAndAngle(coords, progress, transform, metrics, false);
  const normalX = -Math.sin(pose.angle);
  const normalY = Math.cos(pose.angle);
  if (dotted) {
    const segmentCount = 4;
    const segmentLength = (halfWidthPx * 2) / (segmentCount * 2 - 1);
    for (let index = 0; index < segmentCount; index += 1) {
      const startDistance = halfWidthPx - index * segmentLength * 2;
      const endDistance = startDistance - segmentLength;
      graphics.moveTo(
        pose.x + normalX * startDistance,
        pose.y + normalY * startDistance,
      );
      graphics.lineTo(
        pose.x + normalX * endDistance,
        pose.y + normalY * endDistance,
      );
    }
  } else {
    graphics.moveTo(
      pose.x + normalX * halfWidthPx,
      pose.y + normalY * halfWidthPx,
    );
    graphics.lineTo(
      pose.x - normalX * halfWidthPx,
      pose.y - normalY * halfWidthPx,
    );
  }
  graphics.stroke({
    width: strokeWidth,
    color,
    alpha,
    cap: 'square',
  });
  return pose;
}

function drawOffsetProgressPath(
  graphics,
  coords,
  startProgress,
  endProgress,
  transform,
  metrics,
  offsetAtProgress,
  strokeStyle,
  sampleSpacingM = 10,
) {
  if (!coords || coords.length < 2) return;
  const drawSegment = (start, end) => {
    const distance = Math.max(1, (end - start) * (metrics?.totalLength || coords.length));
    const steps = Math.max(
      8,
      Math.ceil(distance / Math.max(0.25, sampleSpacingM)),
    );
    const points = [];
    for (let index = 0; index <= steps; index += 1) {
      const progress = start + ((end - start) * index) / steps;
      const pose = smoothPathPointAndAngle(
        coords,
        progress,
        transform,
        metrics,
      );
      points.push(offsetCanvasPoint(pose, offsetAtProgress(progress % 1)));
    }
    drawPolylinePath(graphics, points, strokeStyle);
  };
  if (startProgress <= endProgress) {
    drawSegment(startProgress, endProgress);
  } else {
    drawSegment(startProgress, 1);
    drawSegment(0, endProgress);
  }
}

function buildTrackWidthSampler(trackWidthProfile, fallbackWidthM) {
  const profile = (trackWidthProfile || [])
    .map(([progress, leftWidthM, rightWidthM]) => ({
      progress: ((Number(progress) % 1) + 1) % 1,
      left: Number(leftWidthM || fallbackWidthM / 2),
      right: Number(rightWidthM || fallbackWidthM / 2),
    }))
    .sort((a, b) => a.progress - b.progress);
  if (!profile.length) {
    return () => ({ left: fallbackWidthM / 2, right: fallbackWidthM / 2 });
  }
  return (progress) => {
    const normalized = ((progress % 1) + 1) % 1;
    let index = profile.length - 1;
    for (let candidate = 0; candidate < profile.length; candidate += 1) {
      if (profile[candidate].progress > normalized) break;
      index = candidate;
    }
    const nextIndex = (index + 1) % profile.length;
    const start = profile[index];
    const end = profile[nextIndex];
    const endProgress = nextIndex === 0 ? end.progress + 1 : end.progress;
    const target = normalized < start.progress ? normalized + 1 : normalized;
    const ratio = Math.min(1, Math.max(
      0,
      (target - start.progress) / Math.max(1e-9, endProgress - start.progress),
    ));
    return {
      left: start.left + (end.left - start.left) * ratio,
      right: start.right + (end.right - start.right) * ratio,
    };
  };
}

function drawVariableWidthTrackSurface(
  graphics,
  coords,
  transform,
  metrics,
  trackLengthM,
  trackWidthProfile,
  fallbackWidthM,
  widthAdditionM,
  color,
  alpha = 1,
) {
  if (!coords?.length || !metrics?.totalLength) return;
  const pixelsPerMeter = (metrics.totalLength / Math.max(1, trackLengthM)) * transform.scale;
  const widthAt = buildTrackWidthSampler(trackWidthProfile, fallbackWidthM);
  const steps = Math.max(256, Math.min(4000, Math.ceil(trackLengthM / 2)));
  const leftPoints = [];
  const rightPoints = [];
  for (let index = 0; index <= steps; index += 1) {
    const progress = index / steps;
    const pose = smoothPathPointAndAngle(coords, progress, transform, metrics);
    const widths = widthAt(progress % 1);
    leftPoints.push(offsetCanvasPoint(
      pose,
      (widths.left + widthAdditionM) * pixelsPerMeter,
    ));
    rightPoints.push(offsetCanvasPoint(
      pose,
      -(widths.right + widthAdditionM) * pixelsPerMeter,
    ));
  }
  graphics.moveTo(leftPoints[0].x, leftPoints[0].y);
  for (let index = 1; index < leftPoints.length; index += 1) {
    graphics.lineTo(leftPoints[index].x, leftPoints[index].y);
  }
  for (let index = rightPoints.length - 1; index >= 0; index -= 1) {
    graphics.lineTo(rightPoints[index].x, rightPoints[index].y);
  }
  graphics.closePath();
  graphics.fill({ color, alpha });
}

function drawSurfaceKerbs(
  graphics,
  coords,
  transform,
  metrics,
  trackLengthM,
  trackWidthM,
  trackWidthProfile,
  surfaceZones,
  physicalScale,
) {
  if (!surfaceZones?.length) return;
  const pixelsPerMeter = (
    (metrics?.totalLength || 0) / Math.max(1, trackLengthM)
  ) * transform.scale;
  const widthAt = buildTrackWidthSampler(trackWidthProfile, trackWidthM);
  if (physicalScale) {
    surfaceZones.forEach((zone) => {
      const sideSign = zone.side === 'right' ? -1 : 1;
      const kerbWidthM = Math.max(0, Number(zone.kerb_width_m || 0));
      const runoffWidthM = Math.max(0, Number(zone.runoff_width_m || 0));
      if (runoffWidthM <= 0) return;
      const runoffColors = {
        asphalt_runoff: 0x55555d,
        grass: 0x214d25,
        gravel: 0xa98f68,
      };
      drawOffsetProgressPath(
        graphics,
        coords,
        Number(zone.start || 0),
        Number(zone.end || 0),
        transform,
        metrics,
        (progress) => {
          const widths = widthAt(progress);
          const edgeWidthM = sideSign > 0 ? widths.left : widths.right;
          return sideSign
            * (edgeWidthM + kerbWidthM + runoffWidthM / 2)
            * pixelsPerMeter;
        },
        {
          width: Math.max(1.5, runoffWidthM * pixelsPerMeter),
          color: runoffColors[zone.runoff_surface] ?? runoffColors.asphalt_runoff,
          alpha: 0.96,
          cap: 'butt',
          join: 'round',
        },
      );
    });
  }
  surfaceZones.forEach((zone) => {
    const sideSign = zone.side === 'right' ? -1 : 1;
    const kerbWidthM = Math.max(0.1, Number(zone.kerb_width_m || 1.2));
    const kerbWidthPx = physicalScale
      ? Math.max(1.5, kerbWidthM * pixelsPerMeter)
      : 5;
    const offsetAtProgress = (progress) => {
      const widths = widthAt(progress);
      const edgeWidthM = sideSign > 0 ? widths.left : widths.right;
      return sideSign * (edgeWidthM + kerbWidthM / 2) * pixelsPerMeter;
    };
    const start = Number(zone.start || 0);
    const end = Number(zone.end || 0);
    const range = progressRangeLength(start, end);
    const stripeCount = Math.max(2, Math.min(80, Math.ceil(
      range * trackLengthM / 8,
    )));
    for (let stripe = 0; stripe < stripeCount; stripe += 1) {
      const stripeStart = (start + range * stripe / stripeCount) % 1;
      const stripeEnd = (start + range * (stripe + 1) / stripeCount) % 1;
      drawOffsetProgressPath(
        graphics,
        coords,
        stripeStart,
        stripeEnd,
        transform,
        metrics,
        offsetAtProgress,
        {
          width: kerbWidthPx,
          color: stripe % 2 === 0 ? 0xf4f4f4 : 0xd70b12,
          alpha: zone.kerb_height === 'high' ? 1 : 0.92,
          cap: 'butt',
          join: 'round',
        },
      );
    }
  });
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

function drawStartingGrid(
  graphics,
  coords,
  metrics,
  transform,
  gridSlots,
  trackLengthM,
  carWidthM,
  carLengthM,
) {
  if (!gridSlots?.length || !metrics?.totalLength) return;
  const pixelsPerMeter = (metrics.totalLength / Math.max(1, trackLengthM)) * transform.scale;
  const halfLength = Math.max(0.55, (carLengthM + 1.4) * pixelsPerMeter * 0.5);
  const halfWidth = Math.max(0.35, (carWidthM + 0.9) * pixelsPerMeter * 0.5);

  gridSlots.forEach((slot) => {
    const pose = pathPointAndAngle(
      coords,
      Number(slot.progress || 0),
      transform,
      metrics,
    );
    const center = offsetCanvasPoint(
      pose,
      Number(slot.lateral_offset_m || 0) * pixelsPerMeter,
    );
    const forwardX = Math.cos(pose.angle);
    const forwardY = Math.sin(pose.angle);
    const lateralX = -forwardY;
    const lateralY = forwardX;
    const corners = [
      [
        center.x + forwardX * halfLength + lateralX * halfWidth,
        center.y + forwardY * halfLength + lateralY * halfWidth,
      ],
      [
        center.x + forwardX * halfLength - lateralX * halfWidth,
        center.y + forwardY * halfLength - lateralY * halfWidth,
      ],
      [
        center.x - forwardX * halfLength - lateralX * halfWidth,
        center.y - forwardY * halfLength - lateralY * halfWidth,
      ],
      [
        center.x - forwardX * halfLength + lateralX * halfWidth,
        center.y - forwardY * halfLength + lateralY * halfWidth,
      ],
    ];
    graphics.moveTo(corners[0][0], corners[0][1]);
    corners.slice(1).forEach(([x, y]) => graphics.lineTo(x, y));
    graphics.lineTo(corners[0][0], corners[0][1]);
    graphics.stroke({
      width: Math.max(0.55, 0.12 * pixelsPerMeter),
      color: 0xf7f7f7,
      alpha: 0.72,
      cap: 'square',
      join: 'miter',
    });
  });
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
  pitLaneWidthM,
  pitSideEntryProgress,
  pitSpeedLimitStart,
  pitBoxProgress,
  pitSpeedLimitEnd,
  pitSideRejoinProgress,
  drsZones,
  sectors,
  sfIndex,
  landmarks,
  transform,
  trackMetrics,
  pitMetrics,
  viewport,
  trackLengthM,
  trackWidthM,
  racingLineProfile,
  trackWidthProfile,
  surfaceZones,
  racingLineCoords,
  gridSlots,
  carWidthM,
  carLengthM,
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

  if (physicalScale) {
    drawVariableWidthTrackSurface(
      trackGfx, activeRouteCoords, transform, trackMetrics, trackLengthM,
      trackWidthProfile, trackWidthM, 5.2, 0x18351b, 0.9,
    );
    drawVariableWidthTrackSurface(
      trackGfx, activeRouteCoords, transform, trackMetrics, trackLengthM,
      trackWidthProfile, trackWidthM, 3.0, 0x4a4a50, 0.95,
    );
    drawVariableWidthTrackSurface(
      trackGfx, activeRouteCoords, transform, trackMetrics, trackLengthM,
      trackWidthProfile, trackWidthM, 1.5, 0x07070b, 0.95,
    );
    drawVariableWidthTrackSurface(
      trackGfx, activeRouteCoords, transform, trackMetrics, trackLengthM,
      trackWidthProfile, trackWidthM, 0, 0x2a2a38, 1,
    );
  } else {
    drawPath(trackGfx, activeRouteCoords, transform, {
      width: surfaceWidth + 6,
      color: 0x07070b,
      alpha: 0.95,
      cap: 'round',
      join: 'round',
    });
    drawPath(trackGfx, activeRouteCoords, transform, {
      width: surfaceWidth,
      color: 0x1e1e2e,
      alpha: 0.95,
      cap: 'round',
      join: 'round',
    });
  }

  drawSurfaceKerbs(
    trackGfx,
    activeRouteCoords,
    transform,
    trackMetrics,
    trackLengthM,
    trackWidthM,
    trackWidthProfile,
    surfaceZones,
    physicalScale,
  );

  if (!physicalScale) {
    drawPath(trackGfx, activeRouteCoords, transform, {
      width: 10,
      color: 0x2a2a38,
      alpha: 1,
      cap: 'round',
      join: 'round',
    });
  }

  if (physicalScale) {
    const widthAt = buildTrackWidthSampler(trackWidthProfile, trackWidthM);
    [-1, 1].forEach((sideSign) => {
      drawOffsetProgressPath(
        trackGfx,
        activeRouteCoords,
        0,
        1,
        transform,
        trackMetrics,
        (progress) => {
          const widths = widthAt(progress);
          return sideSign * (sideSign > 0 ? widths.left : widths.right) * pixelsPerMeter;
        },
        {
          width: Math.max(1, 0.15 * pixelsPerMeter),
          color: 0xf4f4f4,
          alpha: 0.9,
          cap: 'round',
          join: 'round',
        },
        TRACK_BOUNDARY_SAMPLE_SPACING_M,
      );
    });
  }

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
    if (racingLineCoords?.length >= 2) {
      drawPath(trackGfx, racingLineCoords, transform, {
        width: 1.15,
        color: 0xb2b2c0,
        alpha: 0.78,
        cap: 'round',
        join: 'round',
      });
    } else {
      drawMetricRacingLine(
        trackGfx,
        activeRouteCoords,
        transform,
        trackMetrics,
        trackLengthM,
        racingLineProfile,
      );
    }
  }

  (sectors || []).slice(0, 3).forEach((sector, sectorIndex) => {
    const start = Number(sector.start);
    const end = Number(sector.end);
    if (!Number.isFinite(start) || !Number.isFinite(end) || end <= start) return;
    const miniCount = Math.max(1, Number(sector.mini_sector_count) || 6);
    for (let miniIndex = 1; miniIndex < miniCount; miniIndex += 1) {
      const progress = start + ((end - start) * miniIndex) / miniCount;
      drawPitGate(
        trackGfx,
        activeRouteCoords,
        progress,
        transform,
        trackMetrics,
        surfaceWidth / 2,
        0x7f8c9a,
        true,
        physicalScale ? 0.8 : 0.65,
        physicalScale ? 0.32 : 0.2,
      );
    }
    if (sectorIndex < 2) {
      const pose = drawPitGate(
        trackGfx,
        activeRouteCoords,
        end,
        transform,
        trackMetrics,
        surfaceWidth / 2,
        0x00d8ff,
        false,
        physicalScale ? 1.6 : 1.2,
        0.9,
      );
      addPlacedLabel(
        markersContainer,
        placedLabels,
        `S${sectorIndex + 1}`,
        pose,
        0x00d8ff,
        [[8, -26], [8, 10], [-30, -26], [-30, 10]],
        viewport,
      );
    }
  });

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
    const pitWidthPx = physicalScale
      ? Math.max(3.5, pitLaneWidthM * pixelsPerMeter)
      : 10;
    const pitEdgeWidthPx = Math.max(1, 0.15 * pixelsPerMeter);
    const pitLimitStart = Math.min(0.95, Math.max(0.02, pitSpeedLimitStart));
    const pitSideEntry = Math.min(
      pitLimitStart - 0.01,
      Math.max(0.005, pitSideEntryProgress),
    );
    const pitBox = Math.min(
      pitSpeedLimitEnd - 0.02,
      Math.max(pitLimitStart + 0.02, pitBoxProgress),
    );
    const pitLimitEnd = Math.min(0.98, Math.max(pitBox + 0.02, pitSpeedLimitEnd));
    const pitSideRejoin = Math.min(
      0.995,
      Math.max(pitLimitEnd + 0.02, pitSideRejoinProgress),
    );

    drawOpenProgressPath(trackGfx, activePitRouteCoords, pitSideEntry, pitSideRejoin, transform, pitMetrics, {
      width: pitWidthPx,
      color: 0x1e1e2e,
      alpha: 1,
      cap: 'butt',
      join: 'round',
    });

    drawOpenProgressPath(
      trackGfx,
      activePitRouteCoords,
      pitLimitStart,
      pitLimitEnd,
      transform,
      pitMetrics,
      {
        width: Math.max(2.4, pitWidthPx - 1.8),
        color: 0x4a3510,
        alpha: 0.55,
        cap: 'butt',
        join: 'round',
      },
    );

    [-1, 1].forEach((sideSign) => {
      const edgePoints = [];
      const edgeStart = Math.min(pitLimitStart - 0.005, pitSideEntry + 0.018);
      const edgeEnd = Math.max(pitLimitEnd + 0.005, pitSideRejoin - 0.015);
      const edgeSteps = Math.max(
        32,
        Math.ceil(((pitMetrics?.totalLength || 1) * (edgeEnd - edgeStart)) / 8),
      );
      for (let index = 0; index <= edgeSteps; index += 1) {
        const progress = edgeStart + ((index / edgeSteps) * (edgeEnd - edgeStart));
        const pose = pathPointAndAngle(
          activePitRouteCoords,
          progress,
          transform,
          pitMetrics,
          false,
        );
        edgePoints.push(offsetCanvasPoint(pose, sideSign * pitWidthPx / 2));
      }
      drawPolylinePath(trackGfx, edgePoints, {
        width: pitEdgeWidthPx,
        color: 0xf2f2f2,
        alpha: 0.88,
        cap: 'round',
        join: 'round',
      });
    });

    drawOpenProgressPath(
      trackGfx,
      activePitRouteCoords,
      pitSideEntry,
      pitLimitStart,
      transform,
      pitMetrics,
      {
        width: 1.2,
        color: 0x7f8794,
        alpha: 0.72,
        cap: 'round',
        join: 'round',
      },
    );
    drawOpenProgressPath(
      trackGfx,
      activePitRouteCoords,
      pitLimitEnd,
      pitSideRejoin,
      transform,
      pitMetrics,
      {
        width: 1.2,
        color: 0x2ecc71,
        alpha: 0.78,
        cap: 'round',
        join: 'round',
      },
    );

    const entry = pathPointAndAngle(
      activePitRouteCoords,
      pitSideEntry,
      transform,
      pitMetrics,
      false,
    );
    const exitRoad = pathPointAndAngle(
      activePitRouteCoords,
      (pitLimitEnd + pitSideRejoin) / 2,
      transform,
      pitMetrics,
      false,
    );
    const speedLimitEntry = drawPitGate(
      trackGfx,
      activePitRouteCoords,
      pitLimitStart,
      transform,
      pitMetrics,
      pitWidthPx * 0.62,
      0xffd60a,
    );
    const speedLimitExit = drawPitGate(
      trackGfx,
      activePitRouteCoords,
      pitLimitEnd,
      transform,
      pitMetrics,
      pitWidthPx * 0.62,
      0x2ecc71,
    );
    const sideRejoin = drawPitGate(
      trackGfx,
      activePitRouteCoords,
      pitSideRejoin,
      transform,
      pitMetrics,
      pitWidthPx * 0.62,
      0xf2f2f2,
      true,
    );
    const boxPoint = pathPointAndAngle(
      activePitRouteCoords,
      pitBox,
      transform,
      pitMetrics,
      false,
    );

    addPlacedLabel(markersContainer, placedLabels, 'PIT ENTRY', entry, 0xe8e8ee, [
      [-48, 14],
      [-52, -26],
      [10, 14],
      [-48, 30],
      [10, -26],
    ], viewport);
    addPlacedLabel(markersContainer, placedLabels, '60 LIMIT', speedLimitEntry, 0xffd60a, [
      [-62, -28],
      [12, -28],
      [-62, 14],
      [12, 14],
    ], viewport);
    addPlacedLabel(markersContainer, placedLabels, 'BOX', boxPoint, 0xf39c12, [
      [-22, -28],
      [12, -28],
      [-22, 14],
      [12, 14],
    ], viewport);
    addPlacedLabel(markersContainer, placedLabels, '60 END', speedLimitExit, 0x2ecc71, [
      [12, -28],
      [-58, -28],
      [12, 14],
      [-58, 14],
    ], viewport);
    addPlacedLabel(markersContainer, placedLabels, 'EXIT ROAD', exitRoad, 0x2ecc71, [
      [10, 14],
      [-66, 14],
      [10, -26],
      [-66, -26],
    ], viewport);
    addPlacedLabel(markersContainer, placedLabels, 'SIDE REJOIN', sideRejoin, 0xe8e8ee, [
      [10, 14],
      [-82, 14],
      [10, -26],
      [-82, -26],
      [10, 30],
    ], viewport);

    for (let i = 0; i < 10; i += 1) {
      const slotPose = pathPointAndAngle(
        activePitRouteCoords,
        Math.max(
          pitLimitStart + 0.04,
          pitBox - 0.25 + i * 0.05,
        ),
        transform,
        pitMetrics,
        false,
      );
      const slot = offsetCanvasPoint(slotPose, pitBoxOffset);
      trackGfx.roundRect(slot.x - 7, slot.y - 7, 14, 10, 2);
      trackGfx.fill({ color: 0x111118, alpha: 0.95 });
      trackGfx.stroke({ width: 1, color: 0xf39c12, alpha: 0.42 });
    }

  }

  drawStartingGrid(
    trackGfx,
    activeRouteCoords,
    trackMetrics,
    transform,
    gridSlots,
    trackLengthM,
    carWidthM,
    carLengthM,
  );
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
  if (driver.pit_merge_state === 'yield') return 'PIT EXIT YIELD';
  if (driver.pit_merge_state === 'hold') return 'PIT EXIT HOLD';
  if (driver.pit_merge_state === 'merge') return 'PIT EXIT MERGE';
  const pitElapsed = (driver.pit_elapsed || 0).toFixed(1);
  if (driver.pit_phase === 'stop') {
    const stopElapsed = (driver.pit_stop_elapsed || 0).toFixed(1);
    return `BOX ${stopElapsed}s / PIT ${pitElapsed}s`;
  }
  if (driver.pit_phase === 'in') return `PIT IN ${pitElapsed}s`;
  if (driver.pit_phase === 'out') return `PIT OUT ${pitElapsed}s`;
  return `PIT ${pitElapsed}s`;
}

function formatManeuverStatus(driver) {
  if (driver.maneuver_group_size >= 3) {
    const corridor = Number.isInteger(driver.maneuver_group_corridor_index)
      ? ` C${driver.maneuver_group_corridor_index + 1}`
      : '';
    return `${driver.maneuver_group_size}W${corridor}`;
  }
  if (!driver.maneuver_active || !driver.maneuver_phase) return '';
  const role = driver.maneuver_role === 'attacker' ? 'ATK' : 'DEF';
  const corridor = driver.maneuver_corner_active && driver.maneuver_corridor
    ? ` ${String(driver.maneuver_corridor).toUpperCase()}`
    : '';
  return `${role} ${String(driver.maneuver_phase).replace('_', ' ').toUpperCase()}${corridor}`;
}

function formatTrackStatus(driver) {
  if (driver.track_limits_active) return 'TRACK LIMITS';
  if (driver.off_track_cause === 'forced_wide') return 'FORCED WIDE';
  if (driver.off_track) {
    const wheelsOutside = (driver.wheel_surfaces || []).filter((surface) => (
      !['track', 'kerb_low', 'kerb_high'].includes(String(surface))
    )).length;
    return `${Math.max(1, wheelsOutside)}W OFF`;
  }
  if (driver.handling_state === 'run_wide') return 'RUN WIDE';
  return '';
}

function formatContactStatus(driver) {
  if (!driver.contact_active) return '';
  const impact = Number(driver.contact_impact_speed_mps || 0).toFixed(1);
  return `${driver.contact_type === 'side' ? 'SIDE' : 'CONTACT'} ${impact}m/s`;
}

function formatHazardStatus(driver) {
  if (!driver.hazard_active) return '';
  return driver.hazard_cause === 'mechanical' ? 'STOPPED MECHANICAL' : 'STOPPED';
}

function formatAvoidanceStatus(driver) {
  if (!driver.avoidance_active) return '';
  if (driver.emergency_braking || driver.avoidance_side === 'blocked') {
    return 'BRAKE — HAZARD';
  }
  const side = String(driver.avoidance_side || '').toUpperCase();
  const ttc = Number(driver.avoidance_ttc_seconds || 0).toFixed(1);
  return `AVOID ${side} ${ttc}s`;
}

function formatLocalYellowStatus(driver) {
  return driver.local_yellow_active ? 'LOCAL YELLOW' : '';
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
  worldCoordinateFrame = null,
  trackWidthM = 12,
  carWidthM = 1.9,
  carLengthM = 5.0,
  gridSlots = [],
  racingLineProfile = [],
  trackWidthProfile = [],
  surfaceZones = [],
  racingLineCoords = [],
  pitLaneCoords,
  pitBoxOffset = 11,
  pitLaneWidthM = 4,
  pitSideEntryProgress = 0.02,
  pitSpeedLimitStart = 0.12,
  pitBoxProgress = 0.5,
  pitSpeedLimitEnd = 0.88,
  pitSideRejoinProgress = 0.94,
  drsZones = [],
  sectors = [],
  startFinishIndex = 0,
  landmarks = [],
  viewBounds = null,
  displayRotationDeg = 0,
  showBearing = false,
  positions,
  poseTickRef = null,
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
  const poseBuffersRef = useRef(new Map());
  const posePlaybackRef = useRef({
    lastTickPhysicsFrame: -1,
    latestSimulationTimeS: 0,
    latestReceivedAtMs: 0,
    speedMultiplier: 1,
    paused: false,
  });
  const miniMapCircleRefs = useRef(new Map());
  const transformRef = useRef({ scale: 1, offsetX: 0, offsetY: 0 });
  const sourceTrackCoords = useMemo(() => normalizeCoordinatePath(trackCoords), [trackCoords]);
  const sourcePitLaneCoords = useMemo(() => normalizeCoordinatePath(pitLaneCoords), [pitLaneCoords]);
  const sourceRacingLineCoords = useMemo(
    () => normalizeCoordinatePath(racingLineCoords),
    [racingLineCoords],
  );
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
  const normalizedRacingLineCoords = useMemo(
    () => rotateCoordinatePath(sourceRacingLineCoords, displayRotation, displayCenter),
    [sourceRacingLineCoords, displayRotation, displayCenter],
  );
  // Physics evaluates the same closed Catmull-Rom spans between canonical
  // centerline controls. Dense rendering only tessellates those shared spans.
  const smoothedTrackCoords = useMemo(
    () => sampleCatmullRomClosed(
      closedPathControls(normalizedTrackCoords),
      TRACK_RENDER_SAMPLE_SPACING,
    ),
    [normalizedTrackCoords],
  );
  // Pit physics integrates the supplied polyline by exact segment length.
  // Keep that same polyline for graphics instead of changing its arc length
  // with a second Catmull-Rom curve.
  const smoothedPitLaneCoords = normalizedPitLaneCoords;
  const smoothedRacingLineCoords = useMemo(
    () => sampleCatmullRomClosed(
      closedPathControls(normalizedRacingLineCoords),
      TRACK_RENDER_SAMPLE_SPACING,
    ),
    [normalizedRacingLineCoords],
  );
  const trackMetrics = useMemo(() => buildPathMetrics(smoothedTrackCoords), [smoothedTrackCoords]);
  const pitMetrics = useMemo(() => buildPathMetrics(smoothedPitLaneCoords), [smoothedPitLaneCoords]);
  const metaRef = useRef({
    trackCoords: normalizedTrackCoords,
    smoothedTrackCoords,
    pitLaneCoords: normalizedPitLaneCoords,
    smoothedPitLaneCoords,
    pitBoxOffset,
    pitLaneWidthM,
    pitSideEntryProgress,
    pitSpeedLimitStart,
    pitBoxProgress,
    pitSpeedLimitEnd,
    pitSideRejoinProgress,
    drsZones,
    sectors,
    startFinishIndex,
    landmarks,
    viewBounds,
    trackMetrics,
    pitMetrics,
    trackLengthM,
    trackWidthM,
    racingLineProfile,
    trackWidthProfile,
    surfaceZones,
    racingLineCoords: smoothedRacingLineCoords,
    gridSlots,
    carWidthM,
    carLengthM,
    worldCoordinateFrame,
    displayRotation,
    displayCenter,
  });
  const [appReady, setAppReady] = useState(false);
  const [zoomPercent, setZoomPercent] = useState(DEFAULT_ZOOM_PERCENT);
  const [panOffset, setPanOffset] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [followDriverId, setFollowDriverId] = useState(null);
  const zoomRef = useRef(DEFAULT_ZOOM_PERCENT);
  const panRef = useRef({ x: 0, y: 0 });
  const followDriverIdRef = useRef(null);
  const pausedRef = useRef(false);
  const lastPausedStateRef = useRef(paused);
  const dragRef = useRef(null);
  const lastViewKeyRef = useRef('');
  const initialFollowAppliedRef = useRef(false);
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
  const miniMap = useMemo(() => {
    const bounds = computeRenderBounds(
      [normalizedTrackCoords, normalizedPitLaneCoords],
      viewBounds,
    );
    if (!Number.isFinite(bounds.width) || bounds.width <= 0 || bounds.height <= 0) {
      return null;
    }
    const padding = Math.max(bounds.width, bounds.height) * 0.045;
    const driverPoints = (positions || []).flatMap((driver) => {
      if (driver.retired && !driver.hazard_active) return [];
      const xM = Number(driver.world_x_m);
      const yM = Number(driver.world_y_m);
      if (!worldCoordinateFrame || !Number.isFinite(xM) || !Number.isFinite(yM)) return [];
      const metersPerRenderUnit = Math.max(
        1e-9,
        Number(worldCoordinateFrame.metersPerRenderUnit || 1),
      );
      const renderPoint = [
        Number(worldCoordinateFrame.originXRender || 0) + xM / metersPerRenderUnit,
        Number(worldCoordinateFrame.originYRender || 0) + yM / metersPerRenderUnit,
      ];
      const [x, y] = rotatePoint(renderPoint, displayRotation, displayCenter);
      return [{ ...driver, miniX: x, miniY: y }];
    });
    const sectorColors = ['#00d8ff', '#f5a623', '#b26bff'];
    const sectorLines = (sectors || []).slice(0, 3).flatMap((sector, index) => {
      const start = Number(sector.start);
      const end = Number(sector.end);
      if (!Number.isFinite(start) || !Number.isFinite(end) || end <= start) return [];
      const lastIndex = Math.max(1, normalizedTrackCoords.length - 1);
      const startIndex = Math.max(0, Math.floor(start * lastIndex));
      const endIndex = Math.min(lastIndex, Math.ceil(end * lastIndex));
      const points = normalizedTrackCoords
        .slice(startIndex, endIndex + 1)
        .map(([x, y]) => `${x},${y}`)
        .join(' ');
      const labelPoint = normalizedTrackCoords[
        Math.min(lastIndex, Math.round(((start + end) / 2) * lastIndex))
      ];
      return [{
        key: sector.name || `sector-${index + 1}`,
        label: `S${index + 1}`,
        points,
        color: sectorColors[index],
        labelX: labelPoint?.[0],
        labelY: labelPoint?.[1],
      }];
    });
    return {
      viewBox: `${bounds.minX - padding} ${bounds.minY - padding} ${bounds.width + padding * 2} ${bounds.height + padding * 2}`,
      trackPoints: normalizedTrackCoords.map(([x, y]) => `${x},${y}`).join(' '),
      pitPoints: normalizedPitLaneCoords.map(([x, y]) => `${x},${y}`).join(' '),
      sectorLines,
      driverPoints,
      markerRadius: Math.max(bounds.width, bounds.height) * 0.009,
      trackStroke: Math.max(bounds.width, bounds.height) * 0.012,
    };
  }, [
    displayCenter,
    displayRotation,
    normalizedPitLaneCoords,
    normalizedTrackCoords,
    positions,
    sectors,
    viewBounds,
    worldCoordinateFrame,
  ]);

  metaRef.current = {
    trackCoords: normalizedTrackCoords,
    smoothedTrackCoords,
    pitLaneCoords: normalizedPitLaneCoords,
    smoothedPitLaneCoords,
    pitBoxOffset,
    pitLaneWidthM,
    pitSideEntryProgress,
    pitSpeedLimitStart,
    pitBoxProgress,
    pitSpeedLimitEnd,
    pitSideRejoinProgress,
    drsZones,
    sectors,
    startFinishIndex,
    landmarks,
    viewBounds,
    trackMetrics,
    pitMetrics,
    trackLengthM,
    trackWidthM,
    racingLineProfile,
    trackWidthProfile,
    surfaceZones,
    racingLineCoords: smoothedRacingLineCoords,
    gridSlots,
    carWidthM,
    carLengthM,
    worldCoordinateFrame,
    displayRotation,
    displayCenter,
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
      pitLaneWidthM: laneWidthM,
      pitSideEntryProgress: sideEntryProgress,
      pitSpeedLimitStart: limitStart,
      pitBoxProgress: boxProgress,
      pitSpeedLimitEnd: limitEnd,
      pitSideRejoinProgress: sideRejoinProgress,
      drsZones: drs,
      sectors: timingSectors,
      startFinishIndex: sf,
      landmarks: lms,
      viewBounds: fixedViewBounds,
      trackMetrics: mainMetrics,
      pitMetrics: laneMetrics,
      racingLineCoords: lineCoords,
      gridSlots: startingGrid,
      carWidthM: physicalCarWidthM,
      carLengthM: physicalCarLengthM,
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
      laneWidthM,
      sideEntryProgress,
      limitStart,
      boxProgress,
      limitEnd,
      sideRejoinProgress,
      drs,
      timingSectors,
      sf,
      lms,
      transform,
      mainMetrics,
      laneMetrics,
      { width: app.screen.width, height: app.screen.height },
      trackLengthM,
      trackWidthM,
      racingLineProfile,
      trackWidthProfile,
      surfaceZones,
      lineCoords,
      startingGrid,
      physicalCarWidthM,
      physicalCarLengthM,
      zoomRef.current >= PHYSICAL_ZOOM_MIN_PERCENT,
    );
    applyCameraPan();
    return transform;
  }, [
    applyCameraPan,
    racingLineProfile,
    surfaceZones,
    trackLengthM,
    trackWidthM,
    trackWidthProfile,
  ]);

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
    setFollowDriverId(playerDrivers[0]?.driver_id ?? null);
    panRef.current = { x: 0, y: 0 };
    setZoomPercent(DEFAULT_ZOOM_PERCENT);
    setPanOffset({ x: 0, y: 0 });
  }, [playerDrivers]);

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
    const poseBuffers = poseBuffersRef.current;
    const miniMapCircles = miniMapCircleRefs.current;

    app.init({
      background: 0x0a0a0f,
      resizeTo: containerRef.current,
      antialias: true,
      autoDensity: true,
      resolution: Math.min(window.devicePixelRatio || 1, 2),
    }).then(() => {
      if (destroyed) {
        app.destroy(true, { children: true });
        return;
      }

      containerRef.current?.appendChild(app.canvas);
      appRef.current = app;
      app.ticker.maxFPS = 60;

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
          worldCoordinateFrame: coordinateFrame,
          displayRotation: routeDisplayRotation,
          displayCenter: routeDisplayCenter,
        } = metaRef.current;
        const transform = transformRef.current;
        let followedPose = null;
        const liveTick = poseTickRef?.current;
        const playback = posePlaybackRef.current;
        const livePhysicsFrame = Number(liveTick?.physics_frame);
        if (
          Number.isFinite(livePhysicsFrame)
          && livePhysicsFrame !== playback.lastTickPhysicsFrame
        ) {
          if (livePhysicsFrame < playback.lastTickPhysicsFrame) {
            poseBuffers.clear();
          }
          const latestSimulationTimeS = appendPoseTickToBuffers(
            liveTick,
            poseBuffers,
          );
          playback.lastTickPhysicsFrame = livePhysicsFrame;
          if (latestSimulationTimeS > 0) {
            playback.latestSimulationTimeS = latestSimulationTimeS;
            playback.latestReceivedAtMs = now;
          }
          playback.speedMultiplier = Math.max(
            1,
            Number(liveTick?.speed_multiplier || 1),
          );
          playback.paused = Boolean(liveTick?.paused);
        }
        const elapsedSincePacketS = playback.paused
          ? 0
          : Math.max(0, now - playback.latestReceivedAtMs) / 1000;
        const estimatedLatestSimulationTimeS = playback.latestSimulationTimeS
          + elapsedSincePacketS * playback.speedMultiplier;
        const renderSimulationTimeS = estimatedLatestSimulationTimeS
          - (POSE_PLAYBACK_DELAY_MS / 1000) * playback.speedMultiplier;

        dots.forEach((marker, driverId) => {
          const animation = marker.routeAnimation;
          if (!animation || !routeCoords?.length) return;

          const bufferedPose = bufferedWorldPoseAtTime(
            poseBuffers.get(Number(driverId))?.samples,
            renderSimulationTimeS,
          );
          const worldPose = bufferedPose || worldPoseAtTime(animation, now);
          const targetPose = worldPose
            ? markerPoseForWorld(
              worldPose,
              coordinateFrame,
              routeDisplayRotation,
              routeDisplayCenter,
              transform,
            )
            : markerPoseForRoute(
              animation.routeType,
              progressAtTime(animation, now),
              transform,
              routeCoords,
              mainMetrics,
              pitRoute,
              laneMetrics,
              lateralOffsetAtTime(animation, now),
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
          if (worldPose && coordinateFrame) {
            const miniMapCircle = miniMapCircles.get(Number(driverId));
            if (miniMapCircle) {
              const [miniX, miniY] = worldPoseToRenderPoint(
                worldPose,
                coordinateFrame,
                routeDisplayRotation,
                routeDisplayCenter,
              );
              miniMapCircle.setAttribute('cx', String(miniX));
              miniMapCircle.setAttribute('cy', String(miniY));
            }
          }
          if (driverId === followDriverIdRef.current) {
            followedPose = {
              x: x + Number(worldRef.current?.x || 0),
              y: y + Number(worldRef.current?.y || 0),
            };
          }
          const handlingAngle = worldPose
            ? targetPose.angle
            : targetPose.angle + Number(animation.slipAngleRad || 0);
          if (Number.isFinite(handlingAngle)) {
            if (transition) {
              marker.dot.rotation = transition.fromAngle + shortestAngleDelta(
                transition.fromAngle,
                handlingAngle,
              ) * transitionBlend;
            } else {
              // Position and heading are sampled from the same buffered pose.
              // A second fixed rotation filter made the body lag behind its
              // trajectory, especially at 2x, and looked like rear slip.
              marker.dot.rotation = handlingAngle;
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
          const deadzonePx = physicalScale
            ? PHYSICAL_FOLLOW_DEADZONE_PX
            : FOLLOW_DEADZONE_PX;
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
          const followStrength = physicalScale ? PHYSICAL_FOLLOW_STRENGTH : 4;
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
      poseBuffers.clear();
      miniMapCircles.clear();
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
  }, [applyCameraPan, applyViewport, poseTickRef]);

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
    pitLaneWidthM,
    pitSideEntryProgress,
    pitSpeedLimitStart,
    pitBoxProgress,
    pitSpeedLimitEnd,
    pitSideRejoinProgress,
    drsZones,
    startFinishIndex,
    landmarks,
    viewBounds,
    trackMetrics,
    pitMetrics,
    gridSlots,
    carWidthM,
    carLengthM,
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
      if (driver.retired && !driver.hazard_active) {
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
      const hasWorldPose = Boolean(
        worldCoordinateFrame
        && Number.isFinite(Number(driver.world_x_m))
        && Number.isFinite(Number(driver.world_y_m))
        && Number.isFinite(Number(driver.heading_rad)),
      );
      const previousWorldPose = worldPoseAtTime(previousAnimation, now);
      const previousSimulationTimeS = Number(marker.lastSimulationTimeS);
      const observedWorldIntervalMs = marker.lastWorldUpdateAt
        ? now - marker.lastWorldUpdateAt
        : MARKER_WORLD_INTERPOLATION_FALLBACK_MS;
      const boundedWorldIntervalMs = Math.max(
        MARKER_WORLD_INTERPOLATION_MIN_MS,
        Math.min(MARKER_WORLD_INTERPOLATION_MAX_MS, observedWorldIntervalMs),
      );
      marker.worldUpdateIntervalMs = marker.worldUpdateIntervalMs
        ? marker.worldUpdateIntervalMs * 0.72 + boundedWorldIntervalMs * 0.28
        : boundedWorldIntervalMs;
      marker.lastWorldUpdateAt = now;
      const adaptiveWorldInterpolationMs = Math.max(
        MARKER_WORLD_INTERPOLATION_MIN_MS,
        Math.min(
          MARKER_WORLD_INTERPOLATION_MAX_MS,
          marker.worldUpdateIntervalMs * 1.08,
        ),
      );
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

      if (routeChanged && !viewChanged && !hasWorldPose) {
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
        duration: hasWorldPose
          ? (holdVisualProgress ? 1 : adaptiveWorldInterpolationMs)
          : (holdVisualProgress || resetRouteProgress ? 1 : MARKER_INTERPOLATION_MS),
        closed,
        progressRate: displayProgressRate,
        maximumPredictionProgress: (
          MARKER_PREDICTION_MAX_DISTANCE_M / Math.max(1, trackLengthM)
        ),
        fromLateralOffsetM: previousLateralOffsetM,
        toLateralOffsetM: holdVisualProgress
          ? previousLateralOffsetM
          : routeLateralOffsetM,
        lateralSpeedMps: paused
          ? 0
          : Number(driver.lateral_speed_mps || 0) * speedMultiplier,
        slipAngleRad: routeType === 'track'
          ? Number(driver.slip_angle_rad || 0)
          : 0,
        routeTransition: hasWorldPose ? null : routeTransition,
        usesWorldPose: hasWorldPose,
        fromWorldXM: previousWorldPose?.xM ?? Number(driver.world_x_m || 0),
        fromWorldYM: previousWorldPose?.yM ?? Number(driver.world_y_m || 0),
        fromWorldHeadingRad: previousWorldPose?.headingRad
          ?? Number(driver.heading_rad || 0),
        toWorldXM: Number(driver.world_x_m || 0),
        toWorldYM: Number(driver.world_y_m || 0),
        toWorldHeadingRad: Number(driver.heading_rad || 0),
        toWorldVelocityXMps: Number(driver.velocity_x_mps || 0),
        toWorldVelocityYMps: Number(driver.velocity_y_mps || 0),
        toWorldAccelerationXMps2: Number(driver.acceleration_x_mps2 || 0),
        toWorldAccelerationYMps2: Number(driver.acceleration_y_mps2 || 0),
        toWorldYawRateRadS: Number(driver.yaw_rate_rad_s || 0),
        worldPredictionTimeScale: paused ? 0 : speedMultiplier,
        maximumWorldPredictionMs: Math.min(
          MARKER_WORLD_PREDICTION_MAX_MS,
          Math.max(40, marker.worldUpdateIntervalMs * 1.8),
        ),
        maximumWorldPredictionDistanceM: MARKER_WORLD_PREDICTION_MAX_DISTANCE_M,
      };

      if (hasWorldPose) {
        const authoritativeSamples = (driver.trajectory_samples || [])
          .filter((sample) => (
            Number.isFinite(Number(sample.world_x_m))
            && Number.isFinite(Number(sample.world_y_m))
            && Number.isFinite(Number(sample.heading_rad))
            && (
              !Number.isFinite(previousSimulationTimeS)
              || Number(sample.simulation_time_s) > previousSimulationTimeS + 1e-9
            )
          ))
          .map((sample) => ({
            xM: Number(sample.world_x_m),
            yM: Number(sample.world_y_m),
            headingRad: Number(sample.heading_rad),
            simulationTimeS: Number(sample.simulation_time_s),
          }));
        const worldPath = [];
        if (previousWorldPose) {
          worldPath.push({
            ...previousWorldPose,
            simulationTimeS: Number.isFinite(previousSimulationTimeS)
              ? previousSimulationTimeS
              : Number(authoritativeSamples[0]?.simulationTimeS || driver.simulation_time_s),
          });
        }
        worldPath.push(...authoritativeSamples);
        if (worldPath.length < 2) {
          worldPath.push({
            xM: Number(driver.world_x_m || 0),
            yM: Number(driver.world_y_m || 0),
            headingRad: Number(driver.heading_rad || 0),
            simulationTimeS: Number(driver.simulation_time_s || 0),
          });
        }
        marker.routeAnimation.worldPath = worldPath;
        marker.lastSimulationTimeS = Number(driver.simulation_time_s || 0);
      }

      if (isNewMarker || progressDiscontinuous || viewChanged) {
        const initialWorldPose = worldPoseAtTime(marker.routeAnimation, now);
        const { x, y, angle } = initialWorldPose
          ? markerPoseForWorld(
            initialWorldPose,
            worldCoordinateFrame,
            displayRotation,
            displayCenter,
            transform,
          )
          : markerPoseForRoute(
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
        marker.dot.rotation = hasWorldPose
          ? angle
          : angle + Number(driver.slip_angle_rad || 0);
      }

      const stoppedHazard = Boolean(driver.retired && driver.hazard_active);
      const color = stoppedHazard ? 0xff1744 : hexToNumber(driver.team_color);
      const radius = playerSet.has(driver.driver_id) ? 6 : 4;
      const pitStatus = formatPitStatus(driver);
      const maneuverStatus = formatManeuverStatus(driver);
      const trackStatus = formatTrackStatus(driver);
      const contactStatus = formatContactStatus(driver);
      const hazardStatus = formatHazardStatus(driver);
      const avoidanceStatus = formatAvoidanceStatus(driver);
      const localYellowStatus = formatLocalYellowStatus(driver);

      marker.zIndex = driver.position;

      marker.halo.clear();
      if (stoppedHazard) {
        marker.halo.circle(0, 0, radius + 6);
        marker.halo.fill({ color: 0xff1744, alpha: 0.22 });
        marker.halo.stroke({ width: 3, color: 0xffd60a, alpha: 1 });
      } else if (driver.in_pit) {
        marker.halo.circle(0, 0, radius + 3);
        marker.halo.stroke({ width: 1.8, color: 0xf39c12, alpha: 0.9 });
      } else if (driver.contact_active) {
        marker.halo.circle(0, 0, radius + 5);
        marker.halo.fill({ color: 0xfff4f4, alpha: 0.18 });
        marker.halo.stroke({
          width: 2.6,
          color: driver.contact_severity === 'minor' ? 0xff6b35 : 0xff1744,
          alpha: 1,
        });
      } else if (driver.avoidance_active) {
        marker.halo.circle(0, 0, radius + 4);
        marker.halo.fill({ color: 0x00d8ff, alpha: 0.14 });
        marker.halo.stroke({
          width: 2.2,
          color: driver.emergency_braking ? 0xffb000 : 0x00d8ff,
          alpha: 0.95,
        });
      } else if (driver.local_yellow_active) {
        marker.halo.circle(0, 0, radius + 4);
        marker.halo.fill({ color: 0xffd60a, alpha: 0.12 });
        marker.halo.stroke({ width: 2.2, color: 0xffd60a, alpha: 0.95 });
      } else if (driver.handling_state && driver.handling_state !== 'stable') {
        marker.halo.circle(0, 0, radius + 3);
        marker.halo.stroke({
          width: 1.8,
          color: driver.off_track
            ? (driver.track_limits_active
              ? 0xff2bd6
              : (driver.off_track_cause === 'forced_wide' ? 0xff2bd6 : 0xff3b30))
            : driver.handling_state === 'run_wide'
              ? 0xffb000
              : driver.handling_state === 'lockup'
                ? 0xffffff
                : driver.handling_state === 'wheelspin'
                  ? 0x8be9fd
                  : 0xffb000,
          alpha: 0.9,
        });
      } else if (driver.maneuver_active) {
        const maneuverPhase = driver.maneuver_group_phase || driver.maneuver_phase;
        const maneuverColor = MANEUVER_PHASE_COLORS[maneuverPhase] || 0xf5a623;
        marker.halo.circle(0, 0, radius + 4);
        marker.halo.fill({ color: maneuverColor, alpha: 0.12 });
        marker.halo.stroke({ width: 2.2, color: maneuverColor, alpha: 0.95 });
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
      marker.dot.stroke({
        width: stoppedHazard ? 2 : 1,
        color: stoppedHazard ? 0xffd60a : 0xffffff,
        alpha: stoppedHazard ? 1 : 0.45,
      });

      marker.nameLabel.text = stoppedHazard
        ? `DNF ${driver.name}`
        : `${driver.position} ${driver.name}`;
      marker.pitLabel.text = (
        hazardStatus
        || pitStatus
        || contactStatus
        || avoidanceStatus
        || localYellowStatus
        || trackStatus
        || maneuverStatus
      );
      marker.nameLabel.visible = true;
      marker.pitLabel.visible = (
        hazardStatus.length > 0
        || (driver.in_pit && pitStatus.length > 0)
        || (
          zoomPercent >= PHYSICAL_ZOOM_MIN_PERCENT
          && (
            contactStatus.length > 0
            || avoidanceStatus.length > 0
            || localYellowStatus.length > 0
            || trackStatus.length > 0
            || maneuverStatus.length > 0
          )
        )
      );

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
      marker.labelBg.fill({
        color: stoppedHazard ? 0x2b0909 : 0x0a0a0f,
        alpha: driver.in_pit || stoppedHazard ? 0.86 : 0.7,
      });
      marker.labelBg.stroke({
        width: 1,
        color: stoppedHazard ? 0xffd60a : (driver.in_pit ? 0xf39c12 : color),
        alpha: driver.in_pit || stoppedHazard ? 0.9 : 0.55,
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
          maximumPredictionProgress: (
            MARKER_PREDICTION_MAX_DISTANCE_M / Math.max(1, trackLengthM)
          ),
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
    worldCoordinateFrame,
    displayRotation,
    displayCenter,
  ]);

  useEffect(() => {
    if (
      !initialFollowAppliedRef.current
      && zoomPercent >= PHYSICAL_ZOOM_MIN_PERCENT
      && playerDrivers.length > 0
    ) {
      initialFollowAppliedRef.current = true;
      setFollowDriverId(playerDrivers[0].driver_id);
    }
  }, [playerDrivers, zoomPercent]);

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
        {miniMap && (
          <div className="track-canvas__minimap" aria-label="100 percent circuit overview">
            <span className="track-canvas__minimap-label">100% OVERVIEW</span>
            <svg viewBox={miniMap.viewBox} preserveAspectRatio="xMidYMid meet" role="img">
              <polyline
                points={miniMap.trackPoints}
                fill="none"
                stroke="rgba(255,255,255,0.35)"
                strokeWidth={miniMap.trackStroke}
                strokeLinejoin="round"
              />
              {miniMap.sectorLines.map((sector) => (
                <g key={sector.key}>
                  <polyline
                    points={sector.points}
                    fill="none"
                    stroke={sector.color}
                    strokeOpacity="0.72"
                    strokeWidth={miniMap.trackStroke * 0.58}
                    strokeLinejoin="round"
                  />
                  <text
                    x={sector.labelX}
                    y={sector.labelY}
                    fill={sector.color}
                    fontSize={miniMap.trackStroke * 2.2}
                    fontWeight="700"
                  >
                    {sector.label}
                  </text>
                </g>
              ))}
              {miniMap.pitPoints && (
                <polyline
                  points={miniMap.pitPoints}
                  fill="none"
                  stroke="#f5a623"
                  strokeWidth={miniMap.trackStroke * 0.62}
                  strokeLinejoin="round"
                />
              )}
              {miniMap.driverPoints.map((driver) => (
                <circle
                  key={driver.driver_id}
                  ref={(node) => {
                    if (node) miniMapCircleRefs.current.set(driver.driver_id, node);
                    else miniMapCircleRefs.current.delete(driver.driver_id);
                  }}
                  cx={driver.miniX}
                  cy={driver.miniY}
                  r={miniMap.markerRadius * ((playerDriverIds || []).includes(driver.driver_id) ? 1.45 : 1)}
                  fill={driver.retired ? '#ff1744' : driver.team_color}
                  stroke={(playerDriverIds || []).includes(driver.driver_id) ? '#fff' : 'rgba(255,255,255,0.55)'}
                  strokeWidth={miniMap.markerRadius * 0.28}
                />
              ))}
            </svg>
          </div>
        )}
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
