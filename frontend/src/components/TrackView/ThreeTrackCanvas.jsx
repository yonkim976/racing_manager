import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from 'react';
import * as THREE from 'three';
import {
  normalizeCoordinatePath,
  sampleCatmullRomClosed,
} from '../../utils/circuitGeometry';
import {
  appendPoseTickToBuffers,
  bufferedWorldPoseAtTime,
} from './posePlayback';
import { renderToLocalMetricPoint } from './coordinateContract';
import {
  advanceSafetyCarRenderProgress,
  safetyCarProgressAtRenderTime,
} from './safetyCarPlayback';
import './TrackCanvas.css';
import './ThreeTrackCanvas.css';

const ZOOM_LEVELS = [700, 1000, 1500];
const DEFAULT_ZOOM_PERCENT = 1000;
const POSE_PLAYBACK_DELAY_MS = 120;
const TRACK_SAMPLE_SPACING = 0.75;
const CAMERA_HEIGHT_M = 1400;
const CAMERA_TRAILING_M = 430;
const KERB_PATTERN_LENGTH_M = 16;
const TRACK_EDGE_LINE_WIDTH_M = 0.22;
const TRACK_EDGE_LINE_HEIGHT_M = 0.045;
const MEMORY_SAMPLE_RETENTION_MS = 5 * 60 * 1000;
const MAX_MEMORY_SAMPLES = 300;
const MATERIAL_TEXTURE_SLOTS = [
  'map',
  'normalMap',
  'roughnessMap',
  'metalnessMap',
  'aoMap',
  'alphaMap',
  'emissiveMap',
  'bumpMap',
  'displacementMap',
  'environmentMap',
];

function disposeMaterial(material, disposedTextures) {
  if (!material) return;
  MATERIAL_TEXTURE_SLOTS.forEach((slot) => {
    const texture = material[slot];
    if (texture && !disposedTextures.has(texture)) {
      disposedTextures.add(texture);
      texture.dispose?.();
    }
  });
  material.dispose?.();
}

function disposeObject3D(root) {
  const disposedTextures = new Set();
  root?.traverse?.((object) => {
    object.geometry?.dispose?.();
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    materials.forEach((material) => disposeMaterial(material, disposedTextures));
  });
}

function useStableStructuredValue(value) {
  const stableRef = useRef(null);
  if (stableRef.current === null) {
    stableRef.current = {
      source: value,
      signature: JSON.stringify(value ?? null),
      value,
    };
  } else if (stableRef.current.source !== value) {
    const signature = JSON.stringify(value ?? null);
    if (stableRef.current.signature !== signature) {
      stableRef.current = { source: value, signature, value };
    } else {
      stableRef.current.source = value;
    }
  }
  return stableRef.current.value;
}

function normalizeDegrees(value) {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) return 0;
  return ((numeric % 360) + 360) % 360;
}

function stripClosedPoint(coords) {
  if (coords.length < 2) return coords;
  const first = coords[0];
  const last = coords[coords.length - 1];
  if (Math.hypot(first[0] - last[0], first[1] - last[1]) < 1e-6) {
    return coords.slice(0, -1);
  }
  return coords;
}

function rotate2D(x, y, degrees) {
  const radians = degrees * Math.PI / 180;
  const cosine = Math.cos(radians);
  const sine = Math.sin(radians);
  return [x * cosine - y * sine, x * sine + y * cosine];
}

function renderCoordToWorld(coord, coordinateFrame) {
  return renderToLocalMetricPoint(coord, coordinateFrame);
}

function pathMetrics(points, closed = true) {
  const prepared = [...points];
  if (closed && prepared.length > 1) prepared.push(prepared[0]);
  const cumulative = [0];
  for (let index = 1; index < prepared.length; index += 1) {
    cumulative.push(
      cumulative[cumulative.length - 1]
      + Math.hypot(
        prepared[index][0] - prepared[index - 1][0],
        prepared[index][1] - prepared[index - 1][1],
      ),
    );
  }
  return {
    points: prepared,
    cumulative,
    totalLength: cumulative[cumulative.length - 1] || 0,
    closed,
  };
}

function pathPoseAtProgress(metrics, rawProgress) {
  if (metrics.points.length < 2 || metrics.totalLength <= 0) {
    return { x: 0, y: 0, heading: 0 };
  }
  const progress = metrics.closed
    ? ((Number(rawProgress) % 1) + 1) % 1
    : Math.min(1, Math.max(0, Number(rawProgress)));
  const targetDistance = progress * metrics.totalLength;
  let low = 0;
  let high = metrics.cumulative.length - 1;
  while (low + 1 < high) {
    const middle = Math.floor((low + high) / 2);
    if (metrics.cumulative[middle] <= targetDistance) low = middle;
    else high = middle;
  }
  const from = metrics.points[low];
  const to = metrics.points[Math.min(low + 1, metrics.points.length - 1)];
  const segmentLength = Math.max(
    1e-9,
    metrics.cumulative[Math.min(low + 1, metrics.cumulative.length - 1)]
      - metrics.cumulative[low],
  );
  const ratio = Math.min(
    1,
    Math.max(0, (targetDistance - metrics.cumulative[low]) / segmentLength),
  );
  return {
    x: from[0] + (to[0] - from[0]) * ratio,
    y: from[1] + (to[1] - from[1]) * ratio,
    heading: Math.atan2(to[1] - from[1], to[0] - from[0]),
  };
}

function buildWidthSampler(profile, fallbackWidthM) {
  const samples = (profile || [])
    .map(([progress, left, right]) => ({
      progress: ((Number(progress) % 1) + 1) % 1,
      left: Number(left || fallbackWidthM / 2),
      right: Number(right || fallbackWidthM / 2),
    }))
    .sort((a, b) => a.progress - b.progress);
  if (!samples.length) {
    return () => ({ left: fallbackWidthM / 2, right: fallbackWidthM / 2 });
  }
  return (rawProgress) => {
    const progress = ((rawProgress % 1) + 1) % 1;
    let index = samples.length - 1;
    for (let candidate = 0; candidate < samples.length; candidate += 1) {
      if (samples[candidate].progress > progress) break;
      index = candidate;
    }
    const nextIndex = (index + 1) % samples.length;
    const start = samples[index];
    const end = samples[nextIndex];
    const endProgress = nextIndex === 0 ? end.progress + 1 : end.progress;
    const target = progress < start.progress ? progress + 1 : progress;
    const ratio = Math.min(
      1,
      Math.max(0, (target - start.progress) / Math.max(1e-9, endProgress - start.progress)),
    );
    return {
      left: start.left + (end.left - start.left) * ratio,
      right: start.right + (end.right - start.right) * ratio,
    };
  };
}

function smoothCircularBoundary(points, passes = 18, strength = 0.45) {
  let smoothed = points.map((point) => [...point]);
  for (let pass = 0; pass < passes; pass += 1) {
    smoothed = smoothed.map((point, index) => {
      const previous = smoothed[(index - 1 + smoothed.length) % smoothed.length];
      const next = smoothed[(index + 1) % smoothed.length];
      const neighborMidpointX = (previous[0] + next[0]) / 2;
      const neighborMidpointY = (previous[1] + next[1]) / 2;
      return [
        point[0] + (neighborMidpointX - point[0]) * strength,
        point[1] + (neighborMidpointY - point[1]) * strength,
      ];
    });
  }
  return smoothed;
}

function buildSmoothedRibbonEdges(points, widthAt, extraWidthM = 0) {
  const count = points.length;
  const rawLeftEdge = [];
  const rawRightEdge = [];
  for (let index = 0; index < count; index += 1) {
    const previous = points[(index - 1 + count) % count];
    const point = points[index];
    const next = points[(index + 1) % count];
    const dx = next[0] - previous[0];
    const dy = next[1] - previous[1];
    const length = Math.max(1e-9, Math.hypot(dx, dy));
    const normalX = -dy / length;
    const normalY = dx / length;
    const progress = index / count;
    const width = widthAt(progress);
    rawLeftEdge.push([
      point[0] + normalX * (width.left + extraWidthM),
      point[1] + normalY * (width.left + extraWidthM),
    ]);
    rawRightEdge.push([
      point[0] - normalX * (width.right + extraWidthM),
      point[1] - normalY * (width.right + extraWidthM),
    ]);
  }
  // Offset curves can form a visible cusp when the inside radius approaches
  // the half-width of a tight corner. Smooth render boundaries only; vehicle
  // physics, telemetry and the authoritative centreline remain unchanged.
  return {
    leftEdge: smoothCircularBoundary(rawLeftEdge),
    rightEdge: smoothCircularBoundary(rawRightEdge),
  };
}

function buildRibbonGeometry(points, widthAt, heightM = 0, extraWidthM = 0) {
  const count = points.length;
  const { leftEdge, rightEdge } = buildSmoothedRibbonEdges(
    points,
    widthAt,
    extraWidthM,
  );
  const vertices = [];
  const uvs = [];
  const indices = [];
  for (let index = 0; index < count; index += 1) {
    const progress = index / count;
    vertices.push(
      leftEdge[index][0],
      heightM,
      leftEdge[index][1],
      rightEdge[index][0],
      heightM,
      rightEdge[index][1],
    );
    uvs.push(progress * 60, 0, progress * 60, 1);
    const nextIndex = (index + 1) % count;
    indices.push(
      index * 2,
      nextIndex * 2,
      index * 2 + 1,
      index * 2 + 1,
      nextIndex * 2,
      nextIndex * 2 + 1,
    );
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function offsetClosedPath(points, offsetM) {
  const count = points.length;
  return points.map((point, index) => {
    const previous = points[(index - 1 + count) % count];
    const next = points[(index + 1) % count];
    const dx = next[0] - previous[0];
    const dy = next[1] - previous[1];
    const length = Math.max(1e-9, Math.hypot(dx, dy));
    return [
      point[0] - dy / length * offsetM,
      point[1] + dx / length * offsetM,
    ];
  });
}

function buildClosedRibbonGeometry(points, widthM, heightM = 0) {
  const count = points.length;
  if (count < 3) return new THREE.BufferGeometry();
  const vertices = [];
  const uvs = [];
  const indices = [];
  const halfWidth = widthM / 2;
  for (let index = 0; index < count; index += 1) {
    const previous = points[(index - 1 + count) % count];
    const point = points[index];
    const next = points[(index + 1) % count];
    const dx = next[0] - previous[0];
    const dy = next[1] - previous[1];
    const length = Math.max(1e-9, Math.hypot(dx, dy));
    const normalX = -dy / length;
    const normalY = dx / length;
    vertices.push(
      point[0] + normalX * halfWidth,
      heightM,
      point[1] + normalY * halfWidth,
      point[0] - normalX * halfWidth,
      heightM,
      point[1] - normalY * halfWidth,
    );
    uvs.push(index / count, 0, index / count, 1);
    const nextIndex = (index + 1) % count;
    indices.push(
      index * 2,
      nextIndex * 2,
      index * 2 + 1,
      index * 2 + 1,
      nextIndex * 2,
      nextIndex * 2 + 1,
    );
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function closedIndexPoseAtProgress(points, rawProgress) {
  const count = points.length;
  if (!count) return { x: 0, y: 0, heading: 0 };
  const progress = ((Number(rawProgress) % 1) + 1) % 1;
  const scaledIndex = progress * count;
  const index = Math.floor(scaledIndex) % count;
  const nextIndex = (index + 1) % count;
  const ratio = scaledIndex - Math.floor(scaledIndex);
  const point = points[index];
  const next = points[nextIndex];
  const previous = points[(index - 1 + count) % count];
  const following = points[(index + 2) % count];
  return {
    x: point[0] + (next[0] - point[0]) * ratio,
    y: point[1] + (next[1] - point[1]) * ratio,
    heading: Math.atan2(
      following[1] - previous[1],
      following[0] - previous[0],
    ),
  };
}

function buildBoundaryOffsetSegmentPoints(
  boundaryPoints,
  start,
  end,
  offsetM,
  referenceLengthM,
  stepM = 2,
) {
  let range = Number(end) - Number(start);
  if (range < 0) range += 1;
  const steps = Math.max(2, Math.ceil(range * referenceLengthM / stepM));
  return Array.from({ length: steps + 1 }, (_, index) => {
    const progress = (Number(start) + range * index / steps) % 1;
    const pose = closedIndexPoseAtProgress(boundaryPoints, progress);
    return [
      pose.x - Math.sin(pose.heading) * offsetM,
      pose.y + Math.cos(pose.heading) * offsetM,
    ];
  });
}

function buildOpenRibbonGeometry(
  points,
  widthM,
  heightM = 0,
  { taperStartM = 0, taperEndM = 0 } = {},
) {
  if (points.length < 2) return new THREE.BufferGeometry();
  const vertices = [];
  const uvs = [];
  const indices = [];
  const cumulativeLengthsM = [0];
  for (let index = 1; index < points.length; index += 1) {
    cumulativeLengthsM.push(
      cumulativeLengthsM[index - 1]
      + Math.hypot(
        points[index][0] - points[index - 1][0],
        points[index][1] - points[index - 1][1],
      ),
    );
  }
  const totalLengthM = cumulativeLengthsM[cumulativeLengthsM.length - 1];
  for (let index = 0; index < points.length; index += 1) {
    const previous = points[Math.max(0, index - 1)];
    const point = points[index];
    const next = points[Math.min(points.length - 1, index + 1)];
    const dx = next[0] - previous[0];
    const dy = next[1] - previous[1];
    const length = Math.max(1e-9, Math.hypot(dx, dy));
    const normalX = -dy / length;
    const normalY = dx / length;
    const distanceFromStartM = cumulativeLengthsM[index];
    const distanceFromEndM = totalLengthM - distanceFromStartM;
    const startScale = taperStartM > 0
      ? Math.min(1, distanceFromStartM / taperStartM)
      : 1;
    const endScale = taperEndM > 0
      ? Math.min(1, distanceFromEndM / taperEndM)
      : 1;
    // Corner surface zones are finite ribbons. Tapering their ends avoids a
    // full-width square cap being read as a right-angle track boundary.
    const halfWidth = widthM / 2 * Math.min(startScale, endScale);
    vertices.push(
      point[0] + normalX * halfWidth,
      heightM,
      point[1] + normalY * halfWidth,
      point[0] - normalX * halfWidth,
      heightM,
      point[1] - normalY * halfWidth,
    );
    const textureProgress = distanceFromStartM / KERB_PATTERN_LENGTH_M;
    uvs.push(textureProgress, 0, textureProgress, 1);
    if (index < points.length - 1) {
      indices.push(
        index * 2,
        (index + 1) * 2,
        index * 2 + 1,
        index * 2 + 1,
        (index + 1) * 2,
        (index + 1) * 2 + 1,
      );
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function buildSegmentPoints(metrics, start, end, stepM = 4) {
  let range = Number(end) - Number(start);
  if (range < 0) range += 1;
  const steps = Math.max(2, Math.ceil(range * metrics.totalLength / stepM));
  return Array.from({ length: steps + 1 }, (_, index) => {
    const progress = (Number(start) + range * index / steps) % 1;
    const pose = pathPoseAtProgress(metrics, progress);
    return [pose.x, pose.y];
  });
}

function offsetOpenPath(points, offsetM, { taperEndM = 0 } = {}) {
  const remainingLengthsM = Array(points.length).fill(0);
  for (let index = points.length - 2; index >= 0; index -= 1) {
    remainingLengthsM[index] = remainingLengthsM[index + 1]
      + Math.hypot(
        points[index + 1][0] - points[index][0],
        points[index + 1][1] - points[index][1],
      );
  }
  return points.map((point, index) => {
    const previous = points[Math.max(0, index - 1)];
    const next = points[Math.min(points.length - 1, index + 1)];
    const heading = Math.atan2(next[1] - previous[1], next[0] - previous[0]);
    const taperScale = taperEndM > 0
      ? Math.min(1, remainingLengthsM[index] / taperEndM)
      : 1;
    const taperedOffsetM = offsetM * taperScale;
    return [
      point[0] - Math.sin(heading) * taperedOffsetM,
      point[1] + Math.cos(heading) * taperedOffsetM,
    ];
  });
}

function createRoadTexture(renderer) {
  const canvas = document.createElement('canvas');
  canvas.width = 128;
  canvas.height = 128;
  const context = canvas.getContext('2d');
  context.fillStyle = '#565b61';
  context.fillRect(0, 0, 128, 128);
  for (let index = 0; index < 850; index += 1) {
    const shade = 72 + ((index * 37) % 28);
    context.fillStyle = `rgba(${shade},${shade},${shade + 3},0.22)`;
    context.fillRect((index * 53) % 128, (index * 97) % 128, 1, 1);
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
  return texture;
}

function createKerbTexture(renderer) {
  const canvas = document.createElement('canvas');
  canvas.width = 128;
  canvas.height = 16;
  const context = canvas.getContext('2d');
  context.fillStyle = '#f4f4f4';
  context.fillRect(0, 0, 64, 16);
  context.fillStyle = '#d70b12';
  context.fillRect(64, 0, 64, 16);
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.ClampToEdgeWrapping;
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
  return texture;
}

function createTaperedNoseGeometry(lengthM, heightM, rearWidthM, frontWidthM) {
  const halfLength = lengthM / 2;
  const halfHeight = heightM / 2;
  const rearHalfWidth = rearWidthM / 2;
  const frontHalfWidth = frontWidthM / 2;
  const vertices = [
    -halfLength, -halfHeight, -rearHalfWidth,
    -halfLength, -halfHeight, rearHalfWidth,
    -halfLength, halfHeight, -rearHalfWidth,
    -halfLength, halfHeight, rearHalfWidth,
    halfLength, -halfHeight, -frontHalfWidth,
    halfLength, -halfHeight, frontHalfWidth,
    halfLength, halfHeight, -frontHalfWidth,
    halfLength, halfHeight, frontHalfWidth,
  ];
  const indices = [
    0, 4, 6, 0, 6, 2,
    1, 3, 7, 1, 7, 5,
    2, 6, 7, 2, 7, 3,
    0, 1, 5, 0, 5, 4,
    4, 5, 7, 4, 7, 6,
    0, 2, 3, 0, 3, 1,
  ];
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function createCarModel(driver, carWidthM, carLengthM) {
  const group = new THREE.Group();
  const teamColor = new THREE.Color(driver.team_color || '#e10600');
  const bodyMaterial = new THREE.MeshStandardMaterial({
    color: teamColor,
    metalness: 0.48,
    roughness: 0.34,
  });
  const carbonMaterial = new THREE.MeshStandardMaterial({
    color: 0x11151b,
    metalness: 0.12,
    roughness: 0.72,
  });
  const aeroMaterial = new THREE.MeshStandardMaterial({
    color: 0x46505d,
    metalness: 0.30,
    roughness: 0.48,
  });
  const tyreMaterial = new THREE.MeshStandardMaterial({
    color: 0x07080a,
    metalness: 0.04,
    roughness: 0.92,
  });
  const accentMaterial = new THREE.MeshStandardMaterial({
    color: teamColor.clone().offsetHSL(0, 0, 0.16),
    metalness: 0.38,
    roughness: 0.36,
  });

  const chassis = new THREE.Mesh(
    new THREE.BoxGeometry(carLengthM * 0.58, 0.42, carWidthM * 0.58),
    bodyMaterial,
  );
  chassis.position.y = 0.42;
  const nose = new THREE.Mesh(
    createTaperedNoseGeometry(
      carLengthM * 0.24,
      0.18,
      carWidthM * 0.24,
      carWidthM * 0.10,
    ),
    accentMaterial,
  );
  nose.position.set(carLengthM * 0.38, 0.35, 0);
  const cockpit = new THREE.Mesh(
    new THREE.BoxGeometry(carLengthM * 0.23, 0.38, carWidthM * 0.38),
    carbonMaterial,
  );
  cockpit.position.set(-carLengthM * 0.04, 0.70, 0);

  const frontWing = new THREE.Mesh(
    new THREE.BoxGeometry(carLengthM * 0.06, 0.11, carWidthM * 0.96),
    aeroMaterial,
  );
  frontWing.position.set(carLengthM * 0.46, 0.20, 0);
  const rearWing = new THREE.Mesh(
    new THREE.BoxGeometry(carLengthM * 0.08, 0.38, carWidthM * 0.88),
    aeroMaterial,
  );
  rearWing.position.set(-carLengthM * 0.45, 0.52, 0);

  const frontEndplateGeometry = new THREE.BoxGeometry(
    carLengthM * 0.11,
    0.20,
    0.055,
  );
  [-1, 1].forEach((side) => {
    const endplate = new THREE.Mesh(frontEndplateGeometry, accentMaterial);
    endplate.position.set(
      carLengthM * 0.46,
      0.27,
      side * carWidthM * 0.48,
    );
    group.add(endplate);
  });

  const tyreGeometry = new THREE.BoxGeometry(0.72, 0.42, 0.34);
  const axleXs = [carLengthM * 0.27, -carLengthM * 0.29];
  axleXs.forEach((x) => {
    [-1, 1].forEach((side) => {
      const tyre = new THREE.Mesh(tyreGeometry, tyreMaterial);
      tyre.position.set(x, 0.28, side * carWidthM * 0.43);
      group.add(tyre);
    });
  });

  const shadow = new THREE.Mesh(
    new THREE.PlaneGeometry(carLengthM * 0.92, carWidthM * 1.05),
    new THREE.MeshBasicMaterial({
      color: 0x000000,
      transparent: true,
      opacity: 0.34,
      depthWrite: false,
    }),
  );
  shadow.rotation.x = -Math.PI / 2;
  shadow.position.y = 0.035;
  group.add(shadow, chassis, nose, cockpit, frontWing, rearWing);
  group.userData.bodyMaterial = bodyMaterial;
  group.userData.accentMaterial = accentMaterial;
  return group;
}

function createSafetyCarModel(carWidthM, carLengthM) {
  const group = new THREE.Group();
  const yellowMaterial = new THREE.MeshStandardMaterial({
    color: 0xf7d038,
    metalness: 0.54,
    roughness: 0.30,
  });
  const darkMaterial = new THREE.MeshStandardMaterial({
    color: 0x121820,
    metalness: 0.22,
    roughness: 0.48,
  });
  const tyreMaterial = new THREE.MeshStandardMaterial({
    color: 0x050608,
    roughness: 0.94,
  });
  const lightMaterial = new THREE.MeshStandardMaterial({
    color: 0xffa600,
    emissive: 0xff7a00,
    emissiveIntensity: 3.2,
    toneMapped: false,
  });

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(carLengthM * 0.76, 0.68, carWidthM * 0.82),
    yellowMaterial,
  );
  body.position.y = 0.48;
  const cabin = new THREE.Mesh(
    new THREE.BoxGeometry(carLengthM * 0.34, 0.54, carWidthM * 0.68),
    darkMaterial,
  );
  cabin.position.set(-carLengthM * 0.04, 0.96, 0);
  const nose = new THREE.Mesh(
    new THREE.BoxGeometry(carLengthM * 0.22, 0.38, carWidthM * 0.72),
    yellowMaterial,
  );
  nose.position.set(carLengthM * 0.45, 0.42, 0);
  const lightBar = new THREE.Mesh(
    new THREE.BoxGeometry(carLengthM * 0.11, 0.12, carWidthM * 0.62),
    lightMaterial,
  );
  lightBar.position.set(-carLengthM * 0.04, 1.30, 0);

  const tyreGeometry = new THREE.BoxGeometry(0.72, 0.46, 0.36);
  [carLengthM * 0.28, -carLengthM * 0.28].forEach((x) => {
    [-1, 1].forEach((side) => {
      const tyre = new THREE.Mesh(tyreGeometry, tyreMaterial);
      tyre.position.set(x, 0.30, side * carWidthM * 0.47);
      group.add(tyre);
    });
  });

  const shadow = new THREE.Mesh(
    new THREE.PlaneGeometry(carLengthM, carWidthM * 1.15),
    new THREE.MeshBasicMaterial({
      color: 0x000000,
      transparent: true,
      opacity: 0.42,
      depthWrite: false,
    }),
  );
  shadow.rotation.x = -Math.PI / 2;
  shadow.position.y = 0.035;
  group.add(shadow, body, cabin, nose, lightBar);
  group.userData.lightMaterial = lightMaterial;
  group.userData.poseInitialized = false;
  group.userData.renderProgress = null;
  group.userData.renderRoute = null;
  group.visible = false;
  return group;
}

function lerpAngle(current, target, alpha) {
  const difference = Math.atan2(
    Math.sin(target - current),
    Math.cos(target - current),
  );
  return current + difference * alpha;
}

function createGate(
  trackMetrics,
  progress,
  widthM,
  color,
  heightM = 0.07,
  opacity = 1,
) {
  const pose = pathPoseAtProgress(trackMetrics, progress);
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(0.32, 0.06, widthM),
    new THREE.MeshBasicMaterial({
      color,
      toneMapped: false,
      transparent: opacity < 1,
      opacity,
      depthWrite: opacity >= 1,
    }),
  );
  mesh.position.set(pose.x, heightM, pose.y);
  mesh.rotation.y = -pose.heading;
  return mesh;
}

function colorNumber(value, fallback = 0xe10600) {
  try {
    return new THREE.Color(value || fallback).getHex();
  } catch {
    return fallback;
  }
}

function formatPitTiming(driver) {
  if (!driver?.in_pit) return null;
  const totalSeconds = Math.max(0, Number(driver.pit_elapsed || 0));
  const mergeState = String(driver.pit_merge_state || '');
  if (mergeState === 'yield') {
    return { phase: 'yield', primary: 'PIT EXIT · YIELD', secondary: `${totalSeconds.toFixed(1)}s` };
  }
  if (mergeState === 'hold') {
    return { phase: 'hold', primary: 'PIT EXIT · HOLD', secondary: `${totalSeconds.toFixed(1)}s` };
  }
  if (mergeState === 'merge') {
    return { phase: 'merge', primary: 'PIT EXIT · MERGE', secondary: `${totalSeconds.toFixed(1)}s` };
  }
  if (driver.pit_phase === 'stop') {
    const stopSeconds = Math.max(0, Number(driver.pit_stop_elapsed || 0));
    return {
      phase: 'stop',
      primary: `STOP ${stopSeconds.toFixed(1)}s`,
      secondary: `PIT ${totalSeconds.toFixed(1)}s`,
    };
  }
  if (driver.pit_phase === 'in') {
    return { phase: 'in', primary: 'PIT IN', secondary: `${totalSeconds.toFixed(1)}s` };
  }
  if (driver.pit_phase === 'out') {
    return { phase: 'out', primary: 'PIT OUT', secondary: `${totalSeconds.toFixed(1)}s` };
  }
  return { phase: 'pit', primary: 'PIT', secondary: `${totalSeconds.toFixed(1)}s` };
}

function buildGarageTeams(positions) {
  const teams = new Map();
  (positions || []).forEach((driver) => {
    const name = String(driver.team || '').trim();
    const color = String(driver.team_color || '#777777');
    const key = name || color;
    const pitBoxProgress = Number(driver.pit_box_progress);
    if (!teams.has(key)) {
      teams.set(key, {
        key,
        name: name || 'TEAM',
        color,
        pitBoxProgress: Number.isFinite(pitBoxProgress) ? pitBoxProgress : null,
      });
    }
  });
  return [...teams.values()].sort((left, right) => {
    if (
      Number.isFinite(left.pitBoxProgress)
      && Number.isFinite(right.pitBoxProgress)
    ) {
      return left.pitBoxProgress - right.pitBoxProgress;
    }
    return left.name.localeCompare(right.name);
  });
}

function createTeamGarageSign(team) {
  const canvas = document.createElement('canvas');
  canvas.width = 512;
  canvas.height = 96;
  const context = canvas.getContext('2d');
  if (!context) return null;

  context.fillStyle = '#f4f0e8';
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = team.color || '#777777';
  context.fillRect(0, canvas.height - 14, canvas.width, 14);
  context.fillStyle = '#15191d';
  context.font = '700 46px Arial, sans-serif';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillText(String(team.name || 'TEAM').toUpperCase(), 256, 42, 470);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;
  texture.needsUpdate = true;
  return texture;
}

function createPitGarageBay(
  team,
  carLengthM,
  pitBoxOffset,
  pitLaneWidthM,
  garageSpanM,
  isFirstGarage,
  isLastGarage,
) {
  const group = new THREE.Group();
  const teamColor = colorNumber(team.color, 0x777777);
  const buildingSpanM = Math.max(12.0, garageSpanM);
  const pitBoxLengthM = Math.max(7.2, carLengthM + 2.0);
  const pitBoxWidthM = 3.25;
  const garageFrontM = Math.max(pitBoxOffset + pitBoxWidthM / 2 + 1.0, 8);
  const garageDepthM = 7.2;
  const garageHeightM = 4.6;
  const doorCount = 3;
  const doorSpanM = buildingSpanM / doorCount;
  const apronStartZ = pitLaneWidthM / 2 - 0.25;
  const apronEndZ = garageFrontM + 0.18;
  const apronDepthM = apronEndZ - apronStartZ;
  const apronBeforeM = isFirstGarage ? 24 : 0;
  const apronAfterM = isLastGarage ? 24 : 0;
  const apronSpanM = buildingSpanM + apronBeforeM + apronAfterM;
  const apronCenterX = (apronAfterM - apronBeforeM) / 2;

  const concreteMaterial = new THREE.MeshStandardMaterial({
    color: 0xd7d1c5,
    roughness: 0.93,
  });
  const roofMaterial = new THREE.MeshStandardMaterial({
    color: 0xc8c2b7,
    roughness: 0.91,
  });
  const darkMaterial = new THREE.MeshStandardMaterial({
    color: 0x11161a,
    roughness: 0.82,
  });
  const teamMaterial = new THREE.MeshStandardMaterial({
    color: teamColor,
    metalness: 0.08,
    roughness: 0.68,
  });
  const boxMaterial = new THREE.MeshBasicMaterial({
    color: teamColor,
    transparent: true,
    opacity: 0.68,
    toneMapped: false,
  });
  const lineMaterial = new THREE.MeshBasicMaterial({
    color: 0xf4f5f7,
    transparent: true,
    opacity: 0.88,
    toneMapped: false,
  });

  const garageApron = new THREE.Mesh(
    new THREE.BoxGeometry(apronSpanM, 0.035, apronDepthM),
    new THREE.MeshStandardMaterial({
      color: 0x70757c,
      roughness: 0.97,
    }),
  );
  garageApron.position.set(
    apronCenterX,
    -0.004,
    apronStartZ + apronDepthM / 2,
  );

  const fastLaneSeparator = new THREE.Mesh(
    new THREE.BoxGeometry(apronSpanM, 0.025, 0.13),
    lineMaterial,
  );
  fastLaneSeparator.position.set(
    apronCenterX,
    0.045,
    pitLaneWidthM / 2,
  );

  const garageThreshold = new THREE.Mesh(
    new THREE.BoxGeometry(buildingSpanM + 0.08, 0.045, 0.72),
    concreteMaterial,
  );
  garageThreshold.position.set(0, 0.018, garageFrontM - 0.36);

  const boxFloor = new THREE.Mesh(
    new THREE.BoxGeometry(pitBoxLengthM, 0.035, pitBoxWidthM),
    new THREE.MeshStandardMaterial({ color: 0x626870, roughness: 0.97 }),
  );
  boxFloor.position.set(0, 0.035, pitBoxOffset);

  const centerStripe = new THREE.Mesh(
    new THREE.BoxGeometry(pitBoxLengthM * 0.70, 0.025, 0.20),
    boxMaterial,
  );
  centerStripe.position.set(0, 0.075, pitBoxOffset);

  [-1, 1].forEach((side) => {
    const boxLine = new THREE.Mesh(
      new THREE.BoxGeometry(pitBoxLengthM, 0.025, 0.10),
      lineMaterial,
    );
    boxLine.position.set(0, 0.078, pitBoxOffset + side * pitBoxWidthM / 2);
    group.add(boxLine);
  });
  [-1, 1].forEach((side) => {
    const endLine = new THREE.Mesh(
      new THREE.BoxGeometry(0.10, 0.025, pitBoxWidthM),
      lineMaterial,
    );
    endLine.position.set(side * pitBoxLengthM / 2, 0.078, pitBoxOffset);
    group.add(endLine);
  });

  const garageFloor = new THREE.Mesh(
    new THREE.BoxGeometry(buildingSpanM + 0.08, 0.12, garageDepthM),
    concreteMaterial,
  );
  garageFloor.position.set(0, 0.02, garageFrontM + garageDepthM / 2);

  const backWall = new THREE.Mesh(
    new THREE.BoxGeometry(buildingSpanM + 0.08, garageHeightM, 0.28),
    darkMaterial,
  );
  backWall.position.set(0, garageHeightM / 2, garageFrontM + garageDepthM);

  const roof = new THREE.Mesh(
    new THREE.BoxGeometry(buildingSpanM + 0.12, 0.28, garageDepthM + 1.2),
    roofMaterial,
  );
  roof.position.set(0, garageHeightM, garageFrontM + garageDepthM / 2 - 0.38);

  const canopy = new THREE.Mesh(
    new THREE.BoxGeometry(buildingSpanM + 0.12, 0.18, 1.8),
    roofMaterial,
  );
  canopy.position.set(0, garageHeightM - 0.08, garageFrontM - 0.62);

  const upperFacade = new THREE.Mesh(
    new THREE.BoxGeometry(buildingSpanM + 0.08, 0.82, 0.36),
    concreteMaterial,
  );
  upperFacade.position.set(0, garageHeightM - 0.54, garageFrontM);

  const teamBand = new THREE.Mesh(
    new THREE.BoxGeometry(buildingSpanM - 0.35, 0.16, 0.42),
    teamMaterial,
  );
  teamBand.position.set(0, garageHeightM - 0.92, garageFrontM - 0.04);

  const roofTeamStripe = new THREE.Mesh(
    new THREE.BoxGeometry(buildingSpanM - 0.20, 0.035, 0.72),
    teamMaterial,
  );
  roofTeamStripe.position.set(
    0,
    garageHeightM + 0.16,
    garageFrontM + garageDepthM / 2 - 0.38,
  );

  for (let doorIndex = 0; doorIndex < doorCount; doorIndex += 1) {
    const doorCenterX = -buildingSpanM / 2 + doorSpanM * (doorIndex + 0.5);
    const opening = new THREE.Mesh(
      new THREE.BoxGeometry(Math.max(1.0, doorSpanM - 0.36), garageHeightM - 1.1, 0.26),
      darkMaterial,
    );
    opening.position.set(
      doorCenterX,
      (garageHeightM - 1.1) / 2,
      garageFrontM + 0.10,
    );
    group.add(opening);

    const equipment = new THREE.Mesh(
      new THREE.BoxGeometry(Math.max(0.9, doorSpanM * 0.58), 1.05, 0.85),
      teamMaterial,
    );
    equipment.position.set(
      doorCenterX,
      0.58,
      garageFrontM + garageDepthM - 0.72,
    );
    group.add(equipment);
  }

  for (let dividerIndex = 0; dividerIndex < doorCount; dividerIndex += 1) {
    const divider = new THREE.Mesh(
      new THREE.BoxGeometry(0.28, garageHeightM, 0.52),
      concreteMaterial,
    );
    divider.position.set(
      -buildingSpanM / 2 + dividerIndex * doorSpanM,
      garageHeightM / 2,
      garageFrontM,
    );
    group.add(divider);
  }
  if (isLastGarage) {
    const endPillar = new THREE.Mesh(
      new THREE.BoxGeometry(0.28, garageHeightM, 0.52),
      concreteMaterial,
    );
    endPillar.position.set(
      buildingSpanM / 2,
      garageHeightM / 2,
      garageFrontM,
    );
    group.add(endPillar);
  }

  const signTexture = createTeamGarageSign(team);
  if (signTexture) {
    const sign = new THREE.Mesh(
      new THREE.PlaneGeometry(Math.min(buildingSpanM - 0.7, 11.5), 1.65),
      new THREE.MeshBasicMaterial({
        map: signTexture,
        toneMapped: false,
      }),
    );
    sign.position.set(0, garageHeightM - 0.53, garageFrontM - 0.20);
    sign.rotation.y = Math.PI;
    group.add(sign);
  }

  group.add(
    garageApron,
    fastLaneSeparator,
    garageThreshold,
    boxFloor,
    centerStripe,
    garageFloor,
    backWall,
    roof,
    canopy,
    upperFacade,
    teamBand,
    roofTeamStripe,
  );
  group.userData.team = team.name;
  return group;
}

export default function ThreeTrackCanvas({
  trackCoords,
  worldCoordinateFrame = null,
  trackWidthM = 12,
  carWidthM = 1.9,
  carLengthM = 5.0,
  gridSlots = [],
  trackWidthProfile = [],
  surfaceZones = [],
  racingLineCoords = [],
  pitLaneCoords = [],
  pitExitLaneCoords = [],
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
  displayRotationDeg = 0,
  showBearing = false,
  positions = [],
  poseTickRef = null,
  playerDriverIds = [],
  speedMultiplier = 1,
  paused = false,
  racePhase = 'green',
  safetyCarVisible = false,
  safetyCarRoute = 'track',
  safetyCarProgress = 0,
  safetyCarProgressRate = 0,
  safetyCarPitLaneProgress = 0,
  websocketState = 'closed',
  onPerformanceStats = null,
  onRendererDisposed = null,
  performanceResetToken = 0,
}) {
  const containerRef = useRef(null);
  const rendererRef = useRef(null);
  const sceneRef = useRef(null);
  const cameraRef = useRef(null);
  const rootRef = useRef(null);
  const carsRef = useRef(new Map());
  const garageGroupRef = useRef(null);
  const labelsRef = useRef(new Map());
  const miniMapCirclesRef = useRef(new Map());
  const miniMapSvgRef = useRef(null);
  const miniMapViewportRef = useRef(null);
  const safetyCarRef = useRef(null);
  const safetyCarLabelRef = useRef(null);
  const safetyCarMiniMapRef = useRef(null);
  const safetyCarTelemetryRef = useRef({
    racePhase: 'green',
    visible: false,
    route: 'track',
    progress: 0,
    progressRate: 0,
    pitLaneProgress: 0,
    pitLaneProgressRate: 0,
    receivedAtMs: 0,
  });
  const poseBuffersRef = useRef(new Map());
  const playbackRef = useRef({
    lastTickPhysicsFrame: -1,
    latestSimulationTimeS: 0,
    latestReceivedAtMs: 0,
    speedMultiplier: 1,
    paused: false,
    renderSimulationTimeS: null,
  });
  const positionsRef = useRef(positions || []);
  const pausedRef = useRef(paused);
  const speedMultiplierRef = useRef(speedMultiplier);
  const followDriverIdRef = useRef(null);
  const manualCameraRef = useRef(false);
  const zoomRef = useRef(DEFAULT_ZOOM_PERCENT);
  const cameraFocusRef = useRef(new THREE.Vector3());
  const cameraProjectionRef = useRef(() => {});
  const memorySamplesRef = useRef([]);
  const sceneBuildCountRef = useRef(0);
  const garageBuildCountRef = useRef(0);
  const carModelBuildCountRef = useRef(0);
  const rendererLifecycleIdRef = useRef(
    `renderer-${globalThis.crypto?.randomUUID?.() || Math.random().toString(36).slice(2)}`,
  );
  const websocketStateRef = useRef(websocketState);
  const [sceneReady, setSceneReady] = useState(false);
  const [zoomPercent, setZoomPercent] = useState(DEFAULT_ZOOM_PERCENT);
  const [followDriverId, setFollowDriverId] = useState(null);
  const [cameraMode, setCameraMode] = useState('follow');
  const [performanceStats, setPerformanceStats] = useState({
    fps: 0,
    frameP95Ms: 0,
    slowFrames: 0,
    calls: 0,
    triangles: 0,
    geometries: 0,
    textures: 0,
    jsHeapMb: null,
    jsHeapPeakMb: null,
    jsHeapTrendMbPerMin: null,
    domNodes: 0,
    poseSamples: 0,
    sceneBuilds: 0,
    garageBuilds: 0,
    carModelsBuilt: 0,
  });
  const performanceStatsRef = useRef(performanceStats);

  // Socket updates replace the race payload several times per second. Keep
  // circuit data referentially stable so a pose update cannot rebuild WebGL.
  const stableTrackCoords = useStableStructuredValue(trackCoords);
  const stableCoordinateFrame = useStableStructuredValue(worldCoordinateFrame);
  const stableGridSlots = useStableStructuredValue(gridSlots);
  const stableTrackWidthProfile = useStableStructuredValue(trackWidthProfile);
  const stableSurfaceZones = useStableStructuredValue(surfaceZones);
  const stableRacingLineCoords = useStableStructuredValue(racingLineCoords);
  const stablePitLaneCoords = useStableStructuredValue(pitLaneCoords);
  const stablePitExitLaneCoords = useStableStructuredValue(pitExitLaneCoords);
  const stableDrsZones = useStableStructuredValue(drsZones);
  const stableSectors = useStableStructuredValue(sectors);
  const garageTeams = useMemo(() => buildGarageTeams(positions), [positions]);
  const stableGarageTeams = useStableStructuredValue(garageTeams);

  useEffect(() => {
    memorySamplesRef.current = [];
  }, [performanceResetToken]);

  useEffect(() => {
    performanceStatsRef.current = performanceStats;
  }, [performanceStats]);

  useEffect(() => {
    websocketStateRef.current = websocketState;
  }, [websocketState]);

  useEffect(() => {
    const desktopApi = window.desktopDiagnostics;
    if (!desktopApi) return undefined;
    const recordSnapshot = () => {
      const stats = performanceStatsRef.current || {};
      desktopApi.recordRendererSnapshot({
        renderer_lifecycle_id: rendererLifecycleIdRef.current,
        renderer_active: true,
        react_phase: 'race',
        websocket_state: websocketStateRef.current,
        websocket_count: websocketStateRef.current === 'open' ? 1 : 0,
        js_heap_current_bytes: stats.jsHeapMb == null ? null : stats.jsHeapMb * 1024 * 1024,
        js_heap_peak_bytes: stats.jsHeapPeakMb == null ? null : stats.jsHeapPeakMb * 1024 * 1024,
        js_heap_trend_mb_per_min: stats.jsHeapTrendMbPerMin,
        blink_allocated_bytes: null,
        dom_nodes: Number(stats.domNodes || 0),
        canvas_count: document.querySelectorAll('canvas').length,
        active_webgl_contexts: 1,
        geometries: Number(stats.geometries || 0),
        textures: Number(stats.textures || 0),
        render_calls: Number(stats.calls || 0),
        triangles: Number(stats.triangles || 0),
        scene_builds: Number(stats.sceneBuilds || 0),
        garage_builds: Number(stats.garageBuilds || 0),
        car_models_built: Number(stats.carModelsBuilt || 0),
        pose_samples: Number(stats.poseSamples || 0),
        race_end_overlay: false,
      }).catch(() => {});
    };
    recordSnapshot();
    const interval = window.setInterval(recordSnapshot, 5000);
    return () => window.clearInterval(interval);
  }, []);

  const normalizedTrackCoords = useMemo(
    () => normalizeCoordinatePath(stableTrackCoords),
    [stableTrackCoords],
  );
  const denseTrackCoords = useMemo(() => sampleCatmullRomClosed(
    stripClosedPoint(normalizedTrackCoords),
    TRACK_SAMPLE_SPACING,
  ), [normalizedTrackCoords]);
  const worldTrackPoints = useMemo(
    () => stripClosedPoint(denseTrackCoords).map(
      (coord) => renderCoordToWorld(coord, stableCoordinateFrame),
    ),
    [denseTrackCoords, stableCoordinateFrame],
  );
  const worldPitPoints = useMemo(
    () => normalizeCoordinatePath(stablePitLaneCoords).map(
      (coord) => renderCoordToWorld(coord, stableCoordinateFrame),
    ),
    [stableCoordinateFrame, stablePitLaneCoords],
  );
  const worldPitExitLanePoints = useMemo(
    () => normalizeCoordinatePath(stablePitExitLaneCoords).map(
      (coord) => renderCoordToWorld(coord, stableCoordinateFrame),
    ),
    [stableCoordinateFrame, stablePitExitLaneCoords],
  );
  const denseRacingLineCoords = useMemo(() => sampleCatmullRomClosed(
    stripClosedPoint(normalizeCoordinatePath(stableRacingLineCoords)),
    TRACK_SAMPLE_SPACING,
  ), [stableRacingLineCoords]);
  const worldRacingLinePoints = useMemo(
    () => stripClosedPoint(denseRacingLineCoords).map(
      (coord) => renderCoordToWorld(coord, stableCoordinateFrame),
    ),
    [denseRacingLineCoords, stableCoordinateFrame],
  );
  const trackMetrics = useMemo(
    () => pathMetrics(worldTrackPoints, true),
    [worldTrackPoints],
  );
  const pitMetrics = useMemo(
    () => pathMetrics(worldPitPoints, false),
    [worldPitPoints],
  );
  const renderedPitPoints = useMemo(
    () => (
      worldPitExitLanePoints.length >= 2 && pitMetrics.totalLength > 0
        ? buildSegmentPoints(pitMetrics, 0, pitSideRejoinProgress, 2)
        : worldPitPoints
    ),
    [pitMetrics, pitSideRejoinProgress, worldPitExitLanePoints.length, worldPitPoints],
  );
  const displayRotation = normalizeDegrees(displayRotationDeg);
  const playerDrivers = useMemo(
    () => (positions || [])
      .filter((driver) => playerDriverIds.includes(driver.driver_id) && !driver.retired)
      .sort((a, b) => a.position - b.position),
    [playerDriverIds, positions],
  );
  const followedDriver = playerDrivers.find(
    (driver) => driver.driver_id === followDriverId,
  ) || null;

  const miniMap = useMemo(() => {
    if (!worldTrackPoints.length) return null;
    const rotatedTrack = worldTrackPoints.map(([x, y]) => rotate2D(x, y, displayRotation));
    const rotatedPit = [...renderedPitPoints, ...worldPitExitLanePoints]
      .map(([x, y]) => rotate2D(x, y, displayRotation));
    const xs = rotatedTrack.map((point) => point[0]);
    const ys = rotatedTrack.map((point) => point[1]);
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);
    const padding = Math.max(maxX - minX, maxY - minY) * 0.05;
    return {
      viewBox: `${minX - padding} ${minY - padding} ${maxX - minX + padding * 2} ${maxY - minY + padding * 2}`,
      trackPoints: rotatedTrack.map((point) => point.join(',')).join(' '),
      pitPoints: rotatedPit.map((point) => point.join(',')).join(' '),
      markerRadius: Math.max(maxX - minX, maxY - minY) * 0.009,
      strokeWidth: Math.max(maxX - minX, maxY - minY) * 0.012,
    };
  }, [displayRotation, renderedPitPoints, worldPitExitLanePoints, worldTrackPoints]);

  useEffect(() => {
    positionsRef.current = positions || [];
  }, [positions]);

  useEffect(() => {
    pausedRef.current = paused;
  }, [paused]);

  useEffect(() => {
    speedMultiplierRef.current = speedMultiplier;
  }, [speedMultiplier]);

  useEffect(() => {
    const now = performance.now();
    const previous = safetyCarTelemetryRef.current;
    const elapsedSeconds = Math.max(0.001, (now - previous.receivedAtMs) / 1000);
    const nextPitProgress = Number(safetyCarPitLaneProgress || 0);
    const samePitRoute = previous.route === 'pit' && safetyCarRoute === 'pit';
    const measuredPitRate = samePitRoute
      ? (nextPitProgress - previous.pitLaneProgress) / elapsedSeconds
      : 0;
    safetyCarTelemetryRef.current = {
      racePhase,
      visible: Boolean(safetyCarVisible),
      route: safetyCarRoute === 'pit' ? 'pit' : 'track',
      progress: Number(safetyCarProgress || 0),
      progressRate: Number(safetyCarProgressRate || 0),
      pitLaneProgress: nextPitProgress,
      pitLaneProgressRate: Math.max(0, Math.min(0.2, measuredPitRate)),
      receivedAtMs: now,
    };
  }, [
    racePhase,
    safetyCarPitLaneProgress,
    safetyCarProgress,
    safetyCarProgressRate,
    safetyCarRoute,
    safetyCarVisible,
  ]);

  useEffect(() => {
    followDriverIdRef.current = followDriverId;
  }, [followDriverId]);

  useEffect(() => {
    manualCameraRef.current = cameraMode === 'manual';
  }, [cameraMode]);

  useEffect(() => {
    zoomRef.current = zoomPercent;
    cameraProjectionRef.current();
  }, [zoomPercent]);

  useEffect(() => {
    if (followDriverId === null && playerDrivers.length) {
      setFollowDriverId(playerDrivers[0].driver_id);
    }
  }, [followDriverId, playerDrivers]);

  const followDriver = useCallback((driverId) => {
    manualCameraRef.current = false;
    setCameraMode('follow');
    setFollowDriverId(driverId);
  }, []);

  const handleMiniMapClick = useCallback((event) => {
    const svg = miniMapSvgRef.current;
    const screenMatrix = svg?.getScreenCTM();
    if (!svg || !screenMatrix) return;
    const point = svg.createSVGPoint();
    point.x = event.clientX;
    point.y = event.clientY;
    const worldPoint = point.matrixTransform(screenMatrix.inverse());
    cameraFocusRef.current.set(worldPoint.x, 0, worldPoint.y);
    manualCameraRef.current = true;
    setCameraMode('manual');
  }, []);

  useEffect(() => {
    const container = containerRef.current;
    if (!container || worldTrackPoints.length < 4) return undefined;

    let destroyed = false;
    const rendererLifecycleId = rendererLifecycleIdRef.current;
    sceneBuildCountRef.current += 1;
    const cars = carsRef.current;
    const labels = labelsRef.current;
    const miniMapCircles = miniMapCirclesRef.current;
    const poseBuffers = poseBuffersRef.current;
    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      powerPreference: 'high-performance',
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.05;
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x080b0d);
    scene.fog = new THREE.Fog(0x080b0d, 2100, 6200);
    sceneRef.current = scene;

    const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.1, 5000);
    camera.up.set(0, 1, 0);
    cameraRef.current = camera;

    const root = new THREE.Group();
    root.rotation.y = -displayRotation * Math.PI / 180;
    scene.add(root);
    rootRef.current = root;

    const ambient = new THREE.HemisphereLight(0xe8f3ff, 0x26321f, 1.9);
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.6);
    keyLight.position.set(-600, 1200, 420);
    scene.add(ambient, keyLight);

    const rotatedPoints = worldTrackPoints.map(([x, y]) => rotate2D(x, y, displayRotation));
    const xs = rotatedPoints.map((point) => point[0]);
    const ys = rotatedPoints.map((point) => point[1]);
    const bounds = {
      minX: Math.min(...xs),
      maxX: Math.max(...xs),
      minY: Math.min(...ys),
      maxY: Math.max(...ys),
    };
    const boundsWidth = Math.max(1, bounds.maxX - bounds.minX);
    const boundsHeight = Math.max(1, bounds.maxY - bounds.minY);
    cameraFocusRef.current.set(
      (bounds.minX + bounds.maxX) / 2,
      0,
      (bounds.minY + bounds.maxY) / 2,
    );

    const roadTexture = createRoadTexture(renderer);
    const kerbTexture = createKerbTexture(renderer);
    const widthAt = buildWidthSampler(stableTrackWidthProfile, trackWidthM);
    const roadBoundaryEdges = buildSmoothedRibbonEdges(
      worldTrackPoints,
      widthAt,
    );
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(boundsWidth * 2.5, boundsHeight * 2.5),
      new THREE.MeshStandardMaterial({ color: 0x17331c, roughness: 1 }),
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.set(
      (Math.min(...worldTrackPoints.map((point) => point[0]))
        + Math.max(...worldTrackPoints.map((point) => point[0]))) / 2,
      -0.09,
      (Math.min(...worldTrackPoints.map((point) => point[1]))
        + Math.max(...worldTrackPoints.map((point) => point[1]))) / 2,
    );
    root.add(ground);

    const shoulder = new THREE.Mesh(
      buildRibbonGeometry(worldTrackPoints, widthAt, -0.025, 2.3),
      new THREE.MeshStandardMaterial({ color: 0x51555a, roughness: 0.96 }),
    );
    const road = new THREE.Mesh(
      buildRibbonGeometry(worldTrackPoints, widthAt, 0, 0),
      new THREE.MeshStandardMaterial({
        color: 0x858a91,
        map: roadTexture,
        metalness: 0.02,
        roughness: 0.94,
      }),
    );
    const trackEdgeLineMaterial = new THREE.MeshBasicMaterial({
      color: 0xf7f7f4,
      toneMapped: false,
    });
    const leftTrackEdgeLine = new THREE.Mesh(
      buildClosedRibbonGeometry(
        offsetClosedPath(
          roadBoundaryEdges.leftEdge,
          -TRACK_EDGE_LINE_WIDTH_M / 2,
        ),
        TRACK_EDGE_LINE_WIDTH_M,
        TRACK_EDGE_LINE_HEIGHT_M,
      ),
      trackEdgeLineMaterial,
    );
    const rightTrackEdgeLine = new THREE.Mesh(
      buildClosedRibbonGeometry(
        offsetClosedPath(
          roadBoundaryEdges.rightEdge,
          TRACK_EDGE_LINE_WIDTH_M / 2,
        ),
        TRACK_EDGE_LINE_WIDTH_M,
        TRACK_EDGE_LINE_HEIGHT_M,
      ),
      trackEdgeLineMaterial,
    );
    root.add(shoulder, road, leftTrackEdgeLine, rightTrackEdgeLine);

    const runoffColors = {
      asphalt_runoff: 0x55555d,
      grass: 0x214d25,
      gravel: 0xa98f68,
    };
    stableSurfaceZones.forEach((zone) => {
      const sideSign = zone.side === 'right' ? -1 : 1;
      const kerbWidthM = Math.max(0.1, Number(zone.kerb_width_m || 1.2));
      const runoffWidthM = Math.max(0, Number(zone.runoff_width_m || 0));
      const boundaryPoints = sideSign > 0
        ? roadBoundaryEdges.leftEdge
        : roadBoundaryEdges.rightEdge;
      if (runoffWidthM > 0) {
        const runoffPoints = buildBoundaryOffsetSegmentPoints(
          boundaryPoints,
          Number(zone.start || 0),
          Number(zone.end || 0),
          sideSign * (kerbWidthM + runoffWidthM / 2),
          trackMetrics.totalLength,
        );
        root.add(new THREE.Mesh(
          buildOpenRibbonGeometry(
            runoffPoints,
            runoffWidthM,
            -0.012,
            { taperStartM: 6, taperEndM: 6 },
          ),
          new THREE.MeshStandardMaterial({
            color: runoffColors[zone.runoff_surface] ?? runoffColors.asphalt_runoff,
            roughness: 0.98,
          }),
        ));
      }
      const kerbPoints = buildBoundaryOffsetSegmentPoints(
        boundaryPoints,
        Number(zone.start || 0),
        Number(zone.end || 0),
        sideSign * kerbWidthM / 2,
        trackMetrics.totalLength,
      );
      root.add(new THREE.Mesh(
        buildOpenRibbonGeometry(
          kerbPoints,
          kerbWidthM,
          zone.kerb_height === 'high' ? 0.055 : 0.035,
        ),
        new THREE.MeshStandardMaterial({
          color: 0xffffff,
          map: kerbTexture,
          roughness: 0.86,
        }),
      ));
    });

    if (renderedPitPoints.length >= 2) {
      const pitShoulder = new THREE.Mesh(
        buildOpenRibbonGeometry(renderedPitPoints, pitLaneWidthM + 3.2, -0.012),
        new THREE.MeshStandardMaterial({ color: 0x62666c, roughness: 0.96 }),
      );
      const pitRoad = new THREE.Mesh(
        buildOpenRibbonGeometry(renderedPitPoints, pitLaneWidthM, 0.02),
        new THREE.MeshStandardMaterial({
          color: 0x747a82,
          map: roadTexture,
          roughness: 0.94,
        }),
      );
      const pitLineMaterial = new THREE.MeshBasicMaterial({ color: 0xf4f5f7 });
      const leftBoundary = new THREE.Mesh(
        buildOpenRibbonGeometry(offsetOpenPath(renderedPitPoints, pitLaneWidthM / 2), 0.18, 0.07),
        pitLineMaterial,
      );
      const rightBoundary = new THREE.Mesh(
        buildOpenRibbonGeometry(offsetOpenPath(renderedPitPoints, -pitLaneWidthM / 2), 0.18, 0.07),
        pitLineMaterial,
      );
      root.add(pitShoulder, pitRoad, leftBoundary, rightBoundary);
      root.add(
        createGate(pitMetrics, pitSideEntryProgress, pitLaneWidthM, 0xffffff, 0.08),
        createGate(pitMetrics, pitSpeedLimitStart, pitLaneWidthM, 0xe10600, 0.09),
        createGate(pitMetrics, pitBoxProgress, Math.min(2.4, pitLaneWidthM), 0xf5a623, 0.09),
        createGate(pitMetrics, pitSpeedLimitEnd, pitLaneWidthM, 0x29f28c, 0.09),
        createGate(pitMetrics, pitSideRejoinProgress, pitLaneWidthM, 0xffffff, 0.08),
      );

    }

    if (worldPitExitLanePoints.length >= 2) {
      const mergeTaperM = Math.min(
        55,
        Math.max(24, pathMetrics(worldPitExitLanePoints, false).totalLength * 0.4),
      );
      const exitShoulder = new THREE.Mesh(
        buildOpenRibbonGeometry(
          worldPitExitLanePoints,
          pitLaneWidthM + 1.8,
          -0.010,
          { taperEndM: mergeTaperM },
        ),
        new THREE.MeshStandardMaterial({ color: 0x62666c, roughness: 0.96 }),
      );
      const exitRoad = new THREE.Mesh(
        buildOpenRibbonGeometry(
          worldPitExitLanePoints,
          pitLaneWidthM,
          0.022,
          { taperEndM: mergeTaperM },
        ),
        new THREE.MeshStandardMaterial({
          color: 0x747a82,
          map: roadTexture,
          roughness: 0.94,
        }),
      );
      const exitLineMaterial = new THREE.MeshBasicMaterial({ color: 0xf4f5f7 });
      const exitLeftBoundary = new THREE.Mesh(
        buildOpenRibbonGeometry(
          offsetOpenPath(
            worldPitExitLanePoints,
            pitLaneWidthM / 2,
            { taperEndM: mergeTaperM },
          ),
          0.18,
          0.072,
          { taperEndM: mergeTaperM },
        ),
        exitLineMaterial,
      );
      const exitRightBoundary = new THREE.Mesh(
        buildOpenRibbonGeometry(
          offsetOpenPath(
            worldPitExitLanePoints,
            -pitLaneWidthM / 2,
            { taperEndM: mergeTaperM },
          ),
          0.18,
          0.072,
          { taperEndM: mergeTaperM },
        ),
        exitLineMaterial,
      );
      root.add(
        exitShoulder,
        exitRoad,
        exitLeftBoundary,
        exitRightBoundary,
      );
    }

    if (worldRacingLinePoints.length >= 2) {
      const racingGeometry = new THREE.BufferGeometry().setFromPoints(
        [...worldRacingLinePoints, worldRacingLinePoints[0]].map(
          ([x, y]) => new THREE.Vector3(x, 0.08, y),
        ),
      );
      root.add(new THREE.Line(
        racingGeometry,
        new THREE.LineBasicMaterial({ color: 0xaeb4bd, transparent: true, opacity: 0.58 }),
      ));
    }

    stableDrsZones.forEach((zone) => {
      const points = buildSegmentPoints(trackMetrics, zone.start, zone.end, 3);
      const drsMesh = new THREE.Mesh(
        buildOpenRibbonGeometry(points, 0.42, 0.10),
        new THREE.MeshBasicMaterial({ color: 0x29f28c, toneMapped: false }),
      );
      root.add(drsMesh);
    });

    const startProgress = startFinishIndex / Math.max(1, normalizedTrackCoords.length - 1);
    root.add(createGate(trackMetrics, startProgress, trackWidthM, 0xffffff));
    const sectorColors = [0x00d8ff, 0xffb000, 0xe455ff];
    stableSectors.slice(0, 3).forEach((sector, sectorIndex) => {
      const start = Number(sector.start ?? sectorIndex / 3);
      const end = Number(sector.end ?? (sectorIndex + 1) / 3);
      const miniCount = Math.max(1, Number(sector.mini_sector_count || 6));
      for (let miniIndex = 1; miniIndex < miniCount; miniIndex += 1) {
        const progress = start + (end - start) * miniIndex / miniCount;
        const widths = widthAt(progress);
        root.add(createGate(
          trackMetrics,
          progress,
          widths.left + widths.right,
          sectorColors[sectorIndex],
          0.072,
          0.24,
        ));
      }
      if (sectorIndex < 2) {
        const widths = widthAt(end);
        root.add(createGate(
          trackMetrics,
          end,
          widths.left + widths.right,
          sectorColors[sectorIndex + 1],
          0.078,
          0.92,
        ));
      }
    });

    stableGridSlots.forEach((slot) => {
      const pose = pathPoseAtProgress(trackMetrics, Number(slot.progress || 0));
      const lateralOffsetM = Number(slot.lateral_offset_m || 0);
      const x = pose.x - Math.sin(pose.heading) * lateralOffsetM;
      const y = pose.y + Math.cos(pose.heading) * lateralOffsetM;
      const gridBoxGeometry = new THREE.BoxGeometry(
        carLengthM + 1.2,
        0.03,
        carWidthM + 0.8,
      );
      const gridOutlineGeometry = new THREE.EdgesGeometry(gridBoxGeometry);
      gridBoxGeometry.dispose();
      const outline = new THREE.LineSegments(
        gridOutlineGeometry,
        new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.55 }),
      );
      outline.position.set(x, 0.055, y);
      outline.rotation.y = -pose.heading;
      root.add(outline);
    });

    const safetyCar = createSafetyCarModel(carWidthM * 1.08, carLengthM * 0.94);
    root.add(safetyCar);
    safetyCarRef.current = safetyCar;

    const updateProjection = () => {
      const width = Math.max(1, container.clientWidth);
      const height = Math.max(1, container.clientHeight);
      renderer.setSize(width, height, false);
      const aspect = width / height;
      const fitHeight = Math.max(boundsHeight, boundsWidth / aspect) * 1.12;
      const viewHeight = fitHeight / Math.max(1, zoomRef.current / 100);
      camera.left = -viewHeight * aspect / 2;
      camera.right = viewHeight * aspect / 2;
      camera.top = viewHeight / 2;
      camera.bottom = -viewHeight / 2;
      camera.updateProjectionMatrix();
    };
    cameraProjectionRef.current = updateProjection;
    updateProjection();

    const resizeObserver = new ResizeObserver(updateProjection);
    resizeObserver.observe(container);

    const frameSamples = [];
    let lastFrameAt = performance.now();
    let lastStatsAt = lastFrameAt;
    const projected = new THREE.Vector3();
    const worldPosition = new THREE.Vector3();
    const targetFocus = new THREE.Vector3();
    const footprintNear = new THREE.Vector3();
    const footprintFar = new THREE.Vector3();
    const footprintDirection = new THREE.Vector3();

    renderer.setAnimationLoop((now) => {
      const deltaSeconds = Math.min(0.25, Math.max(0.001, (now - lastFrameAt) / 1000));
      frameSamples.push(now - lastFrameAt);
      if (frameSamples.length > 120) frameSamples.shift();
      lastFrameAt = now;

      const liveTick = poseTickRef?.current;
      const playback = playbackRef.current;
      const livePhysicsFrame = Number(liveTick?.physics_frame);
      if (
        Number.isFinite(livePhysicsFrame)
        && livePhysicsFrame !== playback.lastTickPhysicsFrame
      ) {
        const clockWasInitialized = playback.latestSimulationTimeS > 0;
        if (livePhysicsFrame < playback.lastTickPhysicsFrame) {
          poseBuffersRef.current.clear();
          playback.renderSimulationTimeS = null;
        }
        const latestSimulationTimeS = appendPoseTickToBuffers(
          liveTick,
          poseBuffersRef.current,
        );
        playback.lastTickPhysicsFrame = livePhysicsFrame;
        if (latestSimulationTimeS > 0) {
          playback.latestSimulationTimeS = latestSimulationTimeS;
          playback.latestReceivedAtMs = now;
          if (!clockWasInitialized) {
            playback.renderSimulationTimeS = latestSimulationTimeS
              - (POSE_PLAYBACK_DELAY_MS / 1000)
                * Math.max(1, Number(liveTick?.speed_multiplier || 1));
          }
        }
        playback.speedMultiplier = Math.max(
          1,
          Number(liveTick?.speed_multiplier || speedMultiplierRef.current || 1),
        );
        playback.paused = Boolean(liveTick?.paused ?? pausedRef.current);
      }
      const elapsedSincePacketS = playback.paused
        ? 0
        : Math.max(0, now - playback.latestReceivedAtMs) / 1000;
      const estimatedLatestSimulationTimeS = playback.latestSimulationTimeS
        + elapsedSincePacketS * playback.speedMultiplier;
      const desiredRenderSimulationTimeS = estimatedLatestSimulationTimeS
        - (POSE_PLAYBACK_DELAY_MS / 1000) * playback.speedMultiplier;
      if (playback.renderSimulationTimeS === null || playback.paused) {
        playback.renderSimulationTimeS = desiredRenderSimulationTimeS;
      } else {
        const predictedSimulationTimeS = playback.renderSimulationTimeS
          + deltaSeconds * playback.speedMultiplier;
        const clockErrorS = desiredRenderSimulationTimeS - predictedSimulationTimeS;
        const correctionAlpha = 1 - Math.exp(-8 * deltaSeconds);
        const maximumCorrectionS = Math.max(
          0.001,
          deltaSeconds * playback.speedMultiplier * 0.35,
        );
        const correctionS = Math.min(
          maximumCorrectionS,
          Math.max(-maximumCorrectionS, clockErrorS * correctionAlpha),
        );
        playback.renderSimulationTimeS = Math.abs(clockErrorS) > 1.5
          ? desiredRenderSimulationTimeS
          : predictedSimulationTimeS + correctionS;
      }
      const renderSimulationTimeS = playback.renderSimulationTimeS;

      positionsRef.current.forEach((driver) => {
        const marker = carsRef.current.get(Number(driver.driver_id));
        if (!marker) return;
        const markerVisible = !driver.retired || Boolean(driver.hazard_active);
        marker.visible = markerVisible;
        const bufferedPose = bufferedWorldPoseAtTime(
          poseBuffersRef.current.get(Number(driver.driver_id)),
          renderSimulationTimeS,
        );
        const pose = bufferedPose || {
          xM: Number(driver.world_x_m || 0),
          yM: Number(driver.world_y_m || 0),
          headingRad: Number(driver.heading_rad || 0),
        };
        marker.position.set(pose.xM, 0.05, pose.yM);
        marker.rotation.y = -pose.headingRad;

        const [miniX, miniY] = rotate2D(pose.xM, pose.yM, displayRotation);
        const miniCircle = miniMapCirclesRef.current.get(Number(driver.driver_id));
        if (miniCircle) {
          miniCircle.style.display = markerVisible ? '' : 'none';
          if (markerVisible) {
            miniCircle.setAttribute('cx', String(miniX));
            miniCircle.setAttribute('cy', String(miniY));
          }
        }

      });

      const safetyTelemetry = safetyCarTelemetryRef.current;
      const safetyCarMarker = safetyCarRef.current;
      const showSafetyCar = safetyTelemetry.racePhase === 'sc'
        && safetyTelemetry.visible
        && safetyCarMarker;
      if (safetyCarMarker) safetyCarMarker.visible = Boolean(showSafetyCar);
      if (showSafetyCar) {
        const routeIsPit = safetyTelemetry.route === 'pit' && worldPitPoints.length >= 2;
        const routeMetrics = routeIsPit ? pitMetrics : trackMetrics;
        const elapsedSinceTelemetryS = playback.paused
          ? 0
          : Math.min(0.25, Math.max(0, now - safetyTelemetry.receivedAtMs) / 1000);
        const routeProgress = safetyCarProgressAtRenderTime({
          route: routeIsPit ? 'pit' : 'track',
          progress: safetyTelemetry.progress,
          progressRate: safetyTelemetry.progressRate,
          pitLaneProgress: safetyTelemetry.pitLaneProgress,
          pitLaneProgressRate: safetyTelemetry.pitLaneProgressRate,
          elapsedWallSeconds: elapsedSinceTelemetryS,
          speedMultiplier: playback.speedMultiplier,
        });
        const routeKey = routeIsPit ? 'pit' : 'track';
        const routeChanged = safetyCarMarker.userData.renderRoute !== null
          && safetyCarMarker.userData.renderRoute !== routeKey;
        const renderProgress = (
          !safetyCarMarker.userData.poseInitialized
          || routeChanged
          || !Number.isFinite(Number(safetyCarMarker.userData.renderProgress))
        )
          ? routeProgress
          : advanceSafetyCarRenderProgress({
            currentProgress: safetyCarMarker.userData.renderProgress,
            desiredProgress: routeProgress,
            route: routeKey,
            progressRate: safetyTelemetry.progressRate,
            pitLaneProgressRate: safetyTelemetry.pitLaneProgressRate,
            elapsedWallSeconds: deltaSeconds,
            speedMultiplier: playback.speedMultiplier,
          });
        safetyCarMarker.userData.renderProgress = renderProgress;
        safetyCarMarker.userData.renderRoute = routeKey;
        const safetyPose = pathPoseAtProgress(routeMetrics, renderProgress);
        const poseBlend = safetyCarMarker.userData.poseInitialized
          ? Math.min(1, 18 * deltaSeconds)
          : 1;
        safetyCarMarker.position.x += (safetyPose.x - safetyCarMarker.position.x) * poseBlend;
        safetyCarMarker.position.y = 0.05;
        safetyCarMarker.position.z += (safetyPose.y - safetyCarMarker.position.z) * poseBlend;
        safetyCarMarker.rotation.y = lerpAngle(
          safetyCarMarker.rotation.y,
          -safetyPose.heading,
          poseBlend,
        );
        safetyCarMarker.userData.poseInitialized = true;
        safetyCarMarker.userData.lightMaterial.emissiveIntensity = (
          Math.floor(now / 180) % 2 === 0 ? 4.8 : 0.8
        );

        const safetyLabel = safetyCarLabelRef.current;
        if (safetyLabel) {
          safetyCarMarker.getWorldPosition(projected);
          projected.y += 1.8;
          projected.project(camera);
          const inside = Math.abs(projected.x) <= 1.05 && Math.abs(projected.y) <= 1.05;
          safetyLabel.style.display = inside ? 'block' : 'none';
          if (inside) {
            safetyLabel.style.transform = `translate3d(${(projected.x * 0.5 + 0.5) * container.clientWidth}px, ${(-projected.y * 0.5 + 0.5) * container.clientHeight}px, 0)`;
          }
        }
        const safetyMiniMapMarker = safetyCarMiniMapRef.current;
        if (safetyMiniMapMarker) {
          const [miniX, miniY] = rotate2D(safetyPose.x, safetyPose.y, displayRotation);
          safetyMiniMapMarker.setAttribute('cx', String(miniX));
          safetyMiniMapMarker.setAttribute('cy', String(miniY));
        }
      } else {
        if (safetyCarLabelRef.current) safetyCarLabelRef.current.style.display = 'none';
        if (safetyCarMarker) {
          safetyCarMarker.userData.poseInitialized = false;
          safetyCarMarker.userData.renderProgress = null;
          safetyCarMarker.userData.renderRoute = null;
        }
      }

      const followed = carsRef.current.get(Number(followDriverIdRef.current));
      if (followed?.visible && !manualCameraRef.current) {
        followed.getWorldPosition(worldPosition);
        targetFocus.set(worldPosition.x, 0, worldPosition.z);
        const blend = 1 - Math.exp(-8 * Math.min(0.10, deltaSeconds));
        cameraFocusRef.current.lerp(targetFocus, blend);
      }
      const focus = cameraFocusRef.current;
      camera.position.set(focus.x, CAMERA_HEIGHT_M, focus.z + CAMERA_TRAILING_M);
      camera.lookAt(focus.x, 0, focus.z);
      camera.updateMatrixWorld();

      positionsRef.current.forEach((driver) => {
        const marker = carsRef.current.get(Number(driver.driver_id));
        const label = labelsRef.current.get(Number(driver.driver_id));
        if (!label) return;
        if (!marker || !marker.visible) {
          label.style.display = 'none';
          return;
        }
        marker.getWorldPosition(projected);
        projected.y += 1.5;
        projected.project(camera);
        const inside = Math.abs(projected.x) <= 1.05 && Math.abs(projected.y) <= 1.05;
        label.style.display = inside ? 'block' : 'none';
        if (inside) {
          label.style.transform = `translate3d(${(projected.x * 0.5 + 0.5) * container.clientWidth}px, ${(-projected.y * 0.5 + 0.5) * container.clientHeight}px, 0)`;
        }
      });

      if (miniMapViewportRef.current) {
        const footprint = [
          [-1, -1],
          [1, -1],
          [1, 1],
          [-1, 1],
        ].map(([ndcX, ndcY]) => {
          footprintNear.set(ndcX, ndcY, -1).unproject(camera);
          footprintFar.set(ndcX, ndcY, 1).unproject(camera);
          footprintDirection.copy(footprintFar).sub(footprintNear);
          const distance = Math.abs(footprintDirection.y) > 1e-9
            ? -footprintNear.y / footprintDirection.y
            : 0;
          return [
            footprintNear.x + footprintDirection.x * distance,
            footprintNear.z + footprintDirection.z * distance,
          ];
        });
        miniMapViewportRef.current.setAttribute(
          'points',
          footprint.map((point) => point.join(',')).join(' '),
        );
      }
      renderer.render(scene, camera);

      if (now - lastStatsAt >= 1000) {
        const averageFrameMs = frameSamples.reduce((sum, value) => sum + value, 0)
          / Math.max(1, frameSamples.length);
        const orderedFrameSamples = [...frameSamples].sort((a, b) => a - b);
        const p95Index = Math.max(
          0,
          Math.min(
            orderedFrameSamples.length - 1,
            Math.ceil(orderedFrameSamples.length * 0.95) - 1,
          ),
        );
        const frameP95Ms = orderedFrameSamples[p95Index] || 0;
        const slowFrames = frameSamples.filter((sample) => sample > (1000 / 30)).length;
        const usedJsHeapBytes = Number(performance.memory?.usedJSHeapSize);
        const jsHeapMb = Number.isFinite(usedJsHeapBytes)
          ? usedJsHeapBytes / (1024 * 1024)
          : null;
        let jsHeapPeakMb = null;
        let jsHeapTrendMbPerMin = null;
        if (jsHeapMb !== null) {
          const memorySamples = memorySamplesRef.current;
          memorySamples.push({ atMs: now, jsHeapMb });
          while (
            memorySamples.length > 1
            && (
              memorySamples.length > MAX_MEMORY_SAMPLES
              || memorySamples[0].atMs < now - MEMORY_SAMPLE_RETENTION_MS
            )
          ) {
            memorySamples.shift();
          }
          jsHeapPeakMb = memorySamples.reduce(
            (peak, sample) => Math.max(peak, sample.jsHeapMb),
            jsHeapMb,
          );
          const firstMemorySample = memorySamples[0];
          const elapsedMinutes = (
            now - firstMemorySample.atMs
          ) / 60_000;
          if (elapsedMinutes >= 0.5) {
            jsHeapTrendMbPerMin = (
              jsHeapMb - firstMemorySample.jsHeapMb
            ) / elapsedMinutes;
          }
        }
        if (!destroyed) {
          const nextPerformanceStats = {
            fps: Math.round(1000 / Math.max(1, averageFrameMs)),
            frameP95Ms,
            slowFrames,
            calls: renderer.info.render.calls,
            triangles: renderer.info.render.triangles,
            geometries: renderer.info.memory.geometries,
            textures: renderer.info.memory.textures,
            jsHeapMb,
            jsHeapPeakMb,
            jsHeapTrendMbPerMin,
            domNodes: document.getElementsByTagName('*').length,
            poseSamples: [...poseBuffersRef.current.values()].reduce(
              (total, buffer) => total + buffer.count,
              0,
            ),
            sceneBuilds: sceneBuildCountRef.current,
            garageBuilds: garageBuildCountRef.current,
            carModelsBuilt: carModelBuildCountRef.current,
          };
          setPerformanceStats(nextPerformanceStats);
          onPerformanceStats?.(nextPerformanceStats);
        }
        lastStatsAt = now;
      }
    });

    setSceneReady(true);
    return () => {
      destroyed = true;
      setSceneReady(false);
      resizeObserver.disconnect();
      renderer.setAnimationLoop(null);
      disposeObject3D(scene);
      roadTexture.dispose();
      kerbTexture.dispose();
      scene.clear();
      renderer.renderLists.dispose();
      renderer.dispose();
      renderer.forceContextLoss();
      renderer.domElement.width = 1;
      renderer.domElement.height = 1;
      renderer.domElement.remove();
      cars.clear();
      garageGroupRef.current = null;
      labels.clear();
      miniMapCircles.clear();
      poseBuffers.clear();
      memorySamplesRef.current = [];
      safetyCarRef.current = null;
      rendererRef.current = null;
      sceneRef.current = null;
      cameraRef.current = null;
      rootRef.current = null;
      cameraProjectionRef.current = () => {};
      onPerformanceStats?.(null);
      onRendererDisposed?.({
        renderer_lifecycle_id: rendererLifecycleId,
        animation_loop_stopped: true,
        pose_buffers_cleared: true,
        scene_refs_cleared: true,
        renderer_ref_cleared: true,
        websocket_state: 'closed',
        websocket_count: 0,
        canvas_count: document.querySelectorAll('canvas').length,
        webgl_context_count: 'not_observable',
      });
    };
  }, [
    carLengthM,
    carWidthM,
    displayRotation,
    stableDrsZones,
    stableGridSlots,
    normalizedTrackCoords.length,
    onPerformanceStats,
    onRendererDisposed,
    pitLaneWidthM,
    pitMetrics,
    pitBoxProgress,
    pitSideEntryProgress,
    pitSideRejoinProgress,
    pitSpeedLimitEnd,
    pitSpeedLimitStart,
    poseTickRef,
    renderedPitPoints,
    stableSectors,
    stableSurfaceZones,
    startFinishIndex,
    trackMetrics,
    trackWidthM,
    stableTrackWidthProfile,
    worldPitPoints,
    worldPitExitLanePoints,
    worldRacingLinePoints,
    worldTrackPoints,
  ]);

  useEffect(() => {
    const root = rootRef.current;
    if (
      !sceneReady
      || !root
      || worldPitPoints.length < 2
      || !stableGarageTeams.length
    ) {
      return undefined;
    }

    const garageGroup = new THREE.Group();
    garageBuildCountRef.current += 1;
    const garageStart = Math.max(
      Number(pitSpeedLimitStart) + 0.045,
      Number(pitBoxProgress) - 0.24,
    );
    const garageEnd = Math.min(
      Number(pitSpeedLimitEnd) - 0.045,
      Number(pitBoxProgress) + 0.24,
    );
    const garagePlacements = stableGarageTeams.map((team, index) => {
      const ratio = stableGarageTeams.length === 1
        ? 0.5
        : index / (stableGarageTeams.length - 1);
      const fallbackProgress = garageStart + (garageEnd - garageStart) * ratio;
      const progress = Number.isFinite(team.pitBoxProgress)
        ? team.pitBoxProgress
        : fallbackProgress;
      const pose = pathPoseAtProgress(pitMetrics, progress);
      return { team, progress, pose };
    });
    garagePlacements.forEach(({ team, pose }, index) => {
      const previous = garagePlacements[index - 1];
      const next = garagePlacements[index + 1];
      const previousGapM = previous
        ? Math.hypot(
          pose.x - previous.pose.x,
          pose.y - previous.pose.y,
        )
        : null;
      const nextGapM = next
        ? Math.hypot(
          next.pose.x - pose.x,
          next.pose.y - pose.y,
        )
        : null;
      const garageSpanM = Math.min(
        48,
        Math.max(12, previousGapM ?? nextGapM ?? 16, nextGapM ?? previousGapM ?? 16),
      );
      const garage = createPitGarageBay(
        team,
        carLengthM,
        pitBoxOffset,
        pitLaneWidthM,
        garageSpanM,
        index === 0,
        index === garagePlacements.length - 1,
      );
      garage.position.set(pose.x, 0.025, pose.y);
      garage.rotation.y = -pose.heading;
      garageGroup.add(garage);
    });
    root.add(garageGroup);
    garageGroupRef.current = garageGroup;

    return () => {
      garageGroup.parent?.remove(garageGroup);
      disposeObject3D(garageGroup);
      if (garageGroupRef.current === garageGroup) {
        garageGroupRef.current = null;
      }
    };
  }, [
    carLengthM,
    pitBoxOffset,
    pitBoxProgress,
    pitLaneWidthM,
    pitMetrics,
    pitSpeedLimitEnd,
    pitSpeedLimitStart,
    sceneReady,
    stableGarageTeams,
    worldPitPoints.length,
  ]);

  useEffect(() => {
    if (!sceneReady || !rootRef.current) return;
    const visiblePositions = (positions || []).filter(
      (driver) => !driver.retired || Boolean(driver.hazard_active),
    );
    const activeIds = new Set(
      visiblePositions.map((driver) => Number(driver.driver_id)),
    );
    carsRef.current.forEach((car, driverId) => {
      if (activeIds.has(driverId)) return;
      rootRef.current.remove(car);
      disposeObject3D(car);
      carsRef.current.delete(driverId);
      poseBuffersRef.current.delete(driverId);
    });
    visiblePositions.forEach((driver) => {
      const driverId = Number(driver.driver_id);
      let car = carsRef.current.get(driverId);
      if (!car) {
        car = createCarModel(driver, carWidthM, carLengthM);
        carModelBuildCountRef.current += 1;
        rootRef.current.add(car);
        carsRef.current.set(driverId, car);
      }
      const color = colorNumber(driver.team_color);
      car.userData.bodyMaterial?.color.setHex(color);
      car.userData.accentMaterial?.color.setHex(color).offsetHSL(0, 0, 0.16);
    });
  }, [carLengthM, carWidthM, positions, sceneReady]);

  const resetView = useCallback(() => {
    setZoomPercent(DEFAULT_ZOOM_PERCENT);
    followDriver(playerDrivers[0]?.driver_id ?? null);
  }, [followDriver, playerDrivers]);

  return (
    <div className="track-canvas track-canvas--three glass-panel">
      <div className="track-canvas__header">
        <span>TRACK VIEW · THREE.JS</span>
        <div className="track-canvas__tools">
          {playerDrivers.length > 0 && (
            <div className="track-canvas__follow" aria-label="Follow player driver">
              {playerDrivers.map((driver) => (
                <button
                  key={driver.driver_id}
                  type="button"
                  className={`track-canvas__follow-btn ${cameraMode === 'follow' && followDriverId === driver.driver_id ? 'track-canvas__follow-btn--active' : ''}`}
                  onClick={() => followDriver(driver.driver_id)}
                  aria-pressed={cameraMode === 'follow' && followDriverId === driver.driver_id}
                >
                  {driver.name}
                </button>
              ))}
            </div>
          )}
          {followedDriver && followedDriver.source_mode !== 'abstract' && (
            <div className="track-canvas__speed-hud">
              <span className="track-canvas__speed-driver">{followedDriver.name}</span>
              <span className="track-canvas__speed-value">
                {Math.round(Number(followedDriver.speed_kph || 0))} KM/H
              </span>
            </div>
          )}
          {cameraMode === 'manual' && (
            <span className="three-track__camera-mode">FREE VIEW</span>
          )}
          <div className="track-canvas__zoom" aria-label="Three track zoom">
            {ZOOM_LEVELS.map((level) => (
              <button
                key={level}
                type="button"
                className={`track-canvas__zoom-btn ${zoomPercent === level ? 'track-canvas__zoom-btn--active' : ''}`}
                onClick={() => setZoomPercent(level)}
                aria-pressed={zoomPercent === level}
              >
                {level}%
              </button>
            ))}
            <button type="button" className="track-canvas__reset-btn" onClick={resetView}>
              RESET
            </button>
          </div>
          <span
            className="three-track__performance"
            data-testid="three-performance"
            data-fps={performanceStats.fps}
            data-frame-p95-ms={performanceStats.frameP95Ms.toFixed(2)}
            data-slow-frames={performanceStats.slowFrames}
            data-geometries={performanceStats.geometries}
            data-textures={performanceStats.textures}
            data-js-heap-mb={performanceStats.jsHeapMb?.toFixed(1) || ''}
            data-js-heap-peak-mb={performanceStats.jsHeapPeakMb?.toFixed(1) || ''}
            data-js-heap-trend-mb-min={
              performanceStats.jsHeapTrendMbPerMin?.toFixed(2) || ''
            }
            data-dom-nodes={performanceStats.domNodes}
            data-pose-samples={performanceStats.poseSamples}
            data-scene-builds={performanceStats.sceneBuilds}
            data-garage-builds={performanceStats.garageBuilds}
            data-car-models-built={performanceStats.carModelsBuilt}
          >
            {performanceStats.fps} FPS · P95 {performanceStats.frameP95Ms.toFixed(1)}MS
            {' · '}{performanceStats.calls} CALLS · {performanceStats.triangles} TRI
            {' · '}BUF {performanceStats.poseSamples}
            {performanceStats.jsHeapMb !== null
              ? ` · HEAP ${performanceStats.jsHeapMb.toFixed(0)}MB`
              : ''}
            {performanceStats.jsHeapTrendMbPerMin !== null
              ? ` (${performanceStats.jsHeapTrendMbPerMin >= 0 ? '+' : ''}${performanceStats.jsHeapTrendMbPerMin.toFixed(1)}/MIN)`
              : ''}
            {' · '}DOM {performanceStats.domNodes}
          </span>
        </div>
      </div>
      <div className="track-canvas__viewport" ref={containerRef}>
        <div className="three-track__labels" aria-hidden="true">
          {(positions || [])
            .filter((driver) => !driver.retired || Boolean(driver.hazard_active))
            .map((driver) => {
            const pitTiming = formatPitTiming(driver);
            return (
              <span
                key={driver.driver_id}
                ref={(node) => {
                  if (node) labelsRef.current.set(Number(driver.driver_id), node);
                  else labelsRef.current.delete(Number(driver.driver_id));
                }}
                className={[
                  'three-track__label',
                  playerDriverIds.includes(driver.driver_id)
                    ? 'three-track__label--player'
                    : '',
                  pitTiming ? `three-track__label--pit three-track__label--pit-${pitTiming.phase}` : '',
                ].filter(Boolean).join(' ')}
                style={{ '--team-color': driver.team_color }}
                data-testid={pitTiming ? `pit-timing-${driver.driver_id}` : undefined}
              >
                <span className="three-track__driver-label">
                  P{driver.position} {driver.name}
                </span>
                {pitTiming && (
                  <span className="three-track__pit-timing">
                    <strong>{pitTiming.primary}</strong>
                    <span>{pitTiming.secondary}</span>
                  </span>
                )}
              </span>
            );
          })}
          <span
            ref={safetyCarLabelRef}
            className="three-track__label three-track__safety-car-label"
          >
            SC · SAFETY CAR
          </span>
        </div>
        {miniMap && (
          <div className="track-canvas__minimap" aria-label="100 percent circuit overview">
            <span className="track-canvas__minimap-label">100% OVERVIEW · CLICK TO MOVE</span>
            <svg
              ref={miniMapSvgRef}
              viewBox={miniMap.viewBox}
              preserveAspectRatio="xMidYMid meet"
              role="img"
              aria-label="Track overview and current camera area"
              onClick={handleMiniMapClick}
            >
              <polyline
                points={miniMap.trackPoints}
                fill="none"
                stroke="rgba(255,255,255,0.42)"
                strokeWidth={miniMap.strokeWidth}
                strokeLinejoin="round"
              />
              {miniMap.pitPoints && (
                <polyline
                  points={miniMap.pitPoints}
                  fill="none"
                  stroke="#f5a623"
                  strokeWidth={miniMap.strokeWidth * 0.6}
                />
              )}
              <polygon
                ref={miniMapViewportRef}
                className="three-track__minimap-viewport"
                points=""
              />
              {(positions || [])
                .filter((driver) => !driver.retired || Boolean(driver.hazard_active))
                .map((driver) => (
                <circle
                  key={driver.driver_id}
                  ref={(node) => {
                    if (node) miniMapCirclesRef.current.set(Number(driver.driver_id), node);
                    else miniMapCirclesRef.current.delete(Number(driver.driver_id));
                  }}
                  cx="0"
                  cy="0"
                  r={miniMap.markerRadius * (playerDriverIds.includes(driver.driver_id) ? 1.45 : 1)}
                  fill={driver.retired ? '#ff1744' : driver.team_color}
                  stroke={playerDriverIds.includes(driver.driver_id) ? '#fff' : 'rgba(255,255,255,0.55)'}
                  strokeWidth={miniMap.markerRadius * 0.28}
                />
                ))}
              {racePhase === 'sc' && safetyCarVisible && (
                <circle
                  ref={safetyCarMiniMapRef}
                  className="three-track__safety-car-minimap"
                  cx="0"
                  cy="0"
                  r={miniMap.markerRadius * 1.55}
                  fill="#f7d038"
                  stroke="#ffffff"
                  strokeWidth={miniMap.markerRadius * 0.34}
                />
              )}
            </svg>
          </div>
        )}
        {showBearing && (
          <div className="track-canvas__bearing">
            <span className="track-canvas__bearing-text">
              <span className="track-canvas__bearing-label">N</span>
              <span className="track-canvas__bearing-value">
                {String(Math.round(displayRotation)).padStart(3, '0')}°
              </span>
            </span>
          </div>
        )}
      </div>
    </div>
  );
}
