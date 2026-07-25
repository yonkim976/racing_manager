const DEFAULT_RETENTION_SIM_SECONDS = 2.5;
const BINARY_POSE_HEADER_BYTES = 11;
const BINARY_DRIVER_HEADER_BYTES = 4;
const BINARY_POSE_SAMPLE_BYTES = 20;
const POSE_RING_CAPACITY = 160;

function shortestAngleDelta(from, to) {
  let delta = to - from;
  while (delta > Math.PI) delta -= Math.PI * 2;
  while (delta < -Math.PI) delta += Math.PI * 2;
  return delta;
}

function createPoseBuffer() {
  return {
    capacity: POSE_RING_CAPACITY,
    head: 0,
    count: 0,
    physicsFrames: new Uint32Array(POSE_RING_CAPACITY),
    simulationTimes: new Float64Array(POSE_RING_CAPACITY),
    xValues: new Float32Array(POSE_RING_CAPACITY),
    yValues: new Float32Array(POSE_RING_CAPACITY),
    headings: new Float32Array(POSE_RING_CAPACITY),
    lastPhysicsFrame: -1,
    interpolatedPose: { xM: 0, yM: 0, headingRad: 0 },
  };
}

function ringIndex(buffer, logicalIndex) {
  return (buffer.head + logicalIndex) % buffer.capacity;
}

function appendPoseSample(
  buffer,
  physicsFrame,
  simulationTimeS,
  xM,
  yM,
  headingRad,
) {
  if (physicsFrame <= buffer.lastPhysicsFrame) return false;
  let index;
  if (buffer.count < buffer.capacity) {
    index = ringIndex(buffer, buffer.count);
    buffer.count += 1;
  } else {
    index = buffer.head;
    buffer.head = (buffer.head + 1) % buffer.capacity;
  }
  buffer.physicsFrames[index] = physicsFrame;
  buffer.simulationTimes[index] = simulationTimeS;
  buffer.xValues[index] = xM;
  buffer.yValues[index] = yM;
  buffer.headings[index] = headingRad;
  buffer.lastPhysicsFrame = physicsFrame;
  return true;
}

function prunePoseBuffer(buffer, cutoffSimulationTimeS) {
  while (
    buffer.count > 2
    && buffer.simulationTimes[ringIndex(buffer, 1)] < cutoffSimulationTimeS
  ) {
    buffer.head = (buffer.head + 1) % buffer.capacity;
    buffer.count -= 1;
  }
}

export function bufferedWorldPoseAtTime(buffer, targetSimulationTimeS) {
  if (!buffer?.count) return null;
  const output = buffer.interpolatedPose;
  const firstIndex = buffer.head;
  const lastIndex = ringIndex(buffer, buffer.count - 1);
  if (
    buffer.count === 1
    || targetSimulationTimeS <= buffer.simulationTimes[firstIndex]
  ) {
    output.xM = buffer.xValues[firstIndex];
    output.yM = buffer.yValues[firstIndex];
    output.headingRad = buffer.headings[firstIndex];
    return output;
  }
  if (targetSimulationTimeS >= buffer.simulationTimes[lastIndex]) {
    output.xM = buffer.xValues[lastIndex];
    output.yM = buffer.yValues[lastIndex];
    output.headingRad = buffer.headings[lastIndex];
    return output;
  }

  let low = 0;
  let high = buffer.count - 1;
  while (low + 1 < high) {
    const middle = Math.floor((low + high) / 2);
    const middleTime = buffer.simulationTimes[ringIndex(buffer, middle)];
    if (middleTime <= targetSimulationTimeS) low = middle;
    else high = middle;
  }
  const fromIndex = ringIndex(buffer, low);
  const toIndex = ringIndex(buffer, high);
  const fromTime = buffer.simulationTimes[fromIndex];
  const toTime = buffer.simulationTimes[toIndex];
  const fromX = buffer.xValues[fromIndex];
  const fromY = buffer.yValues[fromIndex];
  const fromHeading = buffer.headings[fromIndex];
  const toX = buffer.xValues[toIndex];
  const toY = buffer.yValues[toIndex];
  const toHeading = buffer.headings[toIndex];
  const duration = Math.max(1e-9, toTime - fromTime);
  const segmentT = Math.min(
    1,
    Math.max(0, (targetSimulationTimeS - fromTime) / duration),
  );
  const deltaX = toX - fromX;
  const deltaY = toY - fromY;
  const chordLength = Math.hypot(deltaX, deltaY);
  let xM = fromX + deltaX * segmentT;
  let yM = fromY + deltaY * segmentT;
  if (chordLength > 1e-6) {
    const chordHeading = Math.atan2(deltaY, deltaX);
    const fromAlignment = Math.max(
      0,
      Math.cos(shortestAngleDelta(fromHeading, chordHeading)),
    );
    const toAlignment = Math.max(
      0,
      Math.cos(shortestAngleDelta(toHeading, chordHeading)),
    );
    const tangentLength = chordLength * (
      0.55 + 0.45 * Math.min(fromAlignment, toAlignment)
    );
    const t2 = segmentT * segmentT;
    const t3 = t2 * segmentT;
    const h00 = 2 * t3 - 3 * t2 + 1;
    const h10 = t3 - 2 * t2 + segmentT;
    const h01 = -2 * t3 + 3 * t2;
    const h11 = t3 - t2;
    xM = h00 * fromX
      + h10 * Math.cos(fromHeading) * tangentLength
      + h01 * toX
      + h11 * Math.cos(toHeading) * tangentLength;
    yM = h00 * fromY
      + h10 * Math.sin(fromHeading) * tangentLength
      + h01 * toY
      + h11 * Math.sin(toHeading) * tangentLength;
  }
  output.xM = xM;
  output.yM = yM;
  output.headingRad = fromHeading + shortestAngleDelta(
    fromHeading,
    toHeading,
  ) * segmentT;
  return output;
}

export function appendPoseTickToBuffers(
  tick,
  buffers,
  retentionSimSeconds = DEFAULT_RETENTION_SIM_SECONDS,
) {
  if (tick?.pose_buffer instanceof ArrayBuffer) {
    const view = new DataView(tick.pose_buffer);
    if (view.byteLength < BINARY_POSE_HEADER_BYTES) return 0;
    const driverCount = view.getUint8(10);
    let offset = BINARY_POSE_HEADER_BYTES;
    let latestSimulationTimeS = 0;
    for (let driverIndex = 0; driverIndex < driverCount; driverIndex += 1) {
      if (offset + BINARY_DRIVER_HEADER_BYTES > view.byteLength) break;
      const driverId = view.getUint16(offset, true);
      const flags = view.getUint8(offset + 2);
      const sampleCount = view.getUint8(offset + 3);
      offset += BINARY_DRIVER_HEADER_BYTES;
      const retired = Boolean(flags & 1);
      const hazardActive = Boolean(flags & 2);
      if (retired && !hazardActive) {
        buffers.delete(driverId);
        offset += Math.min(
          sampleCount * BINARY_POSE_SAMPLE_BYTES,
          Math.max(0, view.byteLength - offset),
        );
        continue;
      }
      let buffer = buffers.get(driverId);
      if (!buffer) {
        buffer = createPoseBuffer();
        buffers.set(driverId, buffer);
      }
      for (let sampleIndex = 0; sampleIndex < sampleCount; sampleIndex += 1) {
        if (offset + BINARY_POSE_SAMPLE_BYTES > view.byteLength) break;
        const simulationTimeS = view.getFloat32(offset, true);
        const physicsFrame = view.getUint32(offset + 4, true);
        const xM = view.getFloat32(offset + 8, true);
        const yM = view.getFloat32(offset + 12, true);
        const headingRad = view.getFloat32(offset + 16, true);
        offset += BINARY_POSE_SAMPLE_BYTES;
        const appended = appendPoseSample(
          buffer,
          physicsFrame,
          simulationTimeS,
          xM,
          yM,
          headingRad,
        );
        if (appended) {
          latestSimulationTimeS = Math.max(latestSimulationTimeS, simulationTimeS);
        }
      }
      const cutoff = latestSimulationTimeS - retentionSimSeconds;
      prunePoseBuffer(buffer, cutoff);
    }
    return latestSimulationTimeS;
  }

  let latestSimulationTimeS = 0;
  const compactPoses = tick?.poses;
  const drivers = compactPoses || tick?.positions || [];
  drivers.forEach((driver) => {
    const compact = Array.isArray(driver);
    const driverId = Number(compact ? driver[0] : driver.driver_id);
    const retired = Boolean(compact ? driver[1] : driver.retired);
    const hazardActive = Boolean(compact ? driver[2] : driver.hazard_active);
    if (retired && !hazardActive) {
      buffers.delete(driverId);
      return;
    }
    let buffer = buffers.get(driverId);
    if (!buffer) {
      buffer = createPoseBuffer();
      buffers.set(driverId, buffer);
    }
    const sourceSamples = compact
      ? (driver[3] || [])
      : (driver.trajectory_samples || []);
    if (!sourceSamples.length) {
      sourceSamples.push({
        simulation_time_s: driver.simulation_time_s,
        physics_frame: driver.physics_frame,
        world_x_m: driver.world_x_m,
        world_y_m: driver.world_y_m,
        heading_rad: driver.heading_rad,
      });
    }
    sourceSamples.forEach((sample) => {
      const compactSample = Array.isArray(sample);
      const simulationTimeS = Number(
        compactSample ? sample[0] : sample.simulation_time_s,
      );
      const physicsFrame = Number(
        compactSample ? sample[1] : sample.physics_frame,
      );
      const xM = Number(compactSample ? sample[2] : sample.world_x_m);
      const yM = Number(compactSample ? sample[3] : sample.world_y_m);
      const headingRad = Number(compactSample ? sample[4] : sample.heading_rad);
      if (
        !Number.isFinite(physicsFrame)
        || !Number.isFinite(simulationTimeS)
        || !Number.isFinite(xM)
        || !Number.isFinite(yM)
        || !Number.isFinite(headingRad)
      ) return;
      const appended = appendPoseSample(
        buffer,
        physicsFrame,
        simulationTimeS,
        xM,
        yM,
        headingRad,
      );
      if (appended) {
        latestSimulationTimeS = Math.max(latestSimulationTimeS, simulationTimeS);
      }
    });
    const cutoff = latestSimulationTimeS - retentionSimSeconds;
    prunePoseBuffer(buffer, cutoff);
  });
  return latestSimulationTimeS;
}
