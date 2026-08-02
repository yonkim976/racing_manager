function finiteNumber(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

/**
 * Extrapolate the Safety Car along the route between dashboard packets.
 *
 * Track progress rates are expressed per simulation second.  Pit-lane rates
 * are measured from successive dashboard packets and therefore already use
 * wall-clock seconds; scaling those a second time would overrun the pit lane
 * at 2x.
 */
export function safetyCarProgressAtRenderTime({
  route = 'track',
  progress = 0,
  progressRate = 0,
  pitLaneProgress = 0,
  pitLaneProgressRate = 0,
  elapsedWallSeconds = 0,
  speedMultiplier = 1,
}) {
  const elapsed = Math.max(0, finiteNumber(elapsedWallSeconds));
  if (route === 'pit') {
    return Math.min(
      1,
      Math.max(
        0,
        finiteNumber(pitLaneProgress)
          + finiteNumber(pitLaneProgressRate) * elapsed,
      ),
    );
  }

  const simulationElapsed = elapsed * Math.max(1, finiteNumber(speedMultiplier, 1));
  const rawProgress = finiteNumber(progress)
    + finiteNumber(progressRate) * simulationElapsed;
  return ((rawProgress % 1) + 1) % 1;
}

/**
 * Limit visual correction to the current Safety Car velocity.  A delayed or
 * jittery dashboard packet must not teleport the marker to its new progress.
 */
export function advanceSafetyCarRenderProgress({
  currentProgress,
  desiredProgress,
  route = 'track',
  progressRate = 0,
  pitLaneProgressRate = 0,
  elapsedWallSeconds = 0,
  speedMultiplier = 1,
}) {
  const current = finiteNumber(currentProgress, desiredProgress);
  const desired = finiteNumber(desiredProgress, current);
  const elapsed = Math.max(0, finiteNumber(elapsedWallSeconds));
  const rate = route === 'pit'
    ? Math.abs(finiteNumber(pitLaneProgressRate))
    : Math.abs(finiteNumber(progressRate)) * Math.max(1, finiteNumber(speedMultiplier, 1));
  const maximumStep = route === 'pit'
    ? rate * elapsed * 1.6
    : Math.max(0.00002, rate * elapsed * 1.6);
  const rawDelta = route === 'pit'
    ? desired - current
    : ((desired - current + 0.5) % 1 + 1) % 1 - 0.5;
  const delta = Math.min(maximumStep, Math.max(-maximumStep, rawDelta));
  const next = current + delta;
  return route === 'pit'
    ? Math.min(1, Math.max(0, next))
    : ((next % 1) + 1) % 1;
}
