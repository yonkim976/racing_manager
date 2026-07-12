# Physics V2 design and rollout

## Goal

Move vehicle motion from lap-time-driven progress to fixed-step longitudinal
physics while preserving the existing race strategy, pit, battle and Safety Car
systems during migration.

## Update rates

| Layer | Rate | Responsibility |
|---|---:|---|
| Vehicle longitudinal physics | 50 Hz (`0.02s`) | Target speed, throttle, brake, acceleration, distance |
| Race engine | 10 Hz (`0.1s`) | Laps, positions, incidents, pit stops, race control |
| Strategy AI | 2–10 Hz | Pace and pit decisions using existing cooldowns |
| WebSocket broadcast | 5 Hz (`0.2s`) | Authoritative snapshots |
| Browser track animation | display refresh | Interpolation and short prediction between snapshots |

Game speed changes the amount of simulated time, not the fixed physics step.
For example, one `0.1s` race-engine update contains five `0.02s` Physics V2
steps per active car.

## Implemented in phase 1

- Physics V2 is the only race engine; the Legacy runtime path, development
  switch, command and telemetry mode field have been removed.
- Curvature-derived `raw_speeds_mps` is used directly as the local speed limit.
- The raw profile is calibrated once against the circuit reference lap time.
- Car performance, driver pace, pace mode, tire wear, dirty air, DRS, battle
  effects and SC/VSC limits modify physical power, grip or target speed.
- Speed and distance are integrated with deterministic 50 Hz substeps.
- Lap times remain recorded only when the physically advanced car crosses the
  start/finish line.
- Tick payloads expose the fixed `50Hz` rate, acceleration, target speed,
  throttle and brake.

## Implemented in phase 2

- Each follower receives the leading car's start/end trajectory and applies
  distance control during every 50 Hz substep.
- The green-flag following target varies with speed from `10–28m`; when the
  physical car bodies overlap laterally, the hard longitudinal minimum is one
  `5.0m` car length.
- VSC preserves the gap at deployment and prohibits on-track passing.
- Full SC uses a `35m` queue target and prohibits passing except
  for explicitly authorized unlapping cars.
- Existing attack, forced-wide and side-by-side battle states assign virtual
  racing, defensive, inside or outside lines. Longitudinal passing is allowed
  only when race control and those line states permit it.
- A post-step adjacency solver handles a new neighbor created by a completed
  pass, preventing same-line overlap at the order boundary.
- The active virtual line is included in driver telemetry.

## Implemented in phase 3

- The centerline is the track's metric reference, not the only drivable line.
- Track width defaults to `12m` and can use per-point left/right metric widths.
- Cars have physical `1.9m × 5.0m` bodies, lateral position, lateral velocity
  and a bounded target-line controller running at the 50 Hz physics rate.
- A smooth entry/outside, apex/inside, exit/outside racing line is generated
  automatically from centerline curvature and clamped inside track boundaries.
- Attack, defense and side-by-side states now request metric lateral targets.
  Passing requires at least `2.15m` centre separation; longitudinal overlap is
  prevented whenever the physical body footprints overlap laterally.
- Race telemetry carries physical dimensions and lateral motion. Canvas zoom is
  available in five large steps (`100/250/500/1500/2000%`); the two highest
  steps follow the player and draw track width, a densely sampled metric racing
  line and the `1.9m × 5.0m` vehicle at the same scale. Camera following moves
  the cached Pixi world instead of rebuilding the complete track every frame.

## Deliberately retained during migration

- Pit-lane phases remain on the 10 Hz race engine.
- Incidents remain event-driven.
- Existing battle and order rules remain authoritative.
- Safety Car lifecycle and queue state machine are unchanged; Physics V2 consumes
  their speed limits.

## Next phase

1. Move pit-lane driving from timed phases to physical distance and speed.
2. Add variable-width telemetry to Canvas instead of rendering the circuit base
   width uniformly.
3. Benchmark 1x, 2x, 5x and full-length races.

## Phase 1 acceptance evidence

- A 65-second reference circuit produced first-lap times of approximately
  `67.6–74.1s` for a 20-car field from the standing grid.
- Observed physical speeds were approximately `98–336 km/h`.
- Simulating `74.2s` of race time, including about 3,710 physical substeps per
  car, completed in about `0.43s` on the development machine.
- Backend unit tests, frontend lint/build and live browser mode switching passed.

## Phase 2 acceptance evidence

- A three-lap 20-car run produced physical passes with no gap below `5.0m`
  whenever two cars' lateral body footprints overlapped.
- Physics V2 completed the complete Safety Car lifecycle with the on-track order
  unchanged and a minimum observed queue gap of `5.0m`.
- Backend unit tests increased to 157 and all pass.

## Pause/resume stability

- The boolean `paused` state and command methods use distinct names
  (`pause_race` / `resume_race`) so instance state cannot shadow a method.
- Pause and resume commands immediately broadcast an authoritative tick.
- The browser stops marker prediction optimistically on Pause and holds the
  rendered marker position through the first resumed frame.
- Command failures return an error without terminating the race WebSocket.
- Four repeated Pause/Resume cycles at 5x Physics V2 remained connected and
  returned `PAUSED -> 5x` on every cycle.
