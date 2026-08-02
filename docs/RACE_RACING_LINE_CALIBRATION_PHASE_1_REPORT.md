# Race Racing-Line Calibration Phase 1 Report

Date: 2026-08-02 KST
Status: **Partial — no calibration candidate approved**

## Scope

The run followed the Phase 1 directive for Bahrain (circuit 3) and Red Bull Ring
(circuit 4): race basis, NORMAL, Medium/C3, Standard pace, seed 42, and the
authoritative 0.02 s / 50 Hz physics step. Qualifying-line logic and global tire,
vehicle, pit, Safety Car, and render-width changes were out of scope.

The working tree was already dirty. Existing user changes were preserved. The
reports record the current branch and source hashes for reproducibility.

## Baseline: three identical 5-lap runs

| Circuit | Fastest lap | Lateral p95 / max | Heading p95 | Time >2 m | Reversals | Safety counters |
|---|---:|---:|---:|---:|---:|---|
| Bahrain | 92.907 s | 4.785 / 6.141 m | 0.105 rad | 53.46 s | 0 | all zero |
| Red Bull Ring | 66.990 s | 5.759 / 7.918 m | 0.105 rad | 54.40 s | 2 | all zero |

All three repeats for each circuit produced the same metrics. The approval target
was p95 ≤ 2 m, max ≤ 4 m, heading p95 ≤ 0.08 rad, with no safety regressions.

The largest errors are not a single scalar-speed issue: Bahrain is concentrated
around T4–T6 and T11–T12; RBR around T5–T7 and T9–T10. Diagnostic samples now keep
compiled-centerline progress, active path distance/progress, aligned telemetry
progress, centerline-relative offset, active-line pose error, curvature, and
vehicle lateral speed in explicitly named fields.

## Candidate results

The following candidates were diagnostic-only and were not written into
`data/circuits.json`:

| Candidate | Bahrain | RBR | Decision |
|---|---|---|---|
| Initial smoothing 3/6 | p95 4.720/4.610 m | 5.547/5.479 m | Reject: still over target |
| Max lateral slope .02 | p95 4.169 m | 5.359 m; fallback 150 | Reject: target/fallback |
| Smoothing 6 + slope .02 | p95 4.294 m | 4.921 m; fallback 200 | Reject: target/fallback |
| Smoothing 6 + slope .01 | — | p95 4.374 m | Reject: still over target |
| Reference lateral-speed FF 0.25–1.0 | — | p95 5.693–5.666 m | Reject: no target gain; reversals increased |
| Clean-line response cap 2.5 m/s | p95 6.632 m | p95 6.560 m; off-track 15 | Reject: regression |
| Reference weight 0 | run-wide 2; 95.818 s | planner rejected initial path | Reject: invalid/unsafe |

The existing clean-line response cap remains 1.25 m/s. The optional feed-forward
defaults to zero; active path curvature remains authoritative, so no double
feed-forward was enabled.

## Stint and traffic evidence

The 15-lap single-car stint completed on both circuits with race fuel, Medium, and
NORMAL. Bahrain had lateral p95/max 4.822/6.223 m, zero safety counters, zero
planner fallback, and rear surface/core peaks 112.266/102.414°C. RBR had
5.830/7.971 m, 300 planner-fallback samples, 8 correction reversals, and rear
surface/core peaks 102.702/94.295°C. Both had zero clamp and continuous-overheat
seconds, but RBR therefore fails the approval gate.

The existing 20-car session harness was generalized to both circuits. Its 1x/2x
cadence samples passed:

- Bahrain: effective 0.987x / 1.955x; contact, track-limit, and fallback samples 0.
- RBR: effective 0.983x / 1.919x; contact, track-limit, and fallback samples 0.

A direct 20-car, minimum-10-lap fixed-step approval harness was added as
`backend/tools/run_race_line_traffic.py`, but its first run was stopped after the
runtime became impractical for this interactive pass. It is intentionally not
reported as a pass.

## Implementation and tests

- `backend/tools/run_circuit_baseline.py`: reproducible baseline CLI, source/hash
  identity, coordinate contract, corner signed-error reports, repeat aggregation,
  and diagnostic A/B flags.
- `backend/simulation/race_engine.py`: optional, default-off clean-line lateral
  feed-forward and response-cap injection for isolated experiments.
- `backend/simulation/track_physics.py` and `backend/models/schemas.py`: optional
  per-circuit line transition slope, defaulting to the existing behavior.
- `backend/tools/run_bahrain_runtime_soak.py`: reusable circuit-id argument for
  the 20-car 1x/2x session check.
- `backend/tests/test_circuit_runtime_baseline.py`: coordinate-frame, direction,
  feed-forward-default, and 0.01/0.02/0.05 fixed-step contract tests.

Passed:

- `validate_track_data.py --circuit-id 3/4`
- `audit_tier_a.py --circuit-id 3/4`
- directive regression set: 86 tests, `OK`
- 20-car 1x/2x session checks for both circuits

The long 20-car/17-lap thermal test was not rerun in this pass because it exceeded
the interactive execution budget; the existing project status records its prior
successful direct-engine result. The 10-lap traffic gate remains open.

## Decision and next step

Phase 1 is partial. No candidate meets the quantitative line-error approval gates,
so no per-circuit racing-line calibration value is accepted. The next calibration
attempt should isolate active-line pose/path-distance alignment with a small,
deterministic fixture before changing smoothing, planner slope, or controller
response again.

## Corrective foundation review — 2026-08-02

The failed candidate review found that the first feed-forward A/B changed only
`reference_lateral_speed_mps` while leaving the clean target speed at zero. Since
the bicycle integrator subtracts reference speed from target speed, that experiment
did not test the intended centerline-to-active-line transport contract.

The corrective implementation now:

- computes the active-line offset derivative against active path distance;
- passes the same derivative as target and reference speed, so relative target
  speed remains zero and path curvature remains the only steering feed-forward;
- preserves coefficient `0` as the product default because enabling the still
  unapproved candidate for the full field introduced Safety Car regressions;
- runs the diagnostic CLI with coefficient `1` so the corrected contract can be
  evaluated without silently changing the packaged game;
- keeps the official race distance while stopping the diagnostic after the
  requested sample laps, producing full-race start fuel instead of 5-lap fuel;
- replaces the qualifying-reference 3% acceptance flag with race-line gates;
- records heading-limit duration/ratio and keeps fallback flags in coordinate
  samples;
- measures gradual correction reversals over 0.20-second windows and resets the
  detector at lap boundaries.

The new deterministic three-repeat artifacts are:

```text
backend/data/calibration/bahrain_race_line_foundation_v2.json
backend/data/calibration/red_bull_ring_race_line_foundation_v2.json
```

| Circuit | Fuel basis | Lap | Lateral p95 / max | Heading-limit time | >2m time | Fallback |
|---|---:|---:|---:|---:|---:|---:|
| Bahrain | 57 laps / 108.031kg | 94.149s | 4.410 / 6.057m | 64.46s | 46.64s | 0 |
| Red Bull Ring | 71 laps / 107.364kg | 67.995s | 5.652 / 7.942m | 57.54s | 43.76s | 250/run, 750 total |

The paired transport reduces Bahrain p95 from 4.785m to 4.410m under the now
heavier and correct fuel input, but neither circuit reaches p95 2m / max 4m.
Both still spend material time at the bicycle heading limit of 0.105rad. RBR also
fails the clean-car fallback gate. Increasing global cornering stiffness, reducing
all target speeds, adding offset lookahead, and combining lower line slopes with
the paired transport were tested read-only and rejected: they did not bring both
circuits near the target without fallback, off-track risk, or excessive lap loss.

Validation after the corrective change:

- race-line/track/trajectory/vehicle/telemetry set: 90 tests, `OK`;
- the two SC tests affected by the experimental product-default enablement: `OK`
  after restoring the candidate to diagnostic-only;
- the full 301-test engine run exposed the two already-recorded unrelated failures
  (Bahrain compact pit elapsed tolerance and wake golden) plus two temporary SC
  failures while the candidate was product-enabled; the two SC failures were then
  rerun individually and passed after rollback;
- `git diff --check`: `OK`.

Decision remains **Partial**. The measurement and frame contract are corrected,
but no racing-line calibration is approved and the packaged product keeps the
previous clean-line behavior. The next implementation must address dynamic path
trackability/heading saturation and RBR fallback with a dedicated fixture before
re-enabling paired transport in multi-car sessions.

## Dynamic trackability correction — 2026-08-02

The dedicated constant-radius fixture reproduced the next failure independently
of circuit data. At 55–65m/s the car could have enough total lateral-force
capacity, but the bicycle model kept front/rear cornering stiffness fixed at its
static-load value. The resulting tyre/body slip consumed the complete 0.105rad
controlled-heading allowance. Once the error reached that guard, a feasible fast
corner could look like an untrackable path.

The accepted correction:

- scales front and rear cornering stiffness with the square-root-like
  `vertical-load capacity / static-load capacity` response (exponent 0.65,
  bounded at 2.5x), while leaving the total tyre-force ceiling unchanged;
- passes the nominal tyre envelope separately from the combined-slip force left
  after braking/traction, so longitudinal demand does not incorrectly erase
  vertical-load-derived stiffness;
- adds a 10-second, 55m/s, 0.014/m constant-radius regression that limits heading
  guard contact and path error;
- enables the already-corrected paired clean-line target/reference transport as
  the product default only after the Safety Car queue-formation and six-car
  physical-order reconciliation tests passed with the new load response;
- retains the explicit coefficient `0` replay path for legacy comparison.

Three identical five-lap full-race-fuel runs were regenerated in the existing
foundation-v2 artifacts:

| Circuit | Lap | Lateral p95 / max | Heading-limit time | >2m time | Fallback | Rear surface/core peak |
|---|---:|---:|---:|---:|---:|---:|
| Bahrain | 94.149s | 3.733 / 4.883m | 61.66s | 39.68s | 0 | 107.917 / 94.423°C |
| Red Bull Ring | 67.997s | 5.488 / 7.582m | 53.60s | 41.96s | 250/run | 103.170 / 91.666°C |

Compared with the corrected pre-load foundation, Bahrain improves from
4.410/6.057m and RBR from 5.652/7.942m. All repeats are deterministic, and both
circuits retain zero contact, off-track, track-limit, run-wide, numeric-guard,
visible traction-loss, and thermal-clamp samples. The thermal result therefore
does not regress the earlier tyre approval.

This is an accepted **global vehicle-model correction**, not circuit-line
approval. Bahrain is materially better but still misses p95 2m/max 4m. RBR
remains dominated by the T6–T7 and T9–T10 measured-reference transitions and
still fails the clean-car planner-fallback gate. Read-only candidates that scaled
the RBR reference offsets, increased the clean-line lateral-speed cap, increased
downforce grip, reduced all target speeds, or weakened yaw relaxation were not
accepted: each either remained outside the gates, created off-track/fallback
risk, changed global grip without a physical baseline, or imposed excessive lap
loss. No new per-circuit value was written to `data/circuits.json`.

Final validation:

- path/track/trajectory/vehicle/telemetry plus the two SC order tests: 96 tests,
  `OK` in 105.836s;
- Bahrain 20-car, 1,700-second thermal integration: `OK` in 513.766s, with the
  deterministic rear surface/core golden updated from 119.248/105.247°C to
  117.321/107.975°C; overheat and both clamp counters remain zero;
- five laps × three repeats for each circuit: deterministic `pass`;
- Python compilation and `git diff --check`: `OK`.
