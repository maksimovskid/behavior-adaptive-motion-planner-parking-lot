# Archival Readiness Audit

Date: 2026-11-10. Verdict: **READY WITH MINOR NOTES for the reviewed working copy**.
The public commit still needs the proposed archival corrections reviewed and
published before it receives that verdict. No commit or push was performed.

## Repository State

- Branch: `main`.
- Audited HEAD and freshly fetched `origin/main`:
  `7c8a8a121a229ac7b0275f61419c7cd539ac85ca`.
- Origin: https://github.com/maksimovskid/behavior-adaptive-motion-planner-parking-lot.git
- Nine commits; no tags or public releases at audit time.
- Working tree was clean before this audit. HEAD remains unchanged; the review
  changes listed below are uncommitted and therefore differ from public main.
- Windows 11, build 26200, PowerShell; Python 3.13.9.
- Local packages: NumPy 2.3.4, SciPy 1.17.1, Matplotlib 3.10.7, Pillow 12.0.0.
  CVXPY was absent locally, but none of the registered scenarios required it.

## Evidence And Historical Scope

The [published thesis](https://www.researchgate.net/publication/405001385_Behavior_Adaptive_Motion_Planning_for_Automated_Driving)
identifies Daniel Maksimovski, TH Ingolstadt, and 2020. Its planning concepts
include path/velocity decomposition, Hybrid A*, finite-state decisions, and
local Reeds-Shepp RRT maneuvers. The final parking maneuver is excluded.

Current code follows the corresponding architecture:

`scenario/config -> map -> Hybrid A* -> behavior/local path -> speed reference -> controllers -> bicycle simulation -> plots`.

The finite-state planner selects local paths. Lead-following speed adaptation
is implemented separately in its scenario's velocity helper. There is no single
unified behavior-to-velocity decision interface across all scenarios.

The repository history starts with the already restructured release, not the
original 2020 tree. Exact reproduction of thesis tables, figures, algorithm
parameters, and historical experiments cannot be established from that history.
The README now explicitly describes the repository as an associated research
implementation containing later changes.

## Thesis-To-Code Classification

The classifications below describe the evidence available from current code,
documentation, and release history, not certified equivalence to the original sources.

| Component | Classification | Evidence and qualification |
| --- | --- | --- |
| Hybrid A*, grid A* heuristic, Reeds-Shepp connections | Cleaned/refactored thesis implementation | Adapted upstream modules; current valid scenario paths run successfully. Historical numerical equivalence is unclear. |
| Static obstacle representation | Simplified implementation | Discrete wall and lane-divider points; global and local obstacle sets differ. |
| Goal configuration | Simplified implementation | Planner uses pose; simulation completion checks position only. |
| Finite-state lane/overtaking/return decisions | Cleaned/refactored thesis implementation | Nine named states; all are visited in the four avoidance runs. |
| Lead-vehicle speed following | Simplified implementation | Euclidean rear-axle separation minus 4 m and speed thresholds. |
| RRT / RRT Reeds-Shepp | Simplified implementation | RRT structure is retained; documented scenarios use direct goal sampling by default. Standalone RRT demos are not scenario validation. |
| Local smoothing and suffix stitching | Later engineering improvement | Cubic interpolation, duplicate filtering, and nearest-reference suffix integration. |
| Static-obstacle scenario, JSON configs, shared engines | Later engineering improvement | Current configurations and integration differ from a frozen thesis workspace. |
| Velocity construction | Simplified implementation | Spatial targets plus saturated proportional acceleration produce approximate trapezoidal histories. No explicit lateral-acceleration bound. |
| Longitudinal controller | Simplified implementation | P-only, gain 10, acceleration saturation +/-2 m/s²; historical PID naming does not imply I/D terms. |
| Rear-wheel tracking | Cleaned/refactored thesis implementation with later tuning | Heading gain 1.0, lateral gain 0.7; gain tuning is later work. |
| Vehicle simulation | Simplified implementation | Rear-axle kinematic bicycle state at dt=0.1 s; 2.7 m wheelbase. |
| Dynamic obstacle | Simplified implementation | Prescribed lead path/speed, without perception or interaction uncertainty. |
| Semantic map, yellow markings, live GIF export | Later engineering improvement | Presentation layer independent of planner obstacle points. |
| Final parking, perception/localization uncertainty, V2X | Not implemented | Outside the implemented scenario pipeline. |

## Scenario Results

All six registered scenarios were executed to natural completion through the
runner's opt-in smoke suite. Instrumented runs additionally captured states,
control outputs, references, live frames, and final trajectory/speed figures.
No scenario configuration was changed.

All global path preparations succeeded. All captured state/reference/control
values were finite; no exceptions occurred. Ego speed stayed nonnegative.
Maximum ego displacement per 0.1 s was 0.417 m (0.222 m for static avoidance,
0.361 m for following), consistent with simulated speed and without teleportation.

All ego vehicles start at rear-axle pose `(1.5, 8, 0)` at rest.
Goals have y=32 m and planner yaw=pi. Speeds below are in km/h.

| Scenario | Goal x | Lead/stopped initial x, speed | Lead target | Ego maximum | Completion time | Local planning |
| --- | ---: | --- | ---: | ---: | ---: | --- |
| static_environment | 13 | None | - | 15 | 21.7 s | None; global path tracking |
| lead_vehicle_following | 27 | 9 m, 12.996 | 13 | 13 | 21.2 s | None; speed following |
| static_obstacle_avoidance | 17 | 24 m, 0 | 0 | 8 | 34.5 s | Outward and return |
| lane_change_1 | 5 | 9 m, 0.36 | 0.1 | 15 | 24.4 s | Close, nearly stopped lead; outward and return |
| lane_change_2 | 5 | 15 m, 7.2 | 6.2 | 15 | 22.5 s | Later trigger for farther moving lead; outward and return |
| lane_change_3 | 17 | 9 m, 12.996 | 2.5 | 15 | 21.6 s | Lead decelerates; extended goal; outward and return |

Every lead/stopped vehicle starts at y=8, yaw=0. Initial speeds may differ from
target speeds. Final ego speed is 0.833 m/s, approximately 3 km/h, rather than
zero. Simulation termination therefore demonstrates arrival, not completed parking.

Lane-change event intervals (simulation seconds):

| Scenario | Outward start/end | Return start/end |
| --- | --- | --- |
| static_obstacle_avoidance | 1.0 / 10.8 | 13.8 / 18.9 |
| lane_change_1 | 0.8 / 3.8 | 5.0 / 7.5 |
| lane_change_2 | 3.4 / 5.9 | 9.9 / 12.4 |
| lane_change_3 | 1.6 / 4.2 | 6.6 / 9.1 |

## Geometry And Collision Evidence

The independent audit used oriented 4 m by 2 m rectangles (front/rear extent
3 m / 1 m from the rear axle), separating-axis overlap tests for vehicle pairs,
and polygon clipping against the continuous central island x=0..33, y=14..26.
This check samples dt=0.1 s; it is not continuous swept-volume validation.

| Scenario | Minimum sampled vehicle boundary separation | Vehicle overlap | Continuous-island overlap at sampled poses |
| --- | ---: | --- | --- |
| static_environment | N/A | N/A | None |
| lead_vehicle_following | 3.500 m | None | None |
| static_obstacle_avoidance | 1.790 m | None | None |
| lane_change_1 | 0.434 m | None | None |
| lane_change_2 | 1.731 m | None | **Seven samples**, t=8.8..9.4 s; maximum area 0.04055 m² |
| lane_change_3 | 1.673 m | None | None |

All sampled rectangles stay within the outer drive envelope x=0..41, y=6..34.
The lane_change_2 violation is an edge crossing the central island corner;
checking only rectangle corners would miss it. The repository's point-obstacle
rectangle check reports no wall-point collision in any executed run.

Global planning checks rectangles against discrete obstacle points on sampled
motion primitives. Local RRT uses center-point distances to obstacle points,
rejecting squared distances <=1.1. Local spline smoothing is not collision
revalidated with the vehicle footprint. Only the stopped-obstacle scenario adds
that vehicle's sampled outline to RRT and asserts a center-to-outline clearance;
moving lead vehicles are not inserted into RRT. These checks do not establish
general obstacle-avoidance safety. The README now states this limitation.

## Planner And State-Machine Findings

Hybrid A* uses grid A* as a heuristic and sampled Reeds-Shepp analytic connections.
Valid scenario paths are ordered start-to-goal with approximately 0.1 m steps.
No fabricated fallback path was observed. The review patch rejects nonfinite,
out-of-map, colliding, empty-map, and invalid-resolution input before search.
Search exhaustion returns None; preparation now raises a clear RuntimeError.
Tests exercise invalid starts and forced exhaustion, rather than attempting an
unbounded exhaustive search on every impossible map.

No optimality or completeness claim is made. Grid-resolution globals still
appear in motion expansion, so arbitrary custom resolutions are not validated.

RRT's original `randint(0,1) > 10` never explored. The small review correction
makes the sampling scale 0..100 and explicitly sets the default goal sample
rate to 100. This preserves every documented scenario's goal-only behavior;
lower explicitly supplied rates can now explore. This is not evidence of broad
random-tree exploration in the published experiments. Goal orientation acceptance
is not independently enforced by the goal-region check, though the configured
direct Reeds-Shepp connectors reach their specified end poses.

Raw RRT results are goal-to-start with repeated endpoints. The integration
reverses them, filters near duplicates, interpolates, and appends a nearest
reference suffix. Maximum observed reference segment length was 0.200001 m or
less. Empty/degenerate results and absent Reeds-Shepp connections now fail
explicitly. Target lookahead is clamped at the last reference sample.

States are 0 follow_lane, 1 find_lane_change, 2 lane_change,
3 extend_path_after_lane_change, 4 follow_second_lane,
5 find_double_lane_change, 6 double_lane_change,
7 extend_path_after_double_lane_change, and 8 follow_lane_again.
All four avoidance runs visited 0..8 in order. Triggers use Euclidean separation
minus 4 m, a time gap, speed thresholds, and optional minimum return x.
State selection uses path lengths and flags rather than an explicit transition
table; arbitrary long/short paths can change which conditions override others.
There is no bounded retry or emergency-stop state. Planning failure aborts the
simulation with an exception, rather than safely controlling a physical vehicle.

## Reference And Tracking Contracts

- A path is spatial x/y/yaw plus curvature; it is not time-parameterized.
- PreparedPath keeps the Hybrid A* x/y/yaw arrays; spline curvature is separately
  sampled at nominal 0.1 m and padded/truncated to match their count. Equal
  lengths do not prove exact spatial alignment on arbitrary nonuniform paths.
- Local spline samples nominally use 0.2 m; speed helpers still assume 0.1 m
  for index-based deceleration distances. This limits general timing accuracy.
- Velocity targets have one value per active path sample; nearest-point index
  selects a target. Recorded trajectories are separate dt=0.1 s state histories.
- The goal-reaching state update is checked before history append, so plotted
  histories omit the final accepted update. Reported `len(t)*dt` includes that
  elapsed step; final plotted pose is one update earlier than accepted arrival.
- No explicit curvature-dependent speed cap appears in current velocity helpers.
  Longitudinal saturation, not a full trajectory optimizer, supplies the observed
  acceleration/deceleration ramps.
- Rear-wheel tracking ignores its previous-index argument and searches the full
  path. Near-zero heading error returns zero steering even with lateral offset.
- Controller steering is upper-clamped to 0.6 rad but not lower-clamped. Measured
  minimum commands ranged to -0.754 rad in lane_change_2, while planner geometry
  uses MAX_STEER=0.5 rad. No physical steering actuator or tire model is present.

Tracking was finite and completed the intended routes. This supports the six
configured demonstrations, not general controller stability or hardware feasibility.

## Visual And Media Assessment

Live and final outputs were inspected for all six scenarios, including their
velocity profiles. Yellow lane markings, grid removal, rectangle orientation,
local-path display, and time titles are retained. No large reference connector
or backward trajectory jump was observed. Stationary-obstacle reference/trajectory
labels are redundant but do not change its displayed zero speed. Some tracking
waviness remains; the media shows the actual result.

The generation script was run with output paths redirected to ignored review
storage. All four outputs were byte-for-byte identical to the tracked assets:

| Tracked asset under docs/figures | Decision |
| --- | --- |
| static_environment_planned_and_tracked_trajectory_no_grid.png | KEEP |
| static_environment_live_planning_and_tracking_no_grid.gif (54 frames) | KEEP |
| lane_change_3_velocity_profiles_no_grid.png | KEEP |
| lane_change_3_dynamic_obstacle_live_no_grid.gif (53 frames) | KEEP |

The earlier filenames without `_no_grid` are no longer tracked. README image
references and generator paths point to the current names. No media was overwritten.

## Tests, Installation, Windows And CI

Baseline public tree: 14 tests ran, **13 passed, one skipped**, zero failures.
The skip is the opt-in slow scenario smoke test, not a failing feature.

Reviewed working copy: 19 tests ran, **18 passed, one skipped** by default.
Enabling RUN_SLOW_SCENARIO_SMOKE=1 and selecting all six scenarios produced
**19 passed, zero skipped**, with natural completion enabled. Five new focused
regression cases cover global invalid inputs/exhaustion, degenerate local paths,
RRT failure, and configurable sampling. The latest failure-branch additions also
passed the focused five-test rerun.

`python scripts/verify.py` passes: 46 active Python files compile, fast tests
pass, and static_environment, lead_vehicle_following, static_obstacle_avoidance,
and lane_change_3 complete. Verification now passes `--no-stop-after-goal`, so
post-summary assertions execute. The compile scanner now excludes output and
environment directories; it previously compiled thousands of installed files.

Coverage is strongest for registry/CLI/compilation, selected behavior decisions,
kinematic helper formulas, one valid global path contract, and scenario completion.
Collision geometry, continuous safety, controller robustness, and arbitrary
planner inputs are not comprehensively tested. Media received direct audit
comparison rather than new image-snapshot tests.

A fresh public network clone at the audited SHA installed requirements in a new
venv. Python 3.13.9 resolved NumPy 2.5.4, SciPy 1.18.1, Matplotlib 3.11.2,
Pillow 13.0.0, and CVXPY 1.9.3. Public `verify.py` passed in that environment;
lane_change_3 also passed explicitly with natural completion. No local-only
source or data file was needed. Requirements are entirely unpinned, and
pyproject.toml contains lint settings rather than package/install metadata.
Long-term bit-for-bit reproducibility remains a minor archival limitation.

The list command was executed in both PowerShell and CMD. Scenario CLI and
verification commands were executed in PowerShell. The documented `$env:` syntax
is correctly labeled PowerShell; CMD users need `set NAME=value`. The visual
runner was inspected and live render callbacks were exercised with the Agg
backend; an interactive GUI window was not manually tested during this audit.

The Tests workflow runs ubuntu-latest / Python 3.11 and installs requirements,
then runs verify.py (including four scenarios). The [latest public run](https://github.com/maksimovskid/behavior-adaptive-motion-planner-parking-lot/actions/runs/26094271596)
is successful at the audited SHA. It does not cover Windows, all six scenarios,
or image checks. Public CI has not run the uncommitted review changes.

## Attribution, Citation And Hygiene

MIT LICENSE is present. The upstream MIT copyright line was checked against
[PythonRobotics LICENSE](https://github.com/AtsushiSakai/PythonRobotics/blob/master/LICENSE)
and the third-party notice corrected to the current upstream wording. Notices
now name the adapted global, local, and interpolation modules. Existing source
credits include Zheng Zh (Hybrid A*), Nikos Kanargias (A*), and Atsushi Sakai
(RRT, Reeds-Shepp, and cubic splines). Full historical line-by-line license
provenance is not recoverable without an original upstream revision identifier.

The public CFF used thesis-only properties at its root and was invalid under
CFF 1.2.0. The patch describes the software at root and puts the thesis under
preferred-citation. It validates against the [official schema](https://raw.githubusercontent.com/citation-file-format/citation-file-format/main/schema.json).
The thesis citation title is normalized to the published unhyphenated title;
the hyphenated repository/product name remains a separate label.

Tracked-file searches found no credential patterns, absolute user paths, cache
files, environments, IDE settings, or audit scratch files. This is a targeted
pattern inspection, not proof of absence of every possible secret. Public author
and thesis information is intentional attribution. Only outputs/README.md is
tracked under outputs (KEEP); review diagnostics and the fresh clone are ignored
local artifacts (OPTIONAL). docs/figures assets are KEEP. No generated research
outputs were deleted. No stale repository-level WIP/TODO maturity wording was
found; "temporary" in the output-folder guide remains appropriate.

Scenario wrappers resolve to shared active engines. Standalone algorithm demos
and a spline demonstration remain outside the registry; their plotting/debug
paths are not claimed to have scenario-level validation. No duplicate legacy
scenario shadows a registered import. The CLI provides the documented entry point.

## Relationship To Later Research

This repository concerns parking-lot motion planning. The separate
[PriMa Python repository](https://github.com/maksimovskid/priority-maneuver-coordination)
concerns cooperative maneuver coordination and interaction logic; the
[Artery repository](https://github.com/maksimovskid/artery-maneuver-coordination)
uses an OMNeT++/V2X ITS-G5 simulation environment. Neither later system is part
of this implementation, and equivalence is not required for this archive.

## Freeze Matrix

Statuses refer to the working copy with the proposed corrections unless noted.

| Check | Status | Qualification |
| --- | --- | --- |
| A Local/public HEAD consistent | PASS | Identical SHA; working copy has review changes. |
| B Fresh install | PASS | Public clone, isolated Windows venv. |
| C Fast tests | PASS | 18 passed, one opt-in skip. |
| D Standard verify | PASS | Natural completion and post-run checks enabled. |
| E Registered scenarios | PASS | All six pass real runner smoke tests. |
| F Global planning | PARTIAL | Configured paths valid; arbitrary resolutions/search spaces unvalidated. |
| G Local planning | PARTIAL | Goal-only defaults; sampled checks; no post-spline footprint check. |
| H Behavior transitions | PARTIAL | All states exercised; no safe failure/retry state. |
| I Velocity planning | PARTIAL | Target following works; no lateral-acceleration enforcement. |
| J Tracking | PARTIAL | Configured runs finish; steering/model limitations remain. |
| K Visual media | PASS | All four byte-identical to fresh generation. |
| L README matches code | PASS | Historical provenance and limitations now explicit. |
| M Attribution/license | PASS | MIT and adapted-module notices; upstream revision unknown. |
| N Thesis citation | PASS | Repaired CFF validates; published thesis URL confirmed. |
| O Hidden local dependency | PASS | Fresh clone runs without local data. |
| P Secrets/temp hygiene | PASS | No findings in targeted tracked-file inspection. |
| Q CI | PARTIAL | Public green; Ubuntu 3.11, four scenarios; patch not yet public. |
| R No critical behavioral defect in documented scope | PARTIAL | No crashes/vehicle overlap; island intrusion must remain disclosed. |
| S Understandable publicly | PASS | README, configs, commands, media, and audit explain scope. |

## Remaining Issues And Review Changes

**CRITICAL BEFORE FREEZE, corrected locally:** invalid citation metadata;
verification could terminate before assertions; unclear empty/failed planner
results; out-of-range local targets; misleading omission of historical and
collision-model limitations. Review these changes before treating public main
as the final archive.

**MINOR ARCHIVAL NOTES:** lane_change_2 island overlap under continuous geometry;
sampled collision limits; unmatched steering bounds; nominal path-step assumptions;
arrival without a full stop; goal-only RRT defaults; unpinned dependencies; no
release/tag or original upstream revision. These prevent a safety or exact thesis
reproduction claim. They do not prevent preserving a candid research demonstration.

**GOOD TO HAVE:** a versioned dependency snapshot, archival release/tag, additional
Windows CI and focused continuous-geometry checks. None is required to modernize
the thesis implementation.

**FUTURE-ONLY:** continuous collision-aware smoothing, steering/actuator and
controller redesign, curvature-constrained velocity optimization, generalized
behavior/failure recovery, perception uncertainty, and richer dynamic obstacles.

Files changed for review:

- CITATION.cff: schema-valid software and preferred thesis citation.
- NOTICE.md: published thesis title consistency.
- THIRD_PARTY_NOTICES.md: upstream notice and adapted-module mapping.
- README.md: provenance, current velocity/local planning behavior, limitations.
- docs/scenario_audit.md: identifies historical baseline and links this audit.
- docs/archival_readiness_audit.md: this report.
- motion_planning/global_planning/hybrid_a_star.py: input validation and explicit failure.
- motion_planning/local_planning/behaviour_and_local_planner.py: degenerate paths and target bounds.
- motion_planning/local_planning/rrt_reeds_shepp.py: explicit historical default, valid sampling scale, absent connector handling.
- simulation/path_setup.py: explicit unusable-path error.
- scripts/check_compile.py: exclude environments and generated outputs.
- scripts/verify.py: wait for scenario assertions and process completion.
- tests/test_scenario_runner.py: natural-completion smoke checks.
- tests/test_archival_contracts.py: five focused regression cases.

No controller gains, scenario configurations, physical vehicle parameters, or
tracked media were changed. Diagnostics, state histories, visuals, and the public
clone remain under ignored outputs/archival_audit for local inspection.

## Direct Answers

**If this repository is never modified again, is it defensible as the archived
implementation associated with the Master's thesis?** With the reviewed
corrections and explicit limitations, yes, as an associated, later-maintained
research implementation. It is not an exact 2020 reproduction or a certified
collision-free system. The unchanged public commit still has citation and
verification defects and should receive the corrections before final freeze.

**Does the current public repository materially misrepresent the thesis?** No
fabricated thesis results or unrelated later systems were identified. However,
the public README did not adequately distinguish later changes from the original
experiments, and its planning description omitted relevant simplifications.
The patch makes that distinction explicit. Full numerical equivalence to the
thesis remains unverified and should not be asserted.

## Final Git Checks

`git status --short`:

```text
 M CITATION.cff
 M NOTICE.md
 M README.md
 M THIRD_PARTY_NOTICES.md
 M docs/scenario_audit.md
 M motion_planning/global_planning/hybrid_a_star.py
 M motion_planning/local_planning/behaviour_and_local_planner.py
 M motion_planning/local_planning/rrt_reeds_shepp.py
 M scripts/check_compile.py
 M scripts/verify.py
 M simulation/path_setup.py
 M tests/test_scenario_runner.py
?? docs/archival_readiness_audit.md
?? tests/test_archival_contracts.py
```

`git diff --stat`: 12 tracked files changed, 66 insertions, 18 deletions.
The new audit report and regression-test file are untracked and therefore are
not included in that diff statistic. There are 14 review files in total.

`git diff --check`: exit 0, no whitespace errors. Git emits Windows LF-to-CRLF
conversion warnings for text files; these are not failed checks.

Additional visual-runner check: lead_vehicle_following GIF export completed
naturally with exit 0 (two review frames requested). Its GUI window itself was
not opened. Review artifacts are ignored and do not appear in the status above.
