# Gate 5 Report: Closed-loop Oracle Predictive Navigation

Date: 2026-08-30
Task book: `/home/w417/文档/Oracle预测式导航生死实验_分阶段执行任务书_v1.docx`
Branch: `exp/oracle-g5-closed-loop-2026-08-29`
Experiment commit: `bf47951818887ddd7052bed71b945c74117ce675`
Current HEAD: `bf47951` (`perf: throttle Oracle geometry diagnostics`)

## 1. Gate decision

**Gate 5: BLOCKED.**

The formal batch completed the planned 20 runs, but it did not satisfy the
task-book hard acceptance criteria. The result is not a failure of the whole
repository: Gate 3 and Gate 4 still prove that the Oracle message interface
and `PredictionCritic` plugin work. Gate 5 is specifically the first closed-
loop test of whether that information produces stable navigation behavior.

| Scenario | Reactive accepted | Oracle accepted |
|---|---:|---:|
| S1 crossing, 5 runs each | 5/5 | 3/5 |
| S2 oncoming, 5 runs each | 1/5 | 3/5 |
| **Total** | **6/10** | **6/10** |

Here “accepted” means Nav2 reached the goal and Gazebo reported no robot
contact with the dynamic obstacle or other non-ground object. A Nav2 success
with a physical dynamic contact is not accepted. Startup failures are also
failures, even though no goal was sent.

Therefore the project must not enter Gate 6 statistical claims or the
Transformer training stage yet.

## 2. Experimental scope and fairness

The formal batch used the same dynamic world, start, goal, difficulty, and
deterministic waypoint schedule for both methods:

```text
world:      src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world
scenario:   S1_crossing or S2_oncoming
difficulty: medium
start:      (-8.5, 0.0), yaw 0
goal:       (8.5, 0.0), yaw 0
controller: MPPI DiffDrive, 10 Hz, 30 x 0.10 s = 3.0 s horizon
batch:      500
vx range:   -0.12 .. 0.28 m/s
wz max:     0.90 rad/s
```

Reactive used:

```text
profile:   reactive_mppi_static
parameters: experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml
```

Oracle used the same current RGB-D/RTAB-Map/Nav2 path and added only the
future-information path:

```text
profile:   oracle_mppi_prediction
parameters: experiments/oracle_mppi/configs/nav2_mppi_oracle_footprint005_params.yaml
publisher: experiments/oracle_mppi/configs/oracle_publisher_gate3.yaml
topic:     /oracle/predicted_occupancy
critic:    PredictionCritic, cost_weight=50.0
```

The dynamic obstacle is a collidable Gazebo model. Each run records the
Gazebo contacts stream, model-state ground truth, command velocity, Nav2
result, and post-run time-aligned Oracle analysis. The full local evidence is
under:

```text
experiments/oracle_mppi/gate5/formal_20260830_01/
```

The machine-readable 20-row summary is
[`gate5_smoke_summary.csv`](../gate5/formal_20260830_01/gate5_smoke_summary.csv);
the 10 paired rows are in
[`gate5_paired_summary.csv`](../gate5/formal_20260830_01/gate5_paired_summary.csv).

## 3. Run-level results

`PASS` means the hard acceptance condition passed. The other labels preserve
the independent reason for rejection.

| Scenario | Method | Run | Outcome | Nav2 status | Sim time [s] | Path [m] | Final error [m] | Min dynamic clearance [m] | Dynamic contact |
|---|---|---:|---|---:|---:|---:|---:|---:|---|
| S1 | Reactive | 01 | PASS | 4 | 157.1 | 18.015 | 0.032 | 1.359 | no |
| S1 | Reactive | 02 | PASS | 4 | 141.4 | 18.465 | 0.062 | 1.497 | no |
| S1 | Reactive | 03 | PASS | 4 | 157.8 | 18.235 | 0.042 | 0.473 | no |
| S1 | Reactive | 04 | PASS | 4 | 166.2 | 18.679 | 0.018 | 0.295 | no |
| S1 | Reactive | 05 | PASS | 4 | 164.3 | 18.315 | 0.035 | 1.529 | no |
| S1 | Oracle | 01 | PASS | 4 | 164.2 | 20.246 | 0.024 | 0.000 | no |
| S1 | Oracle | 02 | PASS | 4 | 158.0 | 18.066 | 0.029 | 0.110 | no |
| S1 | Oracle | 03 | NAV_TIMEOUT | 5 | 196.4 | 20.164 | 0.174 | 0.000 | no |
| S1 | Oracle | 04 | NAV_TIMEOUT | 5 | 224.6 | 18.947 | 0.248 | 1.923 | no |
| S1 | Oracle | 05 | PASS | 4 | 158.1 | 17.971 | 0.026 | 1.504 | no |
| S2 | Reactive | 01 | STARTUP_FAILURE | — | — | — | — | — | — |
| S2 | Reactive | 02 | NAV_SUCCESS_WITH_DYNAMIC_CONTACT | 4 | 223.1 | 36.166 | 0.003 | 0.000 | **yes** |
| S2 | Reactive | 03 | PASS | 4 | 159.7 | 18.402 | 0.021 | 0.540 | no |
| S2 | Reactive | 04 | NAV_TIMEOUT_WITH_DYNAMIC_CONTACT | 5 | 269.0 | 24.287 | 0.327 | 0.000 | **yes** |
| S2 | Reactive | 05 | NAV_TIMEOUT | 5 | 224.5 | 18.954 | 0.246 | 2.462 | no |
| S2 | Oracle | 01 | PASS | 4 | 178.0 | 18.472 | 0.020 | 0.795 | no |
| S2 | Oracle | 02 | PASS | 4 | 178.4 | 21.132 | 0.073 | 0.533 | no |
| S2 | Oracle | 03 | STARTUP_FAILURE | — | — | — | — | — | — |
| S2 | Oracle | 04 | PASS | 4 | 174.4 | 18.353 | 0.102 | 0.699 | no |
| S2 | Oracle | 05 | STARTUP_FAILURE | — | — | — | — | — | — |

The complete values, including wall duration, stop/reverse ratios, runner
exit code, evidence completeness, and matrix timestamps, are in the CSV and
should be used for any later statistical processing.

Representative visual evidence committed with this report:

- [S1 Oracle run 01: successful Oracle timeline](../gate5/formal_20260830_01/S1_crossing/oracle_run_01/gate5_timeline.png)
- [S1 Oracle run 03: timeout timeline](../gate5/formal_20260830_01/S1_crossing/oracle_run_03/gate5_timeline.png)
- [S2 Reactive run 02: arrival with dynamic contact](../gate5/formal_20260830_01/S2_oncoming/reactive_run_02/gate5_timeline.png)
- [S2 Reactive run 04: timeout with dynamic contact](../gate5/formal_20260830_01/S2_oncoming/reactive_run_04/gate5_timeline.png)

Each timeline combines the Gazebo top-down trajectory, dynamic interaction
distance, command response, and the audit summary. The remaining per-run
images are retained locally beside their raw CSV evidence.

## 4. Paired interpretation

The run-index pairing gives the following acceptance matrix:

| Scenario/run | Reactive outcome | Oracle outcome | Interpretation |
|---|---|---|---|
| S1/01 | PASS | PASS | both accepted |
| S1/02 | PASS | PASS | both accepted |
| S1/03 | PASS | NAV_TIMEOUT | Oracle closed-loop failure |
| S1/04 | PASS | NAV_TIMEOUT | Oracle closed-loop failure |
| S1/05 | PASS | PASS | both accepted |
| S2/01 | STARTUP_FAILURE | PASS | Reactive orchestration failure |
| S2/02 | dynamic contact | PASS | Reactive physical safety failure |
| S2/03 | PASS | STARTUP_FAILURE | Oracle orchestration failure |
| S2/04 | timeout + dynamic contact | PASS | Reactive safety and completion failure |
| S2/05 | timeout | STARTUP_FAILURE | both conditions failed for different reasons |

There are only three `PASS/PASS` S1/S2 pairs. Three pairs favor Oracle only
because Reactive failed while Oracle passed, and three pairs favor Reactive
only because Oracle failed to complete or start. This is not a stable paired
benefit claim.

## 5. Oracle interface and risk evidence

For every completed Oracle run, the publisher validation exit was `0`, the
publisher became ready, and the critic emitted `status=active` diagnostics.
This confirms that the plugin was not silently omitted. It does not by itself
prove that the risk was relevant to the selected candidate trajectories.

| Oracle run | Active diagnostics | Non-zero `risk_hits` diagnostics | Max reported risk | Max reported cost |
|---|---:|---:|---:|---:|
| S1/01 | 33 | 6 | 1.0 | 1350.0 |
| S1/02 | 32 | 0 | 0.0 | 0.0 |
| S1/03 | 39 | 5 | 1.0 | 1388.5 |
| S1/04 | 45 | 0 | 0.0 | 0.0 |
| S1/05 | 32 | 0 | 0.0 | 0.0 |
| S2/01 | 36 | 0 | 0.0 | 0.0 |
| S2/02 | 36 | 0 | 0.0 | 0.0 |
| S2/04 | 35 | 0 | 0.0 | 0.0 |

The two S1 runs with risk hits do not establish a useful causal result:
S1/01 reached the goal, while S1/03 timed out. The remaining completed S1
and S2 Oracle runs show active critic plumbing but no sampled risk hits in the
throttled diagnostics. The formal evidence therefore supports the statement
“Oracle is connected and can score risk in some conditions,” not
“Oracle consistently anticipates and improves control.”

## 6. Failure evidence

### 6.1 Oracle navigation timeouts

```text
S1_crossing/oracle_run_03
S1_crossing/oracle_run_04
```

Both have Nav2 status `5`, runner exit code `5`, no dynamic physical contact,
and complete dynamic ground-truth evidence. The failures are navigation
completion failures, not contact false positives. Their launch logs include
progress failures; S1/04 also contains repeated “no valid path found” and
control-period warnings.

### 6.2 Startup failures

```text
S2_oncoming/reactive_run_01
S2_oncoming/oracle_run_03
S2_oncoming/oracle_run_05
```

These runs stopped before metrics/trajectory generation. The retained
`startup_readiness.txt` files show that the expected Nav2 lifecycle state was
not reached. They must count as planned-run failures, not be removed from the
denominator and not be interpreted as navigation successes or collisions.

### 6.3 Reactive dynamic contacts

```text
S2_oncoming/reactive_run_02
S2_oncoming/reactive_run_04
```

Both contain a real Gazebo contact pair between
`waffle::base_link::base_collision` and
`oracle_dynamic_obstacle::link::collision`. Run 02 eventually reported Nav2
status `4`, but the physical contact makes it unacceptable. Run 04 both
contacted the obstacle and timed out.

## 7. Runtime diagnostic observations

The launch logs contain repeated controller/replanning stress signals. These
are textual occurrence counts, not unique physical events, but they explain
why completion cannot yet be treated as robust:

| Condition | S1 Reactive | S1 Oracle | S2 Reactive | S2 Oracle |
|---|---:|---:|---:|---:|
| `Failed to make progress` | 19 | 21 | 18 | 12 |
| missed 10 Hz control rate | 5 | 17 | 2 | 8 |
| no valid global plan | 0 | 6 | 10 | 0 |
| BT tick-rate overrun | 1 | 7 | 4 | 1 |

The formal batch also retained all per-run `launch.log`, `navigation.log`,
`gate5_timeline.png`, `dynamic_trajectory_comparison.png`, and trajectory CSV
files locally. Large contacts streams and rosbag databases remain ignored by
Git as configured; their summaries and metadata are still retained beside
each run for forensic review.

## 8. Gate 5 hard-acceptance checklist

| Task-book requirement | Result | Evidence/decision |
|---|---|---|
| S1 and S2 repeated 5 times per method | PASS | 20 planned rows in `matrix_status.csv` |
| Same world/current information and controlled Oracle addition | PASS at configuration level | same commit/world/scenario/profile split |
| Oracle changes control before the conflict in at least one stable scenario | **FAIL / not established** | only 2/8 started Oracle runs show non-zero sampled risk; no stable proactive trend |
| Oracle has no new collision | PASS for recorded Oracle runs | zero Oracle dynamic contacts, but completion/startup stability is insufficient |
| No more than 20% Oracle controller failure/stuck | **FAIL / no margin** | 2/10 Oracle navigation timeouts already consume the 20% allowance; 2 additional Oracle startup failures prevent a stable Gate result |
| Freeze a justified PredictionCritic weight | **FAIL** | current `50.0` is only a candidate; one-run sweep cannot justify freezing |
| Proceed to Gate 6 | **NO** | Gate remains BLOCKED |

## 9. What changed in this continuation

No navigation controller, world, or parameter value was changed after the
formal batch. The changes are evidence infrastructure only:

1. `summarize_gate5.py` now discovers every planned `reactive_run_N` and
   `oracle_run_N` directory from `experiment.yaml`/`scenario.yaml`, including
   startup failures.
2. The summarizer reads `matrix_status.csv`, exposes runner and trial exit
   codes, and adds `run_outcome`, `startup_failure`, `acceptance_pass`, and
   timeout fields.
3. The cost-sweep runner now passes its status CSV to the summarizer so future
   startup failures cannot silently disappear.

## 10. Required next action before Gate 6

The next iteration should remain on a new evidence root and change one class
of variables at a time:

1. Fix and smoke-test lifecycle startup reliability; do not compensate by
   excluding failed starts.
2. Reproduce the current formal profile once and inspect why S1 Oracle can
   produce risk hits but still time out.
3. Run a current-commit single-variable `cost_weight` sweep with at least
   three paired repeats per weight (for example `0, 10, 25, 50`). Keep the
   Reactive configuration unchanged.
4. Require non-zero Oracle risk hits and an earlier control/trajectory change
   to occur repeatedly before interpreting any safety difference.
5. Only after a stable Gate 5 pass may Gate 6 expand to 80+ formal runs.

Do not train a Transformer or write a positive Oracle research conclusion from
this batch. The honest current result is: **the Oracle data path and critic
are operational, but closed-loop dynamic navigation has not passed the Gate 5
stability and causal-benefit acceptance test.**
