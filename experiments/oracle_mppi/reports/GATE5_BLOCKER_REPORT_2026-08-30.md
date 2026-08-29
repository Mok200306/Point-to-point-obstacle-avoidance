# Gate 5 Blocker Report

Date: 2026-08-30
Branch: `exp/oracle-g5-closed-loop-2026-08-29`
Formal batch: `experiments/oracle_mppi/gate5/formal_20260830_01/`
Commit: `bf47951818887ddd7052bed71b945c74117ce675`

## Executive conclusion

Gate 5 is **BLOCKED**, with two different classes of blocker:

1. **Orchestration reliability:** three of twenty planned runs never reached
   a usable Nav2 lifecycle state (`S2 Reactive run 01`, `S2 Oracle run 03`,
   `S2 Oracle run 05`).
2. **Closed-loop navigation stability/causality:** Oracle timed out twice in
   S1, Reactive had two real S2 dynamic contacts, and the formal logs do not
   show a repeatable Oracle-before-conflict control benefit.

The correct next step is targeted diagnosis and a fresh current-commit smoke
matrix. It is not Gate 6, Transformer training, or a broad multi-parameter
rewrite.

## 1. Failure ledger

| Run | Failure class | Evidence | Effect on Gate |
|---|---|---|---|
| `S2_oncoming/reactive_run_01` | Nav2 startup failure | `startup_readiness.txt`: controller/planner inactive, BT navigator unconfigured | planned run failure |
| `S2_oncoming/oracle_run_03` | Nav2 startup failure | `startup_readiness.txt`: controller/planner inactive, BT navigator unconfigured | planned run failure |
| `S2_oncoming/oracle_run_05` | Nav2 startup failure | `startup_readiness.txt`: controller/planner inactive, BT navigator unconfigured | planned run failure |
| `S1_crossing/oracle_run_03` | navigation timeout | Nav2 status 5, 300 s goal timeout, no dynamic contact | Oracle completion failure |
| `S1_crossing/oracle_run_04` | navigation timeout | Nav2 status 5, 300 s goal timeout, no dynamic contact; repeated no-path logs | Oracle completion failure |
| `S2_oncoming/reactive_run_02` | arrived with physical contact | Gazebo pair contains waffle base collision and dynamic obstacle collision | hard safety failure |
| `S2_oncoming/reactive_run_04` | timeout with physical contact | same type of Gazebo pair, Nav2 status 5 | hard safety and completion failure |

The failed directories and their evidence remain intact. No failure was
deleted or removed from the denominator.

## 2. Why the startup failures matter

The prior summarizer searched only for `gate5_analysis.yaml`. That file is
created after a navigation run starts, so startup failures were invisible to
the summary. This continuation fixed the accounting:

- run directories are discovered from `experiment.yaml`/`scenario.yaml`;
- `matrix_status.csv` supplies the runner exit code and timestamps;
- missing metrics are classified as `STARTUP_FAILURE`, not as a successful
  empty run;
- the formal summary now has 20 run rows and 10 paired rows.

The corrected summary files are:

```text
experiments/oracle_mppi/gate5/formal_20260830_01/gate5_smoke_summary.csv
experiments/oracle_mppi/gate5/formal_20260830_01/gate5_paired_summary.csv
```

The three startup-failure readiness snapshots consistently show the launch
process was present, but the required lifecycle activation did not complete.
This is an experiment-runner/system-startup problem and must be fixed or
isolated before the controller comparison can be interpreted statistically.

## 3. Why the dynamic contacts matter

The two Reactive S2 contact runs contain a concrete collision pair:

```text
waffle::base_link::base_collision
oracle_dynamic_obstacle::link::collision
```

Run 02 eventually returned Nav2 status `4`, but physical arrival does not
override a collision. Run 04 both collided and timed out. This demonstrates
that the present Reactive baseline is not a zero-collision dynamic baseline in
this scenario. The contact evidence is independent of the RGB-D map display
and is not a plotting artifact.

## 4. Why the Oracle result is not yet a positive result

The Oracle side has useful evidence:

- all eight Oracle runs that reached the publisher/critic stage passed the
  message validation step;
- the critic produced `status=active` diagnostics;
- no Oracle run recorded a dynamic contact.

However, the formal batch does not demonstrate stable predictive control:

- S1 Oracle runs 03 and 04 timed out;
- S2 Oracle runs 03 and 05 failed before navigation startup;
- only S1 Oracle runs 01 and 03 showed non-zero risk-hit diagnostics in the
  saved throttled logs;
- S2 completed Oracle runs showed active plumbing but zero sampled risk hits;
- risk hits, when observed, did not consistently map to an earlier slowdown
  and successful completion.

Consequently, “no Oracle collision” is a weak observation over the completed
runs, not proof that Oracle improved safety. It may partly reflect different
trajectories, missed temporal overlap, conservative stopping, or the fact
that the candidate trajectories did not sample the predicted obstacle.

## 5. Likely technical hypotheses, ranked

These are hypotheses to test, not established causes:

### H1 — Risk scale or spatial/temporal overlap is poorly calibrated

The formal Oracle configuration uses `cost_weight=50.0`. Some runs report
non-zero risk cost up to about `1388.5`, while other runs have no risk hits at
all. A weight that is too strong can make the optimizer reject useful
trajectories or stall; a weight that is too weak or spatially missed cannot
change behavior. The difference between “no risk hit” and “large risk cost”
must be explained before freezing the weight.

### H2 — MPPI and planner computation are not consistently meeting their rates

Across the formal batch, logs contain repeated `Failed to make progress`,
missed 10 Hz control-rate warnings, planner no-path messages, and BT tick-rate
overruns. These counts are not themselves a root-cause proof, but they are
consistent with a controller/planner under load or with a costmap state that
temporarily has no valid route.

### H3 — Replanning and map/costmap updates can temporarily invalidate the route

Oracle S1/04 contains repeated no-valid-path messages, while Reactive S2/02
and S2/04 contain planner failures around a physical conflict. The current
system uses a rolling RGB-D/RTAB-Map obstacle representation plus dynamic
world truth. A transiently blocked or stale local/global costmap can turn a
recoverable conflict into a stall.

### H4 — Startup lifecycle is a separate cold-start race

The three missing-metrics runs failed before the experiment could collect
trajectory evidence. They should be reproduced with a startup-only smoke
test; tuning `PredictionCritic` cannot repair this class of failure.

## 6. Evidence/infrastructure changes made now

Only evidence handling was changed after the formal run:

```text
experiments/oracle_mppi/scripts/summarize_gate5.py
experiments/oracle_mppi/scripts/run_gate5_cost_sweep.sh
```

The navigation YAMLs, world, Oracle critic implementation, and formal result
data were not changed in this continuation. New output fields include:

```text
matrix_exit_code
trial_exit_code
startup_failure
nav2_succeeded
goal_timed_out
acceptance_pass
run_outcome
```

This separates runner process status, Nav2 goal status, physical contacts,
and evidence completeness.

## 7. Recovery plan

### Step A — startup-only reliability

Run the same launch protocol repeatedly without changing navigation
parameters. Capture lifecycle states and determine whether the failure is
container/DDS/Gazebo startup ordering. A run that never reaches active Nav2
must remain a failure in the matrix.

### Step B — current-commit Oracle smoke

Use a new evidence root. Do not overwrite the formal batch. Run one paired S1
and S2 test first, then three paired repeats if startup is stable. Confirm:

- Oracle publisher ready and validation exit `0`;
- PredictionCritic active;
- non-zero risk hits at the designed conflict window;
- control changes before the closest approach;
- no timeout/stuck behavior.

### Step C — one-variable cost sweep

On the current commit, use the formal S1/S2 world and a new root. Try
`cost_weight` values such as `0, 10, 25, 50`, with at least three paired
repeats per value. Keep Reactive unchanged. Freeze the decision rule before
looking at the results.

### Step D — rerun the full Gate 5 batch

Only after startup and smoke behavior are stable, repeat the required S1/S2
5+5 per method batch. A new batch must still count all failures and must use a
new immutable root.

## 8. Gate transition rule

Do not move to Gate 6 until all of the following are demonstrated in a fresh
batch:

- no unexplained startup failures;
- Oracle reaches the goal without dynamic contacts at the required repeat
  rate;
- at least one scenario shows repeatable pre-conflict Oracle control change;
- Oracle does not exceed the task-book stuck/timeout allowance;
- the chosen cost weight and temporal interpolation are frozen in a dated
  configuration snapshot.

Until then, the current research status is:

```text
Gate 3 interface: PASS
Gate 4 PredictionCritic and zero-risk integration: PASS
Gate 5 closed-loop dynamic benefit: BLOCKED
Transformer stage: NOT AUTHORIZED
```
