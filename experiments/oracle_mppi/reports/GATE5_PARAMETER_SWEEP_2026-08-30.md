# Gate 5 Parameter Sweep Review

Date: 2026-08-30
Scope: PredictionCritic `cost_weight` diagnostic sweep
Formal Gate 5 status: **BLOCKED**

## 1. Purpose and limitation

The sweep was intended to find whether the Oracle risk cost was so strong that
it caused freezing, replanning churn, or loss of progress. It changed only the
Oracle `PredictionCritic.cost_weight`; Reactive was rerun with its unchanged
frozen MPPI configuration for each pair.

The sweep is **diagnostic, not a frozen parameter selection**:

- each weight has only one Reactive/Oracle pair;
- it used the earlier commit `a7c3571025856af4d5ab5f5ea532866be9bddbd2`, not
  the formal Gate 5 commit;
- it used `S2_gate5_conflict` and `gate5_open_corridor.world`, not the formal
  S1/S2 batch world;
- therefore it cannot establish repeatability, significance, or a Gate 5
  acceptance result.

The original evidence is under
`experiments/oracle_mppi/gate5/cost_sweep_20260829_01/`. It was re-summarized
with the corrected status-aware summarizer; the six-row result is
`gate5_smoke_summary_20260830_reanalyzed.csv` and the paired table is
`gate5_paired_summary_20260830_reanalyzed.csv`.

## 2. Sweep matrix

| Weight | Reactive outcome | Oracle outcome | Reactive sim [s] | Oracle sim [s] | Reactive path [m] | Oracle path [m] | Interpretation |
|---:|---|---|---:|---:|---:|---:|---|
| 0 | PASS | PASS | 76.2 | 75.8 | 17.610 | 17.601 | no measurable future-risk influence expected |
| 10 | PASS | PASS | 77.3 | 189.9 | 17.903 | 22.664 | Oracle completed but was much slower/longer |
| 50 | NAV_FAILURE | PASS | 66.9 | 79.1 | 13.761 | 17.615 | Reactive failure makes this pair non-diagnostic for Oracle superiority |

All six runs recorded zero dynamic contacts in this small sweep. That is a
useful safety observation for these particular runs, but it does not overcome
the single-repeat and old-commit limitations.

## 3. Parameter interpretation

`cost_weight=0` is the correct control condition: the Oracle plugin path may
be present, but its risk term contributes no cost. It is not evidence that
future information helps.

At `cost_weight=10`, the Oracle run took `189.9 s` simulation time compared
with `77.3 s` for its paired Reactive run and followed a `22.664 m` path
instead of `17.903 m`. This is consistent with a cost term that influences
candidate selection, but it is also consistent with over-conservative or
poorly timed risk interaction. It should not be called an improvement.

At `cost_weight=50`, Oracle completed in `79.1 s`, but its paired Reactive
run failed with Nav2 status `6`; the pair cannot isolate the effect of the
weight. The formal batch also showed two Oracle timeouts at weight `50`, so
the current value is not safe to freeze as a production setting.

## 4. What the sweep proves and does not prove

It proves:

- the Oracle configuration can be launched at weights 0, 10, and 50;
- the critic cost can affect the selected run behavior;
- the runner can preserve paired results for a parameter sweep.

It does not prove:

- that any weight improves collision rate or minimum clearance;
- that `10` or `50` is robust;
- that the Oracle caused an earlier control response;
- that a result transfers from the open-corridor diagnostic world to the
  formal S1/S2 world.

## 5. Required next sweep

Before choosing a frozen weight, run a new current-commit sweep with the same
formal world and scenario protocol. Keep Reactive exactly unchanged. Use at
least three paired repeats per value, for example:

```text
cost_weight ∈ {0, 10, 25, 50}
scenarios   = S1_crossing, S2_oncoming
repeats     >= 3 paired runs per scenario and weight
```

For every run, compare:

- Nav2 goal result and runner exit code;
- Gazebo dynamic contacts;
- minimum robot–obstacle clearance;
- first future conflict time;
- first proactive slowdown time;
- stop and reverse ratios;
- simulation time and path length;
- number of progress/planner/control-period failures;
- Oracle active/stale/risk-hit diagnostics.

The selection rule should be frozen before looking at the new results:

1. discard any value with startup failure, controller stuck/timeout above the
   Gate threshold, or dynamic contact;
2. among remaining values, require repeatable pre-conflict behavior in the
   Oracle group;
3. prefer the lowest median time only after safety and completion criteria are
   satisfied;
4. record the chosen value in a new dated snapshot and rerun the complete
   Gate 5 acceptance batch.

Until that is done, retain `cost_weight=50.0` as the current formal snapshot
for reproducibility only, not as a validated optimum.
