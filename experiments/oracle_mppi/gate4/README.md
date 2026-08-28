# Gate 4: PredictionCritic and zero-risk Oracle regression

This directory records the Gate 4 implementation, offline tests, pluginlib
lifecycle loading, and zero-risk Oracle static regression required by the
execution task book. Gate 4 verifies that future occupancy can be connected to
MPPI and sampled with the correct time alignment. It does not claim that
Oracle has improved dynamic navigation; that question belongs to Gate 5/6.

## Frozen interface

- Oracle topic: `/oracle/predicted_occupancy`
- frame: `odom`
- grid: `120 x 100` cells, `0.05 m/cell`, physical size `6.0 x 5.0 m`
- prediction `dt`: `0.10 s`
- horizon: `3.0 s`, `31` layers
- temporal interpolation: `linear`
- time alignment: `tau_k = (t_eval - t_msg) + k * model_dt`
- stale threshold: `0.30 s`
- out-of-bounds and out-of-horizon samples: ignored and counted safely

## Build and offline acceptance

```bash
cd /home/w417/RTAB-Map
docker compose up -d
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash &&
   cd /workspaces/rtabmap_tb3_nav &&
   colcon build --symlink-install'

docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash &&
   cd /workspaces/rtabmap_tb3_nav &&
   colcon test --packages-select nav2_mppi_prediction_critic \
     --event-handlers console_direct+ &&
   colcon test-result --verbose'

docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash &&
   source /workspaces/rtabmap_tb3_nav/install/setup.bash &&
   /workspaces/rtabmap_tb3_nav/install/nav2_mppi_prediction_critic/lib/
   nav2_mppi_prediction_critic/prediction_critic_offline_test \
   --output /workspaces/rtabmap_tb3_nav/experiments/oracle_mppi/gate4/critic_debug.csv'
```

## One zero-risk Oracle smoke run

`configs/gate4/s0_zero_risk.yaml` places the deterministic Oracle proxy at
`(100, 100)`, outside the local costmap. Consequently the PredictionCritic
must report zero risk and zero cost while the publisher, message interface,
pluginlib lifecycle, and navigation chain are real.

```bash
./experiments/oracle_mppi/scripts/run_gate1_leg.sh \
  --start-x -8.5 --start-y 0.0 --x 8.5 --y 0.0 --yaw 0.0 \
  --profile oracle_mppi_prediction \
  --nav2-params experiments/oracle_mppi/configs/nav2_mppi_oracle_params.yaml \
  --oracle-scenario experiments/oracle_mppi/configs/gate4/s0_zero_risk.yaml \
  --oracle-publisher-config experiments/oracle_mppi/configs/oracle_publisher_gate3.yaml \
  --label experiments/oracle_mppi/gate4/zero_risk/oracle_run_01
```

The Reactive control uses the same world, start, goal, and MPPI snapshot but
does not start the Oracle publisher:

```bash
./experiments/oracle_mppi/scripts/run_gate1_leg.sh \
  --start-x -8.5 --start-y 0.0 --x 8.5 --y 0.0 --yaw 0.0 \
  --profile reactive_mppi_static \
  --nav2-params experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml \
  --label experiments/oracle_mppi/gate4/zero_risk/reactive_run_01
```

Formal results use `reactive_run_01..03` and `oracle_run_01..03`. The runner
refuses to overwrite a non-empty evidence directory; failed runs remain in
place.

## Evidence checks

Each complete run should contain `metrics.yaml`, `experiment.yaml`,
`trajectory.csv`, `gazebo_trajectory.csv`, `trajectory_comparison.png`, the
parameter/world snapshots, logs, and contact evidence. Oracle runs additionally
contain `oracle_publisher.log`, `oracle_topic_info.txt`, and
`oracle_message_validation.txt`.

Important fields are:

- `succeeded: true`
- `gazebo_non_ground_contact: false`
- Oracle `oracle_message_validation_exit: 0`
- Oracle `oracle_active_log_lines > 0`
- Oracle `PredictionCritic` logs with `max_risk=0` and `max_cost=0`

Create the non-destructive summary with:

```bash
python3 experiments/oracle_mppi/scripts/summarize_gate4.py
```

The script writes `gate4/zero_risk_summary.csv` and never removes or overwrites
run directories.
