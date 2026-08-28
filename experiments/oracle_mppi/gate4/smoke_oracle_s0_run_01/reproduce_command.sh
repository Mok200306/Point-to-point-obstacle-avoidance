#!/usr/bin/env bash
set -Eeuo pipefail
cd "/home/w417/RTAB-Map"
./experiments/oracle_mppi/scripts/run_gate1_leg.sh \
  --start-x "-8.5" --start-y "0.0" \
  --x "8.5" --y "0.0" --yaw "0.0" \
  --profile "oracle_mppi_prediction" --nav2-params "experiments/oracle_mppi/configs/nav2_mppi_oracle_params.yaml" --expected-control-period "0.1" --settle-seconds "5.0" \
  --startup-timeout "110" --contact-timeout "300" \
  --label "experiments/oracle_mppi/gate4/smoke_oracle_s0_run_01" \
  --oracle-scenario "experiments/oracle_mppi/configs/gate4/s0_zero_risk.yaml" \
  --oracle-publisher-config "experiments/oracle_mppi/configs/oracle_publisher_gate3.yaml"
