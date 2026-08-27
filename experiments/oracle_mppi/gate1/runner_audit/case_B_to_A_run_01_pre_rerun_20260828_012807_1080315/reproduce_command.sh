#!/usr/bin/env bash
set -Eeuo pipefail
cd "/home/w417/RTAB-Map"
./experiments/oracle_mppi/scripts/run_gate1_leg.sh \
  --start-x "8.5" --start-y "0.0" \
  --x "-8.5" --y "0.0" --yaw "3.141592653589793" \
  --profile "reactive_mppi_static" --nav2-params "experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml" --expected-control-period "0.1" --settle-seconds "5.0" \
  --startup-timeout "90" --contact-timeout "420" \
  --label "experiments/oracle_mppi/gate1/case_B_to_A/run_01"
