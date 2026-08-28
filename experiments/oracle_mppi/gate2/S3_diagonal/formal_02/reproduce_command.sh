#!/usr/bin/env bash
set -Eeuo pipefail
cd "/home/w417/RTAB-Map"
./experiments/oracle_mppi/scripts/run_gate2_scene.sh \
  --scenario "experiments/oracle_mppi/configs/scenarios/s3_diagonal.yaml" --difficulty "medium" \
  --profile "reactive_mppi_static" --nav2-params "experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml" \
  --expected-control-period "0.1" \
  --settle-seconds "5.0" --startup-timeout "90" \
  --contact-timeout "420" \
  --label "experiments/oracle_mppi/gate2/S3_diagonal/formal_02"
