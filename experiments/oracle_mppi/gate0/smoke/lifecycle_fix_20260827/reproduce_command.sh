#!/usr/bin/env bash
set -Eeuo pipefail
cd "/home/w417/RTAB-Map"
./experiments/oracle_mppi/scripts/run_gate0_leg.sh \
  --start-x "-8.5" --start-y "0.0" \
  --x "8.5" --y "0.0" --yaw "0.0" \
  --profile "adaptive_goal_line_045" --settle-seconds "5.0" \
  --startup-timeout "90" --contact-timeout "420" \
  --label "experiments/oracle_mppi/gate0/smoke/lifecycle_fix_20260827"
