#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
export LOCAL_UID="${LOCAL_UID:-$(id -u)}"
export LOCAL_GID="${LOCAL_GID:-$(id -g "$(id -un)")}"

docker compose exec ros2 bash -lc '
  source /opt/ros/humble/setup.bash
  if [[ -f /workspaces/rtabmap_tb3_nav/install/setup.bash ]]; then
    source /workspaces/rtabmap_tb3_nav/install/setup.bash
  fi
  exec bash
'
