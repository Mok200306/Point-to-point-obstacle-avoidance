#!/usr/bin/env bash
set -euo pipefail

# Record one real-robot navigation goal. The Python program is an action
# client/recorder only; it never publishes a direct chassis velocity command.

cd "$(dirname "${BASH_SOURCE[0]}")/.."

quoted_args=''
for argument in "$@"; do
  printf -v quoted_argument ' %q' "$argument"
  quoted_args+="$quoted_argument"
done
command_text="source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav real_navigation_trial.py${quoted_args}"
escaped="${command_text//\'/\'\\\'\'}"
printf -v quoted "'%s'" "$escaped"

if docker info >/dev/null 2>&1; then
  docker compose exec -T ros2 bash -lc "$command_text"
else
  sg docker -c "docker compose exec -T ros2 bash -lc $quoted"
fi
