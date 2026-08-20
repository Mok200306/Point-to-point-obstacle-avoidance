#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

workspace=/workspaces/rtabmap_tb3_nav
mkdir -p "${HOME:-/tmp/ros_home}" "${ROS_HOME:-/tmp/ros_home/.ros}"

# The source tree is bind-mounted in development. Build it once on first start.
if [[ -f "${workspace}/src/rtabmap_tb3_nav/package.xml" && \
      ! -f "${workspace}/install/rtabmap_tb3_nav/share/rtabmap_tb3_nav/package.xml" ]]; then
  cd "${workspace}"
  colcon build --symlink-install --event-handlers console_direct+
fi

if [[ -f "${workspace}/install/setup.bash" ]]; then
  set +u
  source "${workspace}/install/setup.bash"
  set -u
fi

exec "$@"
