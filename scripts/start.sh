#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
export LOCAL_UID="${LOCAL_UID:-$(id -u)}"
export LOCAL_GID="${LOCAL_GID:-$(id -g "$(id -un)")}"

mkdir -p .ros .gazebo .config .rviz2

# Permit the same local UID inside the container to open Gazebo/RViz.
xhost +si:localuser:"$(id -un)" >/dev/null 2>&1 || true

docker compose up -d
docker compose ps
