#!/usr/bin/env bash
set -euo pipefail

# The official Humble RGB-D demo changes the TurtleBot3 camera SDF from a
# colour camera to a depth camera. This patch also removes the physical LDS
# link and ray sensor so the simulation has no real LiDAR topic or geometry.

model_file="${1:-/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf}"
urdf_file="${2:-/opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf}"

[[ -f "${model_file}" ]] || { echo "Missing Gazebo model: ${model_file}" >&2; exit 1; }
[[ -f "${urdf_file}" ]] || { echo "Missing robot URDF: ${urdf_file}" >&2; exit 1; }

# The upstream TurtleBot3 files are commonly distributed with CRLF endings.
# Normalize them before applying the multiline XML edits.
perl -pi -e 's/\r$//' "${model_file}" "${urdf_file}"

if grep -q 'turtlebot3_laserscan' "${model_file}"; then
  perl -0pi -e '
    s/\n    <link name="base_scan">.*?<\/link>\n//s;
    s/\n    <joint name="lidar_joint"[^>]*>.*?<\/joint>\n//s;
    s/<link name="camera_rgb_frame">/<link name="camera_rgb_optical_frame">/;
    s/(    <link name="camera_rgb_optical_frame">)/    <link name="camera_rgb_frame"\/>\n\n$1/;
    s/<sensor name="camera" type="camera">/<sensor name="camera" type="depth">/;
    # The lower stream size leaves CPU headroom for RTAB-Map, Nav2 and the
    # safety monitor while retaining enough depth detail for this course.
    s/<width>1920<\/width>/<width>320<\/width>/;
    s/<height>1080<\/height>/<height>240<\/height>/;
    s/<update_rate>30<\/update_rate>/<update_rate>15<\/update_rate>/;
    s/<visualize>true<\/visualize>/<visualize>false<\/visualize>/;
    s{(    <joint name="camera_rgb_joint"[^>]*>.*?    </joint>)}{$1

    <joint name="camera_rgb_optical_joint" type="fixed">
      <parent>camera_rgb_frame</parent>
      <child>camera_rgb_optical_frame</child>
      <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>}s;
  ' "${model_file}"
fi

# Route the simulated Gazebo model through nav2_collision_monitor. The monitor
# reads the RGB-D obstacle cloud and publishes /cmd_vel_safe after applying
# stop and slowdown zones. The separate URDF is only used by
# robot_state_publisher and intentionally has no Gazebo drive plugin.
perl -pi -e 's{<command_topic>cmd_vel</command_topic>}{<command_topic>cmd_vel_safe</command_topic>}' \
  "${model_file}"
perl -0pi -e 's{(<plugin name="turtlebot3_diff_drive"[^>]*>\s*<ros>)}{$1\n        <remapping>cmd_vel:=cmd_vel_safe</remapping>}s' \
  "${model_file}"

# The Gazebo robot_state_publisher URDF is a separate copy from the model SDF.
# Keep its TF tree consistent with the LiDAR-free model.
if grep -q '<joint name="scan_joint"' "${urdf_file}"; then
  perl -0pi -e '
    s/\n  <joint name="scan_joint"[^>]*>.*?<\/joint>\n//s;
    s/\n  <link name="base_scan">.*?<\/link>\n//s;
  ' "${urdf_file}"
fi

if grep -Eq 'turtlebot3_laserscan|base_scan|lidar_joint' "${model_file}" || grep -Eq '<joint name="scan_joint"|base_scan' "${urdf_file}"; then
  echo "TurtleBot3 RGB-D patch was not applied completely" >&2
  exit 1
fi

if ! grep -q '<command_topic>cmd_vel_safe</command_topic>' "${model_file}"; then
  echo "TurtleBot3 safety command topic patch was not applied" >&2
  exit 1
fi

if ! grep -q '<remapping>cmd_vel:=cmd_vel_safe</remapping>' "${model_file}"; then
  echo "TurtleBot3 safety command remap was not applied" >&2
  exit 1
fi

echo "Patched TurtleBot3 waffle for RGB-D-only navigation: ${model_file}"
