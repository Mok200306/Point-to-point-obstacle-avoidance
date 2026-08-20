FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    TURTLEBOT3_MODEL=waffle \
    QT_X11_NO_MITSHM=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-nav2-bringup \
    ros-humble-nav2-collision-monitor \
    ros-humble-navigation2 \
    ros-humble-pcl-ros \
    ros-humble-realsense2-camera \
    ros-humble-rtabmap-ros \
    ros-humble-rqt-image-view \
    ros-humble-teleop-twist-keyboard \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-teleop \
    ros-humble-xacro \
    mesa-utils \
    x11-xserver-utils \
    && rm -rf /var/lib/apt/lists/*

# Compose runs the container with the host UID. Keep a passwd entry for the
# common Ubuntu desktop UID so Gazebo can resolve the runtime username.
RUN if ! getent passwd 1000 >/dev/null; then \
      if getent group 1000 >/dev/null; then \
        runtime_group="$(getent group 1000 | cut -d: -f1)"; \
      else \
        groupadd --gid 1000 rosuser; \
        runtime_group=rosuser; \
      fi; \
      useradd --uid 1000 --gid "${runtime_group}" --home-dir /tmp/ros_home \
        --shell /bin/bash --no-create-home rosuser; \
    fi

COPY scripts/patch_turtlebot3_rgbd.sh /usr/local/bin/patch_turtlebot3_rgbd.sh
RUN chmod +x /usr/local/bin/patch_turtlebot3_rgbd.sh \
    && /usr/local/bin/patch_turtlebot3_rgbd.sh

RUN mkdir -p /workspaces/rtabmap_tb3_nav

COPY scripts/entrypoint.sh /usr/local/bin/rtabmap_tb3_entrypoint.sh
RUN chmod +x /usr/local/bin/rtabmap_tb3_entrypoint.sh

ENTRYPOINT ["/usr/local/bin/rtabmap_tb3_entrypoint.sh"]
CMD ["bash", "-lc", "tail -f /dev/null"]
