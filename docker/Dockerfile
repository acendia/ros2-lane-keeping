# Development environment for auto-drive-sim.
#
# Ubuntu 24.04 ARM64 + ROS 2 Jazzy + Gazebo Harmonic (ROS vendor packages).
# The plan targets an Ubuntu VM; this image is the documented Docker fallback.
# Gazebo is installed through `ros-jazzy-ros-gz` vendor packages, which is the
# official pairing for ROS 2 Jazzy. Do not add `gz-harmonic` from
# packages.osrfoundation.org on top of these packages.

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    LIBGL_ALWAYS_SOFTWARE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      gnupg \
      lsb-release \
      software-properties-common \
      build-essential \
      cmake \
      git \
      python3 \
      python3-pip \
      python3-venv \
      python3-opencv \
      python3-numpy \
      python3-pytest \
      libgl1-mesa-dri \
      libegl1 \
      libgles2 \
      libglu1-mesa \
      mesa-utils \
      xvfb \
    && add-apt-repository -y universe \
    && rm -rf /var/lib/apt/lists/*

# Official ROS 2 apt source package (pinned for reproducibility).
ARG ROS_APT_SOURCE_VERSION=1.3.0
RUN curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-tools-vendor \
      ros-jazzy-gz-sim-vendor \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-transport \
      ros-jazzy-image-view \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN printf '%s\n' \
      '#!/bin/bash' \
      'set -e' \
      'source /opt/ros/jazzy/setup.bash' \
      'exec "$@"' \
      > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

WORKDIR /workspace
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
