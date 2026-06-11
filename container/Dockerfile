# Builder image for generating px4_msgs Debian packages with bloom.
# The ROS 2 distribution is selectable so packages can be produced for every
# distribution on the build farm (humble, jazzy, kilted, rolling, ...).
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-core

# Redeclare after FROM so it is available in the build stage.
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

# The tooling required to resolve dependencies and generate/build the Debian
# package. build-essential provides the C/C++ compiler the message typesupport
# build needs; no runtime middleware is needed for an interface package.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  python3-bloom \
  python3-rosdep \
  build-essential \
  cmake \
  fakeroot \
  debhelper \
  dh-python \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

WORKDIR /work

COPY container/build_deb_container.sh /usr/local/bin/build.sh
RUN chmod +x /usr/local/bin/build.sh
