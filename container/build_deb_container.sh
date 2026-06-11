#!/bin/bash
set -euo pipefail

# This script runs INSIDE the px4_msgs-builder container.
# Assumptions:
#   /src        -> px4_msgs source tree (mounted from host)
#   /artifacts  -> output dir (mounted from host, write .deb here)
#   ROS_DISTRO  -> ROS 2 distribution to target (provided by the image)

: "${ROS_DISTRO:?ROS_DISTRO must be set (provided by the builder image)}"

# Debian packaging is Debian/Ubuntu specific. The OS name defaults to ubuntu
# and the OS version is auto-detected by bloom from the base image, so each
# ROS 2 distribution is packaged for its matching Ubuntu release (jammy for
# humble, noble for jazzy/kilted/rolling, ...).
OS_NAME="${OS_NAME:-ubuntu}"

echo "[*] Building px4_msgs Debian package for ROS 2 ${ROS_DISTRO} (${OS_NAME})"

echo "[*] Cleaning previous build artifacts"
cd /src
rm -rf debian obj-* .obj-* CMakeCache.txt CMakeFiles build install log || true

echo "[*] Resolving build dependencies with rosdep"
rosdep install --from-paths . --ignore-src -y --rosdistro "${ROS_DISTRO}"

echo "[*] Generating Debian packaging with bloom"
bloom-generate rosdebian --os-name "${OS_NAME}" --ros-distro "${ROS_DISTRO}"

echo "[*] Building Debian package with fakeroot"
# Skip the test suite while packaging; the linters/tests run in the build CI.
DEB_BUILD_OPTIONS=nocheck fakeroot debian/rules binary

echo "[*] Copying artifacts to /artifacts"
cp /*.deb /artifacts/ 2>/dev/null || true
cp /*.ddeb /artifacts/ 2>/dev/null || true

echo "[*] Done. Artifacts in /artifacts"
