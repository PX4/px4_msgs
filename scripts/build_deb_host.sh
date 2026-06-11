#!/bin/bash
set -euo pipefail

# Build px4_msgs Debian packages locally using the bundled builder container.
#
# Usage:
#   ./scripts/build_deb_host.sh            # builds for humble (default)
#   ROS_DISTRO=jazzy ./scripts/build_deb_host.sh
#
# The resulting *.deb files are written to ./out

# Location of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( realpath "${SCRIPT_DIR}/.." )"

# Source dir is the repo root (px4_msgs checkout)
SRC_DIR="${REPO_ROOT}"

# Output dir for built .deb files
OUT_DIR="${REPO_ROOT}/out"

# ROS 2 distribution to build for (default: humble)
ROS_DISTRO="${ROS_DISTRO:-humble}"
IMAGE="px4io/px4_msgs-builder:${ROS_DISTRO}"

mkdir -p "${OUT_DIR}"

echo "[*] Building px4_msgs .deb packages"
echo "[*] ROS distro: ${ROS_DISTRO}"
echo "[*] Source:     ${SRC_DIR}"
echo "[*] Artifacts:  ${OUT_DIR}"

echo "[*] Building builder image ${IMAGE}"
docker build \
    --build-arg "ROS_DISTRO=${ROS_DISTRO}" \
    -t "${IMAGE}" \
    -f "${REPO_ROOT}/container/Dockerfile" \
    "${REPO_ROOT}"

echo "[*] Running the build in a container"
docker run --rm \
    -v "${SRC_DIR}":/src \
    -v "${OUT_DIR}":/artifacts \
    "${IMAGE}" \
    /usr/local/bin/build.sh

echo "[*] Done. Check ${OUT_DIR} for .deb files"
