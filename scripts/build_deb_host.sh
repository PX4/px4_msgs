#!/bin/bash
set -euo pipefail

# Location of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( realpath "${SCRIPT_DIR}/.." )"

# Source dir is the repo root (px4_msgs checkout)
SRC_DIR="${REPO_ROOT}"

# Output dir for built .deb files
OUT_DIR="${REPO_ROOT}/out"

mkdir -p "${OUT_DIR}"

echo "[*] Building px4_msgs .deb packages using container px4/px4_msgs-builder"
echo "[*] Source:    ${SRC_DIR}"
echo "[*] Artifacts: ${OUT_DIR}"

docker run --rm -it \
    -v "${SRC_DIR}":/src \
    -v "${OUT_DIR}":/artifacts \
    px4io/px4_msgs-builder \
    /usr/local/bin/build.sh

echo "[*] Done. Check ${OUT_DIR} for .deb files"
