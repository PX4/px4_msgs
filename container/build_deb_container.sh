#!/bin/bash
set -euo pipefail

# This script runs INSIDE the container.
# Assumptions:
#   /src        -> px4_msgs source tree (mounted from host)
#   /artifacts  -> output dir (mounted from host, write .deb here)

echo "[*] Cleaning previous build artifacts"
cd /src
rm -rf debian .obj-* CMakeCache.txt CMakeFiles build install log || true

echo "[*] Generating Debian packaging with bloom"
bloom-generate rosdebian

echo "[*] Building Debian package with fakeroot"
fakeroot debian/rules binary

echo "[*] Copying artifacts to /artifacts"
cp /*.deb /artifacts/ 2>/dev/null || true
cp /*.ddeb /artifacts/ 2>/dev/null || true

echo "[*] Done. Artifacts in /artifacts"
