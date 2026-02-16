#!/bin/bash
# cmake -Bbuild -H. -D_MAVLINK_INCLUDE_DIR=/workspaces/px4-gzsim-plugins/mavlink/install -DCMAKE_EXPORT_COMPILE_COMMANDS=ON#!/bin/bash
set -euo pipefail

BUILD_DIR=build
INSTALL_DIR=/workspaces/px4-gzsim-plugins/install
MAVLINK_DIR=/workspaces/px4-gzsim-plugins/mavlink/install

# Configure
cmake -S . -B "${BUILD_DIR}" \
  -D_MAVLINK_INCLUDE_DIR="${MAVLINK_DIR}" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"

# Build
cmake --build "${BUILD_DIR}" --parallel

# Install
cmake --install "${BUILD_DIR}"

echo "Build complete. Shared libraries:"
find "${BUILD_DIR}" -name '*.so' -type f
echo "Installed libraries:"
find "${INSTALL_DIR}" -name '*.so' -type f 2>/dev/null || echo "No files installed yet"
