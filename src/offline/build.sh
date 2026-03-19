#!/bin/bash
# Build script for Super-LIO offline processing
# Usage: ./build.sh [clean]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

# Clean if requested
if [ "$1" == "clean" ]; then
    echo "Cleaning build directory..."
    rm -rf "${BUILD_DIR}"
    echo "Build directory cleaned."
    exit 0
fi

# Create build directory
mkdir -p "${BUILD_DIR}"

# Run cmake
echo "Running cmake..."
cd "${BUILD_DIR}"
cmake .. \
    -DCMAKE_PREFIX_PATH="${AMENT_PREFIX_PATH}"

# Build
echo "Building..."
make -j$(nproc)

echo ""
echo "Build complete! Executable: ${BUILD_DIR}/run_offline"