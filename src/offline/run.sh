#!/bin/bash
# Run script for Super-LIO offline processing
# Usage: ./run.sh <bag_path> <config_path>
# Example: ./run.sh /path/to/bag ../config/livox_360.yaml

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
EXECUTABLE="${BUILD_DIR}/run_offline"

# Check if executable exists
if [ ! -f "${EXECUTABLE}" ]; then
    echo "Error: Executable not found. Please run ./build.sh first."
    exit 1
fi

# Check arguments
if [ -z "$1" ]; then
    echo "Usage: $0 <bag_path> <config_path>"
    echo "Example: $0 /path/to/bag ../config/livox_360.yaml"
    echo ""
    echo "Available bags in parent directories:"
    find "${SCRIPT_DIR}/../.." -name "metadata.yaml" -exec dirname {} \; 2>/dev/null | head -10
    exit 1
fi

BAG_PATH="$1"
CONFIG_PATH="${2:-${SCRIPT_DIR}/config/offline.yaml}"

# Check bag path
if [ ! -d "${BAG_PATH}" ] && [ ! -f "${BAG_PATH}" ]; then
    echo "Error: Bag path not found: ${BAG_PATH}"
    exit 1
fi

# Check config file
if [ ! -f "${CONFIG_PATH}" ]; then
    echo "Error: Config file not found: ${CONFIG_PATH}"
    exit 1
fi

echo "Running Super-LIO offline..."
echo "  Bag: ${BAG_PATH}"
echo "  Config: ${CONFIG_PATH}"
echo ""

cd "${SCRIPT_DIR}"
"${EXECUTABLE}" --input_bag="${BAG_PATH}" --config="${CONFIG_PATH}"

echo ""
echo "Done!"