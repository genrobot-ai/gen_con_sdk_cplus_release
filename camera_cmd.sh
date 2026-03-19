#!/bin/bash
#
# Camera calibration command script
# Usage: ./camera_cmd.sh [left|right] <command>
#
# Optional first argument:
#   left   - use /dev/ttyDeviceLeft (default if omitted)
#   right  - use /dev/ttyDeviceRight
# Override with env: SERIAL_PORT=/dev/...
#
# Commands (YAML names include device side: _left or _right):
#   camerarc  - Center camera calibration -> cam0_sensor_<left|right>.yaml
#   camerarl  - Left camera calibration   -> cam1_sensor_<left|right>.yaml
#   camerarr  - Right camera calibration  -> cam2_sensor_<left|right>.yaml
#   1234      - Calibration complete confirmation
#   MCUID     - Query device MCUID
#
# Examples:
#   ./camera_cmd.sh MCUID
#   ./camera_cmd.sh right MCUID
#   ./camera_cmd.sh left camerarc




#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
EXECUTABLE="${BUILD_DIR}/camera_cmd"

# Check if executable exists
if [ ! -f "${EXECUTABLE}" ]; then
    echo "Error: Executable not found: ${EXECUTABLE}"
    echo ""
    echo "Please build the project first:"
    echo "  cd ${SCRIPT_DIR}"
    echo "  mkdir -p build && cd build && cmake .. && make"
    exit 1
fi

# Run with all arguments
exec "${EXECUTABLE}" "$@"
