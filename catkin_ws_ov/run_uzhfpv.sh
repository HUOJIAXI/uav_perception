#!/bin/bash

# Quick launcher for run_uzhfpv_indoor.sh
# This is a shortcut in the workspace root

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$SCRIPT_DIR/src/open_vins/run_uzhfpv_indoor.sh" ]; then
    exec "$SCRIPT_DIR/src/open_vins/run_uzhfpv_indoor.sh" "$@"
else
    echo "Error: Cannot find run_uzhfpv_indoor.sh"
    echo "Expected location: $SCRIPT_DIR/src/open_vins/run_uzhfpv_indoor.sh"
    echo ""
    echo "Please make sure OpenVINS is properly set up."
    exit 1
fi
