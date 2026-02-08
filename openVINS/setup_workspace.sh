#!/bin/bash

# OpenVINS Workspace Setup Script
# This script creates the workspace structure and configures environment variables

set -e

echo "======================================"
echo "OpenVINS Workspace Setup"
echo "======================================"

# Get the current directory (should be the openVINS repo root)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# We'll use the parent directory as the workspace
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Create a catkin workspace structure on the host
CATKIN_WS="$WORKSPACE_DIR/catkin_ws_ov"
mkdir -p "$CATKIN_WS/src"

echo ""
echo "Current OpenVINS directory: $SCRIPT_DIR"
echo "Workspace directory: $WORKSPACE_DIR"
echo "Catkin workspace: $CATKIN_WS"
echo ""

# Create a symbolic link in the workspace if it doesn't exist
if [ ! -L "$CATKIN_WS/src/open_vins" ]; then
    ln -s "$SCRIPT_DIR" "$CATKIN_WS/src/open_vins"
    echo "Created symlink: $CATKIN_WS/src/open_vins -> $SCRIPT_DIR"
fi

# Create datasets directory if it doesn't exist
DATASETS_DIR="$HOME/datasets"
mkdir -p "$DATASETS_DIR"
echo "Datasets directory: $DATASETS_DIR"

# Check if bashrc already has the configuration
if grep -q "DOCKER_OPENVINS" ~/.bashrc; then
    echo ""
    echo "Docker configuration already exists in ~/.bashrc"
    echo "Updating configuration..."
    # Remove old configuration
    sed -i '/# OpenVINS Docker Configuration/,/alias ov_docker=/d' ~/.bashrc
fi

# Add configuration to bashrc
echo ""
echo "Adding Docker configuration to ~/.bashrc..."
cat >> ~/.bashrc << EOF

# OpenVINS Docker Configuration
xhost +local:docker &> /dev/null
export DOCKER_CATKINWS=$CATKIN_WS
export DOCKER_DATASETS=$DATASETS_DIR
alias ov_docker="docker run -it --net=host --gpus all \
    --env=\"NVIDIA_DRIVER_CAPABILITIES=all\" --env=\"DISPLAY\" \
    --env=\"QT_X11_NO_MITSHM=1\" --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" \
    --mount type=bind,source=\$DOCKER_CATKINWS,target=/catkin_ws \
    --mount type=bind,source=\$DOCKER_DATASETS,target=/datasets \$1"
EOF

# Source the updated bashrc
source ~/.bashrc

echo ""
echo "======================================"
echo "Workspace setup complete!"
echo "======================================"
echo ""
echo "Environment variables set:"
echo "  DOCKER_CATKINWS=$CATKIN_WS"
echo "  DOCKER_DATASETS=$DATASETS_DIR"
echo ""
echo "You can now use: ov_docker ov_ros2_22_04 bash"
echo ""
echo "NOTE: The entire workspace will be mounted, so build artifacts"
echo "      (build/, install/, log/) will persist between container restarts."
echo "      You only need to compile once!"
echo ""
echo "To reload the configuration in this terminal, run:"
echo "  source ~/.bashrc"
echo ""
