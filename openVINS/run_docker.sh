#!/bin/bash

# OpenVINS Docker Run Script
# Convenience script to start the OpenVINS Docker container

VERSION=ros2_22_04
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
CATKIN_WS="$WORKSPACE_DIR/catkin_ws_ov"
DATASETS_DIR="$HOME/datasets"

# Create workspace directories
mkdir -p "$CATKIN_WS/src"
mkdir -p "$CATKIN_WS/build"
mkdir -p "$CATKIN_WS/install"
mkdir -p "$CATKIN_WS/log"

# Enable X11 forwarding
xhost +local:docker > /dev/null 2>&1

echo "Starting OpenVINS Docker container..."
echo "Image: ov_$VERSION"
echo ""
echo "Mounted directories:"
echo "  OpenVINS:  $SCRIPT_DIR -> /catkin_ws/src/open_vins"
echo "  Workspace: $CATKIN_WS -> /catkin_ws (build artifacts)"
echo "  Datasets:  $DATASETS_DIR -> /datasets"
echo ""
echo "Build artifacts will persist on the host!"
echo ""

# Set ROS_DOMAIN_ID if not already set (for ROS2 topic communication)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Set ROS_LOCALHOST_ONLY to fix DDS discovery issues
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}

# Run the docker container
# Mount OpenVINS source directly + workspace for build artifacts
docker run -it --rm --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
    --env="ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY" \
    --ipc=host \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --mount type=bind,source=$SCRIPT_DIR,target=/catkin_ws/src/open_vins \
    --mount type=bind,source=$CATKIN_WS/build,target=/catkin_ws/build \
    --mount type=bind,source=$CATKIN_WS/install,target=/catkin_ws/install \
    --mount type=bind,source=$CATKIN_WS/log,target=/catkin_ws/log \
    --mount type=bind,source=$DATASETS_DIR,target=/datasets \
    ov_$VERSION bash
