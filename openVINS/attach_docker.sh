#!/bin/bash

# OpenVINS Docker Attach Script
# Use this to open additional terminals in an EXISTING Docker container
# This ensures all terminals share the same ROS2 environment

echo "======================================"
echo "OpenVINS Docker Attach"
echo "======================================"
echo ""

VERSION=ros2_22_04

# Find running OpenVINS container
CONTAINER=$(docker ps --filter "ancestor=ov_$VERSION" --format "{{.ID}}" | head -1)

if [ -z "$CONTAINER" ]; then
    echo "No running OpenVINS container found!"
    echo ""
    echo "Please start a container first with:"
    echo "  ./run_docker.sh"
    echo ""
    echo "Then use this script to open additional terminals."
    exit 1
fi

CONTAINER_NAME=$(docker ps --filter "id=$CONTAINER" --format "{{.Names}}")

echo "Found running container:"
echo "  ID: $CONTAINER"
echo "  Name: $CONTAINER_NAME"
echo ""
echo "Attaching new terminal..."
echo ""

# Attach to the running container with a new bash shell
docker exec -it $CONTAINER bash
