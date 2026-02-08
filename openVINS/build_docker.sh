#!/bin/bash

# OpenVINS Docker Build Script
# This script builds the OpenVINS Docker image

set -e

echo "======================================"
echo "Building OpenVINS Docker Image"
echo "======================================"

# Set the version
export VERSION=ros2_22_04

# Build the Docker image
echo "Building Docker image: ov_$VERSION"
echo "This may take 10-20 minutes..."
docker build -t ov_$VERSION -f Dockerfile_$VERSION .

echo ""
echo "======================================"
echo "Docker image built successfully!"
echo "======================================"
echo ""
echo "Image name: ov_$VERSION"
echo ""
echo "Next steps:"
echo "1. Run ./setup_workspace.sh to configure your workspace"
echo "2. Use the provided run_docker.sh script to start the container"
echo ""
