#!/bin/bash

# OpenVINS Docker Setup Script
# This script will install Docker, NVIDIA Container Toolkit, and set up the environment

set -e

echo "======================================"
echo "OpenVINS Docker Setup for ROS 2 Humble"
echo "======================================"

# Step 1: Install Docker
echo ""
echo "Step 1: Installing Docker..."
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io

# Add current user to docker group to run docker without sudo
echo "Adding user to docker group..."
sudo usermod -aG docker $USER

# Step 2: Install NVIDIA Container Toolkit (for GPU support and GUI)
echo ""
echo "Step 2: Installing NVIDIA Container Toolkit..."
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) && \
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && \
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Step 3: Enable X11 forwarding for GUI applications
echo ""
echo "Step 3: Enabling X11 forwarding..."
xhost +local:docker

echo ""
echo "======================================"
echo "Docker installation complete!"
echo "======================================"
echo ""
echo "IMPORTANT: You need to log out and log back in (or restart) for docker group changes to take effect."
echo ""
echo "After logging back in, run:"
echo "  ./build_docker.sh"
echo ""
