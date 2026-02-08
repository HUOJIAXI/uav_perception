#!/bin/bash

# Launch OpenVINS for Isaac Sim
# Make sure Isaac Sim is running and publishing topics before launching this

# Source the workspace
source install/setup.bash

# Launch OpenVINS with Isaac Sim configuration
ros2 launch ov_msckf subscribe.launch.py \
    config:=isaac_sim \
    max_cameras:=1 \
    use_stereo:=false \
    verbosity:=INFO \
    rviz_enable:=true
