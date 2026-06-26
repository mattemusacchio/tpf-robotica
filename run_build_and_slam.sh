#!/usr/bin/env bash
set -e

WORKSPACE=/mnt/c/Users/Matteo/Documents/workspace/facultad/cuarto/robotica/tpf-robotica

source /opt/ros/humble/setup.bash
source /home/matteo/ros2_ws/install/setup.bash

cd $WORKSPACE
echo "=== Building tpf_navigation and tpf_slam ==="
colcon build --packages-select tpf_navigation tpf_slam 2>&1
echo "=== Build done ==="

bash $WORKSPACE/run_casa_slam.sh
