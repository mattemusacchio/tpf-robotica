#!/usr/bin/env bash
set -e

WORKSPACE=/mnt/c/Users/Matteo/Documents/workspace/facultad/cuarto/robotica/tpf-robotica

source /opt/ros/humble/setup.bash
source /home/matteo/ros2_ws/install/setup.bash
source $WORKSPACE/install/setup.bash

echo "=== Regenerating laberinto map: 5cm, min-hits=1, component-filter=8 ==="
python3 -m tpf_slam.occupancy_grid_builder \
    --bag $WORKSPACE/data/rosbags/laberinto \
    --optimized-graph $WORKSPACE/log/laberinto_second_pass_graph.json \
    --output-dir $WORKSPACE/log/maps \
    --map-name laberinto_map \
    --resolution 0.05 \
    --beam-stride 1 \
    --scan-stride 1 \
    --min-hits-for-occupied 1 \
    --min-occupied-component-cells 8 \
    --inflate-radius-m 0.0 \
    --min-scan-travel-m 0.08

echo "=== ALL DONE ==="
