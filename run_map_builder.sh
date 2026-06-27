#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

# Source workspace if built
WORKSPACE=/mnt/c/Users/Matteo/Documents/workspace/facultad/cuarto/robotica/tpf-robotica
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
    echo "Workspace sourced OK"
else
    echo "WARNING: workspace not built yet, using only ROS2 base"
fi

cd "$WORKSPACE"

echo "=== Running occupancy_grid_builder ==="
python3 -m tpf_slam.occupancy_grid_builder \
    --bag data/rosbags/laberinto \
    --optimized-graph log/laberinto_second_pass_graph.json \
    --output-dir log/maps \
    --map-name laberinto_map \
    --resolution 0.05 \
    --beam-stride 1 \
    --scan-stride 1 \
    --min-hits-for-occupied 2 \
    --min-occupied-component-cells 1 \
    --inflate-radius-m 0.0 \
    --min-scan-travel-m 0.08

echo "=== Done ==="
