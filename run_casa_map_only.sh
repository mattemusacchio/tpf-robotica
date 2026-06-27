#!/usr/bin/env bash
set -e
WS=/mnt/c/Users/Matteo/Documents/workspace/facultad/cuarto/robotica/tpf-robotica
source /opt/ros/humble/setup.bash
source /home/matteo/ros2_ws/install/setup.bash
source $WS/install/setup.bash

echo "=== Rebuilding casa occupancy map from existing optimized graph ==="
python3 -m tpf_slam.occupancy_grid_builder \
    --bag $WS/data/rosbags/casa_gazebo \
    --optimized-graph $WS/log/casa_optimized_graph.json \
    --output-dir $WS/log/maps \
    --map-name casa_map \
    --resolution 0.05 \
    --beam-stride 1 \
    --scan-stride 1 \
    --min-hits-for-occupied 1 \
    --min-occupied-component-cells 1 \
    --inflate-radius-m 0.0 \
    --min-scan-travel-m 0.05 \
    --no-tf-static \
    --laser-x-m -0.032 \
    --laser-y-m 0.0 \
    --laser-yaw-rad 0.0 \
    --scan-topic /scan

# Copy to nav package
cp $WS/log/maps/casa_map.pgm $WS/src/tpf_navigation/maps/casa_map.pgm
echo "=== ALL DONE ==="
