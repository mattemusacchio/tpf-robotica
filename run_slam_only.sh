#!/usr/bin/env bash
set -e

WORKSPACE=/mnt/c/Users/Matteo/Documents/workspace/facultad/cuarto/robotica/tpf-robotica
BAG_PATH=$WORKSPACE/data/rosbags/casa_gazebo
FRONTEND_GRAPH=$WORKSPACE/log/casa_frontend_graph.json
OPTIMIZED_GRAPH=$WORKSPACE/log/casa_optimized_graph.json
MAP_OUT=$WORKSPACE/log/maps
MAP_NAME=casa_map
NAV_MAP=$WORKSPACE/src/tpf_navigation/maps/casa_map

source /opt/ros/humble/setup.bash
source /home/matteo/ros2_ws/install/setup.bash
source $WORKSPACE/install/setup.bash

echo "=== Building SLAM graph ==="
ros2 run tpf_slam offline_rosbag_graph_builder --bag $BAG_PATH --output $FRONTEND_GRAPH --scan-topic /scan --odom-topic /odom --no-aruco --min-keyframe-translation-m 0.15 --min-keyframe-rotation-rad 0.20 --icp-max-range-m 3.5

echo "=== Optimizing graph ==="
ros2 run tpf_slam graph_slam_backend --input $FRONTEND_GRAPH --output $OPTIMIZED_GRAPH

echo "=== Building occupancy map ==="
python3 -m tpf_slam.occupancy_grid_builder --bag $BAG_PATH --optimized-graph $OPTIMIZED_GRAPH --output-dir $MAP_OUT --map-name $MAP_NAME --resolution 0.05 --beam-stride 1 --scan-stride 1 --min-hits-for-occupied 2 --min-occupied-component-cells 1 --inflate-radius-m 0.0 --min-scan-travel-m 0.05 --no-tf-static --laser-x-m -0.032 --laser-y-m 0.0 --laser-yaw-rad 0.0 --scan-topic /scan

echo "=== Copying map to navigation package ==="
cp $MAP_OUT/${MAP_NAME}.pgm ${NAV_MAP}.pgm
cp $MAP_OUT/${MAP_NAME}.yaml ${NAV_MAP}_raw.yaml

ORIGIN_X=$(python3 -c "import json; d=json.load(open('$MAP_OUT/${MAP_NAME}_summary.json')); print(d['origin']['x'])")
ORIGIN_Y=$(python3 -c "import json; d=json.load(open('$MAP_OUT/${MAP_NAME}_summary.json')); print(d['origin']['y'])")

cat > ${NAV_MAP}.yaml << EOF
image: casa_map.pgm
resolution: 0.05
origin: [$ORIGIN_X, $ORIGIN_Y, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF

echo "=== ALL DONE ==="
