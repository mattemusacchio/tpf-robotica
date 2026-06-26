#!/usr/bin/env bash
# Full automated pipeline: launch headless Gazebo casa.world, explore, record bag, run SLAM.
set -e

WORKSPACE=/mnt/c/Users/Matteo/Documents/workspace/facultad/cuarto/robotica/tpf-robotica
ROS2_WS=/home/matteo/ros2_ws
BAG_PATH=$WORKSPACE/data/rosbags/casa_gazebo
FRONTEND_GRAPH=$WORKSPACE/log/casa_frontend_graph.json
OPTIMIZED_GRAPH=$WORKSPACE/log/casa_optimized_graph.json
MAP_OUT=$WORKSPACE/log/maps
MAP_NAME=casa_map
NAV_MAP=$WORKSPACE/src/tpf_navigation/maps/casa_map

export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1

source /opt/ros/humble/setup.bash
source $ROS2_WS/install/setup.bash
source $WORKSPACE/install/setup.bash
echo "=== ROS2 + workspaces sourced OK ==="

# Remove old bag if exists
rm -rf $BAG_PATH

# ── 1. Launch headless Gazebo (gzserver only) + robot ────────────────────────
echo "=== Launching headless Gazebo + TurtleBot3 in casa.world ==="
ros2 launch tpf_navigation casa_headless_sim.launch.py \
    use_sim_time:=true &
SIM_PID=$!
echo "Simulation PID: $SIM_PID"

echo "Waiting 12s for Gazebo to fully start and spawn robot..."
sleep 12

# ── 2. Start recording bag ────────────────────────────────────────────────────
echo "=== Recording bag ==="
ros2 bag record /scan /odom /tf /tf_static \
    -o $BAG_PATH &
BAG_PID=$!
echo "Bag recorder PID: $BAG_PID"
sleep 2

# ── 3. Run exploration node ───────────────────────────────────────────────────
echo "=== Starting explorer (200s coverage) ==="
python3 $WORKSPACE/casa_explorer.py &
EXPLORER_PID=$!
echo "Explorer PID: $EXPLORER_PID"

# Wait for explorer to finish (MAX_DURATION_S = 200s, plus buffer)
wait $EXPLORER_PID || true
echo "Explorer finished."

# ── 4. Stop bag recorder and simulation ───────────────────────────────────────
echo "=== Stopping bag recorder ==="
kill $BAG_PID 2>/dev/null || true
sleep 2

echo "=== Stopping simulation ==="
kill $SIM_PID 2>/dev/null || true
sleep 3

echo "=== Bag recorded at $BAG_PATH ==="
ls -lh $BAG_PATH/

# ── 5. Run offline graph builder (no ArUco — Gazebo has no camera) ────────────
echo "=== Building SLAM graph (odom + ICP scan matching) ==="
ros2 run tpf_slam offline_rosbag_graph_builder \
    --bag $BAG_PATH \
    --output $FRONTEND_GRAPH \
    --scan-topic /scan \
    --odom-topic /odom \
    --no-aruco \
    --min-keyframe-translation-m 0.10 \
    --min-keyframe-rotation-rad 0.15 \
    --icp-max-range-m 3.5

echo "=== Optimizing graph ==="
ros2 run tpf_slam graph_slam_backend \
    --input $FRONTEND_GRAPH \
    --output $OPTIMIZED_GRAPH

# ── 6. Build occupancy map ────────────────────────────────────────────────────
echo "=== Building occupancy map ==="
python3 -m tpf_slam.occupancy_grid_builder \
    --bag $BAG_PATH \
    --optimized-graph $OPTIMIZED_GRAPH \
    --output-dir $MAP_OUT \
    --map-name $MAP_NAME \
    --resolution 0.05 \
    --beam-stride 1 \
    --scan-stride 1 \
    --min-hits-for-occupied 3 \
    --min-occupied-component-cells 10 \
    --inflate-radius-m 0.0 \
    --min-scan-travel-m 0.05 \
    --no-tf-static \
    --laser-x-m -0.032 \
    --laser-y-m 0.0 \
    --laser-yaw-rad 0.0 \
    --scan-topic /scan

# ── 7. Copy map to navigation folder ─────────────────────────────────────────
echo "=== Copying map to navigation package ==="
cp $MAP_OUT/${MAP_NAME}.pgm  ${NAV_MAP}.pgm
cp $MAP_OUT/${MAP_NAME}.yaml ${NAV_MAP}_raw.yaml

# Write correct nav yaml (origin from builder output)
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

echo "=== casa_map.pgm written to navigation maps ==="
echo "=== ALL DONE ==="
