#!/usr/bin/env bash
# Genera el mapa de ocupación offline desde el bag del laboratorio (tb4_1).
#
# Ejecutar desde la raíz del repo con el entorno ROS sourced:
#   source /opt/ros/humble/setup.bash && source install/setup.bash
#   bash scripts/gen_labo_map.sh
#
# Salida: log/maps/labo_map.{pgm,yaml,png}  + log/labo_optimized_graph.json
#
# Si el laberinto físico no tiene ArUcos visibles o el detector falla,
# agregar --no-aruco al paso 1; el ICP solo también cierra loops cortos.

set -e

BAG=data/labo
FRONTEND=log/labo_frontend_graph.json
OPTIMIZED=log/labo_optimized_graph.json
MAP_DIR=log/maps
MAP_NAME=labo_map

echo "=== Paso 1: construir grafo front-end desde el bag del lab ==="
ros2 run tpf_slam offline_rosbag_graph_builder \
  --bag "$BAG" \
  --output "$FRONTEND" \
  --scan-topic /tb4_1/scan \
  --odom-topic /tb4_1/odom \
  --image-topic /tb4_1/oakd/rgb/preview/image_raw \
  --image-stride 10 \
  --progress-interval 100

echo ""
echo "=== Paso 2: optimizar grafo con Graph SLAM back-end ==="
ros2 run tpf_slam graph_slam_backend \
  --input "$FRONTEND" \
  --output "$OPTIMIZED" \
  --max-iterations 200

echo ""
echo "=== Paso 3: construir grilla de ocupación con trayectoria optimizada ==="
ros2 run tpf_slam occupancy_grid_builder \
  --bag "$BAG" \
  --optimized-graph "$OPTIMIZED" \
  --output-dir "$MAP_DIR" \
  --map-name "$MAP_NAME" \
  --resolution 0.05 \
  --max-range-m 5.0 \
  --scan-topic /tb4_1/scan \
  --no-tf-static \
  --laser-x-m -0.04 \
  --laser-yaw-rad 1.5707963267948966 \
  --inflate-radius-m 0.0 \
  --min-occupied-component-cells 4 \
  --min-hits-for-occupied 3

echo ""
echo "=== Mapa generado en $MAP_DIR/$MAP_NAME.{pgm,yaml,png} ==="
echo ""
echo "Para validar Partes B y C:"
echo "  Terminal 1: ros2 launch tpf_navigation part_c_rosbag_validation.launch.py \\"
echo "    map_yaml:=\$(pwd)/$MAP_DIR/$MAP_NAME.yaml \\"
echo "    scan_topic:=/tb4_1/scan \\"
echo "    odom_topic:=/tb4_1/odom \\"
echo "    image_topic:=/tb4_1/oakd/rgb/preview/image_raw \\"
echo "    camera_info_topic:=/tb4_1/oakd/rgb/preview/camera_info"
echo "  Terminal 2: ros2 bag play $BAG --clock"
