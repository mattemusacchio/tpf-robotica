#!/usr/bin/env bash
# Genera el mapa de ocupación offline desde el bag del laboratorio (tb4_1).
#
# Ejecutar desde la raíz del repo con el entorno ROS sourced:
#   source /opt/ros/humble/setup.bash && source install/setup.bash
#   bash scripts/gen_labo_map.sh
#
# Salida: log/maps/labo_map.{pgm,yaml,png} + log/labo_optimized_graph_final.json
#
# Si el laberinto físico no tiene ArUcos visibles o el detector falla,
# agregar --no-aruco al paso 1; el ICP solo también cierra loops cortos.
#
# Nota: este bag usa el robot tb4_1, con calibracion de camara propia
# (distinta a la de tb4_0 usada en el bag "laberinto"). Los offsets de
# camara y laser se resuelven automaticamente desde /tb4_1/tf_static.

set -e

BAG=data/labo
FRONTEND=log/labo_frontend_graph.json
OPTIMIZED=log/labo_optimized_graph.json
SECOND_PASS=log/labo_second_pass_graph.json
OPTIMIZED_FINAL=log/labo_optimized_graph_final.json
MAP_DIR=log/maps
MAP_NAME=labo_map

SCAN_TOPIC=/tb4_1/scan
ODOM_TOPIC=/tb4_1/odom
IMAGE_TOPIC=/tb4_1/oakd/rgb/preview/image_raw
TF_STATIC_TOPIC=/tb4_1/tf_static

# Calibracion TurtleBot4 #1 (ver data/calibration/README.md).
CAMERA_MATRIX="201.37 0.0 123.78 0.0 357.31 131.25 0.0 0.0 1.0"
DIST_COEFFS="7.823812007904053 -116.5168228149414 0.0008780899806879461 0.000634733063634485 378.764404296875"

echo "=== Paso 1: construir grafo front-end desde el bag del lab ==="
ros2 run tpf_slam offline_rosbag_graph_builder \
  --bag "$BAG" \
  --output "$FRONTEND" \
  --scan-topic "$SCAN_TOPIC" \
  --odom-topic "$ODOM_TOPIC" \
  --image-topic "$IMAGE_TOPIC" \
  --tf-static-topic "$TF_STATIC_TOPIC" \
  --camera-matrix $CAMERA_MATRIX \
  --dist-coeffs $DIST_COEFFS \
  --image-stride 10 \
  --progress-interval 200

echo ""
echo "=== Paso 2: optimizar grafo con Graph SLAM back-end ==="
#    --max-nfev cuenta evaluaciones de funcion (no iteraciones); 20000 evita que
#    el solver termine antes de converger.
ros2 run tpf_slam graph_slam_backend \
  --input "$FRONTEND" \
  --output "$OPTIMIZED" \
  --max-nfev 20000

echo ""
echo "=== Paso 3: segunda pasada de ICP con las poses ya optimizadas ==="
ros2 run tpf_slam second_pass_icp \
  --frontend-graph "$FRONTEND" \
  --optimized-graph "$OPTIMIZED" \
  --bag "$BAG" \
  --output "$SECOND_PASS" \
  --scan-topic "$SCAN_TOPIC" \
  --laser-x-m -0.04 \
  --laser-y-m 0.0 \
  --laser-yaw-rad 1.5707963267948966 \
  --icp-max-range-m 3.5

ros2 run tpf_slam graph_slam_backend \
  --input "$SECOND_PASS" \
  --output "$OPTIMIZED_FINAL" \
  --max-nfev 20000

echo ""
echo "=== Paso 4: construir grilla de ocupación con trayectoria corregida ==="
#    La inflacion NO se hornea en el mapa exportado (--inflate-radius-m 0.0): eso
#    corresponde al costmap de Nav2.
ros2 run tpf_slam occupancy_grid_builder \
  --bag "$BAG" \
  --optimized-graph "$OPTIMIZED_FINAL" \
  --output-dir "$MAP_DIR" \
  --map-name "$MAP_NAME" \
  --resolution 0.03 \
  --max-range-m 5.0 \
  --scan-topic "$SCAN_TOPIC" \
  --tf-static-topic "$TF_STATIC_TOPIC" \
  --scan-stride 1 \
  --beam-stride 1 \
  --inflate-radius-m 0.0 \
  --min-occupied-component-cells 4 \
  --min-hits-for-occupied 4

echo ""
echo "=== Mapa generado en $MAP_DIR/$MAP_NAME.{pgm,yaml,png} ==="
echo ""
echo "Para validar Partes B y C:"
echo "  Terminal 1: ros2 launch tpf_navigation part_c_rosbag_validation.launch.py \\"
echo "    map_yaml:=\$(pwd)/$MAP_DIR/$MAP_NAME.yaml \\"
echo "    scan_topic:=$SCAN_TOPIC \\"
echo "    odom_topic:=$ODOM_TOPIC \\"
echo "    image_topic:=$IMAGE_TOPIC \\"
echo "    camera_info_topic:=/tb4_1/oakd/rgb/preview/camera_info"
echo "  Terminal 2: ros2 bag play $BAG --clock"
