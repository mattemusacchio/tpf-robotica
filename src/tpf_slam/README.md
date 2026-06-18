# tpf_slam

Paquete ROS 2 para empezar el Graph SLAM del TP final. Esta primera etapa es un **front-end**: todavía no optimiza, sino que arma un grafo inspeccionable con odometría y observaciones ArUco.

## Nodo principal

```bash
ros2 launch tpf_slam graph_slam_frontend.launch.py
```

Para correr detector ArUco + front-end juntos:

```bash
ros2 launch tpf_slam aruco_graph_frontend.launch.py
```

En otra terminal:

```bash
ros2 bag play data/rosbags/aruco_estimation
```

## Entradas

- `/tb4_0/odom` (`nav_msgs/Odometry`)
- `/aruco/detections` (`std_msgs/String` JSON publicado por `tpf_perception`)

## Salidas

- `/slam/graph_snapshot` (`std_msgs/String`): JSON con nodos y aristas.
- `/slam/keyframes` (`geometry_msgs/PoseArray`): poses guardadas.
- `/slam/landmarks` (`visualization_msgs/MarkerArray`): landmarks estimados por ID.
- `/slam/graph_edges` (`visualization_msgs/MarkerArray`): aristas odométricas y visuales.
- `log/slam_frontend_graph.json`: snapshot persistido para análisis offline.

## Qué contiene el grafo

- Nodos de pose: keyframes `(x, y, theta)` derivados de odometría.
- Aristas odométricas: delta entre keyframes consecutivos.
- Nodos de landmark: posición promedio por ID ArUco.
- Aristas visuales: mediciones `(range, bearing)` desde keyframe a landmark.

## Limitaciones actuales

- Usa el frame de odometría como aproximación de `map`.
- La posición global del landmark es una inicialización por promedio, no una optimización.
- No hay back-end Graph SLAM todavía; el próximo paso es optimizar este grafo.