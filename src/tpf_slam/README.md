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
- La posición global inicial del landmark se calcula por promedio antes de pasar al back-end.
- El back-end actual es offline; todavía falta publicar la trayectoria optimizada como nodo ROS para RViz/mapa.

## Back-end offline

Una vez generado `log/slam_frontend_graph.json`, optimizar offline con:

```bash
ros2 run tpf_slam graph_slam_backend \
  --input log/slam_frontend_graph.json \
  --output log/slam_optimized_graph.json
```

Exporta:

- `log/slam_optimized_graph.json`: grafo con poses y landmarks optimizados.
- `log/optimized_trajectory.csv`: trayectoria optimizada.
- `log/optimized_landmarks.csv`: landmarks optimizados.

El solver fija la primera pose como ancla de gauge y minimiza:

- error odométrico entre keyframes consecutivos;
- error visual de rango/bearing desde keyframes hacia landmarks.

Usa `scipy.optimize.least_squares` si está disponible en el entorno ROS.
## Extracción offline desde RosBag

Para bags grandes como `laberinto`, reproducir en tiempo real puede ser lento. El extractor offline lee el `.db3` directamente, procesa una muestra distribuida de imágenes y arma el mismo grafo:

```bash
ros2 run tpf_slam offline_rosbag_graph_builder \
  --bag data/rosbags/laberinto \
  --output log/laberinto_frontend_graph.json \
  --image-stride 50 \
  --progress-interval 200
```

Luego optimizar:

```bash
ros2 run tpf_slam graph_slam_backend \
  --input log/laberinto_frontend_graph.json \
  --output log/laberinto_optimized_graph.json \
  --max-iterations 200
```

Validación actual con el bag completo `laberinto` usando `--image-stride 50`:

- `737` keyframes.
- `50` landmarks ArUco.
- `736` aristas odométricas.
- `768` aristas visuales.
- Costo inicial `3126.09`, costo final `1804.04`, reducción aproximada `42.3%`.

El solver puede reportar `success=false` si llega a `max_nfev`, pero el JSON incluye `solver.usable_solution=true` cuando la solución reduce el costo y es finita.