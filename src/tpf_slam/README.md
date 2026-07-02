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
  --image-stride 10 \
  --progress-interval 200
```

Luego optimizar (`--max-nfev` cuenta evaluaciones de funcion, no iteraciones):

```bash
ros2 run tpf_slam graph_slam_backend \
  --input log/laberinto_frontend_graph.json \
  --output log/laberinto_optimized_graph.json \
  --max-nfev 20000
```

Validación actual con el bag completo `laberinto` usando `--image-stride 50`:

- `737` keyframes.
- `50` landmarks ArUco.
- `736` aristas odométricas.
- `768` aristas visuales.
- Costo inicial `3126.09`, costo final `1804.04`, reducción aproximada `42.3%`.

El solver puede reportar `success=false` si llega a `max_nfev`, pero el JSON incluye `solver.usable_solution=true` cuando la solución reduce el costo y es finita.
## Mapa de ocupación offline

Con el grafo optimizado del laberinto se puede construir un mapa de ocupación desde `/tb4_0/scan`:

```bash
ros2 run tpf_slam occupancy_grid_builder \
  --bag data/rosbags/laberinto \
  --optimized-graph log/laberinto_optimized_graph.json \
  --output-dir log/maps \
  --map-name laberinto_map \
  --resolution 0.05 \
  --max-range-m 5.0 \
  --scan-stride 2 \
  --beam-stride 1 \
  --inflate-radius-m 0.0 \
  --min-occupied-component-cells 3 \
  --no-tf-static --laser-x-m -0.04 --laser-yaw-rad 1.5707963267948966
```

> La inflacion no se hornea en el mapa exportado (`--inflate-radius-m 0.0`): eso
> corresponde al costmap de Nav2. Para una referencia extra-nitida usar
> `--resolution 0.03 --scan-stride 1`.

> Importante: el láser de este bag está montado a `yaw = pi/2`. Si se usa
> `--no-tf-static`, hay que pasar `--laser-yaw-rad 1.5707963267948966` (y
> `--laser-x-m -0.04`); de lo contrario los scans quedan rotados 90° y las
> paredes salen dobladas. Sin `--no-tf-static`, el mapper resuelve la
> transformada desde `/tb4_0/tf_static`.

Exporta:

- `log/maps/laberinto_map.pgm`: mapa ROS occupancy-grid.
- `log/maps/laberinto_map.yaml`: metadata compatible con ROS map server.
- `log/maps/laberinto_map.png`: vista rápida con trayectoria y landmarks dibujados.
- `log/maps/optimized_trajectory.csv`: trayectoria usada.
- `log/maps/optimized_landmarks.json`: landmarks usados.
- `log/maps/laberinto_map_summary.json`: parámetros y estadísticas.

Validación actual con `laberinto`:

- `1080` scans procesados de `10797` (`--scan-stride 10`).
- `139857` rayos válidos.
- resolución `0.08 m/celda`.
- mapa `238 x 233` celdas.
- paredes/pasillos visibles en el PNG de debug.

El mapper lee `/tb4_0/tf_static` y aplica la transformada `base_link -> rplidar_link` detectada en el bag. En `laberinto`, la transformada usada fue `x=-0.04 m`, `y=0.0 m`, `yaw=1.5708 rad`.
### Validación con Nav2 map_server

El mapa exportado fue probado con `nav2_map_server`:

```bash
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:=/mnt/c/Users/Matteo/Documents/workspace/facultad/cuarto/robotica/tpf-robotica/log/maps/laberinto_map.yaml

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 topic echo /map --once --qos-durability transient_local --qos-reliability reliable
```

Resultado validado:

- lifecycle `configure`: OK.
- lifecycle `activate`: OK.
- `/map` publicado con `frame_id: map`.
- resolución `0.08`.
- dimensiones `238 x 233`.

## Demo progresiva en ROS/RViz

Para ver el mapa crecer como video sin esperar a un SLAM online completo, el
paquete incluye un nodo de demo que lee internamente el bag `laberinto`, usa la
trayectoria optimizada offline y publica el mapa de ocupacion incrementalmente.

```bash
source install/setup.bash
ros2 launch tpf_slam progressive_mapping_demo.launch.py rviz:=true
```

Topicos publicados:

- `/map` (`nav_msgs/OccupancyGrid`): mapa incremental con QoS transient local.
- `/slam/demo_path` (`nav_msgs/Path`): trayectoria optimizada recorrida hasta el
  scan actual.
- `/slam/demo_landmarks` (`visualization_msgs/MarkerArray`): landmarks ArUco
  optimizados con etiquetas de ID.
- `/slam/demo_status` (`std_msgs/String`): JSON con scans procesados, rayos
  validos, dimensiones del mapa y transformada laser usada.

Parametros utiles:

```bash
ros2 launch tpf_slam progressive_mapping_demo.launch.py \
  rviz:=true \
  playback_rate_hz:=12.0 \
  scan_stride:=20 \
  beam_stride:=10
```

Por defecto el demo usa la transformada laser del bag ya validada
(`base_link -> rplidar_link`: `x=-0.04`, `y=0.0`, `yaw=1.5708`) como
parametros para arrancar rapido. Si se quiere releer `/tb4_0/tf_static` desde
el `.db3`, lanzar con `use_tf_static:=true`.

Para una prueba rapida sin RViz:

```bash
ros2 launch tpf_slam progressive_mapping_demo.launch.py \
  playback_rate_hz:=0.0 \
  max_processed_scans:=40
```

Esta opcion es una superficie de visualizacion/reproduccion: no re-optimiza el
grafo en vivo, sino que muestra progresivamente como los scans del rosbag se
integran contra la trayectoria ya optimizada.


## Cierre Parte A segun consigna

Para defender la Parte A opcion 3, usar el launch final:

```bash
source install/setup.bash
ros2 launch tpf_slam part_a_graph_slam_demo.launch.py rviz:=true
```

Topicos alineados con la consigna:

- `/map` (`nav_msgs/OccupancyGrid`): grilla de ocupacion resultante.
- `/belief` (`nav_msgs/Path`): trayectoria corregida/optimizada por Graph SLAM.
- `/landmarks` (`visualization_msgs/MarkerArray`): ArUco landmarks optimizados con etiquetas de ID.
- `/poses_guardadas` (`geometry_msgs/PoseArray`): keyframes/nodos del grafo.
- `/slam/part_a_status` (`std_msgs/String`): resumen JSON de grafo, scans y mapa.

La opcion 3 de la consigna pide Graph SLAM obligatorio con ArUco sobre RosBag. El flujo implementado queda:

1. detectar ArUco y caracterizar mediciones con `aruco_estimation`;
2. construir el grafo de poses/landmarks desde `laberinto`;
3. optimizar el grafo offline;
4. reproducir/procesar LIDAR con la trayectoria corregida para generar `/map`;
5. mostrar en RViz `/belief`, `/landmarks`, `/poses_guardadas` y `/map`.

### Reporte de calidad y loop closure

Generar un reporte reproducible con:

```bash
ros2 run tpf_slam slam_quality_report \
  --frontend-graph log/laberinto_frontend_graph.json \
  --optimized-graph log/laberinto_optimized_graph.json \
  --map-summary log/maps/laberinto_map_summary.json \
  --output log/part_a/part_a_summary.json
```

El reporte incluye cantidad de keyframes, landmarks, aristas odometricas, aristas visuales, reduccion de costo del solver y evidencia de cierre de lazo por reobservacion de IDs ArUco desde multiples keyframes.

### Checklist de regeneracion Parte A

```bash
# 1) Construir grafo inicial desde el rosbag largo.
ros2 run tpf_slam offline_rosbag_graph_builder \
  --bag data/rosbags/laberinto \
  --output log/laberinto_frontend_graph.json \
  --image-stride 10 \
  --progress-interval 200

# 2) Optimizar trayectoria y landmarks con Graph SLAM.
#    --max-nfev cuenta evaluaciones de funcion (no iteraciones); 20000 evita que
#    el solver termine antes de converger.
ros2 run tpf_slam graph_slam_backend \
  --input log/laberinto_frontend_graph.json \
  --output log/laberinto_optimized_graph.json \
  --max-nfev 20000

# 3) Proyectar LIDAR usando la trayectoria corregida.
#    La inflacion NO se hornea en el mapa exportado (--inflate-radius-m 0.0): eso
#    corresponde al costmap de Nav2. Para una referencia extra-nitida usar
#    --resolution 0.03 --scan-stride 1.
ros2 run tpf_slam occupancy_grid_builder \
  --bag data/rosbags/laberinto \
  --optimized-graph log/laberinto_optimized_graph.json \
  --output-dir log/maps \
  --map-name laberinto_map \
  --resolution 0.05 \
  --max-range-m 5.0 \
  --scan-stride 2 \
  --beam-stride 1 \
  --inflate-radius-m 0.0 \
  --min-occupied-component-cells 3

# 4) Emitir reporte de defensa.
ros2 run tpf_slam slam_quality_report \
  --frontend-graph log/laberinto_frontend_graph.json \
  --optimized-graph log/laberinto_optimized_graph.json \
  --map-summary log/maps/laberinto_map_summary.json \
  --output log/part_a/part_a_summary.json
```

### Calidad del mapa

Ajustes aplicados para mejorar la calidad de la Parte A:

- **Convergencia del solver:** el back-end usa `--max-nfev 20000` (evaluaciones de
  funcion, no iteraciones) y `x_scale='jac'`, de modo que el least_squares deja de
  quedarse corto y realmente reduce el costo.
- **Offset de camara:** las observaciones ArUco se corrigen por la posicion del
  frame optico respecto de `base_link`, resuelta desde `tf_static` del bag (o el
  valor de config si no esta disponible).
- **Filtrado de outliers:** se descartan mediciones ArUco detras de camara, fuera
  de rango, con error de reproyeccion alto, con angulo rasante excesivo, y frames
  con motion blur; el back-end tambien descarta aristas visuales con rango
  imposible (`< 0.1 m` o `> 3.5 m`).
- **Sin inflacion horneada:** el mapa exportado no infla las paredes
  (`--inflate-radius-m 0.0`); la inflacion corresponde al costmap de Nav2.
