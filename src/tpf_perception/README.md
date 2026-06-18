# tpf_perception

Paquete ROS 2 para la primera fase del TP final: detectar ArUco Tags desde los RosBags del TurtleBot4 y publicar observaciones que después alimentan el Graph SLAM.

## Nodo principal

```bash
ros2 run tpf_perception aruco_detector_node
```

O con launch/config:

```bash
ros2 launch tpf_perception aruco_detector.launch.py
```

En otra terminal, reproducir el bag chico de calibración:

```bash
ros2 bag play data/rosbags/aruco_estimation
```

## Entradas

Por defecto usa los tópicos del RosBag descargado:

- `/tb4_0/oakd/rgb/preview/image_raw`
- `/tb4_0/oakd/rgb/preview/camera_info`

## Salidas

- `/aruco/detections` (`std_msgs/String`): JSON con `id`, pose relativa en frame óptico y aproximación planar `range`/`bearing` para SLAM.
- `/aruco/poses` (`geometry_msgs/PoseArray`): poses relativas para inspección rápida.
- `/aruco/markers` (`visualization_msgs/MarkerArray`): markers con labels para RViz.
- `/aruco/debug_image` (`sensor_msgs/Image`): imagen anotada con bordes y ejes.

Ejemplo para ver detecciones:

```bash
ros2 topic echo /aruco/detections
```

## Calibración usada

El default del YAML usa TurtleBot 0, indicado por la cátedra:

- `MARKER_SIZE_M = 0.0889`
- matriz intrínseca y distorsión documentadas en `data/calibration/README.md`

Por defecto usa la calibración estática del YAML (`use_static_calibration: true`), que coincide con las distancias esperadas del bag de calibración. Si se configura `use_static_calibration: false`, usa `/camera_info`.

## Validación esperada con `aruco_estimation`

Primer sanity check de distancias:

| Real aprox. | Estimada aprox. |
| ---: | ---: |
| 0.30 m | 0.28 m |
| 0.695 m | 0.72 m |
| 1.01 m | 1.04 m |
| 1.49 m | 1.52 m |

## Si no detecta nada

1. Probar otro diccionario ArUco en `config/aruco_detector.yaml`:
   - `DICT_4X4_50`
   - `DICT_5X5_100`
   - `DICT_6X6_250`
2. Verificar que el tópico de imagen publica:

```bash
ros2 topic hz /tb4_0/oakd/rgb/preview/image_raw
```

3. Ver la imagen anotada:

```bash
rqt_image_view /aruco/debug_image
```
