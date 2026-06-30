# Datos del TP Final

Estructura local de materiales descargados para la opción 3 del TP final.

## RosBags

Los bags quedaron acomodados para poder reproducirse directamente con:

```bash
ros2 bag play data/rosbags/<nombre_del_bag>
```

| Bag | Uso principal | Duración aprox. | Mensajes | Comando |
| --- | --- | ---: | ---: | --- |
| `aruco_estimation` | Calibrar/caracterizar detección de ArUco a distancias controladas | 33.2 s | 4,666 | `ros2 bag play data/rosbags/aruco_estimation` |
| `laberinto` | SLAM final: trayectoria larga del laberinto con loops | 23.2 min | 196,471 | `ros2 bag play data/rosbags/laberinto` |
| `laberinto_conos` | Validación de visión y búsqueda de conos para Parte C | 15.0 min | 126,677 | `ros2 bag play data/rosbags/laberinto_conos` |

Tópicos comunes disponibles:

- `/tb4_0/odom` (`nav_msgs/msg/Odometry`)
- `/tb4_0/scan` (`sensor_msgs/msg/LaserScan`)
- `/tb4_0/oakd/rgb/preview/image_raw` (`sensor_msgs/msg/Image`)
- `/tb4_0/oakd/rgb/preview/camera_info` (`sensor_msgs/msg/CameraInfo`)
- `/tb4_0/imu` (`sensor_msgs/msg/Imu`)
- `/tb4_0/tf` y `/tb4_0/tf_static`

## Bag del laboratorio (robot real)

Grabado en la sesión de laboratorio con el robot físico `tb4_1` recorriendo el laberinto real.

| Bag | Uso principal | Duración aprox. | Mensajes | Namespace |
| --- | --- | ---: | ---: | --- |
| `labo/vivo_a3_0.db3` | SLAM + nav real: laberinto físico completo | 9.7 min | 59,196 | `/tb4_1/` |

Tópicos disponibles:

- `/tb4_1/odom` (`nav_msgs/msg/Odometry`)
- `/tb4_1/scan` (`sensor_msgs/msg/LaserScan`)
- `/tb4_1/oakd/rgb/preview/image_raw` (`sensor_msgs/msg/Image`)
- `/tb4_1/oakd/rgb/preview/camera_info` (`sensor_msgs/msg/CameraInfo`)
- `/tb4_1/tf` y `/tb4_1/tf_static`

Para generar el mapa offline desde este bag, ver `scripts/gen_labo_map.sh`.

Para validar Partes B y C con el mapa generado:

```bash
# Terminal 1: stack de navegación apuntando al nuevo mapa y tópicos tb4_1
ros2 launch tpf_navigation part_c_rosbag_validation.launch.py \
  map_yaml:=$(pwd)/log/maps/labo_map.yaml \
  scan_topic:=/tb4_1/scan \
  odom_topic:=/tb4_1/odom \
  image_topic:=/tb4_1/oakd/rgb/preview/image_raw \
  camera_info_topic:=/tb4_1/oakd/rgb/preview/camera_info

# Terminal 2: reproducir el bag del labo
ros2 bag play data/labo --clock
```

## Calibración

Ver `data/calibration/README.md` para la matriz de cámara, coeficientes de distorsión y tamaño del marcador ArUco.

## Nota de versionado

Los archivos `.db3` de RosBag son muy pesados y están ignorados por `.gitignore`. Se recomienda versionar solo documentación, código, metadata y scripts; los bags quedan como datos locales descargables.
