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

## Calibración

Ver `data/calibration/README.md` para la matriz de cámara, coeficientes de distorsión y tamaño del marcador ArUco.

## Nota de versionado

Los archivos `.db3` de RosBag son muy pesados y están ignorados por `.gitignore`. Se recomienda versionar solo documentación, código, metadata y scripts; los bags quedan como datos locales descargables.
