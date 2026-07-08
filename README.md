# TP Final Robótica — Opción 3 (Features con Cámara: ArUco + LIDAR + Graph SLAM)

Sistema ROS 2 (Humble) para las tres partes del TP:

| Parte | Qué hace | Paquete |
| --- | --- | --- |
| **A** | SLAM: Graph SLAM con ArUco + odometría + ICP → mapa de ocupación | `tpf_slam` |
| **B** | Navegación autónoma (MCL + A* + Pure Pursuit + máquina de estados) | `tpf_navigation` |
| **C** | Búsqueda de conos rojos → goal al planner (no atraviesa paredes) | `tpf_navigation` + `tpf_perception` |

> Runbook detallado para el robot real: [`docs/RUNBOOK_robot_real.md`](docs/RUNBOOK_robot_real.md).

---

## 0) Compilar (una vez) y sourcear (cada terminal)

```bash
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tpf_slam tpf_navigation tpf_perception
source install/setup.bash
```
En **cada terminal nueva**:
```bash
source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash && cd ~/ws/src/TP_FINAL
```

El mapa del laberinto ya viene versionado en `log/maps/laberinto_map.yaml` (Parte A).
Los rosbags (`data/rosbags/*.db3`) NO están en git por tamaño.

---

## PARTE A — SLAM y mapa

### Demo en RViz (usa un grafo ya optimizado)
```bash
ros2 launch tpf_slam part_a_graph_slam_demo.launch.py rviz:=true playback_rate_hz:=0.0 \
  optimized_graph_path:=log/laberinto_optimized_graph_v2.json
```
Publica `/map`, `/belief` (trayectoria), `/landmarks`, `/poses_guardadas`.

### Generar el mapa desde un bag (offline)
```bash
export PYTHONPATH=src/tpf_slam:$PYTHONPATH
# 1) front-end (grafo ArUco + ICP)
python3 -m tpf_slam.offline_rosbag_graph_builder --bag data/rosbags/laberinto \
  --output log/laberinto_frontend_graph.json
# 2) optimizar (Graph SLAM)
python3 -m tpf_slam.graph_slam_backend --input log/laberinto_frontend_graph.json \
  --output log/laberinto_opt.json
# 3) (opcional) segunda pasada de ICP -> mas loop closures
ros2 run tpf_slam second_pass_icp --frontend-graph log/laberinto_frontend_graph.json \
  --optimized-graph log/laberinto_opt.json --bag data/rosbags/laberinto \
  --output log/laberinto_secondpass_graph.json
python3 -m tpf_slam.graph_slam_backend --input log/laberinto_secondpass_graph.json \
  --output log/laberinto_optimized_graph_v2.json
# 4) mapa de ocupacion (OJO: laser del bag a yaw=pi/2)
python3 -m tpf_slam.occupancy_grid_builder --bag data/rosbags/laberinto \
  --optimized-graph log/laberinto_optimized_graph_v2.json \
  --output-dir log/maps --map-name laberinto_map --resolution 0.05 --beam-stride 2 \
  --no-tf-static --laser-x-m -0.04 --laser-yaw-rad 1.5707963267948966
```

### Mapear EN VIVO con el robot real (tb4_1)
```bash
# T1 — grabar mientras manejas (ajusta tb4_1 si el robot es otro)
ros2 bag record -o vivo_a \
  /tb4_1/scan /tb4_1/odom /tb4_1/tf /tb4_1/tf_static \
  /tb4_1/oakd/rgb/preview/image_raw /tb4_1/oakd/rgb/preview/camera_info
# T2 — manejar por todo el laberinto (volver a lugares = loop closures)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/tb4_1/cmd_vel
# Ctrl+C en T1 al terminar, y procesar con los topicos de tb4_1:
export PYTHONPATH=src/tpf_slam:$PYTHONPATH
python3 -m tpf_slam.offline_rosbag_graph_builder --bag vivo_a --output log/vivo_front.json \
  --scan-topic /tb4_1/scan --odom-topic /tb4_1/odom --image-topic /tb4_1/oakd/rgb/preview/image_raw
python3 -m tpf_slam.graph_slam_backend --input log/vivo_front.json --output log/vivo_opt.json
python3 -m tpf_slam.occupancy_grid_builder --bag vivo_a --optimized-graph log/vivo_opt.json \
  --output-dir log/maps --map-name vivo_map --resolution 0.05 --beam-stride 2 \
  --scan-topic /tb4_1/scan --no-tf-static --laser-x-m -0.04 --laser-yaw-rad 1.5707963267948966
```
> Si ArUco molesta en vivo, agregá `--no-aruco` al front-end (mapa LIDAR-only).

---

## PARTE B — Navegación autónoma

### En Gazebo (simulado)
```bash
ros2 launch tpf_navigation part_b_navigation.launch.py            # mundo custom_casa
ros2 launch tpf_navigation part_b_navigation.launch.py world:=custom_casa_obs
```
En RViz: **2D Pose Estimate** → **2D Goal Pose**.

### En el robot real
Se usa el mismo stack de Parte C (MCL + A* + Pure Pursuit); mandás el goal a mano:
```bash
ros2 launch tpf_navigation part_c_real_robot.launch.py robot:=tb4_1 map_yaml:=log/maps/labo_map_v2.yaml
# RViz: 2D Pose Estimate, luego 2D Goal Pose
```

---

## PARTE C — Búsqueda de conos rojos

### Validación con RosBag (open-loop)
```bash
# T1 — bag/map del laberinto original
ros2 launch tpf_navigation part_c_rosbag_validation.launch.py
# T2
ros2 bag play data/rosbags/laberinto_conos --clock
```
Para ensayar con el **mapa del laboratorio ya armado** (`log/maps/labo_map_v2.yaml`) y el bag `data/labo`:
```bash
# T1
ros2 launch tpf_navigation part_c_labo_validation.launch.py rviz:=true
# T2
ros2 bag play data/labo --clock
```

### Robot real (closed-loop)
```bash
ros2 launch tpf_navigation part_c_real_robot.launch.py robot:=tb4_1 map_yaml:=log/maps/labo_map_v2.yaml
```
- **Cambiar de robot** = un argumento: `robot:=tb4_0` o `robot:=tb4_1` (remapea scan/odom/imagen/cmd_vel).
- RViz: **2D Pose Estimate** sobre la pose real → el robot busca conos y navega hacia ellos por el mapa.

---

## Grabar un bag de la sesión (para el informe)
```bash
# liviano (recomendado)
ros2 bag record -o sesion_lab \
  /tb4_1/scan /tb4_1/odom /tb4_1/tf /tb4_1/tf_static \
  /tb4_1/oakd/rgb/preview/image_raw /tb4_1/oakd/rgb/preview/camera_info
# todo (pesado por la camara)
ros2 bag record -a -o sesion_lab_full
```

---

## Troubleshooting (robot real)

| Síntoma | Causa | Fix |
| --- | --- | --- |
| `ros2 topic list` no muestra `/tb4_X/*` | red / discovery | mismo `ROS_DOMAIN_ID` y red que el robot |
| Scan rotado 90°, no localiza | `laser_yaw_rad` | probar `0.0 / 1.5708 / -1.5708` (en occupancy `--laser-yaw-rad` y en `navigation_params_real.yaml`) |
| Robot quieto en el seed | tópicos mal | usar `robot:=tb4_1`; confirmar nombres |
| Nube de partículas diverge | poco ruido para patinaje | subir `alpha1..4` en `navigation_params_real.yaml` |
| Robot no se mueve | `cmd_vel` no llega | `ros2 topic info /tb4_X/cmd_vel`; ajustar remap |
| Pasillo angosto cortado | margen excesivo | bajar `robot_radius_m` |
| Goal/robot fuera del mapa | MCL perdió tracking | re-hacer **2D Pose Estimate** |

**Sim-to-real (consigna 1.5):** no se exige perfección; sí documentar la brecha
(patinaje, iluminación/blur en conos, latencia, mapa vs maze real, calibración).
Decisión clave: la detección del cono se convierte en **coordenada-objetivo para el
planner**, nunca en comando directo → por eso no atraviesa paredes.
