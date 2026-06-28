# Runbook — Parte C en el ROBOT REAL (sesión de laboratorio)

Guía paso a paso para la prueba en el TurtleBot real. Pensada para ejecutarse
dentro de la ventana de 2 h del laboratorio. **Leé "Antes de ir" hoy.**

Stack: localización **MCL** + planner **A\*** + control **Pure Pursuit** + máquina
de estados + **detección de conos rojos**. Objetivo: el robot explora el laberinto,
detecta conos rojos y navega hacia ellos **sin atravesar paredes** (la detección
va al planner, no al control directo).

---

## Antes de ir (hacelo HOY, en casa)

```bash
cd /home/catalina/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tpf_slam tpf_navigation tpf_perception
source install/setup.bash
```

Confirmá que el flujo anda con el bag (ensayo general):
```bash
cd /home/catalina/ws/src/TP_FINAL
# Terminal 1:
ros2 launch tpf_navigation part_c_rosbag_validation.launch.py
# Terminal 2:
ros2 bag play data/rosbags/laberinto_conos --clock
```
Si en RViz ves el robot localizado + el camino verde a los conos, estás listo.

---

## En el lab — paso a paso

> Toda terminal nueva arranca con:
> `source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash && cd ~/ws/src/TP_FINAL`

### 0) Conectar al robot y CONFIRMAR TÓPICOS (lo primero)
```bash
ros2 topic list
```
Anotá los nombres reales de: **scan**, **odom**, **image**, **camera_info**.
Puede que NO sean `/tb4_0/*` (otro número de robot, otro namespace).
- Si difieren → editá `src/tpf_navigation/config/navigation_params_real.yaml`
  (campos `scan_topic` y `odom_topic` en `mcl_localizer` y `astar_planner`),
  **recompilá** (`colcon build --packages-select tpf_navigation && source ~/ws/install/setup.bash`).
- Los de cámara se pasan por launch (`image_topic:=...`, `camera_info_topic:=...`).

### 1) Verificar la orientación del LIDAR
Abrí RViz con el mapa y el scan, o mirá `/scan` crudo. El scan tiene que **caer
sobre las paredes**. Si está **rotado 90°**, ajustá `laser_yaw_rad` en
`navigation_params_real.yaml` (probá `0.0`, `1.5708`, `-1.5708`, `3.1416`),
recompilá y volvé a mirar. **Este es el error más común y rompe la localización.**

### 2) Verificar la cámara y el detector de conos
```bash
ros2 run tpf_perception red_cone_detector_node \
  --ros-args -p image_topic:=<image> -p camera_info_topic:=<camera_info>
```
Acercá un cono rojo y confirmá que lo detecta (logs `Published red-cone goal`).
Si detecta de más/menos, ajustar umbrales HSV en
`src/tpf_perception/config/red_cone_detector.yaml`.

### 3) Mapa
- **Si el laberinto del lab es el mismo del bag** → usás `log/maps/laberinto_map.yaml` (default). ✅
- **Si es distinto** → hay que mapear primero con Parte A en vivo y exportar un mapa nuevo, y pasarlo con `map_yaml:=/ruta/nuevo_map.yaml`.

### 4) Lanzar el stack de Parte C real (closed-loop)
```bash
ros2 launch tpf_navigation part_c_real_robot.launch.py \
  image_topic:=<image> camera_info_topic:=<camera_info>
```
(si los tópicos son `/tb4_0/*`, andá sin argumentos).

### 5) Fijar la pose inicial — IMPRESCINDIBLE
En RViz: **"2D Pose Estimate"** → click sobre la **posición real** del robot en el
mapa, arrastrando en la dirección que mira. Que el **scan (puntos) calce con las
paredes**. MCL re-siembra ahí (tópico `/initialpose`). Repetí hasta que la nube de
partículas se **junte** sobre el robot.

### 6) Operar y observar
El robot pasa a buscar conos. Al detectar uno, publica un **goal** y A\* traza el
**camino sobre el mapa** (esquiva paredes). Pure Pursuit lo sigue.
**Qué mostrar / criterios (consigna 1.5):**
- Autonomía y eficiencia explorando y buscando conos.
- Robustez del filtro ante patinaje (la nube no debe divergir).
- Precisión en la aproximación final al cono.

---

## Ajustes rápidos (dónde tocar) — todo en `navigation_params_real.yaml`

| Síntoma | Causa probable | Ajuste |
| --- | --- | --- |
| Robot localiza mal / scan rotado | `laser_yaw_rad` incorrecto | probar `0.0 / 1.5708 / -1.5708 / 3.1416` |
| MCL no recibe datos (robot quieto en seed) | tópicos mal | corregir `scan_topic` / `odom_topic` |
| Nube de partículas diverge / "salta" | poco ruido de movimiento para el patinaje real | subir `alpha1..4` (p. ej. 0.25) |
| Robot roza/choca paredes | poco margen | subir `robot_radius_m` (0.15) o `inflation_radius_m` |
| Pasillo angosto se "corta" | margen excesivo | bajar `robot_radius_m` (0.11) |
| Robot muy rápido/inestable | control | bajar `v_max` (0.15) |
| Goal del cono cae fuera del mapa | detección espuria | subir umbral de confianza HSV / descartar conos lejanos |

Tras editar el yaml: `colcon build --packages-select tpf_navigation && source ~/ws/install/setup.bash`.

---

## Para el informe — análisis Sim-to-Real (consigna 1.5)

La consigna dice que **no es reprobación** si no sale perfecto, pero **es
condición** analizar la brecha. Tener listo:
- **Deslizamiento mecánico** de ruedas → error de odometría → se mitiga con
  `alpha1..4` más altos y re-localización por LIDAR.
- **Iluminación / motion blur** en la cámara → afecta detección de conos (HSV);
  mitigación: filtrar por tamaño/forma y consistencia temporal.
- **Latencia / pérdida de frames** en hardware real vs reproducción ideal del bag.
- **Mapa**: si el laberinto físico difiere del mapeado, la localización degrada.
- **Calibración** de cámara/extrínsecas (transformada del LIDAR).
- Decisión de diseño clave: la detección visual del cono se convierte en
  **coordenada-objetivo para el planner**, nunca en comando directo → por eso el
  robot no atraviesa paredes aunque "vea" el cono a través de una rejilla.
