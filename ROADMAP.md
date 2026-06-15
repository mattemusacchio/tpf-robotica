# Roadmap TP Final Robótica — Opción 3: Features con Cámara

Este roadmap resume qué hay que construir para el TP final eligiendo la **opción 3 de la Parte A: landmarks visuales con cámara + ArUco Tags + Graph SLAM**.

## Estado inicial de materiales

La documentación quedó en `docs/` y los datos descargados quedaron ordenados así:

```text
docs/
  PRA_TPFinal.pdf
  PRA_TPFinal_Parte_A.pdf
  PRA_TPFinal_Parte_B.pdf
  PRA_TPFinal_Parte_C.pdf
  Flowchart TP Final.png

data/
  README.md
  calibration/
    README.md
    Matrices, coeficientes y estimaciones.docx
  rosbags/
    aruco_estimation/
      metadata.yaml
      aruco_estimation_0.db3
    laberinto/
      metadata.yaml
      laberinto_0.db3
    laberinto_conos/
      metadata.yaml
      laberinto_conos_0.db3
```

Bags principales:

| Bag | Para qué usarlo |
| --- | --- |
| `data/rosbags/aruco_estimation` | Primer detector ArUco, calibración, prueba de distancias y ruido visual. |
| `data/rosbags/laberinto` | SLAM visual + LIDAR definitivo de Parte A. Es el bag largo del laberinto completo. |
| `data/rosbags/laberinto_conos` | Parte C: detección de conos, ajuste de color y validación de búsqueda visual. |

Comando base:

```bash
ros2 bag play data/rosbags/laberinto
```

---

## Objetivo global

Construir un sistema ROS 2 que haga:

1. detección robusta de ArUco Tags con cámara real;
2. Graph SLAM obligatorio usando odometría + observaciones visuales;
3. generación de mapa de ocupación con LIDAR y trayectoria corregida;
4. navegación autónoma en Gazebo usando el mapa y landmarks simulados;
5. despliegue en robot real para buscar conos rojos y navegar hacia ellos sin atravesar paredes.

---

## Fase 0 — Setup, estructura y sanity checks

**Objetivo:** dejar el entorno listo antes de programar lógica pesada.

### Tareas

- Crear workspace ROS 2, por ejemplo:

```text
src/
  tpf_slam/
  tpf_navigation/
  tpf_perception/
  tpf_bringup/
```

- Verificar que los bags reproducen:

```bash
ros2 bag info data/rosbags/aruco_estimation
ros2 bag play data/rosbags/aruco_estimation
```

- Confirmar tópicos:

```bash
ros2 topic list
ros2 topic echo /tb4_0/odom --once
ros2 topic echo /tb4_0/oakd/rgb/preview/camera_info --once
```

- Abrir RViz y visualizar como mínimo:
  - `/tb4_0/scan`
  - `/tb4_0/odom`
  - imagen de cámara, si usan plugin de imagen o ventana OpenCV aparte.

### Resultado esperado

- Workspace compila con `colcon build`.
- Se pueden reproducir los RosBags.
- Se ven odometría, LIDAR, cámara y camera_info.

---

## Fase 1 — Detector ArUco y modelo de medición

**Objetivo:** convertir imágenes en observaciones útiles para SLAM.

### Tareas

- Implementar nodo `aruco_detector_node`.
- Suscribirse a:
  - `/tb4_0/oakd/rgb/preview/image_raw`
  - `/tb4_0/oakd/rgb/preview/camera_info`
- Usar la calibración de `data/calibration/README.md`:
  - `MARKER_SIZE_M = 0.0889`
  - matriz de cámara del TurtleBot 0
  - coeficientes de distorsión del TurtleBot 0
- Detectar IDs de ArUco y estimar pose relativa cámara-tag.
- Publicar detecciones en un tópico propio, por ejemplo:
  - `/aruco/detections`
  - `/landmark_observations`
- Dibujar detecciones sobre la imagen para debug.

### Validación mínima

Con `aruco_estimation`, comparar distancias estimadas contra las del documento:

| Real aprox. | Esperada aprox. |
| ---: | ---: |
| 0.30 m | 0.28 m |
| 0.695 m | 0.72 m |
| 1.01 m | 1.04 m |
| 1.49 m | 1.52 m |

### Resultado esperado

- El nodo detecta tags con ID estable.
- La distancia estimada tiene error razonable.
- Hay una visualización clara para mostrar en la defensa.

---

## Fase 2 — Front-end de SLAM: odometría + observaciones

**Objetivo:** transformar datos crudos en restricciones para Graph SLAM.

### Tareas

- Suscribirse a `/tb4_0/odom`.
- Calcular incrementos relativos entre poses consecutivas:
  - `δtrans`
  - `δθ1`
  - `δθ2`
- Asociar detecciones ArUco con timestamps cercanos de odometría.
- Convertir observaciones cámara-tag al frame del robot.
- Definir estructura de datos para el grafo:
  - nodos de pose: `x_i = (x, y, θ)`
  - nodos de landmark: `l_j = (x, y, id)`
  - aristas odométricas: pose-pose
  - aristas visuales: pose-landmark

### Decisiones importantes

- Usar keyframes en vez de guardar todas las poses si el bag es muy largo.
- Crear un nuevo nodo de pose si:
  - el robot avanzó más de cierto umbral;
  - giró más de cierto umbral;
  - observó un ArUco importante;
  - pasó cierto tiempo.

### Resultado esperado

- Archivo/log con grafo inicial.
- Visualización de `/poses_guardadas` y `/landmarks` preliminares.

---

## Fase 3 — Graph SLAM obligatorio

**Objetivo:** optimizar trayectoria y landmarks usando odometría + ArUco.

### Tareas

- Implementar back-end de optimización del grafo.
- Minimizar errores de:
  - odometría relativa entre poses;
  - observaciones relativas de tags;
  - cierres de lazo cuando se vuelve a ver un mismo tag o zona.
- Incorporar incertidumbre/covarianzas:
  - odometría: más incertidumbre con distancia y giro;
  - ArUco: más incertidumbre si el tag está lejos, borroso o en ángulo malo.
- Detectar loop closures mediante reobservación de IDs ArUco y consistencia geométrica.
- Publicar:
  - `/belief`: trayectoria corregida;
  - `/landmarks`: posiciones optimizadas de tags;
  - `/poses_guardadas`: nodos del grafo.

### Resultado esperado

- Trayectoria corregida más coherente que `/tb4_0/odom`.
- Landmarks fijos y repetibles por ID.
- Loop closures visibles al volver a zonas ya visitadas.

---

## Fase 4 — Mapa de ocupación con LIDAR

**Objetivo:** generar la grilla métrica que se usará para navegación.

### Tareas

- Reproducir `data/rosbags/laberinto` una segunda vez.
- Usar la trayectoria corregida del Graph SLAM.
- Proyectar `/tb4_0/scan` sobre una grilla de ocupación.
- Aplicar modelo inverso de sensor LIDAR:
  - celdas libres a lo largo del rayo;
  - celda ocupada en el impacto;
  - saturación de log-odds.
- Filtrar ruido y limpiar obstáculos fantasma.
- Exportar mapa:
  - `map.yaml`
  - `map.pgm` o `map.png`
  - landmarks con IDs, por ejemplo `landmarks.yaml` o `landmarks.json`.

### Criterios de aceptación del mapa

- Paredes nítidas.
- Pasillos y esquinas reconocibles.
- Sin distorsiones severas ni muros duplicados.
- Apto para planificar con A* o Dijkstra.

---

## Fase 5 — Parte B: navegación autónoma en Gazebo

**Objetivo:** mover el robot de una pose inicial a una pose objetivo usando mapa, localización, planner y control.

### Tareas principales

- Leer pose inicial desde `/initialpose`.
- Leer objetivo desde `/goal_pose`.
- Localizar al robot durante el movimiento con filtro probabilístico:
  - Particle Filter, EKF o variante equivalente.
- Planificar camino seguro sobre mapa de ocupación:
  - A*, Dijkstra, RRT o similar.
- Inflar obstáculos para no pasar demasiado cerca de paredes.
- Implementar path following:
  - Pure Pursuit, Stanley, controlador proporcional o similar.
- Cumplir orientación final, no solo posición.
- Replanificar si cambia el goal.
- Detectar obstáculos no mapeados y esquivarlos.

### Extra por haber elegido cámara

Como en Gazebo no están los ArUco reales, hay que crear un **sensor virtual de landmarks**:

- poblar landmarks simulados con densidad parecida a los tags reales;
- calcular línea de visión robot-landmark;
- no publicar landmarks ocluidos por paredes u obstáculos;
- agregar ruido a distancia/ángulo;
- publicar observaciones compatibles con las usadas en Parte A.

### Máquina de estados sugerida

```text
IDLE
  -> WAIT_INITIAL_POSE
  -> LOCALIZING
  -> WAIT_GOAL
  -> PLANNING
  -> FOLLOWING_PATH
  -> AVOIDING_OBSTACLE
  -> REPLANNING
  -> ALIGNING_FINAL_YAW
  -> GOAL_REACHED
  -> ERROR_RECOVERY
```

### Mundos de prueba

```bash
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
ros2 launch turtlebot3_custom_simulation custom_casa_obs.launch.py
```

Opcional:

```bash
ros2 launch turtlebot3_custom_simulation custom_casa_obs2.launch.py
```

---

## Fase 6 — Parte C: visión para conos rojos

**Objetivo:** detectar conos rojos, ignorar distractores y convertir detecciones en objetivos navegables.

### Tareas

- Usar `data/rosbags/laberinto_conos`.
- Implementar nodo `red_cone_detector_node`.
- Segmentar color rojo en HSV o espacio robusto equivalente.
- Filtrar por geometría del cono:
  - tamaño mínimo;
  - forma triangular/trapezoidal;
  - consistencia temporal;
  - posición en imagen.
- Ignorar conos no rojos.
- Estimar dirección o posición relativa del cono.
- Enviar objetivo al planner, no al controlador directo.

### Punto crítico: paredes con huecos

Si el robot ve un cono a través de una pared/rejilla, **no debe avanzar en línea recta hacia él**.

La detección visual debe convertirse en una coordenada objetivo aproximada y el planificador debe calcular un camino válido sobre el mapa, esquivando muros reales.

---

## Fase 7 — Integración en robot real

**Objetivo:** llevar lo validado en RosBags y Gazebo al TurtleBot real.

### Tareas

- Confirmar nombres reales de tópicos en laboratorio.
- Ajustar parámetros:
  - ruido odométrico;
  - umbrales de ArUco;
  - umbrales HSV de rojo;
  - velocidades máximas;
  - tolerancias de llegada.
- Validar por módulos antes de correr todo:
  1. cámara;
  2. LIDAR;
  3. odometría;
  4. detección de conos;
  5. localización;
  6. planificación;
  7. control.
- Grabar videos y logs para informe/defensa.

### Si falla en hardware

No es necesariamente reprobación, pero el informe debe explicar causas y mitigaciones:

- iluminación distinta;
- motion blur;
- deslizamiento de ruedas;
- ruido de LIDAR;
- oclusiones;
- calibración imperfecta;
- latencia o pérdida de frames.

---

## Fase 8 — Informe y defensa

**Objetivo:** documentar bien la solución y preparar la defensa de 20 minutos.

### Informe técnico

Debe incluir:

- arquitectura general del sistema;
- decisiones de diseño;
- modelo de cámara y ArUco;
- formulación de Graph SLAM;
- loop closure;
- generación de mapa de ocupación;
- localización y planificación;
- máquina de estados de Parte B y C;
- resultados con imágenes del mapa;
- diagnóstico sim-to-real si hubo problemas.

### Defensa

Preparar slides. No alcanza abrir el código o el informe.

Material recomendado:

- video de detector ArUco sobre RosBag;
- comparación odometría vs trayectoria Graph SLAM;
- mapa final;
- demo de navegación en Gazebo;
- video o evidencia del robot real/conos;
- diagrama de máquina de estados.

---

## Prioridad de implementación: MVP razonable

Si el tiempo aprieta, priorizar en este orden:

1. Detector ArUco funcionando y medido con `aruco_estimation`.
2. Graph SLAM básico con odometría + reobservación de ArUco.
3. Mapa de ocupación usable desde trayectoria corregida.
4. Navegación Gazebo: initialpose, goal_pose, planner, path following.
5. Sensor virtual de landmarks para cerrar coherencia con opción 3.
6. Detección de conos rojos con RosBag.
7. Integración real y videos.
8. Pulido de informe y defensa.

---

## Riesgos principales

| Riesgo | Mitigación |
| --- | --- |
| ArUco inestable por blur o poca densidad | Filtrar detecciones, usar keyframes, descartar mediciones lejanas/oblicuas. |
| Graph SLAM se vuelve complejo | Empezar offline y simple; después convertir a nodo ROS. |
| Mapa sale distorsionado | Ajustar loop closures y usar trayectoria optimizada, no odometría pura. |
| Gazebo no tiene landmarks reales | Implementar sensor virtual con línea de visión y ruido. |
| Conos vistos a través de paredes | Pasar detección al planner, nunca al control directo. |
| Robot real no replica simulación | Registrar logs/videos y documentar análisis sim-to-real. |

---

## Definition of Done del TP

El trabajo está en buen estado cuando pueden mostrar:

- `colcon build` compila todos los paquetes;
- detector ArUco corriendo sobre RosBag;
- Graph SLAM publica trayectoria corregida y landmarks;
- mapa de ocupación exportado;
- navegación en Gazebo desde `/initialpose` hasta `/goal_pose`;
- replanning ante nuevo objetivo u obstáculo;
- detector de conos rojos validado con RosBag;
- máquina de estados documentada;
- slides listas con videos/evidencia;
- informe con análisis técnico y sim-to-real.
