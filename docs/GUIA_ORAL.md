# Guía de defensa oral — TP Final Robótica (Opción 3)

> Sistema ROS 2 (Humble) con tres paquetes: `tpf_slam` (Parte A), `tpf_navigation` (Parte B), `tpf_perception` (Parte C).
> Esta guía conecta el **código real** (referencias `archivo:línea`) con la **teoría de las clases**. Está pensada para releer antes del oral.

## Índice
- [0. Mapa mental de todo el TP (el "elevator pitch")](#0-mapa-mental)
- [1. Teoría general (te la pueden pedir para comparar)](#1-teoria-general)
  - 1.1 Filtro de Bayes recursivo — la base de todo
  - 1.2 La familia de filtros: histograma, Kalman, EKF, UKF, partículas
  - 1.3 Kalman / EKF / UKF en detalle
  - 1.4 Comparación de métodos de SLAM: EKF-SLAM vs FastSLAM vs Graph SLAM
- [2. Parte A — Graph SLAM + ICP + mapa de ocupación](#2-parte-a)
- [3. Parte B — Navegación (MCL + A\* + Pure Pursuit + FSM)](#3-parte-b)
- [4. Parte C — Percepción (ArUco + conos rojos)](#4-parte-c)
- [5. Preguntas transversales y "trampa"](#5-preguntas-transversales)
- [6. Sim-to-real: qué falló y cómo defenderlo](#6-sim-to-real)

---

<a name="0-mapa-mental"></a>
## 0. Mapa mental de todo el TP (el "elevator pitch")

Si te piden que expliques el sistema en 2 minutos:

> "Elegimos la **Opción 3: features con cámara**. En la **Parte A** hacemos **Graph SLAM offline** sobre un rosbag: un front-end arma un grafo de poses (keyframes de odometría) con landmarks **ArUco** (medición range/bearing por PnP) y restricciones de scan-matching **ICP** del LiDAR; un back-end de **mínimos cuadrados no lineales** optimiza toda la trayectoria y los landmarks. Con la trayectoria corregida proyectamos el LiDAR en una **grilla de ocupación log-odds** y exportamos un mapa Nav2. En la **Parte B** navegamos autónomamente: **MCL** (filtro de partículas) para localizar, **A\*** sobre el costmap inflado para planificar, **Pure Pursuit** para seguir el camino, todo coordinado por una **máquina de estados**. En la **Parte C**, un detector de **conos rojos** en HSV convierte la detección en un **goal** para el planner (nunca en comando directo), de modo que aunque vea el cono a través de una reja no atraviese la pared."

Los tres bloques comparten un hilo teórico: **estimación probabilística de estado** (SLAM y localización son el mismo filtro de Bayes visto de dos maneras: batch/smoothing vs recursivo/filtering).

**Flujo de datos global:**
```
                 PARTE A (offline, sobre rosbag)
  rosbag ──► front-end (grafo: odom + ArUco + ICP) ──► back-end (least squares)
                                                            │
                                              trayectoria optimizada
                                                            │
                                              occupancy grid (log-odds) ──► mapa .pgm/.yaml
                                                            │
                 PARTE B (online)                           ▼
  /odom + /scan + /map ──► MCL ──► /pose_estimate ──► A* ──► /plan ──► Pure Pursuit ──► /cmd_vel
                                        ▲                                                  ▲
                 PARTE C               │                                                  │
  cámara ──► detector cono (HSV) ──► /goal_pose ────────────► A* replanifica ────────────┘
```

---

<a name="1-teoria-general"></a>
## 1. Teoría general (te la pueden pedir para comparar)

Esta sección es la que pediste reforzar. **Ojo:** de todos estos filtros, en el código sólo implementaron el **filtro de partículas (MCL)** para localización y **mínimos cuadrados** para Graph SLAM. Kalman/EKF/UKF **no** están en el código, pero te los pueden preguntar para que compares y justifiques por qué elegiste otra cosa. Sabé explicarlos y, sobre todo, **por qué NO los usaste**.

### 1.1 Filtro de Bayes recursivo — la base de todo

Todo el problema de "dónde estoy" (localización) y "dónde estoy y cómo es el mundo" (SLAM) es estimar una **creencia** (belief) sobre el estado `x` dado todo lo que medí y todos los controles que apliqué:

```
Bel(xₜ) = p(xₜ | z₁:ₜ, u₁:ₜ)
```

El filtro de Bayes lo calcula **recursivamente** en dos pasos:

1. **Predicción** (motion update) — uso el control/odometría `uₜ` y el modelo de movimiento:
   ```
   Bel⁻(xₜ) = ∫ p(xₜ | uₜ, xₜ₋₁) · Bel(xₜ₋₁) dxₜ₋₁
   ```
   La creencia se **ensancha** (agrego incertidumbre; me muevo, sé menos).

2. **Corrección** (measurement update) — uso la medición `zₜ` y el modelo de sensor:
   ```
   Bel(xₜ) = η · p(zₜ | xₜ) · Bel⁻(xₜ)
   ```
   La creencia se **afina** (la medición me da información; `η` es normalización).

Dos supuestos clave (Markov):
- El estado es **completo**: el futuro sólo depende del presente, no del pasado.
- Las mediciones son **condicionalmente independientes** dado el estado.

**Todos los filtros de abajo son la misma ecuación** — sólo cambian *cómo representan `Bel(x)`*.

### 1.2 La familia de filtros: cómo representa cada uno la creencia

| Filtro | Representa `Bel(x)` como… | No linealidad | Multimodal | Costo | Dónde aparece en el TP |
| --- | --- | --- | --- | --- | --- |
| **Discreto / histograma** | grilla de celdas con probabilidad | sí (exacto) | sí | alto (memoria, resolución fija) | clase 08 (base conceptual) |
| **Kalman (KF)** | 1 gaussiana `(μ, Σ)` | **no** (sólo lineal) | no | muy bajo | teoría (clase 10) |
| **EKF** | 1 gaussiana | linealiza con **Jacobiano** (Taylor 1er orden) | no | bajo | teoría (clase 11, EKF-SLAM clase 13) |
| **UKF** | 1 gaussiana | **unscented transform** (sigma points, sin Jacobiano) | no | bajo-medio | teoría (clase 11b) |
| **Partículas (PF/MCL)** | N muestras pesadas | sí (arbitraria) | **sí** | alto (N×rayos) | **implementado**: `mcl_localizer.py` |

Regla mental: **gaussiana = unimodal = no puede decir "estoy en A o en B"**. Partículas = puede representar cualquier forma, incluida multimodal → es lo que necesitás para localización global / robot secuestrado.

### 1.3 Kalman / EKF / UKF en detalle

**Filtro de Kalman (KF).** Es el filtro de Bayes **óptimo** cuando (a) los modelos de movimiento y sensor son **lineales** y (b) todo el ruido es **gaussiano**. La creencia es una gaussiana `(μ, Σ)` que se mantiene gaussiana en cada paso. Ciclo:
- Predicción: `μ⁻ = A·μ + B·u`, `Σ⁻ = A·Σ·Aᵀ + R` (R = ruido de proceso).
- Corrección: ganancia de Kalman `K = Σ⁻·Cᵀ·(C·Σ⁻·Cᵀ + Q)⁻¹`; `μ = μ⁻ + K·(z − C·μ⁻)`; `Σ = (I − K·C)·Σ⁻`.
- Intuición de `K`: **cuánto le creo a la medición vs a la predicción**. Si el sensor es muy preciso (Q chico), `K` grande → me muevo hacia la medición. Si el sensor es ruidoso, `K` chico → confío en mi predicción.

**EKF (Extended Kalman Filter).** El mundo real **no es lineal** (un robot que gira, un sensor range/bearing). El EKF **lineariza** los modelos con una **expansión de Taylor de primer orden**, evaluando el **Jacobiano** (matriz de derivadas parciales) en la media actual. Con eso reutiliza las ecuaciones del KF. Limitaciones:
- La linealización introduce error si la función es muy curva o la incertidumbre es grande → puede volverse **inconsistente** (subestima su propia incertidumbre).
- Linealiza **una sola vez por paso** y no puede deshacerlo (importante para SLAM).
- Sigue siendo **unimodal**.

**UKF (Unscented KF).** En vez de linealizar con derivadas, usa la **unscented transform**: elige un conjunto determinístico de **sigma points** alrededor de la media, los pasa por la función **no lineal exacta**, y recompone media y covarianza a partir de los puntos transformados. Ventajas sobre EKF:
- **No necesita Jacobianos** (útil si la función es difícil de derivar).
- Captura la no linealidad hasta **2do orden** (EKF sólo 1er orden) → más preciso y consistente.
- Costo similar al EKF. Sigue siendo unimodal/gaussiano.

**Pregunta típica: "¿Diferencia entre EKF y UKF?"** → Ambos mantienen una gaussiana; EKF la propaga **linealizando con Jacobianos** (Taylor 1er orden), UKF la propaga **muestreando sigma points y pasándolos por la función real** (sin derivadas, precisión 2do orden). UKF es más preciso cuando la no linealidad es fuerte, sin costo extra grande.

### 1.4 Comparación de métodos de SLAM (¡pregunta casi segura!)

Hay tres grandes familias. **Elegimos Graph SLAM.** Tenés que poder defender por qué.

| | **EKF-SLAM** (clase 13) | **FastSLAM** (clase 14) | **Graph SLAM** (clase 16) ← *el nuestro* |
| --- | --- | --- | --- |
| Idea | Una gaussiana gigante sobre pose + todos los landmarks | Partículas para la trayectoria + un EKF chico por landmark (Rao-Blackwell) | Grafo de poses/landmarks; se optimiza toda la historia (smoothing) |
| Estado | `[pose, l₁..lₙ]` con covarianza **densa** | 1 trayectoria por partícula + landmarks independientes | todos los nodos a la vez |
| Escala | **O(n²)** por la covarianza densa | O(M·log n) con M partículas | disperso (**sparse**); escala bien |
| Linealización | una vez, **irreversible** | por partícula | **re-lineariza en cada iteración** |
| Loop closure | difícil de "corregir hacia atrás" | depleción de partículas en loops largos | **natural**: es agregar una arista |
| Data association | frágil (una mala asociación contamina todo) | por partícula (más robusto) | ArUco la resuelve con el ID |
| Online/offline | online (recursivo) | online | típicamente **batch/offline** (el nuestro lo es) |

**Por qué Graph SLAM y no los otros (respuesta lista para el oral):**
1. **Loop closures**: Graph SLAM incorpora restricciones entre poses no consecutivas simplemente agregando una arista, y **re-lineariza en cada iteración**, corrigiendo el drift hacia atrás. El EKF no puede deshacer una linealización pasada.
2. **Escala y estructura dispersa**: cada arista toca sólo 2 nodos → el Hessiano es disperso y se resuelve eficiente (Sparse Pose Adjustment). El EKF-SLAM tiene covarianza densa O(n²).
3. **Flexibilidad de sensores**: fusionar odom + ArUco + ICP es sólo agregar tipos de arista (la clase 16 lo destaca: "flexible para agregar GPS, IMU…").
4. **Es offline**: el rosbag ya está grabado; no necesito un filtro recursivo online, me conviene un batch que optimice global de una.
5. FastSLAM sufre **depleción de partículas** en loops largos y no aporta ventaja acá.

---

<a name="2-parte-a"></a>
## 2. Parte A — Graph SLAM + ICP + mapa de ocupación

**Paquete:** `tpf_slam`. **Estructura clásica front-end / back-end** (clase 16).

```
rosbag ─► [1] offline_rosbag_graph_builder.py  (FRONT-END: arma el grafo)
       ─► [2] graph_slam_backend.py            (BACK-END: optimiza, scipy least_squares)
       ─► [3] second_pass_icp.py               (re-ICP con poses optimizadas → más loop closures) → back-end otra vez
       ─► [4] occupancy_grid_builder.py        (mapa log-odds con la trayectoria corregida)
```

### 2.1 Qué implementaron

**Front-end** (`offline_rosbag_graph_builder.py`, clase `OfflineRosbagGraphBuilder:104`). Lee el `.db3` directo por SQLite (el bag dura ~23 min). Produce un grafo con 4 elementos:
- **Nodos pose (keyframes)** por **muestreo espacial**, no por mensaje: `_should_add_keyframe:629` dispara cuando avanzó ≥ 0.20 m, giró ≥ 0.175 rad (~10°) o pasaron 2 s. Cada keyframe guarda `(x,y,θ)` de odometría.
- **Aristas odométricas**: desplazamiento relativo entre keyframes expresado en el frame del anterior (`_add_keyframe:640`).
- **Nodos landmark ArUco + aristas visuales** (`_on_image:514`): descarta frames borrosos (varianza del Laplaciano < 60), detecta `DICT_4X4_50`, estima pose con `cv2.aruco.estimatePoseSingleMarkers` (PnP), y convierte a observación planar `(range, bearing)` en el frame del robot (`_planar_observation:593`).
- **Aristas ICP (LiDAR)** (`_build_icp_edges:268`): secuenciales (refinan odometría) y **loop closures** por proximidad espacial (revisit con salto de índice ≥ 25 y distancia ≤ 1.2 m).

**Gating de ArUco** (control de outliers, `_on_image:550-580`): rechaza si el marcador está detrás de la cámara, rango implausible (>3 m), error de reproyección > 2 px, ángulo rasante > 72° (visto de canto), o si el landmark "saltaría" > 0.6 m (descarta flips de 180°). Además poda landmarks con < 3 observaciones (`snapshot:415`).

**Back-end** (`graph_slam_backend.py`, `GraphSlamBackend:88`): minimiza `½ Σ eᵀΩe` con `scipy.optimize.least_squares`.
- **Gauge anchor**: fija la primera pose (`_make_layout:523-525`) — sin esto el grafo "flota" (invariante a transformación rígida global → indeterminado).
- **Residuos** normalizados por σ (`residuals:284`): pose-pose (3 comp.) y visual range/bearing (2 comp.). Dividir por σ ≡ multiplicar por `Ω^{1/2}`.
- **Modelo de ruido** (`BackendConfig:45`): odometría muy confiable (σ=0.03), ICP casi (≈0.05), ArUco más ruidoso y **σ que crece con la distancia** (`_visual_sigmas:446`: `σ_range = 0.05 + 0.04·r²`). Un ArUco lejano pesa menos.
- **Jacobiano analítico disperso** (`jac:325`) → cada evaluación de ~100 ms a <1 ms.
- Solver con **pérdida robusta Cauchy** (atenúa outliers), reporta `cost_reduction_ratio` y bandera `usable_solution` (costo bajó y es finito, aunque el solver pare por `max_nfev`).

**ICP** (`icp.py`, `icp_match:88`): **point-to-point 2D con SVD** (clase 18). Por iteración: aplica transformada, **asocia por vecino más cercano** (`cKDTree`), **rechaza outliers** (Trimmed ICP, 80% + gate 3×mediana), **calcula R,t cerrado por SVD** (`W = ΣΔsource·Δtargetᵀ`, `R = Vᵀ·Uᵀ`), compone y chequea convergencia. Devuelve `fitness` (fracción inliers) usado para aceptar/rechazar aristas.

**Second-pass ICP** (`second_pass_icp.py`): con poses ya optimizadas como init, re-corre ICP con correspondencia más ajustada y radio de loop mayor → encuentra más y mejores loop closures. Materializa la iteración front↔back de la clase 16.

**Mapa de ocupación** (`occupancy_grid_builder.py`, `OccupancyGridBuilder:113`): grilla **log-odds** con **poses conocidas** (la trayectoria optimizada) = "mapeo con poses conocidas" (clase 12). Interpola la pose al stamp de cada scan, hace **ray casting con Bresenham** (`_trace_ray:321`), suma log-odds (free −0.35, occupied +0.85, `_update:337`), convierte a probabilidad `p = 1 − 1/(1+exp(l))` y umbraliza a libre/ocupado/desconocido. Exporta `.pgm/.yaml/.png` para Nav2.

### 2.2 Teoría (clases 16, 18, 12)

- **Graph SLAM** (16): nodos = poses/landmarks, aristas = restricciones espaciales con incertidumbre. Costo global `x* = argmin Σ eᵢⱼᵀ Ωᵢⱼ eᵢⱼ` (forma de Mahalanobis). `Ω` (matriz de información = inversa de covarianza) **pesa** cada restricción. Se resuelve con **Gauss-Newton** (linealizar `e(x+Δx)≈e+JΔx`, resolver `H·Δx=−b` con `H=JᵀΩJ`), aprovechando que `H` es **disperso**.
- **ICP** (18): con correspondencias conocidas → solución cerrada por SVD; con correspondencias desconocidas → iterar {asociar (vecino más cercano) → estimar R,t → aplicar} hasta converger. **Converge sólo con buen init** (por eso inicializamos con odometría). Point-to-point vs **point-to-plane** (minimiza distancia al plano tangente, mejor en superficies suaves pero necesita normales).
- **Occupancy grid** (12): celdas binarias **independientes**, mundo **estático**. Filtro de Bayes binario en **log-odds** (`l_t = l_{t-1} + inv_sensor − l_0`): el producto de Bayes se vuelve **suma** → eficiente y estable. Inverse sensor model: celdas antes del hit → libres, celda del hit → ocupada.

### 2.3 Decisiones de diseño
- **Graph SLAM** (ver §1.4): loop closures naturales, dispersión, flexibilidad, offline.
- **ArUco como landmark**: da **asociación de datos gratis** (ID único, sin ambigüedad) y **loop closures triviales** (mismo ID desde keyframes lejanos).
- **Point-to-point ICP**: en LiDAR 2D las nubes son ralas y estimar normales (para point-to-plane) es ruidoso; con buen init de odometría y trimming, point-to-point es robusto y barato.
- **Gating multicapa + σ(distancia) + Cauchy**: defensa en capas contra outliers del PnP (flips de 180°, marcadores lejanos/borrosos).
- **Sin inflación horneada** en el mapa: la inflación es del costmap de Nav2 (Parte B/C), no del mapa estático.

### 2.4 Preguntas probables (Parte A)
1. **¿Qué es un loop closure y cómo lo detectan?** Restricción entre poses **no consecutivas** que corresponden al mismo lugar; corrige el drift. Detección: (a) reobservación del mismo **ID ArUco**; (b) **ICP** entre keyframes cercanos con salto de índice ≥ 25. Se acepta sólo si el ICP converge, tiene fitness alto y **no discrepa de la odometría** (anti perceptual-aliasing).
2. **¿Por qué la matriz de información / dividir por σ?** Porque no todas las mediciones son igual de confiables; `Ω=Σ⁻¹` pesa cada residuo por su precisión. Dividir por σ = multiplicar por `Ω^{1/2}`. La odometría (σ=0.03) tira más que un ArUco lejano (σ∝r²).
3. **¿Qué pasa si el ICP converge mal?** Cae en mínimos locales / falsos matches (pasillos parecidos). Mitigación: init con odometría, exigir `fitness ≥ 0.65`, y rechazar loop closures que discrepen de la odometría > 0.4 m / 0.3 rad.
4. **¿Point-to-point o point-to-plane?** Point-to-point con SVD; en LiDAR 2D las normales son poco confiables.
5. **¿Cómo obtienen el mapa del grafo optimizado?** El grafo da la trayectoria corregida (poses conocidas); reproyectan cada scan con log-odds + Bresenham (filtro de Bayes binario, clase 12).
6. **¿Por qué log-odds?** El producto de Bayes se vuelve suma → eficiente y estable (no colapsa a 0/1).
7. **¿Por qué fijan la primera pose (gauge)?** El problema es invariante a transformación rígida global → indeterminado; anclar la 1ª pose lo hace único.
8. **¿Con qué método optimizan?** `scipy.least_squares` (trust-region / Levenberg-Marquardt, variante robusta de Gauss-Newton) con Jacobiano analítico disperso y pérdida Cauchy.
9. **¿Front-end vs back-end?** Front-end arma el grafo desde datos crudos; back-end optimiza las posiciones de los nodos. El second-pass ICP es el back-end realimentando al front-end.

### 2.5 Gotchas Parte A
- **Todo offline** (no SLAM online; la demo sólo reproduce el resultado optimizado).
- **Odometría como aproximación de `map`** (se apoya en el bajo drift del TB4).
- **`Ω` diagonal** (σ independientes, sin covarianzas cruzadas) — simplificación.
- **`success=false` frecuente**: el solver para por `max_nfev`; se confía en `usable_solution` (defendé: reducción de costo ~42% con solución finita = óptimo local válido).
- **Mapa 2D de mundo 3D**: obstáculos fuera del plano del láser no se mapean.

---

<a name="3-parte-b"></a>
## 3. Parte B — Navegación (MCL + A\* + Pure Pursuit + FSM)

**Paquete:** `tpf_navigation`. Stack clásico en 4 nodos:
```
/odom + /scan + /map ─► mcl_localizer ─► /pose_estimate ─► astar_planner ─► /plan ─► pure_pursuit ─► /cmd_vel
                                              ▲ (/goal_pose)                                   
                        navigation_sm  ◄──── orquesta todo (estados, recovery, evasión)
```

### 3.1 MCL — `mcl_localizer.py`
Cada partícula = hipótesis `[x,y,θ]` (`_particles` N×3, `:118`). **Es MCL clásico** (N fijo), pese al nombre "AMCL".

- **Inicialización**: uniforme sobre celdas libres (localización global) o gaussiana alrededor de una pose (tracking). Re-siembra desde RViz vía `/initialpose`.
- **Predicción — modelo de odometría** (`_predict:280`): descompone en `rot1 → trans → rot2` y agrega ruido gaussiano con desvíos `alpha1..4` (es el `sample_motion_model_odometry` de Thrun, clase 06). Si el robot está quieto **saltea la predicción pero igual corrige con el LiDAR** (si no, nunca convergería parado).
- **Corrección — likelihood field** (`_update_lidar:301`): precomputa `distance_transform_edt` (distancia de cada celda al obstáculo más cercano, **una vez** por mapa, `:172`); proyecta el endpoint de cada rayo y pesa con `p = z_hit·exp(−d²/2σ²) + z_rand`. `z_rand` evita el colapso de pesos ante obstáculos fuera del mapa. Trabaja en **log-space**.
- **Corrección — landmarks ArUco** (`_update_landmarks:364`): modelo range/bearing, con más peso (`lm_weight=2.0`) porque un ID desambigua globalmente.
- **Resampling** (`_normalize_and_resample:396`): calcula **N_eff = 1/Σwᵢ²** y **sólo resamplea si N_eff < N/2** (adaptativo, preserva diversidad). Usa **low-variance / systematic resampling** (O(N), baja varianza).
- **Salida**: pose = media pesada (yaw con `atan2(Σw·sinθ, Σw·cosθ)`); publica `/pose_estimate`, el TF `map→odom`, y la nube de partículas. Detecta "perdido" si `var_x+var_y > 2.0`.

### 3.2 A\* — `astar_planner.py`
- **Costmap con inflación** (`_build_costmap:240`): zona **letal** si `dist < robot_radius` (0.13 m); zona de **gradiente exponencial** hasta `inflation_radius` (0.28 m) que empuja el camino al centro del pasillo sin prohibirlo.
- **A\*** (`_astar:56`): grilla de **8 vecinos** (recto 1.0, diagonal √2), heurística **euclidiana** (`:79`), **estado aumentado con heading** `(col,row,dir)` para penalizar giros (`turn_weight=1.0`). Costo `g = step + cell_cost + turn_weight·turn`.
- **Utilidades**: `_snap_to_free` (BFS si start/goal caen en celda letal), `_smooth` (media móvil para un path más seguible).
- **Capa dinámica** (`_on_scan:286`): confirma obstáculo sólo si aparece en **≥2 de 3 scans** (voto), replanifica a 1 Hz sólo si el path se invalidó.

### 3.3 Pure Pursuit — `pure_pursuit.py`
- **Lookahead** (`_find_lookahead:181`): primer punto del path a distancia ≥ `lookahead_m` (0.40 m).
- **Ley de control** (`_follow:137`): `alpha` = ángulo al lookahead en frame del robot; **curvatura** `γ = 2·sin(alpha)/Ld`; comando `omega = γ·v` (robot diferencial). Rampa de velocidad al acercarse; **stop-and-turn** si `|alpha| > 40°` (gira en el lugar antes de avanzar).
- **Alineación final** (`_align:193`): Pure Pursuit no controla orientación final → fase de **control proporcional** sobre el yaw hasta ~5°.

### 3.4 Máquina de estados — `navigation_sm.py`
9 estados (IDLE, LOCALIZING, WAIT_GOAL, PLANNING, FOLLOWING_PATH, AVOIDING_OBSTACLE, ALIGNING_FINAL_YAW, GOAL_REACHED, ERROR_RECOVERY), loop a 10 Hz. En LOCALIZING **gira en el lugar** para que las partículas converjan. Aborta a LOCALIZING si MCL se pierde, a ERROR_RECOVERY si se atasca (no se movió >5 cm en 8 s). Máximo 3 recuperaciones.

### 3.5 Teoría (clases 06, 07, 08, 09, Tutorial 12)
- **Filtro de partículas / MCL** (09): la posterior se aproxima con muestras pesadas. **Importance sampling**: la **proposal es el modelo de movimiento**, el **peso es el modelo de sensor** (`w ∝ p(z|x)`). Por eso en el código el peso es sólo el likelihood: el movimiento ya está "horneado" en de dónde salieron las partículas.
  - **¿Por qué resampling?** Con N finito, sin él el peso degenera en pocas partículas (el resto son hipótesis muertas). Resamplear las concentra donde importa.
  - **Kidnapped robot / particle deprivation**: secuestro = teletransporte sin aviso (necesita partículas dispersas o inyección aleatoria); deprivation = por azar del resampling no queda ninguna partícula cerca de la verdad. Mitigación: resampling adaptativo (N_eff), más partículas.
- **Modelo de movimiento** (06): `⟨δrot1, δtrans, δrot2⟩` con ruido dependiente del movimiento. α1=rot-por-rot, α2=rot-por-trans, α3=trans-por-trans, α4=trans-por-rot.
- **Modelo de sensor** (07): **beam model** (mezcla de 4: hit/short/max/rand, con ray-casting, exacto pero lento y poco suave) vs **likelihood field** (sólo el endpoint sobre un campo de distancias precomputado; eficiente y suave, pero ignora la física del haz). **Implementaron likelihood field.**
- **A\***: óptimo si la heurística es **admisible** (no sobreestima) y consistente. La euclidiana es cota inferior del costo real en la grilla → admisible.
- **Pure Pursuit** (Tut. 12): de la geometría del arco, `r = L²/(2x)`, curvatura `γ = 2·sin(alpha)/L`, y `ω = v·γ`. `γ ∝ 1/L`: el lookahead es el parámetro central.

### 3.6 Decisiones de diseño
- **MCL en vez de EKF**: el EKF es **unimodal** → no hace localización global ni maneja ambigüedad (mapas simétricos). MCL representa distribuciones multimodales y arbitrarias, sin linealizar. Costo: más caro (N×rayos).
- **Likelihood field en vez de beam model**: eficiente (campo precomputado) y suave (converge mejor). `z_rand` recupera robustez ante obstáculos no mapeados.
- **Heurística euclidiana**: admisible → garantiza óptimo.
- **Inflación en dos zonas**: letal (radio robot) + gradiente (empuja al centro).
- **α1..4 se duplican en el robot real** (`navigation_params_real.yaml:20-23`) para absorber el **patinaje mecánico**; partículas suben de 500 a 2500.
- **`laser_yaw_rad = π/2`** en el TB4 real (LiDAR rotado 90°): debe coincidir en MCL, A\* y SM o toda la geometría queda rotada.

### 3.7 Preguntas probables (Parte B)
1. **¿Por qué resampling?** (ver arriba) evita la degeneración de pesos.
2. **¿Likelihood field vs beam model?** Endpoint sobre campo precomputado: más eficiente y suave; ignora la física del haz.
3. **¿A\* es óptimo?** Sí, con heurística admisible/consistente; usan euclidiana.
4. **¿Lookahead muy chico o grande?** Chico → oscila/inestable; grande → corta curvas, menos preciso. Usan 0.40 m + stop-and-turn para curvas cerradas.
5. **¿Cómo se recupera MCL si se pierde?** Detecta LOST por varianza → la SM gira en LOCALIZING para re-observar; o re-siembra desde RViz. **Limitación**: no inyecta partículas aleatorias → mala recuperación ante secuestro real.
6. **¿MCL vs AMCL?** AMCL agrega N adaptativo (KLD-sampling) e inyección de partículas aleatorias (recupera del secuestro). El código es **MCL clásico** con resampling adaptativo por N_eff (que no es lo mismo).
7. **¿Qué son α1..4 y por qué se suben en real?** Coeficientes del ruido de odometría; se duplican para modelar más patinaje mecánico.
8. **¿Por qué la proposal es el movimiento y el peso el sensor?** Por importance sampling: propagando con `p(x|x',u)`, el peso `w=f/g` se simplifica a `∝ p(z|x)`.
9. **¿Para qué `z_rand`?** Piso de probabilidad; sin él un rayo "imposible" (obstáculo no mapeado) anularía la partícula.
10. **¿Orientación final?** Fase ALIGNING con control proporcional (Pure Pursuit sólo controla posición).
11. **¿Systematic resampling vs ruleta?** Systematic es O(N) y de menor varianza (un aleatorio + saltos equiespaciados sobre la CDF).
12. **¿Qué es N_eff?** `1/Σwᵢ²`; mide degeneración; resamplea sólo si < N/2.
13. **¿Por qué gira en LOCALIZING?** Para que el LiDAR vea distintas paredes y desambigüe rápido.

### 3.8 Gotchas Parte B
- **No es AMCL real** (sin KLD ni inyección aleatoria).
- **"Perdido" por varianza es débil**: el filtro puede estar *confiadamente equivocado* (baja varianza, pose errónea) en entornos simétricos.
- **Likelihood field ve "a través" de paredes finas** (ignora oclusiones).
- **Independencia de rayos es falsa** → sobre-agudiza los pesos (over-confidence); el stride lo mitiga por accidente.
- **Consistencia del montaje del láser** entre los 3 nodos es crítica (el yaw=π/2 rompía en el robot real).
- **Pure Pursuit ataja curvas cerradas**; el stop-and-turn es un parche.

### 3.9 Cheat-sheet sim → real
| Parámetro | Sim | Real | Por qué |
|---|---|---|---|
| `num_particles` | 500 | 2500 | más dispersión real |
| `alpha1..4` | 0.10/0.10/0.05/0.05 | 0.20/0.20/0.10/0.10 | patinaje |
| `z_rand` | 0.30 | 0.20 | robustez obstáculos no mapeados |
| `laser_yaw_rad` | 0 | π/2 | LiDAR rotado 90° en TB4 |
| `v_max` | 0.20 | 0.18 | más lento en real |

---

<a name="4-parte-c"></a>
## 4. Parte C — Percepción (ArUco + conos rojos)

**Paquete:** `tpf_perception`. Dos nodos de visión sobre el RGB del OAK-D. **Idea central (repetila): la percepción produce un OBJETIVO, no una ACCIÓN.**

### 4.1 ArUco — `aruco_detector_node.py`
Alimenta el **SLAM de la Parte A** (no controla el robot). Pipeline (`_on_image:204`):
1. A **escala de grises** (ArUco codifica en blanco/negro, el color no importa).
2. `detectMarkers` (`DICT_4X4_50`): umbralado adaptativo → contornos cuadrangulares → corrección de perspectiva → muestreo de la grilla interna → decodificación del ID con corrección de errores.
3. Refinamiento **sub-pixel** de esquinas (clave para la precisión).
4. **PnP**: `estimatePoseSingleMarkers` (marcador de **0.0889 m** conocido) → `rvec`, `tvec` en el frame óptico.
5. Convierte a observación planar range/bearing en `base_link` (`aruco_geometry.py:63`), aplicando el **offset de montaje** (~0.06 m).

**De pixel a 3D**: lo hace `solvePnP` — conoce las 4 esquinas en coordenadas de objeto (cuadrado de lado conocido) y sus proyecciones; con `K` y distorsión resuelve `(R,t)`. Por eso **un solo marcador de tamaño conocido** da distancia y orientación absolutas.

### 4.2 Conos rojos — `red_cone_detector_node.py` (la misión de la Parte C)
1. **Segmentación HSV** (`_red_mask:257`): rojo está en el **wrap del círculo de tono** → **dos bandas** (H≈0 y H≈180) unidas por OR, con S/V mínimos.
2. **Morfología**: apertura (borra ruido) + cierre (rellena huecos).
3. **Gating geométrico** (`_find_candidates:271`): área, alto, fill ratio, aspect ratio y **triangularidad ≥ 0.38** (un cono es triangular, no un blob). Filtra distractores rojos.
4. **Distancia monocular por altura conocida** (`_estimate_relative:316`): `rng = cone_height_m · fy / h_px` (triángulos semejantes, modelo pinhole). Bearing del centro del bbox. Posición relativa → `base_link` con offset.
5. **A `map`** (`_relative_to_map:329`): rota+traslada por la pose de MCL (`/pose_estimate`). **Necesita localización previa.**
6. **Validación temporal** (`_update_history:337`): estable sólo si ≥4 de las últimas 8 detecciones caen en 0.35 m. No basta ver rojo una vez.
7. **Goal para el planner** (`_make_goal:345`): sólo si es estable, publica un `/goal_pose` con **standoff de 0.35 m** delante del cono, mirando al cono. Rate-limit 5 s.

### 4.3 Teoría
- **Modelo pinhole**: `s·[u,v,1]ᵀ = K·[R|t]·[X,Y,Z,1]ᵀ`. **Intrínsecos** = `K` (focales `fx,fy`, centro `cx,cy`) + distorsión. **Extrínsecos** = pose `[R|t]` (acá, el offset cámara→base_link). La fórmula `rng = cone_height·fy/h_px` es pinhole por triángulos semejantes.
- **Calibración**: estima `K` y coeficientes de distorsión (típicamente con tablero). `solvePnP` los necesita para des-distorsionar antes de resolver geometría.
- **ArUco robusto** porque: borde negro (contorno fácil con umbralado adaptativo, insensible a luz uniforme), codificación binaria con **distancia de Hamming** (corrige errores, casi cero falsos positivos), 4 esquinas → pose 6-DoF con un marcador, **ID = data association trivial**.
- **solvePnP** = Perspective-n-Point: dados n puntos 3D y sus proyecciones + `K`, halla `(R,t)` minimizando el error de reproyección.
- **HSV vs RGB**: HSV separa tono (Hue) de brillo (Value) → el umbral de "rojo" resiste cambios de iluminación; en RGB, la luz altera los 3 canales a la vez.
- **La cámara** es sensor **exteroceptivo, pasivo, basado en intensidad** (clase 04); no mide profundidad directa (por eso la inferimos por tamaño conocido — monocular).

### 4.4 Preguntas probables (Parte C)
1. **¿Cómo estiman la distancia al cono?** Pinhole con altura conocida: `rng = 0.30·fy/h_px`. Monocular, sin estéreo.
2. **¿Y al ArUco?** Distinto: `solvePnP` sobre las 4 esquinas de tamaño conocido → `tvec`.
3. **¿Por qué HSV y no RGB?** Separa color de brillo → robusto a iluminación; el rojo en el wrap se maneja con dos bandas.
4. **¿Intrínsecos vs extrínsecos?** Intrínsecos = `K` + distorsión (internos); extrínsecos = pose de la cámara (acá el offset de montaje).
5. **¿Cómo funciona ArUco por dentro?** (ver 4.3) umbralado → contornos → decodificación con Hamming → refinamiento → PnP.
6. **¿Por qué convertir el cono en goal y no ir directo?** Porque el control directo iría en línea recta y cruzaría paredes que la cámara ve a través de rejas; el goal deja que A\* rodee los muros. **Decisión central de la Parte C.**
7. **¿Cómo evitan falsos positivos?** 3 capas: color (2 bandas + S/V), geometría (triangularidad, área, aspect), y consistencia temporal (4/8 en 0.35 m).
8. **¿Por qué gris para ArUco y color para conos?** ArUco codifica en blanco/negro; los conos se distinguen por color.
9. **¿Qué es el standoff?** Punto 0.35 m delante del cono (no su centro), para no planificar dentro de su celda.

### 4.5 Gotchas Parte C (sim-to-real)
- **Profundidad monocular frágil**: depende de conocer `cone_height_m` y medir bien `h_px` (oclusión/recorte sesga). Calibrar en el lab.
- **Iluminación**: los rangos HSV están tuneados; luz distinta corre el rojo → reajustar.
- **Motion blur + baja resolución** (cámara "preview" ~245 px): la validación temporal ayuda pero agrega latencia.
- **Depende de MCL**: si la localización deriva, el goal cae desviado.
- **Offset plano ≈ TF aproximado** (lo correcto sería un lookup TF camera→base_link).

---

<a name="5-preguntas-transversales"></a>
## 5. Preguntas transversales y "trampa"

- **"¿SLAM y localización son lo mismo?"** No: localización asume mapa **conocido** y estima sólo la pose; SLAM estima pose **y** mapa simultáneamente. Ambos son el mismo filtro de Bayes, pero SLAM (batch) es *smoothing* (optimiza toda la historia) y MCL es *filtering* (recursivo, sólo el estado actual).
- **"¿Por qué offline el SLAM y online la navegación?"** El SLAM se hace una vez sobre un bag grabado (batch, mapa reutilizable); la navegación tiene que reaccionar en tiempo real. Herramienta distinta para cada régimen.
- **"¿Dónde usan gaussianas y dónde no?"** Gaussianas (implícitas) en el ruido del back-end de Graph SLAM y en el modelo de sensor de MCL; pero la **creencia de MCL es no-gaussiana** (partículas). No usamos ningún filtro de Kalman.
- **"¿Qué es la matriz de información y por qué aparece en SLAM y en Kalman?"** Es la inversa de la covarianza; mide **cuánta información** (certeza) tenés. En Graph SLAM pesa las restricciones; en el filtro de información (dual del Kalman) es la representación natural del estado.
- **"¿Por qué A\* y no RRT / Dijkstra / MDP?"** Dijkstra es A\* con h=0 (explora más). RRT es para espacios de alta dimensión / no-holonómicos complejos (overkill en una grilla 2D). MDP/value iteration modela incertidumbre en las transiciones (no la necesitamos con un mapa estático). A\* da el óptimo con heurística admisible y es simple.
- **"¿Por qué Pure Pursuit y no Stanley o un PID?"** Pure Pursuit es geométrico, simple, estable a baja velocidad y natural para diferencial; Stanley se usa más en Ackermann/autos. Igual agregamos una fase P para la orientación final.
- **"¿Y si te sacan un supuesto?"** (mundo estático, celdas independientes, rayos independientes, ruido gaussiano) → conocé cada supuesto, por qué es falso en la realidad, y qué parche lo mitiga (z_rand, voto de obstáculos, gating, Cauchy).

---

<a name="6-sim-to-real"></a>
## 6. Sim-to-real: qué falló y cómo defenderlo

La consigna (1.5) **no exige perfección**, sí documentar la brecha. Puntos para tener claros:

- **La demo de Parte C no se completó en vivo, y la causa NO fue percepción ni control.** Fue un **doble inflado del entorno**: el mapa estático inflado (~0.15 m) + el costmap (~0.41 m) ≈ 0.56 m por lado, que dejó pasillos de 0.5–0.7 m intransitables. Agravado por drift de SLAM y posible mala calibración del `laser_yaw_rad`. Es una **falla de parametrización geométrica del mapa**, recuperable **sin tocar percepción ni control**. El mapa se regeneró offline desde el mismo rosbag con los parámetros correctos.
  - Sabé distinguir: la detección de conos estaba integrada y validada; lo que falló fue el margen del mapa/costmap.
- **Brechas típicas a mencionar**: patinaje de ruedas (por eso se duplican α1..4), iluminación/blur en la cámara (por eso HSV + validación temporal), latencia/pérdida de frames, LiDAR rotado 90° en el robot real, calibración imperfecta.

**Posibles inconsistencias que el jurado podría notar (tenelas claras):**
- La presentación menciona **46 landmarks** (rosbag laberinto) pero el mapa regenerado del labo real tiene **33**. Son **datasets distintos** (laberinto sim vs. labo real), no un error.
- El offset de cámara difiere entre nodos: **0.06 m** (ArUco) vs **0.05 m** (conos). Son valores aproximados/tuneados por separado, no un TF calibrado.
- La teoría de **pinhole/PnP** no está desarrollada en las slides de la cátedra (sólo triangulación óptica/estéreo en clase 04). Preséntala como **conocimiento propio**, no como "visto en clase".

---

### Frase de cierre para el oral
> "Todo el TP es estimación probabilística de estado: SLAM (Parte A) como optimización batch de un grafo con ArUco + ICP, localización (Parte B) como filtro de partículas con likelihood field, y percepción (Parte C) que produce objetivos para el planner, no comandos. Elegimos Graph SLAM por sus loop closures naturales y estructura dispersa, MCL por su capacidad multimodal frente al EKF unimodal, A\* por su optimalidad con heurística admisible, y Pure Pursuit por su simplicidad geométrica. Donde algo falló en el robot real, fue parametrización del entorno, no de los algoritmos, y lo documentamos."
