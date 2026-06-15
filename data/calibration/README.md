# Calibración de cámara y ArUco

Fuente: `Matrices, coeficientes y estimaciones.docx` descargado de la cátedra.

## Marcador ArUco

```python
MARKER_SIZE_M = 0.0889
```

## TurtleBot 0

El RosBag del TP fue grabado con el TurtleBot 4 número 0, así que esta debería ser la calibración principal.

```python
CAMERA_MATRIX = np.array([
    [203.14, 0.0,   122.57],
    [0.0,   361.13, 123.33],
    [0.0,   0.0,    1.0]
], dtype=np.float64)

DIST_COEFFS = np.array([
    -0.9904393553733826,     # k1
    -47.16939926147461,      # k2
    -0.0007601691759191453,  # p1
    -0.00031758102704770863, # p2
    306.0343933105469        # k3
], dtype=np.float64)
```

Coeficientes opcionales indicados en el documento, por si se quiere probar el modelo racional:

```python
# k4 = -1.1441469192504883
# k5 = -45.59364700317383
# k6 = 299.4920654296875
```

## TurtleBot 1

```python
CAMERA_MATRIX = np.array([
    [201.37, 0.0,   123.78],
    [0.0,   357.31, 131.25],
    [0.0,   0.0,    1.0]
], dtype=np.float64)

DIST_COEFFS = np.array([
    7.823812007904053,       # k1
    -116.5168228149414,      # k2
    0.0008780899806879461,   # p1
    0.000634733063634485,    # p2
    378.764404296875         # k3
], dtype=np.float64)
```

Coeficientes opcionales:

```python
# k4 = 7.6177897453308105
# k5 = -114.77053833007812
# k6 = 373.6795654296875
```

## Estimaciones del bag `aruco_estimation`

| Distancia real aprox. | Distancia estimada aprox. |
| ---: | ---: |
| 0.30 m | 0.28 m |
| 0.695 m | 0.72 m |
| 1.01 m | 1.04 m |
| 1.49 m | 1.52 m |

Estas mediciones sirven como primer sanity check del detector ArUco y de la calibración.
