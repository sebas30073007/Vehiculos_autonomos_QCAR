---
title: "Físico"
parent: "OGM + SLAM"
grand_parent: Avances
nav_order: 2
---

# Punto 2 — OGM + SLAM (QCar real)

## 1) Resumen ejecutivo

En este Punto 2 físico se implementó un pipeline completo de **mapeo simultáneo con localización** sobre el QCar real. El robot avanza de forma autónoma mientras construye un **Occupancy Grid Map (OGM) global** en tiempo real usando el RPLidar. La pose se estima con odometría simple (integración de velocidad) y se refina frame a frame mediante **ICP traslacional** entre escaneos consecutivos. El mapa se actualiza con **log-odds y ray casting (Bresenham)** y se exporta como video `.mp4` con timestamp. El pipeline también cuenta con una variante (`qcar_slam_L.py`) que admite **un giro de 90°** y usa una resolución más fina.

Scripts de referencia:
- `qcar_slam.py` — versión principal: avance recto + ICP traslacional
- `qcar_slam_L.py` — variante: giro único + cinemática diferencial, sin ICP

---

## 2) Contexto y objetivo

### ¿Qué es OGM + SLAM en este contexto?

| Concepto | Definición práctica |
|----------|---------------------|
| **OGM** (Occupancy Grid Map) | Mapa 2D dividido en celdas; cada celda almacena la probabilidad de estar ocupada. Se representa como log-odds para actualizaciones eficientes. |
| **SLAM** (Simultaneous Localization And Mapping) | El robot estima dónde está (localización) mientras construye el mapa, sin GPS ni posición externa. En este punto usamos una variante ligera: odometría + ICP traslacional. |
| **Log-odds** | Representación logarítmica de la probabilidad: `l = log(p / (1-p))`. Permite sumar evidencias de forma numericamente estable. |
| **Ray casting (Bresenham)** | Para cada rayo LiDAR se trazan todas las celdas entre el robot y el punto de impacto. Las celdas intermedias se marcan como libres, la celda final como ocupada. |
| **ICP traslacional** | Corrección de la pose usando la similitud entre dos nubes de puntos consecutivas. Solo corrige traslación (dx, dy), no rotación. |

El objetivo del Punto 2 **no es la conducción autónoma**, sino producir un mapa consistente del entorno que pueda usarse en el Punto 3 (navegación).

---

## 3) Alcance y entregables

### Alcance

- Validar el pipeline LiDAR → OGM global sobre el QCar real.
- Confirmar que el mapa resultante es geométricamente coherente con el entorno físico.
- Exportar evidencia auditable: video del mapa construyéndose en tiempo real.
- Documentar parámetros calibrados y criterios de fallo.

### Entregables

- Script ejecutable `qcar_slam.py` (y variante `qcar_slam_L.py` con giro).
- Video `.mp4` del OGM generado durante un run (`qcar_ogm_logodds_YYYYMMDD_HHMMSS.mp4`).
- Capturas de pantalla del mapa en distintos momentos del recorrido.
- Esta bitácora con procedimiento, parámetros y debug documentados.

### Definition of Done (DoD)

Se considera completado el Punto 2 físico cuando:

- El script corre sin errores en el QCar real desde inicio hasta paro limpio.
- El OGM muestra paredes y obstáculos reconocibles del entorno real.
- Se guarda al menos un video de un run completo con timestamp.
- Se documenta un set de parámetros validados (throttle, trim, resolución).
- Se registran las incidencias encontradas y sus soluciones.

---

## 4) Setup físico — Prerrequisitos

### Checklist antes de correr

- [ ] Baterías del QCar cargadas y verificadas.
- [ ] Conexión SSH activa a la Jetson del QCar (o sesión local en el hardware).
- [ ] Entorno Python con dependencias instaladas (ver sección 4.1).
- [ ] Espacio libre en disco para el video de salida (≥ 500 MB recomendado).
- [ ] Zona de prueba despejada: pasillo o habitación sin obstáculos móviles no controlados.
- [ ] RPLidar encendido y reconocido por el sistema (`/dev/ttyUSB0` o equivalente).
- [ ] Scripts copiados en la Jetson (`/home/nvidia/Documents/` o ruta equivalente).

### 4.1 Dependencias Python

```bash
# Librerías Quanser PAL (preinstaladas en la Jetson del QCar)
# pal.products.qcar, pal.utilities.lidar

# Librerías adicionales
pip install numpy opencv-python
```

> **Nota:** `pal.products.qcar` y `pal.utilities.lidar` son librerías propietarias de Quanser que solo corren en el hardware real del QCar (Jetson Orin). No son simulables directamente en QLabs sin adaptadores.

---

## 5) Arquitectura del pipeline

El pipeline principal (`qcar_slam.py`) opera en un bucle de control a **50 Hz**:

```
Inicialización
├── QCar(readMode=0)          ← API de bajo nivel (throttle, steering, LEDs)
└── Lidar(RPLidar, 360 meas)  ← RPLidar en modo 2, sin interpolación

Bucle principal (50 Hz)
├── A) Movimiento
│   └── read_write_std(throttle, steering, LEDs)  ← comando al QCar
├── B) Estimación de pose (predicción por odometría)
│   └── x_pred = V_EST * elapsed  (avance recto, theta = 0)
├── C) Lectura y filtrado LiDAR
│   ├── lidar.read()
│   ├── Conversión a body frame: a_bf = -a_lidar + π/2
│   └── Filtro: d > 0.03m, finito, d < MAX_LIDAR_R
├── D) Proyección al marco global (pose predicha)
│   ├── xr = d * cos(a_bf),  yr = -d * sin(a_bf)  (body frame)
│   └── xw = x_pred + xr*cos(θ) - yr*sin(θ)       (global)
├── E) Corrección ICP traslacional
│   ├── Comparar scan actual con scan anterior (ambos en global)
│   ├── Submuestra a 200 puntos (brute-force nearest neighbor)
│   └── dx_icp, dy_icp = centroide_prev - centroide_curr
├── F) Pose corregida
│   └── x = x_pred + dx_icp,  y = y_pred + dy_icp
├── G) Actualización OGM (log-odds + Bresenham)
│   ├── Para cada punto de impacto en el mapa global:
│   │   ├── Trazar rayo con bresenham_line(robot → hit)
│   │   ├── Celdas intermedias: log_odds += L_FREE (-0.5)
│   │   └── Celda final: log_odds += L_OCC (+1.2)
│   └── Saturar: clip(log_odds, L_MIN, L_MAX)
├── H) Visualización + export
│   ├── p_occ = sigmoid(log_odds)  → imagen uint8 (negro=ocupado, blanco=libre)
│   ├── draw_robot_triangle(mapa, x, y, θ)  ← triángulo rojo
│   ├── cv2.imshow(...)
│   └── video_out.write(frame)
└── Condiciones de paro
    ├── traveled_est ≥ DIST_TARGET_MAX (20 m)
    ├── front_d ≤ STOP_DISTANCE (0.5 m)
    └── KeyboardInterrupt (Ctrl+C)
```

### 5.1 Variante con giro (`qcar_slam_L.py`)

Esta variante reemplaza el ICP por cinemática diferencial completa y añade un **giro único de 90° a la derecha** cuando el obstáculo frontal está a ≤ 2.0 m:

```
Pose: x += v*dt*cos(θ),  y += v*dt*sin(θ),  θ += ω*dt
Giro: THROTTLE_TURN=0.052, STEERING_TURN=-0.43, ω≈30°/s
Resolución más fina: RES=0.05 m/celda (vs 0.15 en la versión base)
```

---

## 6) Parámetros clave

### 6.1 `qcar_slam.py` (versión principal)

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `SAMPLE_RATE` | 50 Hz | Frecuencia del bucle de control |
| `DT` | 0.02 s | Paso de tiempo |
| `THROTTLE_CMD` | 0.04 (PWM) | Velocidad de avance (sintonizada) |
| `STEERING_TRIM` | -0.054 | Corrección de desviación al avanzar recto |
| `V_EST` | ~0.190 m/s | Velocidad estimada (`SPEED_PER_THROTTLE × THROTTLE_CMD`) |
| `STOP_DISTANCE` | 0.5 m | Umbral de paro por obstáculo frontal |
| `DIST_TARGET_MAX` | 20.0 m | Distancia máxima del recorrido |
| `MAX_LIDAR_R` | 6.0 m | Rango máximo de lecturas LiDAR a mapear |
| `NUM_MEAS` | 360 | Número de mediciones por scan del RPLidar |
| `FRONT_DEG` | ±15° | Cono frontal para detección de obstáculos |
| `RES` | 0.15 m/celda | Resolución del mapa OGM |
| `MAP_LEN_M` | 22.0 m | Largo del mapa (eje x) |
| `MAP_WID_M` | 6.0 m | Ancho del mapa (eje y) |
| `L_OCC` | +1.2 | Incremento log-odds por celda ocupada |
| `L_FREE` | -0.5 | Incremento log-odds por celda libre |
| `L_MIN / L_MAX` | -5.0 / +5.0 | Saturación del mapa |

### 6.2 Variante `qcar_slam_L.py`

| Parámetro | Valor | Diferencia vs versión base |
|-----------|-------|---------------------------|
| `RES` | 0.05 m/celda | Resolución 3× más fina |
| `MAP_LEN_M` | 34.0 m | Mapa más largo |
| `MAP_WID_M` | 12.0 m | Mapa más ancho |
| `THROTTLE_TURN` | 0.052 | Throttle durante el giro |
| `STEERING_TURN` | -0.43 | Dirección del giro (derecha) |
| `TURN_RATE_EST` | 30°/s | Velocidad angular estimada del giro |
| ICP | No | Usa cinemática diferencial en su lugar |

---

## 7) Procedimiento paso a paso

### 7.1 Acceder al QCar

```bash
# Desde tu PC, conectarse por SSH a la Jetson
ssh nvidia@<IP_del_QCar>
# Contraseña por defecto: nvidia

# Navegar al directorio de scripts
cd /home/nvidia/Documents/scripts/punto2/
```

### 7.2 Verificar el LiDAR

```bash
# Comprobar que el RPLidar es reconocido
ls /dev/ttyUSB*
# Debe aparecer /dev/ttyUSB0 o similar

# Dar permisos si es necesario
sudo chmod 666 /dev/ttyUSB0
```

### 7.3 Ejecutar el script principal

```bash
# Versión base (avance recto + ICP)
python3 qcar_slam.py

# Variante con giro (recomendada para espacios con obstáculos frontales)
python3 qcar_slam_L.py
```

Al iniciar verás en la terminal:

```
QCar SLAM (log-odds + ICP traslacional): avanzando y mapeando con LiDAR...
THROTTLE_CMD = 0.040, V_EST ≈ 0.190 m/s
t=1.23s | x= 0.23m | dist_est= 0.23m | front_d= 3.45m
```

### 7.4 ¿Qué ves en pantalla?

Se abre una ventana OpenCV llamada **"OGM global log-odds (x horizontal, y vertical)"**:
- **Negro** = celda ocupada (pared / obstáculo).
- **Blanco** = celda libre (espacio navegable).
- **Gris** = incertidumbre (aún no observada o con evidencia débil).
- **Triángulo rojo** = posición y orientación actual del robot.

El mapa se va llenando de izquierda a derecha conforme el robot avanza.

### 7.5 Parar de forma segura

```bash
# Presionar Ctrl+C en la terminal
# El script detiene el QCar, libera el LiDAR y guarda el video
```

Salida esperada al terminar:

```
Interrumpido por el usuario.
Video guardado en: qcar_ogm_logodds_20260305_143201.mp4
Fin. Motivo de paro: KeyboardInterrupt
Tiempo total: 45.23s | dist_est final: 8.61m
```

### 7.6 Recuperar el video

```bash
# Desde tu PC
scp nvidia@<IP_del_QCar>:/home/nvidia/Documents/scripts/punto2/qcar_ogm_logodds_*.mp4 ./
```

---

## 8) Evidencia y resultados esperados

### Placeholders de imágenes

Guardar las capturas en `assets/images/` con los nombres sugeridos:

| Nombre de archivo | Qué debe mostrar |
|-------------------|-----------------|
| `p2_ogm_inicio.png` | Mapa al inicio del run (casi vacío, solo primeras celdas) |
| `p2_ogm_medio.png` | Mapa a mitad del recorrido (paredes laterales visibles) |
| `p2_ogm_final.png` | Mapa al finalizar (recorrido completo mapeado) |
| `p2_ogm_robot.png` | Triángulo rojo del robot bien posicionado en el mapa |
| `p2_terminal.png` | Salida de terminal con parámetros y mensaje de video guardado |

*[Placeholder — agregar imagen del mapa cuando se complete el run físico]*

```markdown
![OGM - estado final del mapa]({{ "/assets/images/p2_ogm_final.png" | relative_url }})
*Mapa OGM generado al finalizar un recorrido de ~X metros en el entorno físico.*
```

### Resultados esperados

- Se espera ver **paredes y obstáculos del entorno como zonas negras** bien delimitadas, con pasillos visibles como zonas blancas.
- Se espera que el **triángulo del robot** avance de izquierda a derecha en el mapa, siguiendo la trayectoria real.
- Se espera que el **mapa converja** (deje de cambiar drásticamente) tras ~10–15 segundos de recorrido por zonas ya vistas.
- El video `.mp4` debe mostrar el mapa actualizándose suavemente a ~50 FPS.

---

## 9) Checklist de validación

### Por corrida

- [ ] El script inició sin errores de importación (`pal`, `cv2`, `numpy`).
- [ ] La ventana OpenCV se abrió y muestra el mapa actualizándose.
- [ ] El triángulo rojo del robot se mueve en la dirección correcta.
- [ ] El robot paró correctamente (obstáculo o Ctrl+C, no crash).
- [ ] El video `.mp4` fue guardado con nombre correcto.
- [ ] Las paredes del entorno son visibles como zonas negras en el mapa.

### Por documentación

- [ ] Parámetros del run registrados (¿se cambió algo del default?).
- [ ] Incidencias encontradas documentadas en la sección de debug.
- [ ] Al menos una captura de pantalla del mapa guardada en `assets/images/`.

---

## 10) Problemas comunes y debug

### El mapa aparece rotado o con los ejes invertidos

**Causa:** La conversión de body frame puede tener signo incorrecto según el montaje físico del LiDAR.

**Solución:**
```python
# En to_body_frame(), probar variantes:
return -angles_rad + (np.pi / 2.0)   # default (convención RPLidar estándar)
return  angles_rad - (np.pi / 2.0)   # si el mapa aparece espejado en y
return -angles_rad - (np.pi / 2.0)   # si rotado 180°
```

### Las paredes se ven muy gruesas o el mapa tiene ruido excesivo

**Causa:** Lecturas LiDAR ruidosas, resolución muy fina, o `L_OCC`/`L_FREE` mal balanceados.

**Soluciones:**
- Aumentar `RES` de 0.05 a 0.10–0.15 m (más grueso, menos ruido).
- Reducir `L_OCC` o aumentar `L_MIN` para que el mapa sea menos "pegajoso".
- Agregar filtro de mediana en el scan:
```python
# Antes de usar d_use en el OGM:
d_use = np.sort(d_use)
# o filtro de mediana por sector angular
```

### El robot acumula drift y el mapa se desdobla

**Causa:** El ICP traslacional no corrige rotación. Si el robot gira (por superficie desigual o trim mal calibrado), la pose estimada diverge.

**Diagnóstico:** Observar si el triángulo rojo "sale" del área donde están las paredes mapeadas.

**Soluciones:**
- Ajustar `STEERING_TRIM` para que el robot avance más recto.
- Usar la variante `qcar_slam_L.py` que integra theta explícitamente.
- Para recorridos con giros, el ICP traslacional no es suficiente: es una limitación conocida del pipeline actual.

### El robot sale del área del mapa

**Causa:** El mapa (`MAP_LEN_M`, `MAP_WID_M`) es más pequeño que el recorrido real, o la pose se desvió mucho.

**Diagnóstico:** Mensaje en terminal: `"robot fuera del área del mapa"`.

**Solución:** Ampliar `MAP_LEN_M` y `MAP_WID_M`, o reiniciar con mejor calibración de dirección.

### El LiDAR no entrega datos (scan vacío)

**Causa:** Permisos de puerto, LiDAR no encendido, cable USB suelto.

**Diagnóstico:** En terminal: `front_d=inf` sin cambios.

**Solución:**
```bash
# Verificar dispositivo
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
# Reconectar el LiDAR y reiniciar el script
```

### El video no se guarda correctamente

**Causa:** El codec `mp4v` puede no estar disponible en todas las instalaciones de OpenCV.

**Solución:** Cambiar el codec:
```python
# Alternativas:
fourcc = cv2.VideoWriter_fourcc(*'XVID')   # .avi más compatible
fourcc = cv2.VideoWriter_fourcc(*'H264')   # si está disponible
```

---

## 11) Siguiente paso

Una vez validado el OGM físico, el siguiente avance es usar el mapa generado como base para la **planeación de trayectorias** en el [Punto 3 — Navegación](./punto-3-navegacion).

---

## 12) Anexos

### 12.1 Función de conversión body frame

```python
def to_body_frame(angles_rad):
    # Convención RPLidar → cuerpo del QCar:
    # 0° del LiDAR = frente del robot después de esta transformación
    return -angles_rad + (np.pi / 2.0)
```

### 12.2 Algoritmo Bresenham (ray casting)

```python
def bresenham_line(x0, y0, x1, y1):
    """
    Devuelve todas las celdas de la línea discreta entre (x0,y0) y (x1,y1).
    Se usa para marcar celdas libres (rayo) y ocupada (impacto).
    """
    # [ver implementación completa en qcar_slam.py]
```

### 12.3 ICP traslacional (paso único)

```python
def icp_translation_step(prev_pts, curr_pts, max_pairs=200):
    """
    Corrección de pose: traslación únicamente.
    No corrige rotación → válido solo para trayectorias aproximadamente rectas.
    dx, dy = centroide(prev) - centroide(curr_predicho)
    """
    # [ver implementación completa en qcar_slam.py]
```
