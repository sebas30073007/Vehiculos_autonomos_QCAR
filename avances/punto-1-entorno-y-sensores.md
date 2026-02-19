---
title: Punto 1 — Exploración del entorno y sensores (Virtual + QCar real)
parent: Avances
nav_order: 2
---

# Punto 1 — Exploración del entorno y sensores (Virtual + QCar real)

## 1) Resumen ejecutivo

En este Punto 1 se realizó una **instrumentación inicial** del ecosistema QCar en dos frentes: simulación (MATLAB/Simulink + entorno virtual) y plataforma física (QCar/QCar2 con scripts de adquisición). El objetivo fue confirmar que la cadena de datos de sensores funciona de extremo a extremo y que puede dejar trazabilidad útil para fases posteriores. Se cubrieron cuatro familias de sensores: **RGB-D/profundidad, LiDAR, IMU y encoders**. También se dejaron lineamientos de guardado para construir datasets comparables entre corridas. Como resultado tangible, este avance entrega estructura de logs, recomendaciones de visualización base y criterios mínimos de validación para que el equipo replique pruebas sin depender de memoria informal.

## 2) Relación con la competencia

El contexto de la CPS-IoT 2026 exige resolver conducción tipo **taxi autónomo en Quanser City** y, para la entrega oficial, mantener un stack de **MATLAB/Simulink** en el submission. Por eso este Punto 1 no busca “ganar” la tarea final, sino establecer una base técnica sólida: verificar sensores, formato de datos, sincronización temporal, coherencia de unidades y *sanity checks*.

> **Nota importante de stack:** en este punto se permitió usar Python para exploración de hardware real y adquisición rápida; sin embargo, el pipeline que llegue a competición se migrará/portará a MATLAB/Simulink para cumplir el marco de entrega.

## 3) Alcance y entregables

### Alcance

- Explorar lectura y visualización básica de sensores en entorno virtual.
- Explorar adquisición en QCar real con logging reproducible.
- Definir estructura mínima de datasets y convenciones de nombres.
- Identificar problemas típicos de integración y mitigaciones.

### Entregables del Punto 1

- Scripts/cuadernos de adquisición y visualización (según disponibilidad del repo/equipo).
- Logs/datasets de prueba en formato tabular y/o binario.
- Capturas de entorno virtual y setup físico.
- Plots básicos de IMU y LiDAR para validación rápida.

### Definition of Done (DoD)

Se considera completado el Punto 1 cuando:

- Se demuestra captura de al menos una corrida en virtual y una en físico.
- Existen timestamps y metadatos suficientes para reconstruir la secuencia.
- Hay evidencia visual (capturas/plots) por sensor crítico.
- Se documentan incidencias y solución aplicada o pendiente.

## 4) Setup (Virtual vs Real)

### 4.1 Virtual (QLabs/Simulink/MATLAB)

**Checklist de prerequisitos**

- MATLAB y Simulink instalados (versión del curso/lab).
- Acceso al entorno virtual de QLabs/Quanser City.
- Paquetes/toolboxes requeridos por los ejemplos del curso.
- Permisos de escritura en carpeta de trabajo para logs (`.mat`, `.csv`).

**Flujo sugerido (alto nivel)**

1. Abrir escena base de Quanser City.
2. Ubicar/instanciar el QCar en una posición inicial controlada.
3. Ejecutar simulación corta de validación (run de pocos segundos).
4. Verificar que llegan señales de sensores al bloque/script de logging.
5. Guardar evidencia inicial (captura de escena + primer plot).

### 4.2 Real (QCar/QCar2)

**Checklist de prerequisitos**

- Energía y estado físico del vehículo verificados.
- Conectividad local (red/cable según laboratorio) operativa.
- Zona segura de prueba, sin obstáculos no controlados.
- Usuario con permisos para crear carpetas y escribir logs.

**Notas de entorno Python**

- Definir un entorno virtual dedicado (`venv` o conda).
- Instalar dependencias de captura y visualización que realmente use el proyecto.
- Si aparece `ModuleNotFoundError: pyrealsense2`, evaluar:
  - instalación explícita del paquete compatible con SO/arquitectura, o
  - uso de wrappers provistos por Quanser/PAL en el entorno del curso.

## 5) Sensores y datos

| Sensor | Qué mide | Unidad típica | Qué se guardó | Uso inmediato (plot/preview) | Riesgos |
|---|---|---|---|---|---|
| RGB-D / Profundidad | Imagen RGB + distancia por píxel | color (8-bit), profundidad en m o mm | Frames RGB, mapa depth, timestamp | Preview en vivo, validación de rango y exposición | Escala depth inconsistente, desalineación RGB/depth, saturación |
| LiDAR | Distancia angular del entorno | m (rango), rad/deg (ángulo) | Scan crudo por barrido + timestamp | Polar plot / mapa 2D rápido | Valores `0/inf`, reflexión en superficies, ruido |
| IMU | Aceleración y velocidad angular (orientación derivada) | m/s², rad/s (o deg/s) | Series temporales + timestamp | Plot `ax/ay/az`, `gx/gy/gz`, tendencia de orientación | Drift, ejes mal interpretados, offset |
| Encoders | Rotación de ruedas para velocidad/desplazamiento | ticks, rad, m/s, m | ticks/contador y tiempo | Velocidad estimada y distancia acumulada | Conversión con factor incorrecto, signo invertido |

> `TODO(poner rate real medido de cada sensor y versión de firmware/controlador)`

## 6) Flujo de trabajo — Virtual (MATLAB)

### 6.1 Objetivo

Validar que el pipeline de simulación produce señales coherentes por sensor y que el logging permite análisis offline sin pérdida de contexto temporal.

### 6.2 Procedimiento (paso a paso)

1. Iniciar MATLAB en una carpeta de trabajo del proyecto.
2. Abrir modelo/script de simulación del entorno virtual.
3. Configurar ruta de salida de datos (carpeta por corrida).
4. Correr escenario corto con maniobra simple (avance + giro leve).
5. Visualizar señales en tiempo real o al finalizar la corrida.
6. Exportar resultados a `.mat` y resumen tabular a `.csv`.
7. Guardar captura de pantalla de escena y plots clave.

### 6.3 Resultados esperados

- Señales continuas sin cortes abruptos por timeout.
- Curvas IMU con magnitudes plausibles según movimiento inducido.
- LiDAR con estructura espacial consistente con obstáculos visibles.
- Archivos de salida legibles y ordenados por timestamp.

### 6.4 Plots realizados

- **IMU:** series `ax/ay/az` y `gx/gy/gz`; si aplica, estimación de `yaw/pitch/roll`.
- **LiDAR:** representación polar o proyección XY 2D.
- **Control de calidad:** gráfico simple de intervalo entre muestras (`delta_t`).

### 6.5 Guardado de datos

- Usar `.mat` para conservar estructuras y arrays nativos de MATLAB.
- Exportar `.csv` para interoperabilidad (revisión rápida en Python/Excel/R).
- Mantener archivo de metadatos (`README` o `.txt`) por corrida con:
  - fecha/hora,
  - escenario,
  - operador,
  - observaciones.

### 6.6 Placeholders de imágenes (INSERTAR CAPTURA AQUÍ)

![QLabs – vista general de Quanser City](../assets/images/punto-1/qlabs-quanser-city.png)

*(INSERTAR CAPTURA AQUÍ: vista general del entorno virtual con el QCar en escena).* 

![MATLAB – plot IMU](../assets/images/punto-1/matlab-imu-plot.png)

*(INSERTAR CAPTURA AQUÍ: gráfico temporal de aceleración y/o giro).* 

![MATLAB – plot LiDAR](../assets/images/punto-1/matlab-lidar-plot.png)

*(INSERTAR CAPTURA AQUÍ: visualización polar o XY del barrido LiDAR).* 

## 7) Flujo de trabajo — Real (Python)

### 7.1 Objetivo

Confirmar adquisición de sensores sobre hardware real y producir logs limpios para depuración y comparación contra simulación.

### 7.2 Procedimiento (paso a paso)

**A) RGB-D**

1. Levantar stream RGB y depth por separado.
2. Mostrar preview dual (ventana RGB + ventana depth).
3. Aplicar colormap al depth para inspección rápida de rango.
4. Verificar que timestamp y frame index se registran en cada captura.

**B) LiDAR**

1. Capturar scan crudo por iteración.
2. Filtrar inválidos (`0`, `inf`, `nan`) antes de graficar.
3. Generar vista 2D instantánea (polar o XY).
4. Guardar scan junto con timestamp común al resto de sensores.

**C) IMU**

1. Leer aceleración y giro en la misma base temporal del logger.
2. Ejecutar *sanity checks*:
   - en reposo, magnitud de aceleración cercana a gravedad,
   - deriva razonable en giroscopio (sin saltos extremos),
   - continuidad de muestra a muestra.

**D) Encoders**

1. Leer ticks/contador de rueda por muestra.
2. Calcular variación de ticks y `delta_t`.
3. Convertir a velocidad/distancia con fórmula genérica:

```text
omega_rueda = (delta_ticks / ticks_por_vuelta) * 2*pi / delta_t
v_lineal = omega_rueda * radio_rueda
distancia_acumulada += v_lineal * delta_t
```

### 7.3 Exportación / logging

**Estructura sugerida**

```text
/data/punto-1/YYYY-MM-DD_run01/
  ├── rgb/
  ├── depth/
  ├── lidar/
  ├── imu/
  └── encoders/
```

**Convención de nombres**

- Imágenes: `rgb_000001_<timestamp>.png`, `depth_000001_<timestamp>.png`
- LiDAR: `lidar_000001_<timestamp>.csv`
- Series: `imu_<timestamp_inicio>.csv`, `encoders_<timestamp_inicio>.csv`

**Timestamp recomendado**

- Guardar `epoch_ms` para sincronización computable.
- Opcional: agregar columna ISO-8601 para lectura humana.

### 7.4 Snippets de código (plantillas seguras)

> Si en el repo se agregan APIs concretas de hardware, reemplazar estas plantillas por snippets reales y cortos.

**Pseudocódigo de adquisición multi-sensor**

```text
inicializar_sensores()
inicializar_logger()

while corrida_activa:
    ts = timestamp_epoch_ms()

    frame_rgb, frame_depth = leer_rgbd()
    scan_lidar = leer_lidar()
    imu = leer_imu()
    enc = leer_encoders()

    scan_lidar = filtrar_invalidos(scan_lidar)
    guardar_muestras(ts, frame_rgb, frame_depth, scan_lidar, imu, enc)

cerrar_sensores()
finalizar_logger()
```

**Ejemplo mínimo de logging CSV con timestamps (genérico)**

```python
from pathlib import Path
import csv
import time

out = Path("data/punto-1/2026-01-01_run01/imu")
out.mkdir(parents=True, exist_ok=True)

csv_path = out / "imu_2026-01-01T10-00-00.csv"
with csv_path.open("w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(["epoch_ms", "ax", "ay", "az", "gx", "gy", "gz"])

    for _ in range(10):
        ts = int(time.time() * 1000)
        ax, ay, az = 0.0, 0.0, 9.81  # TODO(reemplazar por lectura real)
        gx, gy, gz = 0.0, 0.0, 0.0   # TODO(reemplazar por lectura real)
        writer.writerow([ts, ax, ay, az, gx, gy, gz])
```

### 7.5 Placeholders de imágenes (INSERTAR CAPTURA AQUÍ)

![QCar real – setup](../assets/images/punto-1/qcar-setup.jpg)

*(INSERTAR CAPTURA AQUÍ: foto del vehículo real y zona de pruebas).* 

![Python – RGB](../assets/images/punto-1/python-rgb.png)

*(INSERTAR CAPTURA AQUÍ: preview de stream RGB).* 

![Python – Depth](../assets/images/punto-1/python-depth.png)

*(INSERTAR CAPTURA AQUÍ: preview depth con colormap).* 

![Python – LiDAR map](../assets/images/punto-1/python-lidar-map.png)

*(INSERTAR CAPTURA AQUÍ: mapa/scan LiDAR en 2D).* 

## 8) Problemas típicos y fixes

### 8.1 `Permission denied` al escribir archivos

- Confirmar que la carpeta destino existe y es escribible.
- Probar una ruta en el directorio de usuario (por ejemplo `~/data/...`).
- Revisar permisos (`chmod`/propietario) y políticas de laboratorio.
- Evitar escribir en rutas de sistema protegidas.

### 8.2 `ModuleNotFoundError: pyrealsense2`

- Verificar versión de Python y compatibilidad del paquete.
- Probar instalación en entorno limpio.
- Si el laboratorio usa wrappers Quanser/PAL, preferir ese camino para consistencia del curso.

### 8.3 Timeouts / latencia alta

- Reducir resolución de cámara o FPS de adquisición.
- Evitar procesamiento pesado dentro del loop de captura.
- Separar adquisición y escritura en procesos/hilos (buffer asíncrono).
- Monitorear CPU/memoria para detectar cuello de botella.

### 8.4 Unidades mezcladas (m vs mm, rad vs deg)

- Definir unidad canónica por variable desde el inicio.
- Convertir en un solo punto del pipeline (no en múltiples lugares).
- Añadir validadores simples en carga de datos.

### 8.5 LiDAR con valores cero/infinito

- Filtrar `0`, `inf`, `nan` antes de usar el scan.
- Recortar rango útil para visualización inicial.
- Registrar porcentaje de puntos inválidos como métrica de salud del sensor.

## 9) Checklist de evidencia (para evaluación)

- [ ] Capturas virtuales (escena + plots MATLAB).
- [ ] Fotos/capturas del setup real (RGB, depth, LiDAR).
- [ ] Logs organizados por corrida y sensor.
- [ ] Metadatos de corrida (fecha, responsable, observaciones).
- [ ] Hash de commit que respalda la evidencia.

## 10) Próximos pasos (puente al Punto 2)

- Calibración y verificación de *frames* (extrínsecos/intrínsecos).
- Estrategia de sincronización multi-sensor más robusta.
- Preprocesamiento para percepción básica (detección/segmentación inicial).
- Integración gradual al stack objetivo de MATLAB/Simulink.
- Definición de métricas comparables entre virtual y real.
