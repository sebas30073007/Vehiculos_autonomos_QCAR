---
title: Punto 1 — Exploración del entorno y sensores (Virtual)
parent: Avances
nav_order: 
---

# Punto 1 — Exploración del entorno y sensores (Virtual)

## 1) Resumen ejecutivo

En este Punto 1 se realizó la **instrumentación y validación integral** del stack sensorial del **QCar2 virtual** dentro del ecosistema **MATLAB/Simulink + QUARC (Monitor & Tune / External Mode) + QLabs**. El objetivo fue confirmar que la cadena de datos de sensores funciona de extremo a extremo y que la información es coherente (unidades, magnitudes y estabilidad temporal) para soportar fases posteriores de control y navegación autónoma.

Se validaron los siguientes componentes:

- **LiDAR** (distancias, ángulos y bandera de nueva lectura).
- **IMU/gyro y cinemática** (velocidad y giro).
- **Estimación de pose** (estado `[x, y, heading]`).
- **Telemetría eléctrica** (voltaje, corriente, nivel y potencia).
- Se intentó integrar **cámara Depth/RGB** (Video3D), documentando la limitación observada en External Mode.

Como resultado tangible, este avance entrega: (i) una secuencia reproducible de arranque del entorno, (ii) instrumentación por sensor con evidencias, (iii) una MATLAB Function robusta para LiDAR con señales de tamaño variable y (iv) criterios mínimos de validación para que el equipo replique las pruebas sin depender de “memoria informal”.

---

## 2) Contexto y objetivo

El contexto de la CPS-IoT 2026 exige resolver conducción tipo **taxi autónomo en Quanser City** manteniendo el stack en **MATLAB/Simulink**. Por ello, este Punto 1 no busca “ganar” el reto final, sino establecer una base técnica sólida: verificar sensores, formato de datos, sincronización temporal, coherencia de unidades y *sanity checks*.

> **Nota crítica de ejecución:** el modelo incluye bloques tipo **HIL Initialize**, por lo que **Run normal** no es compatible para la ejecución completa. La validación se realizó en **QUARC → Monitor & Tune (External Mode)**.

---

## 3) Alcance y entregables

### Alcance

- Configurar el entorno virtual y ubicar el QCar de forma controlada.
- Establecer el pipeline de ejecución en tiempo real (External Mode).
- Validar LiDAR, IMU/cinemática, pose estimada y telemetría eléctrica.
- Identificar y documentar limitaciones de cámara (Depth/RGB) en External Mode.
- Dejar evidencia visual (capturas de QLabs y Scopes) por sensor crítico.

### Entregables del Punto 1

- Secuencia reproducible de arranque (scripts + orden de ejecución).
- Instrumentación por sensor (Scopes y funciones MATLAB).
- Capturas de QLabs (entorno + posición del QCar).
- Capturas de Simulink (bloques de sensores, MATLAB Function, Scopes).
- Lista de incidencias y decisión técnica (cámara).
- MATLAB Function del LiDAR utilizada para obtener `dmin` y `angMin`.

### Definition of Done (DoD)

Se considera completado el Punto 1 cuando:

- Se demuestra recepción estable del LiDAR (flag `lidarNewReading` activo).
- Se obtiene `dmin` y `angMin` sin errores por señales de tamaño variable.
- Se validan señales cinemáticas (velocidad/gyro) coherentes.
- Se observa pose estimada (`currentPose`) en tiempo real.
- Se monitorea telemetría eléctrica en un Scope 2×2 con rangos definidos.

---

## 4) Setup Virtual (QLabs + QUARC)

### Checklist de prerequisitos

- MATLAB/Simulink con QUARC instalado y funcional.
- QLabs operativo (sesión iniciada y sin actores “zombies” de corridas previas).
- Proyecto Quanser correcto (rutas, modelos y scripts accesibles).
- Preparación para ejecutar el modelo en **Monitor & Tune (External Mode)**.

![Quanser Interactive Labs — entorno virtual]({{ "/assets/images/quanserInteractiveLabs.png" | relative_url }})
*Vista del entorno virtual con el mapa cargado y el QCar visible.*

---

## 5) Arranque correcto del entorno (QLabs + mapa + QCar)

El primer paso fue preparar QLabs y el mapa de competencia mediante `setup_competition_map.m`. Este script se encarga de abrir conexión con QLabs, limpiar actores previos, spawnear el mapa y finalmente spawnear el QCar2. Adicionalmente, inicializa el RT model necesario para que el stack corra con QUARC.

La lógica de arranque se aplicó en dos fases:

- **Fase de calibración:** `spawn_location = 1` → ejecutar `setup_competition_map.m` → QCar en zona de calibración.  
- **Fase de navegación:** `spawn_location = 2` → ejecutar nuevamente `setup_competition_map.m` → QCar en **taxi hub**.

> Este orden es importante: si el QCar no se spawnea en la ubicación correcta o el RT model no está activo, Simulink puede abrir, pero los sensores no entregan datos utilizables.

---

## 6) Calibración previa y verificación base

Con `spawn_location = 1`, se ejecutó el modelo `virtual_calibrate.slx` como corrida corta de validación. Esta etapa sirve para confirmar que el sistema está alineado (en especial el LiDAR respecto a los frames del mapa) y que el vehículo está activo.

![Modelo de calibración — virtual_calibrate]({{ "/assets/images/QCar_virtual_calibrate.png" | relative_url }})
*Ejecución del modelo de calibración previo (`virtual_calibrate.slx`) para verificar alineación y adquisición base.*

Una vez verificada la calibración, se cambió el spawn a taxi hub para operar el stack en el escenario de reto.

---

## 7) Carga de parámetros globales (`setup_QCar_params`)

Antes de instrumentar sensores dentro del stack, se ejecutó `setup_QCar_params` (o equivalente `Setup_QCar2_Params.m`). Este paso define tiempos de muestreo, tamaño del scan del LiDAR (`SPR_qcar2`), offsets/rotaciones de calibración, parámetros de control PD, parámetros de KF/EKF, y carga de calibraciones (`.mat`) necesarias para la instrumentación.

---

## 8) Ejecución del modelo principal (`virtual_self_driving_stack_v2`)

Con QLabs corriendo, el QCar spawneado correctamente y parámetros globales cargados, se ejecutó el modelo principal `virtual_self_driving_stack_v2`.

> **Modo obligatorio:** ejecutar en **QUARC → Monitor & Tune (External Mode)**. En “Run normal” se bloquea la ejecución completa por la presencia de bloques **HIL Initialize**.

---

## 9) Instrumentación del LiDAR (medición estable y sin errores)

El LiDAR fue el sensor más crítico y el que más restricciones impone por el manejo de señales **variable-size**.

### 9.1 Señales instrumentadas

- `lidarDistances [var] (m)`
- `lidarHeadings [var] (rad)`
- `lidarNewReading [1] (bool)`
- (Salida procesada) `dmin`, `angMin`

### 9.2 Validación de nueva lectura

- `lidarNewReading` se conectó a un Scope (*stairs*) para confirmar adquisición en tiempo real.

### 9.3 Problema principal (variable-size)

- `lidarDistances` y `lidarHeadings` salen como señales de tamaño variable (`[var]`).
- Bloques estándar (p. ej. MinMax, Saturation) pueden fallar con errores tipo *“fixed-size expected”*.

### 9.4 Solución aplicada (MATLAB Function robusta)

Se procesó el scan mediante una **MATLAB Function** compatible con variable-size, filtrando inválidos (`NaN/Inf` y rangos fuera de operación) y calculando:

- `dmin`: distancia mínima válida  
- `angMin`: ángulo asociado al mínimo

![Subbloque LiDAR — captura y offsets de frame]({{ "/assets/images/lidarCapture.png" | relative_url }})
*Bloque de captura/procesamiento de LiDAR y ajustes de offsets/frames.*

![LiDAR — distancias, headings y cálculo de nearest obstacle]({{ "/assets/images/lidarDistanceHeading.png" | relative_url }})
*Instrumentación de `lidarDistances`, `lidarHeadings`, `newLidarReading` y salida `dmin/angMin`.*

![Mapa LiDAR 2D]({{ "/assets/images/Lidar.png" | relative_url }})
*Visualización 2D de la nube de puntos del LiDAR detectando la geometría del espacio.*

---

## 10) IMU y cinemática (odometría)

Una vez estable el LiDAR, se instrumentaron señales cinemáticas:

- `measuredSpeed` (m/s)  
- `gyro` (rad/s)  
- (opcional) acelerómetro (m/s²) si está disponible en el modelo

Se validó coherencia física:
- si el vehículo acelera → `measuredSpeed` sube  
- si el vehículo gira → `gyro` cambia  
- no hay discontinuidades raras

![Velocidad medida — `measuredSpeed` (m/s)]({{ "/assets/images/speedQCar.png" | relative_url }})
*Scope de velocidad (`measuredSpeed`) mostrando variación coherente durante la corrida.*

![IMU / acelerómetro — validación temporal]({{ "/assets/images/IMU.png" | relative_url }})
*Scope del acelerómetro/IMU para ver estabilidad y coherencia de magnitudes.*

---

## 11) Pose estimada (`currentPose`)

Se instrumentó el estado `[x, y, heading]` calculado por el stack, usando `lidarPose` y `currentPose`. Para visualizarlo se usó Scope con “Elements as channels” o `Demux(3)`.

Sanity checks:
- `x` e `y` cambian suave conforme el vehículo se mueve  
- `heading` cambia cuando gira  

![Pose estimada — Scope de `currentPose` (x, y, theta)]({{ "/assets/images/medicionCurrentPose.png" | relative_url }})
*Visualización de `currentPose`: evolución de `x`, `y` y `theta` en tiempo real.*

---

## 12) Telemetría eléctrica (voltaje, corriente, nivel, potencia)

Finalmente se instrumentaron señales internas del QCar:

- `batteryVoltage` (V)
- `motorCurrent` (A)
- `batteryLevel` (%)
- `motorPower` (W)

Se organizó la visualización en un Scope 2×2 y se asignaron rangos manuales para que fuera legible. Se verificó que corriente/potencia suban al acelerar y que los valores sean plausibles.

![Lectura de telemetría — readQCarADC]({{ "/assets/images/readQCarADC.png" | relative_url }})
*Bloque de lectura/instrumentación de telemetría eléctrica y señales internas.*

---

## 13) Evidencia de mapa / trayectoria (visualización 2D)

Como evidencia adicional de consistencia global (pose + entorno), se generó una visualización 2D en MATLAB donde se aprecia el trazo del vehículo respecto al mapa.

![Ruta trazada — evidencia de consistencia global]({{ "/assets/images/rutaTrazada.png" | relative_url }})
*Figura 2D de trayectoria / referencia del entorno para validar consistencia espacial.*

---

## 14) Cámara (Depth/RGB) — intento y decisión técnica

Se intentó activar la cámara con Video3D. En External Mode se presentó el error:

- `video format is not supported`

Conclusión: la configuración actual del pipeline Video3D no es compatible con External Mode (formato/resolución/FPS/stream). Para no romper ejecución se decidió:

- **Posponer cámara**
- Priorizar **LiDAR + pose + cinemática**, suficientes para avanzar con navegación

---

## 15) Resultado final

Después de este proceso, el stack sensorial quedó funcionando con:

- LiDAR midiendo y entregando `dmin/angMin` sin errores por variable-size
- IMU y odometría coherentes (velocidad y gyro)
- Pose estimada viva (`currentPose`)
- Telemetría eléctrica monitoreada
- Cámara documentada como limitación (por ahora)

Y lo más importante: se dejó un flujo **reproducible** para que cualquier integrante del equipo pueda levantar el entorno, ejecutar el setup, correr el stack en External Mode e instrumentar sensores sin depender de “prueba y error”.

---

## 16) Anexos

### 16.1 MATLAB Function utilizada para el LiDAR (`nearestObstacle`)

```matlab
function [dmin, angMin] = nearestObstacle(dist, ang)
dist = dist(:); ang = ang(:);

valid = isfinite(dist) & dist > 0.05 & dist < 12 & isfinite(ang);

if any(valid)
    dv = dist(valid);
    av = ang(valid);
    [dmin, k] = min(dv);
    angMin = av(k);
else
    dmin = NaN;
    angMin = NaN;
end
end