---
title: Punto 1 — Exploración del entorno y sensores (QCar real)
parent: Avances
nav_order: 
---

# Punto 1 — Exploración del entorno y sensores (QCar real)

## 1) Resumen ejecutivo

En este Punto 1 se realizó una **instrumentación inicial** del ecosistema QCar enfocada netamente en la plataforma física (QCar con scripts de adquisición). El objetivo fue confirmar que la cadena de datos de sensores funciona de extremo a extremo y que puede dejar trazabilidad útil para fases posteriores. Se cubrieron cuatro familias de sensores: **Cámaras (visión 360 / RGB-D), LiDAR, IMU y encoders**. También se dejaron lineamientos de guardado para construir datasets comparables entre corridas. Como resultado tangible, este avance entrega estructura de logs, recomendaciones de visualización base y criterios mínimos de validación para que el equipo replique pruebas en hardware real de forma estructurada.

## 2) Contexto y objetivo

El desarrollo exige resolver tareas complejas de percepción y control (como conducción autónoma o teleoperación). Este Punto 1 establece la base técnica física: verificar que el hardware entrega datos, validar el formato, la sincronización temporal, la coherencia de unidades y establecer *sanity checks*. 

Para la exploración de hardware real y adquisición rápida se utiliza **Python**, asegurando que los scripts se comuniquen correctamente con las APIs del vehículo.

## 3) Alcance y entregables

### Alcance

- Explorar adquisición en el QCar real con logging reproducible.
- Integrar y visualizar la lectura de cámaras 360 y escáner LiDAR.
- Definir estructura mínima de datasets y convenciones de nombres.
- Identificar problemas típicos de integración de hardware y mitigaciones.

### Entregables del Punto 1

- Scripts/cuadernos de adquisición y visualización en Python.
- Logs/datasets de prueba en formato tabular y/o binario.
- Capturas del setup físico y streams de sensores (Cámaras y LiDAR).
- Plots básicos de IMU para validación rápida.

### Definition of Done (DoD)

Se considera completado el Punto 1 cuando:

- Se demuestra captura de al menos una corrida física completa.
- Existen timestamps y metadatos suficientes para reconstruir la secuencia.
- Hay evidencia visual (capturas/plots) por sensor crítico integrado.
- Se documentan incidencias de hardware/software y su solución aplicada o pendiente.

## 4) Setup Físico (QCar)

### Checklist de prerequisitos

- Energía (baterías cargadas) y estado físico del vehículo verificados.
- Conectividad local (Wi-Fi/red del laboratorio) operativa para acceder a la computadora a bordo.
- Zona segura de prueba, sin obstáculos no controlados.
- Usuario con permisos para ejecutar scripts, crear carpetas y escribir logs en el sistema del QCar.

![QCar real – setup](20260219_183606.jpg)
*Vista del hardware del QCar instrumentado sobre la mesa de trabajo, mostrando el LiDAR superior, cámaras y la electrónica expuesta.*

### Notas de entorno Python

- Definir un entorno virtual dedicado (`venv` o conda) en la placa de desarrollo del QCar.
- Instalar dependencias de captura y visualización (ej. librerías de Quanser HAL/PAL, OpenCV).
- Si se utilizan cámaras de profundidad y aparece `ModuleNotFoundError: pyrealsense2`, evaluar la instalación explícita del paquete compatible con el SO/arquitectura ARM del vehículo.

## 5) Sensores y datos

| Sensor | Qué mide | Unidad típica | Qué se guardó | Uso inmediato (plot/preview) | Riesgos |
|---|---|---|---|---|---|
| Cámaras (360 / RGB) | Imágenes del entorno alrededor del vehículo | color (8-bit) | Frames RGB concatenados, timestamp | Preview en vivo de Combined View | Desalineación de cámaras, baja exposición, lag de transmisión |
| LiDAR | Distancia angular del entorno 2D | m (rango), rad/deg (ángulo) | Scan crudo por barrido + timestamp | Mapa 2D de puntos / Polar plot | Valores `0/inf`, reflexión en ventanas/metales, ruido |
| IMU | Aceleración y velocidad angular | m/s², rad/s (o deg/s) | Series temporales + timestamp | Plot `ax/ay/az`, `gx/gy/gz` | Drift, ejes mal interpretados, offset de calibración |
| Encoders | Rotación de ruedas para odometría | ticks, rad, m/s, m | ticks/contador y tiempo | Velocidad estimada y distancia | Conversión con factor incorrecto, signo invertido |

## 6) Flujo de trabajo — Adquisición (Python)

### 6.1 Objetivo

Confirmar adquisición de sensores sobre el hardware real, visualizar los datos en tiempo real para depuración y producir logs limpios.

### 6.2 Procedimiento (paso a paso)

**A) Visión (Cámaras 360 / RGB)**

1. Inicializar el stream de las múltiples cámaras del QCar.
2. Ensamblar los frames en una vista combinada (Combined View) para tener perspectiva total.
3. Verificar la tasa de refresco (FPS) y la correcta iluminación del entorno.
4. Asegurar que el timestamp se registra en cada captura guardada.

![Cámaras 360 - Combined View](Screenshot%20from%202026-02-19%2018-28-23.jpg)
*Vista combinada de las cámaras del QCar capturando el entorno del laboratorio en 360 grados.*

**B) LiDAR**

1. Capturar el array de datos crudos por iteración del escáner.
2. Filtrar lecturas inválidas (`0`, `inf`, `nan`) antes de mapear.
3. Generar vista 2D instantánea (proyección XY).
4. Guardar scan junto con el timestamp unificado del logger.

![Mapa LiDAR 2D](Screenshot%20from%202026-02-19%2018-29-23.png)
*Visualización 2D de la nube de puntos del LiDAR detectando la geometría del espacio.*

**C) IMU y Encoders**

1. Leer aceleración, giroscopio y contadores de rueda en la misma base temporal.
2. Ejecutar *sanity checks* (ej. magnitud de aceleración en reposo ~9.81 m/s²).
3. Calcular variación de ticks y `delta_t` para estimar velocidad.

### 6.3 Exportación / logging

**Estructura sugerida**

```text
/data/punto-1/YYYY-MM-DD_run01/
  ├── cameras_360/
  ├── lidar/
  ├── imu/
  └── encoders/