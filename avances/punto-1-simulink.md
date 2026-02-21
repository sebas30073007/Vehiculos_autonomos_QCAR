---
title: Punto 1 — Exploración del entorno y sensores (Virtual + QCar real)
parent: Avances
nav_order:
---

# Punto 1 — Exploración del entorno y sensores (Virtual + QCar real)

## 1) Resumen ejecutivo

En este Punto 1 se realizó la **instrumentación y validación integral** del stack sensorial del **QCar2 virtual** dentro del ecosistema **MATLAB/Simulink + QUARC (Monitor & Tune / External Mode) + QLabs**. El objetivo fue confirmar que la cadena de datos de sensores funciona de extremo a extremo y que la información es coherente (unidades, magnitudes, estabilidad temporal) para soportar fases posteriores de control y navegación autónoma.

Se validaron los siguientes componentes:

- **LiDAR** (distancias, ángulos y bandera de nueva lectura).
- **IMU/gyro y cinemática** (velocidad y giro).
- **Estimación de pose** (estado `[x, y, heading]`).
- **Telemetría eléctrica** (voltaje, corriente, nivel, potencia).
- Se intentó integrar **cámara Depth/RGB** (Video3D), pero se documentó la limitación en External Mode.

Como resultado tangible, este avance entrega: (i) secuencia reproducible de arranque del entorno, (ii) instrumentación por sensor con evidencias, (iii) funciones MATLAB robustas para señales LiDAR de tamaño variable y (iv) criterios mínimos de validación para que el equipo replique las pruebas sin depender de “memoria informal”.

---

## 2) Relación con la competencia

El contexto de la CPS-IoT 2026 exige resolver conducción tipo **taxi autónomo en Quanser City** y mantener un stack en **MATLAB/Simulink**. Por ello este Punto 1 no busca “ganar” el reto final, sino establecer una base técnica sólida: verificar sensores, formato de datos, sincronización temporal, coherencia de unidades y *sanity checks*.

> **Nota importante de ejecución:** el modelo contiene bloques tipo **HIL Initialize**, por lo que **Run normal** no es compatible para la ejecución completa. La validación se realizó en **QUARC → Monitor & Tune (External Mode)**.

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

### Definition of Done (DoD)

Se considera completado el Punto 1 cuando:

- Se demuestra recepción estable del LiDAR (flag `lidarNewReading` activo).
- Se obtiene `dmin` y `angMin` sin errores por señales de tamaño variable.
- Se validan señales cinemáticas (velocidad/gyro) coherentes.
- Se observa pose estimada (`currentPose`) en tiempo real.
- Se monitorea telemetría eléctrica en un Scope 2×2 con rangos definidos.

## 4) Setup (Virtual)

### 4.1 Checklist de prerequisitos

- MATLAB/Simulink instalados (versión del curso/lab).
- QLabs con escena de Quanser City disponible.
- QUARC configurado para ejecución en External Mode.
- Permisos para correr scripts y lanzar RT models.

### 4.2 Flujo sugerido (alto nivel)

1. Ejecutar script de mapa/escena y spawn del QCar.
2. Realizar calibración inicial en spot de calibración.
3. Reposicionar al taxi hub (zona de navegación).
4. Cargar parámetros globales (timings, LiDAR, control, filtros).
5. Ejecutar el modelo principal del stack y proceder con instrumentación sensorial.

---

## 5) Secuencia completa de arranque del entorno (paso a paso)

### 5.1 `setup_competition_map.m` (mapa, QLabs, spawn, RT model)

El script `setup_competition_map.m` se utilizó para:

- Conectar a QLabs.
- Limpiar actores previos (`destroy_all_spawned_actors`).
- Spawnear flooring, muros y cámaras.
- Spawnear el QCar2 con una ubicación definida por `spawn_location`.
- Lanzar el RT model asociado (`quarc_run ... QCar2_Workspace_studio.rt-win64`).

**Código utilizado (fragmento relevante):**

```matlab
%% Configurable Params

% Choose spawn location of QCar
% 1 => calibration location
% 2 => taxi hub area
spawn_location = 2;

%% Set up QLabs Connection and Variables

newPathEntry = fullfile(getenv('QAL_DIR'), '0_libraries', 'matlab', 'qvl');
pathCell = regexp(path, pathsep, 'split');
if ispc
  onPath = any(strcmpi(newPathEntry, pathCell));
else
  onPath = any(strcmp(newPathEntry, pathCell));
end
if onPath == 0
    path(path, newPathEntry)
    savepath
end

% Stop RT models
try
    qc_stop_model('tcpip://localhost:17000', 'QCar2_Workspace')
catch error
end
pause(1)

try
    qc_stop_model('tcpip://localhost:17000', 'QCar2_Workspace_studio')
    pause(1)
catch error
end
pause(1)

% QLab connection
qlabs = QuanserInteractiveLabs();
connection_established = qlabs.open('localhost');

if connection_established == false
    disp("Failed to open connection.")
    return
end
disp('Connected')
verbose = true;
num_destroyed = qlabs.destroy_all_spawned_actors();

%World Objects
x_offset = 0.13;
y_offset = 1.67;
hFloor = QLabsQCarFlooring(qlabs);
hFloor.spawn_degrees([x_offset, y_offset, 0.001],[0, 0, -90]);

%region: Walls
hWall = QLabsWalls(qlabs);
hWall.set_enable_dynamics(false);

for y = 0:4
    hWall.spawn_degrees([-2.4 + x_offset, (-y*1.0)+2.55 + y_offset, 0.001], [0, 0, 0]);
end

for x = 0:4
    hWall.spawn_degrees([-1.9+x + x_offset, 3.05+ y_offset, 0.001], [0, 0, 90]);
end

for y = 0:5
    hWall.spawn_degrees([2.4+ x_offset, (-y*1.0)+2.55 + y_offset, 0.001], [0, 0, 0]);
end

for x = 0:3
    hWall.spawn_degrees([-0.9+x+ x_offset, -3.05+ y_offset, 0.001], [0, 0, 90]);
end

hWall.spawn_degrees([-2.03 + x_offset, -2.275+ y_offset, 0.001], [0, 0, 48]);
hWall.spawn_degrees([-1.575+ x_offset, -2.7+ y_offset, 0.001], [0, 0, 48]);

%spawn cameras
camera1Loc = [0.15, 1.7, 5];
camera1Rot = [0, 90, 0];
camera1 = QLabsFreeCamera(qlabs);
camera1.spawn_degrees(camera1Loc, camera1Rot);
camera1.possess();

camera2Loc = [-0.36+ x_offset, -3.691+ y_offset, 2.652];
camera2Rot = [0, 47, 90];
camera2=QLabsFreeCamera(qlabs);
camera2.spawn_degrees (camera2Loc, camera2Rot);

%% Spawn QCar 2 and start rt model

calibration_location_rotation = [0, 2.13, 0.005, 0, 0, -90];
taxi_hub_location_rotation = [-1.205, -0.83, 0.005, 0, 0, -44.7];

myCar = QLabsQCar2(qlabs);

switch spawn_location
    case 1
        spawn = calibration_location_rotation;
    case 2
        spawn = taxi_hub_location_rotation;
end

myCar.spawn_id_degrees(0, spawn(1:3), spawn(4:6), [1/10, 1/10, 1/10], 1);

% Start RT models
file_workspace = fullfile(getenv('RTMODELS_DIR'), 'QCar2', 'QCar2_Workspace_studio.rt-win64');
pause(2)
system(['quarc_run -D -r -t tcpip://localhost:17000 ', file_workspace]);
pause(3)

qlabs.close()

Este archivo define:

1) Tiempos de muestreo (controller, LiDAR, RealSense, etc.).

2) Tamaño de captura del LiDAR (SPR).

3) Calibraciones angulares del LiDAR (virtual-to-physical, map rotation).

4) Control PD de dirección (virtual/physical).

5) Parametrización de KF/EKF.

6) Carga de calibraciones (distance_new_qcar2.mat, angles_new_qcar2.mat).

7) Carga y visualización de rutas SDCS.
