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

Texto completo — Obtención correcta de sensores (QCar2 Virtual)

Para obtener correctamente las mediciones de los sensores del QCar2 en el entorno virtual, primero se aseguró que el sistema completo estuviera ejecutándose en el modo correcto (QUARC / tiempo real), y después se instrumentaron los sensores uno por uno verificando que cada señal estuviera “viva”, que fuera coherente con el movimiento del vehículo y que no provocara errores de compilación por el tipo de dato o por el tamaño de las señales.

## 1) Arranque correcto del entorno (QLabs + mapa + QCar)

El primer paso fue preparar QLabs y el mapa de competencia. Para esto se utilizó el archivo setup_competition_map.m, que se encarga de abrir conexión con QLabs, limpiar actores previos, spawnear el piso y muros del mapa, crear cámaras y finalmente spawnear el QCar2. Ese script también inicia el RT model necesario para que el stack pueda correr con QUARC.

La lógica de arranque se hizo en dos fases:

Fase de calibración: se pone spawn_location = 1, se corre setup_competition_map.m y se spawnea el QCar en la zona de calibración.

Fase de navegación: después, se cambia spawn_location = 2, se corre otra vez el mismo script, y el QCar aparece en la zona del taxi hub (que es donde realmente se quiere operar el stack del reto).

Esto es importante porque si no se spawnea en el lugar correcto o si no se inicia el RT model, Simulink puede abrir pero los sensores no entregan datos reales.

## 2) Calibración previa y verificación base

Con el spawn_location = 1, se ejecutó el modelo virtual_calibrate.slx. Esta corrida corta sirve para confirmar que los sistemas están bien alineados, en especial el LiDAR y su relación con frames de referencia del mapa. En esta etapa también se confirmó que QLabs realmente estaba corriendo y que el vehículo estaba activo (no congelado).

Después de esa calibración, se volvió al script de competencia y se cambió el spawn a la ubicación de taxi hub.

## 3) Carga de parámetros globales (setup_QCar_params)

Antes de instrumentar sensores dentro del stack, se ejecutó el script setup_QCar_params (o equivalente Setup_QCar2_Params). Este paso es clave porque ahí se definen:

Tiempos de muestreo de cada subsistema (controller, LiDAR, RealSense, etc.).

Tamaño del scan del LiDAR (SPR_qcar2).

Rotaciones de calibración (por ejemplo offsets virtual-to-physical y map frame).

Parámetros de control PD para dirección.

Parámetros del KF / EKF.

Carga de archivos de calibración de ángulos y distancias del LiDAR (matfiles).

Carga y visualización de paths SDCS.

Si este script no se corre antes de ejecutar el modelo principal, aparecen errores de variables no definidas o el stack funciona pero de forma inconsistente.

## 4) Ejecución del modelo principal (virtual_self_driving_stack_v2)

Con QLabs corriendo, el QCar spawneado correctamente y los parámetros cargados, se abrió y ejecutó el modelo principal virtual_self_driving_stack_v2. Aquí hubo una consideración crítica: no se puede correr en “Run normal”, porque el modelo tiene bloques tipo HIL Initialize. Por seguridad, QUARC bloquea el acceso a ese hardware/RT en modo normal, así que el modo correcto fue correr siempre en:

Monitor & Tune (External Mode)

Ese fue el modo que permitió que las señales fueran entregadas de forma continua y en tiempo real.

## 5) Instrumentación del LiDAR: cómo se logró que midiera sin errores

El LiDAR fue el sensor más importante y también el que más problemas puede causar si se instrumenta mal.

Primero se validó el flag de nueva lectura:

lidarNewReading se conectó a un Scope en modo stairs.

Cuando la señal da pulsos o se mantiene actualizando en 1, se confirma que sí están llegando nuevos scans.

Después vino el problema principal: lidarDistances sale como señal de tamaño variable ([var]). Por eso cuando se intentó usar bloques como Saturation o MinMax directamente, Simulink marcaba error de “fixed-size expected”.

La solución fue no usar esos bloques estándar y en su lugar procesar el scan con una MATLAB Function robusta y compatible con variable-size. Esa función filtra inválidos (NaN/Inf y rangos fuera de operación) y calcula:

dmin: la distancia mínima válida

angMin: el ángulo donde ocurre esa distancia mínima

Con eso, el scan de 1000 puntos se convirtió en dos señales escalares estables que se podían visualizar con Display o Scope sin volver a romper el modelo.

Además, se explicó y validó qué significa lidarHeading: cada distancia tiene un ángulo asociado, así que al obtener el índice del mínimo también se obtiene la dirección del obstáculo más cercano.

## 6) Instrumentación de IMU y odometría (cinemática)

Una vez que el LiDAR estaba estable, se pasó a señales cinemáticas:

measuredSpeed para velocidad lineal

gyro (rad/s) para giro

Se conectaron ambos a scopes normales, con rangos manuales para que fuera legible. Se validó coherencia física:

Si el vehículo acelera, la velocidad sube.

Si el vehículo gira, el gyro cambia.

No hay saltos raros o discontinuidades.

## 7) Instrumentación de pose (estado estimado)

Después se buscó y validó el estado [x, y, heading] que el stack calcula, usando:

lidarPose como salida de localización por LiDAR

currentPose como la pose estimada final (fusión/estado)

Para visualizarlo se usó Scope con “Elements as channels” o se separó con un Demux(3). Con esto se observó que:

x e y cambian suave conforme el coche se mueve

heading cambia cuando gira

Esto confirmó que la estimación de estado está viva y usable para control.

## 8) Telemetría eléctrica (sensores internos del sistema)

Finalmente se instrumentaron señales internas del QCar:

batteryVoltage

motorCurrent

batteryLevel

motorPower

Se pusieron en un Scope 2×2 y se asignaron rangos manuales. Se verificó que corriente y potencia suban cuando hay aceleración, y que los valores sean plausibles (sin negativos raros).

## 9) Cámara (Depth/RGB): intento y decisión

Se intentó activar la captura de cámara con Video3D. Aquí se detectó un problema fuerte:

En External Mode apareció el error “video format is not supported”.

Se concluyó que la configuración actual del pipeline Video3D no es compatible con External Mode bajo ese formato (por tamaño, fps o tipo). Dado que el objetivo era instrumentar el stack sensorial sin romper ejecución, se tomó la decisión técnica de:

Posponer cámara

Priorizar LiDAR + pose + cinemática, que ya era suficiente para avanzar con navegación.

Resultado final

Después de este proceso, el stack sensorial quedó funcionando con:

LiDAR midiendo y entregando dmin/angMin sin errores por variable-size

IMU y odometría coherentes

Pose estimada viva (currentPose)

Telemetría eléctrica monitoreada

Cámara documentada como limitación (por ahora)

Y lo más importante: se dejó un flujo reproducible para que cualquier integrante del equipo pueda volver a levantar el entorno, correr el setup, lanzar el stack y validar sensores en tiempo real sin “prueba y error”.