---
title: Punto 1 — Exploración del entorno y sensores (Virtual + QCar real)
parent: Avances
nav_order: 1
---

# Resumen ejecutivo del entorno virtual en Simulink

Esta sección documenta la configuración técnica y el flujo de trabajo dentro del ecosistema **MATLAB/Simulink** y **QLabs**. El enfoque principal es garantizar que el stack de software sea capaz de comunicarse con el gemelo digital del QCar2 de manera determinista.

## Alcances y limitaciones en Simulink

Es fundamental entender las fronteras de nuestro entorno de desarrollo para evitar errores de diseño:

* **Alcances:**
    * **Control en Tiempo Real:** Uso de *Simulink Desktop Real-Time* para garantizar pasos de integración constantes.
    * **Acceso Total a Sensores:** Recepción de datos de LiDAR, cámaras RGB-D, IMU y odometría.
    * **Entorno Seguro:** Pruebas de algoritmos de colisión sin riesgo de daño físico al hardware.
* **Limitaciones:**
    * **Fidelidad de Sensores:** El ruido en el LiDAR virtual es gaussiano idealizado, a diferencia del ruido por superficies reflectantes en el mundo real.
    * **Carga Computacional:** El renderizado de QLabs y la ejecución del modelo de Simulink en la misma máquina pueden generar *overruns* (pérdida de pasos de tiempo) si no se optimiza el solver.


## Flujo de trabajo - Pasos a seguir en Simulink

Para garantizar la repetitibilidad de las pruebas, el equipo siguió este orden estricto:

### 1. Definición del entorno virtual (Mapa de competencia)
Se utiliza el entorno **Quanser City**. Es vital verificar que la versión de QLabs sea la correspondiente a la **CPS-IoT 2026**, ya que los nodos de navegación y señales de tráfico están configurados bajo este estándar.

### 2. Calibración del QCar2 virtual
Antes de mover el vehículo, se debe validar el archivo de configuración de parámetros. Esto incluye:
* Verificar constantes de la planta (distancia entre ejes, radio de rueda).
* Configurar los puertos de comunicación (por defecto, la IP suele ser `127.0.0.1` para entornos locales).



### 3. Reubicación del QCar al área de navegación
El vehículo no siempre spawnea en la línea de salida. Se debe utilizar el comando de reubicación o los bloques de *Initialization* para posicionar el QCar en las coordenadas $x, y, z$ y orientación $\psi$ (yaw) específicas del punto de partida del "taxi autónomo".

### 4. Ejecución de `Setup_QCar2_Params.m`
Este script es la columna vertebral del proyecto. Define en el **Workspace de MATLAB**:
* Matrices de ganancia para el control.
* Límites de saturación de los motores.
* Frecuencias de muestreo ($T_s$) para los sensores.
> **Importante:** Si este script no se corre previamente, los bloques de Simulink mostrarán errores de "Variable no definida".

### 5. Apertura del Virtual Self-Driving Stack
Finalmente, se abre el modelo principal de Simulink. Este modelo integra:
* **Percepción:** Bloques que transforman el stream de datos del LiDAR y cámara.
* **Decisión:** Lógica de seguimiento de carril o evasión.
* **Actuación:** Bloques de escritura que envían comandos de aceleración y ángulo de dirección al simulador.

## Punto 1 — Exploración del entorno y sensores (Virtual + QCar real)

1) Resumen ejecutivo

En este Punto 1 se realizó la instrumentación y validación completa de los sensores del QCar2 en entorno virtual utilizando MATLAB/Simulink junto con QLabs y ejecución en modo QUARC (Monitor & Tune).

El objetivo principal no fue implementar aún el algoritmo final de conducción autónoma, sino garantizar que el sistema sensorial del vehículo funciona correctamente, que las señales son coherentes con el movimiento del entorno virtual y que el modelo puede ejecutarse en tiempo real sin errores estructurales.

Se validaron los siguientes sistemas:

- LiDAR (detección de obstáculos)
- IMU (velocidad angular)
- Odometría (velocidad lineal)
- Estimación de pose (posición y orientación del vehículo)
- Telemetría eléctrica (estado interno del sistema)
- Intento de integración de cámara (Depth/RGB)

El resultado fue un stack virtual estable, completamente instrumentado y listo para avanzar hacia control y navegación.

2) Contexto técnico de ejecución

El modelo contiene bloques tipo HIL Initialize, lo que implica que no puede ejecutarse en modo Run normal.

Por esta razón, toda la validación se realizó en modo Monitor & Tune (External Mode), lo cual permite:

- Comunicación determinista con el entorno virtual.
- Actualización en tiempo real de señales.
- Visualización continua mediante Scopes.

Este detalle es importante porque explica por qué ciertos errores aparecían al intentar correr el modelo en modo normal.

3) Validación del LiDAR
3.1 Señales utilizadas

Se trabajó con las siguientes señales:

- lidarDistances → vector de distancias medidas.
- lidarHeadings → vector de ángulos asociados a cada medición.
- lidarNewReading → bandera booleana que indica llegada de un nuevo barrido.

3.2 Confirmación de funcionamiento

Se conectó lidarNewReading a un Scope en modo tipo escalón (stairs).
La presencia de pulsos confirmó que el sensor estaba generando lecturas activamente.

3.3 Problema detectado: tamaño variable

La señal lidarDistances aparece como variable-size ([var]).

Bloques como Saturation o MinMax requieren tamaño fijo, lo que generaba errores de compilación en External Mode.

3.4 Solución implementada

Se reemplazó la lógica de mínimo por una función MATLAB robusta que:

- Convierte el vector a columna.
- Filtra valores inválidos (NaN, infinito o fuera de rango).
- Calcula el mínimo solo sobre valores válidos.

Esto permitió calcular la distancia mínima (dmin) sin errores.

Además, se implementó el cálculo del ángulo correspondiente (angMin), lo que permite conocer no solo qué tan cerca está un obstáculo, sino en qué dirección se encuentra.

Resultado: LiDAR completamente funcional y usable para navegación reactiva.

4) Validación de IMU y Odometría
4.1 Señales instrumentadas

measuredSpeed (velocidad lineal del vehículo)

gyro (velocidad angular en yaw)

Estas señales se conectaron a Scopes normales con rango manual definido para evitar autoescala inestable.

4.2 Validación física

Se verificó que:

 - Cuando el vehículo acelera, measuredSpeed aumenta.
 - Cuando el vehículo gira, el gyro refleja velocidad angular coherente.
 - No existen saltos abruptos o discontinuidades artificiales.

Resultado: cinemática consistente con el movimiento observado en QLabs.

5) Estimación de pose
5.1 Señales identificadas

lidarPose [x, y, heading]

currentPose [x, y, heading]

Estas representan la estimación de estado del vehículo.

5.2 Método de visualización

Se utilizaron dos enfoques:

- Scope con “Elements as channels”
- Demux(3) para separar x, y y heading

Se configuró el rango del heading entre aproximadamente -3.2 y 3.2 rad (±π).

5.3 Validación

Se observó que:

- x e y cambian suavemente conforme el vehículo se desplaza.
- heading varía coherentemente con los giros.
- No hay saltos erráticos.

Resultado: la estimación de estado es estable y consistente.

6) Telemetría eléctrica

Se instrumentaron las siguientes señales internas del vehículo:

- batteryVoltage
- motorCurrent
- batteryLevel
- motorPower

Se utilizó un Scope 2×2 para visualización simultánea.

Validaciones realizadas

 - El voltaje permanece dentro de rango lógico.
 - La corriente aumenta cuando el vehículo acelera.
 - La potencia se correlaciona con la demanda dinámica.
 - No existen valores negativos físicamente imposibles.

Resultado: monitoreo energético operativo y coherente.

7) Intento de integración de cámara

Se intentó activar el bloque Video3D Capture para Depth/RGB.

Problema observado:
En External Mode aparece el error “video format is not supported”.

Interpretación técnica:

El pipeline Video3D no es completamente compatible con la configuración actual de External Mode, probablemente por formato de datos o resolución.

Decisión tomada:

Posponer la integración de cámara.

Priorizar LiDAR + odometría + pose para garantizar estabilidad del sistema.

8) Problemas encontrados y soluciones aplicadas

 - Error en Run normal → ejecutar en Monitor & Tune.
 - Error en bloques de tamaño fijo → usar MATLAB Function robusta.
 - Error en Video3D → posponer integración.
 - Escalas incorrectas en Scope → definir límites manuales.

9) Estado final del Punto 1

Al finalizar esta fase:

 - LiDAR entrega datos válidos y procesables.
 - IMU y velocidad son coherentes.
 - Pose estimada es estable.
 - Telemetría eléctrica funciona correctamente.

Se documentó la limitación del módulo de cámara.

El stack virtual se encuentra completamente instrumentado a nivel sensorial y listo para avanzar hacia control y navegación autónoma.
