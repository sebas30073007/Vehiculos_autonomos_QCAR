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