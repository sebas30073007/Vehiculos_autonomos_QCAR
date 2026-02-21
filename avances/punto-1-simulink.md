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

## 6. Instrumentación y validación integral de sensores

Con el entorno virtual correctamente configurado y el modelo ejecutándose en modo **Monitor & Tune (External Mode)**, se procedió a instrumentar y validar cada uno de los sensores disponibles en el QCar2 virtual.

El objetivo de esta fase no fue implementar aún la lógica de navegación autónoma, sino garantizar que:

- Las señales se reciben correctamente.
- No existen errores estructurales en el modelo.
- Las magnitudes físicas son coherentes.
- Las unidades están correctamente interpretadas.
- El sistema es estable en tiempo real.

Esta validación constituye la base sobre la cual se construirá el sistema de percepción y control.

---

### 6.1 Validación del sistema LiDAR

#### Señales analizadas

- `lidarDistances` → vector de distancias (m)
- `lidarHeadings` → vector de ángulos correspondientes (rad)
- `lidarNewReading` → bandera booleana de nueva lectura

#### Confirmación de adquisición

Se conectó `lidarNewReading` a un Scope configurado en modo tipo *stairs*.  
La presencia de pulsos periódicos confirmó la correcta recepción de barridos completos del sensor.

#### Problema detectado: señal de tamaño variable

La señal `lidarDistances` se identifica como `[var]`, es decir, de tamaño variable.  
Esto generaba errores al utilizar bloques como `Saturation` o `MinMax`, ya que estos requieren señales de tamaño fijo en External Mode.

#### Solución implementada

Se desarrolló una función MATLAB personalizada que:

- Convierte el vector a formato columna.
- Filtra valores inválidos (`NaN`, `Inf`, fuera de rango útil).
- Calcula la distancia mínima únicamente sobre valores válidos.

Esto permitió obtener una variable escalar `dmin` robusta y compatible con ejecución en tiempo real.

Adicionalmente, se calculó el ángulo correspondiente al obstáculo más cercano utilizando `lidarHeadings`, permitiendo determinar no solo proximidad, sino dirección del obstáculo.

#### Resultado

El LiDAR quedó completamente funcional, permitiendo:

- Detección reactiva de obstáculos.
- Extracción de métricas simplificadas (mínimo, dirección).
- Base para futuras estrategias de evasión.

---

**[INSERTAR CAPTURA AQUÍ — Scope de lidarNewReading]**

**[INSERTAR CAPTURA AQUÍ — Gráfica de distancia mínima en tiempo real]**

**[INSERTAR CAPTURA AQUÍ — Visualización del ángulo del obstáculo más cercano]**

---

### 6.2 Validación de IMU y cinemática

#### Señales instrumentadas

- `measuredSpeed` (m/s)
- `gyro` (rad/s)

#### Configuración

Se conectaron a Scopes independientes con límites manuales definidos para evitar autoescalado inestable:

- Velocidad: 0 a 5 m/s
- Velocidad angular (yaw): −3 a 3 rad/s

#### Validación física

Se verificó que:

- Al acelerar, la velocidad lineal aumenta de forma suave.
- Durante giros, el giroscopio refleja cambios coherentes en la orientación.
- No existen discontinuidades abruptas o valores físicamente imposibles.

#### Resultado

La dinámica del vehículo virtual es consistente con las señales cinemáticas, lo que confirma correcta integración entre el modelo dinámico y los sensores.

---

**[INSERTAR CAPTURA AQUÍ — Scope de velocidad lineal]**

**[INSERTAR CAPTURA AQUÍ — Scope de velocidad angular (gyro)]**

---

### 6.3 Estimación de pose y estado del vehículo

#### Señales analizadas

- `lidarPose [x, y, heading]`
- `currentPose [x, y, heading]`

Estas señales representan la estimación del estado completo del vehículo en el plano.

#### Método de visualización

Se emplearon dos estrategias:

1. Scope con opción “Elements as channels”.
2. Bloque Demux(3) para separar:
   - x (posición longitudinal)
   - y (posición lateral)
   - heading (orientación)

El rango angular se configuró entre −π y π rad para mantener coherencia física.

#### Validación

Se observó que:

- x e y evolucionan suavemente durante el desplazamiento.
- heading cambia progresivamente durante giros.
- No existen saltos erráticos o discontinuidades no físicas.

#### Resultado

El estado del vehículo es observable en tiempo real, lo que habilita integración futura con controladores basados en estado.

---

**[INSERTAR CAPTURA AQUÍ — Scope de posición X y Y]**

**[INSERTAR CAPTURA AQUÍ — Scope de heading]**

---

### 6.4 Monitoreo de telemetría eléctrica

#### Señales instrumentadas

- `batteryVoltage`
- `motorCurrent`
- `batteryLevel`
- `motorPower`

#### Configuración

Se utilizó un Scope con distribución 2×2 para monitoreo simultáneo.

Rangos definidos:

- Voltaje: 0–16 V
- Corriente: 0–30 A
- Nivel de batería: 0–100 %
- Potencia: 0–300 W

#### Validación

Se verificó que:

- La corriente aumenta bajo aceleración.
- La potencia se correlaciona con la demanda dinámica.
- El voltaje se mantiene dentro de rango estable.
- No aparecen valores negativos físicamente imposibles.

#### Resultado

La telemetría eléctrica es consistente y permite evaluar el consumo energético en función de la maniobra.

---

**[INSERTAR CAPTURA AQUÍ — Scope 2×2 de telemetría eléctrica]**

---

### 6.5 Intento de integración de cámara Depth/RGB

Se intentó habilitar el bloque `Video3D Capture` para obtener imágenes RGB y mapa de profundidad.

En ejecución en External Mode apareció el error:

> "video format is not supported"

Esto indica incompatibilidad entre el formato del dispositivo virtual y el modo de ejecución en tiempo real.

#### Decisión técnica

- Priorizar sensores críticos para navegación (LiDAR, pose, odometría).
- Posponer integración de cámara hasta estabilizar configuración o evaluar modo alternativo.

---

## 7. Estado técnico al cierre del Punto 1

Al finalizar esta etapa se logró:

- Confirmar recepción estable del LiDAR.
- Resolver errores por señales de tamaño variable.
- Validar coherencia cinemática (velocidad y giro).
- Observar el estado completo del vehículo en tiempo real.
- Monitorear variables eléctricas internas.
- Identificar limitaciones del módulo de cámara en External Mode.

El sistema sensorial virtual se encuentra completamente instrumentado y estable, constituyendo una base sólida para el desarrollo del módulo de control y navegación autónoma.