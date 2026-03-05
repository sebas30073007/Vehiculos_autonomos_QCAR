import time
import numpy as np
import sys
sys.path.append('/home/nvidia/Documents/Quanser/libraries/python')
from pal.products.qcar import QCar

# Parámetros
SAMPLE_RATE = 50.0
DT = 1.0 / SAMPLE_RATE

# Movimiento durante el giro
THROTTLE_TURN = 0.055
STEERING_TURN = -0.40 - 0.03     # giro + trim
V_TURN_EST = 0.06 * 8.5 * 0.56 * 0.6   # velocidad aprox durante el giro
TURN_RATE_EST = -np.deg2rad(30)       # rad/s (giro a la derecha)
ANGLE_TARGET = -np.pi / 2.0           # -90°

def giro_90_derecha(myCar):
    """
    Ejecuta un giro de 90° a la derecha manteniendo un movimiento hacia adelante.
    Solo realiza el giro, sin fases rectas antes o después.
    """

    print("Iniciando giro de 90° a la derecha...")

    # Estado inicial
    theta = 0.0
    theta_start = theta

    try:
        while True:
            loop_start = time.time()

            # Comandos de giro
            myCar.read_write_std(
                throttle=THROTTLE_TURN,
                steering=STEERING_TURN,
                LEDs=np.array([0, 0, 0, 0, 0, 0, 1, 1])
            )

            # Integración aproximada del ángulo
            theta += TURN_RATE_EST * DT
            delta_theta = theta - theta_start

            # Condición de fin: -90 grados
            if delta_theta <= ANGLE_TARGET:
                print("\nGiro de 90° completado.")
                break

            # Debug
            print(
                f"\rθ = {np.rad2deg(theta):6.1f}°  "
                f"(faltan {np.rad2deg(ANGLE_TARGET - delta_theta):6.1f}°)",
                end=""
            )

            # Mantener frecuencia
            comp = time.time() - loop_start
            if DT - comp > 0:
                time.sleep(DT - comp)

    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")

    finally:
        # Stop seguro
        myCar.read_write_std(
            throttle=0.0,
            steering=0.0,
            LEDs=np.zeros(8, dtype=int)
        )
        print("\nQCar detenido después del giro.")


# Ejemplo de uso:
myCar = QCar(readMode=0)
giro_90_derecha(myCar)
