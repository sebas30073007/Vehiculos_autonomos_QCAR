import time
import numpy as np
import cv2
from datetime import datetime

from pal.products.qcar import QCar
from pal.utilities.lidar import Lidar


# ----------------------------
# Parámetros de muestreo
# ----------------------------
SAMPLE_RATE = 50.0
DT = 1.0 / SAMPLE_RATE

# ----------------------------
# Movimiento (calibrado)
# ----------------------------
THROTTLE_CMD = 0.04       # PWM (ya probado)
STEERING_BASE = 0.0        # recto
STEERING_TRIM  = -0.054

SPEED_PER_THROTTLE = 8.5 * 0.56   # tu factor final calibrado
V_EST = SPEED_PER_THROTTLE * THROTTLE_CMD   # ~ m/s

DIST_TARGET_MAX = 20.0    # longitud máx del pasillo (m)
STOP_DISTANCE   = 0.5     # paro por obstáculo frontal (m)

# ----------------------------
# LIDAR
# ----------------------------
NUM_MEAS     = 360
RANGING_MODE = 2
INTERP_MODE  = 0
FRONT_DEG    = 15.0
FRONT_RAD    = np.deg2rad(FRONT_DEG)
MAX_LIDAR_R  = 6.0        # m, rango que nos interesa mapear

def to_body_frame(angles_rad):
    # misma convención que antes: lidar -> cuerpo
    return -angles_rad + (np.pi / 2.0)

def init_lidar():
    return Lidar(
        type='RPLidar',
        numMeasurements=NUM_MEAS,
        rangingDistanceMode=RANGING_MODE,
        interpolationMode=INTERP_MODE
    )

def read_scan(lidar):
    lidar.read()
    d = np.asarray(lidar.distances, dtype=np.float32)
    a = np.asarray(lidar.angles,    dtype=np.float32)
    a_bf = to_body_frame(a)

    # Filtrar lecturas inválidas y demasiado cercanas
    mask = np.isfinite(d) & np.isfinite(a_bf) & (d > MIN_LIDAR_R)
    d = d[mask]
    a_bf = a_bf[mask]

    return d, a_bf


def get_front_distance(distances, angles_bf):
    if distances.size == 0:
        return np.inf
    mask = (angles_bf > -FRONT_RAD) & (angles_bf < FRONT_RAD)
    if not np.any(mask):
        return np.inf
    d_front = distances[mask]
    d_front = d_front[(d_front > 0.03) & (d_front < MAX_LIDAR_R)]
    if d_front.size == 0:
        return np.inf
    return float(np.min(d_front))

# ----------------------------
# OGM (mapa global)
# ----------------------------

# Resolución
RES = 0.05               # m / celda
# Dimensiones del mapa (m)
MAP_LEN_M = 34.0        # largo del pasillo + margen
MAP_WID_M = 12.0         # ancho (4 m + márgenes)

MAP_NX = int(MAP_LEN_M / RES)   # celdas en x
MAP_NY = int(MAP_WID_M / RES)   # celdas en y

# Rango de coordenadas del mundo
X_MIN = 0.0
X_MAX = MAP_LEN_M
Y_MIN = -MAP_WID_M / 2.0   # -3 m
Y_MAX = MAP_WID_M / 2.0    # +3 m

def world_to_map(xw, yw):
    """
    Convierte coord. en mundo (x,y) a índices del mapa (ix,iy).
    x -> eje horizontal (columnas), y -> vertical (filas).
    """
    ix = ((xw - X_MIN) / RES).astype(int)
    iy = ((yw - Y_MIN) / RES).astype(int)
    return ix, iy

def in_map(ix, iy):
    return (ix >= 0) & (ix < MAP_NX) & (iy >= 0) & (iy < MAP_NY)

# ----------------------------
# NUEVO: log-odds y ray-casting
# ----------------------------

# Parámetros de log-odds
L_OCC  = 1.2     # incremento cuando vemos ocupado
L_FREE = -0.3    # un poco menos agresivo para libres
L_MIN  = -5.0
L_MAX  = 5.0

# Filtros LiDAR
MIN_LIDAR_R   = 0.15   # ignorar lecturas muy cercanas al sensor
MEDIAN_WIN    = 5      # ventana impar para filtro de mediana (en ángulo)


def bresenham_line(x0, y0, x1, y1):
    """
    Bresenham entre (x0,y0) y (x1,y1) en coordenadas de celdas enteras.
    Devuelve dos arrays (xs, ys) con todas las celdas de la línea.
    """
    x0 = int(x0)
    y0 = int(y0)
    x1 = int(x1)
    y1 = int(y1)

    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    xs = []
    ys = []

    x, y = x0, y0
    while True:
        xs.append(x)
        ys.append(y)
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy

    return np.array(xs, dtype=np.int32), np.array(ys, dtype=np.int32)

# Movimiento durante el giro
THROTTLE_TURN = 0.052
STEERING_TURN = -0.40 - 0.03     # giro + trim
V_TURN_EST = 0.06 * 8.5 * 0.56 * 0.6   # velocidad aprox durante el giro
TURN_RATE_EST = np.deg2rad(30)       # rad/s (giro a la derecha)
ANGLE_TARGET = np.pi / 2.0           # -90°

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

def draw_robot_triangle(ogm_gray, x, y, theta):
    """
    Dibuja un triángulo que representa al robot en el mapa OGM.
    ogm_gray: imagen en escala de grises (uint8)
    x, y, theta: pose del robot en coordenadas del mundo [m, m, rad]
    Devuelve una imagen BGR con el triángulo dibujado.
    """
    # Dimensiones aproximadas del robot (en metros)
    robot_len = 0.6   # largo (frente a atrás)
    robot_wid = 0.3   # ancho

    # Triángulo en marco del robot (x hacia delante, y hacia la izquierda)
    pts_robot = np.array([
        [ robot_len/2.0,    0.0              ],  # punta (frente)
        [-robot_len/2.0,   -robot_wid/2.0    ],  # trasera derecha
        [-robot_len/2.0,    robot_wid/2.0    ]   # trasera izquierda
    ], dtype=np.float32)

    # Rotación por theta y traslación a marco del mundo
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    R = np.array([[cos_t, -sin_t],
                  [sin_t,  cos_t]], dtype=np.float32)

    pts_world = (pts_robot @ R.T) + np.array([x, y], dtype=np.float32)

    # Convertir cada vértice a índices de mapa
    xs = pts_world[:, 0]
    ys = pts_world[:, 1]
    ix, iy = world_to_map(xs, ys)

    # Mantener solo los vértices dentro del mapa
    valid = in_map(ix, iy)
    if np.count_nonzero(valid) < 3:
        # Muy fuera del mapa o pose rara → no dibujamos nada
        ogm_color = cv2.cvtColor(ogm_gray, cv2.COLOR_GRAY2BGR)
        return ogm_color

    ixv = ix[valid]
    iyv = iy[valid]

    # OpenCV espera puntos como (col, fila) = (x, y) en pixeles
    pts_pix = np.stack([ixv, iyv], axis=1).astype(np.int32)

    # Convertir a BGR para dibujar en color
    ogm_color = cv2.cvtColor(ogm_gray, cv2.COLOR_GRAY2BGR)

    # Rellenar el triángulo en rojo
    cv2.fillConvexPoly(ogm_color, pts_pix, (0, 0, 255))
    cv2.polylines(ogm_color, [pts_pix], isClosed=True, color=(0, 0, 0), thickness=1)

    return ogm_color


def median_filter_1d(x, win=5):
    """
    Filtro de mediana 1D muy simple para suavizar las distancias LiDAR.
    Preserva el orden de las muestras (por ángulo).
    """
    x = np.asarray(x, dtype=np.float32)
    n = x.size
    if n == 0 or win <= 1:
        return x

    win = int(win)
    if win % 2 == 0:
        win += 1
    half = win // 2

    x_filt = np.empty_like(x)
    for i in range(n):
        i0 = max(0, i - half)
        i1 = min(n, i + half + 1)
        x_filt[i] = np.median(x[i0:i1])
    return x_filt



def main():
    # QCar
    myCar = QCar(readMode=0)
    time.sleep(0.5)

    # LiDAR
    lidar = init_lidar()

    # LEDs
    LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

    # Pose inicial
    x = 0.0
    y = 0.0
    theta = 0.0

    # Mapa de log-odds
    log_odds = np.zeros((MAP_NY, MAP_NX), dtype=np.float32)

    print("QCar OGM: avanzando, mapeando y con UN SOLO giro permitido.")

    start_time = time.time()
    last_time = start_time
    traveled_est = 0.0
    stop_reason = "None"

    # -------- video (creación diferida hasta primer frame) --------
    video_out = None
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    video_filename = f"qcar_ogm_logodds_{timestamp}.mp4"


    # -------- flag para permitir SOLO UN giro --------
    # -------- flags --------
    modo = "RECTO"
    ya_giro = False
    theta_obj = None

    try:
        loop_counter = 0

        while True:
            loop_counter += 1
            loop_start = time.time()

            # Tiempo real
            now = time.time()
            dt = now - last_time
            last_time = now
            elapsed = now - start_time

            # ------------------------------------------------
            #   MODO RECTO (default)
            # ------------------------------------------------
            if modo == "RECTO":
                throttle_cmd = THROTTLE_CMD
                steering_cmd = STEERING_BASE + STEERING_TRIM
                v = V_EST
                omega = 0.0

            # ------------------------------------------------
            #   MODO GIRO (arco)
            # ------------------------------------------------
            elif modo == "GIRO":
                throttle_cmd = THROTTLE_TURN
                steering_cmd = STEERING_TURN
                v = V_TURN_EST
                omega = TURN_RATE_EST   # rad/s (negativo)

            # Aplicar movimiento al QCar
            myCar.read_write_std(
                throttle=throttle_cmd,
                steering=steering_cmd,
                LEDs=LEDs
            )

            # Actualizar pose con cinemática diferencial
            x += v * dt * np.cos(theta)
            y += v * dt * np.sin(theta)
            theta += omega * dt

            traveled_est += abs(v * dt)

            # ------------------------------------------------
            #   Percepción
            # ------------------------------------------------
            d, a_bf = read_scan(lidar)
            front_d = get_front_distance(d, a_bf)

            if loop_counter % 20 == 0:
                print(f"[DBG] modo={modo} | front_d={front_d:.2f} m | pose=({x:.2f}, {y:.2f}, {np.rad2deg(theta):.1f}°)")

            # ------------------------------------------------
            #   Paro duro
            # ------------------------------------------------
            if front_d <= STOP_DISTANCE:
                stop_reason = f"obstáculo MUY cerca ({front_d:.2f} m)"
                print("\n[STOP] <0.5 m.")
                break

            # ------------------------------------------------
            #   DETONAR GIRO (solo 1 vez)
            # ------------------------------------------------
            if modo == "RECTO" and not ya_giro and front_d <= 2.0:
                print(f"\n[EVENT] pared a {front_d:.2f} m → iniciar giro único")

                ya_giro = True
                modo = "GIRO"

                # Definir ángulo objetivo global
                theta_inicio = theta
                theta_obj = theta_inicio + ANGLE_TARGET  # -90°

                continue

            # ------------------------------------------------
            #   TERMINAR GIRO
            # ------------------------------------------------
            if modo == "GIRO":
                if theta >= theta_obj:   # giro horario
                    print("\n[INFO] Giro completado.")
                    theta = theta_obj    # corrección numérica
                    modo = "RECTO"       # volver a avanzar recto
                    last_time = time.time()
                    continue

            # ------------------------------------------------
            #   Límite de distancia
            # ------------------------------------------------
            if traveled_est >= DIST_TARGET_MAX:
                stop_reason = "distancia máxima"
                print("[STOP] distancia alcanzada.")
                break

            # ------------------------------------------------
            #   OGM + Ray casting
            # ------------------------------------------------
            if d.size > 0:
                # Rango útil: [MIN_LIDAR_R, MAX_LIDAR_R)
                mask_r = (d > MIN_LIDAR_R) & (d < MAX_LIDAR_R)
                d_use = d[mask_r]
                a_use = a_bf[mask_r]

                # Filtro de mediana en las distancias para limpiar outliers
                d_use = median_filter_1d(d_use, win=MEDIAN_WIN)

                cos_t = np.cos(theta)
                sin_t = np.sin(theta)

                rx_arr, ry_arr = world_to_map(np.array([x]), np.array([y]))
                ix_r = int(rx_arr[0]); iy_r = int(ry_arr[0])

                if in_map(ix_r, iy_r):
                    for r, ang in zip(d_use, a_use):
                        # Coordenadas en marco del robot
                        xr = r * np.cos(ang)
                        yr = -r * np.sin(ang)

                        # Transformar a marco del mundo
                        xw = x + xr * cos_t - yr * sin_t
                        yw = y + xr * sin_t + yr * cos_t

                        ix_h_arr, iy_h_arr = world_to_map(np.array([xw]), np.array([yw]))
                        ix_h = int(ix_h_arr[0]); iy_h = int(iy_h_arr[0])

                        if not in_map(ix_h, iy_h):
                            continue

                        # Ray casting
                        xs, ys = bresenham_line(ix_r, iy_r, ix_h, iy_h)
                        if xs.size == 0:
                            continue

                        # Celdas libres (todas menos la última)
                        if xs.size > 1:
                            free_x = xs[:-1]
                            free_y = ys[:-1]
                            valid = in_map(free_x, free_y)
                            log_odds[free_y[valid], free_x[valid]] += L_FREE

                        # Celda ocupada
                        log_odds[iy_h, ix_h] += L_OCC

                log_odds = np.clip(log_odds, L_MIN, L_MAX)


            # ------------------------------------------------
            #   Visualización (mapa grande)
            # ------------------------------------------------
            SCALE = 3  # puedes subirlo a 4 si quieres aún más grande

            p_occ = 1.0 / (1.0 + np.exp(-log_odds))
            ogm_img = ((1.0 - p_occ) * 255).astype(np.uint8)
            ogm_robot = draw_robot_triangle(ogm_img, x, y, theta)

            h, w = ogm_robot.shape[:2]
            ogm_big = cv2.resize(ogm_robot, (w * SCALE, h * SCALE), interpolation=cv2.INTER_NEAREST)

            cv2.imshow("OGM Global", ogm_big)
            cv2.waitKey(1)

            # Crear el VideoWriter SOLO cuando ya tengamos el primer frame correcto
            if video_out is None:
                h, w = ogm_big.shape[:2]
                video_out = cv2.VideoWriter(video_filename, fourcc, SAMPLE_RATE, (w, h))


            video_out.write(ogm_big)

            # Frecuencia
            comp = time.time() - loop_start
            if DT - comp > 0:
                time.sleep(DT - comp)


    except KeyboardInterrupt:
        stop_reason = "KeyboardInterrupt"
        print("\nInterrumpido por usuario.")

    finally:
        myCar.read_write_std(throttle=0.0, steering=0.0, LEDs=np.zeros(8, dtype=int))
        try: lidar.terminate()
        except: pass
        myCar.terminate()
        cv2.destroyAllWindows()
        video_out.release()

        total_time = time.time() - start_time
        print(f"\nFIN | razón: {stop_reason}")
        print(f"tiempo: {total_time:.2f}s | dist={traveled_est:.2f}m")


if __name__ == "__main__":
    main()
