#LIBRERIAS
import time
import numpy as np
import cv2
from datetime import datetime
from pal.products.qcar import QCar
from pal.utilities.lidar import Lidar


#Delay pre-inicialización
print("Iniciando en 5 segundos...")
#time.sleep(5)


#PARAMETROS
#Muestreo
SAMPLE_RATE = 50.0
DT = 1.0 / SAMPLE_RATE
#Movimiento (calibrado)
THROTTLE_CMD = 0.04       # PWM (sintonizado)
STEERING_BASE = 0.0        # recto
STEERING_TRIM  = -0.054
SPEED_PER_THROTTLE = 8.5 * 0.56   #(calibrado)
V_EST = SPEED_PER_THROTTLE * THROTTLE_CMD   # ~ m/s
DIST_TARGET_MAX = 20.0    # longitud máx del pasillo (m)
STOP_DISTANCE   = 0.5     # paro por obstáculo frontal (m)
#LIDAR
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

    mask = np.isfinite(d) & np.isfinite(a_bf) & (d > 0.03)
    return d[mask], a_bf[mask]

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
RES = 0.15               # m / celda
# Dimensiones del mapa (m)
MAP_LEN_M = 22.0        # largo del pasillo + margen
MAP_WID_M = 6.0         # ancho (4 m + márgenes)

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
L_FREE = -0.5    # incremento cuando vemos libre
L_MIN  = -5.0
L_MAX  = 5.0

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

def icp_translation_step(prev_pts, curr_pts, max_pairs=200):
    """
    Paso simple de 'ICP' SOLO TRASLACIÓN:
    - prev_pts: (N1, 2) puntos del scan anterior en marco global
    - curr_pts: (N2, 2) puntos del scan actual en marco global (pose PREDICHA)
    Regresa (dx, dy) que corrige la pose actual hacia la anterior.
    """
    if prev_pts is None or prev_pts.shape[0] < 10 or curr_pts.shape[0] < 10:
        return 0.0, 0.0

    P = prev_pts
    Q = curr_pts

    # Submuestreo para no hacerlo carísimo
    if P.shape[0] > max_pairs:
        idxP = np.random.choice(P.shape[0], max_pairs, replace=False)
        P = P[idxP]
    if Q.shape[0] > max_pairs:
        idxQ = np.random.choice(Q.shape[0], max_pairs, replace=False)
        Q = Q[idxQ]

    # Para cada punto en Q, encontrar vecino más cercano en P (brute force)
    # Q: (m,2), P: (n,2)
    # Expandimos: dist2[j,i] = ||Q[j] - P[i]||^2
    diff = Q[:, None, :] - P[None, :, :]   # (m, n, 2)
    dist2 = np.sum(diff**2, axis=2)        # (m, n)
    idx_min = np.argmin(dist2, axis=1)     # (m,)

    Q_match = Q                             # puntos actuales
    P_match = P[idx_min]                    # vecinos más cercanos en prev

    # Centroides
    cQ = np.mean(Q_match, axis=0)
    cP = np.mean(P_match, axis=0)

    # Traslación que queremos aplicar para alinear CURR con PREV:
    # P ≈ Q + [dx, dy]  => [dx, dy] ≈ cP - cQ
    dx, dy = (cP - cQ)

    return float(dx), float(dy)


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

def main():
    # QCar
    myCar = QCar(readMode=0)
    time.sleep(0.5)

    # LiDAR
    lidar = init_lidar()

    # LEDs
    LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

    # Pose inicial (mundo)
    x = 0.0
    y = 0.0
    theta = 0.0

    # Mapa de LOG-ODDS (inicialmente 0 = p=0.5)
    log_odds = np.zeros((MAP_NY, MAP_NX), dtype=np.float32)

    # Scan anterior en marco global (para ICP)
    prev_scan_world = None

    print("QCar SLAM (log-odds + ICP traslacional): avanzando y mapeando con LiDAR...")
    print(f"THROTTLE_CMD = {THROTTLE_CMD:.3f}, V_EST ≈ {V_EST:.3f} m/s")

    start_time = time.time()
    traveled_est = 0.0
    stop_reason = "None"

    # --- VideoWriter para guardar el mapa como video ---
    SCALE = 4  # el mismo que uses en la visualización
    base_h, base_w = log_odds.shape[:2]
    frame_size = (base_w * SCALE, base_h * SCALE)  # (width, height)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    video_filename = f"qcar_ogm_logodds_{timestamp}.mp4"
    video_out = cv2.VideoWriter(
        video_filename,
        fourcc,
        SAMPLE_RATE,              # fps aprox = SAMPLE_RATE
        frame_size                # (width, height)
    )

    try:
        while True:
            loop_start = time.time()

            # --- A) Movimiento / odometría: comando al QCar ---
            steering_cmd = STEERING_BASE + STEERING_TRIM
            myCar.read_write_std(
                throttle=THROTTLE_CMD,
                steering=steering_cmd,
                LEDs=LEDs
            )

            # Tiempo y odometría (avance recto)
            elapsed = time.time() - start_time
            traveled_est = V_EST * elapsed

            # PREDICCIÓN de pose (modelo simple recto)
            x_pred = traveled_est       # integrando v_est * t
            y_pred = y                  # mantenemos último y
            theta_pred = theta          # de momento 0

            # --- B) Percepción: LiDAR ---
            d, a_bf = read_scan(lidar)

            # Caso sin datos de LiDAR → seguir solo con odometría
            if d.size == 0:
                x, y, theta = x_pred, y_pred, theta_pred
                front_d = np.inf
            else:
                # Filtrar por rango máximo
                mask_r = d < MAX_LIDAR_R
                d_use = d[mask_r]
                a_use = a_bf[mask_r]

                if d_use.size == 0:
                    x, y, theta = x_pred, y_pred, theta_pred
                    front_d = np.inf
                else:
                    # B1) Scan actual en marco del ROBOT
                    xr = d_use * np.cos(a_use)
                    yr = -d_use * np.sin(a_use)   # eje lateral corregido

                    # B2) Scan actual en marco GLOBAL usando pose PREDICHA
                    cos_t = np.cos(theta_pred)
                    sin_t = np.sin(theta_pred)

                    xw_pred = x_pred + xr * cos_t - yr * sin_t
                    yw_pred = y_pred + xr * sin_t + yr * cos_t

                    curr_scan_world_pred = np.stack([xw_pred, yw_pred], axis=1)

                    # B3) CORRECCIÓN de pose con ICP traslacional
                    if prev_scan_world is not None:
                        dx_icp, dy_icp = icp_translation_step(prev_scan_world,
                                                              curr_scan_world_pred)
                    else:
                        dx_icp, dy_icp = 0.0, 0.0

                    # Pose corregida
                    x = x_pred + dx_icp
                    y = y_pred + dy_icp
                    theta = theta_pred  # por ahora sin corrección de yaw

                    # Recalcular trig
                    cos_t = np.cos(theta)
                    sin_t = np.sin(theta)

                    # B4) Scan actual en marco GLOBAL usando pose CORREGIDA
                    xw = x + xr * cos_t - yr * sin_t
                    yw = y + xr * sin_t + yr * cos_t

                    curr_scan_world = np.stack([xw, yw], axis=1)

                    # Guardar este scan para el próximo ICP
                    prev_scan_world = curr_scan_world.copy()

                    # Índice del robot en el mapa (con pose corregida)
                    rx_arr, ry_arr = world_to_map(np.array([x]), np.array([y]))
                    ix_r = int(rx_arr[0])
                    iy_r = int(ry_arr[0])

                    if not in_map(ix_r, iy_r):
                        stop_reason = "robot fuera del área del mapa"
                        break

                    # B5) Actualizar mapa con ray-casting y log-odds
                    for px, py in curr_scan_world:
                        ix_hit_arr, iy_hit_arr = world_to_map(
                            np.array([px]), np.array([py])
                        )
                        ix_h = int(ix_hit_arr[0])
                        iy_h = int(iy_hit_arr[0])

                        if not in_map(ix_h, iy_h):
                            continue

                        xs, ys = bresenham_line(ix_r, iy_r, ix_h, iy_h)
                        if xs.size < 1:
                            continue

                        # Celdas libres (todas menos la última)
                        if xs.size > 1:
                            free_x = xs[:-1]
                            free_y = ys[:-1]
                            valid_free = in_map(free_x, free_y)
                            free_x = free_x[valid_free]
                            free_y = free_y[valid_free]
                            log_odds[free_y, free_x] += L_FREE

                        # Celda ocupada (última)
                        occ_x = xs[-1]
                        occ_y = ys[-1]
                        if in_map(occ_x, occ_y):
                            log_odds[occ_y, occ_x] += L_OCC

                    # Saturar log-odds
                    log_odds = np.clip(log_odds, L_MIN, L_MAX)

                    # Distancia frontal usando scan sin filtrar (igual que antes)
                    front_d = get_front_distance(d, a_bf)

            # --- C) Condiciones de paro ---
            if traveled_est >= DIST_TARGET_MAX:
                stop_reason = "fin de pasillo (distancia máx)"
                break

            if d.size > 0 and front_d <= STOP_DISTANCE:
                stop_reason = f"obstáculo frontal a {front_d:.2f} m"
                break

            # --- D) Visualización del mapa + robot ---
            # Convertir log-odds a probabilidad de ocupado
            p_occ = 1.0 / (1.0 + np.exp(-log_odds))
            ogm_img = (1.0 - p_occ) * 255.0
            ogm_img = ogm_img.astype(np.uint8)

            # Dibujar triángulo del robot
            ogm_with_robot = draw_robot_triangle(ogm_img, x, y, theta)

            # Escalar
            h, w = ogm_with_robot.shape[:2]
            ogm_big = cv2.resize(
                ogm_with_robot,
                (w * SCALE, h * SCALE),
                interpolation=cv2.INTER_NEAREST
            )

            # Mostrar en ventana
            cv2.imshow("OGM global log-odds (x horizontal, y vertical)", ogm_big)
            cv2.waitKey(1)

            # Guardar frame en el video
            video_out.write(ogm_big)

            # Debug en terminal
            print(
                f"t={elapsed:4.2f}s | "
                f"x={x:5.2f}m | "
                f"dist_est={traveled_est:4.2f}m | "
                f"front_d={front_d:4.2f}m",
                end="\r"
            )

            # Respetar frecuencia
            comp = time.time() - loop_start
            sleep_t = DT - comp
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
        stop_reason = "KeyboardInterrupt"

    finally:
        # Detener QCar y terminar LiDAR / OpenCV
        try:
            myCar.read_write_std(
                throttle=0.0,
                steering=0.0,
                LEDs=np.zeros(8, dtype=int)
            )
        except Exception:
            pass

        try:
            lidar.terminate()
        except Exception:
            pass

        myCar.terminate()
        cv2.destroyAllWindows()
        video_out.release()
        print(f"Video guardado en: {video_filename}")

        total_time = time.time() - start_time
        print(f"\nFin. Motivo de paro: {stop_reason}")
        print(f"Tiempo total: {total_time:.2f}s | "
              f"dist_est final: {traveled_est:.2f}m")


if __name__ == "__main__":
    main()
