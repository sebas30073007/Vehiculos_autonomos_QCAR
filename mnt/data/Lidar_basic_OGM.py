# Lidar_basic_OGM.py
# Quanser QCar - LiDAR Occupancy Grid Mapping (log-odds)
# Requiere: pal.utilities.lidar (Quanser PAL), numpy, opencv-python
# Compatibilidad: Python 3.7+ (sin tipos "modernos")

import time
import numpy as np
import cv2
from pal.utilities.lidar import Lidar

# =========================
# Parámetros de ejecución
# =========================
SAMPLE_RATE       = 20.0            # Hz (sube/baja según desempeño)
SAMPLE_TIME       = 1.0 / SAMPLE_RATE
RUNTIME_SEC       = 60.0           # duración total
MAX_DISTANCE_M    = 8.9             # corte de alcance (m)
BEAM_STEP         = 1               # usa 1 para todos, 2 para cada 2, etc.

# Mapa (local) en metros y resolución
MAP_SIZE_M        = 18.0             # 8m x 8m
PIXELS_PER_METER  = 50              # 50 px por metro (~5 cm/celda)
MAP_SIDE          = int(MAP_SIZE_M * PIXELS_PER_METER)
SWAP_XY_ORDER     = False           # pon True si ves el mapa rotado

# Log-odds (probabilidades)
def logit(p):   return np.log(p / (1.0 - p))
def inv_logit(l): return 1.0 / (1.0 + np.exp(-l))

P_OCC   = 0.90                       # prob. ocupada por impacto
P_FREE  = 0.40                       # prob. libre por paso del rayo
L_OCC   = logit(P_OCC)               # ~ +2.197
L_FREE  = logit(P_FREE)              # ~ -0.405
L_MIN   = -4.6                       # clamp ~1%
L_MAX   =  4.6                       # clamp ~99%
L_DECAY = 1.00                       # <1.0 para "olvidar" lento hacia desconocido (0.995 recomendado si hay dinámicos)

# Visual
WINDOW_NAME       = "QCar - LiDAR OGM"
ROBOT_RADIUS_M    = 0.18

# LiDAR (RPLidar)
NUM_MEAS          = 360
RANGING_MODE      = 2
INTERP_MODE       = 0


# =========================
# Utilidades geométricas
# =========================
def to_body_frame(angles_rad):
    # Original: anglesInBodyFrame = lidar.angles * -1 + pi/2
    return (-angles_rad) + (np.pi / 2.0)

def meters_to_pixels(x_m, y_m):
    """Convierte coordenadas métricas al marco de imagen con robot en el centro."""
    px = (MAP_SIDE / 2.0 - x_m * PIXELS_PER_METER).astype(np.int32)
    py = (MAP_SIDE / 2.0 - y_m * PIXELS_PER_METER).astype(np.int32)
    np.clip(px, 0, MAP_SIDE - 1, out=px)
    np.clip(py, 0, MAP_SIDE - 1, out=py)
    return px, py

def bresenham(x0, y0, x1, y1):
    """Devuelve arrays (xs, ys) de la línea discreta entre dos píxeles (enteros)."""
    dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
    err = dx + dy
    xs, ys = [], []
    while True:
        xs.append(x0); ys.append(y0)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy; x0 += sx
        if e2 <= dx:
            err += dx; y0 += sy
    return np.asarray(xs, np.int32), np.asarray(ys, np.int32)


# =========================
# OGM: actualización por rayo
# =========================
def update_ogm_beam(log_odds, px_end, py_end, hit=True):
    """Actualiza el mapa en log-odds con un rayo: libres a lo largo, ocupada en el final si hit."""
    cx = cy = MAP_SIDE // 2
    xs, ys = bresenham(cx, cy, int(px_end), int(py_end))
    if xs.size == 0:
        return

    # celdas libres: todas menos la última
    if xs.size > 1:
        xf, yf = xs[:-1], ys[:-1]
        if SWAP_XY_ORDER:
            log_odds[xf, yf] = np.clip(log_odds[xf, yf] + L_FREE, L_MIN, L_MAX)
        else:
            log_odds[yf, xf] = np.clip(log_odds[yf, xf] + L_FREE, L_MIN, L_MAX)

    # celda ocupada: la última, solo si hubo impacto real
    if hit:
        xe, ye = xs[-1], ys[-1]
        if SWAP_XY_ORDER:
            log_odds[xe, ye] = np.clip(log_odds[xe, ye] + L_OCC, L_MIN, L_MAX)
        else:
            log_odds[ye, xe] = np.clip(log_odds[ye, xe] + L_OCC, L_MIN, L_MAX)


# =========================
# Render
# =========================
def render(log_odds, fps, n_rays):
    """Convierte log-odds -> prob -> imagen en escala de grises + HUD."""
    # decaimiento opcional
    if L_DECAY < 1.0:
        log_odds *= L_DECAY

    # Calcula probabilidad 0..1
    prob = inv_logit(np.clip(log_odds, L_MIN, L_MAX))

    # Escala de grises: negro = ocupado (prob alta), blanco = libre
    img8 = (255.0 * (1.0 - prob)).astype(np.uint8)
    gray = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)

    # Dibuja el robot
    center = (MAP_SIDE // 2, MAP_SIDE // 2)
    r_px = max(2, int(ROBOT_RADIUS_M * PIXELS_PER_METER))
    cv2.circle(gray, center, r_px, (0, 0, 255), 1)  # círculo rojo pequeño

    # HUD
    cv2.putText(gray, "FPS: %4.1f" % fps, (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(gray, "Rays: %3d  Res: %.0fpx/m  Size: %.1fm"
                % (n_rays, PIXELS_PER_METER, MAP_SIZE_M),
                (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return gray



# =========================
# Adquisición LiDAR
# =========================
def init_lidar():
    return Lidar(
        type='RPLidar',
        numMeasurements=NUM_MEAS,
        rangingDistanceMode=RANGING_MODE,
        interpolationMode=INTERP_MODE
    )

def read_scan(lidar):
    """Lee el LiDAR y devuelve distancias y ángulos en el marco del cuerpo."""
    lidar.read()
    d = np.asarray(lidar.distances, dtype=np.float32)
    a = np.asarray(lidar.angles,    dtype=np.float32)
    a_bf = to_body_frame(a)
    # Filtros básicos: finitos y positivos
    mask = np.isfinite(d) & np.isfinite(a_bf) & (d > 0.03)
    return d[mask], a_bf[mask]


# =========================
# Main
# =========================
def main():
    print("Sample Time: %.6f s" % SAMPLE_TIME)

    log_odds = np.zeros((MAP_SIDE, MAP_SIDE), dtype=np.float32)
    lidar = init_lidar()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 720, 720)

    t0 = time.time()
    last = t0
    fps_ma = 0.0

    try:
        while (time.time() - t0) < RUNTIME_SEC:
            loop_start = time.time()

            # Lectura LiDAR
            dist, ang = read_scan(lidar)
            if dist.size == 0:
                # sin datos -> renderiza y continúa
                vis = render(log_odds, fps_ma, 0)
                cv2.imshow(WINDOW_NAME, vis)
                cv2.waitKey(1)
                continue

            # endpoint en metros (si no hubo hit, usa MAX_DISTANCE_M)
            d_clip = np.minimum(dist, MAX_DISTANCE_M)
            x = d_clip * np.cos(ang)
            y = d_clip * np.sin(ang)
            px, py = meters_to_pixels(x, y)

            # Actualización OGM por rayo (submuestreo opcional)
            n_rays = 0
            for i in range(0, px.size, BEAM_STEP):
                hit = dist[i] < (MAX_DISTANCE_M - 1e-3)
                update_ogm_beam(log_odds, px[i], py[i], hit)
                n_rays += 1

            # FPS (media móvil)
            dt = loop_start - last
            last = loop_start
            inst_fps = 1.0 / max(dt, 1e-6)
            fps_ma = 0.9 * fps_ma + 0.1 * inst_fps if fps_ma > 0 else inst_fps

            # Render
            vis = render(log_odds, fps_ma, n_rays)
            cv2.imshow(WINDOW_NAME, vis)

            # Sincronización a SAMPLE_TIME
            compute = time.time() - loop_start
            sleep_time = SAMPLE_TIME - (compute % SAMPLE_TIME)
            ms = max(1, int(1000 * sleep_time))
            key = cv2.waitKey(ms) & 0xFF
            if key in (27, ord('q')):   # ESC o q
                break

    except KeyboardInterrupt:
        print("User interrupted!")
    finally:
        try:
            lidar.terminate()
        except Exception:
            pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
