# Lidar_OGM_all_room.py
# Quanser QCar - Global Occupancy Grid Mapping
# LiDAR móvil con posición del robot y transformación al marco global

import time
import numpy as np
import cv2
from pal.utilities.lidar import Lidar

# --- Parámetros generales ---
SAMPLE_RATE = 15.0
SAMPLE_TIME = 1.0 / SAMPLE_RATE
RUNTIME_SEC = 120.0
MAX_DISTANCE_M = 5.9

# --- Parámetros del mapa global ---
MAP_SIZE_M = 20.0          # 20x20 metros
PIXELS_PER_METER = 40
MAP_SIDE = int(MAP_SIZE_M * PIXELS_PER_METER)
CENTER_OFFSET = MAP_SIDE // 2  # centro del mapa
ROBOT_RADIUS_M = 0.18

# Log-odds
def logit(p): return np.log(p / (1.0 - p))
def inv_logit(l): return 1.0 / (1.0 + np.exp(-l))

P_OCC, P_FREE = 0.9, 0.4
L_OCC, L_FREE = logit(P_OCC), logit(P_FREE)
L_MIN, L_MAX = -4.6, 4.6
L_DECAY = 1.0

# --- LiDAR settings ---
NUM_MEAS = 360
RANGING_MODE = 2
INTERP_MODE = 0

# --- Simulación de movimiento (reemplázalo por datos reales) ---
def get_robot_pose(t):
    """Simula una trayectoria (puedes sustituirla con odometría real)."""
    # Movimiento circular de 3 m de radio, 10 s por vuelta
    R = 3.0
    omega = 2 * np.pi / 60.0
    x = R * np.cos(omega * t)
    y = R * np.sin(omega * t)
    theta = omega * t + np.pi / 2  # orientado tangencialmente
    return x, y, theta

# --- Funciones geométricas ---
def lidar_to_body_frame(angles):
    return (-angles) + (np.pi / 2)

def to_global(x_l, y_l, x_r, y_r, theta_r):
    """Transforma coordenadas del LiDAR al marco global."""
    x_g = x_r + x_l * np.cos(theta_r) - y_l * np.sin(theta_r)
    y_g = y_r + x_l * np.sin(theta_r) + y_l * np.cos(theta_r)
    return x_g, y_g

def meters_to_pixels(x_m, y_m):
    px = (CENTER_OFFSET + x_m * PIXELS_PER_METER).astype(np.int32)
    py = (CENTER_OFFSET - y_m * PIXELS_PER_METER).astype(np.int32)
    return px, py

# --- LIDAR ---
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
    a = np.asarray(lidar.angles, dtype=np.float32)
    a_bf = lidar_to_body_frame(a)
    mask = np.isfinite(d) & np.isfinite(a_bf) & (d > 0.03)
    return d[mask], a_bf[mask]

# --- OGM update ---
def update_ogm(log_odds, px, py, hit_mask):
    """Actualiza celdas ocupadas/libres en el mapa global."""
    valid = (px >= 0) & (px < MAP_SIDE) & (py >= 0) & (py < MAP_SIDE)
    px, py = px[valid], py[valid]
    hit_mask = hit_mask[valid]

    log_odds[py, px] += (hit_mask * L_OCC) + (~hit_mask * L_FREE)
    np.clip(log_odds, L_MIN, L_MAX, out=log_odds)
    return log_odds

# --- Visualización ---
def render(log_odds, pose, fps):
    if L_DECAY < 1.0:
        log_odds *= L_DECAY

    prob = inv_logit(np.clip(log_odds, L_MIN, L_MAX))
    img8 = (255.0 * (1.0 - prob)).astype(np.uint8)
    gray = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)

    # Dibuja robot
    x_r, y_r, theta_r = pose
    px, py = meters_to_pixels(x_r, y_r)
    cv2.circle(gray, (px, py), int(ROBOT_RADIUS_M * PIXELS_PER_METER), (0, 0, 255), 2)
    # dirección
    hx = int(px + 15 * np.cos(theta_r))
    hy = int(py - 15 * np.sin(theta_r))
    cv2.line(gray, (px, py), (hx, hy), (255, 255, 255), 2)

    cv2.putText(gray, f"FPS: {fps:4.1f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return gray

# --- Main ---
def main():
    log_odds = np.zeros((MAP_SIDE, MAP_SIDE), dtype=np.float32)
    lidar = init_lidar()
    cv2.namedWindow("Global OGM", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Global OGM", 800, 800)

    t0 = time.time()
    last_t = t0
    fps = 0.0

    try:
        while (time.time() - t0) < RUNTIME_SEC:
            t = time.time() - t0
            x_r, y_r, theta_r = get_robot_pose(t)

            dist, ang = read_scan(lidar)
            if dist.size == 0:
                continue

            # Clip por rango
            d_clip = np.minimum(dist, MAX_DISTANCE_M)
            x_l = d_clip * np.cos(ang)
            y_l = d_clip * np.sin(ang)
            hit_mask = dist < (MAX_DISTANCE_M - 1e-3)

            # Transformar a global
            x_g, y_g = to_global(x_l, y_l, x_r, y_r, theta_r)
            px, py = meters_to_pixels(x_g, y_g)

            # Actualizar OGM
            log_odds = update_ogm(log_odds, px, py, hit_mask)

            # Visualización
            dt = time.time() - last_t
            fps = 0.9 * fps + 0.1 * (1.0 / max(dt, 1e-6)) if fps > 0 else 1.0 / dt
            img = render(log_odds, (x_r, y_r, theta_r), fps)
            cv2.imshow("Global OGM", img)

            key = cv2.waitKey(int(SAMPLE_TIME * 1000)) & 0xFF
            if key in (27, ord('q')):
                break
            last_t = time.time()

    except KeyboardInterrupt:
        print("User interrupted.")
    finally:
        try:
            lidar.terminate()
        except Exception:
            pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
