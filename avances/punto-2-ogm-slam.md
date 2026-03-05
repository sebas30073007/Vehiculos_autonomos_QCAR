---
title: "OGM + SLAM"
parent: Avances
has_children: true
nav_order: 2
---

# P2 — OGM + SLAM (Mapeo)

Semana 2 (2 mar – 8 mar 2026). Construcción de un **Occupancy Grid Map (OGM)** global en tiempo real usando datos del RPLidar, con estimación de pose por odometría y corrección ICP traslacional.

## Resumen

- El QCar avanza de forma autónoma mientras mapea el entorno con el RPLidar.
- El mapa se representa como una grilla de log-odds: cada celda acumula evidencia de estar ocupada o libre.
- El algoritmo de ray casting (Bresenham) propaga esa evidencia a lo largo de cada rayo LiDAR.
- La corrección ICP traslacional reduce el drift entre frames consecutivos.
- El resultado se exporta como video `.mp4` con timestamp para evidencia auditable.

## Estado por modalidad

| Modalidad | Script principal | Estado | DoD |
|-----------|-----------------|--------|-----|
| [Físico](./punto-2-ogm-slam-fisico) | `qcar_slam.py` + variante `qcar_slam_L.py` | ⚡ Activo | Mapa estable + video guardado |
| [Virtual](./punto-2-ogm-slam-virtual) | `Lidar_basic_OGM.py` (referencia) | 📋 Planificando | OGM coherente con QLabs + pose real |

## Entregables del Punto 2

| Entregable | Físico | Virtual |
|------------|--------|---------|
| Script ejecutable | `qcar_slam.py` ✓ | Pendiente (ver TODOs) |
| Video del mapa generado | `qcar_ogm_logodds_*.mp4` | Pendiente |
| Captura del OGM final | `assets/images/p2_ogm_final.png` | `assets/images/p2_ogm_virtual.png` |
| Parámetros documentados | ✓ (ver página Físico) | Pendiente |
| Bitácora de incidencias | ✓ (sección debug) | Pendiente |

## Definition of Done (Punto 2 completo)

El Punto 2 se considera completado cuando:

1. El script físico genera un OGM reconocible del entorno real (paredes visibles, sin drift excesivo).
2. Se guarda y adjunta al menos un video de un run físico completo.
3. Se documenta un set de parámetros validados (throttle, trim, resolución).
4. La implementación virtual tiene al menos el TODO 4 completado (test estático en QLabs).
5. Esta bitácora refleja todas las incidencias encontradas y sus soluciones.

---

**Siguiente paso:** [P3 — Navegación](./punto-3-navegacion)
