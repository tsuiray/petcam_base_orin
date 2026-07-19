"""Python mirror of ESP32 ``imu_sim.cpp`` L-home lap (world-frame, fixed dt)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


# Same vertices as petcam_esp32_s3/main/imu_sim.cpp (~490 sq ft)
VERTICES: List[Tuple[float, float]] = [
    (0.0, 0.0),
    (8.0, 0.0),
    (8.0, 4.0),
    (4.5, 4.0),
    (4.5, 7.0),
    (0.0, 7.0),
]


@dataclass
class SimSample:
    ax: float
    ay: float
    gz: float
    yaw: float
    speed: float
    edge: int


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _dist(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)


def footprint_area_m2() -> float:
    s = 0.0
    n = len(VERTICES)
    for i in range(n):
        j = (i + 1) % n
        s += VERTICES[i][0] * VERTICES[j][1] - VERTICES[j][0] * VERTICES[i][1]
    return abs(s) * 0.5


def build_lap_samples(dt: float = 0.02) -> List[SimSample]:
    """Precompute one closed lap matching ESP32 imuSimBegin/buildLapBuffer."""
    out: List[SimSample] = []
    n_v = len(VERTICES)

    def push(ax: float, ay: float, gz: float, yaw: float, speed: float, edge: int) -> None:
        out.append(SimSample(ax, ay, gz, yaw, speed, edge))

    def append_edge(edge_idx: int, x0: float, y0: float, x1: float, y1: float, yaw: float) -> None:
        L = _dist(x0, y0, x1, y1)
        if L < 1e-3:
            return
        ux = (x1 - x0) / L
        uy = (y1 - y0) / L
        n = int(L / (0.5 * dt))
        n = max(20, min(400, n))
        a_mag = L / (float(n * n) * dt * dt)
        v = 0.0
        for _ in range(n):
            v += a_mag * dt
            push(a_mag * ux, a_mag * uy, 0.0, yaw, v, edge_idx)
        for _ in range(n):
            v -= a_mag * dt
            if v < 0.0:
                v = 0.0
            push(-a_mag * ux, -a_mag * uy, 0.0, yaw, v, edge_idx)
        for _ in range(3):
            push(0.0, 0.0, 0.0, yaw, 0.0, edge_idx)

    def append_turn(yaw0: float, yaw1: float, edge_idx: int) -> None:
        dyaw = _wrap_pi(yaw1 - yaw0)
        if abs(dyaw) < 1e-4:
            return
        turn_rate = 0.9
        t = abs(dyaw) / turn_rate
        n = int(t / dt + 0.5)
        n = max(10, min(300, n))
        gz = dyaw / (float(n) * dt)
        for i in range(1, n + 1):
            yaw = _wrap_pi(yaw0 + gz * float(i) * dt)
            push(0.0, 0.0, gz, yaw, 0.0, edge_idx)
        for _ in range(3):
            push(0.0, 0.0, 0.0, yaw1, 0.0, edge_idx)

    for e in range(n_v):
        i0, i1, i2 = e, (e + 1) % n_v, (e + 2) % n_v
        x0, y0 = VERTICES[i0]
        x1, y1 = VERTICES[i1]
        yaw = math.atan2(y1 - y0, x1 - x0)
        append_edge(e, x0, y0, x1, y1, yaw)
        yaw_next = math.atan2(VERTICES[i2][1] - y1, VERTICES[i2][0] - x1)
        append_turn(yaw, yaw_next, e)

    return out
