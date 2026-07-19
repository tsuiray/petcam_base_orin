"""Dead-reckon 2D pose from MPU6050 IMU using per-packet time deltas."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class ImuSample:
    stamp_sec: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


@dataclass
class OdomState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    path_xy: List[Tuple[float, float]] = field(default_factory=list)
    distance_m: float = 0.0


class ImuDeadReckoner:
    """Integrate gyro yaw + horizontal accel with measured dt between packets."""

    def __init__(
        self,
        *,
        gravity: float = 9.80665,
        accel_in_g: bool = False,
        enable_zupt: bool = True,
        zupt_accel_epsilon: float = 0.35,
        zupt_gyro_epsilon: float = 0.08,
        max_dt_sec: float = 0.1,
        min_dt_sec: float = 0.001,
        path_max_poses: int = 5000,
        calibrate_on_start_sec: float = 1.0,
    ) -> None:
        self.gravity = gravity
        self.accel_in_g = accel_in_g
        self.enable_zupt = enable_zupt
        self.zupt_accel_epsilon = zupt_accel_epsilon
        self.zupt_gyro_epsilon = zupt_gyro_epsilon
        self.max_dt_sec = max_dt_sec
        self.min_dt_sec = min_dt_sec
        self.path_max_poses = path_max_poses
        self.calibrate_on_start_sec = calibrate_on_start_sec

        self.state = OdomState()
        self._prev_stamp: Optional[float] = None
        self._bias_ax = 0.0
        self._bias_ay = 0.0
        self._calib_samples: List[Tuple[float, float]] = []
        self._calib_start: Optional[float] = None
        self._calibrated = calibrate_on_start_sec <= 0.0

    def reset(self) -> None:
        self.state = OdomState()
        self._prev_stamp = None
        self._bias_ax = 0.0
        self._bias_ay = 0.0
        self._calib_samples.clear()
        self._calib_start = None
        self._calibrated = self.calibrate_on_start_sec <= 0.0

    def update(self, sample: ImuSample) -> Optional[OdomState]:
        ax, ay, az = sample.ax, sample.ay, sample.az
        if self.accel_in_g:
            ax *= self.gravity
            ay *= self.gravity
            az *= self.gravity

        if not self._calibrated:
            if self._calib_start is None:
                self._calib_start = sample.stamp_sec
            self._calib_samples.append((ax, ay))
            if sample.stamp_sec - self._calib_start >= self.calibrate_on_start_sec:
                arr = np.asarray(self._calib_samples, dtype=np.float64)
                self._bias_ax = float(np.mean(arr[:, 0]))
                self._bias_ay = float(np.mean(arr[:, 1]))
                self._calibrated = True
                self._prev_stamp = sample.stamp_sec
                self.state.path_xy.append((0.0, 0.0))
            return None

        if self._prev_stamp is None:
            self._prev_stamp = sample.stamp_sec
            self.state.path_xy.append((0.0, 0.0))
            return self.state

        dt = sample.stamp_sec - self._prev_stamp
        self._prev_stamp = sample.stamp_sec
        if dt < self.min_dt_sec or dt > self.max_dt_sec:
            # Drop bad / out-of-order packets; keep stamp for next delta
            return self.state

        # Body-frame horizontal accel with simple bias removal
        ax_b = ax - self._bias_ax
        ay_b = ay - self._bias_ay

        # Integrate yaw from gyro z (rad/s)
        self.state.yaw = _wrap_pi(self.state.yaw + sample.gz * dt)
        c = math.cos(self.state.yaw)
        s = math.sin(self.state.yaw)

        # Rotate body accel into map/world (2D floor plane)
        ax_w = c * ax_b - s * ay_b
        ay_w = s * ax_b + c * ay_b

        gyro_norm = math.sqrt(sample.gx * sample.gx + sample.gy * sample.gy + sample.gz * sample.gz)
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        stationary = (
            self.enable_zupt
            and abs(accel_norm - self.gravity) < self.zupt_accel_epsilon
            and gyro_norm < self.zupt_gyro_epsilon
        )

        if stationary:
            self.state.vx = 0.0
            self.state.vy = 0.0
        else:
            self.state.vx += ax_w * dt
            self.state.vy += ay_w * dt

        dx = self.state.vx * dt
        dy = self.state.vy * dt
        step = math.hypot(dx, dy)
        self.state.x += dx
        self.state.y += dy
        self.state.distance_m += step
        self.state.path_xy.append((self.state.x, self.state.y))
        if len(self.state.path_xy) > self.path_max_poses:
            self.state.path_xy = self.state.path_xy[-self.path_max_poses :]

        return self.state


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
