"""Dead-reckon 2D pose from IMU — body-frame (REAL) or world-frame (ESP32 SIM)."""

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
    dt: float = 0.0
    dt_raw: float = 0.0
    ax_raw: float = 0.0
    ay_raw: float = 0.0
    ax_body: float = 0.0
    ay_body: float = 0.0
    ax_world: float = 0.0
    ay_world: float = 0.0
    zupt_active: bool = False
    still_sec: float = 0.0
    path_appended: bool = False
    dt_clamped: bool = False


class ImuDeadReckoner:
    """Integrate horizontal accel (+ optional gyro yaw).

    accel_frame:
      - ``body``: REAL MPU6050 — rotate by yaw into map
      - ``world``: ESP32 SIM — ax/ay already world-frame; do not rotate
    use_fixed_dt:
      - True: each sample is exactly ``default_dt_sec`` (20 ms for ESP32 SIM)
      - False: dt from packet timestamps / receive times
    """

    def __init__(
        self,
        *,
        gravity: float = 9.80665,
        accel_in_g: bool = False,
        accel_frame: str = 'body',
        use_fixed_dt: bool = False,
        enable_zupt: bool = True,
        zupt_accel_epsilon: float = 0.45,
        zupt_gyro_epsilon: float = 0.15,
        zupt_hold_sec: float = 0.35,
        max_dt_sec: float = 0.05,
        min_dt_sec: float = 1e-4,
        default_dt_sec: float = 0.02,
        path_max_poses: int = 5000,
        path_min_step_m: float = 0.0005,
        calibrate_on_start_sec: float = 1.0,
    ) -> None:
        self.gravity = gravity
        self.accel_in_g = accel_in_g
        self.accel_frame = accel_frame if accel_frame in ('body', 'world') else 'body'
        self.use_fixed_dt = use_fixed_dt
        self.enable_zupt = enable_zupt
        self.zupt_accel_epsilon = zupt_accel_epsilon
        self.zupt_gyro_epsilon = zupt_gyro_epsilon
        self.zupt_hold_sec = zupt_hold_sec
        self.max_dt_sec = max_dt_sec
        self.min_dt_sec = min_dt_sec
        self.default_dt_sec = default_dt_sec
        self.path_max_poses = path_max_poses
        self.path_min_step_m = path_min_step_m
        self.calibrate_on_start_sec = calibrate_on_start_sec

        self.state = OdomState()
        self._prev_stamp: Optional[float] = None
        self._bias_ax = 0.0
        self._bias_ay = 0.0
        self._calib_samples: List[Tuple[float, float]] = []
        self._calib_start: Optional[float] = None
        self._calibrated = calibrate_on_start_sec <= 0.0
        self._still_sec = 0.0
        self.samples_integrated = 0
        self.samples_clamped_dt = 0

    @property
    def bias_xy(self) -> Tuple[float, float]:
        return self._bias_ax, self._bias_ay

    def reset(self) -> None:
        self.state = OdomState()
        self._prev_stamp = None
        self._bias_ax = 0.0
        self._bias_ay = 0.0
        self._calib_samples.clear()
        self._calib_start = None
        self._calibrated = self.calibrate_on_start_sec <= 0.0
        self._still_sec = 0.0
        self.samples_integrated = 0
        self.samples_clamped_dt = 0

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
                self.state.path_xy = [(0.0, 0.0)]
                self.state.ax_raw = ax
                self.state.ay_raw = ay
                self.state.path_appended = True
                return self.state
            return None

        if self._prev_stamp is None:
            self._prev_stamp = sample.stamp_sec
            if not self.state.path_xy:
                self.state.path_xy.append((0.0, 0.0))
            self.state.path_appended = True
            return self.state

        dt_raw = sample.stamp_sec - self._prev_stamp
        self._prev_stamp = sample.stamp_sec

        dt_clamped = False
        if self.use_fixed_dt:
            # ESP32 SIM: each published sample represents exactly default_dt_sec
            # of motion, independent of Wi-Fi / DDS receive jitter.
            dt = self.default_dt_sec
            if dt_raw < self.min_dt_sec:
                # Still integrate — sample is valid content, only stamp/receive
                # time collided. Do not drop (burst delivery is common).
                pass
        else:
            if dt_raw < self.min_dt_sec:
                self.state.dt = dt_raw
                self.state.dt_raw = dt_raw
                self.state.path_appended = False
                return self.state
            if dt_raw > self.max_dt_sec:
                dt = self.default_dt_sec
                dt_clamped = True
                self.samples_clamped_dt += 1
                self.state.vx = 0.0
                self.state.vy = 0.0
                self._still_sec = 0.0
            else:
                dt = dt_raw

        ax_b = ax - self._bias_ax
        ay_b = ay - self._bias_ay

        # Yaw from gyro (pose display); SIM also sends gz during turns.
        self.state.yaw = _wrap_pi(self.state.yaw + sample.gz * dt)
        c = math.cos(self.state.yaw)
        s = math.sin(self.state.yaw)

        if self.accel_frame == 'world':
            # ESP32 imu_sim: ax/ay are already map/world frame.
            ax_w = ax_b
            ay_w = ay_b
        else:
            ax_w = c * ax_b - s * ay_b
            ay_w = s * ax_b + c * ay_b

        gyro_norm = math.sqrt(
            sample.gx * sample.gx + sample.gy * sample.gy + sample.gz * sample.gz
        )
        horiz = math.hypot(ax_b, ay_b)

        if self.accel_frame == 'world':
            # SIM has az=0 (no gravity). Stillness = near-zero horizontal accel + gyro.
            sample_still = horiz < self.zupt_accel_epsilon and gyro_norm < self.zupt_gyro_epsilon
        else:
            accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
            sample_still = (
                abs(accel_norm - self.gravity) < self.zupt_accel_epsilon
                and gyro_norm < self.zupt_gyro_epsilon
                and horiz < self.zupt_accel_epsilon
            )

        if sample_still:
            self._still_sec += dt
        else:
            self._still_sec = 0.0

        # SIM corners publish a=0 settle samples; snap velocity quickly so laps overlap.
        zupt_hold = (
            min(self.zupt_hold_sec, 3.0 * self.default_dt_sec)
            if self.accel_frame == 'world'
            else self.zupt_hold_sec
        )
        zupt_active = self.enable_zupt and self._still_sec >= zupt_hold
        if zupt_active:
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
        self.samples_integrated += 1

        appended = False
        if not self.state.path_xy:
            self.state.path_xy.append((self.state.x, self.state.y))
            appended = True
        else:
            lx, ly = self.state.path_xy[-1]
            if math.hypot(self.state.x - lx, self.state.y - ly) >= self.path_min_step_m:
                self.state.path_xy.append((self.state.x, self.state.y))
                appended = True
                if len(self.state.path_xy) > self.path_max_poses:
                    self.state.path_xy = self.state.path_xy[-self.path_max_poses :]

        self.state.dt = dt
        self.state.dt_raw = dt_raw
        self.state.dt_clamped = dt_clamped
        self.state.ax_raw = ax
        self.state.ay_raw = ay
        self.state.ax_body = ax_b
        self.state.ay_body = ay_b
        self.state.ax_world = ax_w
        self.state.ay_world = ay_w
        self.state.zupt_active = zupt_active
        self.state.still_sec = self._still_sec
        self.state.path_appended = appended
        return self.state


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
