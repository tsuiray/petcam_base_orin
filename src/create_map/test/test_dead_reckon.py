"""Unit tests for IMU dead-reckoning (no ROS runtime required)."""

from create_map.dead_reckon import ImuDeadReckoner, ImuSample


def test_stationary_calibration_then_forward_motion():
    dr = ImuDeadReckoner(
        calibrate_on_start_sec=0.2,
        enable_zupt=False,
        accel_in_g=False,
    )
    t = 0.0
    dt = 0.02
    # 0.2 s still for bias calib
    for _ in range(12):
        dr.update(ImuSample(t, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    # 1.0 s of +0.5 m/s^2 in x → v=0.5, x≈0.25
    for _ in range(50):
        state = dr.update(ImuSample(t, 0.5, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    assert state is not None
    assert state.x > 0.15
    assert abs(state.y) < 0.05
    assert state.distance_m > 0.15


def test_dt_uses_packet_time_gap():
    dr = ImuDeadReckoner(calibrate_on_start_sec=0.0, enable_zupt=False)
    dr.update(ImuSample(0.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
    # Exactly 20 ms gap as ESP32 cadence
    state = dr.update(ImuSample(0.02, 1.0, 0.0, 9.81, 0.0, 0.0, 0.0))
    assert state is not None
    # v = 1.0 * 0.02 = 0.02; x = 0.02 * 0.02 = 0.0004
    assert abs(state.vx - 0.02) < 1e-9
    assert abs(state.x - 0.0004) < 1e-9
