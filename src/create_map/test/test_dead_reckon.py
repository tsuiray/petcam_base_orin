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
    state = None
    for _ in range(12):
        state = dr.update(ImuSample(t, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    assert state is not None  # calib completion returns origin
    for _ in range(50):
        state = dr.update(ImuSample(t, 0.5, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    assert state is not None
    assert state.x > 0.15
    assert abs(state.y) < 0.05
    assert state.distance_m > 0.15
    assert len(state.path_xy) > 1


def test_dt_uses_packet_time_gap():
    dr = ImuDeadReckoner(calibrate_on_start_sec=0.0, enable_zupt=False, path_min_step_m=0.0)
    dr.update(ImuSample(0.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
    state = dr.update(ImuSample(0.02, 1.0, 0.0, 9.81, 0.0, 0.0, 0.0))
    assert state is not None
    assert abs(state.vx - 0.02) < 1e-9
    assert abs(state.x - 0.0004) < 1e-9


def test_zupt_does_not_kill_coasting_immediately():
    dr = ImuDeadReckoner(
        calibrate_on_start_sec=0.0,
        enable_zupt=True,
        zupt_hold_sec=0.35,
        path_min_step_m=0.0,
    )
    t = 0.0
    dt = 0.02
    for _ in range(10):
        dr.update(ImuSample(t, 1.0, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    assert dr.state.vx > 0.1
    for _ in range(10):
        state = dr.update(ImuSample(t, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    assert state is not None
    assert state.vx > 0.05
    assert state.distance_m > 0.02
    assert not state.zupt_active


def test_zupt_engages_after_hold():
    dr = ImuDeadReckoner(
        calibrate_on_start_sec=0.0,
        enable_zupt=True,
        zupt_hold_sec=0.2,
        path_min_step_m=0.0,
    )
    t = 0.0
    dt = 0.02
    for _ in range(5):
        dr.update(ImuSample(t, 1.0, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    for _ in range(15):
        state = dr.update(ImuSample(t, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    assert state is not None
    assert state.zupt_active
    assert abs(state.vx) < 1e-9


def test_large_dt_is_clamped_not_dropped():
    """Regression: dropping dt>max left map stuck at points=1."""
    dr = ImuDeadReckoner(
        calibrate_on_start_sec=0.0,
        enable_zupt=False,
        max_dt_sec=0.05,
        default_dt_sec=0.02,
        path_min_step_m=0.0005,
    )
    dr.update(ImuSample(0.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
    # Simulate sparse DDS delivery: 0.5 s gap with real horizontal accel
    state = dr.update(ImuSample(0.5, 0.6, 0.0, 9.81, 0.0, 0.0, 0.0))
    assert state is not None
    assert state.dt_clamped
    assert abs(state.dt - 0.02) < 1e-9
    assert state.distance_m > 0.0 or abs(state.vx) > 0.0
    # Continue with normal cadence — path should grow
    t = 0.52
    for _ in range(40):
        state = dr.update(ImuSample(t, 0.6, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += 0.02
    assert len(state.path_xy) > 1
    assert state.distance_m > 0.01
