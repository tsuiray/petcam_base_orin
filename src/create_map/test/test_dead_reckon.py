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
    for _ in range(12):
        dr.update(ImuSample(t, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    for _ in range(50):
        state = dr.update(ImuSample(t, 0.5, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    assert state is not None
    assert state.x > 0.15
    assert abs(state.y) < 0.05
    assert state.distance_m > 0.15


def test_dt_uses_packet_time_gap():
    dr = ImuDeadReckoner(calibrate_on_start_sec=0.0, enable_zupt=False, path_min_step_m=0.0)
    dr.update(ImuSample(0.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
    state = dr.update(ImuSample(0.02, 1.0, 0.0, 9.81, 0.0, 0.0, 0.0))
    assert state is not None
    assert abs(state.vx - 0.02) < 1e-9
    assert abs(state.x - 0.0004) < 1e-9


def test_zupt_does_not_kill_coasting_immediately():
    """Constant-velocity coast has |a|≈g; instant ZUPT must not zero v every sample."""
    dr = ImuDeadReckoner(
        calibrate_on_start_sec=0.0,
        enable_zupt=True,
        zupt_hold_sec=0.35,
        path_min_step_m=0.0,
    )
    t = 0.0
    dt = 0.02
    # Accel pulse 0.2 s
    for _ in range(10):
        dr.update(ImuSample(t, 1.0, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    v_after_accel = dr.state.vx
    assert v_after_accel > 0.1
    # Coast 0.2 s with gravity-only accel (looks "still" to naive ZUPT)
    for _ in range(10):
        state = dr.update(ImuSample(t, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    assert state is not None
    assert state.vx > 0.05  # still coasting; not wiped
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
    for _ in range(15):  # 0.3 s still > hold
        state = dr.update(ImuSample(t, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0))
        t += dt
    assert state is not None
    assert state.zupt_active
    assert abs(state.vx) < 1e-9
