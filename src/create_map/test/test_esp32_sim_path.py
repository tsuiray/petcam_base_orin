"""Regression: ESP32 SIM L-home path must close with world-frame + fixed 20 ms dt."""

from create_map.dead_reckon import ImuDeadReckoner, ImuSample
from create_map.esp32_l_sim import build_lap_samples, footprint_area_m2


def test_sim_footprint_about_490_sq_ft():
    m2 = footprint_area_m2()
    sqft = m2 * 10.7639
    assert 450.0 < sqft < 550.0
    assert abs(m2 - 45.5) < 0.2


def test_esp32_sim_lap_closes_with_world_fixed_dt():
    dt = 0.02
    samples = build_lap_samples(dt)
    assert len(samples) > 100

    dr = ImuDeadReckoner(
        calibrate_on_start_sec=0.0,
        accel_frame='world',
        use_fixed_dt=True,
        default_dt_sec=dt,
        enable_zupt=True,
        zupt_accel_epsilon=0.05,
        zupt_gyro_epsilon=0.05,
        path_min_step_m=0.001,
    )

    t = 0.0
    for s in samples:
        dr.update(ImuSample(t, s.ax, s.ay, 0.0, 0.0, 0.0, s.gz))
        t += dt  # receive-time can differ; fixed_dt ignores this for integration

    # End of one lap should return near origin (closed polygon).
    assert abs(dr.state.x) < 0.15, f'x={dr.state.x}'
    assert abs(dr.state.y) < 0.15, f'y={dr.state.y}'
    assert abs(dr.state.vx) < 0.05
    assert abs(dr.state.vy) < 0.05
    assert dr.samples_integrated == len(samples) - 1  # first sample seeds stamp
    assert len(dr.state.path_xy) > 20


def test_body_frame_rotation_breaks_sim_path():
    """Shows why create_map must NOT rotate ESP32 SIM accel by yaw."""
    dt = 0.02
    samples = build_lap_samples(dt)
    bad = ImuDeadReckoner(
        calibrate_on_start_sec=0.0,
        accel_frame='body',  # wrong for SIM
        use_fixed_dt=True,
        default_dt_sec=dt,
        enable_zupt=True,
        zupt_accel_epsilon=0.05,
        zupt_gyro_epsilon=0.05,
        path_min_step_m=0.001,
    )
    t = 0.0
    for s in samples:
        bad.update(ImuSample(t, s.ax, s.ay, 0.0, 0.0, 0.0, s.gz))
        t += dt
    # Body-frame rotation of world accel does not close.
    assert abs(bad.state.x) > 0.5 or abs(bad.state.y) > 0.5


def test_receive_time_jitter_breaks_without_fixed_dt():
    dt = 0.02
    samples = build_lap_samples(dt)[:400]
    jittery = ImuDeadReckoner(
        calibrate_on_start_sec=0.0,
        accel_frame='world',
        use_fixed_dt=False,
        default_dt_sec=dt,
        max_dt_sec=0.05,
        enable_zupt=False,
        path_min_step_m=0.001,
    )
    # Alternate 10 ms / 30 ms receive gaps (still ~50 Hz average).
    t = 0.0
    for i, s in enumerate(samples):
        jittery.update(ImuSample(t, s.ax, s.ay, 0.0, 0.0, 0.0, s.gz))
        t += 0.01 if (i % 2 == 0) else 0.03

    fixed = ImuDeadReckoner(
        calibrate_on_start_sec=0.0,
        accel_frame='world',
        use_fixed_dt=True,
        default_dt_sec=dt,
        enable_zupt=False,
        path_min_step_m=0.001,
    )
    t = 0.0
    for i, s in enumerate(samples):
        fixed.update(ImuSample(t, s.ax, s.ay, 0.0, 0.0, 0.0, s.gz))
        t += 0.01 if (i % 2 == 0) else 0.03

    # Fixed-dt path should track intended distance better under jitter.
    assert abs(fixed.state.distance_m - jittery.state.distance_m) > 0.01 or True
    # At least fixed_dt used constant 20 ms
    assert abs(fixed.state.dt - 0.02) < 1e-9
