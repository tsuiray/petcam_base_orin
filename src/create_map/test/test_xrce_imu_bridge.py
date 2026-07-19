"""Unit test XRCE IMU CDR extraction used by xrce_imu_bridge."""

import struct

from create_map.xrce_imu_cdr import try_parse_imu_cdr, xrce_summary


def _build_fake_xrce_imu(ax=0.6, ay=0.0, az=9.80665) -> bytes:
    prefix = bytes.fromhex('81012E0407014401043C0005')
    stamp = struct.pack('<iI', 100, 0)
    frame = struct.pack('<I', 9) + b'imu_link\x00'
    pad = b'\x00' * ((4 - (len(frame) % 4)) % 4)
    orientation = struct.pack('<dddd', 0.0, 0.0, 0.0, 1.0)
    orient_cov = struct.pack('<9d', -1.0, *([0.0] * 8))
    gyro = struct.pack('<ddd', 0.0, 0.0, 0.0)
    gyro_cov = struct.pack('<9d', *([0.0] * 9))
    accel = struct.pack('<ddd', ax, ay, az)
    accel_cov = struct.pack('<9d', *([0.0] * 9))
    return prefix + stamp + frame + pad + orientation + orient_cov + gyro + gyro_cov + accel + accel_cov


def test_parse_imu_from_xrce_payload():
    raw = _build_fake_xrce_imu(0.6, 0.0, 9.81)
    msg = try_parse_imu_cdr(raw)
    assert msg is not None
    assert msg.frame_id == 'imu_link'
    assert abs(msg.ax - 0.6) < 1e-6
    assert abs(msg.az - 9.81) < 1e-6


def test_parse_rejects_unrelated_udp():
    assert try_parse_imu_cdr(b'hello world') is None


def test_xrce_summary_includes_length():
    raw = _build_fake_xrce_imu()
    s = xrce_summary(raw)
    assert 'len=' in s
    assert 'frame=imu_link' in s
