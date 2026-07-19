"""Parse sensor_msgs/Imu CDR fragments out of micro-ROS XRCE UDP payloads."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional


# XRCE submessage IDs (DDS-XRCE Table 6)
_XRCE_CREATE = 0x01
_XRCE_WRITE_DATA = 0x15
_XRCE_DATA = 0x16
_XRCE_READ_DATA = 0x17
_XRCE_HEARTBEAT = 0x0D
_XRCE_ACKNACK = 0x0E
_XRCE_TIMESTAMP = 0x11
_XRCE_TIMESTAMP_REPLY = 0x12

_FRAME_MARKERS = (
    b'imu_link',
    b'imu',
    b'base_link',
)


@dataclass
class ParsedImu:
    sec: int
    nanosec: int
    frame_id: str
    qx: float
    qy: float
    qz: float
    qw: float
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float


def xrce_summary(payload: bytes) -> str:
    """Short human summary of an XRCE datagram for diagnostics."""
    if len(payload) < 4:
        return f'len={len(payload)} <hdr'
    session = payload[0]
    stream = payload[1]
    seq = struct.unpack_from('<H', payload, 2)[0]
    has_key = session <= 0x7F
    off = 8 if has_key else 4
    subs = []
    while off + 4 <= len(payload) and len(subs) < 6:
        sid = payload[off]
        length = struct.unpack_from('<H', payload, off + 2)[0]
        subs.append(f'0x{sid:02X}({length})')
        off += 4 + length
        if length % 4:
            off += 4 - (length % 4)
    marker = 'imu_link' if b'imu_link' in payload else ('imu' if b'imu' in payload else '-')
    return (
        f'len={len(payload)} sess=0x{session:02X} stream=0x{stream:02X} '
        f'seq={seq} sub=[{",".join(subs) or "-"}] frame={marker}'
    )


def _parse_imu_at_frame(payload: bytes, idx: int, frame_id: str) -> Optional[ParsedImu]:
    if idx < 4:
        return None
    slen_off = idx - 4
    if slen_off < 8:
        return None
    try:
        slen = struct.unpack_from('<I', payload, slen_off)[0]
        if slen < 2 or slen > 64:
            return None
        stamp_off = slen_off - 8
        sec, nsec = struct.unpack_from('<iI', payload, stamp_off)
        str_end = slen_off + 4 + slen
        pad = (4 - (str_end % 4)) % 4
        ori_off = str_end + pad
        # orientation(4d) + cov(9d) + angular(3d) + cov(9d) + linear(3d)
        need = ori_off + 32 + 72 + 24 + 72 + 24
        if need > len(payload):
            return None
        qx, qy, qz, qw = struct.unpack_from('<dddd', payload, ori_off)
        av_off = ori_off + 32 + 72
        gx, gy, gz = struct.unpack_from('<ddd', payload, av_off)
        la_off = av_off + 24 + 72
        ax, ay, az = struct.unpack_from('<ddd', payload, la_off)
    except struct.error:
        return None

    # SIM settle/turns have a≈0; REAL rest has |a|≈g. Only reject absurd values.
    amag = (ax * ax + ay * ay + az * az) ** 0.5
    if amag > 80.0:
        return None

    return ParsedImu(
        sec=int(sec),
        nanosec=int(nsec),
        frame_id=frame_id,
        qx=float(qx),
        qy=float(qy),
        qz=float(qz),
        qw=float(qw),
        gx=float(gx),
        gy=float(gy),
        gz=float(gz),
        ax=float(ax),
        ay=float(ay),
        az=float(az),
    )


def try_parse_imu_cdr(payload: bytes) -> Optional[ParsedImu]:
    """Extract Imu fields from an XRCE UDP datagram if present."""
    for marker in _FRAME_MARKERS:
        start = 0
        while True:
            idx = payload.find(marker, start)
            if idx < 0:
                break
            # Prefer exact CDR string: uint32 len + c_str including NUL
            parsed = _parse_imu_at_frame(payload, idx, marker.decode('ascii'))
            if parsed is not None:
                return parsed
            start = idx + 1
    return None
