"""Parse sensor_msgs/Imu CDR fragments out of micro-ROS XRCE UDP payloads."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional


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


def try_parse_imu_cdr(payload: bytes) -> Optional[ParsedImu]:
    """Extract Imu fields from an XRCE UDP datagram if present."""
    idx = payload.find(b'imu_link')
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
        if ori_off + 32 + 72 + 24 + 72 + 24 > len(payload):
            return None
        qx, qy, qz, qw = struct.unpack_from('<dddd', payload, ori_off)
        av_off = ori_off + 32 + 72
        gx, gy, gz = struct.unpack_from('<ddd', payload, av_off)
        la_off = av_off + 24 + 72
        ax, ay, az = struct.unpack_from('<ddd', payload, la_off)
    except struct.error:
        return None

    return ParsedImu(
        sec=int(sec),
        nanosec=int(nsec),
        frame_id='imu_link',
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
