"""Small quaternion / transform helpers.

Plain math only - no numpy, no tf. Quaternions are (x, y, z, w) throughout,
matching geometry_msgs.
"""

import math
import random
from typing import Sequence, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def quat_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q: Quat, v: Sequence[float]) -> Vec3:
    """Rotate vector v by quaternion q."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quat_normalise(q: Quat) -> Quat:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return IDENTITY
    return (x / norm, y / norm, z / norm, w / norm)


def quat_to_rpy(q: Quat) -> Vec3:
    """Fixed-axis XYZ (roll, pitch, yaw), matching URDF and spawn_model."""
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


def random_quat(rng: random.Random = random) -> Quat:
    """Uniform over SO(3) - Shoemake's method.

    Sampling roll/pitch/yaw uniformly instead would cluster orientations near
    the poles, which is not what "entirely random" means.
    """
    u1, u2, u3 = rng.random(), rng.random(), rng.random()
    r1, r2 = math.sqrt(1.0 - u1), math.sqrt(u1)
    t1, t2 = 2.0 * math.pi * u2, 2.0 * math.pi * u3
    return (r1 * math.sin(t1), r1 * math.cos(t1),
            r2 * math.sin(t2), r2 * math.cos(t2))


def quat_from_axis_angle(axis: Sequence[float], angle: float) -> Quat:
    ax, ay, az = axis
    half = angle / 2.0
    sin_half = math.sin(half)
    return (ax * sin_half, ay * sin_half, az * sin_half, math.cos(half))


def compose(parent_xyz: Vec3, parent_q: Quat,
            child_xyz: Vec3, child_q: Quat) -> Tuple[Vec3, Quat]:
    """Chain two transforms: parent (*) child."""
    rotated = quat_rotate(parent_q, child_xyz)
    xyz = (parent_xyz[0] + rotated[0],
           parent_xyz[1] + rotated[1],
           parent_xyz[2] + rotated[2])
    return xyz, quat_normalise(quat_mul(parent_q, child_q))
