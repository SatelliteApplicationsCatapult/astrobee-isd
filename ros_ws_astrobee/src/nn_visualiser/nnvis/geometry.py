"""Rotation / transform helpers.  All batched over a leading time axis.

Quaternions are (x, y, z, w) throughout, matching geometry_msgs/Quaternion.
"""
import numpy as np


def quat_to_R(q):
    """(..., 4) xyzw -> (..., 3, 3).  Normalises defensively."""
    q = np.asarray(q, float)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y * y + z * z)
    R[..., 0, 1] = 2 * (x * y - z * w)
    R[..., 0, 2] = 2 * (x * z + y * w)
    R[..., 1, 0] = 2 * (x * y + z * w)
    R[..., 1, 1] = 1 - 2 * (x * x + z * z)
    R[..., 1, 2] = 2 * (y * z - x * w)
    R[..., 2, 0] = 2 * (x * z - y * w)
    R[..., 2, 1] = 2 * (y * z + x * w)
    R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def R_to_mat9(R):
    """(..., 3, 3) -> (..., 9) row-major: r00 r01 r02 r10 ...

    The observation carries the rotation matrix, not a quaternion.  q and -q
    are the same rotation, so a quaternion channel jumps by up to 2.0 in one
    step for an orientation that barely moved -- four such flips in the 46 s
    test bag.  Tracking sign continuity would fix the jump at the cost of
    making the observation history-dependent, which breaks the Markov
    property behavioural cloning relies on.  The matrix has no sign
    ambiguity at all: R(q) = R(-q).

    Redundant by six constraints (nine numbers, three degrees of freedom).
    The 6D form of Zhou et al. 2019 -- first two columns, third recovered by
    cross product -- is the non-redundant continuous alternative if the input
    width ever matters.

    Must stay byte-for-byte compatible with the training script, which does
    quaternion_matrix(q)[:3, :3].flatten().
    """
    m = np.asarray(R, float)
    return m.reshape(m.shape[:-2] + (9,))


def rot_apply(R, v):
    """R @ v, batched."""
    return np.einsum('nij,nj->ni', R, v)


def rot_apply_T(R, v):
    """R.T @ v, batched."""
    return np.einsum('nji,nj->ni', R, v)


def relative_twist(p_rob, R_wb, v_rob, w_rob, p_tool, v_tool, w_tool, R_wp):
    """Twist of the tool relative to perch_cam, expressed in perch_cam axes.

        w_rel = R_wp^T (w_tool - w_body)
        v_rel = R_wp^T (v_tool - v_body - w_body x (p_tool - p_body))

    The lever arm runs from the BODY origin, not from perch_cam.  This looks
    wrong and is not: the camera's own offset appears twice with opposite sign
    and cancels exactly --

        w x (p_pc - p_body) + w x (p_tool - p_pc) = w x (p_tool - p_body)

    Verified numerically on two independent bags by finite-differencing the
    tool pose in the perch frame.  The body-origin form is ~20x closer than
    the perch-origin form and ~50x closer than omitting the lever arm.
    Gazebo's ModelStates twist is reported at the link origin, not the centre
    of mass (a least-squares CoM fit returns zero), so no further correction
    is needed.
    """
    r = p_tool - p_rob
    v_rel_w = v_tool - v_rob - np.cross(w_rob, r)
    w_rel_w = w_tool - w_rob
    return rot_apply_T(R_wp, v_rel_w), rot_apply_T(R_wp, w_rel_w)
