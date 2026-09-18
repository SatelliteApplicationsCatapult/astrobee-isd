"""Turn a rosbag into the arrays the visualiser plays back.

Reads the bag once into memory (the filtered bags are ~60 MB), derives the
13-element observation and the 7-element action, then resamples both onto a
uniform FPS grid.  Nothing here touches the display.
"""
import numpy as np

from . import bagread as br
from . import config as C
from .geometry import quat_to_R, R_to_mat9, rot_apply_T, relative_twist


class BagError(RuntimeError):
    pass


def _stamp(t):
    return t[0] + t[1] * 1e-9


def load(path, robot=C.DEFAULT_ROBOT, tool=C.DEFAULT_TOOL, log=print):
    bag = br.Bag(path)

    ts, p_rob, q_rob, v_rob, w_rob = [], [], [], [], []
    p_tool, q_tool, v_tool, w_tool = [], [], [], []
    jw_t, jw = [], []
    grip_t = None
    grip_source = None
    tf_static = []
    missing_robot = missing_tool = 0

    for topic, cid, data, t in bag.read_messages():
        stamp = _stamp(t)

        if topic == C.TOPIC_MODEL_STATES:
            names, poses, twists = br.model_states(data)
            try:
                i = names.index(robot)
            except ValueError:
                missing_robot += 1
                continue
            try:
                j = names.index(tool)
            except ValueError:
                missing_tool += 1
                continue
            ts.append(stamp)
            p_rob.append(poses[i][:3]); q_rob.append(poses[i][3:])
            v_rob.append(twists[i][:3]); w_rob.append(twists[i][3:])
            p_tool.append(poses[j][:3]); q_tool.append(poses[j][3:])
            v_tool.append(twists[j][:3]); w_tool.append(twists[j][3:])

        elif topic == C.TOPIC_JOY_WRENCH:
            _, _, wr = br.wrench_stamped(data)
            jw_t.append(stamp); jw.append(wr)

        elif topic == C.TOPIC_JOY and grip_source != 'goal':
            # buttons[0] is "A".  Rising edge = the operator's close command.
            # NOTE buttons[0], not axes[0] -- axes[0] is the left stick X.
            _, _, btns = br.joy(data)
            if len(btns) > C.GRIPPER_BUTTON and btns[C.GRIPPER_BUTTON]:
                if grip_t is None:
                    grip_t, grip_source = stamp, 'joy'

        elif topic == C.TOPIC_ARM_GOAL:
            _, cmd, _ = br.arm_action_goal(data)
            if cmd == C.ARM_GRIPPER_CLOSE:
                # The goal is published on the same timestamp as the button
                # press and is the more explicit signal, so it wins.
                grip_t, grip_source = stamp, 'goal'

        elif topic == '/tf_static':
            tf_static += br.tf_message(data)

    if not ts:
        raise BagError(
            'no usable %s messages: robot %r missing from %d, tool %r from %d'
            % (C.TOPIC_MODEL_STATES, robot, missing_robot, tool, missing_tool))
    if missing_tool:
        log('tool %r absent from %d model_states messages (not yet spawned)'
            % (tool, missing_tool))

    ts = np.array(ts)
    p_rob = np.array(p_rob); q_rob = np.array(q_rob)
    v_rob = np.array(v_rob); w_rob = np.array(w_rob)
    p_tool = np.array(p_tool); q_tool = np.array(q_tool)
    v_tool = np.array(v_tool); w_tool = np.array(w_tool)

    # gazebo publishes several messages per sim tick; keep the last of each
    keep = np.ones(len(ts), bool)
    keep[:-1] = np.diff(ts) > 1e-9
    ts, p_rob, q_rob, v_rob, w_rob, p_tool, q_tool, v_tool, w_tool = (
        a[keep] for a in (ts, p_rob, q_rob, v_rob, w_rob,
                          p_tool, q_tool, v_tool, w_tool))

    # ---- static body -> perch_cam ------------------------------------
    bp_xyz, bp_quat, tf_src = C.BODY_TO_PERCH_XYZ, C.BODY_TO_PERCH_QUAT, 'fallback'
    want = ('%s/body' % robot, '%s/perch_cam' % robot)
    for parent, child, tr, q, _ in tf_static:
        if (parent, child) == want:
            bp_xyz, bp_quat, tf_src = tr, q, '/tf_static'
            break
    log('body->perch_cam from %s: t=%s' % (tf_src, np.round(bp_xyz, 4)))

    R_bp = quat_to_R(np.array(bp_quat))
    R_wb = quat_to_R(q_rob)
    R_wt = quat_to_R(q_tool)
    R_wp = R_wb @ R_bp
    p_pc = p_rob + np.einsum('nij,j->ni', R_wb, np.asarray(bp_xyz, float))

    pos = rot_apply_T(R_wp, p_tool - p_pc)
    # Tool orientation in perch_cam, as the nine elements of the rotation
    # matrix.  Same channel order as the training script's
    # quaternion_matrix(q)[:3, :3].flatten() -- if these two ever disagree the
    # network is fed a permuted observation and nothing on screen says so.
    rot = R_to_mat9(np.einsum('nji,njk->nik', R_wp, R_wt))
    lin, ang = relative_twist(p_rob, R_wb, v_rob, w_rob,
                              p_tool, v_tool, w_tool, R_wp)

    obs = np.concatenate([pos, rot, lin, ang], 1)           # (N, 18)

    # ---- actions -----------------------------------------------------
    jw_t = np.array(jw_t); jw = np.array(jw) if len(jw) else np.zeros((0, 6))

    # ---- resample onto a uniform grid --------------------------------
    t0, t1 = ts[0], ts[-1]
    n = max(2, int((t1 - t0) * C.FPS))
    grid = t0 + np.arange(n) / C.FPS

    obs_r = obs[_nearest(ts, grid)]
    if len(jw_t):
        act_ft = jw[_nearest(jw_t, grid)]
    else:
        act_ft = np.zeros((n, 6))
        log('no %s in bag -- actual wrench row will read zero' % C.TOPIC_JOY_WRENCH)

    if grip_t is None:
        grip = np.zeros(n)
        log('no gripper close found -- gripper channel stays 0 throughout')
    else:
        grip = (grid >= grip_t).astype(float)
        log('gripper close at t+%.3f s (from %s), held thereafter'
            % (grip_t - t0, grip_source))

    act = np.concatenate([act_ft, grip[:, None]], 1)        # (n, 7)

    return {
        'obs': obs_r,
        'act': act,
        'times': grid - t0,
        'duration': grid[-1] - t0,
        'n_raw': len(ts),
        'grip_t': None if grip_t is None else grip_t - t0,
        'grip_source': grip_source,
    }


def _nearest(src, grid):
    """Index of the nearest src sample for each grid point (zero-order hold)."""
    i = np.searchsorted(src, grid)
    i = np.clip(i, 1, len(src) - 1)
    left = grid - src[i - 1]
    right = src[i] - grid
    return np.where(left <= right, i - 1, i)


def standardise(obs):
    """Per-channel mean/std over the whole bag.

    This is the reason the bag is read before playback starts.  Raw inputs
    span two orders of magnitude across channels -- position in metres,
    quaternion components in [-1, 1], velocities of order 0.02 -- so feeding
    them raw into randomly initialised weights gives either a dead layer or a
    saturated one.  The same statistics set the display ranges, which is why
    they are not hardcoded: the second test bag spins the tool 4x faster than
    the first, and fixed ranges tuned to one would clip the other.
    """
    mean = obs.mean(0)
    std = obs.std(0)
    std[std < 1e-6] = 1.0
    return mean, std
