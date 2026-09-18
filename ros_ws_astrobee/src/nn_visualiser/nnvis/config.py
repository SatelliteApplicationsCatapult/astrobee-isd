"""All tunables in one place."""

# ---- ROS ---------------------------------------------------------------
TOPIC_MODEL_STATES = '/gazebo/model_states'
TOPIC_JOY_WRENCH = '/joy_wrench'
TOPIC_JOY = '/joy'
TOPIC_ARM_GOAL = '/honey/beh/arm/goal'

# Placeholders for the real estimator, once it exists.  When these topics are
# present in the bag they are used verbatim and the model_states path is
# skipped entirely (see dataset.py).
TOPIC_EST_POSE = '/estimated_pose'
TOPIC_EST_TWIST = '/estimated_twist'

GRIPPER_BUTTON = 0            # sensor_msgs/Joy buttons[0] == "A" on an Xbox pad
ARM_GRIPPER_CLOSE = 8         # ff_msgs/ArmGoal GRIPPER_CLOSE
ARM_GRIPPER_OPEN = 7          # ff_msgs/ArmGoal GRIPPER_OPEN

# Static honey/body -> honey/perch_cam, read from /tf_static when available.
# These are the fallback values if /tf_static is missing from the bag.
BODY_TO_PERCH_XYZ = (-0.1331, 0.0509, -0.0166)
BODY_TO_PERCH_QUAT = (0.0, -0.7071067811865476, 0.0, 0.7071067811865476)

DEFAULT_ROBOT = 'honey'
DEFAULT_TOOL = 'ratchet_wrench'

# ---- network -----------------------------------------------------------
# Orientation is carried as the nine elements of the rotation matrix,
# row-major, NOT as a quaternion.  q and -q are the same rotation, so a
# quaternion observation jumps by up to 2.0 in a single step for a rotation
# that barely moved; the training script drops the quaternion for the same
# reason and the two halves must agree channel for channel.
N_IN = 18                     # 3 pos + 9 rotation matrix + 3 lin vel + 3 ang vel
N_OUT = 7                     # fx fy fz tx ty tz gripper

IN_LABELS = ['px', 'py', 'pz',
             'r00', 'r01', 'r02',
             'r10', 'r11', 'r12',
             'r20', 'r21', 'r22',
             'vx', 'vy', 'vz',
             'wx', 'wy', 'wz']
OUT_LABELS = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz', 'grip']

# Stand-in only -- a loaded policy takes its shape from the weights file.
HIDDEN = (128, 64)
SEED = 20260824

# ---- playback ----------------------------------------------------------
FPS = 30.0

# ---- edge aggregation --------------------------------------------------
# Below this many connections a gap is drawn in full: every edge, no ribbon
# substrate.  Aggregation exists for legibility at 128x64 (10,304 edges), not
# for throughput -- canvas renders 10k fine.  A 32x32 gap is 1,024 edges, so
# pooling it into 4x4 blocks would be aggregation that does not aggregate.
DENSE_EDGE_MAX = 2000

# Layers at or below this size get one ribbon endpoint per neuron.
RIBBON_UNGROUPED_MAX = 16
RIBBON_TARGET_PER_GROUP = 8
RIBBON_GROUPS_MIN = 4
RIBBON_GROUPS_MAX = 16

# Bright individual edges are a true top-K of |a_i * W_ij| within each gap.
# A fraction of the gap's edge count, clamped -- a fixed threshold gave 280
# edges in the 128x64 gap and 9 in the 64x7 gap, which read as two different
# drawings.  Constant density per gap is what makes the picture legible.
EDGE_FRACTION = 0.05
EDGE_MIN = 20
EDGE_CAP = 220


def ribbon_groups(n):
    """How many ribbon endpoint blocks a layer of n neurons is pooled into."""
    if n <= RIBBON_UNGROUPED_MAX:
        return n
    g = int(round(n / float(RIBBON_TARGET_PER_GROUP)))
    return max(RIBBON_GROUPS_MIN, min(RIBBON_GROUPS_MAX, g))


# ---- canvas ------------------------------------------------------------
# Columns are laid out from the number of layers, not hard-coded, since the
# hidden stack is whatever the loaded policy happens to be.
COL_X0 = 136                  # input column
COL_GAP = 260                 # input -> hidden -> ... -> predicted
PRED_ACT_GAP = 125            # predicted -> actual
RIGHT_MARGIN = 105
CANVAS_W_MIN = 1180
CANVAS_W_MAX = 1720           # beyond this the gap is compressed instead

# Headers sit at COL_TOP-56 and their sub-captions at COL_TOP-40.  The clock
# occupies y=28 and the frame counter y=48, so COL_TOP below ~150 puts the
# column headings into the frame counter.
CANVAS_H = 840
COL_TOP = 168
COL_BOTTOM = 796


def column_x(n_layers):
    """(x positions, canvas width) for n_layers columns plus the actual strip.

    n_layers counts input + hidden + output, i.e. len(net.sizes).  The
    returned list has one more entry than that: the "actual" strip mirroring
    the output column.
    """
    gaps = n_layers - 1
    gap = COL_GAP
    width = COL_X0 + gap * gaps + PRED_ACT_GAP + RIGHT_MARGIN
    if width > CANVAS_W_MAX:
        gap = (CANVAS_W_MAX - COL_X0 - PRED_ACT_GAP - RIGHT_MARGIN) / float(gaps)
        width = CANVAS_W_MAX
    xs = [COL_X0 + gap * k for k in range(n_layers)]
    xs.append(xs[-1] + PRED_ACT_GAP)
    return [float(x) for x in xs], float(max(CANVAS_W_MIN, width))


PALETTE = {
    'bg': '#07080B',
    'grid': '#141821',
    'grey': '#2E3440',           # zero end of every ramp
    'electric': '#31D9FF',       # positive / firing
    'amber': '#FFA51F',          # negative
    'text': '#8A94A6',
    'text_bright': '#D9E2F0',
    'gripper_on': '#B45CFF',
    'warn': '#FF6B6B',
}
