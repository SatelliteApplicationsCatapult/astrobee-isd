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
HIDDEN = (128, 64)
N_IN = 13                     # 3 pos + 4 quat + 3 lin vel + 3 ang vel
N_OUT = 7                     # fx fy fz tx ty tz gripper
SEED = 20260824

IN_LABELS = ['px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw',
             'vx', 'vy', 'vz', 'wx', 'wy', 'wz']
OUT_LABELS = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz', 'grip']

# ---- playback ----------------------------------------------------------
FPS = 30.0

# ---- edge aggregation --------------------------------------------------
# Number of ribbon groups per layer.  Input and output layers are never
# grouped (every neuron gets its own ribbon endpoint).
RIBBON_GROUPS = {0: N_IN, 1: 16, 2: 8, 3: N_OUT}

# Bright individual edges are a true top-K of |a_i * W_ij| within each gap.
# A fraction of the gap's edge count, clamped -- a fixed threshold gave 280
# edges in the 128x64 gap and 9 in the 64x7 gap, which read as two different
# drawings.  Constant density per gap is what makes the picture legible.
EDGE_FRACTION = 0.05
EDGE_MIN = 20
EDGE_CAP = 220

# ---- canvas ------------------------------------------------------------
CANVAS_W = 1180
CANVAS_H = 760
COL_X = [110, 420, 720, 950, 1075]     # in, h1, h2, predicted, actual
COL_TOP = 96
COL_BOTTOM = 716

PALETTE = {
    'bg': '#07080B',
    'grid': '#141821',
    'grey': '#2E3440',           # zero end of every ramp
    'electric': '#31D9FF',       # positive / firing
    'amber': '#FFA51F',          # negative
    'text': '#8A94A6',
    'text_bright': '#D9E2F0',
    'gripper_on': '#B45CFF',
}
