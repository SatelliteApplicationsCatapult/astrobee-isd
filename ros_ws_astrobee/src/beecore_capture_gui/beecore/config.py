"""Constants and persisted settings."""

import json
import logging
import math
import os
from dataclasses import asdict, dataclass, field
from typing import Dict, List

log = logging.getLogger('beecore_capture_gui')

# --- paths -------------------------------------------------------------------

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(PACKAGE_DIR)
ASSETS_DIR = os.path.join(PROJECT_DIR, 'assets')

# White artwork on the violet banner.
LOGO_FILE = 'SA_SM_White.png'

DEFAULT_SAVE_DIR = '/src/astrobee-isd/data/training/'
DEFAULT_MODELS_DIR = os.path.join(
    os.environ.get('CUSTOM_WS', '/src/custom_ws'), 'models')
CONFIG_PATH = os.environ.get(
    'BEECORE_GUI_CONFIG', os.path.expanduser('~/.beecore_capture_gui.json'))
# Read once if the new file does not exist yet, so a rename does not silently
# reset the camera offset, topic selection and experiment ID.
LEGACY_CONFIG_PATH = os.path.expanduser('~/.astrobee_data_gui.json')

# --- ros / server ------------------------------------------------------------

NODE_NAME = 'beecore_capture_gui'
BAG_NODE_NAME = 'beecore_bag_recorder'
GUI_PORT = int(os.environ.get('BEECORE_GUI_PORT',
                              os.environ.get('ASTROBEE_GUI_PORT', 8090)))
GUI_HOST = os.environ.get('BEECORE_GUI_HOST',
                          os.environ.get('ASTROBEE_GUI_HOST', '0.0.0.0'))

DEFAULT_NS = 'honey'

# --- capture outcome ---------------------------------------------------------

OUTCOME_NA = 'N/A'
OUTCOME_SUCCESS = 'SUCCESS'
OUTCOME_FAILURE = 'FAILURE'

# --- diagnostics -------------------------------------------------------------

STALE_AFTER_S = 2.0         # no message for this long -> amber
DIAG_POLL_S = 1.0

# ff_msgs/FaultState enum. Verified against
# communications/ff_msgs/msg/FaultState.msg
FAULT_STATES = {
    0: 'STARTING_UP',
    1: 'FUNCTIONAL',
    2: 'FAULT',
    3: 'BLOCKED',
    4: 'RELOADING_NODELETS',
}
FAULT_READY = 1             # FUNCTIONAL
FAULT_CLEAR_PUBLISH = 0     # what the GUI publishes to clear; transitions to 1

# --- follow_cam --------------------------------------------------------------
#
# The camera is a STANDALONE Gazebo model, not part of the robot description.
# Nothing here requires editing the Astrobee URDF.
#
# The offset is expressed in perch_cam axes, and the SDF puts the
# perch_cam -> Gazebo-camera rotation on the sensor, so these are the axes the
# operator sees:
#
#     +X down     +Y right     +Z forward (out of the lens)
#
# Rotation senses, right-hand rule about those axes:
#     pan  (+X)  positive swings the view LEFT
#     tilt (+Y)  positive pitches the view DOWN
#     roll (+Z)  positive rolls the scene clockwise

SDF_TEMPLATE = os.path.join(PROJECT_DIR, 'models', 'follow_cam.sdf.template')
CAM_MODEL_NAME = 'follow_cam'
CAM_FRAME = 'follow_cam'
PIN_RATE_HZ = 60

CAM_AXES = ('x', 'y', 'z', 'pan', 'tilt', 'roll')
CAM_LINEAR = ('x', 'y', 'z')
CAM_ROTARY = ('pan', 'tilt', 'roll')

# Step per click is operator-adjustable; these are the slider bounds.
CAM_STEP_MM_MIN, CAM_STEP_MM_MAX = 1, 30
CAM_STEP_DEG_MIN, CAM_STEP_DEG_MAX = 1, 30

# Generous, because nothing physical constrains a teleported model. These exist
# only to stop a stuck key sending the camera to the far side of the ISS.
CAM_TRAVEL_M = 3.0
CAM_ROT_LIMIT_RAD = math.pi

CAM_ZERO = {axis: 0.0 for axis in CAM_AXES}

CAM_FOV_MIN_DEG = 20
CAM_FOV_MAX_DEG = 200

# --- image viewer ------------------------------------------------------------
#
# web_video_server serves MJPEG over HTTP, and the GUI starts and stops it.
#
# Port 8091 to sit next to the GUI on 8090. NOT the web_video_server default of
# 8080: on this host 8080 belongs to Monitorix, a system monitoring daemon
# running outside the container.
#
#     8080  Monitorix      8090  this GUI      8091  video
#
# video_host is what the BROWSER resolves - the <img> src is fetched by the
# browser, not from inside the container - while video_port is also what the
# server is launched on, so the two can never drift apart.
VIDEO_HOST = os.environ.get('BEECORE_VIDEO_HOST', 'localhost')
VIDEO_PORT = int(os.environ.get('BEECORE_VIDEO_PORT', 8091))
VIEWER_WINDOW_NAME = 'beecore_view'    # named target: reused, never duplicated

# --- tools -------------------------------------------------------------------
#
# label -> (sdf path relative to models_dir, gazebo model name, link name)

TOOLS = {
    'Ratchet Wrench': ('tools/ratchet_wrench.sdf', 'ratchet_wrench', 'link'),
    '10 mm Wrench': ('tools/wrench_10mm.sdf', 'wrench_10mm', 'link'),
}

# Spawn envelope, expressed in the perch_cam frame. 0.5 m cube.
# perch_cam: +Z forward (out of the lens), +X down, +Y right.
# The near face stays at 0.2 m so nothing spawns inside the robot.
SPAWN_BOX = {'x': (-0.25, 0.25), 'y': (-0.25, 0.25), 'z': (0.2, 0.7)}

# Static transform body -> perch_cam, from:
#     rosrun tf tf_echo honey/body honey/perch_cam
PERCH_CAM_XYZ = (-0.133, 0.051, -0.017)
PERCH_CAM_QUAT = (0.0, -0.707, 0.0, 0.707)      # x y z w

# Perturbation: a short impulse, then the tool coasts.
IMPULSE_S = 0.1

# Slider bounds. The maxima are set by what stays useful in frame: at 0.5 N a
# 0.5 kg tool leaves at 0.1 m/s, which crosses the 0.5 m box in 5 s.
FORCE_MAX_N = 0.5
TORQUE_MAX_MNM = 10.0       # millinewton-metres, so the slider has sane steps

# Assumed tool properties, used only to show the operator roughly what a given
# force or torque will do. Nothing depends on these being right.
NOMINAL_TOOL_MASS_KG = 0.5
NOMINAL_TOOL_INERTIA = 1e-3

# How hard off-axis spawns are pushed back toward the perch_cam boresight.
# 0 = uniform random, 1 = fully biased inward. Applies to X and Y only.
CENTRING_BIAS = 0.7

# --- misc --------------------------------------------------------------------

LOG_LINES = 500
STOP_TIMEOUT_S = 20


@dataclass
class Settings:
    """Everything that survives a restart."""

    # storage
    save_dir: str = DEFAULT_SAVE_DIR
    models_dir: str = DEFAULT_MODELS_DIR

    # run metadata
    suffix: str = ''
    experiment_id: int = 1
    wanted_topics: List[str] = field(default_factory=list)
    record_all: bool = False
    buffer_mb: int = 1024
    split_mb: int = 0

    # robot
    robot_ns: str = DEFAULT_NS

    # follow_cam: live offset, plus the operator's saved "default" pose
    cam_offset: Dict[str, float] = field(
        default_factory=lambda: dict(CAM_ZERO))
    cam_defaults: Dict[str, float] = field(
        default_factory=lambda: dict(CAM_ZERO))
    cam_fov_deg: float = 90.0
    cam_step_mm: float = 5.0
    cam_step_deg: float = 1.0

    # image viewer
    video_host: str = VIDEO_HOST
    video_port: int = VIDEO_PORT
    video_autostart: bool = True
    stream_width: int = 1280
    stream_quality: int = 80
    preview_enabled: bool = False
    cam_width: int = 1920
    cam_height: int = 1080
    cam_rate_hz: float = 60.0

    # tools
    tool_label: str = 'Ratchet Wrench'
    force_axes: Dict[str, bool] = field(
        default_factory=lambda: {'x': True, 'y': True, 'z': True})
    torque_axes: Dict[str, bool] = field(
        default_factory=lambda: {'x': True, 'y': True, 'z': True})
    max_force_n: float = 0.15
    max_torque_mnm: float = 2.0

    def load(self, path: str = CONFIG_PATH) -> None:
        if not os.path.isfile(path):
            if os.path.isfile(LEGACY_CONFIG_PATH):
                log.info('Migrating settings from %s', LEGACY_CONFIG_PATH)
                self.load(LEGACY_CONFIG_PATH)
                self.save(path)
            return
        try:
            with open(path) as fh:
                data = json.load(fh)
            for key, value in data.items():
                if not hasattr(self, key):
                    continue
                try:
                    setattr(self, key, value)
                except AttributeError:
                    # A key from an older schema that is now a read-only
                    # property (video_base_url became host + port). Skip it -
                    # one stale key must never cost the whole config file.
                    log.info('Ignoring obsolete setting "%s".', key)
            # a new joint added to CAM_JOINTS must not KeyError on an old file
            for name, zero in CAM_ZERO.items():
                self.cam_offset.setdefault(name, zero)
                self.cam_defaults.setdefault(name, zero)
            log.info('Loaded settings from %s', path)
        except Exception as exc:                               # noqa: BLE001
            log.warning('Could not read %s (%s). Using defaults.', path, exc)

    def save(self, path: str = CONFIG_PATH) -> None:
        try:
            with open(path, 'w') as fh:
                json.dump(asdict(self), fh, indent=2)
        except OSError as exc:
            log.warning('Could not write %s (%s). Settings will not persist.',
                        path, exc)

    # --- namespaced names ----------------------------------------------------

    @property
    def ns(self) -> str:
        return (self.robot_ns or DEFAULT_NS).strip('/')

    @property
    def joint_prefix(self) -> str:
        return '{}/'.format(self.ns)

    def topic(self, tail: str) -> str:
        return '/{}/{}'.format(self.ns, tail.lstrip('/'))

    @property
    def image_topic(self) -> str:
        return '/{}/follow_cam/image_raw'.format(self.ns)

    @property
    def video_base_url(self) -> str:
        return 'http://{}:{}'.format(self.video_host or VIDEO_HOST,
                                     int(self.video_port))

    def stream_url(self, width: int = 0) -> str:
        """MJPEG URL for web_video_server. width=0 uses the configured value."""
        return '{}/stream?topic={}&width={}&quality={}'.format(
            self.video_base_url, self.image_topic,
            int(width or self.stream_width),
            int(self.stream_quality))

    @property
    def cam_step_m(self) -> float:
        return float(self.cam_step_mm) / 1000.0

    @property
    def cam_step_rad(self) -> float:
        return math.radians(float(self.cam_step_deg))

    @property
    def max_torque_nm(self) -> float:
        return float(self.max_torque_mnm) / 1000.0

    @property
    def tool(self):
        return TOOLS.get(self.tool_label, TOOLS['Ratchet Wrench'])


settings = Settings()
