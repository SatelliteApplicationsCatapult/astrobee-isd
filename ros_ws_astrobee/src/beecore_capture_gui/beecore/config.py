"""Constants and persisted settings."""

import json
import logging
import math
import os
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

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

# Four independent cameras, each its own Gazebo model with its own offset,
# optics and image topic. They are all pinned from ONE subscriber - the robot
# pose is read once per /gazebo/model_states message and reused for all four,
# so going from one camera to four costs four extra ModelState publishes per
# message and nothing else on the ROS side.
#
#     model / frame   follow_cam_1 .. follow_cam_4
#     image topic     /<ns>/follow_cam_<n>/image_raw
#
# The rendering is not free: four sensors is four Gazebo render passes and
# four JPEG encoders. Resolution is the knob that matters - see CAM_RESOLUTIONS.
CAM_COUNT = 4
CAM_PREFIX = 'follow_cam'


def cam_model_name(index: int) -> str:
    """Gazebo model name and TF frame for camera `index` (0-based)."""
    return '{}_{}'.format(CAM_PREFIX, int(index) + 1)

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

# Sensor resolution presets. Baked into the SDF at spawn, so changing one
# respawns that camera. These are viewers, not recorded sources, so the only
# cost of a higher setting is render and encode time - but with four cameras
# that cost is paid four times.
CAM_RESOLUTIONS = {
    '320 x 240  (QVGA)': (320, 240),
    '640 x 480  (VGA)': (640, 480),
    '800 x 600  (SVGA)': (800, 600),
    '1280 x 720  (720p)': (1280, 720),
    '1920 x 1080  (1080p)': (1920, 1080),
    '2560 x 1440  (1440p)': (2560, 1440),
    '3840 x 2160  (4K)': (3840, 2160),
}

def default_camera(index: int) -> dict:
    """A fresh camera entry.

    Camera 1 keeps what the single-camera build used, so an existing setup
    behaves exactly as before. Cameras 2-4 start at VGA/30 Hz rather than
    1080p/60: four 1080p sensors at 60 Hz is a large step up in render load to
    take without being asked. Raise them on the Camera tab if the machine
    copes.
    """
    first = (index == 0)
    return {
        'label': 'Camera {}'.format(index + 1),
        'offset': dict(CAM_ZERO),
        'defaults': dict(CAM_ZERO),
        'fov_deg': 90.0,
        'width': 1920 if first else 640,
        'height': 1080 if first else 480,
        'rate_hz': 60.0 if first else 30.0,
    }


CAMERA_KEYS = tuple(default_camera(0).keys())

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

# Spawn envelope, expressed in the perch_cam frame. A 0.5 m cube: each axis
# spans 0.5 m, so the tool lands at most 0.25 m off the boresight.
# perch_cam: +Z forward (out of the lens), +X down, +Y right.
# The near face stays at 0.2 m so nothing spawns inside the robot.
SPAWN_BOX = {'x': (-0.25, 0.25), 'y': (-0.25, 0.25), 'z': (0.2, 0.7)}

# Static transform body -> perch_cam, from:
#     rosrun tf tf_echo honey/body honey/perch_cam
PERCH_CAM_XYZ = (-0.133, 0.051, -0.017)
PERCH_CAM_QUAT = (0.0, -0.707, 0.0, 0.707)      # x y z w

# Perturbation: a short impulse, then the tool coasts.
IMPULSE_S = 0.1

# Tool properties, read off tools/ratchet_wrench.sdf: mass 1.0 kg, diagonal
# inertia 0.083 kg.m^2 on all three axes.
#
# These are not decoration - the slider maxima below are derived from them, and
# the hint under each slider is computed from them. The earlier values (0.5 kg,
# 1e-3 kg.m^2) were guesses, and the inertia guess was 83x too small, which is
# why torque appeared to do nothing: at the old 10 mNm maximum the tool span up
# at 0.69 deg/s. The wrench WAS being applied; it was just imperceptible.
#
# Note 0.083 = 1/12, i.e. a 1 m cube - the placeholder from the Gazebo inertia
# tutorial rather than a wrench. A real ~0.3 m wrench is nearer 0.008, which
# would spin ten times faster for the same torque. If you replace the dummy
# inertia in the SDF, halve these maxima.
NOMINAL_TOOL_MASS_KG = 1.0
NOMINAL_TOOL_INERTIA = 0.083

# Slider bounds, chosen as "what still stays in frame":
#     F = m*v/dt   ->  3.0 N  gives 0.30 m/s, crossing the 0.5 m box in 1.7 s
#     T = I*w/dt   ->  6.0 Nm gives 7.23 rad/s = 414 deg/s
# Torque is in NEWTON-metres now, not milli: at this inertia a mNm slider could
# not reach anything visible even at its top stop.
FORCE_MAX_N = 3.0
TORQUE_MAX_NM = 6.0

# How hard off-axis spawns are pushed back toward the perch_cam boresight.
# 0 = uniform random, 1 = fully biased inward. Applies to X and Y only.
CENTRING_BIAS = 0.7

# --- simulation reset --------------------------------------------------------
#
# The robot is teleported back to a fixed home pose, then the system monitor's
# fault state is cleared. Both were verified by hand before being wired in:
#
#     rosservice call /gazebo/set_model_state \
#         "{model_state: {model_name: honey, ...}}"
#     rostopic pub /honey/mgt/sys_monitor/state ff_msgs/FaultState '{state: 0}' -1
#
# The home pose is not hard-coded here because it belongs to the world, not to
# the GUI. It is captured from Gazebo the first time the GUI sees the robot,
# saved to the config file, and re-capturable from the Experiment tab. No home
# pose means the teleport is skipped - never a guessed one.
HOME_SETTLE_S = 0.0         # deliberately zero for now

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

    # Where "Reset experiment" puts the robot: x y z qx qy qz qw, world frame.
    # None until captured from the running simulation.
    home_pose: Optional[Dict[str, float]] = None

    # follow_cam: one entry per camera - live offset, saved "default" pose,
    # optics. See default_camera(). The step sizes are the operator's, not a
    # camera's, so they stay flat.
    cameras: List[dict] = field(
        default_factory=lambda: [default_camera(i) for i in range(CAM_COUNT)])
    cam_step_mm: float = 5.0
    cam_step_deg: float = 1.0

    # image viewer
    video_host: str = VIDEO_HOST
    video_port: int = VIDEO_PORT
    video_autostart: bool = True
    stream_quality: int = 80
    preview_enabled: bool = False

    # tools
    tool_label: str = 'Ratchet Wrench'
    force_axes: Dict[str, bool] = field(
        default_factory=lambda: {'x': True, 'y': True, 'z': True})
    torque_axes: Dict[str, bool] = field(
        default_factory=lambda: {'x': True, 'y': True, 'z': True})
    max_force_n: float = 1.0        # 0.10 m/s on a 1 kg tool
    max_torque_nm: float = 0.5      # 35 deg/s at 0.083 kg.m^2

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
            self._migrate_cameras(data)
            self._normalise_cameras()
            log.info('Loaded settings from %s', path)
        except Exception as exc:                               # noqa: BLE001
            log.warning('Could not read %s (%s). Using defaults.', path, exc)

    # --- cameras -------------------------------------------------------------

    def _migrate_cameras(self, data: dict) -> None:
        """Fold a single-camera config file into camera 1.

        The old file has cam_offset / cam_fov_deg / cam_width and no `cameras`
        key. Those attributes no longer exist, so load() skips them silently -
        without this the operator would lose a camera offset that took real
        time to set up. Cameras 2-4 inherit the offset, so all four start
        co-located and following the robot the same way; move them apart from
        the Camera tab.
        """
        legacy = ('cam_offset', 'cam_defaults', 'cam_fov_deg', 'cam_width',
                  'cam_height', 'cam_rate_hz')
        if 'cameras' in data or not any(key in data for key in legacy):
            return

        first = self.cameras[0]
        first['offset'].update(data.get('cam_offset') or {})
        first['defaults'].update(data.get('cam_defaults') or {})
        for key, source in (('fov_deg', 'cam_fov_deg'), ('width', 'cam_width'),
                            ('height', 'cam_height'), ('rate_hz', 'cam_rate_hz')):
            if source in data:
                first[key] = data[source]

        for camera in self.cameras[1:]:
            camera['offset'] = dict(first['offset'])
            camera['defaults'] = dict(first['defaults'])
        log.info('Migrated the single-camera settings into camera 1; cameras '
                 '2-4 start at the same offset.')

    def _normalise_cameras(self) -> None:
        """Make the list exactly CAM_COUNT well-formed entries.

        A short list, a missing key or a newly added axis must never be able to
        KeyError the GUI on startup over a stale config file.
        """
        cameras = list(self.cameras or [])[:CAM_COUNT]
        while len(cameras) < CAM_COUNT:
            cameras.append(default_camera(len(cameras)))

        for index, camera in enumerate(cameras):
            reference = default_camera(index)
            if not isinstance(camera, dict):
                cameras[index] = reference
                continue
            for key in CAMERA_KEYS:
                camera.setdefault(key, reference[key])
            for axis, zero in CAM_ZERO.items():
                camera['offset'].setdefault(axis, zero)
                camera['defaults'].setdefault(axis, zero)
        self.cameras = cameras

    def camera(self, index: int) -> dict:
        """Bounds-safe accessor - the UI holds an index across a config reload."""
        self._normalise_cameras()
        return self.cameras[max(0, min(CAM_COUNT - 1, int(index)))]

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

    def resolution_label(self, index: int) -> str:
        camera = self.camera(index)
        size = (int(camera['width']), int(camera['height']))
        for label, preset in CAM_RESOLUTIONS.items():
            if preset == size:
                return label
        return '{} x {}'.format(*size)

    def image_topic(self, index: int) -> str:
        return '/{}/{}/image_raw'.format(self.ns, cam_model_name(index))

    @property
    def video_base_url(self) -> str:
        return 'http://{}:{}'.format(self.video_host or VIDEO_HOST,
                                     int(self.video_port))

    def stream_url(self, index: int) -> str:
        """MJPEG URL for camera `index` from web_video_server.

        All four cameras come off ONE web_video_server: the topic is a query
        parameter, so a second server would add a port to manage and nothing
        else.

        No `width` parameter, and this is not an oversight. web_video_server
        parses width and height off the query string and then, a few lines
        later, overwrites both with the input image size before the resize
        check runs - so the resize branch is unreachable and the parameter can
        never do anything. Upstream bug, reported as issue #119 against
        RobotWebTools/web_video_server: two `if (output_width_ == -1)` guards
        were deleted, and the fix (PR #130) is still unmerged.

        Not verified against this container's build - it is the published
        cause of exactly the symptom we saw, which is not the same thing. To
        check locally:

            curl -s "http://localhost:8091/snapshot?topic=/honey/follow_cam\
/image_raw&width=320" -o /tmp/s.jpg && file /tmp/s.jpg

        `file` prints the JPEG dimensions. If they are the sensor resolution
        rather than 320 wide, this build has the bug. /snapshot shares the
        same streamer code as /stream, and is a single JPEG rather than a
        multipart stream, so it is the easier one to measure.

        `quality` is applied at JPEG encode time and does work. To change the
        streamed image size, change the sensor resolution on the Camera tab:
        that changes what Gazebo renders, so it is also the only one of the
        two that reduces load.
        """
        return '{}/stream?topic={}&quality={}'.format(
            self.video_base_url, self.image_topic(index),
            int(self.stream_quality))

    @property
    def cam_step_m(self) -> float:
        return float(self.cam_step_mm) / 1000.0

    @property
    def cam_step_rad(self) -> float:
        return math.radians(float(self.cam_step_deg))

    @property
    def tool(self):
        return TOOLS.get(self.tool_label, TOOLS['Ratchet Wrench'])


settings = Settings()
