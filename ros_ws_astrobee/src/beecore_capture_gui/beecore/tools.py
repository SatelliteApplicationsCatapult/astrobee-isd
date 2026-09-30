"""Tool discovery and spawning for the reset action.

Discovery
---------
Nothing is hard-coded. settings.models_dir is the tools folder (the one on
gzserver's GAZEBO_MODEL_PATH). Every sub-folder with a model.sdf is a model;
<name>_bright is the bright variant of <name>. The Gazebo model name is the
folder name. A thumbnail is shown if <model>/thumb.png exists.

Reset sequence
--------------
    1. delete every model in Gazebo whose name matches a folder in the tools dir
    2. sample a pose inside SPAWN_BOX (0.5 m) in front of the perch cam
    3. spawn the selected SDF there with a uniformly random orientation
    4. set its velocity with /gazebo/set_model_state (pose re-asserted)

Frames
------
    perch_cam: +Z forward (out of the lens), +X down, +Y right

Linear velocity is chosen per perch_cam axis (drift relative to the view).
Angular velocity is chosen per TOOL BODY axis (link origin = CoM, Z = long
axis), then rotated to world with the spawn orientation, because
set_model_state takes the twist in the reference frame ('world').

Magnitudes
----------
Each ticked axis gets exactly the slider value (speed_m_s / spin_deg_s) with a
random sign. X/Y linear signs are biased toward the boresight so edge spawns do
not drift straight out of view. No force or torque is involved, so the result
is independent of the tool's mass and inertia.

The velocity is READ BACK from /gazebo/model_states and logged next to the
requested one, so a set that did not land shows up as a number.
"""

import math
import os
import random
import subprocess
import time
from typing import Dict, List, Optional, Tuple

from .config import (BRIGHT_SUFFIX, CENTRING_BIAS, PERCH_CAM_QUAT, PERCH_CAM_XYZ, SPAWN_BOX, THUMBNAIL, Settings)
from .geometry import Quat, Vec3, compose, quat_normalise, quat_rotate, quat_to_rpy, random_quat
from .logbridge import log
from .ros_link import delete_model, model_pose, model_velocity, set_model_state, wait_for_model, world_model_names

AXES = ('x', 'y', 'z')


class ToolError(Exception):
    """Raised for conditions the operator needs to see."""


# --- discovery ---------------------------------------------------------------

def model_folders(tools_dir: str) -> List[str]:
    """Every folder in tools_dir holding a model.sdf, both variants."""
    if not os.path.isdir(tools_dir):
        return []
    return sorted(name for name in os.listdir(tools_dir) if os.path.isfile(os.path.join(tools_dir, name, 'model.sdf')))


def list_tools(tools_dir: str) -> List[str]:
    """Base tool names (no _bright), sorted."""
    return [name for name in model_folders(tools_dir) if not name.endswith(BRIGHT_SUFFIX)]


def tool_label(name: str) -> str:
    """combination_wrench_08mm -> Combination wrench 08mm"""
    return name.replace('_', ' ').capitalize()


def has_bright(tools_dir: str, name: str) -> bool:
    return os.path.isfile(os.path.join(tools_dir, name + BRIGHT_SUFFIX, 'model.sdf'))


def model_name(tools_dir: str, name: str, bright: bool) -> str:
    """Folder / Gazebo model name for the requested variant. Falls back to the
    default variant, with a warning, if the bright one does not exist."""
    if bright and not has_bright(tools_dir, name):
        log.warning('No bright variant for "%s" - using the original colours.', name)
        bright = False
    return name + BRIGHT_SUFFIX if bright else name


def thumbnail(tools_dir: str, model: str) -> Optional[str]:
    path = os.path.join(tools_dir, model, THUMBNAIL)
    return path if os.path.isfile(path) else None


def selected_tool(settings: Settings) -> str:
    """settings.tool_name if it still exists, else the first tool found."""
    tools = list_tools(settings.models_dir)
    if not tools:
        raise ToolError('No tool models (sub-folders with a model.sdf) in {}'.format(settings.models_dir))
    if settings.tool_name in tools:
        return settings.tool_name
    if settings.tool_name:
        log.warning('Tool "%s" not found in %s - using "%s".', settings.tool_name, settings.models_dir, tools[0])
    return tools[0]


def check_gazebo_model_path(tools_dir: str) -> None:
    """Raise if the running gzserver cannot resolve model://<name>/meshes/...

    The mesh URIs need the tools folder ITSELF on gzserver's GAZEBO_MODEL_PATH
    (model:// is not recursive). The server's environment is what matters, not
    ours, so it is read from /proc. If gzserver is not visible from here (other
    container), the check is skipped with a warning.
    """
    pid = None
    for entry in os.listdir('/proc'):
        if entry.isdigit():
            try:
                with open('/proc/{}/comm'.format(entry)) as fh:
                    if fh.read().strip() == 'gzserver':
                        pid = entry
                        break
            except OSError:
                continue
    if pid is None:
        log.warning('gzserver process not visible - cannot check its GAZEBO_MODEL_PATH.')
        return
    try:
        with open('/proc/{}/environ'.format(pid), 'rb') as fh:
            env = dict(item.split(b'=', 1) for item in fh.read().split(b'\0') if b'=' in item)
    except OSError as exc:
        log.warning('Could not read gzserver environment (%s) - GAZEBO_MODEL_PATH not checked.', exc)
        return
    paths = env.get(b'GAZEBO_MODEL_PATH', b'').decode('utf-8', 'replace').split(':')
    target = os.path.realpath(tools_dir)
    if not any(path and os.path.realpath(path) == target for path in paths):
        raise ToolError('{} is not on gzserver\'s GAZEBO_MODEL_PATH - meshes would not resolve. '
                        'Export it before launching the sim.'.format(tools_dir))


# --- sampling ----------------------------------------------------------------

def sample_offset(rng: random.Random) -> Vec3:
    """A point inside the spawn box, in perch_cam coordinates."""
    return tuple(rng.uniform(*SPAWN_BOX[axis]) for axis in AXES)


def sample_velocity(offset: Vec3, axes: Dict[str, bool], speed: float, rng: random.Random) -> Vec3:
    """Linear velocity in perch_cam coordinates: +/-speed on each ticked axis.

    X/Y: the sign points back toward the boresight with probability
    0.5 + 0.5 * CENTRING_BIAS * |normalised offset|.
    """
    result = []
    for index, axis in enumerate(AXES):
        if not axes.get(axis, True):
            result.append(0.0)
            continue
        sign = rng.choice((-1.0, 1.0))
        if axis in ('x', 'y'):
            low, high = SPAWN_BOX[axis]
            half, centre = (high - low) / 2.0, (high + low) / 2.0
            normalised = (offset[index] - centre) / half if half else 0.0
            inward = -1.0 if normalised > 0 else 1.0
            sign = inward if rng.random() < 0.5 + 0.5 * CENTRING_BIAS * abs(normalised) else -inward
        result.append(sign * speed)
    return tuple(result)


def sample_spin(axes: Dict[str, bool], spin_rad: float, rng: random.Random) -> Vec3:
    """Angular velocity in TOOL BODY coordinates: +/-spin on each ticked axis."""
    return tuple(rng.choice((-1.0, 1.0)) * spin_rad if axes.get(axis, True) else 0.0 for axis in AXES)


def randomise(settings: Settings, rng: random.Random = random) -> None:
    """The Randomise button: axes ticked, both slider values, and - if
    settings.random_tool is set - the tool and its colour variant. Never the maxima.

    Each of the six axes is a coin flip, redrawn if all six come up empty, so a
    randomised reset always moves the tool.
    """
    while True:
        force = {axis: rng.random() < 0.5 for axis in AXES}
        torque = {axis: rng.random() < 0.5 for axis in AXES}
        if any(force.values()) or any(torque.values()):
            break
    settings.force_axes, settings.torque_axes = force, torque
    settings.speed_m_s = round(rng.uniform(0.0, settings.speed_max_m_s), 3)
    settings.spin_deg_s = round(rng.uniform(0.0, settings.spin_max_deg_s), 1)
    names = list_tools(settings.models_dir)
    if settings.random_tool and names:
        settings.tool_name = rng.choice(names)
        settings.tool_bright = has_bright(settings.models_dir, settings.tool_name) and rng.random() < 0.5
    settings.save()


# --- pose --------------------------------------------------------------------

def perch_cam_pose(settings: Settings) -> Tuple[Vec3, Quat]:
    """World pose of the perch cam, via the robot's model state."""
    pose = model_pose(settings.ns)
    if pose is None:
        raise ToolError('Could not read the pose of model "{}" from Gazebo.'.format(settings.ns))
    body_xyz, body_quat = pose
    return compose(body_xyz, quat_normalise(body_quat), PERCH_CAM_XYZ, quat_normalise(PERCH_CAM_QUAT))


# --- actions -----------------------------------------------------------------

def delete_all_tools(tools_dir: str) -> int:
    """Remove every model in Gazebo whose name is a folder in tools_dir."""
    present = world_model_names()
    if present is None:
        log.warning('Could not list Gazebo models - existing tools not deleted.')
        return 0
    removed = 0
    for name in set(model_folders(tools_dir)) & set(present):
        ok, message = delete_model(name)
        if ok:
            log.info('Deleted existing model "%s".', name)
            removed += 1
        else:
            log.warning('Could not delete "%s": %s', name, message)
    return removed


def spawn_tool(tools_dir: str, model: str, xyz: Vec3, quat: Quat) -> None:
    sdf_path = os.path.join(tools_dir, model, 'model.sdf')
    if not os.path.isfile(sdf_path):
        raise ToolError('SDF not found: {}'.format(sdf_path))

    roll, pitch, yaw = quat_to_rpy(quat)
    cmd = ['rosrun', 'gazebo_ros', 'spawn_model', '-sdf', '-file', sdf_path, '-model', model,
           '-x', '{:.5f}'.format(xyz[0]), '-y', '{:.5f}'.format(xyz[1]), '-z', '{:.5f}'.format(xyz[2]),
           '-R', '{:.5f}'.format(roll), '-P', '{:.5f}'.format(pitch), '-Y', '{:.5f}'.format(yaw)]

    log.info('Spawning %s at [%.3f %.3f %.3f]', model, *xyz)
    try:
        result = subprocess.run(cmd, timeout=30, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    except subprocess.TimeoutExpired:
        raise ToolError('spawn_model timed out after 30 s.')
    except OSError as exc:
        raise ToolError('Could not run spawn_model: {}'.format(exc))

    if result.returncode != 0:
        tail = (result.stdout or b'').decode('utf-8', 'replace').strip()[-400:]
        raise ToolError('spawn_model failed: {}'.format(tail))


def _norm(vector) -> float:
    return math.sqrt(sum(component * component for component in vector))


def _measure(model: str) -> dict:
    """Read the tool's velocity back once the set has had a physics step or two."""
    time.sleep(0.2)
    velocity = model_velocity(model)
    if velocity is None:
        log.warning('Could not read back the velocity of "%s".', model)
        return {}
    linear, angular = velocity
    return {
        'speed_m_s': round(_norm(linear), 4),
        'spin_deg_s': round(math.degrees(_norm(angular)), 2),
        'linear_world_m_s': [round(v, 4) for v in linear],
        'angular_world_rad_s': [round(v, 4) for v in angular],
    }


def set_tool_motion(settings: Settings, model: str, xyz: Vec3, quat: Quat, cam_quat: Quat, offset: Vec3, rng: random.Random) -> dict:
    """Set the sampled twist. Returns what was requested and measured, for metadata."""
    if not wait_for_model(model, timeout=3.0):
        log.warning('Model "%s" has not appeared in model_states; setting its velocity anyway.', model)

    linear_cam = sample_velocity(offset, settings.force_axes, float(settings.speed_m_s), rng)
    angular_body = sample_spin(settings.torque_axes, math.radians(float(settings.spin_deg_s)), rng)
    linear_world = quat_rotate(cam_quat, linear_cam)
    angular_world = quat_rotate(quat, angular_body)

    ok, message = set_model_state(model, xyz, quat, linear_world, angular_world)
    if not ok:
        log.warning('set_model_state failed for %s: %s', model, message)

    measured = _measure(model)
    log.info('Tool motion: requested %.3f m/s, %.1f deg/s | measured %s m/s, %s deg/s',
             _norm(linear_world), math.degrees(_norm(angular_world)), measured.get('speed_m_s', '?'), measured.get('spin_deg_s', '?'))

    return {
        'linear_perchcam_m_s': [round(v, 4) for v in linear_cam],
        'angular_body_rad_s': [round(v, 4) for v in angular_body],
        'linear_world_m_s': [round(v, 4) for v in linear_world],
        'angular_world_rad_s': [round(v, 4) for v in angular_world],
        'speed_m_s': settings.speed_m_s,
        'spin_deg_s': settings.spin_deg_s,
        'force_axes': dict(settings.force_axes),
        'torque_axes': dict(settings.torque_axes),
        'state_set': ok,
        'measured': measured,
    }


def reset_tools(settings: Settings, seed: Optional[int] = None) -> dict:
    """Full sequence. Returns a metadata dict describing what was done."""
    rng = random.Random(seed)
    tools_dir = settings.models_dir
    check_gazebo_model_path(tools_dir)

    name = selected_tool(settings)
    model = model_name(tools_dir, name, settings.tool_bright)

    cam_xyz, cam_quat = perch_cam_pose(settings)
    delete_all_tools(tools_dir)

    offset = sample_offset(rng)
    quat = random_quat(rng)
    world_xyz, world_quat = compose(cam_xyz, cam_quat, offset, quat)

    spawn_tool(tools_dir, model, world_xyz, world_quat)
    motion = set_tool_motion(settings, model, world_xyz, world_quat, cam_quat, offset, rng)

    meta = {
        'tool': name,
        'model': model,
        'variant': 'bright' if model.endswith(BRIGHT_SUFFIX) else 'default',
        'spawn_offset_perchcam_m': [round(v, 4) for v in offset],
        'spawn_world_xyz_m': [round(v, 4) for v in world_xyz],
        'spawn_world_quat_xyzw': [round(v, 5) for v in world_quat],
    }
    meta.update(motion)
    return meta
