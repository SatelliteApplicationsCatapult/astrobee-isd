"""Tool spawning for the reset action.

Sequence:
    1. delete every known tool model that is currently in Gazebo
    2. sample a pose inside a 1 m box in front of the perch cam
    3. spawn the selected SDF there with a uniformly random orientation
    4. apply a short wrench impulse to give it a small drift

Frames
------
The perch_cam offset is static, so instead of a TF lookup we take the robot's
world pose off /gazebo/model_states and compose the known constant transform.

    perch_cam: +Z forward (out of the lens), +X down, +Y right

Perturbation
------------
apply_body_wrench takes force, not velocity, so the target velocity is turned
into a wrench with F = m*v/dt. Rather than querying each tool's mass we work
from configured maxima (MAX_FORCE_N, MAX_TORQUE_NM) chosen for a ~0.5 kg tool
over a 0.1 s impulse. Adjust in config.py if your tools are much heavier.

The XY components are biased back toward the boresight: a tool spawned near
the edge of the box gets pushed inward, so it does not immediately drift out
of view. Z is left uniform because drifting away along the view axis is fine.
"""

import os
import random
import subprocess
from typing import Dict, Optional, Tuple

from .config import (CENTRING_BIAS, IMPULSE_S, PERCH_CAM_QUAT, PERCH_CAM_XYZ,
                     SPAWN_BOX, TOOLS, Settings)
from .geometry import (Quat, Vec3, compose, quat_normalise, quat_rotate,
                       quat_to_rpy, random_quat)
from .logbridge import log
from .ros_link import apply_body_wrench, delete_model, model_exists, model_pose


class ToolError(Exception):
    """Raised for conditions the operator needs to see."""


# --- sampling ----------------------------------------------------------------

def sample_offset(rng: random.Random) -> Vec3:
    """A point inside the spawn box, in perch_cam coordinates."""
    return tuple(rng.uniform(*SPAWN_BOX[axis]) for axis in ('x', 'y', 'z'))


def sample_velocity(offset: Vec3, force_axes: Dict[str, bool],
                    rng: random.Random) -> Vec3:
    """Linear velocity in perch_cam coordinates, biased toward the boresight.

    For X and Y: sample uniformly, then subtract a term proportional to how far
    off-axis the spawn point is. A tool at the +X edge gets a -X push.
    """
    result = []
    for index, axis in enumerate(('x', 'y', 'z')):
        if not force_axes.get(axis, True):
            result.append(0.0)
            continue

        value = rng.uniform(-1.0, 1.0)
        if axis in ('x', 'y'):
            low, high = SPAWN_BOX[axis]
            half = (high - low) / 2.0
            centre = (high + low) / 2.0
            normalised = (offset[index] - centre) / half if half else 0.0
            value -= CENTRING_BIAS * normalised
            value = max(-1.0, min(1.0, value))
        result.append(value)
    return tuple(result)


def sample_spin(torque_axes: Dict[str, bool], rng: random.Random) -> Vec3:
    return tuple(rng.uniform(-1.0, 1.0) if torque_axes.get(axis, True) else 0.0
                 for axis in ('x', 'y', 'z'))


# --- pose --------------------------------------------------------------------

def perch_cam_pose(settings: Settings) -> Tuple[Vec3, Quat]:
    """World pose of the perch cam, via the robot's model state."""
    pose = model_pose(settings.ns)
    if pose is None:
        raise ToolError('Could not read the pose of model "{}" from Gazebo.'
                        .format(settings.ns))
    body_xyz, body_quat = pose
    return compose(body_xyz, quat_normalise(body_quat),
                   PERCH_CAM_XYZ, quat_normalise(PERCH_CAM_QUAT))


# --- actions -----------------------------------------------------------------

def delete_all_tools() -> int:
    """Remove every known tool model currently present. Returns the count."""
    removed = 0
    for _label, (_sdf, model_name, _link) in TOOLS.items():
        if not model_exists(model_name):
            continue
        ok, message = delete_model(model_name)
        if ok:
            log.info('Deleted existing model "%s".', model_name)
            removed += 1
        else:
            log.warning('Could not delete "%s": %s', model_name, message)
    return removed


def spawn_tool(settings: Settings, xyz: Vec3, quat: Quat) -> None:
    """Spawn via the verified rosrun command, run from the models directory."""
    sdf_rel, model_name, _link = settings.tool
    models_dir = settings.models_dir

    if not os.path.isdir(models_dir):
        raise ToolError('Models directory not found: {}'.format(models_dir))
    sdf_path = os.path.join(models_dir, sdf_rel)
    if not os.path.isfile(sdf_path):
        raise ToolError('SDF not found: {}'.format(sdf_path))

    roll, pitch, yaw = quat_to_rpy(quat)
    cmd = ['rosrun', 'gazebo_ros', 'spawn_model',
           '-sdf', '-file', sdf_rel, '-model', model_name,
           '-x', '{:.5f}'.format(xyz[0]),
           '-y', '{:.5f}'.format(xyz[1]),
           '-z', '{:.5f}'.format(xyz[2]),
           '-R', '{:.5f}'.format(roll),
           '-P', '{:.5f}'.format(pitch),
           '-Y', '{:.5f}'.format(yaw)]

    log.info('Spawning %s at [%.3f %.3f %.3f]', model_name, *xyz)
    try:
        result = subprocess.run(cmd, cwd=models_dir, timeout=30,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    except subprocess.TimeoutExpired:
        raise ToolError('spawn_model timed out after 30 s.')
    except OSError as exc:
        raise ToolError('Could not run spawn_model: {}'.format(exc))

    if result.returncode != 0:
        tail = (result.stdout or b'').decode('utf-8', 'replace').strip()[-400:]
        raise ToolError('spawn_model failed: {}'.format(tail))


def perturb_tool(settings: Settings, cam_quat: Quat,
                 offset: Vec3, rng: random.Random) -> dict:
    """Apply the randomised impulse. Returns what was applied, for metadata."""
    _sdf, model_name, link = settings.tool

    velocity = sample_velocity(offset, settings.force_axes, rng)
    spin = sample_spin(settings.torque_axes, rng)

    # Scale to the operator's slider maxima, then rotate perch_cam -> world.
    max_force = float(settings.max_force_n)
    max_torque = settings.max_torque_nm
    force_cam = tuple(component * max_force for component in velocity)
    torque_cam = tuple(component * max_torque for component in spin)
    force_world = quat_rotate(cam_quat, force_cam)
    torque_world = quat_rotate(cam_quat, torque_cam)

    body_name = '{}::{}'.format(model_name, link)
    ok, message = apply_body_wrench(body_name, force_world, torque_world,
                                    IMPULSE_S)
    if ok:
        log.info('Impulse on %s: F=[%.4f %.4f %.4f] N  T=[%.5f %.5f %.5f] Nm '
                 'for %.2f s', body_name, *(list(force_world) +
                                            list(torque_world) + [IMPULSE_S]))
    else:
        log.warning('apply_body_wrench failed for %s: %s', body_name, message)

    return {
        'model': model_name,
        'body': body_name,
        'force_world_N': [round(v, 6) for v in force_world],
        'torque_world_Nm': [round(v, 8) for v in torque_world],
        'impulse_s': IMPULSE_S,
        'max_force_n': max_force,
        'max_torque_nm': max_torque,
        'force_axes': dict(settings.force_axes),
        'torque_axes': dict(settings.torque_axes),
    }


def reset_tools(settings: Settings,
                seed: Optional[int] = None) -> dict:
    """Full sequence. Returns a metadata dict describing what was done."""
    rng = random.Random(seed)

    cam_xyz, cam_quat = perch_cam_pose(settings)

    delete_all_tools()

    offset = sample_offset(rng)
    quat = random_quat(rng)
    world_xyz, world_quat = compose(cam_xyz, cam_quat, offset, quat)

    spawn_tool(settings, world_xyz, world_quat)
    wrench = perturb_tool(settings, cam_quat, offset, rng)

    meta = {
        'tool': settings.tool_label,
        'spawn_offset_perchcam_m': [round(v, 4) for v in offset],
        'spawn_world_xyz_m': [round(v, 4) for v in world_xyz],
        'spawn_world_quat_xyzw': [round(v, 5) for v in world_quat],
    }
    meta.update(wrench)
    return meta
