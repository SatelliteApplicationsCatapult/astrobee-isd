"""follow_cam as a standalone Gazebo model, pinned to the robot in software.

WHY NOT THE URDF
----------------
Putting the camera in the robot description makes it part of Astrobee's
rigid-body dynamics, which drags in everything that has nothing to do with
looking at things: links need inertia, bad inertia NaNs the whole articulated
body, unactuated joints wobble, actuated ones need controllers, and a mistake
breaks the robot rather than the camera. None of that is inherent to "put a
camera somewhere and let it follow the robot".

Here the camera is its own model, outside the robot entirely. We read the
robot's pose from /gazebo/model_states, compose the offset, and write the
result to the /gazebo/set_model_state TOPIC (not the service - the topic is
designed for exactly this and is far cheaper at rate).

Consequences: no URDF edits, no added mass, no joints, no inertia tuning, no
NaN, no wobble, and instant 6 DOF with no limits. The camera is teleported
rather than simulated.

The cost is that pinning happens at PIN_RATE_HZ while physics runs faster, so
under hard acceleration the camera lags by a sub-millimetre. We publish the
robot's twist along with the pose, which smooths what remains.

FRAMES
------
The SDF puts the perch_cam -> Gazebo-camera rotation on the SENSOR pose, so
the MODEL pose we write here is in plain perch_cam convention:

    +X down     +Y right     +Z forward (out of the lens)

Offset rotations are applied pan (about X) then tilt (about Y) then roll
(about Z), matching the operator's mental model.
"""

import math
import os
import subprocess
import tempfile
import threading
from string import Template
from typing import Dict, Optional, Tuple

import rospy

from .config import (CAM_FRAME, CAM_MODEL_NAME, PERCH_CAM_QUAT, PERCH_CAM_XYZ,
                     PIN_RATE_HZ, SDF_TEMPLATE, Settings)
from .geometry import (Quat, Vec3, compose, quat_from_axis_angle, quat_mul,
                       quat_normalise)
from .logbridge import log
from .ros_link import delete_model, model_exists

MOUNT_PITCH = -math.pi / 2      # perch_cam axes -> Gazebo camera axes


class FollowCamError(Exception):
    """Raised for conditions the operator needs to see."""


class FollowCam:

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self._robot_pose = None     # type: Optional[Tuple[Vec3, Quat]]
        self._robot_twist = None
        self._sub = None
        self._pub = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._warned = False
        self.spawned = False

    # --- offset --------------------------------------------------------------

    @property
    def offset(self) -> Dict[str, float]:
        return self.settings.cam_offset

    @property
    def defaults(self) -> Dict[str, float]:
        return self.settings.cam_defaults

    def nudge(self, axis: str, delta: float) -> float:
        value = self.offset.get(axis, 0.0) + delta
        self.offset[axis] = value
        self.settings.save()
        return value

    def reset(self) -> None:
        self.settings.cam_offset = dict(self.defaults)
        self.settings.save()

    def zero(self) -> None:
        self.settings.cam_offset = {key: 0.0 for key in self.offset}
        self.settings.save()

    def set_as_default(self) -> None:
        self.settings.cam_defaults = dict(self.offset)
        self.settings.save()
        log.info('follow_cam default offset saved: %s', self.summary())

    def summary(self) -> dict:
        return {key: round(value, 6) for key, value in self.offset.items()}

    # --- pose maths ----------------------------------------------------------

    def offset_transform(self) -> Tuple[Vec3, Quat]:
        """Offset relative to perch_cam, in perch_cam axes."""
        off = self.offset
        xyz = (off.get('x', 0.0), off.get('y', 0.0), off.get('z', 0.0))
        quat = quat_normalise(quat_mul(quat_mul(
            quat_from_axis_angle((1, 0, 0), off.get('pan', 0.0)),
            quat_from_axis_angle((0, 1, 0), off.get('tilt', 0.0))),
            quat_from_axis_angle((0, 0, 1), off.get('roll', 0.0))))
        return xyz, quat

    def target_pose(self) -> Optional[Tuple[Vec3, Quat]]:
        """World pose for the camera model: robot * perch_cam * offset."""
        with self._lock:
            pose = self._robot_pose
        if pose is None:
            return None
        body_xyz, body_quat = pose
        cam_xyz, cam_quat = compose(body_xyz, quat_normalise(body_quat),
                                    PERCH_CAM_XYZ, quat_normalise(PERCH_CAM_QUAT))
        off_xyz, off_quat = self.offset_transform()
        return compose(cam_xyz, cam_quat, off_xyz, off_quat)

    # --- model lifecycle -----------------------------------------------------

    def render_sdf(self) -> str:
        with open(SDF_TEMPLATE) as fh:
            template = Template(fh.read())
        ns = self.settings.ns
        return template.substitute(
            model_name=CAM_MODEL_NAME,
            mount_pitch='{:.6f}'.format(MOUNT_PITCH),
            rate='{:.1f}'.format(self.settings.cam_rate_hz),
            hfov='{:.6f}'.format(math.radians(self.settings.cam_fov_deg)),
            width=int(self.settings.cam_width),
            height=int(self.settings.cam_height),
            camera_name='follow_cam',
            image_topic='/{}/follow_cam/image_raw'.format(ns),
            info_topic='/{}/follow_cam/camera_info'.format(ns),
            frame_name='{}/{}'.format(ns, CAM_FRAME))

    def spawn(self) -> None:
        """(Re)spawn the camera model. Safe to call repeatedly."""
        self.despawn()

        pose = self.target_pose()
        if pose is None:
            raise FollowCamError(
                'No pose for model "{}" yet - is Gazebo up and the robot '
                'spawned?'.format(self.settings.ns))
        xyz, quat = pose

        sdf = self.render_sdf()
        handle, path = tempfile.mkstemp(suffix='.sdf', prefix='follow_cam_')
        try:
            with os.fdopen(handle, 'w') as fh:
                fh.write(sdf)

            from .geometry import quat_to_rpy
            roll, pitch, yaw = quat_to_rpy(quat)
            cmd = ['rosrun', 'gazebo_ros', 'spawn_model',
                   '-sdf', '-file', path, '-model', CAM_MODEL_NAME,
                   '-x', '{:.5f}'.format(xyz[0]),
                   '-y', '{:.5f}'.format(xyz[1]),
                   '-z', '{:.5f}'.format(xyz[2]),
                   '-R', '{:.5f}'.format(roll),
                   '-P', '{:.5f}'.format(pitch),
                   '-Y', '{:.5f}'.format(yaw)]
            result = subprocess.run(cmd, timeout=30, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT)
            if result.returncode != 0:
                tail = (result.stdout or b'').decode('utf-8', 'replace')[-400:]
                raise FollowCamError('spawn_model failed: {}'.format(tail.strip()))
        except subprocess.TimeoutExpired:
            raise FollowCamError('spawn_model timed out.')
        except OSError as exc:
            raise FollowCamError('Could not run spawn_model: {}'.format(exc))
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass

        self.spawned = True
        log.info('follow_cam spawned: %.0f deg FOV, %dx%d, topic /%s/follow_cam/image_raw',
                 self.settings.cam_fov_deg, self.settings.cam_width,
                 self.settings.cam_height, self.settings.ns)

    def despawn(self) -> None:
        if model_exists(CAM_MODEL_NAME):
            ok, message = delete_model(CAM_MODEL_NAME)
            if ok:
                log.info('Removed existing follow_cam model.')
            else:
                log.warning('Could not remove follow_cam: %s', message)
        self.spawned = False

    # --- pinning -------------------------------------------------------------

    def start(self) -> None:
        from gazebo_msgs.msg import ModelState, ModelStates

        self._pub = rospy.Publisher('/gazebo/set_model_state', ModelState,
                                    queue_size=1)
        self._sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                     self._on_states, queue_size=1,
                                     tcp_nodelay=True)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        log.info('follow_cam pin loop running at %d Hz.', PIN_RATE_HZ)

    def stop(self) -> None:
        self._stop.set()
        for handle in (self._sub, self._pub):
            try:
                handle.unregister()
            except Exception:                                  # noqa: BLE001
                pass

    def _on_states(self, msg) -> None:
        name = self.settings.ns
        if name not in msg.name:
            return
        index = msg.name.index(name)
        pose, twist = msg.pose[index], msg.twist[index]
        with self._lock:
            self._robot_pose = (
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w))
            self._robot_twist = twist

    def _run(self) -> None:
        from gazebo_msgs.msg import ModelState

        period = 1.0 / float(PIN_RATE_HZ)
        while not self._stop.is_set():
            try:
                if self.spawned:
                    pose = self.target_pose()
                    if pose is not None:
                        xyz, quat = pose
                        state = ModelState()
                        state.model_name = CAM_MODEL_NAME
                        state.reference_frame = 'world'
                        (state.pose.position.x, state.pose.position.y,
                         state.pose.position.z) = xyz
                        (state.pose.orientation.x, state.pose.orientation.y,
                         state.pose.orientation.z,
                         state.pose.orientation.w) = quat
                        # Matching the robot's twist smooths the residual
                        # between pins; without it the camera visibly judders
                        # when the robot accelerates.
                        with self._lock:
                            twist = self._robot_twist
                        if twist is not None:
                            state.twist = twist
                        self._pub.publish(state)
                        self._warned = False
            except Exception as exc:                           # noqa: BLE001
                if not self._warned:
                    log.error('follow_cam pin loop error: %s', exc)
                    self._warned = True
            self._stop.wait(period)
