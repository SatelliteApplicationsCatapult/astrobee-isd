"""Four standalone Gazebo camera models, pinned to the robot in software.

WHY NOT THE URDF
----------------
Putting the cameras in the robot description makes them part of Astrobee's
rigid-body dynamics, which drags in everything that has nothing to do with
looking at things: links need inertia, bad inertia NaNs the whole articulated
body, unactuated joints wobble, actuated ones need controllers, and a mistake
breaks the robot rather than the camera. With four of them the argument only
gets stronger.

Each camera is its own model, outside the robot entirely. We read the robot's
pose from /gazebo/model_states, compose each camera's offset, and write the
result to the /gazebo/set_model_state TOPIC (not the service - the topic is
designed for exactly this and is far cheaper at rate).

ONE SUBSCRIBER, NO THREAD
-------------------------
Pinning happens INSIDE the model_states callback, once per message, using the
pose and twist from that same message. The earlier design sampled a shared
pose from a 60 Hz wall-clock thread, which was wrong in three ways: it ran
slower than the 125 Hz Gazebo publishes at, its phase against sim time
wandered as the real-time factor moved, and it read pose and twist under two
separate lock acquisitions so they could come from different messages. All
three showed up as visible jitter under motion. Callback-driven pinning is
sim-clock-locked by construction.

The robot pose is composed once per message and reused for all four cameras,
so they cannot drift a frame apart. Four subscribers would do the same work
four times over for nothing.

Consequences: no URDF edits, no added mass, no joints, no inertia tuning, no
NaN, and instant 6 DOF with no limits. The cameras are teleported rather than
simulated.

LEVER ARM
---------
Each camera sits ~0.45 m off the body origin, so its linear velocity is NOT
the body's. It is

    v_cam = v_body + omega x r        r = cam_world - body_world

Publishing the body twist verbatim - which this module used to do - gives the
camera the wrong velocity whenever the robot rotates, so it coasts off target
between pins and is snapped back by the next one. At 30 deg/s the missing term
is ~0.24 m/s, which is not small.

RESIDUAL LAG
------------
This does not reach zero. The pose Gazebo applies is at best one model_states
period behind (~8 ms at 125 Hz), and the camera sensors render asynchronously
to the pin, so any given frame sees 0-8 ms of stale pose. At 30 deg/s that is
up to 0.24 deg. Removing it entirely would mean setting the pose inside
Gazebo's own update tick, i.e. a compiled model plugin. Not done.

FRAMES
------
The SDF puts the perch_cam -> Gazebo-camera rotation on the SENSOR pose, so
the MODEL poses we write here are in plain perch_cam convention:

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
from typing import Dict, List, Optional, Tuple

import rospy

from .config import (CAM_COUNT, PERCH_CAM_QUAT, PERCH_CAM_XYZ, SDF_TEMPLATE,
                     Settings, cam_model_name)
from .geometry import (Quat, Vec3, compose, cross, quat_from_axis_angle,
                       quat_mul, quat_normalise, quat_to_rpy)
from .logbridge import log
from .ros_link import delete_model, model_exists

MOUNT_PITCH = -math.pi / 2      # perch_cam axes -> Gazebo camera axes


class FollowCamError(Exception):
    """Raised for conditions the operator needs to see."""


class FollowCam:
    """The rig: all four cameras, one pin loop.

    Every method that touches a single camera takes a 0-based index. Settings
    remain the single source of truth for offsets and optics - this class
    holds only what is genuinely runtime: the robot pose, the pin thread, and
    which models are currently spawned.
    """

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self._robot_pose = None     # type: Optional[Tuple[Vec3, Quat]]
        self._sub = None
        self._pub = None
        self._lock = threading.Lock()
        self._warned = False
        self._spawned = [False] * CAM_COUNT

    # --- per-camera accessors ------------------------------------------------

    @staticmethod
    def model_name(index: int) -> str:
        return cam_model_name(index)

    def config(self, index: int) -> dict:
        return self.settings.camera(index)

    def label(self, index: int) -> str:
        return self.config(index).get('label', 'Camera {}'.format(index + 1))

    def offset(self, index: int) -> Dict[str, float]:
        return self.config(index)['offset']

    def defaults(self, index: int) -> Dict[str, float]:
        return self.config(index)['defaults']

    def spawned(self, index: int) -> bool:
        return self._spawned[index]

    @property
    def any_spawned(self) -> bool:
        return any(self._spawned)

    # --- offset edits --------------------------------------------------------

    def nudge(self, index: int, axis: str, delta: float) -> float:
        offset = self.offset(index)
        offset[axis] = offset.get(axis, 0.0) + delta
        self.settings.save()
        return offset[axis]

    def reset(self, index: int) -> None:
        self.config(index)['offset'] = dict(self.defaults(index))
        self.settings.save()

    def set_as_default(self, index: int) -> None:
        self.config(index)['defaults'] = dict(self.offset(index))
        self.settings.save()
        log.info('%s default offset saved: %s', self.label(index),
                 self.summary_one(index))

    def summary_one(self, index: int) -> dict:
        return {key: round(value, 6)
                for key, value in self.offset(index).items()}

    def summary(self) -> List[dict]:
        """Everything a bag needs to reproduce the four viewpoints."""
        result = []
        for index in range(CAM_COUNT):
            camera = self.config(index)
            result.append({
                'label': camera.get('label'),
                'model': self.model_name(index),
                'topic': self.settings.image_topic(index),
                'offset_perchcam': self.summary_one(index),
                'fov_deg': camera['fov_deg'],
                'width': int(camera['width']),
                'height': int(camera['height']),
                'rate_hz': camera['rate_hz'],
                'spawned': self._spawned[index],
            })
        return result

    # --- pose maths ----------------------------------------------------------

    def offset_transform(self, index: int) -> Tuple[Vec3, Quat]:
        """Offset relative to perch_cam, in perch_cam axes."""
        off = self.offset(index)
        xyz = (off.get('x', 0.0), off.get('y', 0.0), off.get('z', 0.0))
        quat = quat_normalise(quat_mul(quat_mul(
            quat_from_axis_angle((1, 0, 0), off.get('pan', 0.0)),
            quat_from_axis_angle((0, 1, 0), off.get('tilt', 0.0))),
            quat_from_axis_angle((0, 0, 1), off.get('roll', 0.0))))
        return xyz, quat

    def perch_cam_pose(self) -> Optional[Tuple[Vec3, Quat]]:
        with self._lock:
            pose = self._robot_pose
        if pose is None:
            return None
        body_xyz, body_quat = pose
        return compose(body_xyz, quat_normalise(body_quat),
                       PERCH_CAM_XYZ, quat_normalise(PERCH_CAM_QUAT))

    def target_pose(self, index: int) -> Optional[Tuple[Vec3, Quat]]:
        """World pose for one camera model: robot * perch_cam * offset."""
        base = self.perch_cam_pose()
        if base is None:
            return None
        off_xyz, off_quat = self.offset_transform(index)
        return compose(base[0], base[1], off_xyz, off_quat)

    # --- model lifecycle -----------------------------------------------------

    def render_sdf(self, index: int) -> str:
        with open(SDF_TEMPLATE) as fh:
            template = Template(fh.read())
        camera = self.config(index)
        name = self.model_name(index)
        ns = self.settings.ns
        return template.substitute(
            model_name=name,
            mount_pitch='{:.6f}'.format(MOUNT_PITCH),
            rate='{:.1f}'.format(camera['rate_hz']),
            hfov='{:.6f}'.format(math.radians(camera['fov_deg'])),
            width=int(camera['width']),
            height=int(camera['height']),
            camera_name=name,
            image_topic=self.settings.image_topic(index),
            info_topic='/{}/{}/camera_info'.format(ns, name),
            frame_name='{}/{}'.format(ns, name))

    def spawn(self, index: int) -> None:
        """(Re)spawn one camera. Safe to call repeatedly."""
        self.despawn(index)

        pose = self.target_pose(index)
        if pose is None:
            raise FollowCamError(
                'No pose for model "{}" yet - is Gazebo up and the robot '
                'spawned?'.format(self.settings.ns))
        xyz, quat = pose
        name = self.model_name(index)
        camera = self.config(index)

        sdf = self.render_sdf(index)
        handle, path = tempfile.mkstemp(suffix='.sdf', prefix=name + '_')
        try:
            with os.fdopen(handle, 'w') as fh:
                fh.write(sdf)

            roll, pitch, yaw = quat_to_rpy(quat)
            cmd = ['rosrun', 'gazebo_ros', 'spawn_model',
                   '-sdf', '-file', path, '-model', name,
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
                raise FollowCamError('spawn_model failed for {}: {}'.format(
                    name, tail.strip()))
        except subprocess.TimeoutExpired:
            raise FollowCamError('spawn_model timed out for {}.'.format(name))
        except OSError as exc:
            raise FollowCamError('Could not run spawn_model: {}'.format(exc))
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass

        self._spawned[index] = True
        log.info('%s spawned: %.0f deg FOV, %dx%d @ %.0f Hz, topic %s',
                 self.label(index), camera['fov_deg'], int(camera['width']),
                 int(camera['height']), camera['rate_hz'],
                 self.settings.image_topic(index))

    def spawn_all(self) -> None:
        """Spawn every camera. One failure must not abandon the other three."""
        failures = []
        for index in range(CAM_COUNT):
            try:
                self.spawn(index)
            except FollowCamError as exc:
                failures.append(str(exc))
        if failures:
            raise FollowCamError(' / '.join(failures))

    def despawn(self, index: int) -> None:
        # Clear the flag FIRST. delete_model takes a moment, and the pin loop
        # runs at 60 Hz - leaving it set means a burst of
        # "Updating ModelState: model [follow_cam_n] does not exist" while the
        # deletion is in flight.
        self._spawned[index] = False
        name = self.model_name(index)
        if model_exists(name):
            ok, message = delete_model(name)
            if ok:
                log.info('Removed existing %s model.', name)
            else:
                log.warning('Could not remove %s: %s', name, message)

    def despawn_all(self) -> None:
        for index in range(CAM_COUNT):
            self.despawn(index)

    # --- pinning -------------------------------------------------------------

    def start(self) -> None:
        from gazebo_msgs.msg import ModelState, ModelStates

        # queue_size=1 on BOTH ends: a stale pin is worse than a dropped one.
        # tcp_nodelay because these are small messages at rate and Nagle
        # batching adds tens of milliseconds of variable delay.
        self._pub = rospy.Publisher('/gazebo/set_model_state', ModelState,
                                    queue_size=1, tcp_nodelay=True)
        self._sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                     self._on_states, queue_size=1,
                                     tcp_nodelay=True)
        log.info('Pinning %d cameras from /gazebo/model_states.', CAM_COUNT)

    def stop(self) -> None:
        for handle in (self._sub, self._pub):
            try:
                handle.unregister()
            except Exception:                                  # noqa: BLE001
                pass
        self._sub = self._pub = None

    def _on_states(self, msg) -> None:
        """Pose in, pins out - same message, same call, no shared clock."""
        name = self.settings.ns
        if name not in msg.name:
            return
        index = msg.name.index(name)
        pose, twist = msg.pose[index], msg.twist[index]

        body_xyz = (pose.position.x, pose.position.y, pose.position.z)
        body_quat = quat_normalise((pose.orientation.x, pose.orientation.y,
                                    pose.orientation.z, pose.orientation.w))
        # Kept only so spawn() has somewhere to read a pose from.
        with self._lock:
            self._robot_pose = (body_xyz, body_quat)

        if self._pub is not None and self.any_spawned:
            self._pin(body_xyz, body_quat, twist)

    def _pin(self, body_xyz: Vec3, body_quat: Quat, twist) -> None:
        from gazebo_msgs.msg import ModelState

        try:
            # Composed once, not once per camera: the robot pose and the
            # perch_cam transform are the same for all four.
            base_xyz, base_quat = compose(
                body_xyz, body_quat,
                PERCH_CAM_XYZ, quat_normalise(PERCH_CAM_QUAT))

            v_body = (twist.linear.x, twist.linear.y, twist.linear.z)
            omega = (twist.angular.x, twist.angular.y, twist.angular.z)

            for index in range(CAM_COUNT):
                if not self._spawned[index]:
                    continue
                off_xyz, off_quat = self.offset_transform(index)
                xyz, quat = compose(base_xyz, base_quat, off_xyz, off_quat)

                # v_cam = v_body + omega x r. Without the lever arm the camera
                # coasts off target between pins whenever the robot rotates.
                lever = cross(omega, (xyz[0] - body_xyz[0],
                                      xyz[1] - body_xyz[1],
                                      xyz[2] - body_xyz[2]))

                state = ModelState()
                state.model_name = self.model_name(index)
                state.reference_frame = 'world'
                (state.pose.position.x, state.pose.position.y,
                 state.pose.position.z) = xyz
                (state.pose.orientation.x, state.pose.orientation.y,
                 state.pose.orientation.z, state.pose.orientation.w) = quat
                state.twist.linear.x = v_body[0] + lever[0]
                state.twist.linear.y = v_body[1] + lever[1]
                state.twist.linear.z = v_body[2] + lever[2]
                # Rigid attachment: same angular velocity, no lever term.
                state.twist.angular.x = omega[0]
                state.twist.angular.y = omega[1]
                state.twist.angular.z = omega[2]
                self._pub.publish(state)
            self._warned = False
        except Exception as exc:                               # noqa: BLE001
            if not self._warned:
                log.error('Pin error: %s', exc)
                self._warned = True
