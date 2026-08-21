"""Thin wrapper around rospy and the Gazebo services.

The UI layer never imports rospy directly.
"""

from typing import List, Optional, Tuple

import rospy

from .config import NODE_NAME
from .geometry import Quat, Vec3
from .logbridge import log

# ff_msgs is only present when the Astrobee stack is sourced. The GUI must
# still start without it - it just cannot decode the fault state.
try:
    from ff_msgs.msg import FaultState
    HAVE_FF_MSGS = True
except Exception:                                              # noqa: BLE001
    FaultState = None
    HAVE_FF_MSGS = False


def init_node() -> bool:
    """Register with the master. Returns False if there is no master."""
    try:
        # disable_signals: uvicorn owns SIGINT/SIGTERM, not rospy.
        rospy.init_node(NODE_NAME, anonymous=False, disable_signals=True)
        log.info('ROS node "%s" registered with master.', NODE_NAME)
        if not HAVE_FF_MSGS:
            log.warning('ff_msgs not importable - fault state will show as '
                        'unknown. Source the Astrobee workspace.')
        return True
    except Exception as exc:                                   # noqa: BLE001
        log.error('ROS init failed: %s. GUI runs, recording is disabled.', exc)
        return False


def master_alive() -> bool:
    try:
        rospy.get_master().getPid()
        return True
    except Exception:                                          # noqa: BLE001
        return False


def list_topics() -> List[str]:
    try:
        return sorted(name for name, _type in rospy.get_published_topics())
    except Exception as exc:                                   # noqa: BLE001
        log.error('Could not list topics: %s', exc)
        return []


def has_publisher(topic: str) -> bool:
    try:
        return any(name == topic for name, _t in rospy.get_published_topics())
    except Exception:                                          # noqa: BLE001
        return False


def service_available(name: str, timeout: float = 0.3) -> bool:
    try:
        rospy.wait_for_service(name, timeout=timeout)
        return True
    except Exception:                                          # noqa: BLE001
        return False


def call_set_bool(name: str, value: bool) -> Tuple[bool, str]:
    from std_srvs.srv import SetBool
    try:
        rospy.wait_for_service(name, timeout=2.0)
        response = rospy.ServiceProxy(name, SetBool)(value)
        return bool(response.success), str(response.message)
    except Exception as exc:                                   # noqa: BLE001
        return False, str(exc)


def model_pose(model_name: str) -> Optional[Tuple[Vec3, Quat]]:
    """World pose of a Gazebo model, straight off /gazebo/model_states.

    Cheaper and simpler than a TF lookup, and the perch_cam offset is static
    so we can apply it ourselves.
    """
    from gazebo_msgs.msg import ModelStates
    try:
        msg = rospy.wait_for_message('/gazebo/model_states', ModelStates,
                                     timeout=3.0)
    except Exception as exc:                                   # noqa: BLE001
        log.error('No /gazebo/model_states (%s). Is Gazebo running?', exc)
        return None

    if model_name not in msg.name:
        log.error('Model "%s" not in Gazebo. Present: %s',
                  model_name, ', '.join(msg.name))
        return None

    pose = msg.pose[msg.name.index(model_name)]
    xyz = (pose.position.x, pose.position.y, pose.position.z)
    quat = (pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
    return xyz, quat


def set_model_configuration(model_name: str, urdf_param: str,
                            joint_names: List[str],
                            joint_positions: List[float]) -> Tuple[bool, str]:
    from gazebo_msgs.srv import SetModelConfiguration
    try:
        rospy.wait_for_service('/gazebo/set_model_configuration', timeout=2.0)
        proxy = rospy.ServiceProxy('/gazebo/set_model_configuration',
                                   SetModelConfiguration)
        response = proxy(model_name=model_name,
                         urdf_param_name=urdf_param,
                         joint_names=joint_names,
                         joint_positions=joint_positions)
        return bool(response.success), str(response.status_message)
    except Exception as exc:                                   # noqa: BLE001
        return False, str(exc)


def delete_model(model_name: str) -> Tuple[bool, str]:
    from gazebo_msgs.srv import DeleteModel
    try:
        rospy.wait_for_service('/gazebo/delete_model', timeout=2.0)
        response = rospy.ServiceProxy('/gazebo/delete_model',
                                      DeleteModel)(model_name)
        return bool(response.success), str(response.status_message)
    except Exception as exc:                                   # noqa: BLE001
        return False, str(exc)


def model_exists(model_name: str) -> bool:
    from gazebo_msgs.srv import GetWorldProperties
    try:
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=2.0)
        response = rospy.ServiceProxy('/gazebo/get_world_properties',
                                      GetWorldProperties)()
        return model_name in response.model_names
    except Exception:                                          # noqa: BLE001
        return False


def apply_body_wrench(body_name: str, force: Vec3, torque: Vec3,
                      duration_s: float) -> Tuple[bool, str]:
    from geometry_msgs.msg import Point, Vector3, Wrench
    from gazebo_msgs.srv import ApplyBodyWrench
    try:
        rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=2.0)
        wrench = Wrench(force=Vector3(*force), torque=Vector3(*torque))
        response = rospy.ServiceProxy('/gazebo/apply_body_wrench',
                                      ApplyBodyWrench)(
            body_name=body_name,
            reference_frame='world',
            reference_point=Point(0.0, 0.0, 0.0),
            wrench=wrench,
            start_time=rospy.Time(0),
            duration=rospy.Duration(duration_s))
        return bool(response.success), str(response.status_message)
    except Exception as exc:                                   # noqa: BLE001
        return False, str(exc)


class FaultStatePublisher:
    """Latched publisher for clearing the system monitor's fault state."""

    def __init__(self) -> None:
        self._pub = None
        self._topic = None

    def publish(self, topic: str, value: int) -> Tuple[bool, str]:
        if not HAVE_FF_MSGS:
            return False, 'ff_msgs not available'
        try:
            if self._pub is None or self._topic != topic:
                self._pub = rospy.Publisher(topic, FaultState,
                                            queue_size=1, latch=True)
                self._topic = topic
                rospy.sleep(0.3)        # let the connection establish
            msg = FaultState()
            msg.state = value
            self._pub.publish(msg)
            return True, 'published'
        except Exception as exc:                               # noqa: BLE001
            return False, str(exc)
