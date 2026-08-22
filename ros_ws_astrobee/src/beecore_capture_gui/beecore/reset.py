"""The full "Reset experiment" sequence.

    1. teleport the robot back to its home pose, velocity zeroed
    2. clear the system monitor's fault state
    3. delete, respawn and perturb the tool  (tools.py)

Steps 1 and 2 are the two commands that were verified by hand long before
they were wired in:

    rosservice call /gazebo/set_model_state "{model_state: {model_name: honey, ...}}"
    rostopic pub /honey/mgt/sys_monitor/state ff_msgs/FaultState '{state: 0}' -1

THE HOME POSE
-------------
Not hard-coded. The robot always starts in the same place, but that place is a
property of the world file, not of this GUI, and typing it in twice is how the
two drift apart. Instead it is captured from Gazebo the first time the GUI sees
the robot after launch, saved to the config file, and re-capturable from the
Experiment tab.

If no home pose has been captured, the teleport is SKIPPED and logged. A
guessed pose that flings the robot into a wall is worse than no teleport.

ORDER
-----
Robot first, then the tool: the spawn box is defined relative to perch_cam, so
sampling it before the teleport would place the tool relative to wherever the
robot happened to have drifted to. tools.py re-reads the robot pose itself, so
it picks up the teleported position.

No settle time between the two - the teleport is instantaneous and the tool
spawn is a separate service call, so by the time it lands the robot is already
at the home pose. If the FSW is later found to need a moment to notice it has
moved, HOME_SETTLE_S in config.py is the place to put it.
"""

from typing import Dict, Optional, Tuple

from .config import FAULT_CLEAR_PUBLISH, HOME_SETTLE_S, Settings
from .geometry import Quat, Vec3
from .logbridge import log
from .ros_link import model_pose, set_model_state
from .tools import ToolError, reset_tools

_KEYS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')


# --- home pose ---------------------------------------------------------------

def pose_to_dict(xyz: Vec3, quat: Quat) -> Dict[str, float]:
    return dict(zip(_KEYS, [round(value, 6) for value in
                            list(xyz) + list(quat)]))


def dict_to_pose(stored: Dict[str, float]) -> Optional[Tuple[Vec3, Quat]]:
    """None if the stored dict is incomplete, rather than a partial pose."""
    if not stored or any(key not in stored for key in _KEYS):
        return None
    values = [float(stored[key]) for key in _KEYS]
    return tuple(values[:3]), tuple(values[3:])


def describe_home(settings: Settings) -> str:
    pose = dict_to_pose(settings.home_pose or {})
    if pose is None:
        return 'not captured - the robot will not be moved'
    xyz, quat = pose
    return 'xyz [{:.3f} {:.3f} {:.3f}]  quat [{:.3f} {:.3f} {:.3f} {:.3f}]'.format(
        *(list(xyz) + list(quat)))


def capture_home(settings: Settings) -> bool:
    """Store the robot's current pose as home. Returns False if unreadable."""
    pose = model_pose(settings.ns)
    if pose is None:
        log.error('Could not read the pose of "%s" - home pose unchanged.',
                  settings.ns)
        return False
    settings.home_pose = pose_to_dict(*pose)
    settings.save()
    log.info('Home pose captured: %s', describe_home(settings))
    return True


def capture_home_if_unset(settings: Settings) -> None:
    """Called once at startup, after the robot appears in Gazebo."""
    if dict_to_pose(settings.home_pose or {}) is not None:
        return
    log.info('No home pose stored; taking the robot\'s current pose as home.')
    capture_home(settings)


# --- steps -------------------------------------------------------------------

def reset_robot(settings: Settings) -> dict:
    """Teleport the robot home. Returns metadata; never raises."""
    pose = dict_to_pose(settings.home_pose or {})
    if pose is None:
        log.warning('No home pose captured - skipping the robot teleport. '
                    'Use "Set home to current pose" on the Experiment tab.')
        return {'moved': False, 'reason': 'no home pose captured'}

    xyz, quat = pose
    ok, message = set_model_state(settings.ns, xyz, quat)
    if ok:
        log.info('Robot "%s" returned to home: %s', settings.ns,
                 describe_home(settings))
    else:
        log.error('Could not move "%s": %s', settings.ns, message)

    if HOME_SETTLE_S:
        import time
        time.sleep(HOME_SETTLE_S)

    return {'moved': bool(ok), 'home_pose': dict(settings.home_pose or {}),
            'message': message}


def clear_fault(settings: Settings, fault_pub) -> dict:
    """Publish FaultState 0, which the monitor takes to FUNCTIONAL itself.

    Latched, because the system monitor may not be subscribed at the instant
    we publish. The Astrobee state LED on the Experiment tab is what confirms
    it took - this only reports whether the publish succeeded.
    """
    topic = settings.topic('mgt/sys_monitor/state')
    ok, message = fault_pub.publish(topic, FAULT_CLEAR_PUBLISH)
    if ok:
        log.info('Published FaultState %d to %s.', FAULT_CLEAR_PUBLISH, topic)
    else:
        log.warning('Could not publish to %s: %s', topic, message)
    return {'published': bool(ok), 'topic': topic, 'message': message}


# --- the whole thing ---------------------------------------------------------

def full_reset(settings: Settings, fault_pub) -> dict:
    """Robot, then fault state, then tools. Raises ToolError from the tools."""
    meta = {'robot': reset_robot(settings),
            'fault': clear_fault(settings, fault_pub)}
    meta.update(reset_tools(settings))
    return meta


__all__ = ['ToolError', 'capture_home', 'capture_home_if_unset',
           'describe_home', 'full_reset']
