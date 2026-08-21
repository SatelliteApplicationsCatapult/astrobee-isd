"""Runtime state - what the robot is doing right now, not what persists.

Deliberately a single module-level instance: recording is a property of the
robot, not of a browser tab.
"""

from dataclasses import dataclass, field
from typing import List, Optional

from .config import OUTCOME_NA


@dataclass
class RuntimeState:
    outcome: str = OUTCOME_NA
    available_topics: List[str] = field(default_factory=list)
    ros_ok: bool = False

    # Services expose no readable state, so the GUI remembers what it last
    # commanded. None means "never set by this GUI since launch".
    gnc_enabled: Optional[bool] = None
    custom_start: Optional[bool] = None

    # tool spawn / reset in progress
    busy: bool = False

    # Mirrors BagRecorder.recording so the camera tab can refuse a respawn
    # without importing the recorder.
    recording_guard: bool = False


state = RuntimeState()
