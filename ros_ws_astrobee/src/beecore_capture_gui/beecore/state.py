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

    # SetBool answers, so the GUI records both what it last commanded and
    # whether that call was accepted. custom_start is None until the first
    # call; custom_start_failed says the last attempt was refused, which is
    # ambiguous - we do not know what the node is actually doing.
    #
    # BOTH ARE CLEARED BACK TO None/False when Diagnostics sees the start
    # service disappear. They describe a conversation with one instance of the
    # node, and that instance is gone. Reporting them against its replacement
    # would be a lie, not staleness - the new node starts with start = False.
    custom_start: Optional[bool] = None
    custom_start_failed: bool = False

    # tool spawn / reset in progress
    busy: bool = False

    # What the last reset actually did: which tool, where it was spawned, what
    # impulse it was given. reset_tools() already computes all of it; keeping
    # it here is what lets metadata.json record the initial conditions of the
    # run instead of just the outcome. None means no successful reset since
    # launch, and a failed reset clears it rather than leaving the previous
    # one to be attributed to this bag.
    last_reset: Optional[dict] = None

    # Mirrors BagRecorder.recording so the camera tab can refuse a respawn
    # without importing the recorder.
    recording_guard: bool = False


state = RuntimeState()
