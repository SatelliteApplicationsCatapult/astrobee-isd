"""Diagnostic monitors for the Experiment tab.

Runs on a daemon thread and writes into a plain dict; the UI reads that dict
from its own timer. Nothing here touches NiceGUI.

Colour language: GREEN always means "ready to run", never "this thing is on".
The Astrobee state LED is graded on the FaultState value alone - see
FAULT_GREEN / FAULT_RED in config.

RUNNER LEDS GRADE INTENT AGAINST ACTUAL
---------------------------------------
A runner LED is not "is it running" - that would make a runner the operator
deliberately stopped look the same as one that crashed. It compares what was
asked for with what poll() sees:

    off / not running   grey     off, as asked
    on  / running       green
    on  / not running   RED      died, or never started
    off / running       AMBER    shutdown failed - should not happen

The runner watchdog lives here, on this thread, next to the other watchdogs.
It used to run inside the UI refresh, which had the view driving the model.

/honey/start is a SetBool, so the response is real feedback about whether the
call was accepted. It is not feedback about what the node does afterwards, so
the service disappearing is graded ambiguous rather than off.
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import rospy

from .config import (DIAG_POLL_S, FAULT_GREEN, FAULT_RED, FAULT_STATES,
                     STALE_AFTER_S, Settings)
from .logbridge import log
from .ros_link import HAVE_FF_MSGS, has_publisher, service_available
from .state import state

OK = 'ok'
WARN = 'warn'
STALE = 'stale'
DOWN = 'down'
UNKNOWN = 'unknown'


@dataclass
class Reading:
    status: str = UNKNOWN
    detail: str = ''
    last_msg_at: Optional[float] = None


@dataclass
class Diagnostics:
    settings: Settings
    runners: object = None
    readings: Dict[str, Reading] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self._lock = threading.Lock()
        self._subs = []
        self._subscribed_ns = None
        self._stop = threading.Event()
        self._fault_value = None        # type: Optional[int]
        keys = ['joy', 'points', 'fault']
        if self.runners is not None:
            keys.extend(runner.key for runner in self.runners)
        keys.append('start')
        for key in keys:
            self.readings[key] = Reading()

    # --- topic names ---------------------------------------------------------

    @property
    def joy_topic(self) -> str:
        return '/joy'               # deliberately not namespaced

    @property
    def points_topic(self) -> str:
        return self.settings.topic('hw/depth_perch/points')

    @property
    def fault_topic(self) -> str:
        return self.settings.topic('mgt/sys_monitor/state')

    @property
    def start_service(self) -> str:
        return self.settings.topic('start')

    # --- lifecycle -----------------------------------------------------------

    def start(self) -> None:
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._ensure_subscribers()
                self._poll()
            except Exception as exc:                           # noqa: BLE001
                log.debug('Diagnostics poll error: %s', exc)
            self._stop.wait(DIAG_POLL_S)

    # --- subscriptions -------------------------------------------------------

    def _ensure_subscribers(self) -> None:
        """Re-subscribe when the namespace input changes."""
        if self._subscribed_ns == self.settings.ns and self._subs:
            return

        for sub in self._subs:
            try:
                sub.unregister()
            except Exception:                                  # noqa: BLE001
                pass
        self._subs = []

        # AnyMsg means we do not need the message type to know it is alive.
        self._subs.append(rospy.Subscriber(
            self.joy_topic, rospy.AnyMsg,
            lambda _m: self._touch('joy'), queue_size=1))
        self._subs.append(rospy.Subscriber(
            self.points_topic, rospy.AnyMsg,
            lambda _m: self._touch('points'), queue_size=1))

        if HAVE_FF_MSGS:
            from ff_msgs.msg import FaultState
            self._subs.append(rospy.Subscriber(
                self.fault_topic, FaultState, self._on_fault, queue_size=1))
        else:
            self._subs.append(rospy.Subscriber(
                self.fault_topic, rospy.AnyMsg,
                lambda _m: self._touch('fault'), queue_size=1))

        self._subscribed_ns = self.settings.ns
        log.info('Diagnostics watching namespace "%s".', self.settings.ns)

    def _touch(self, key: str) -> None:
        with self._lock:
            self.readings[key].last_msg_at = time.time()

    def _on_fault(self, msg) -> None:
        with self._lock:
            self._fault_value = int(msg.state)
            self.readings['fault'].last_msg_at = time.time()

    # --- polling -------------------------------------------------------------

    def _poll(self) -> None:
        now = time.time()

        with self._lock:
            joy = self.readings['joy']
            points = self.readings['points']
            fault = self.readings['fault']
            fault_value = self._fault_value

        self._grade(joy, self.joy_topic, now)
        self._grade(points, self.points_topic, now)

        # Astrobee state: graded on the FaultState value alone. Liveness is
        # deliberately NOT folded in - a stale FUNCTIONAL still reads green.
        if fault_value is None:
            fault.status = UNKNOWN
            fault.detail = ('ff_msgs unavailable' if not HAVE_FF_MSGS
                            else 'no message yet')
        else:
            name = FAULT_STATES.get(fault_value, 'UNRECOGNISED')
            fault.detail = '{} ({})'.format(name, fault_value)
            if fault_value in FAULT_GREEN:
                fault.status = OK
            elif fault_value in FAULT_RED:
                fault.status = DOWN
            else:
                fault.status = WARN

        self._grade_runners()
        self._grade_start()

    def _grade(self, reading: Reading, topic: str, now: float) -> None:
        if reading.last_msg_at is None:
            reading.status = DOWN if not has_publisher(topic) else STALE
            reading.detail = ('no publisher' if reading.status == DOWN
                              else 'publisher up, no data yet')
            return
        age = now - reading.last_msg_at
        if age <= STALE_AFTER_S:
            reading.status = OK
            reading.detail = 'live'
        elif has_publisher(topic):
            reading.status = STALE
            reading.detail = '{:.0f} s since last message'.format(age)
        else:
            reading.status = DOWN
            reading.detail = 'publisher gone'

    def _grade_runners(self) -> None:
        """Intent vs actual. poll() first - it is the observation."""
        if self.runners is None:
            return
        self.runners.poll()
        for runner in self.runners:
            reading = self.readings[runner.key]
            if runner.desired and runner.running:
                reading.status = OK
            elif runner.desired and not runner.running:
                reading.status = DOWN
            elif runner.running:
                reading.status = WARN     # asked to stop, still alive
            else:
                reading.status = UNKNOWN  # off, as asked
            reading.detail = runner.detail

    def _grade_start(self) -> None:
        """SetBool answers, so green/red are earned. Amber is genuine doubt."""
        reading = self.readings['start']
        name = self.start_service
        if state.custom_start_failed:
            reading.status = WARN
            reading.detail = 'last call failed - state unknown'
            return
        if not service_available(name):
            reading.status = WARN
            reading.detail = 'service not advertised - state unknown'
            return
        if state.custom_start is None:
            reading.status = UNKNOWN
            reading.detail = 'available, not called this session'
            return
        reading.status = OK if state.custom_start else DOWN
        reading.detail = 'last call set {} and was accepted'.format(
            'true' if state.custom_start else 'false')

    # --- read ----------------------------------------------------------------

    def snapshot(self) -> Dict[str, Reading]:
        with self._lock:
            return {key: Reading(value.status, value.detail, value.last_msg_at)
                    for key, value in self.readings.items()}

    @property
    def fault_value(self) -> Optional[int]:
        with self._lock:
            return self._fault_value

    def all_ready(self) -> bool:
        return all(reading.status == OK for reading in self.readings.values())
