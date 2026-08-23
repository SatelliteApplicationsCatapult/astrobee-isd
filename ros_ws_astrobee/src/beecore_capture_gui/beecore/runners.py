"""rosrun / roslaunch commands as GUI-managed subprocesses.

Same machinery as VideoServer and BagRecorder, and for the same reasons:

    Popen(preexec_fn=os.setsid)   -> its own process group
    killpg(pgid, SIGINT)          -> wait -> killpg(SIGKILL) if it will not go

killpg with SIGINT is what a terminal sends on Ctrl+C: the whole foreground
process group, not just the parent. That matters most for roslaunch, which
traps SIGINT to shut its children down cleanly - signalling the roslaunch PID
alone would leave the nodes it started running and registered with the master.

The group is then SWEPT. A clean exit by the group leader does not mean the
group is empty: children can ignore SIGINT (a backgrounded job in a
non-interactive shell has it set to ignore), and once the leader is reaped
nothing is left to wait on them. So after the leader goes we check whether
anything is still in the group and SIGKILL it if so. The pgid is captured at
spawn time because after wait() the pid is reaped and getpgid() fails.

INTENT AND ACTUAL ARE SEPARATE
------------------------------
`desired` is what the operator asked for. It changes ONLY when they move the
switch. `running` is what poll() observes. The LED grades one against the
other, so a runner that died shows desired=True, running=False - red - and
the switch stays where the operator left it.

Writing the observation back into the switch would be wrong twice over: a
control that moves on its own is disorienting, and in NiceGUI assigning
`switch.value` fires on_change, so the view would be calling back into the
model every time a process died.

OUTPUT GOES TO DEVNULL
----------------------
Not PIPE. Nothing here drains the pipes, and a chatty node would fill the
64 KB kernel pipe buffer and then block forever on its next write. DEVNULL or
a drain thread are the only safe options; DEVNULL is what VideoServer does.
To see a runner's output, run its command in a terminal.
"""

import os
import signal
import stat
import subprocess
from typing import Callable, Dict, List, Optional

from .logbridge import log

STOP_TIMEOUT_S = 10         # roslaunch shutdown is not instant
START_GRACE_S = 1.0         # long enough to catch "package not found"


class RunnerError(Exception):
    """Raised for conditions the operator needs to see."""


# --- pre-start hooks ---------------------------------------------------------

JS_DEVICE = '/dev/input/js0'
JS_MAJOR, JS_MINOR = 13, 0      # linux joystick char device
JS_MODE = 0o666


def ensure_js0() -> None:
    """Recreate /dev/input/js0 if it has vanished.

    The node disappears from inside the container often enough that the only
    other remedy is restarting the container. Recreating it is what the
    operator was doing by hand:

        mknod /dev/input/js0 c 13 0
        chmod 666 /dev/input/js0

    The explicit chmod is not redundant: mknod's mode is masked by the
    process umask, so 0666 typically lands as 0644.

    This recreates the NODE, not the device. If the underlying joystick is
    genuinely gone rather than just its /dev entry, joy_node will open the
    node and fail on read - a node existing is not evidence a gamepad does.
    Gamepad Data on the diagnostics column is what settles that.

    Raises rather than warning: joy_node does not exit when the device is
    missing, it retries, so a silent failure here would leave a green switch
    over a runner that can never work.
    """
    if os.path.exists(JS_DEVICE):
        return

    log.warning('%s is missing; recreating it.', JS_DEVICE)
    directory = os.path.dirname(JS_DEVICE)
    try:
        if not os.path.isdir(directory):
            os.makedirs(directory)
        os.mknod(JS_DEVICE, stat.S_IFCHR | JS_MODE,
                 os.makedev(JS_MAJOR, JS_MINOR))
        os.chmod(JS_DEVICE, JS_MODE)
    except OSError as exc:
        raise RunnerError(
            'Could not recreate {} ({}). mknod needs root or CAP_MKNOD inside '
            'the container.'.format(JS_DEVICE, exc))
    log.info('Recreated %s as char %d:%d, mode %o.',
             JS_DEVICE, JS_MAJOR, JS_MINOR, JS_MODE)


# key -> (label, argv, prepare). Order is the order they appear in the UI.
# `prepare` runs immediately before the command and may raise RunnerError.
RUNNERS = (
    ('joy_node', 'Joystick Node',
     ['rosrun', 'joy', 'joy_node', '_autorepeat_rate:=20', '_default_trig_val:=true'], ensure_js0),
    ('joy_convert', 'Joystick Command Converter',
     ['rosrun', 'astrobee_joy_teleop', 'astrobee_joy_arm_wrench.py'], None),
    ('fam_control', 'Custom FAM Control',
     ['roslaunch', 'astrobee_ros_demo', 'python_joy_client.launch'], None),
)


class Runner:
    """One managed command."""

    def __init__(self, key: str, label: str, argv: List[str],
                 prepare: Optional[Callable[[], None]] = None) -> None:
        self.key = key
        self.label = label
        self.argv = list(argv)
        self.prepare = prepare
        self.proc = None            # type: Optional[subprocess.Popen]
        self._pgid = None           # type: Optional[int]
        self.desired = False        # operator intent; never set by poll()
        self.detail = 'not running'

    @property
    def running(self) -> bool:
        return self.proc is not None

    @property
    def command(self) -> str:
        return ' '.join(self.argv)

    def start(self) -> None:
        if self.running:
            return

        # Before the process, not after: a fix that lands once the node is
        # already retrying is a fix the node may never notice.
        if self.prepare is not None:
            try:
                self.prepare()
            except RunnerError:
                self.detail = 'pre-start check failed'
                raise

        try:
            self.proc = subprocess.Popen(
                self.argv, preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            self.detail = 'could not launch'
            raise RunnerError('{}: could not launch "{}": {}'.format(
                self.label, self.command, exc))

        # rosrun and roslaunch both exit fast when the package or node is not
        # found, so an immediate death is worth reporting now rather than
        # leaving the switch on for a process that never existed.
        try:
            self.proc.wait(timeout=START_GRACE_S)
        except subprocess.TimeoutExpired:
            try:
                self._pgid = os.getpgid(self.proc.pid)
            except OSError:
                self._pgid = None
            self.detail = 'running (pid {})'.format(self.proc.pid)
            log.info('%s started: %s', self.label, self.command)
            return

        code = self.proc.returncode
        self.proc = None
        self._pgid = None
        self.detail = 'exited immediately (code {})'.format(code)
        raise RunnerError(
            '{} exited immediately (code {}). Check the package is built and '
            'sourced: {}'.format(self.label, code, self.command))

    def _group_alive(self) -> bool:
        """Is anything still in the process group? Signal 0 only tests."""
        if self._pgid is None:
            return False
        try:
            os.killpg(self._pgid, 0)
            return True
        except OSError:
            return False

    def stop(self) -> None:
        if self.proc is None:
            self._pgid = None
            return
        log.info('Stopping %s ...', self.label)
        pgid = self._pgid
        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGINT)
            else:
                self.proc.terminate()
        except OSError:
            self.proc.terminate()

        try:
            self.proc.wait(timeout=STOP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            log.warning('%s ignored SIGINT after %d s; killing.',
                        self.label, STOP_TIMEOUT_S)
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGKILL)
                else:
                    self.proc.kill()
            except OSError:
                self.proc.kill()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                log.error('%s would not die.', self.label)

        # The leader is gone; the group may not be. Anything left here ignored
        # SIGINT and now has nothing waiting on it.
        if self._group_alive():
            log.warning('%s left processes behind after SIGINT; killing the '
                        'group.', self.label)
            try:
                os.killpg(pgid, signal.SIGKILL)
            except OSError:
                pass

        self.proc = None
        self._pgid = None
        self.detail = 'not running'

    def poll(self) -> bool:
        """Watchdog. True if it died on its own since the last check."""
        if self.proc is not None and self.proc.poll() is not None:
            code = self.proc.returncode
            self.proc = None
            # Leader gone; sweep whatever it left behind.
            if self._group_alive():
                log.warning('%s died leaving processes behind; killing the '
                            'group.', self.label)
                try:
                    os.killpg(self._pgid, signal.SIGKILL)
                except OSError:
                    pass
            self._pgid = None
            self.detail = 'exited on its own (code {})'.format(code)
            log.error('%s exited unexpectedly (code %s).', self.label, code)
            return True
        return False


class RunnerSet:
    """The three runners, kept in declared order."""

    def __init__(self) -> None:
        self.runners = [Runner(key, label, argv, prepare)
                        for key, label, argv, prepare in RUNNERS]
        self._by_key = {runner.key: runner for runner in self.runners}

    def __iter__(self):
        return iter(self.runners)

    def get(self, key: str) -> Runner:
        return self._by_key[key]

    def set_running(self, key: str, value: bool) -> None:
        """Record intent, then act on it. Intent is recorded even if the
        start fails - that mismatch is exactly what the LED is for."""
        runner = self.get(key)
        runner.desired = bool(value)
        if value:
            runner.start()
        else:
            runner.stop()

    def poll(self) -> bool:
        """True if any runner died on its own."""
        return any([runner.poll() for runner in self.runners])

    def states(self) -> Dict[str, bool]:
        return {runner.key: runner.running for runner in self.runners}

    def desired(self) -> Dict[str, bool]:
        return {runner.key: runner.desired for runner in self.runners}

    def stop_all(self) -> None:
        """setsid detaches these, so nothing else will clean them up."""
        for runner in self.runners:
            try:
                runner.desired = False
                runner.stop()
            except Exception as exc:                           # noqa: BLE001
                log.error('Error stopping %s: %s', runner.label, exc)
