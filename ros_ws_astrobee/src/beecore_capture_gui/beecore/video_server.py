"""web_video_server as a managed subprocess.

The GUI owns the process so you do not have to keep a terminal open for it.
Same machinery as BagRecorder, and for the same reasons:

  Popen(preexec_fn=os.setsid)  -> its own process group
  killpg(pgid, SIGINT) -> wait -> killpg(SIGKILL) if it will not go

setsid matters because signalling only the parent can leave children behind,
and because it detaches the server from the terminal that started the GUI -
so Ctrl+C in that terminal cannot hang waiting on it.

Not fatal if it is missing or refuses to start: the GUI logs it, the viewer
shows "No stream" and retries, and everything else carries on.

RECLAIMING THE PORT
-------------------
setsid is also why the server outlives a GUI that was killed rather than shut
down. The orphan keeps the port, and if roscore was restarted underneath it,
it is registered with a master that no longer exists: it still accepts HTTP,
still returns 200 on the index, and can no longer resolve a single image
topic. The viewer goes blank while the port looks busy.

The old rule - "port in use, leave it alone" - is wrong for that case, and
right for every other one. So start() identifies WHAT is holding the port
before deciding. The holder is found by matching the listening socket's inode
in /proc/net/tcp against /proc/<pid>/fd, then confirmed by reading
/proc/<pid>/cmdline. Only a process whose command line is web_video_server is
killed; anything else keeps the port and is logged by name.

No lsof and no fuser: neither is guaranteed present in the container, and
under host networking /proc/net/tcp shows host sockets while /proc/<pid>/fd
shows only our own namespace - so a holder outside the container resolves to
no PID and is correctly left alone rather than mistaken for ours.
"""

import os
import signal
import socket
import subprocess
import time
from typing import List, Optional, Tuple

from .config import Settings
from .logbridge import log

STOP_TIMEOUT_S = 8


class VideoServerError(Exception):
    """Raised for conditions the operator needs to see."""


def port_in_use(port: int, host: str = '0.0.0.0') -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, port))
            return False
        except OSError:
            return True


# --- who is holding the port -------------------------------------------------

TCP_LISTEN = '0A'               # /proc/net/tcp st column for LISTEN
SERVER_MARKER = 'web_video_server'


def _listening_inodes(port: int) -> set:
    """Socket inodes LISTENing on `port`, from /proc/net/tcp and tcp6."""
    inodes = set()
    for source in ('/proc/net/tcp', '/proc/net/tcp6'):
        try:
            with open(source) as fh:
                lines = fh.read().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            if len(fields) < 10 or fields[3] != TCP_LISTEN:
                continue
            try:
                if int(fields[1].rsplit(':', 1)[1], 16) == port:
                    inodes.add(fields[9])
            except (IndexError, ValueError):
                continue
    return inodes


def _pids_holding(inodes: set) -> set:
    """PIDs in OUR namespace with one of these sockets open."""
    if not inodes:
        return set()
    targets = {'socket:[{}]'.format(inode) for inode in inodes}
    pids = set()
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        fd_dir = '/proc/{}/fd'.format(entry)
        try:
            names = os.listdir(fd_dir)
        except OSError:
            continue                # gone, or not ours to read
        for fd in names:
            try:
                if os.readlink(os.path.join(fd_dir, fd)) in targets:
                    pids.add(int(entry))
                    break
            except OSError:
                continue
    return pids


def _cmdline(pid: int) -> str:
    try:
        with open('/proc/{}/cmdline'.format(pid), 'rb') as fh:
            raw = fh.read()
    except OSError:
        return ''
    return raw.decode('utf-8', 'replace').replace('\0', ' ').strip()


def port_holders(port: int) -> List[Tuple[int, str]]:
    """[(pid, cmdline)] for whatever is listening on `port`.

    Empty either because nothing is listening or because the holder is
    outside this PID namespace. The two are not distinguishable from here,
    which is why the caller must never take an empty result as permission.
    """
    return sorted((pid, _cmdline(pid))
                  for pid in _pids_holding(_listening_inodes(port)))


class VideoServer:

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.proc = None            # type: Optional[subprocess.Popen]
        self.external = False       # someone else's server is on our port

    # --- state ---------------------------------------------------------------

    @property
    def running(self) -> bool:
        return self.proc is not None

    @staticmethod
    def available() -> bool:
        """Is the package installed?"""
        try:
            result = subprocess.run(['rospack', 'find', 'web_video_server'],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, timeout=10)
            return result.returncode == 0
        except Exception:                                      # noqa: BLE001
            return False

    def status(self) -> str:
        if self.running:
            return 'managed by the GUI on port {}'.format(self.settings.video_port)
        if port_in_use(self.settings.video_port):
            return 'already running on port {} (not started by the GUI)'.format(
                self.settings.video_port)
        return 'not running'

    # --- lifecycle -----------------------------------------------------------

    def _reclaim(self, port: int) -> bool:
        """Kill an orphaned web_video_server on `port`. True if the port is free.

        SIGKILL, not SIGINT: the case this exists for is a server that has
        lost its master and is no longer doing anything worth shutting down
        cleanly, and a wedged one will not answer SIGINT anyway. It is not our
        child, so there is nothing to wait() on - we poll the port instead.

        Returns False for every case where we are not certain, which includes
        an empty holder list: under host networking that means "outside this
        container", not "nothing there".
        """
        holders = port_holders(port)
        if not holders:
            log.warning('Port %d is in use but the process holding it is not '
                        'visible from this container. Leaving it alone.', port)
            return False

        ours = [(pid, cmd) for pid, cmd in holders if SERVER_MARKER in cmd]
        if not ours:
            for pid, cmd in holders:
                log.warning('Port %d is held by pid %d (%s), which is not a '
                            'web_video_server. Leaving it alone.',
                            port, pid, cmd or 'unknown command')
            return False

        for pid, cmd in ours:
            log.warning('Killing orphaned web_video_server on port %d: '
                        'pid %d (%s).', port, pid, cmd)
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError as exc:
                log.error('Could not kill pid %d: %s', pid, exc)

        # The socket closes as the process is reaped, which is not instant.
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if not port_in_use(port):
                log.info('Port %d reclaimed.', port)
                return True
            time.sleep(0.1)

        log.error('Port %d is still in use after the kill.', port)
        return False

    def start(self) -> None:
        if self.running:
            return
        port = int(self.settings.video_port)

        if port_in_use(port) and not self._reclaim(port):
            # Something we did not start, and could not identify as one of
            # ours, is on the port. Do not fight it - a second bind would
            # abort. Checked before the install test: if something is already
            # serving, whether the package exists is beside the point.
            self.external = True
            return

        if not self.available():
            raise VideoServerError(
                'web_video_server is not installed. '
                'sudo apt install ros-noetic-web-video-server')

        cmd = ['rosrun', 'web_video_server', 'web_video_server',
               '_port:={}'.format(port)]
        try:
            self.proc = subprocess.Popen(
                cmd, preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            raise VideoServerError(
                'Could not launch web_video_server: {}'.format(exc))

        # It binds almost immediately; if it aborted, say so now rather than
        # leaving the operator to wonder why the viewer is blank.
        time.sleep(1.0)
        if self.proc.poll() is not None:
            code = self.proc.returncode
            self.proc = None
            raise VideoServerError(
                'web_video_server exited immediately (code {}). Port {} may be '
                'taken.'.format(code, port))

        self.external = False
        log.info('web_video_server started on port %d.', port)

    def stop(self) -> None:
        if self.proc is None:
            return
        log.info('Stopping web_video_server ...')
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except OSError:
            self.proc.terminate()
        try:
            self.proc.wait(timeout=STOP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except OSError:
                self.proc.kill()
            self.proc.wait(timeout=3)
        self.proc = None

    def poll(self) -> bool:
        """Watchdog. True if it died on its own."""
        if self.proc is not None and self.proc.poll() is not None:
            log.error('web_video_server exited unexpectedly (code %s).',
                      self.proc.returncode)
            self.proc = None
            return True
        return False
