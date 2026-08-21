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
"""

import os
import signal
import socket
import subprocess
import time
from typing import Optional

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

    def start(self) -> None:
        if self.running:
            return
        port = int(self.settings.video_port)

        if port_in_use(port):
            # Could be a previous instance, could be something unrelated -
            # either way we must not fight it, and a second bind would abort.
            # Checked before the install test: if something is already serving,
            # whether the package exists is beside the point.
            self.external = True
            log.info('Port %d is already serving; leaving it alone.', port)
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
