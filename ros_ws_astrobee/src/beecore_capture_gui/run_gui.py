#!/usr/bin/env python3
"""BEECORE Capture GUI.

Run:
    source /opt/ros/noetic/setup.bash
    source <astrobee workspace>/devel/setup.bash
    source ~/guienv/bin/activate
    python3 run_gui.py

Environment overrides:
    BEECORE_GUI_PORT    default 8090
    BEECORE_GUI_HOST    default 0.0.0.0
    BEECORE_GUI_CONFIG  default ~/.beecore_capture_gui.json
    CUSTOM_WS            used to derive the default tool models directory
"""

import socket
import sys
import threading
import time
from datetime import datetime

from nicegui import app, ui

from beecore import logbridge
from beecore.follow_cam import FollowCam, FollowCamError
from beecore.config import ASSETS_DIR, GUI_HOST, GUI_PORT, settings
from beecore.diagnostics import Diagnostics
from beecore.logbridge import log
from beecore.naming import scan_next_experiment_id
from beecore.recorder import BagRecorder
from beecore.runners import RunnerSet
from beecore.reset import ToolError, capture_home_if_unset, full_reset
from beecore.ros_link import FaultStatePublisher, init_node
from beecore.state import state
from beecore.ui import viewer
from beecore.video_server import VideoServer, VideoServerError
from beecore.ui.page import MainPage


def port_free(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, port))
            return True
        except OSError:
            return False


def make_reset(fault_pub):
    """Reset action: robot home, fault state cleared, tool respawned.

    Runs on a worker thread - the teleport, spawn_model and the velocity
    readback take a few seconds between them and must not block the UI event
    loop. state.busy gates the buttons meanwhile.
    """

    def worker() -> None:
        # Cleared first: if this reset fails, the previous one must not end up
        # described in the next bag's metadata.
        state.last_reset = None
        try:
            meta = full_reset(settings, fault_pub)
            meta['at'] = datetime.now().isoformat(timespec='seconds')
            state.last_reset = meta
            log.info('Reset complete: %s spawned.', meta.get('tool'))
        except ToolError as exc:
            log.error('Reset failed: %s', exc)
        except Exception as exc:                               # noqa: BLE001
            log.error('Reset failed unexpectedly: %s', exc)
        finally:
            state.busy = False

    def on_reset() -> None:
        if state.busy:
            return
        # Do not touch UI here: this callback is shared across clients, and each
        # session's tick picks up state.busy on its own.
        state.busy = True
        threading.Thread(target=worker, daemon=True).start()

    return on_reset


def main() -> None:
    logbridge.setup()

    # Fail before registering with the master, so a doomed second launch cannot
    # knock a working GUI off ROS.
    if not port_free(GUI_HOST, GUI_PORT):
        sys.exit('Port {0} is already in use. Free it, or set BEECORE_GUI_PORT.\n'
                 "Check with: sudo ss -lptn 'sport = :{0}'".format(GUI_PORT))

    settings.load()
    state.ros_ok = init_node()

    scanned = scan_next_experiment_id(settings.save_dir)
    if scanned and scanned > settings.experiment_id:
        log.info('Existing runs found on disk. Next experiment ID set to %03d.',
                 scanned)
        settings.experiment_id = scanned

    fault_pub = FaultStatePublisher()
    recorder = BagRecorder(settings)
    runners = RunnerSet()
    video = VideoServer(settings)
    camera = FollowCam(settings)
    diagnostics = Diagnostics(settings, runners)

    app.add_static_files('/assets', ASSETS_DIR)
    app.on_shutdown(recorder.shutdown)
    app.on_shutdown(diagnostics.stop)
    app.on_shutdown(camera.stop)
    app.on_shutdown(camera.despawn_all)
    app.on_shutdown(video.stop)
    # setsid detaches the runners, so nothing else would clean them up.
    app.on_shutdown(runners.stop_all)

    # recorder.on_change is rebound per client inside the page handler.
    page = MainPage(recorder, diagnostics, camera,
                    on_reset=make_reset(fault_pub), runners=runners,
                    fault_pub=fault_pub, video=video)
    page.register()
    viewer.register()       # /view - deliberately outside the single-tab lock

    if state.ros_ok and settings.video_autostart:
        try:
            video.start()
        except VideoServerError as exc:
            log.warning('%s', exc)

    if state.ros_ok:
        diagnostics.start()

        # The cameras are standalone models, so they have to be spawned. Wait
        # for Gazebo to have the robot before working out where to put them,
        # then start the pin loop that keeps them glued there.
        def start_camera() -> None:
            camera.start()
            for attempt in range(30):
                time.sleep(2.0)
                try:
                    camera.spawn_all()
                    # The robot is up and at its start pose - the one moment
                    # we can be sure "current pose" means "home".
                    capture_home_if_unset(settings)
                    return
                except FollowCamError as exc:
                    if attempt == 0:
                        log.info('Waiting for the robot before spawning the '
                                 'cameras (%s)', exc)
            log.warning('Gave up spawning the cameras. Use "Apply optics" on '
                        'the Camera tab once the simulation is up.')

        threading.Thread(target=start_camera, daemon=True).start()

    # reload=False: the auto-reloader re-executes this module, and
    # rospy.init_node cannot be called twice in one process.
    # storage_secret is NOT what the single-tab lock runs on - the lock keys
    # on the NiceGUI client id precisely because sessionStorage is copied
    # into a duplicated tab. It is kept only so app.storage stays usable.
    ui.run(title='BEECORE Capture GUI', host=GUI_HOST, port=GUI_PORT,
           reload=False, show=False, dark=True, favicon='🐝',
           storage_secret='beecore-gui-local')


main()
