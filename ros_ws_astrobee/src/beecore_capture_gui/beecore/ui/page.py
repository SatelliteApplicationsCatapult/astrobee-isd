"""Page layout: brand banner, status strip, tabs over debug pane.

PER-CLIENT STATE
----------------
Everything a browser tab owns - its widgets, its timers, its identity - is
created inside the page function and captured in closures. It must NOT live on
a shared object.

A NiceGUI @ui.page handler runs once per client but the surrounding object is
shared, so writing `self._tab = client.id` means a second tab overwrites the
first tab's identity. The first tab then compares the lock holder against the
*second* tab's id, concludes it still holds control, and keeps running. That is
exactly the "new tab gains control but the old one stays active" bug. The same
applies to every widget reference.

Shared services (recorder, diagnostics, camera) are genuinely global - they
describe the robot, not the tab - and stay on MainPage.
"""

import time
from typing import Dict, Optional

from fastapi import Request
from nicegui import Client, app, ui

from .. import logbridge, theme
from ..config import (LOGO_FILE, LOG_LINES, OUTCOME_FAILURE, OUTCOME_SUCCESS,
                      settings)
from ..logbridge import log
from ..recorder import free_space_gb
from ..ros_link import master_alive
from ..state import state
from .camera_tab import CameraTab
from .experiment_tab import ExperimentTab
from .other_tab import OtherTab
from .topics_tab import TopicsTab

# --- lock --------------------------------------------------------------------
#
# Identity is the NiceGUI client id, unique per page load. Not app.storage.tab:
# that is keyed by an id in sessionStorage, and both Firefox and Chrome COPY
# sessionStorage into a duplicated tab, so a duplicate would inherit the
# original's identity and slip through.

_LOCK = {'client_id': None, 'seen_at': 0.0, 'grant_until': 0.0}

LOCK_GRACE_S = 3.0          # holder silent for this long -> lock is free
GRANT_WINDOW_S = 10.0       # how long a takeover grant stays open


def _lock_is_free(client_id: str) -> bool:
    holder = _LOCK['client_id']
    if holder in (None, client_id):
        return True
    return (time.time() - float(_LOCK['seen_at'])) > LOCK_GRACE_S


def _claim(client_id: str) -> None:
    if _LOCK['client_id'] != client_id:
        log.info('Control claimed by client %s.', client_id[:8])
    _LOCK['client_id'] = client_id
    _LOCK['seen_at'] = time.time()


def _release(client_id: str) -> None:
    if _LOCK['client_id'] == client_id:
        _LOCK['client_id'] = None
        _LOCK['seen_at'] = 0.0


def _take_control() -> None:
    """Hand control to whoever renders next.

    Do NOT claim with the current client id: reloading mints a new one, so the
    claim would lock the reloaded page out with its own stale entry.
    """
    _LOCK['client_id'] = None
    _LOCK['seen_at'] = 0.0
    _LOCK['grant_until'] = time.time() + GRANT_WINDOW_S
    log.info('Control released for takeover.')
    ui.navigate.reload()


@app.post('/_gui/release')
async def _release_endpoint(request: Request):
    """Called by navigator.sendBeacon on pagehide.

    Not a security boundary - it only frees a local UI lock.
    """
    try:
        client_id = (await request.body()).decode('utf-8', 'replace').strip()
    except Exception:                                          # noqa: BLE001
        return {'ok': False}
    if client_id:
        _release(client_id)
    return {'ok': True}


def _install_release_beacon(client_id: str) -> None:
    ui.add_body_html(
        "<script>window.addEventListener('pagehide', function () {"
        "  navigator.sendBeacon('/_gui/release', '" + client_id + "');"
        "});</script>")


def _lock_screen() -> None:
    with ui.column().classes('absolute-center items-center gap-4'):
        ui.icon('lock', size='xl').style('color: {}'.format(theme.AMBER))
        ui.label('This GUI is open in another tab').classes('text-xl')
        ui.label('Only one tab controls an experiment, so the two can never '
                 'disagree about what is recording.'
                 ).classes('text-sm text-center max-w-md').style(
                     'color: {}'.format(theme.MUTED))
        ui.button('Take control here', icon='login',
                  on_click=_take_control).props('color=primary unelevated')


class _Session:
    """One browser tab's worth of UI. Never shared."""

    def __init__(self, client_id: str, recorder, diagnostics, camera,
                 on_reset, runners, fault_pub, video=None) -> None:
        self.client_id = client_id
        self.recorder = recorder
        self.diagnostics = diagnostics
        self.camera = camera
        self.lost = False
        self.log_pane = None
        self.signature = None

        self.experiment = ExperimentTab(recorder, diagnostics, on_reset, camera,
                                        runners, fault_pub)
        self.camera_tab = CameraTab(camera)
        self.topics = TopicsTab()
        self.video = video
        self.other = OtherTab(on_save_dir_change=self.experiment.refresh,
                              video=video)

    # --- chrome --------------------------------------------------------------

    def build(self) -> None:
        self._banner()
        self._status_strip()
        self._body()

    def _banner(self) -> None:
        with ui.row().classes('w-full items-center gap-4 px-5 py-3').style(
                'background: {}'.format(theme.VIOLET)):
            ui.image('/assets/' + LOGO_FILE).style(
                'width: 113px; height: 36px; object-fit: contain')
            ui.element('div').classes('h-8').style(
                'width: 1px; background: rgba(255,255,255,0.35)')
            ui.label('BEECORE Capture GUI').classes(
                'text-lg tracking-wide').style('color: #FFFFFF')

    def _status_strip(self) -> None:
        """Kept off the banner so red/green status stays legible."""
        with ui.row().classes('w-full items-center gap-3 px-5 py-1').style(
                'background: {}; border-bottom: 1px solid {}'.format(
                    theme.SURFACE, theme.BORDER)):
            self.rec_label = ui.label('idle').classes('font-mono text-sm')
            ui.space()
            self.ns_label = ui.label('').classes('font-mono text-xs').style(
                'color: {}'.format(theme.MUTED))
            self.disk_label = ui.label('').classes('font-mono text-xs').style(
                'color: {}'.format(theme.MUTED))
            self.ros_label = ui.label('').classes('font-mono text-xs')

    def _body(self) -> None:
        # A definite height, not just flex-grow: Quasar sizes splitter panels as
        # percentages, and a percentage of an indefinite height collapses.
        splitter = ui.splitter(horizontal=True, value=72).classes(
            'w-full').style('height: calc(100vh - 96px); min-height: 0')
        with splitter:
            with splitter.before:
                with ui.column().classes('w-full h-full gap-0'):
                    with ui.tabs().classes('w-full') as tabs:
                        tab_exp = ui.tab('Experiment control')
                        tab_cam = ui.tab('Camera control')
                        tab_top = ui.tab('Topics')
                        tab_oth = ui.tab('Other')
                    with ui.tab_panels(tabs, value=tab_exp).classes(
                            'w-full flex-grow'):
                        with ui.tab_panel(tab_exp):
                            self.experiment.build()
                        with ui.tab_panel(tab_cam):
                            self.camera_tab.build()
                        with ui.tab_panel(tab_top).classes('h-full'):
                            self.topics.build()
                        with ui.tab_panel(tab_oth):
                            self.other.build()

            with splitter.after:
                with ui.column().classes('w-full h-full gap-0 debug-pane'):
                    with ui.row().classes('items-center px-3 py-1 w-full debug-bar'):
                        ui.label('Debug').classes('eyebrow').style('color: #FFFFFF')
                        ui.space()
                        ui.button(icon='delete_sweep',
                                  on_click=lambda: self.log_pane.clear()
                                  ).props('flat dense round color=white')
                    self.log_pane = ui.log(max_lines=LOG_LINES).classes(
                        'w-full flex-grow font-mono text-xs')

    # --- periodic ------------------------------------------------------------

    def tick(self) -> None:
        # Compare the lock against THIS session's id, held in the closure.
        if _LOCK['client_id'] not in (None, self.client_id):
            if not self.lost:
                self.lost = True        # reload once, not once per tick
                log.info('Control taken by another tab - locking this one.')
                ui.navigate.reload()
            return
        _claim(self.client_id)

        signature = (self.recorder.recording, state.busy, state.outcome)
        if signature != self.signature:
            self.signature = signature
            self.experiment.refresh()

        connected = master_alive()
        state.ros_ok = connected
        self.ros_label.set_text('ROS master up' if connected else 'ROS MASTER DOWN')
        self.ros_label.style('color: {}'.format(
            theme.GREEN if connected else theme.RED))

        self.ns_label.set_text('ns: {}'.format(settings.ns))
        self.disk_label.set_text('{:.1f} GB free'.format(
            free_space_gb(settings.save_dir)))

        if self.recorder.poll():
            self.experiment.refresh()
        if self.video is not None:
            self.video.poll()

        if self.recorder.recording:
            elapsed = int(self.recorder.elapsed_s)
            self.rec_label.set_text('REC  {:03d}   {:02d}:{:02d}   {:.0f} MB'.format(
                self.recorder.active_id or 0, elapsed // 60, elapsed % 60,
                self.recorder.bytes_on_disk() / 1e6))
            self.rec_label.classes(add='rec-live')
            self.rec_label.style('color: {}'.format(theme.RED))
        else:
            self.rec_label.set_text('busy' if state.busy else 'idle')
            self.rec_label.classes(remove='rec-live')
            self.rec_label.style('color: {}'.format(
                theme.AMBER if state.busy else theme.MUTED))

        state.recording_guard = self.recorder.recording
        self.experiment.refresh_diagnostics()
        self.camera_tab.refresh_status()

    # --- keyboard ------------------------------------------------------------

    def on_key(self, event) -> None:
        """Function keys only - they cannot collide with typing in a field."""
        if not event.action.keydown:
            return
        key = str(event.key)
        recording = self.recorder.recording
        if key == 'F8' and not recording:
            self.experiment.start()
        elif key == 'F9' and recording:
            self.experiment.stop()
        elif key == 'F6' and recording:
            self.experiment.set_outcome(OUTCOME_SUCCESS)
        elif key == 'F7' and recording:
            self.experiment.set_outcome(OUTCOME_FAILURE)


class MainPage:
    """Holds only the shared services. All per-tab state lives in _Session."""

    def __init__(self, recorder, diagnostics, camera, on_reset, runners,
                 fault_pub, video=None) -> None:
        self.recorder = recorder
        self.diagnostics = diagnostics
        self.camera = camera
        self.on_reset = on_reset
        self.runners = runners
        self.fault_pub = fault_pub
        self.video = video

    def register(self) -> None:
        ui.page('/')(self._render)

    def _render(self, client: Client) -> None:
        ui.dark_mode().enable()
        theme.apply_theme()
        ui.query('.nicegui-content').classes(
            'h-screen w-full max-w-full p-0 gap-0 flex flex-col')

        granted = time.time() < float(_LOCK['grant_until'])
        if granted:
            _LOCK['grant_until'] = 0.0
        elif not _lock_is_free(client.id):
            # The beacon goes on the lock screen too, so reloading or closing a
            # locked-out tab leaves no claim behind.
            _install_release_beacon(client.id)
            _lock_screen()
            return

        _claim(client.id)
        _install_release_beacon(client.id)
        client.on_disconnect(lambda: _release(client.id))

        session = _Session(client.id, self.recorder, self.diagnostics,
                           self.camera, self.on_reset, self.runners,
                           self.fault_pub, self.video)
        session.build()

        self.recorder.on_change = session.experiment.refresh

        ui.keyboard(on_key=session.on_key)
        ui.timer(0.2, lambda: logbridge.drain(session.log_pane))
        ui.timer(1.0, session.tick)

        log.info('GUI ready. Saving to %s', settings.save_dir)
