"""Camera control for the standalone follow_cam model.

The camera is its own Gazebo model, pinned to the robot by software. Nothing
here touches the Astrobee URDF.

DIRECTIONS
----------
Offsets are in perch_cam axes: +X down, +Y right, +Z forward. Buttons are
written in operator terms ("up", "pan right") and _SIGN converts each to the
axis sign. To flip a direction, change one entry here.

    up/down        x     -1   perch_cam +X is DOWN
    right/left     y     -1   corrected against the live sim
    forward/back   z     +1
    pan right      pan   +1   corrected against the live sim
    tilt up        tilt  +1   corrected against the live sim
    roll clockwise roll  +1

These were derived from the frame algebra and three of them came out wrong in
practice, so the table is now what the simulator actually does, not what the
maths predicted. Verify against the image, not against a diagram.
"""

import math
import threading

from nicegui import ui

from ..config import (CAM_FOV_MAX_DEG, CAM_FOV_MIN_DEG, CAM_ROT_LIMIT_RAD,
                      CAM_ROTARY, CAM_STEP_DEG_MAX, CAM_STEP_DEG_MIN,
                      CAM_STEP_MM_MAX, CAM_STEP_MM_MIN, CAM_TRAVEL_M,
                      VIEWER_WINDOW_NAME, settings)
from ..follow_cam import FollowCamError
from ..logbridge import log
from ..state import state
from .. import theme

_SIGN = {'x': -1, 'y': -1, 'z': +1, 'pan': +1, 'tilt': +1, 'roll': +1}

_ROWS = (
    ('z', 'Forward', 'm'),
    ('y', 'Right', 'm'),
    ('x', 'Down', 'm'),
    ('pan', 'Pan  (+ = left)', 'deg'),
    ('tilt', 'Tilt (+ = down)', 'deg'),
    ('roll', 'Roll', 'deg'),
)


def _clamp(axis: str, value: float) -> float:
    limit = CAM_ROT_LIMIT_RAD if axis in CAM_ROTARY else CAM_TRAVEL_M
    return max(-limit, min(limit, value))


class CameraTab:

    def __init__(self, camera) -> None:
        self.camera = camera
        self._values = {}
        self._defaults = {}

    # --- build ---------------------------------------------------------------

    def build(self) -> None:
        with ui.row().classes('w-full gap-4 p-4 items-start no-wrap'):
            with ui.column().classes('gap-4').style('flex: 0 0 400px'):
                self._build_pad()
                self._build_rotation()
            with ui.column().classes('gap-4').style('flex: 1 1 0; min-width: 0'):
                self._build_readout()
                self._build_optics()
        self.refresh()

    def _arrow(self, icon: str, axis: str, direction: int, tip: str):
        """direction is +1/-1 in OPERATOR terms; _SIGN maps it to the axis.

        The step size is read when the button is pressed, not when it is built,
        so the sliders take effect immediately.
        """
        button = ui.button(icon=icon,
                           on_click=lambda: self._nudge(axis, direction))
        button.props('outline color=secondary').classes('w-14 h-12')
        button.tooltip(tip)
        return button

    def _build_pad(self) -> None:
        with ui.card().classes('w-full'):
            self.pad_title = ui.label('').classes('eyebrow')

            with ui.grid(columns=3).classes('gap-2 justify-items-center'):
                ui.label('')
                self._arrow('keyboard_arrow_up', 'x', +1, 'Up')
                ui.label('')

                self._arrow('keyboard_arrow_left', 'y', -1, 'Left')
                ui.button(icon='my_location', on_click=self._reset
                          ).props('flat color=warning').classes('w-14 h-12'
                          ).tooltip('Reset to saved default')
                self._arrow('keyboard_arrow_right', 'y', +1, 'Right')

                ui.label('')
                self._arrow('keyboard_arrow_down', 'x', -1, 'Down')
                ui.label('')

            ui.separator().classes('my-2')
            with ui.row().classes('items-center gap-2 justify-center w-full'):
                self._arrow('arrow_upward', 'z', +1, 'Forward, out of the lens')
                ui.label('forward / back').classes('text-xs').style(
                    'color: {}'.format(theme.MUTED))
                self._arrow('arrow_downward', 'z', -1, 'Back')

            ui.separator().classes('my-3')
            with ui.row().classes('items-center gap-3 w-full no-wrap'):
                ui.label('Linear').classes('text-sm w-16')
                self.step_mm_slider = ui.slider(
                    min=CAM_STEP_MM_MIN, max=CAM_STEP_MM_MAX, step=1,
                    value=settings.cam_step_mm,
                    on_change=self._on_step_mm).classes('flex-grow').props(
                        'color=secondary label-always')
                self.step_mm_value = ui.label('').classes(
                    'font-mono text-xs w-16 text-right')

            with ui.row().classes('items-center gap-3 w-full no-wrap'):
                ui.label('Angular').classes('text-sm w-16')
                self.step_deg_slider = ui.slider(
                    min=CAM_STEP_DEG_MIN, max=CAM_STEP_DEG_MAX, step=1,
                    value=settings.cam_step_deg,
                    on_change=self._on_step_deg).classes('flex-grow').props(
                        'color=secondary label-always')
                self.step_deg_value = ui.label('').classes(
                    'font-mono text-xs w-16 text-right')

            ui.label('Step per arrow click. FOV is unaffected by these.'
                     ).classes('text-xs mt-1').style(
                         'color: {}'.format(theme.MUTED))

    def _build_rotation(self) -> None:
        with ui.card().classes('w-full'):
            self.rot_title = ui.label('').classes('eyebrow')
            for label, axis, icons, tips in (
                    ('Tilt', 'tilt', ('expand_less', 'expand_more'),
                     ('Tilt up', 'Tilt down')),
                    ('Pan', 'pan', ('chevron_left', 'chevron_right'),
                     ('Pan left', 'Pan right')),
                    ('Roll', 'roll', ('rotate_left', 'rotate_right'),
                     ('Roll anticlockwise', 'Roll clockwise'))):
                with ui.row().classes('items-center gap-3 w-full mt-1'):
                    ui.label(label).classes('text-sm w-12')
                    self._arrow(icons[0], axis, -1, tips[0])
                    self._arrow(icons[1], axis, +1, tips[1])

    def _build_readout(self) -> None:
        with ui.card().classes('w-full'):
            ui.label('Offset from perch_cam').classes('eyebrow')
            with ui.grid(columns=4).classes('gap-x-4 gap-y-1 w-full items-center'):
                ui.label('')
                ui.label('current').classes('text-xs text-right').style(
                    'color: {}'.format(theme.MUTED))
                ui.label('default').classes('text-xs text-right').style(
                    'color: {}'.format(theme.MUTED))
                ui.label('')
                for axis, label, unit in _ROWS:
                    ui.label(label).classes('text-sm')
                    self._values[axis] = ui.label('').classes(
                        'font-mono text-sm text-right')
                    self._defaults[axis] = ui.label('').classes(
                        'font-mono text-xs text-right').style(
                            'color: {}'.format(theme.MUTED))
                    ui.label(unit).classes('text-xs').style(
                        'color: {}'.format(theme.MUTED))

            ui.separator().classes('my-3')
            with ui.row().classes('gap-2 flex-wrap'):
                ui.button('Set as default', icon='push_pin',
                          on_click=self._set_default
                          ).props('color=secondary unelevated')
                ui.button('Reset to default', icon='restart_alt',
                          on_click=self._reset).props('outline color=warning')
                ui.button('Zero', icon='exposure_zero',
                          on_click=self._zero).props('flat')

            ui.label('Moves take effect on the next pin, within ~16 ms. The '
                     'offset persists across restarts; "Set as default" is what '
                     '"Reset" returns to.'
                     ).classes('text-xs mt-2').style('color: {}'.format(theme.MUTED))

    def _build_optics(self) -> None:
        with ui.card().classes('w-full'):
            ui.label('Optics').classes('eyebrow')
            with ui.row().classes('items-center gap-4 w-full no-wrap'):
                self.fov_slider = ui.slider(
                    min=CAM_FOV_MIN_DEG, max=CAM_FOV_MAX_DEG, step=1,
                    value=settings.cam_fov_deg).classes('flex-grow').props(
                        'color=secondary label-always')
                self.fov_number = ui.number(
                    'FOV', value=settings.cam_fov_deg,
                    min=CAM_FOV_MIN_DEG, max=CAM_FOV_MAX_DEG, step=1,
                    ).props('outlined dense suffix=deg').classes('w-32')
            self.fov_slider.on('update:model-value',
                               lambda e: self.fov_number.set_value(e.args))
            self.fov_number.on('blur',
                               lambda _e: self.fov_slider.set_value(
                                   self.fov_number.value))

            with ui.row().classes('items-center gap-3 mt-2 flex-wrap'):
                self.respawn_btn = ui.button(
                    'Apply optics (respawns camera)', icon='refresh',
                    on_click=self._respawn).props('color=secondary unelevated')
                ui.button('Open image view', icon='open_in_new',
                          on_click=self._open_viewer).props('outline'
                          ).tooltip('Opens one dedicated window. Clicking again '
                                    'reuses it rather than opening another.')
                ui.button('Remove camera', icon='videocam_off',
                          on_click=self._despawn).props('flat color=warning')

            self.status = ui.label('').classes('text-xs mt-2')

            ui.label('FOV and resolution are baked into the model at spawn, so '
                     'changing them deletes and respawns it. The image topic '
                     'drops for about a second - never do this mid-recording.'
                     ).classes('text-xs mt-2').style('color: {}'.format(theme.AMBER))

    # --- actions -------------------------------------------------------------

    def _nudge(self, axis: str, direction: int) -> None:
        step = (settings.cam_step_rad if axis in CAM_ROTARY
                else settings.cam_step_m)
        delta = step * direction * _SIGN[axis]
        current = self.camera.offset.get(axis, 0.0)
        self.camera.offset[axis] = _clamp(axis, current + delta)
        settings.save()
        self.refresh()

    def _on_step_mm(self, event) -> None:
        settings.cam_step_mm = float(event.value or 1)
        settings.save()
        self._update_step_labels()

    def _on_step_deg(self, event) -> None:
        settings.cam_step_deg = float(event.value or 1)
        settings.save()
        self._update_step_labels()

    def _update_step_labels(self) -> None:
        self.pad_title.set_text(
            'Translate  ({:.0f} mm per click, perch_cam axes)'.format(
                settings.cam_step_mm))
        self.rot_title.set_text(
            'Rotate  ({:.0f} deg per click)'.format(settings.cam_step_deg))
        self.step_mm_value.set_text('{:.0f} mm'.format(settings.cam_step_mm))
        self.step_deg_value.set_text('{:.0f} deg'.format(settings.cam_step_deg))

    def _reset(self) -> None:
        self.camera.reset()
        self.refresh()

    def _zero(self) -> None:
        self.camera.zero()
        self.refresh()

    def _set_default(self) -> None:
        self.camera.set_as_default()
        self.refresh()
        ui.notify('Saved as the default offset.', type='positive')

    def _respawn(self) -> None:
        if state.recording_guard:
            ui.notify('Not while recording - the image topic would drop.',
                      type='warning')
            return
        settings.cam_fov_deg = float(self.fov_number.value or 90)
        settings.save()
        self._set_status('Respawning ...', theme.AMBER)

        def worker() -> None:
            try:
                self.camera.spawn()
            except FollowCamError as exc:
                log.error('follow_cam respawn failed: %s', exc)

        threading.Thread(target=worker, daemon=True).start()

    def _open_viewer(self) -> None:
        """Named window target, not _blank.

        With '_blank' every click spawns another window. A named target makes
        the BROWSER own the lifecycle: reuse if open, create if closed. No
        server-side tracking, nothing for the GUI to get out of sync with.

        Note it reuses without reloading, so a namespace change will not reach
        an already-open viewer - use the reload link in its overlay.
        """
        ui.run_javascript(
            "window.open('/view', '{}');".format(VIEWER_WINDOW_NAME))

    def _despawn(self) -> None:
        threading.Thread(target=self.camera.despawn, daemon=True).start()
        self._set_status('Removed.', theme.MUTED)

    def _set_status(self, text: str, colour: str) -> None:
        self.status.set_text(text)
        self.status.style('color: {}'.format(colour))

    # --- refresh -------------------------------------------------------------

    def refresh(self) -> None:
        self._update_step_labels()
        offset = self.camera.offset
        defaults = self.camera.defaults
        for axis, _label, unit in _ROWS:
            raw = offset.get(axis, 0.0)
            default = defaults.get(axis, 0.0)
            if unit == 'deg':
                self._values[axis].set_text('{:+8.1f}'.format(math.degrees(raw)))
                self._defaults[axis].set_text('{:+.1f}'.format(math.degrees(default)))
            else:
                self._values[axis].set_text('{:+8.3f}'.format(raw))
                self._defaults[axis].set_text('{:+.3f}'.format(default))

    def refresh_status(self) -> None:
        """Called from the page tick."""
        if self.camera.spawned:
            self._set_status('Camera live, pinned to {}.'.format(settings.ns),
                             theme.GREEN)
        else:
            self._set_status('Camera not spawned.', theme.MUTED)
        self.respawn_btn.set_enabled(not state.recording_guard)
