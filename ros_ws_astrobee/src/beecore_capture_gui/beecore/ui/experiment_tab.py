"""Experiment control.

Left column  - everything configurable: run control, metadata, outcome,
               tool selection and perturbation axes.
Right column - read-only diagnostics.

The namespace input sits at the top of the right column because it
parameterises every reading below it.
"""

import math
import os
import re
import threading

from nicegui import ui

from ..config import (FAULT_STATES, FORCE_MAX_N, IMPULSE_S,
                      NOMINAL_TOOL_INERTIA, NOMINAL_TOOL_MASS_KG,
                      OUTCOME_FAILURE, OUTCOME_NA, OUTCOME_SUCCESS,
                      TORQUE_MAX_MNM, TOOLS, settings)
from ..diagnostics import DOWN, OK, STALE, UNKNOWN
from ..logbridge import log
from ..naming import build_folder_name
from ..recorder import RecorderError
from ..ros_link import call_set_bool
from ..state import state
from .. import theme

_LED_COLOUR = {
    OK: theme.GREEN,
    STALE: theme.AMBER,
    DOWN: theme.RED,
    UNKNOWN: theme.GREY,
}

# key -> (label, hint shown under the label)
_DIAG_ROWS = (
    ('joy', 'Gamepad', '/joy'),
    ('points', 'Depth perch cloud', 'hw/depth_perch/points'),
    ('fault', 'Astrobee state', 'mgt/sys_monitor/state'),
    ('gnc', 'GNC disarmed', 'gnc/ctl/enable  (green = OFF)'),
    ('start', 'Custom control', 'start'),
)


class ExperimentTab:

    def __init__(self, recorder, diagnostics, on_reset, camera) -> None:
        self.recorder = recorder
        self.diagnostics = diagnostics
        self.on_reset = on_reset
        self.camera = camera
        self._leds = {}
        self._details = {}

    # --- build ---------------------------------------------------------------

    def build(self) -> None:
        with ui.row().classes('w-full gap-4 p-4 items-start no-wrap'):
            with ui.column().classes('gap-4').style('flex: 1 1 0; min-width: 0'):
                self._build_run_control()
                self._build_metadata()
                self._build_outcome()
                self._build_control()
                self._build_tools()
            with ui.column().classes('gap-4').style('flex: 0 0 340px'):
                self._build_diagnostics()
        self.refresh()

    # --- left column ---------------------------------------------------------

    def _build_run_control(self) -> None:
        with ui.card().classes('w-full'):
            ui.label('Run control').classes('eyebrow')
            with ui.row().classes('items-center gap-3 w-full'):
                self.start_btn = ui.button(
                    'Start recording', icon='fiber_manual_record',
                    on_click=self.start).props('color=positive unelevated').classes('w-48')
                self.stop_btn = ui.button(
                    'Stop recording', icon='stop',
                    on_click=self.stop).props('color=negative unelevated').classes('w-48')
            with ui.row().classes('items-center gap-3 w-full'):
                self.reset_btn = ui.button(
                    'Reset experiment', icon='restart_alt',
                    on_click=self.confirm_reset).props('outline color=warning')
            self.stop_tip = ui.label('').classes('text-xs').style(
                'color: {}'.format(theme.MUTED))

    def _build_metadata(self) -> None:
        with ui.card().classes('w-full'):
            ui.label('Run metadata').classes('eyebrow')
            with ui.row().classes('items-center gap-4 w-full'):
                self.id_input = ui.input(
                    'Experiment ID',
                    value='{:03d}'.format(settings.experiment_id),
                    on_change=self._on_id_change,
                ).props('outlined dense').classes('w-32')
                self.suffix_input = ui.input(
                    'Bag suffix', value=settings.suffix,
                    placeholder='e.g. handrail grasp',
                    on_change=self._on_suffix_change,
                ).props('outlined dense').classes('flex-grow')
            ui.label('Next bag folder').classes('eyebrow mt-2')
            self.preview = ui.label('').classes('font-mono text-xs break-all').style(
                'color: {}'.format(theme.VIOLET))

    def _build_outcome(self) -> None:
        with ui.card().classes('w-full'):
            ui.label('Capture outcome').classes('eyebrow')
            self.outcome_toggle = ui.toggle(
                {OUTCOME_NA: 'N/A',
                 OUTCOME_SUCCESS: 'Capture success',
                 OUTCOME_FAILURE: 'Capture failure'},
                value=OUTCOME_NA,
                on_change=self._on_outcome_change).props('no-caps unelevated')
            ui.label('Resets to N/A when recording starts. Stop unlocks once this '
                     'is set. F6 marks success, F7 marks failure.'
                     ).classes('text-xs').style('color: {}'.format(theme.MUTED))

    def _build_control(self) -> None:
        """The two SetBool services.

        A service has no readable state, so these switches are also the only
        thing that can move the matching indicators: the GUI shows what it
        last commanded. Anything else touching the service goes unnoticed.
        """
        with ui.card().classes('w-full'):
            ui.label('Control services').classes('eyebrow')
            with ui.row().classes('items-center gap-4 w-full'):
                self.gnc_switch = ui.switch(
                    'GNC control enabled',
                    value=bool(state.gnc_enabled),
                    on_change=lambda e: self._call_service(
                        'gnc', settings.topic('gnc/ctl/enable'), e.value))
            with ui.row().classes('items-center gap-4 w-full'):
                self.start_switch = ui.switch(
                    'Custom control started',
                    value=bool(state.custom_start),
                    on_change=lambda e: self._call_service(
                        'start', settings.topic('start'), e.value))
            self.service_status = ui.label('').classes('text-xs').style(
                'color: {}'.format(theme.MUTED))

    def _call_service(self, key: str, name: str, value: bool) -> None:
        """Service calls block for up to 2 s - keep them off the event loop."""

        def worker() -> None:
            ok, message = call_set_bool(name, value)
            if ok:
                if key == 'gnc':
                    state.gnc_enabled = value
                else:
                    state.custom_start = value
                log.info('%s -> %s (%s)', name, value, message or 'ok')
            else:
                log.error('%s failed: %s', name, message)

        self.service_status.set_text('Calling {} ...'.format(name))
        threading.Thread(target=worker, daemon=True).start()

    def _build_tools(self) -> None:
        with ui.card().classes('w-full'):
            ui.label('Tool spawn').classes('eyebrow')
            self.tool_select = ui.select(
                list(TOOLS.keys()), value=settings.tool_label,
                label='Tool', on_change=self._on_tool_change,
            ).props('outlined dense').classes('w-64')

            ui.label('Initial perturbation axes').classes('eyebrow mt-3')
            with ui.grid(columns=4).classes('gap-x-4 gap-y-1 items-center'):
                ui.label('').classes('text-xs')
                for axis in ('X', 'Y', 'Z'):
                    ui.label(axis).classes('text-xs text-center').style(
                        'color: {}'.format(theme.MUTED))

                ui.label('Force').classes('text-sm')
                for axis in ('x', 'y', 'z'):
                    ui.checkbox(
                        value=settings.force_axes.get(axis, True),
                        on_change=lambda e, a=axis: self._on_axis('force_axes', a, e.value),
                    ).props('dense color=secondary').classes('justify-center')

                ui.label('Torque').classes('text-sm')
                for axis in ('x', 'y', 'z'):
                    ui.checkbox(
                        value=settings.torque_axes.get(axis, True),
                        on_change=lambda e, a=axis: self._on_axis('torque_axes', a, e.value),
                    ).props('dense color=secondary').classes('justify-center')

            ui.label('Unticked axes get no perturbation on that component. '
                     'Force and torque are independent, so X torque can be on '
                     'while X force is off.'
                     ).classes('text-xs mt-2').style('color: {}'.format(theme.MUTED))

            ui.label('Perturbation magnitude').classes('eyebrow mt-3')
            with ui.row().classes('items-center gap-3 w-full no-wrap'):
                ui.label('Force').classes('text-sm w-16')
                self.force_slider = ui.slider(
                    min=0.0, max=FORCE_MAX_N, step=0.01,
                    value=settings.max_force_n,
                    on_change=self._on_force).classes('flex-grow').props(
                        'color=secondary label-always')
                self.force_value = ui.label('').classes(
                    'font-mono text-xs w-20 text-right')

            with ui.row().classes('items-center gap-3 w-full no-wrap'):
                ui.label('Torque').classes('text-sm w-16')
                self.torque_slider = ui.slider(
                    min=0.0, max=TORQUE_MAX_MNM, step=0.1,
                    value=settings.max_torque_mnm,
                    on_change=self._on_torque).classes('flex-grow').props(
                        'color=secondary label-always')
                self.torque_value = ui.label('').classes(
                    'font-mono text-xs w-20 text-right')

            self.impulse_hint = ui.label('').classes('text-xs mt-1').style(
                'color: {}'.format(theme.MUTED))
            self._update_impulse_hint()

    # --- right column --------------------------------------------------------

    def _build_diagnostics(self) -> None:
        with ui.card().classes('w-full'):
            ui.label('Robot').classes('eyebrow')
            ui.input('Namespace', value=settings.robot_ns,
                     on_change=self._on_ns_change,
                     ).props('outlined dense').classes('w-full')

        with ui.card().classes('w-full'):
            ui.label('Diagnostics').classes('eyebrow')
            for key, label, hint in _DIAG_ROWS:
                with ui.row().classes('items-center gap-3 w-full no-wrap py-1'):
                    led = ui.element('div').classes('led')
                    led.style('color: {c}; background: {c}'.format(c=theme.GREY))
                    self._leds[key] = led
                    with ui.column().classes('gap-0 min-w-0'):
                        ui.label(label).classes('text-sm leading-tight')
                        detail = ui.label(hint).classes(
                            'text-xs leading-tight truncate').style(
                                'color: {}'.format(theme.MUTED))
                        self._details[key] = detail

            ui.separator().classes('my-2')
            ui.label('ff_msgs/FaultState').classes('eyebrow')
            self.fault_label = ui.label('unknown').classes('font-mono text-sm')

    # --- actions -------------------------------------------------------------

    def start(self) -> None:
        try:
            folder = self.recorder.start(settings.experiment_id, settings.suffix)
        except RecorderError as exc:
            ui.notify(str(exc), type='negative')
            log.error('Start refused: %s', exc)
            return

        state.outcome = OUTCOME_NA
        self.outcome_toggle.set_value(OUTCOME_NA)
        settings.experiment_id += 1
        settings.save()
        self.refresh()
        ui.notify('Recording {}'.format(folder), type='positive')

    def stop(self) -> None:
        if state.outcome == OUTCOME_NA:
            ui.notify('Set the capture outcome before stopping.', type='warning')
            return
        self.recorder.stop(state.outcome, extra_meta={
            'camera_offset': self.camera.summary(),
            'camera_fov_deg': settings.cam_fov_deg,
            'robot_ns': settings.ns,
        })
        self.refresh()

    def confirm_reset(self) -> None:
        with ui.dialog() as dialog, ui.card():
            ui.label('Reset the experiment?').classes('text-lg')
            ui.label('Removes any spawned tool, then spawns "{}" in front of '
                     'the perch cam with a random pose and a small impulse.'
                     .format(settings.tool_label)
                     ).classes('text-sm').style('color: {}'.format(theme.MUTED))
            if self.recorder.recording:
                ui.label('A recording is in progress. Stop it first.'
                         ).classes('text-sm').style('color: {}'.format(theme.RED))
            with ui.row().classes('w-full justify-end'):
                ui.button('Cancel', on_click=dialog.close).props('flat')
                confirm = ui.button(
                    'Reset', on_click=lambda: (dialog.close(), self.on_reset())
                ).props('color=warning unelevated')
                confirm.set_enabled(not self.recorder.recording)
        dialog.open()

    # --- handlers ------------------------------------------------------------

    def _on_id_change(self, event) -> None:
        digits = re.sub(r'\D', '', str(event.value or ''))
        settings.experiment_id = int(digits) if digits else 0
        settings.save()
        self._update_preview()

    def _on_suffix_change(self, event) -> None:
        settings.suffix = str(event.value or '')
        settings.save()
        self._update_preview()

    def _on_outcome_change(self, event) -> None:
        state.outcome = event.value or OUTCOME_NA
        self.refresh()

    def _on_ns_change(self, event) -> None:
        settings.robot_ns = str(event.value or '').strip() or 'honey'
        settings.save()
        log.info('Robot namespace set to "%s".', settings.ns)

    def _on_tool_change(self, event) -> None:
        settings.tool_label = event.value or 'Ratchet Wrench'
        settings.save()

    def _on_force(self, event) -> None:
        settings.max_force_n = float(event.value or 0.0)
        settings.save()
        self._update_impulse_hint()

    def _on_torque(self, event) -> None:
        settings.max_torque_mnm = float(event.value or 0.0)
        settings.save()
        self._update_impulse_hint()

    def _update_impulse_hint(self) -> None:
        """Show what the sliders actually mean in units the operator cares about."""
        self.force_value.set_text('{:.2f} N'.format(settings.max_force_n))
        self.torque_value.set_text('{:.1f} mNm'.format(settings.max_torque_mnm))

        speed = settings.max_force_n * IMPULSE_S / NOMINAL_TOOL_MASS_KG
        spin = math.degrees(
            settings.max_torque_nm * IMPULSE_S / NOMINAL_TOOL_INERTIA)
        self.impulse_hint.set_text(
            'The impulse is applied for {:.1f} s, then the tool coasts. At these '
            'maxima a nominal {:.1f} kg tool leaves at up to {:.3f} m/s and '
            '{:.0f} deg/s. Each axis is scaled randomly within that.'.format(
                IMPULSE_S, NOMINAL_TOOL_MASS_KG, speed, spin))

    def _on_axis(self, group: str, axis: str, value: bool) -> None:
        getattr(settings, group)[axis] = bool(value)
        settings.save()

    def set_outcome(self, outcome: str) -> None:
        """Called by the keyboard shortcuts."""
        if self.recorder.recording:
            self.outcome_toggle.set_value(outcome)

    # --- refresh -------------------------------------------------------------

    def _update_preview(self) -> None:
        folder = build_folder_name(settings.experiment_id, settings.suffix)
        self.preview.set_text(os.path.join(settings.save_dir, folder) + '/')

    def refresh(self) -> None:
        recording = self.recorder.recording

        self.start_btn.set_enabled(not recording and not state.busy)
        self.stop_btn.set_enabled(recording and state.outcome != OUTCOME_NA)
        self.reset_btn.set_enabled(not recording and not state.busy)
        self.id_input.set_enabled(not recording)
        self.suffix_input.set_enabled(not recording)
        self.outcome_toggle.set_enabled(recording)

        if state.busy:
            tip = 'Resetting - spawning tool ...'
        elif not recording:
            tip = 'Not recording.'
        elif state.outcome == OUTCOME_NA:
            tip = 'Set the capture outcome first - Stop stays locked until you do.'
        else:
            tip = 'Stop closes the bag and writes the {} marker.'.format(state.outcome)
        self.stop_tip.set_text(tip)

        self.id_input.value = '{:03d}'.format(settings.experiment_id)
        self._update_preview()

    def refresh_diagnostics(self) -> None:
        snapshot = self.diagnostics.snapshot()
        for key, _label, hint in _DIAG_ROWS:
            reading = snapshot.get(key)
            if reading is None:
                continue
            colour = _LED_COLOUR.get(reading.status, theme.GREY)
            self._leds[key].style('color: {c}; background: {c}'.format(c=colour))
            self._details[key].set_text(reading.detail or hint)

        if state.gnc_enabled is not None:
            self.gnc_switch.value = state.gnc_enabled
        if state.custom_start is not None:
            self.start_switch.value = state.custom_start
        self.service_status.set_text('')

        value = self.diagnostics.fault_value
        if value is None:
            self.fault_label.set_text('unknown')
            self.fault_label.style('color: {}'.format(theme.GREY))
        else:
            self.fault_label.set_text('{}  =  {}'.format(
                value, FAULT_STATES.get(value, 'UNRECOGNISED')))
            self.fault_label.style('color: {}'.format(
                theme.GREEN if value == 1 else theme.AMBER))
