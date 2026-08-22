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
                      TORQUE_MAX_NM, TOOLS, settings)
from ..diagnostics import DOWN, OK, STALE, UNKNOWN, WARN
from ..logbridge import log
from ..naming import build_folder_name
from ..recorder import RecorderError
from ..reset import clear_fault
from ..ros_link import call_set_bool
from ..runners import RunnerError
from ..state import state
from .. import theme

_LED_COLOUR = {
    OK: theme.GREEN,
    WARN: theme.AMBER,
    STALE: theme.AMBER,
    DOWN: theme.RED,
    UNKNOWN: theme.GREY,
}

# Diagnostics rows, top to bottom. The order is the signal chain: robot state
# first, then joystick in, converted, out to the FAM.
#
# label None means "take it from the RunnerSet", so a runner's name lives in
# exactly one place - the switch and its LED cannot end up disagreeing.
_DIAG_ORDER = (
    ('fault', 'Astrobee State', 'mgt/sys_monitor/state'),
    ('points', 'PerchCam Point Cloud', 'hw/depth_perch/points'),
    ('joy_node', None, None),
    ('joy', 'Gamepad Data', '/joy'),
    ('joy_convert', None, None),
    ('fam_control', None, None),
    ('start', 'Custom FAM Control Started', 'start'),
)


class ExperimentTab:

    def __init__(self, recorder, diagnostics, on_reset, camera, runners,
                 fault_pub) -> None:
        self.recorder = recorder
        self.diagnostics = diagnostics
        self.on_reset = on_reset
        self.camera = camera
        self.runners = runners
        self.fault_pub = fault_pub
        from_model = {r.key: (r.label, r.command) for r in runners}
        self.diag_rows = tuple(
            (key,) + (from_model[key] if label is None else (label, hint))
            for key, label, hint in _DIAG_ORDER)
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
        """Runners, then toggles.

        The three runners are subprocesses, so their switches show what is
        actually alive - poll() clears the flag if one dies and the switch
        follows. The Custom FAM Control Start switch is a service call, which
        has no readable state, so that one shows only what the GUI last
        commanded; anything else touching /honey/start goes unnoticed.
        """
        with ui.card().classes('w-full'):
            ui.label('Control services').classes('eyebrow')

            for runner in self.runners:
                with ui.row().classes('items-center gap-4 w-full'):
                    # Seeded from intent, not from poll(): a new browser tab
                    # should show what was asked for, and nothing writes here
                    # afterwards.
                    ui.switch(runner.label, value=runner.desired,
                              on_change=lambda e, key=runner.key:
                                  self._toggle_runner(key, e.value))
                ui.label(runner.command).classes(
                    'font-mono text-xs break-all -mt-2 ml-14').style(
                        'color: {}'.format(theme.MUTED))

            ui.separator().classes('my-2')

            with ui.row().classes('items-center gap-4 w-full'):
                self.start_switch = ui.switch(
                    'Custom FAM Control Start',
                    value=bool(state.custom_start),
                    on_change=lambda e: self._call_service(
                        settings.topic('start'), e.value))
            with ui.row().classes('items-center gap-4 w-full'):
                ui.button('Clear Astrobee Fault State', icon='clear_all',
                          on_click=self._clear_fault).props('outline color=warning')
            self.service_status = ui.label('').classes('text-xs').style(
                'color: {}'.format(theme.MUTED))

    def _toggle_runner(self, key: str, value: bool) -> None:
        """Start/stop blocks for up to STOP_TIMEOUT_S - keep it off the loop."""

        def worker() -> None:
            try:
                self.runners.set_running(key, value)
            except RunnerError as exc:
                log.error('%s', exc)

        self.service_status.set_text(
            '{} {} ...'.format('Starting' if value else 'Stopping',
                               self.runners.get(key).label))
        threading.Thread(target=worker, daemon=True).start()

    def _clear_fault(self) -> None:
        """One-fire, not a toggle: publish FaultState 0 and return."""

        def worker() -> None:
            result = clear_fault(settings, self.fault_pub)
            if not result.get('published'):
                log.error('Clear fault state failed: %s',
                          result.get('message') or 'unknown error')

        self.service_status.set_text('Clearing fault state ...')
        threading.Thread(target=worker, daemon=True).start()

    def _call_service(self, name: str, value: bool) -> None:
        """Service calls block for up to 2 s - keep them off the event loop."""

        def worker() -> None:
            ok, message = call_set_bool(name, value)
            state.custom_start_failed = not ok
            if ok:
                state.custom_start = value
                log.info('%s -> %s (%s)', name, value, message or 'ok')
            else:
                # custom_start is deliberately left as it was: a refused call
                # tells us nothing new about the node, only that we do not
                # know. The LED goes amber on the flag.
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

            ui.label('Perturbation magnitude').classes('eyebrow mt-3')
            with ui.row().classes('items-center gap-3 w-full no-wrap'):
                ui.label('Force').classes('text-sm w-16')
                self.force_slider = ui.slider(
                    min=0.0, max=FORCE_MAX_N, step=0.05,
                    value=settings.max_force_n,
                    on_change=self._on_force).classes('flex-grow').props(
                        'color=secondary label-always')
                self.force_value = ui.label('').classes(
                    'font-mono text-xs w-20 text-right')

            with ui.row().classes('items-center gap-3 w-full no-wrap'):
                ui.label('Torque').classes('text-sm w-16')
                self.torque_slider = ui.slider(
                    min=0.0, max=TORQUE_MAX_NM, step=0.05,
                    value=settings.max_torque_nm,
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
            for key, label, hint in self.diag_rows:
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
        # 'reset' carries its own timestamp, so a run recorded long after the
        # last reset - or with no reset at all, where it is null - is visible
        # in the metadata rather than implied by it.
        self.recorder.stop(state.outcome, extra_meta={
            'cameras': self.camera.summary(),
            'robot_ns': settings.ns,
            'reset': state.last_reset,
        })
        self.refresh()

    def confirm_reset(self) -> None:
        with ui.dialog() as dialog, ui.card():
            ui.label('Reset the experiment?').classes('text-lg')
            steps = ['Clear the system monitor fault state',
                     'Remove any spawned tool, then spawn "{}" in front of the '
                     'perch cam with a random pose and a small impulse'
                     .format(settings.tool_label)]
            if settings.home_pose:
                steps.insert(0, 'Return the robot to its home pose')
            else:
                steps.insert(0, 'Leave the robot where it is - no home pose '
                                'captured yet')
            for index, step in enumerate(steps, 1):
                ui.label('{}. {}'.format(index, step)).classes(
                    'text-sm').style('color: {}'.format(theme.MUTED))
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
        settings.max_torque_nm = float(event.value or 0.0)
        settings.save()
        self._update_impulse_hint()

    def _update_impulse_hint(self) -> None:
        """Show what the sliders actually mean in units the operator cares about.

        Computed from the tool's real mass and inertia (config.py), read off
        the SDF. The figures are a prediction; the log prints the MEASURED
        velocity after every reset, and that is the one to believe.
        """
        self.force_value.set_text('{:.2f} N'.format(settings.max_force_n))
        self.torque_value.set_text('{:.2f} Nm'.format(settings.max_torque_nm))

        speed = settings.max_force_n * IMPULSE_S / NOMINAL_TOOL_MASS_KG
        spin = math.degrees(
            settings.max_torque_nm * IMPULSE_S / NOMINAL_TOOL_INERTIA)
        self.impulse_hint.set_text(
            'Up to {:.3f} m/s and {:.0f} deg/s.'.format(speed, spin))

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
        for key, _label, hint in self.diag_rows:
            reading = snapshot.get(key)
            if reading is None:
                continue
            colour = _LED_COLOUR.get(reading.status, theme.GREY)
            self._leds[key].style('color: {c}; background: {c}'.format(c=colour))
            self._details[key].set_text(reading.detail or hint)

        # Nothing here writes to a switch. The switches carry operator intent
        # and move only when the operator moves them; the LEDs above carry
        # what was observed. A mismatch is the signal, not something to
        # silently correct. (The runner watchdog runs on the diagnostics
        # thread - the view does not drive the model.)
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
