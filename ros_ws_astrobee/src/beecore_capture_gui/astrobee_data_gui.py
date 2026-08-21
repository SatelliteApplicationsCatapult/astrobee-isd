#!/usr/bin/env python3
"""
Astrobee data-collection GUI (ROS Noetic / Python 3.8 / NiceGUI 2.x)

Run:  python3 astrobee_data_gui.py
Then: http://<host>:8080

Requires:  pip install "nicegui<3"     # NiceGUI 3.0 dropped Python 3.8
"""

import json
import logging
import os
import queue
import re
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional

import rosbag
import rospy
from nicegui import Client, app, ui

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------

DEFAULT_SAVE_DIR = '/src/astrobee-isd/data/training/'
CONFIG_PATH = os.path.expanduser('~/.astrobee_data_gui.json')
NODE_NAME = 'astrobee_data_gui'
BAG_NODE_NAME = 'astrobee_gui_bag_recorder'   # so we can find/kill it if orphaned
LOG_LINES = 500
GUI_PORT = 8080

OUTCOME_NA = 'N/A'
OUTCOME_SUCCESS = 'SUCCESS'
OUTCOME_FAILURE = 'FAILURE'


# ----------------------------------------------------------------------------
# Logging -> debug pane
#
# ROS callbacks and the rosbag watchdog run on non-UI threads. NiceGUI elements
# must not be touched from those threads, so everything funnels through a queue
# that a ui.timer drains on the UI thread.
# ----------------------------------------------------------------------------

LOG_QUEUE: 'queue.Queue[str]' = queue.Queue(maxsize=5000)


class _QueueHandler(logging.Handler):
    def emit(self, record: logging.LogRecord) -> None:
        try:
            LOG_QUEUE.put_nowait(self.format(record))
        except queue.Full:
            pass


log = logging.getLogger('astrobee_gui')
log.setLevel(logging.INFO)
_handler = _QueueHandler()
_handler.setFormatter(logging.Formatter('%(asctime)s  %(levelname)-7s %(message)s',
                                        datefmt='%H:%M:%S'))
log.addHandler(_handler)
log.addHandler(logging.StreamHandler())
logging.getLogger('rosout').addHandler(_handler)   # rospy.loginfo etc.


# ----------------------------------------------------------------------------
# Application state (single global instance - this mirrors the robot, not a tab)
# ----------------------------------------------------------------------------

@dataclass
class State:
    # persisted
    save_dir: str = DEFAULT_SAVE_DIR
    suffix: str = ''
    experiment_id: int = 1            # ID that the NEXT recording will use
    wanted_topics: List[str] = field(default_factory=list)
    record_all: bool = False
    buffer_mb: int = 1024
    split_mb: int = 0                 # 0 = no splitting

    # runtime
    recording: bool = False
    outcome: str = OUTCOME_NA
    proc: Optional[subprocess.Popen] = None
    bag_dir: Optional[str] = None
    bag_path: Optional[str] = None
    started_at: Optional[float] = None
    active_id: Optional[int] = None
    available_topics: List[str] = field(default_factory=list)


state = State()
UI: Dict[str, object] = {}            # element registry (single controlling client)


# ----------------------------------------------------------------------------
# Persistence
# ----------------------------------------------------------------------------

_PERSISTED = ('save_dir', 'suffix', 'experiment_id', 'wanted_topics',
              'record_all', 'buffer_mb', 'split_mb')


def load_config() -> None:
    if not os.path.isfile(CONFIG_PATH):
        return
    try:
        with open(CONFIG_PATH) as fh:
            data = json.load(fh)
        for key in _PERSISTED:
            if key in data:
                setattr(state, key, data[key])
        log.info('Loaded settings from %s', CONFIG_PATH)
    except Exception as exc:                                   # noqa: BLE001
        log.warning('Could not read %s (%s). Using defaults.', CONFIG_PATH, exc)


def save_config() -> None:
    try:
        with open(CONFIG_PATH, 'w') as fh:
            json.dump({k: getattr(state, k) for k in _PERSISTED}, fh, indent=2)
    except Exception as exc:                                   # noqa: BLE001
        log.warning('Could not write %s (%s). Settings will not persist.',
                    CONFIG_PATH, exc)


# ----------------------------------------------------------------------------
# Naming
# ----------------------------------------------------------------------------

_SAFE = re.compile(r'[^A-Za-z0-9_\-]')
_FOLDER_RE = re.compile(r'^\d{8}_\d{6}_(\d{3})')


def sanitise_suffix(raw: str) -> str:
    """Spaces -> underscores, strip anything else that is awkward in a path."""
    cleaned = _SAFE.sub('_', raw.strip().replace(' ', '_'))
    return re.sub(r'_+', '_', cleaned).strip('_')


def build_folder_name(exp_id: int, suffix: str, when: Optional[datetime] = None) -> str:
    stamp = (when or datetime.now()).strftime('%Y%m%d_%H%M%S')
    parts = [stamp, '{:03d}'.format(exp_id)]
    clean = sanitise_suffix(suffix)
    if clean:
        parts.append(clean)
    return '_'.join(parts)


def scan_next_experiment_id(save_dir: str) -> Optional[int]:
    """Highest ID already on disk, +1. Keeps IDs unique across GUI restarts."""
    if not os.path.isdir(save_dir):
        return None
    highest = 0
    for name in os.listdir(save_dir):
        match = _FOLDER_RE.match(name)
        if match:
            highest = max(highest, int(match.group(1)))
    return highest + 1 if highest else None


# ----------------------------------------------------------------------------
# ROS
# ----------------------------------------------------------------------------

def init_ros() -> bool:
    try:
        # disable_signals: uvicorn owns SIGINT/SIGTERM, not rospy.
        rospy.init_node(NODE_NAME, anonymous=False, disable_signals=True)
        log.info('ROS node "%s" registered with master.', NODE_NAME)
        return True
    except Exception as exc:                                   # noqa: BLE001
        log.error('ROS init failed: %s. GUI will run, recording is disabled.', exc)
        return False


def master_alive() -> bool:
    try:
        rospy.get_master().getPid()
        return True
    except Exception:                                          # noqa: BLE001
        return False


def list_topics() -> List[str]:
    try:
        return sorted(name for name, _type in rospy.get_published_topics())
    except Exception as exc:                                   # noqa: BLE001
        log.error('Could not list topics: %s', exc)
        return []


# ----------------------------------------------------------------------------
# Recording
# ----------------------------------------------------------------------------

def build_record_command(bag_path: str) -> List[str]:
    cmd = ['rosbag', 'record', '-O', bag_path,
           '--buffsize={}'.format(state.buffer_mb),
           '__name:={}'.format(BAG_NODE_NAME)]
    if state.split_mb > 0:
        cmd += ['--split', '--size={}'.format(state.split_mb)]
    if state.record_all or not state.wanted_topics:
        cmd.append('-a')
    else:
        cmd += list(state.wanted_topics)
    return cmd


def start_recording() -> None:
    if state.recording:
        return
    if not master_alive():
        ui.notify('No ROS master. Start roscore before recording.', type='negative')
        log.error('Start refused: no ROS master.')
        return
    if not state.record_all and not state.wanted_topics:
        ui.notify('No topics selected. Pick topics on the Topics tab, '
                  'or tick "Record all topics".', type='warning')
        log.warning('Start refused: topic selection is empty.')
        return

    exp_id = state.experiment_id
    folder = build_folder_name(exp_id, state.suffix)
    bag_dir = os.path.join(state.save_dir, folder)

    try:
        os.makedirs(bag_dir, exist_ok=False)
    except FileExistsError:
        ui.notify('Folder already exists: {}'.format(bag_dir), type='negative')
        log.error('Start refused: %s already exists.', bag_dir)
        return
    except OSError as exc:
        ui.notify('Cannot create {}: {}'.format(bag_dir, exc), type='negative')
        log.error('Start refused: cannot create %s (%s)', bag_dir, exc)
        return

    bag_path = os.path.join(bag_dir, folder + '.bag')
    cmd = build_record_command(bag_path)

    try:
        # setsid: rosbag record spawns children. We need to signal the whole
        # process group, otherwise SIGINT leaves a half-written .bag.active.
        proc = subprocess.Popen(cmd, preexec_fn=os.setsid, cwd=bag_dir)
    except OSError as exc:
        ui.notify('Could not launch rosbag record: {}'.format(exc), type='negative')
        log.error('rosbag record failed to launch: %s', exc)
        shutil.rmtree(bag_dir, ignore_errors=True)
        return

    state.proc = proc
    state.recording = True
    state.bag_dir = bag_dir
    state.bag_path = bag_path
    state.started_at = time.time()
    state.active_id = exp_id
    state.outcome = OUTCOME_NA

    state.experiment_id = exp_id + 1
    save_config()

    log.info('Recording %s (%s)', folder,
             'all topics' if (state.record_all or not state.wanted_topics)
             else '{} topics'.format(len(state.wanted_topics)))
    log.info('Command: %s', ' '.join(cmd))
    refresh_controls()


def stop_recording() -> None:
    if not state.recording or state.proc is None:
        return
    if state.outcome == OUTCOME_NA:
        ui.notify('Set the capture outcome before stopping.', type='warning')
        return

    outcome = state.outcome
    proc, bag_dir = state.proc, state.bag_dir
    log.info('Stopping recording, marking run as %s ...', outcome)

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except OSError as exc:
        log.error('Could not signal rosbag (%s). Trying terminate().', exc)
        proc.terminate()

    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        log.error('rosbag did not close within 20 s. Killing it. '
                  'The bag may need "rosbag reindex".')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except OSError:
            proc.kill()
        proc.wait(timeout=5)

    duration = time.time() - (state.started_at or time.time())

    state.recording = False
    state.proc = None
    state.started_at = None
    refresh_controls()

    # Finalising touches the disk and can take a moment - keep it off the UI thread.
    threading.Thread(target=_finalise_bag,
                     args=(bag_dir, outcome, state.active_id, duration),
                     daemon=True).start()


def _finalise_bag(bag_dir: str, outcome: str, exp_id: Optional[int],
                  duration: float) -> None:
    """Reindex if needed, write the outcome marker and a metadata sidecar."""
    try:
        with open(os.path.join(bag_dir, outcome), 'w'):
            pass
        log.info('Wrote outcome marker: %s/%s', bag_dir, outcome)
    except OSError as exc:
        log.error('Could not write outcome marker in %s: %s', bag_dir, exc)

    for name in sorted(os.listdir(bag_dir)):
        if name.endswith('.bag.active'):
            log.warning('Found %s - reindexing.', name)
            try:
                subprocess.run(['rosbag', 'reindex', os.path.join(bag_dir, name)],
                               check=True, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT)
            except Exception as exc:                           # noqa: BLE001
                log.error('Reindex failed for %s: %s', name, exc)

    summary = {}
    for name in sorted(os.listdir(bag_dir)):
        if not name.endswith('.bag'):
            continue
        path = os.path.join(bag_dir, name)
        try:
            with rosbag.Bag(path) as bag:
                summary[name] = {
                    'duration_s': round(bag.get_end_time() - bag.get_start_time(), 2),
                    'messages': bag.get_message_count(),
                    'topics': sorted(bag.get_type_and_topic_info()[1].keys()),
                    'size_mb': round(bag.size / 1e6, 1),
                }
            log.info('%s: %s msgs, %.1f s, %.1f MB', name,
                     summary[name]['messages'], summary[name]['duration_s'],
                     summary[name]['size_mb'])
        except Exception as exc:                               # noqa: BLE001
            log.error('Could not read %s: %s', name, exc)

    meta = {
        'experiment_id': '{:03d}'.format(exp_id) if exp_id is not None else None,
        'suffix': sanitise_suffix(state.suffix),
        'outcome': outcome,
        'recorded_at': datetime.now().isoformat(timespec='seconds'),
        'wall_duration_s': round(duration, 2),
        'record_all': state.record_all,
        'requested_topics': [] if state.record_all else list(state.wanted_topics),
        'bags': summary,
    }
    try:
        with open(os.path.join(bag_dir, 'metadata.json'), 'w') as fh:
            json.dump(meta, fh, indent=2)
    except OSError as exc:
        log.error('Could not write metadata.json: %s', exc)

    log.info('Run complete: %s', bag_dir)


def reset_experiment() -> None:
    """TODO: reset the simulation / respawn the Astrobee to its start pose."""
    log.info('Reset Experiment pressed (no action wired up yet).')
    ui.notify('Reset is not wired up yet.', type='info')


# ----------------------------------------------------------------------------
# Helpers for the status strip
# ----------------------------------------------------------------------------

def dir_size_mb(path: Optional[str]) -> float:
    if not path or not os.path.isdir(path):
        return 0.0
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total / 1e6


def free_space_gb(path: str) -> float:
    probe = path
    while probe and not os.path.isdir(probe):
        probe = os.path.dirname(probe)
    try:
        return shutil.disk_usage(probe or '/').free / 1e9
    except OSError:
        return 0.0


# ----------------------------------------------------------------------------
# Single-controller lock
#
# One browser tab holds control. Others get a lock screen with an explicit
# takeover, so a crashed or forgotten tab can never strand the GUI.
# ----------------------------------------------------------------------------

_LOCK: Dict[str, Optional[str]] = {'client_id': None}


def _release(client_id: str) -> None:
    if _LOCK['client_id'] == client_id:
        _LOCK['client_id'] = None
        log.info('Control released.')


# ----------------------------------------------------------------------------
# UI
# ----------------------------------------------------------------------------

def refresh_controls() -> None:
    """Single source of truth for enabled/disabled state."""
    if 'start_btn' not in UI:
        return

    UI['start_btn'].set_enabled(not state.recording)
    UI['stop_btn'].set_enabled(state.recording and state.outcome != OUTCOME_NA)
    UI['id_input'].set_enabled(not state.recording)
    UI['suffix_input'].set_enabled(not state.recording)
    UI['outcome_toggle'].set_enabled(state.recording)

    if state.recording:
        tip = 'Stop recording and write the outcome marker.'
        if state.outcome == OUTCOME_NA:
            tip = 'Set the capture outcome first - Stop stays locked until you do.'
    else:
        tip = 'Not recording.'
    UI['stop_tip'].set_text(tip)

    preview = build_folder_name(state.experiment_id, state.suffix)
    UI['preview'].set_text(os.path.join(state.save_dir, preview) + '/')
    UI['id_input'].value = '{:03d}'.format(state.experiment_id)


def update_status() -> None:
    if 'rec_label' not in UI:
        return

    connected = master_alive()
    UI['ros_chip'].set_text('ROS master: up' if connected else 'ROS master: DOWN')
    UI['ros_chip'].props('color={}'.format('green-8' if connected else 'red-9'))

    UI['disk_chip'].set_text('{:.1f} GB free'.format(free_space_gb(state.save_dir)))

    if state.recording:
        elapsed = int(time.time() - (state.started_at or time.time()))
        UI['rec_label'].set_text(
            'REC  {:03d}  {:02d}:{:02d}  {:.0f} MB'.format(
                state.active_id or 0, elapsed // 60, elapsed % 60,
                dir_size_mb(state.bag_dir)))
        UI['rec_label'].classes(replace='text-red-400 font-mono text-sm animate-pulse')
        # rosbag died on its own (disk full, killed externally, bad topic name)
        if state.proc is not None and state.proc.poll() is not None:
            log.error('rosbag record exited unexpectedly (code %s).',
                      state.proc.returncode)
            state.recording = False
            state.proc = None
            refresh_controls()
    else:
        UI['rec_label'].set_text('idle')
        UI['rec_label'].classes(replace='text-gray-500 font-mono text-sm')


def drain_log() -> None:
    pane = UI.get('log')
    if pane is None:
        return
    while True:
        try:
            pane.push(LOG_QUEUE.get_nowait())
        except queue.Empty:
            return


# --- Tab 1 ------------------------------------------------------------------

def build_experiment_tab() -> None:
    with ui.column().classes('w-full gap-4 p-4'):

        with ui.card().classes('w-full'):
            ui.label('Run control').classes('text-sm text-gray-400 uppercase')
            with ui.row().classes('items-center gap-3'):
                UI['start_btn'] = ui.button(
                    'Start recording', icon='fiber_manual_record',
                    on_click=start_recording).props('color=green-8').classes('w-48')
                UI['stop_btn'] = ui.button(
                    'Stop recording', icon='stop',
                    on_click=stop_recording).props('color=red-9').classes('w-48')
                ui.space()
                ui.button('Reset experiment', icon='restart_alt',
                          on_click=lambda: confirm_reset()).props('outline color=orange')
            UI['stop_tip'] = ui.label('').classes('text-xs text-gray-400')

        with ui.card().classes('w-full'):
            ui.label('Run metadata').classes('text-sm text-gray-400 uppercase')
            with ui.row().classes('items-center gap-4 w-full'):
                UI['id_input'] = ui.input(
                    'Experiment ID',
                    value='{:03d}'.format(state.experiment_id),
                    on_change=on_id_change).props('outlined dense').classes('w-32')
                UI['suffix_input'] = ui.input(
                    'Bag suffix', value=state.suffix, placeholder='e.g. handrail grasp',
                    on_change=on_suffix_change).props('outlined dense').classes('flex-grow')
            ui.label('Next bag folder').classes('text-xs text-gray-400 mt-2')
            UI['preview'] = ui.label('').classes(
                'font-mono text-sm text-cyan-300 break-all')

        with ui.card().classes('w-full'):
            ui.label('Capture outcome').classes('text-sm text-gray-400 uppercase')
            UI['outcome_toggle'] = ui.toggle(
                {OUTCOME_NA: 'N/A',
                 OUTCOME_SUCCESS: 'Capture success',
                 OUTCOME_FAILURE: 'Capture failure'},
                value=OUTCOME_NA,
                on_change=on_outcome_change).props('no-caps')
            ui.label('Resets to N/A on start. Stop unlocks once this is set. '
                     'F6 = success, F7 = failure.').classes('text-xs text-gray-400')


def on_id_change(event) -> None:
    raw = re.sub(r'\D', '', str(event.value or ''))
    state.experiment_id = int(raw) if raw else 0
    preview = build_folder_name(state.experiment_id, state.suffix)
    UI['preview'].set_text(os.path.join(state.save_dir, preview) + '/')
    save_config()


def on_suffix_change(event) -> None:
    state.suffix = str(event.value or '')
    preview = build_folder_name(state.experiment_id, state.suffix)
    UI['preview'].set_text(os.path.join(state.save_dir, preview) + '/')
    save_config()


def on_outcome_change(event) -> None:
    state.outcome = event.value or OUTCOME_NA
    refresh_controls()


def confirm_reset() -> None:
    with ui.dialog() as dialog, ui.card():
        ui.label('Reset the experiment?').classes('text-lg')
        ui.label('This will return the simulation to its start state.'
                 ).classes('text-sm text-gray-400')
        if state.recording:
            ui.label('A recording is in progress. Stop it first.'
                     ).classes('text-sm text-red-400')
        with ui.row().classes('w-full justify-end'):
            ui.button('Cancel', on_click=dialog.close).props('flat')
            ui.button('Reset', on_click=lambda: (dialog.close(), reset_experiment())
                      ).props('color=orange').set_enabled(not state.recording)
    dialog.open()


# --- Tab 2 ------------------------------------------------------------------

def build_topics_tab() -> None:
    with ui.column().classes('w-full h-full gap-3 p-4'):
        with ui.row().classes('items-center gap-4 w-full'):
            ui.button('Refresh list', icon='refresh',
                      on_click=refresh_topics).props('outline')
            ui.checkbox('Record all topics (-a)', value=state.record_all,
                        on_change=on_record_all_change)
            ui.button('Select all', on_click=lambda: bulk_select(True)).props('flat dense')
            ui.button('Clear', on_click=lambda: bulk_select(False)).props('flat dense')
            ui.space()
            UI['topic_count'] = ui.label('').classes('text-sm text-gray-400')

        UI['topic_box'] = ui.scroll_area().classes(
            'w-full flex-grow border border-gray-700 rounded')
        refresh_topics()


def on_record_all_change(event) -> None:
    state.record_all = bool(event.value)
    save_config()
    render_topic_list()


def bulk_select(select: bool) -> None:
    state.wanted_topics = list(state.available_topics) if select else []
    save_config()
    render_topic_list()


def refresh_topics() -> None:
    state.available_topics = list_topics()
    log.info('Found %d published topics.', len(state.available_topics))
    render_topic_list()


def render_topic_list() -> None:
    box = UI.get('topic_box')
    if box is None:
        return
    box.clear()

    wanted = set(state.wanted_topics)
    missing = sorted(wanted - set(state.available_topics))

    with box:
        with ui.column().classes('p-3 gap-1 w-full'):
            if not state.available_topics:
                ui.label('No topics published. Is the simulation running? '
                         'Press Refresh list.').classes('text-sm text-gray-400')
            for topic in state.available_topics:
                ui.checkbox(topic, value=topic in wanted,
                            on_change=lambda e, t=topic: on_topic_toggle(t, e.value)
                            ).props('dense').classes('text-sm')
            if missing:
                ui.separator().classes('my-2')
                ui.label('Selected but not currently published').classes(
                    'text-xs text-orange-400 uppercase')
                for topic in missing:
                    ui.checkbox(topic, value=True,
                                on_change=lambda e, t=topic: on_topic_toggle(t, e.value)
                                ).props('dense').classes('text-sm text-orange-300')

    if 'topic_count' in UI:
        UI['topic_count'].set_text(
            'all topics' if state.record_all
            else '{} of {} selected'.format(len(state.wanted_topics),
                                            len(state.available_topics)))


def on_topic_toggle(topic: str, checked: bool) -> None:
    if checked and topic not in state.wanted_topics:
        state.wanted_topics.append(topic)
    elif not checked and topic in state.wanted_topics:
        state.wanted_topics.remove(topic)
    save_config()
    if 'topic_count' in UI:
        UI['topic_count'].set_text(
            'all topics' if state.record_all
            else '{} of {} selected'.format(len(state.wanted_topics),
                                            len(state.available_topics)))


# --- Tab 3 ------------------------------------------------------------------

def build_other_tab() -> None:
    with ui.column().classes('w-full gap-4 p-4'):
        with ui.card().classes('w-full'):
            ui.label('Storage').classes('text-sm text-gray-400 uppercase')
            ui.input('Bag save directory', value=state.save_dir,
                     on_change=on_save_dir_change
                     ).props('outlined dense').classes('w-full')
            UI['dir_status'] = ui.label('').classes('text-xs')

        with ui.card().classes('w-full'):
            ui.label('rosbag options').classes('text-sm text-gray-400 uppercase')
            with ui.row().classes('gap-4'):
                ui.number('Buffer (MB)', value=state.buffer_mb, min=0, step=256,
                          on_change=lambda e: set_int('buffer_mb', e.value)
                          ).props('outlined dense').classes('w-40')
                ui.number('Split size (MB, 0 = off)', value=state.split_mb, min=0,
                          step=256, on_change=lambda e: set_int('split_mb', e.value)
                          ).props('outlined dense').classes('w-56')
            ui.label('A larger buffer avoids dropped messages on image-heavy topics.'
                     ).classes('text-xs text-gray-400')

        ui.label('Settings are saved to {}'.format(CONFIG_PATH)
                 ).classes('text-xs text-gray-500')
    check_save_dir()


def set_int(attr: str, value) -> None:
    try:
        setattr(state, attr, int(value or 0))
        save_config()
    except (TypeError, ValueError):
        pass


def on_save_dir_change(event) -> None:
    state.save_dir = str(event.value or '').strip() or DEFAULT_SAVE_DIR
    save_config()
    check_save_dir()
    refresh_controls()


def check_save_dir() -> None:
    label = UI.get('dir_status')
    if label is None:
        return
    if os.path.isdir(state.save_dir):
        writable = os.access(state.save_dir, os.W_OK)
        label.set_text('Directory exists' + ('' if writable else ' but is not writable'))
        label.classes(replace='text-xs ' + ('text-green-400' if writable else 'text-red-400'))
    else:
        label.set_text('Directory does not exist. It will be created on first recording.')
        label.classes(replace='text-xs text-orange-400')


# --- Page -------------------------------------------------------------------

def build_lock_screen() -> None:
    with ui.column().classes('absolute-center items-center gap-4'):
        ui.icon('lock', size='xl').classes('text-orange-400')
        ui.label('This GUI is open in another tab').classes('text-xl')
        ui.label('Only one tab can control an experiment, so the two cannot '
                 'disagree about what is recording.').classes(
                     'text-sm text-gray-400 text-center max-w-md')
        ui.button('Take control here', icon='login',
                  on_click=take_control).props('color=orange')


def take_control() -> None:
    _LOCK['client_id'] = ui.context.client.id
    log.info('Control taken over by a new tab.')
    ui.navigate.reload()


@ui.page('/')
def index(client: Client) -> None:
    ui.dark_mode().enable()
    ui.query('.nicegui-content').classes('h-screen w-full max-w-full p-0 gap-0 flex flex-col')

    if _LOCK['client_id'] not in (None, client.id):
        build_lock_screen()
        return

    _LOCK['client_id'] = client.id
    client.on_disconnect(lambda: _release(client.id))
    UI.clear()

    # Header
    with ui.row().classes('w-full items-center gap-4 px-4 py-2 '
                          'bg-gray-900 border-b border-gray-700'):
        ui.label('CATAPULT\nLOGO').classes(
            'text-[10px] leading-tight text-center text-gray-500 font-mono '
            'border border-dashed border-gray-600 rounded px-3 py-2 whitespace-pre')
        ui.label('Astrobee data collection').classes('text-lg')
        ui.space()
        UI['rec_label'] = ui.label('idle').classes('text-gray-500 font-mono text-sm')
        UI['disk_chip'] = ui.chip('').props('dense color=grey-9')
        UI['ros_chip'] = ui.chip('').props('dense')

    # Tabs over debug pane, draggable divider between them
    with ui.splitter(horizontal=True, value=72).classes('w-full flex-grow') as splitter:
        with splitter.before:
            with ui.column().classes('w-full h-full gap-0'):
                with ui.tabs().classes('w-full') as tabs:
                    tab_exp = ui.tab('Experiment control')
                    tab_top = ui.tab('Topics')
                    tab_oth = ui.tab('Other')
                with ui.tab_panels(tabs, value=tab_exp).classes('w-full flex-grow'):
                    with ui.tab_panel(tab_exp):
                        build_experiment_tab()
                    with ui.tab_panel(tab_top).classes('h-full'):
                        build_topics_tab()
                    with ui.tab_panel(tab_oth):
                        build_other_tab()

        with splitter.after:
            with ui.column().classes('w-full h-full gap-0'):
                with ui.row().classes('items-center px-3 py-1 bg-gray-900 w-full'):
                    ui.label('Debug').classes('text-xs uppercase text-gray-400')
                    ui.space()
                    ui.button(icon='delete_sweep',
                              on_click=lambda: UI['log'].clear()).props('flat dense round')
                UI['log'] = ui.log(max_lines=LOG_LINES).classes(
                    'w-full flex-grow font-mono text-xs bg-black')

    ui.keyboard(on_key=on_key)
    ui.timer(0.2, drain_log)
    ui.timer(1.0, update_status)
    refresh_controls()
    check_save_dir()
    log.info('GUI ready. Saving to %s', state.save_dir)


def on_key(event) -> None:
    """Function keys only - they cannot collide with typing in a text field."""
    if not event.action.keydown:
        return
    key = str(event.key)
    if key == 'F8' and not state.recording:
        start_recording()
    elif key == 'F9' and state.recording:
        stop_recording()
    elif key == 'F6' and state.recording:
        UI['outcome_toggle'].set_value(OUTCOME_SUCCESS)
    elif key == 'F7' and state.recording:
        UI['outcome_toggle'].set_value(OUTCOME_FAILURE)


# ----------------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------------

def on_shutdown() -> None:
    if state.recording and state.proc is not None:
        log.warning('Shutting down with a recording active - closing the bag.')
        try:
            os.killpg(os.getpgid(state.proc.pid), signal.SIGINT)
            state.proc.wait(timeout=15)
        except Exception:                                      # noqa: BLE001
            pass


load_config()
init_ros()
scanned = scan_next_experiment_id(state.save_dir)
if scanned and scanned > state.experiment_id:
    log.info('Existing runs found on disk. Next experiment ID set to %03d.', scanned)
    state.experiment_id = scanned

app.on_shutdown(on_shutdown)

if __name__ in {'__main__', '__mp_main__'}:
    # reload=False: the auto-reloader re-executes this module and rospy.init_node
    # cannot be called twice in one process.
    ui.run(title='Astrobee data collection', port=GUI_PORT, host='0.0.0.0',
           reload=False, dark=True, favicon='🐝')
