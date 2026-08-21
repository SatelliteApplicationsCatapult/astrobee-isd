"""rosbag record lifecycle.

Owns the subprocess, the folder, and the post-run finalisation. Knows nothing
about the UI - it reports through the on_change callback and the log.
"""

import json
import os
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime
from typing import Callable, List, Optional, Tuple

import rosbag

from .config import BAG_NODE_NAME, STOP_TIMEOUT_S, Settings
from .logbridge import log
from .naming import build_folder_name, sanitise_suffix


class RecorderError(Exception):
    """Raised for conditions the operator needs to see and act on."""


class BagRecorder:

    def __init__(self, settings: Settings,
                 on_change: Optional[Callable[[], None]] = None) -> None:
        self.settings = settings
        self.on_change = on_change or (lambda: None)

        self.proc = None            # type: Optional[subprocess.Popen]
        self.bag_dir = None         # type: Optional[str]
        self.bag_path = None        # type: Optional[str]
        self.started_at = None      # type: Optional[float]
        self.active_id = None       # type: Optional[int]
        self.active_suffix = ''

    # --- state ---------------------------------------------------------------

    @property
    def recording(self) -> bool:
        return self.proc is not None

    @property
    def elapsed_s(self) -> float:
        return time.time() - self.started_at if self.started_at else 0.0

    def bytes_on_disk(self) -> int:
        if not self.bag_dir or not os.path.isdir(self.bag_dir):
            return 0
        total = 0
        for root, _dirs, files in os.walk(self.bag_dir):
            for name in files:
                try:
                    total += os.path.getsize(os.path.join(root, name))
                except OSError:
                    pass
        return total

    # --- command -------------------------------------------------------------

    def _command(self, bag_path: str) -> List[str]:
        cfg = self.settings
        cmd = ['rosbag', 'record', '-O', bag_path,
               '--buffsize={}'.format(cfg.buffer_mb),
               '__name:={}'.format(BAG_NODE_NAME)]
        if cfg.split_mb > 0:
            cmd += ['--split', '--size={}'.format(cfg.split_mb)]
        if cfg.record_all or not cfg.wanted_topics:
            cmd.append('-a')
        else:
            cmd += list(cfg.wanted_topics)
        return cmd

    # --- start / stop --------------------------------------------------------

    def start(self, experiment_id: int, suffix: str) -> str:
        """Begin recording. Returns the folder name. Raises RecorderError."""
        if self.recording:
            raise RecorderError('Already recording.')

        cfg = self.settings
        if not cfg.record_all and not cfg.wanted_topics:
            raise RecorderError('No topics selected. Pick topics on the Topics '
                                'tab, or tick "Record all topics".')

        folder = build_folder_name(experiment_id, suffix)
        bag_dir = os.path.join(cfg.save_dir, folder)

        try:
            os.makedirs(bag_dir, exist_ok=False)
        except FileExistsError:
            raise RecorderError('Folder already exists: {}'.format(bag_dir))
        except OSError as exc:
            raise RecorderError('Cannot create {}: {}'.format(bag_dir, exc))

        bag_path = os.path.join(bag_dir, folder + '.bag')
        cmd = self._command(bag_path)

        try:
            # setsid: rosbag record spawns children. We must signal the whole
            # process group, or SIGINT leaves a half-written .bag.active.
            proc = subprocess.Popen(cmd, preexec_fn=os.setsid, cwd=bag_dir)
        except OSError as exc:
            shutil.rmtree(bag_dir, ignore_errors=True)
            raise RecorderError('Could not launch rosbag record: {}'.format(exc))

        self.proc = proc
        self.bag_dir = bag_dir
        self.bag_path = bag_path
        self.started_at = time.time()
        self.active_id = experiment_id
        self.active_suffix = suffix

        scope = ('all topics' if (cfg.record_all or not cfg.wanted_topics)
                 else '{} topics'.format(len(cfg.wanted_topics)))
        log.info('Recording %s (%s)', folder, scope)
        log.info('Command: %s', ' '.join(cmd))
        self.on_change()
        return folder

    def stop(self, outcome: str, extra_meta: Optional[dict] = None) -> None:
        """Close the bag, then finalise on a worker thread."""
        if not self.recording or self.proc is None:
            return

        proc, bag_dir = self.proc, self.bag_dir
        duration, exp_id = self.elapsed_s, self.active_id
        suffix = self.active_suffix
        log.info('Stopping recording, marking run as %s ...', outcome)

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except OSError as exc:
            log.error('Could not signal rosbag (%s). Falling back to terminate.', exc)
            proc.terminate()

        try:
            proc.wait(timeout=STOP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            log.error('rosbag did not close within %d s. Killing it - the bag '
                      'will need reindexing.', STOP_TIMEOUT_S)
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except OSError:
                proc.kill()
            proc.wait(timeout=5)

        self.proc = None
        self.started_at = None
        self.on_change()

        threading.Thread(
            target=self._finalise,
            args=(bag_dir, outcome, exp_id, suffix, duration, extra_meta or {}),
            daemon=True).start()

    def poll(self) -> bool:
        """Watchdog. True if rosbag died on its own (disk full, bad topic...)."""
        if self.proc is not None and self.proc.poll() is not None:
            log.error('rosbag record exited unexpectedly (code %s).',
                      self.proc.returncode)
            self.proc = None
            self.started_at = None
            self.on_change()
            return True
        return False

    def shutdown(self) -> None:
        """Best-effort bag close when the GUI process exits."""
        if self.proc is None:
            return
        log.warning('Shutting down with a recording active - closing the bag.')
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=STOP_TIMEOUT_S)
        except Exception:                                      # noqa: BLE001
            pass

    # --- finalisation --------------------------------------------------------

    def _finalise(self, bag_dir: Optional[str], outcome: str,
                  exp_id: Optional[int], suffix: str, duration: float,
                  extra_meta: dict) -> None:
        if not bag_dir:
            return

        try:
            with open(os.path.join(bag_dir, outcome), 'w'):
                pass
            log.info('Wrote outcome marker: %s/%s', bag_dir, outcome)
        except OSError as exc:
            log.error('Could not write outcome marker in %s: %s', bag_dir, exc)

        self._reindex_actives(bag_dir)
        summary = self._summarise(bag_dir)

        meta = {
            'experiment_id': None if exp_id is None else '{:03d}'.format(exp_id),
            'suffix': sanitise_suffix(suffix),
            'outcome': outcome,
            'recorded_at': datetime.now().isoformat(timespec='seconds'),
            'wall_duration_s': round(duration, 2),
            'record_all': self.settings.record_all,
            'requested_topics': ([] if self.settings.record_all
                                 else list(self.settings.wanted_topics)),
            'bags': summary,
        }
        meta.update(extra_meta)
        try:
            with open(os.path.join(bag_dir, 'metadata.json'), 'w') as fh:
                json.dump(meta, fh, indent=2)
        except OSError as exc:
            log.error('Could not write metadata.json: %s', exc)

        log.info('Run complete: %s', bag_dir)

    @staticmethod
    def _reindex_actives(bag_dir: str) -> None:
        try:
            names = sorted(os.listdir(bag_dir))
        except OSError:
            return
        for name in names:
            if not name.endswith('.bag.active'):
                continue
            log.warning('Found %s - reindexing.', name)
            try:
                subprocess.check_output(
                    ['rosbag', 'reindex', os.path.join(bag_dir, name)],
                    stderr=subprocess.STDOUT)
            except Exception as exc:                           # noqa: BLE001
                log.error('Reindex failed for %s: %s', name, exc)

    @staticmethod
    def _summarise(bag_dir: str) -> dict:
        summary = {}
        try:
            names = sorted(os.listdir(bag_dir))
        except OSError:
            return summary
        for name in names:
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
                info = summary[name]
                log.info('%s: %s msgs, %.1f s, %.1f MB', name,
                         info['messages'], info['duration_s'], info['size_mb'])
            except Exception as exc:                           # noqa: BLE001
                log.error('Could not read %s: %s', name, exc)
        return summary


def free_space_gb(path: str) -> float:
    probe = path
    while probe and not os.path.isdir(probe):
        probe = os.path.dirname(probe)
    try:
        return shutil.disk_usage(probe or '/').free / 1e9
    except OSError:
        return 0.0


def dir_is_writable(path: str) -> Tuple[bool, bool]:
    """Returns (exists, writable)."""
    if not os.path.isdir(path):
        return False, False
    return True, os.access(path, os.W_OK)
