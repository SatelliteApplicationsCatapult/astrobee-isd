"""Route Python and rospy logging into the GUI debug pane.

ROS callbacks, the recorder watchdog and the diagnostics poller all run on
non-UI threads, and NiceGUI elements must not be touched from those.
Everything funnels through a queue that a ui.timer drains on the UI thread.
"""

import logging
import queue

LOG_QUEUE = queue.Queue(maxsize=5000)          # type: queue.Queue

log = logging.getLogger('beecore_capture_gui')


class _QueueHandler(logging.Handler):
    def emit(self, record: logging.LogRecord) -> None:
        try:
            LOG_QUEUE.put_nowait(self.format(record))
        except queue.Full:
            pass                                # drop rather than block a callback


def setup(level: int = logging.INFO) -> None:
    if getattr(setup, '_done', False):
        return

    handler = _QueueHandler()
    handler.setFormatter(logging.Formatter(
        '%(asctime)s  %(levelname)-7s %(message)s', datefmt='%H:%M:%S'))

    log.setLevel(level)
    log.addHandler(handler)
    log.addHandler(logging.StreamHandler())
    logging.getLogger('rosout').addHandler(handler)

    setup._done = True                          # type: ignore[attr-defined]


def drain(pane) -> None:
    """Push everything queued into a ui.log element. Call from a ui.timer."""
    if pane is None:
        return
    while True:
        try:
            pane.push(LOG_QUEUE.get_nowait())
        except queue.Empty:
            return
