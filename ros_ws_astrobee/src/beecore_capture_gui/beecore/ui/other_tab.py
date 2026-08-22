"""Other: storage locations and rosbag options."""

from nicegui import ui

from ..config import (CONFIG_PATH, DEFAULT_MODELS_DIR, DEFAULT_SAVE_DIR,
                      VIDEO_HOST, settings)
from ..logbridge import log
from ..video_server import VideoServerError
from ..recorder import dir_is_writable
from .. import theme


class OtherTab:

    def __init__(self, on_save_dir_change=None, video=None) -> None:
        self.on_save_dir_change = on_save_dir_change or (lambda: None)
        self.video = video

    def build(self) -> None:
        with ui.column().classes('w-full gap-4 p-4'):
            with ui.card().classes('w-full'):
                ui.label('Storage').classes('eyebrow')
                ui.input('Bag save directory', value=settings.save_dir,
                         on_change=self._on_dir_change,
                         ).props('outlined dense').classes('w-full')
                self.dir_status = ui.label('').classes('text-xs')

                ui.input('Tool models directory', value=settings.models_dir,
                         on_change=self._on_models_change,
                         ).props('outlined dense').classes('w-full mt-3')
                self.models_status = ui.label('').classes('text-xs')

            with ui.card().classes('w-full'):
                ui.label('rosbag options').classes('eyebrow')
                with ui.row().classes('gap-4'):
                    ui.number('Buffer (MB)', value=settings.buffer_mb, min=0, step=256,
                              on_change=lambda e: self._set_int('buffer_mb', e.value),
                              ).props('outlined dense').classes('w-40')
                    ui.number('Split size (MB, 0 = off)', value=settings.split_mb,
                              min=0, step=256,
                              on_change=lambda e: self._set_int('split_mb', e.value),
                              ).props('outlined dense').classes('w-56')
                ui.label('A larger buffer avoids dropped messages on image-heavy '
                         'topics. rosbag defaults to 256 MB and drops silently.'
                         ).classes('text-xs').style('color: {}'.format(theme.MUTED))

            with ui.card().classes('w-full'):
                ui.label('Image viewer').classes('eyebrow')
                with ui.row().classes('gap-4 w-full'):
                    ui.input('Video host (as the browser sees it)',
                             value=settings.video_host,
                             on_change=self._on_video_host,
                             ).props('outlined dense').classes('flex-grow')
                    ui.number('Port', value=settings.video_port,
                              min=1024, max=65535, step=1,
                              on_change=lambda e: self._set_int('video_port', e.value),
                              ).props('outlined dense').classes('w-32')
                with ui.row().classes('gap-4'):
                    ui.number('JPEG quality', value=settings.stream_quality,
                              min=10, max=100, step=5,
                              on_change=lambda e: self._set_int('stream_quality', e.value),
                              ).props('outlined dense').classes('w-40')

                with ui.row().classes('items-center gap-3 mt-2 flex-wrap'):
                    ui.button('Start server', icon='play_arrow',
                              on_click=self._start_video
                              ).props('color=secondary unelevated')
                    ui.button('Stop server', icon='stop',
                              on_click=self._stop_video).props('outline color=warning')
                    ui.checkbox('Start automatically with the GUI',
                                value=settings.video_autostart,
                                on_change=self._on_autostart).props('dense')

                self.video_status = ui.label('').classes('text-xs mt-1')
                self.stream_hint = ui.label('').classes('text-xs break-all').style(
                    'color: {}'.format(theme.MUTED))
                ui.label('The port is what the server is launched on AND what '
                         'the browser fetches, so the two cannot drift apart. '
                         'Quality trades CPU for image fidelity. There is no '
                         'stream width control: web_video_server ignores the '
                         'parameter (upstream bug). Use the sensor resolution '
                         'on the Camera tab instead - that changes what Gazebo '
                         'renders, which is what actually costs anything.'
                         ).classes('text-xs').style('color: {}'.format(theme.MUTED))

            ui.label('Settings are saved to {}'.format(CONFIG_PATH)
                     ).classes('text-xs').style('color: {}'.format(theme.MUTED))

        self.check_dirs()
        self._update_stream_hint()

    def _set_int(self, attr: str, value) -> None:
        try:
            setattr(settings, attr, int(value or 0))
            settings.save()
            self._update_stream_hint()
        except (TypeError, ValueError):
            pass

    def _on_video_host(self, event) -> None:
        settings.video_host = str(event.value or '').strip() or VIDEO_HOST
        settings.save()
        self._update_stream_hint()

    def _on_autostart(self, event) -> None:
        settings.video_autostart = bool(event.value)
        settings.save()

    def _start_video(self) -> None:
        if self.video is None:
            return
        try:
            self.video.start()
            # start() is a no-op if the port is already serving, so report the
            # resulting state rather than claiming a start that did not happen.
            if self.video.running:
                ui.notify('Video server started.', type='positive')
            else:
                ui.notify('Port {} was already serving - left it alone.'.format(
                    settings.video_port), type='warning')
        except VideoServerError as exc:
            log.error('%s', exc)
            ui.notify(str(exc), type='negative')
        self._update_stream_hint()

    def _stop_video(self) -> None:
        if self.video is not None:
            self.video.stop()
        self._update_stream_hint()

    def _update_stream_hint(self) -> None:
        if hasattr(self, 'stream_hint'):
            # One example URL - the other three differ only in `topic`.
            self.stream_hint.set_text(settings.stream_url(0))
        if hasattr(self, 'video_status') and self.video is not None:
            status = self.video.status()
            self.video_status.set_text('web_video_server: {}'.format(status))
            # Green means "the GUI owns this and will clean it up". A server
            # someone else started still works, but stays amber - matching the
            # colour language on the Experiment tab.
            colour = theme.GREEN if self.video.running else theme.AMBER
            self.video_status.style('color: {}'.format(colour))

    def _on_dir_change(self, event) -> None:
        settings.save_dir = str(event.value or '').strip() or DEFAULT_SAVE_DIR
        settings.save()
        self.check_dirs()
        self.on_save_dir_change()

    def _on_models_change(self, event) -> None:
        settings.models_dir = str(event.value or '').strip() or DEFAULT_MODELS_DIR
        settings.save()
        self.check_dirs()

    def check_dirs(self) -> None:
        exists, writable = dir_is_writable(settings.save_dir)
        if exists and writable:
            self.dir_status.set_text('Directory exists and is writable.')
            colour = theme.GREEN
        elif exists:
            self.dir_status.set_text('Directory exists but is not writable. '
                                     'Check ownership on the bind mount.')
            colour = theme.RED
        else:
            self.dir_status.set_text('Directory does not exist. It will be '
                                     'created on the first recording.')
            colour = theme.AMBER
        self.dir_status.style('color: {}'.format(colour))

        import os
        if os.path.isdir(settings.models_dir):
            self.models_status.set_text('Directory found.')
            self.models_status.style('color: {}'.format(theme.GREEN))
        else:
            self.models_status.set_text('Not found - tool spawning will fail.')
            self.models_status.style('color: {}'.format(theme.RED))
