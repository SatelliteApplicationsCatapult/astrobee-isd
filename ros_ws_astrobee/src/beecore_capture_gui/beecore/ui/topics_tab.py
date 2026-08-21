"""Topic selection: which topics go into the bag.

The selection persists across restarts. Topics you have selected that are not
currently published are kept and flagged rather than silently dropped - a node
being down at refresh time should not quietly change what gets recorded.
"""

from nicegui import ui

from ..config import settings
from ..logbridge import log
from ..ros_link import list_topics
from ..state import state
from .. import theme


class TopicsTab:

    def build(self) -> None:
        with ui.column().classes('w-full h-full gap-3 p-4'):
            with ui.row().classes('items-center gap-3 w-full'):
                ui.button('Refresh list', icon='refresh',
                          on_click=self.refresh_topics).props('outline color=secondary')
                ui.checkbox('Record all topics (-a)', value=settings.record_all,
                            on_change=self._on_record_all)
                ui.button('Select all',
                          on_click=lambda: self._bulk(True)).props('flat dense')
                ui.button('Clear',
                          on_click=lambda: self._bulk(False)).props('flat dense')
                ui.space()
                self.count_label = ui.label('').classes('text-sm').style(
                    'color: {}'.format(theme.MUTED))

            self.box = ui.scroll_area().classes('w-full flex-grow rounded').style(
                'border: 1px solid {}'.format(theme.BORDER))

        self.refresh_topics()

    def refresh_topics(self) -> None:
        state.available_topics = list_topics()
        log.info('Found %d published topics.', len(state.available_topics))
        self.render()

    def _on_record_all(self, event) -> None:
        settings.record_all = bool(event.value)
        settings.save()
        self.render()

    def _bulk(self, select: bool) -> None:
        settings.wanted_topics = list(state.available_topics) if select else []
        settings.save()
        self.render()

    def _toggle(self, topic: str, checked: bool) -> None:
        if checked and topic not in settings.wanted_topics:
            settings.wanted_topics.append(topic)
        elif not checked and topic in settings.wanted_topics:
            settings.wanted_topics.remove(topic)
        settings.save()
        self._update_count()

    def _update_count(self) -> None:
        if settings.record_all:
            text = 'all topics'
        else:
            text = '{} of {} selected'.format(len(settings.wanted_topics),
                                              len(state.available_topics))
        self.count_label.set_text(text)

    def render(self) -> None:
        self.box.clear()
        wanted = set(settings.wanted_topics)
        missing = sorted(wanted - set(state.available_topics))

        with self.box:
            with ui.column().classes('p-3 gap-1 w-full'):
                if not state.available_topics:
                    ui.label('No topics published. Is the simulation running? '
                             'Press Refresh list.').classes('text-sm').style(
                                 'color: {}'.format(theme.MUTED))
                for topic in state.available_topics:
                    ui.checkbox(
                        topic, value=topic in wanted,
                        on_change=lambda e, t=topic: self._toggle(t, e.value),
                    ).props('dense color=secondary').classes('text-sm')

                if missing:
                    ui.separator().classes('my-2')
                    ui.label('Selected but not currently published').classes(
                        'eyebrow').style('color: {}'.format(theme.AMBER))
                    for topic in missing:
                        ui.checkbox(
                            topic, value=True,
                            on_change=lambda e, t=topic: self._toggle(t, e.value),
                        ).props('dense color=warning').classes('text-sm')

        self._update_count()
