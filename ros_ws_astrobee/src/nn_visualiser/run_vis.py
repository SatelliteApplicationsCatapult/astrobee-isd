#!/usr/bin/env python3
"""BEECORE network visualiser.

    python3 run_vis.py            # then open http://localhost:8095
"""
from nicegui import ui

from nnvis.ui import page  # noqa: F401  (registers the route)

if __name__ in {'__main__', '__mp_main__'}:
    ui.run(host='0.0.0.0', port=8095, title='BEECORE NN visualiser',
           dark=True, reload=False, show=False, favicon='🧠')
