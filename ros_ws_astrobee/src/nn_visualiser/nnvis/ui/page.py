"""NiceGUI shell.

Deliberately thin.  Python's only jobs are: pick a bag, build the scene, and
serve three static blobs.  Everything that happens per frame happens in the
browser -- see canvas.js.  No NiceGUI element is created per neuron, and
nothing is pushed over the websocket during playback.
"""
import os
import traceback

from fastapi import Response
from nicegui import app, ui

from .. import config as C
from .. import scene

_STATE = {'meta': None, 'frames': b'', 'weights': b'', 'log': []}

_STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
app.add_static_files('/nnvis_static', _STATIC)

# Cache-buster for canvas.js.  The fetches inside canvas.js already carry
# ?v=<timestamp>, but the <script> tag loading canvas.js did not -- so a
# browser that had visited once kept running the stale renderer from cache,
# and re-copying the folder changed nothing on screen.
try:
    _JS_V = str(int(os.path.getmtime(os.path.join(_STATIC, 'canvas.js'))))
except OSError:
    _JS_V = '0'


@app.get('/nnvis/meta')
def _meta():
    return _STATE['meta'] or {}


@app.get('/nnvis/frames')
def _frames():
    return Response(content=_STATE['frames'], media_type='application/octet-stream')


@app.get('/nnvis/weights')
def _weights():
    return Response(content=_STATE['weights'], media_type='application/octet-stream')


TRANSPORT = '''
<div id="nnvis-wrap" style="display:flex;flex-direction:column;gap:10px;">
  <canvas id="nnvis-canvas" style="border-radius:10px;background:#07080B;
          box-shadow:0 0 0 1px #171B24, 0 18px 48px rgba(0,0,0,.55);"></canvas>
  <div style="display:flex;align-items:center;gap:14px;font:500 12px ui-monospace,Menlo,monospace;color:#8A94A6;">
    <button id="nnvis-play" onclick="window.nnvis.toggle()"
      style="width:38px;height:30px;border-radius:7px;border:1px solid #232936;
             background:#11151D;color:#D9E2F0;cursor:pointer;font-size:12px;">&#9654;</button>
    <button onclick="window.nnvis.step(-1)"
      style="width:32px;height:30px;border-radius:7px;border:1px solid #232936;
             background:#11151D;color:#8A94A6;cursor:pointer;">&#8249;</button>
    <button onclick="window.nnvis.step(1)"
      style="width:32px;height:30px;border-radius:7px;border:1px solid #232936;
             background:#11151D;color:#8A94A6;cursor:pointer;">&#8250;</button>
    <input id="nnvis-scrub" type="range" min="0" max="1000" value="0" step="1"
      oninput="window.nnvis.seek(this.value/1000)"
      style="flex:1;accent-color:#31D9FF;height:4px;">
    <select onchange="window.nnvis.setRate(parseFloat(this.value))"
      style="height:30px;border-radius:7px;border:1px solid #232936;background:#11151D;
             color:#D9E2F0;padding:0 8px;cursor:pointer;">
      <option value="0.1">0.1x</option>
      <option value="0.25">0.25x</option>
      <option value="0.5">0.5x</option>
      <option value="1" selected>1x</option>
      <option value="2">2x</option>
    </select>
  </div>
</div>
'''


@ui.page('/')
def main():
    ui.add_head_html(
        '<script src="/nnvis_static/canvas.js?v=%s"></script>' % _JS_V +
        '<style>body{background:#0A0C11;}</style>')
    ui.dark_mode(True)

    with ui.column().classes('w-full items-center gap-4 p-6'):
        ui.label('BEECORE — Neural Network Visualiser').style(
            'font:600 15px ui-monospace,Menlo,monospace;color:#D9E2F0;'
            'letter-spacing:.14em;')

        with ui.row().classes('items-end gap-3'):
            bag = ui.input('bag file', value='').props('dense outlined').style('width:420px')
            robot = ui.input('robot', value=C.DEFAULT_ROBOT).props('dense outlined').style('width:130px')
            tool = ui.input('tool', value=C.DEFAULT_TOOL).props('dense outlined').style('width:180px')

        with ui.row().classes('items-end gap-3'):
            # Left blank, the untrained stand-in is used.  obs_stats.json is
            # picked up from the same directory automatically.
            policy = ui.input('policy.pt (blank = stand-in)', value='').props(
                'dense outlined').style('width:620px')
            load_btn = ui.button('Load')

        status = ui.label('').style('font:400 11px ui-monospace,Menlo,monospace;color:#8A94A6;')
        ui.html(TRANSPORT)

    def do_load():
        path = bag.value.strip()
        if not path:
            status.text = 'give a path to a .bag file'
            return
        if not os.path.isfile(path):
            status.text = 'not a file: %s' % path
            return
        load_btn.disable()
        status.text = 'reading %s ...' % os.path.basename(path)
        lines = []
        pol = policy.value.strip() or None
        if pol and not os.path.isfile(pol):
            status.text = 'not a file: %s' % pol
            load_btn.enable()
            return
        try:
            meta, frames, weights = scene.build(
                path, robot.value.strip(), tool.value.strip(),
                policy_path=pol, log=lines.append)
        except Exception as exc:
            traceback.print_exc()
            status.text = '%s: %s' % (type(exc).__name__, exc)
            load_btn.enable()
            return
        _STATE.update(meta=meta, frames=frames, weights=weights)
        status.text = '%d frames @ %g fps  |  %.1f s  |  %s' % (
            meta['n_frames'], meta['fps'], meta['duration'], '  |  '.join(lines))
        load_btn.enable()
        ui.run_javascript('window.nnvis.load()')

    load_btn.on('click', do_load)
