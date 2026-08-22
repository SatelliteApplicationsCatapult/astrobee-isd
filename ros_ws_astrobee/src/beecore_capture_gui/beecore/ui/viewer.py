"""Standalone image viewer page.

A bare page at /view showing nothing but the four MJPEG streams from
web_video_server, in a 2x2 grid. Open it in its own browser window, drag it to
a second display, press F11, and it is a clean fullscreen monitor.

ONE SERVER, FOUR STREAMS
------------------------
All four <img> tags point at the SAME web_video_server on the same port; only
the `topic` query parameter differs. web_video_server spins up an encoder per
connection, so four streams need one server, not four. A second server would
be another port to manage, another process to reap, and no benefit.

Deliberately NOT subject to the single-tab lock: the lock is registered on '/'
only, and a viewer window locking out the control window would be maddening.

Nothing here runs a subprocess. The <img> src is resolved by the browser, so
there is no X11, no signals, and nothing that can leave a hung process behind
when you close the window - which was the whole point compared to shelling out
to rqt_image_view.

The streams also survive a GUI restart: they point at web_video_server, not at
this process.
"""

from nicegui import ui

from ..config import CAM_COUNT, settings
from .. import theme

# Per-tile reconnect. Written against a NodeList rather than one element, so a
# camera that is respawned (FOV or resolution change) recovers on its own
# without disturbing the other three.
_OVERLAY_JS = '''
<script>
(function () {
  var bar = document.getElementById('overlay');

  // MJPEG connections do not reconnect on their own. Without this, anything
  // that interrupts a stream - respawning that camera to change the FOV,
  // restarting web_video_server - leaves its tile permanently blank until
  // someone reloads the window by hand.
  //
  // The subtle case: if the topic does not exist, web_video_server ACCEPTS the
  // connection and simply never sends a frame. The <img> fires neither 'load'
  // nor 'error' - it just spins forever. So liveness cannot be judged on
  // events alone; each tile tracks whether a first frame ever arrived.
  var tiles = [];
  document.querySelectorAll('img.stream').forEach(function (img) {
    var tile = {
      img: img,
      msg: document.getElementById('msg-' + img.dataset.index),
      base: img.getAttribute('data-src'),
      loaded: false,
      retry: null
    };

    function reconnect() {
      if (tile.retry) { return; }
      tile.retry = setTimeout(function () {
        tile.retry = null;
        tile.loaded = false;
        // Drop the pending request first, or the browser keeps the old
        // never-completing connection (and its spinner) alive alongside
        // the new one.
        tile.img.removeAttribute('src');
        tile.img.src = tile.base + '&t=' + Date.now();
      }, 3000);
    }
    tile.reconnect = reconnect;

    img.addEventListener('load', function () {
      tile.loaded = true;
      tile.msg.style.display = 'none';
    });
    img.addEventListener('error', function () {
      tile.loaded = false;
      tile.msg.style.display = 'flex';
      reconnect();
    });
    tiles.push(tile);
  });

  // Only watches the never-loaded case. Deliberately NOT a stall detector:
  // an <img> gives no per-frame signal for MJPEG, so anything that treated
  // "no event recently" as a stall would keep tearing down healthy streams.
  setInterval(function () {
    tiles.forEach(function (tile) {
      if (!tile.loaded) {
        tile.msg.style.display = 'flex';
        tile.reconnect();
      }
    });
  }, 1000);

  // Overlay fades out once you stop moving the mouse, so a fullscreen
  // viewer is genuinely just the images.
  var timer = null;
  function poke() {
    bar.style.opacity = '1';
    if (timer) { clearTimeout(timer); }
    timer = setTimeout(function () { bar.style.opacity = '0'; }, 2500);
  }
  document.addEventListener('mousemove', poke);
  poke();
})();
</script>
'''

_TILE = '''
  <div style="position:relative; background:#000; overflow:hidden;
              display:flex; align-items:center; justify-content:center;
              border:1px solid {border};">
    <img class="stream" data-index="{index}" src="{url}" data-src="{base_url}"
         style="max-width:100%; max-height:100%; object-fit:contain;"/>
    <div id="msg-{index}"
         style="display:none; position:absolute; inset:0;
                align-items:center; justify-content:center;
                flex-direction:column; gap:0.35rem;
                color:{muted}; font-family:monospace; font-size:0.8rem;
                text-align:center; padding:0.5rem;">
      <div style="font-size:1rem; color:{amber};">No stream</div>
      <div style="word-break:break-all;">{topic}</div>
      <div style="font-size:0.75rem;">retrying every 3 s</div>
    </div>
    <div style="position:absolute; top:0; left:0; padding:0.15rem 0.5rem;
                background:rgba(0,0,0,0.55); color:{text};
                font-family:monospace; font-size:0.7rem;">{label}</div>
  </div>
'''


def register() -> None:
    ui.page('/view')(_render)


def _render() -> None:
    ui.dark_mode().enable()
    ui.query('.nicegui-content').classes('w-full h-screen p-0 gap-0')
    ui.add_head_html(
        '<style>body { background: #000; overflow: hidden; margin: 0; }</style>')

    # Cache-buster: without it a reload can re-attach to the stale stream.
    import time
    stamp = int(time.time())

    tiles = []
    for index in range(CAM_COUNT):
        base_url = settings.stream_url(index)
        tiles.append(_TILE.format(
            index=index,
            url='{}&t={}'.format(base_url, stamp),
            base_url=base_url,
            topic=settings.image_topic(index),
            label=settings.camera(index).get(
                'label', 'Camera {}'.format(index + 1)),
            border=theme.BORDER, muted=theme.MUTED,
            amber=theme.AMBER, text=theme.TEXT))

    # 2x2 grid sized in viewport units rather than percentages: a percentage
    # of an indefinite height collapses, which is the same trap as the
    # splitter on the main page.
    ui.add_body_html('''
        <div style="position:fixed; inset:0; background:#000;
                    display:grid; grid-template-columns:1fr 1fr;
                    grid-template-rows:1fr 1fr; gap:2px;">
          {tiles}
        </div>
        <div id="overlay"
             style="position:fixed; left:0; right:0; bottom:0;
                    padding:0.5rem 1rem; background:rgba(0,0,0,0.55);
                    color:{text}; font-family:monospace; font-size:0.75rem;
                    display:flex; gap:1rem; align-items:center;
                    transition:opacity 0.4s ease;">
          <span>{count} cameras from {base}</span>
          <span style="flex:1"></span>
          <span style="color:{muted};">F11 for fullscreen</span>
          <a href="/view" style="color:{violet}; text-decoration:none;">reload</a>
        </div>
    '''.format(tiles=''.join(tiles), count=CAM_COUNT,
               base=settings.video_base_url,
               text=theme.TEXT, muted=theme.MUTED, violet=theme.VIOLET))

    ui.add_body_html(_OVERLAY_JS)
