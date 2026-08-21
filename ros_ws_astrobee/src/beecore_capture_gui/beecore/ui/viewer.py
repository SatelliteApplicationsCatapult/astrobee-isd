"""Standalone image viewer page.

A bare page at /view showing nothing but the MJPEG stream from
web_video_server. Open it in its own browser window, drag it to a second
display, press F11, and it is a clean fullscreen monitor.

Deliberately NOT subject to the single-tab lock: the lock is registered on '/'
only, and a viewer window locking out the control window would be maddening.

Nothing here runs a subprocess. The <img> src is resolved by the browser, so
there is no X11, no signals, and nothing that can leave a hung process behind
when you close the window - which was the whole point compared to shelling out
to rqt_image_view.

The stream also survives a GUI restart: it points at web_video_server, not at
this process.
"""

from nicegui import ui

from ..config import settings
from .. import theme

_OVERLAY_JS = '''
<script>
(function () {
  var img = document.getElementById('stream');
  var msg = document.getElementById('streammsg');
  var bar = document.getElementById('overlay');
  if (!img) { return; }

  // MJPEG connections do not reconnect on their own. Without this, anything
  // that interrupts the stream - respawning the camera to change the FOV,
  // restarting web_video_server - leaves the window permanently blank until
  // someone reloads it by hand.
  var base = img.getAttribute('data-src');
  var retry = null;
  function reconnect() {
    if (retry) { return; }
    retry = setTimeout(function () {
      retry = null;
      img.src = base + '&t=' + Date.now();
    }, 2000);
  }

  img.addEventListener('error', function () {
    msg.style.display = 'flex';
    reconnect();
  });
  img.addEventListener('load', function () {
    msg.style.display = 'none';
  });

  // A dropped MJPEG stream often stalls rather than firing 'error', so watch
  // for the frame going stale as well.
  var last = -1, stalls = 0;
  setInterval(function () {
    var now = img.naturalWidth * img.naturalHeight;
    if (now === last && img.complete) {
      if (++stalls > 6) { stalls = 0; reconnect(); }
    } else {
      stalls = 0;
    }
    last = now;
  }, 1000);

  // Overlay fades out once you stop moving the mouse, so a fullscreen
  // viewer is genuinely just the image.
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


def register() -> None:
    ui.page('/view')(_render)


def _render() -> None:
    ui.dark_mode().enable()
    ui.query('.nicegui-content').classes('w-full h-screen p-0 gap-0')
    ui.add_head_html(
        '<style>body { background: #000; overflow: hidden; margin: 0; }</style>')

    # Cache-buster: without it a reload can re-attach to the stale stream.
    import time
    url = settings.stream_url() + '&t={}'.format(int(time.time()))

    ui.add_body_html('''
        <div style="position:fixed; inset:0; background:#000;
                    display:flex; align-items:center; justify-content:center;">
          <img id="stream" src="{url}" data-src="{base_url}"
               style="max-width:100%; max-height:100%; object-fit:contain;"/>
          <div id="streammsg"
               style="display:none; position:absolute; inset:0;
                      align-items:center; justify-content:center;
                      flex-direction:column; gap:0.5rem;
                      color:{muted}; font-family:monospace; font-size:0.9rem;">
            <div style="font-size:1.1rem; color:{amber};">No stream</div>
            <div>{topic}</div>
            <div>Check that web_video_server is running on {base}</div>
            <div style="font-size:0.8rem;">retrying every 2 s</div>
          </div>
          <div id="overlay"
               style="position:fixed; left:0; right:0; bottom:0;
                      padding:0.5rem 1rem; background:rgba(0,0,0,0.55);
                      color:{text}; font-family:monospace; font-size:0.75rem;
                      display:flex; gap:1rem; align-items:center;
                      transition:opacity 0.4s ease;">
            <span>{topic}</span>
            <span style="flex:1"></span>
            <span style="color:{muted};">F11 for fullscreen</span>
            <a href="/view" style="color:{violet}; text-decoration:none;">reload</a>
          </div>
        </div>
    '''.format(url=url, base_url=settings.stream_url(),
               topic=settings.image_topic,
               base=settings.video_base_url,
               text=theme.TEXT, muted=theme.MUTED,
               amber=theme.AMBER, violet=theme.VIOLET))

    ui.add_body_html(_OVERLAY_JS)
