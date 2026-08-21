# BEECORE Capture GUI

Browser GUI for driving capture runs on the Astrobee simulator: set up a
scenario, record a rosbag, mark the outcome. ROS Noetic / Python 3.8 / NiceGUI.

## Layout

```
beecore_capture_gui/            <- the project; this is the docker cp target
├── run_gui.py                  entry point: pre-flight, wiring, ui.run
├── README.md
├── assets/
│   ├── SA_SM_White.png         used on the violet banner
│   └── SA_SM_Colour.png        spare, for a light banner
├── models/
│   └── follow_cam.sdf.template standalone camera model, spawned by the GUI
└── beecore/                    <- the importable package
    ├── config.py               constants + Settings (everything persisted)
    ├── state.py                runtime state shared across tabs
    ├── theme.py                brand palette and CSS
    ├── logbridge.py            thread-safe logging into the debug pane
    ├── naming.py               bag folder naming, experiment ID scanning
    ├── geometry.py             quaternion helpers, no numpy
    ├── ros_link.py             the only module that imports rospy
    ├── recorder.py             BagRecorder: rosbag subprocess lifecycle
    ├── follow_cam.py           spawns and pins the camera model
    ├── video_server.py         manages web_video_server
    ├── diagnostics.py          background poller for the LED column
    ├── tools.py                delete / spawn / perturb
    └── ui/
        ├── page.py             banner, status strip, splitter, lock
        ├── viewer.py           /view - the standalone image window
        ├── experiment_tab.py
        ├── camera_tab.py
        ├── topics_tab.py
        └── other_tab.py
```

The outer folder is the project, the inner one is the Python package - the
standard Python split. `recorder.py`, `tools.py`, `geometry.py`,
`video_server.py` and `diagnostics.py` import no NiceGUI, so they can be driven
from a plain Python shell.

## Install

```bash
python3 -m venv --system-site-packages ~/guienv   # keeps rospy visible
source ~/guienv/bin/activate
pip install "nicegui<3"                           # 3.0 dropped Python 3.8
```

## Run

```bash
source /opt/ros/noetic/setup.bash
source <astrobee_ws>/devel/setup.bash             # needed for ff_msgs
source ~/guienv/bin/activate
python3 run_gui.py
```

Browse to `http://localhost:8090`.

| Variable | Default | Purpose |
| --- | --- | --- |
| `BEECORE_GUI_PORT` | `8090` | 8080 is taken by Monitorix on this host |
| `BEECORE_GUI_HOST` | `0.0.0.0` | |
| `BEECORE_GUI_CONFIG` | `~/.beecore_capture_gui.json` | settings file |
| `BEECORE_VIDEO_HOST` | `localhost` | as the **browser** resolves it |
| `BEECORE_VIDEO_PORT` | `8091` | web_video_server, started by the GUI |

On first run the GUI migrates `~/.astrobee_data_gui.json` if the new file does
not exist, so the rename does not reset your camera offset, topic selection or
experiment ID. The old `ASTROBEE_GUI_PORT` / `ASTROBEE_GUI_HOST` variables are
still honoured as fallbacks.
| `CUSTOM_WS` | `/src/custom_ws` | derives the default tool models directory |

## The camera

`follow_cam` is a **standalone Gazebo model**, not part of the robot
description. Nothing in this project requires editing the Astrobee URDF — if
you added the old xacro, revert it.

`models/follow_cam.sdf.template` is spawned by `beecore_capture_gui/follow_cam.py`,
which then keeps it glued to the robot: read the robot pose from
`/gazebo/model_states`, compose your offset, publish to the
`/gazebo/set_model_state` **topic** at 60 Hz. The topic, not the service — it
is designed for exactly this and is far cheaper at rate.

### Why not the URDF

Putting the camera in the robot description makes it part of Astrobee's
rigid-body dynamics, which drags in everything unrelated to looking at things:
links need inertia, bad inertia NaNs the whole articulated body, unactuated
joints wobble, actuated ones need controllers, and a mistake breaks the robot
rather than the camera.

As a separate model there is no physics coupling at all. No joints, no inertia,
no damping, no NaN, no wobble, no added mass, no FSW mass mismatch. Instant
6 DOF. The camera is teleported, not simulated.

The cost: pinning runs at 60 Hz while physics runs faster, so under hard
acceleration the camera lags by a sub-millimetre. The robot's twist is
published along with the pose, which smooths what remains.

### Frames

The SDF puts the perch_cam-to-Gazebo-camera rotation on the **sensor** pose, so
the model pose is in plain perch_cam convention:

| axis | direction |
| --- | --- |
| +X | down |
| +Y | right |
| +Z | forward, out of the lens |

Rotation senses about those axes: **pan** +ve swings the view left, **tilt**
+ve pitches it down, **roll** +ve rolls the scene clockwise. The GUI inverts
whichever it needs to; every flip is one entry in `_SIGN` at the top of
`ui/camera_tab.py`.

### Offsets and optics

- **Set as default** captures the current offset; persists across restarts.
- **Reset to default** returns to it; **Zero** puts the camera exactly on
  perch_cam.
- Offset changes take effect on the next pin, within ~16 ms.
- **FOV and resolution are baked in at spawn**, so changing them deletes and
  respawns the model. The image topic drops for about a second — the button is
  disabled while recording.

## Image viewer

**Open image view** on the Camera tab opens `/view` — a bare page showing only
the MJPEG stream from `web_video_server`. Drag it to a second display and press
F11 for a clean fullscreen monitor.

It uses a **named window target**, not `_blank`, so the browser owns the
lifecycle: clicking again reuses the window rather than opening another, and if
you closed it you get a fresh one. Nothing is tracked server-side.

Reuse means no reload, so a **namespace change will not reach an already-open
viewer** — use the reload link in its overlay (which fades out after a couple of
seconds of no mouse movement).

The page is deliberately outside the single-tab lock; a viewer window locking
out the control window would be maddening.

No subprocess is involved, so there is nothing that can hang the way
`rqt_image_view` does when SIGINT arrives while Qt is blocked in C. The stream
also survives a GUI restart, since it points at `web_video_server` rather than
at this process.

Base URL, width and JPEG quality are on the Other tab. The URL is resolved by
the **browser**, not the container, so it has to be an address your browser can
reach.

`web_video_server` is a separate node and is not started by the GUI:

```bash
rosrun web_video_server web_video_server _port:=8081
curl -s http://localhost:8081/streams.html | head    # should list the topic
```

Port 8081 rather than the default 8080, because 8080 on this host belongs to
**Monitorix**, a system monitor running outside the container. Port map:
8080 Monitorix, 8090 GUI, 8081 video.

## Diagnostics

Green always means **ready to run**, never "this is switched on" — which is why
GNC shows green when it is *disarmed*.

Services expose no readable state, so `gnc/ctl/enable` and `start` report two
things separately: whether the service is advertised, and what this GUI last
commanded. The switches on the left column are what command them, and therefore
the only thing that can move those two indicators — anything else calling those
services goes unnoticed.

## Single-tab lock

One browser tab holds control; others get a lock screen with an explicit
takeover, so a crashed tab can never strand the GUI.

Identity is the NiceGUI **client id**, unique per page load. Not
`app.storage.tab` — that is keyed by an id in `sessionStorage`, and both
Firefox and Chrome *copy* `sessionStorage` into a duplicated tab, so a
duplicate inherits the original's identity and slips straight through.

A client id changes on reload too, which would make a refresh look like a
second tab. Two things cover that:

- A `pagehide` beacon to `/_gui/release` — installed on the lock screen as well
  as the main page — so a closing or reloading tab leaves no claim behind.
- **Take control does not claim the lock.** It drops it and opens a 10 second
  grant window that the reloading page walks into. Claiming with the current
  client id would be useless, because the reload mints a new one and the page
  would lock itself out with its own stale entry.

If the beacon never fires (browser crash, bfcache), a 3 second heartbeat
staleness frees the lock instead. A tab that loses control reloads once, not
once per tick.

## Reset

Currently tools only. Robot teleport and the fault-state clear are not wired in.

1. delete every known tool model present in Gazebo
2. sample a pose in a 1 m box in front of the perch cam
   (`x,y ∈ [-0.5, 0.5]`, `z ∈ [0.2, 1.2]`, perch_cam frame)
3. spawn the selected SDF with a uniformly random orientation (Shoemake, not
   random Euler — that would cluster near the poles)
4. apply a 0.1 s wrench impulse

The perch_cam pose comes from `/gazebo/model_states` composed with the known
static `body -> perch_cam` transform, so there is no TF lookup in the loop.

XY perturbation is biased back toward the boresight in proportion to how far
off-axis the tool spawned, so edge spawns do not immediately drift out of view.
Z is left uniform. Tune with `CENTRING_BIAS` in `config.py`.

`MAX_FORCE_N` and `MAX_TORQUE_NM` assume roughly a 0.5 kg tool with
~1e-3 kg·m² inertia over a 0.1 s impulse, giving about 0.03 m/s and 10 deg/s.
Adjust if your tools are much heavier.

## Output

```
<save_dir>/20260816_163245_001_handrail_grasp/
├── 20260816_163245_001_handrail_grasp.bag
├── SUCCESS                       # or FAILURE
└── metadata.json                 # includes camera joint state and namespace
```

Play back with a glob, not the folder:

```bash
rosbag play 20260816_163245_001_handrail_grasp/*.bag
```

## Keyboard

Function keys only, so they cannot fire while typing in a text field.

| Key | Action |
| --- | --- |
| F8 | Start recording |
| F9 | Stop recording |
| F6 | Mark capture success |
| F7 | Mark capture failure |
