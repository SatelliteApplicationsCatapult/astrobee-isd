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
    ├── follow_cam.py           spawns and pins the four camera models
    ├── video_server.py         manages web_video_server
    ├── diagnostics.py          background poller for the LED column
    ├── tools.py                delete / spawn / perturb
    ├── reset.py                robot home + fault clear + tools
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
| `CUSTOM_WS` | `/src/custom_ws` | derives the default tool models directory |

On first run the GUI migrates `~/.astrobee_data_gui.json` if the new file does
not exist, so the rename does not reset your camera offset, topic selection or
experiment ID. The old `ASTROBEE_GUI_PORT` / `ASTROBEE_GUI_HOST` variables are
still honoured as fallbacks.

## The cameras

There are **four**, each a **standalone Gazebo model**, not part of the robot
description. Nothing in this project requires editing the Astrobee URDF — if
you added the old xacro, revert it.

| | model / TF frame | image topic |
| --- | --- | --- |
| Camera 1 | `follow_cam_1` | `/<ns>/follow_cam_1/image_raw` |
| Camera 2 | `follow_cam_2` | `/<ns>/follow_cam_2/image_raw` |
| Camera 3 | `follow_cam_3` | `/<ns>/follow_cam_3/image_raw` |
| Camera 4 | `follow_cam_4` | `/<ns>/follow_cam_4/image_raw` |

Note the topic **renamed** from `/<ns>/follow_cam/image_raw`. If you had it
picked on the Topics tab, reselect it.

`models/follow_cam.sdf.template` is rendered once per camera and spawned by
`beecore/follow_cam.py`, which then keeps them glued to the robot: read the
robot pose from `/gazebo/model_states`, compose each offset, publish to the
`/gazebo/set_model_state` **topic** at 60 Hz. The topic, not the service — it
is designed for exactly this and is far cheaper at rate.

**One subscriber, one pin thread.** The robot pose is read once per
`model_states` message and reused for all four; a single thread publishes four
`ModelState` messages per tick. Four subscribers and four threads would do the
same work four times over and let the cameras drift a frame apart from each
other.

Each camera carries its own offset, saved default, FOV, resolution and frame
rate. Camera 1 keeps whatever the single-camera build had; cameras 2–4 start
at that same offset — so all four are co-located until you move them apart —
at 640×480/30 Hz rather than 1080p/60, because four 1080p sensors is a large
step up in render load to take unasked. Raise them on the Camera tab if the
machine copes.

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

The selector at the top of the Camera tab chooses which camera every control
below acts on — pad, rotation, readout, defaults, optics, respawn. It is one
set of widgets refreshed from the selected camera, not four copies of the tab,
and the selection is per browser tab rather than persisted.

- **Set as default** captures the selected camera's current offset; persists
  across restarts, per camera.
- **Reset to default** returns that camera to it.
- Offset changes take effect on the next pin, within ~16 ms.
- **FOV and resolution are baked in at spawn**, so changing them deletes and
  respawns *that* camera. Its image topic drops for about a second; the other
  three are unaffected. The button is disabled while recording.
- The line beside the selector reads *n of 4 spawned*, so a camera that failed
  to spawn is visible without switching to it.

## Image viewer

**Open image view** on the Camera tab opens `/view` — a bare page showing the
four MJPEG streams from `web_video_server` in a 2×2 grid. Drag it to a second
display and press F11 for a clean fullscreen monitor.

**Four streams, one server.** All four `<img>` tags point at the same
`web_video_server` on the same port; only the `topic` query parameter differs,
and the server spins up an encoder per connection. A second server would be
another port to manage, another process to reap, and no benefit.

Each tile reconnects independently, so respawning one camera to change its FOV
does not disturb the other three.

It uses a **named window target**, not `_blank`, so the browser owns the
lifecycle: clicking again reuses the window rather than opening another, and if
you closed it you get a fresh one. Nothing is tracked server-side.

Reuse means no reload, so a **namespace change (or a camera rename) will not
reach an already-open viewer** — use the reload link in its overlay (which
fades out after a couple of seconds of no mouse movement).

The page is deliberately outside the single-tab lock; a viewer window locking
out the control window would be maddening.

No subprocess is involved, so there is nothing that can hang the way
`rqt_image_view` does when SIGINT arrives while Qt is blocked in C. The stream
also survives a GUI restart, since it points at `web_video_server` rather than
at this process.

Host, port and JPEG quality are on the Other tab. The URL is resolved by the
**browser**, not the container, so it has to be an address your browser can
reach. The port is both what the server is launched on and what the browser
fetches, so the two cannot drift apart.

`web_video_server` is started and stopped by the GUI (Other tab, or
automatically at launch if **Start automatically** is ticked). If something is
already serving on the port the GUI leaves it alone rather than fighting it,
and says so. Port 8091 rather than the default 8080, because 8080 on this host
belongs to **Monitorix**, a system monitor running outside the container. Port
map: 8080 Monitorix, 8090 GUI, 8091 video.

### No stream width control

There is no width setting, and that is deliberate. `web_video_server` parses
`width` and `height` off the query string and then overwrites both with the
input image size before the resize check runs, so the resize branch is
unreachable — upstream issue #119 against `RobotWebTools/web_video_server`,
where two `if (output_width_ == -1)` guards were deleted. The fix (PR #130) is
unmerged. This is the published cause of exactly the symptom we saw; it has not
been confirmed against this container's build. To check:

```bash
curl -s "http://localhost:8091/snapshot?topic=/honey/follow_cam/image_raw&width=320" \
     -o /tmp/s.jpg && file /tmp/s.jpg
```

If `file` reports the sensor resolution rather than 320 wide, this build has
the bug. `quality` is applied at encode time and does work. To change the
streamed image size, use the **sensor resolution** on the Camera tab — that
changes what Gazebo renders, which is also the only one of the two that
reduces load.

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

1. teleport the robot to its home pose, velocity zeroed
   (`/gazebo/set_model_state`, the service)
2. clear the fault state — publish `ff_msgs/FaultState {state: 0}` to
   `<ns>/mgt/sys_monitor/state`, latched; the monitor takes itself to
   FUNCTIONAL from there
3. delete every known tool model present in Gazebo
4. sample a pose in a 0.5 m box in front of the perch cam
   (`x,y ∈ [-0.25, 0.25]`, `z ∈ [0.2, 0.7]`, perch_cam frame — each axis
   spans 0.5 m, so the tool lands at most 0.25 m off the boresight)
5. spawn the selected SDF with a uniformly random orientation (Shoemake, not
   random Euler — that would cluster near the poles)
6. apply a 0.1 s wrench impulse, then read the tool's velocity back

Robot before tool: the spawn box is relative to `perch_cam`, so sampling it
first would place the tool relative to wherever the robot had drifted to. No
settle time between the two — `HOME_SETTLE_S` in `config.py` if that turns out
to be needed.

### The home pose

Not hard-coded. The robot always starts in the same place, but that place
belongs to the world file, not to this GUI, and typing it in twice is how the
two drift apart. It is captured from Gazebo the first time the GUI sees the
robot after launch, saved to the config file, and re-capturable with **Set home
to current pose** on the Experiment tab. The current value is shown under the
reset button.

**No home pose captured means the teleport is skipped**, logged, and said so in
the confirm dialog. A guessed pose that puts the robot inside a wall is worse
than no teleport.

The perch_cam pose comes from `/gazebo/model_states` composed with the known
static `body -> perch_cam` transform, so there is no TF lookup in the loop.

XY perturbation is biased back toward the boresight in proportion to how far
off-axis the tool spawned, so edge spawns do not immediately drift out of view.
Z is left uniform. Tune with `CENTRING_BIAS` in `config.py`.

### Perturbation magnitude

Bounded by `FORCE_MAX_N` (3.0 N) and `TORQUE_MAX_NM` (6.0 Nm), derived from the
tool's real properties in `config.py` — mass 1.0 kg, inertia 0.083 kg·m², read
off `ratchet_wrench.sdf`. Over a 0.1 s impulse that is up to **0.30 m/s** and
**414 °/s**.

Torque is in **newton-metres**, not milli. The old mNm slider topped out at
0.69 °/s on this tool, which is why torque looked like it was not being applied
at all — it was, at a rate you could not see. `NOMINAL_TOOL_INERTIA` had been
guessed at 1e-3, 83× too small.

The SDF's 0.083 is `1/12`, i.e. the placeholder for a 1 m cube from the Gazebo
inertia tutorial rather than anything wrench-shaped. A real ~0.3 m wrench is
nearer 0.008, which would spin **ten times faster** for the same torque. If you
put a real inertia in the SDF, halve these maxima.

After every impulse the tool's velocity is read back off `/gazebo/model_states`
and logged as measured m/s and °/s, and stored in `metadata.json`. The figures
under the sliders are a prediction; the log line is the measurement. If it
reports zeros, the wrench did not land — that is a different bug from "too
small to see".

What the reset actually did — tool, spawn pose in both frames, and the applied
wrench — is kept and written into the next bag's `metadata.json` under `reset`,
with its own timestamp. If no reset has succeeded since launch, or the last one
failed, that key is `null` rather than stale.

## Output

```
<save_dir>/20260816_163245_001_handrail_grasp/
├── 20260816_163245_001_handrail_grasp.bag
├── SUCCESS                       # or FAILURE
└── metadata.json                 # topics, bag summary, all four camera
                                  # poses and optics, namespace, and the reset
                                  # that set the scene (measured tool velocity
                                  # included)
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
