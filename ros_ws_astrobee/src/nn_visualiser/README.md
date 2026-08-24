# BEECORE NN visualiser

Standalone 2D visualiser for the behavioural-cloning policy. Reads a rosbag
directly, derives the 13-element observation and 7-element action, runs them
through a network, and draws the whole thing as an animated graph.

Entirely separate from `beecore_capture_gui`. No roscore, no ROS master, no
`rosbag play`, no `rospy`.

## Run

```bash
python3 run_vis.py          # http://localhost:8095
```

Port 8095 to stay clear of 8090/8091. Paste a bag path, set robot and tool
names, press Load.

Only dependency beyond the standard library is `nicegui` and `numpy`.
`torch` is needed **only** on the `Network.from_sb3` path, imported lazily —
the visualiser itself never touches it.

## Layout

```
nn_visualiser/
├── run_vis.py
├── README.md
└── nnvis/
    ├── config.py       every tunable: colours, layout, network shape, topics
    ├── bagread.py      pure-Python rosbag v2.0 reader + five deserialisers
    ├── geometry.py     quaternions, the relative-twist derivation
    ├── dataset.py      bag -> observations/actions, resampled to 30 Hz
    ├── network.py      the stand-in MLP + the SB3 adapter contract
    ├── scene.py        assembles the binary payload the browser fetches
    └── ui/
        ├── page.py     NiceGUI shell and the three data routes
        └── static/canvas.js   the entire renderer
```

## How it draws

**Python ships the whole sequence once.** A 46 s bag at 30 fps is 1394 frames
× 232 floats = 1.3 MB, fetched in one request. Weights are another 41 KB.
After that there is **no websocket traffic during playback at all** — the
browser owns the playhead, so scrub, pause, frame-step and rate are instant
and frame timing cannot be disturbed by anything Python is doing.

No NiceGUI element is created per neuron. That would be one socket.io message
per property change; ~200 neurons at 30 fps is 6000 messages/s and it falls
over gradually enough that you would blame ROS first.

**All 211 neurons are drawn.** Only the edges are aggregated, and only
because 13→128→64→7 is 10,304 connections — at any usable alpha the 128×64
block is a solid grey rectangle. Legibility, not throughput.

**Edges are drawn in two passes.**

1. *Ribbons* — the weight matrix block-pooled to 16 and 8 groups, giving
   392 tapered bands. Brightness is `Σ|aᵢ·Wᵢⱼ|` over the block, hue is
   `sign(Σ aᵢ·Wᵢⱼ)`. Pooling the signed values alone would cancel to ~0 in
   every block precisely when the layer is busiest, so magnitude and sign are
   pooled separately. Both factorise (`Σ|aᵢ||Wᵢⱼ| = |aᵢ|·Σ|Wᵢⱼ|`), so the
   per-frame cost is 128×8 rather than 128×64.
2. *Bright edges* — a true top-K of `|aᵢ·Wᵢⱼ|` per gap, at 5% of that gap's
   edge count (83 / 220 / 22). A fixed threshold was tried first and gave 280
   edges in the wide gap against 9 in the narrow one, which read as two
   different drawings.

Edges are driven by **contribution**, not weight. Weight alone is static —
the picture would be beautiful and never change.

**Hidden units are sorted by activation variance** over the bag. Hidden-unit
ordering is arbitrary, so any grouping of them is decorative; sorting at least
makes the grouping stable and puts the busy units together, so the ribbons
show a gradient instead of static. Applied as a permutation of the weight
matrices, so the drawn network is numerically identical to the fitted one.

**Colour.** Grey → electric blue for ReLU activations, which are never
negative. Amber ← grey → electric blue for the signed channels: observations
and F/T. Violet for the gripper. Both ramps run through `pow(t, 0.65)` — a
linear ramp makes almost everything look dead.

Output channels are scaled **per channel** — force saturates at 0.8 N and
torque at 0.05 Nm, so one shared scale renders every torque channel flat grey.

**The printed number and the colour are different quantities**, and every
channel prints its own denominator so they can't be confused. In the
observation column the colour is `(value − mean) / 3σ`, i.e. "how unusual is
this for this channel", while the number is metres or rad/s — so `pz = 0.01`
saturates (3σ below its mean) while `qz = 0.01` stays grey (sitting at its
mean). The `σ` caption is the missing denominator. In the action columns the
`±` caption plays the same role: without it, the same printed 0.01 is 6×
brighter on a torque channel than a force one.

Decimal places are per channel, ~3 significant figures across each channel's
range. `toFixed(2)` on a torque channel whose full range is 0.05 Nm leaves
five printable levels across the whole range.

## The network is a stand-in, and not a trained one

Hidden layers are random He-initialised features. **Only the output layer is
fitted**, by ridge regression onto the recorded `/joy_wrench`. That makes the
predicted column visibly track the actual column instead of shimmering — but
it is a random-features linear readout, not behavioural cloning, and nothing
about its behaviour predicts how a real policy will behave.

On the 2026-08-24 bag: R² of 0.34–0.64 on the six force/torque channels, 0.996
on the gripper. No dead units in either hidden layer; ~50% ReLU sparsity,
which is about right for the look.

`Network.from_sb3(bc_trainer.policy)` is the swap-in point. It reads
`policy.mlp_extractor.policy_net` and `policy.action_net`, and deliberately
ignores `mlp_extractor.value_net`. **Three things to check there rather than
assume**, all documented in the docstring: SB3 defaults to tanh not ReLU;
`action_net` emits a Gaussian *mean* with `log_std` and a clip living outside
those weights; and SB3 has no mixed continuous/discrete action space, so the
gripper must live in the same `Box` and be thresholded.

## Observation

13 channels: 3 position, 4 quaternion, 3 linear velocity, 3 angular velocity —
all tool-relative-to-`perch_cam`, expressed in `perch_cam` axes.

`config.TOPIC_EST_POSE` / `TOPIC_EST_TWIST` are placeholders for the real
estimator. Until those topics exist, everything is derived from
`/gazebo/model_states` ground truth. **That is fine for the visualiser and not
fine for training a policy you intend to run on estimates.**

The relative twist is

```
ω_rel = R_wp^T (ω_tool − ω_body)
v_rel = R_wp^T (v_tool − v_body − ω_body × (p_tool − p_body))
```

The lever arm runs from the **body** origin, not perch_cam — the camera offset
appears twice with opposite sign and cancels exactly. Verified on two
independent bags by finite-differencing the tool pose in the perch frame:
body-origin is ~20× closer than perch-origin and ~50× closer than omitting the
lever arm, with median error 0.19% of signal linear and 0.06% angular. A
least-squares CoM fit returns zero, so Gazebo's `ModelStates` twist is at the
link origin, not the CoM.

`body → perch_cam` is read from `/tf_static` when present, with the
`tf_echo` values as fallback.

Quaternions are canonicalised to `w ≥ 0`. This produces a genuine
discontinuity where the trajectory passes through `w = 0` — four times in the
46 s bag. **Accepted, not a bug.** Do not fix it by tracking sign continuity
across samples: that makes the observation history-dependent and breaks the
Markov property BC relies on. A 6D rotation representation is the real fix if
it ever matters.

## Action

7 channels: `Fx Fy Fz Tx Ty Tz grip`.

The first six come from `/joy_wrench`. The gripper is a **held** 0/1 flag,
taken from `/honey/beh/arm/goal` (`GRIPPER_CLOSE = 8`), falling back to
`/joy` `buttons[0]` — note `buttons[0]`, **not** `axes[0]`, which is the left
stick X.

Not from the observed gripper motion, for two reasons. First, in simulation
there is no motion: `joint_sample` `angle_pos` takes exactly two values, 0 and
100, and `angle_vel` is exactly 0.0000 at all 456 samples. Second, the state
change lands **1.000 s** after the button press — labelling on it would put
the policy 125 timesteps behind the operator's decision.

Class imbalance is real and unaddressed: one 0→1 transition per demonstration.

## Normalisation

Observations are standardised per channel using statistics from a pre-pass
over the whole bag. The same statistics set the display ranges. Not hardcoded
because the second test bag spins the tool 4× faster than the first (0.90 vs
0.23 rad/s), so ranges tuned to one clip the other badly.

## Verified vs not

Verified against the real bag:

- Bag parsing — all topic counts match `rosbag info` exactly.
- Relative twist — finite-difference agreement on two independent bags.
- `body → perch_cam` — `/tf_static` matches the hardcoded fallback.
- Gripper timeline — button, goal and state transition all decoded.
- Frame packing — the JS layer offsets round-trip to the Python activations
  bit-exactly (max error 0.00e+00 on both hidden layers).
- Renderer maths — `canvas.js` was driven headlessly under Node with a stub
  2D context for 300 frames: **1.8 ms/frame**, 661 fills, 325 strokes, 316
  arcs per frame.

**Not verified, because it cannot be from here:**

- Nothing has ever been rendered in a browser. `nicegui` is not installed in
  the environment this was written in. The 1.8 ms figure is the *maths* only —
  actual rasterisation of ~350 large translucent bezier quads per frame is the
  unmeasured part, and it is the part most likely to disappoint on Mesa/Intel.
  If it stutters, drop `RIBBON_GROUPS` to `{1: 12, 2: 6}` first; that is 156
  ribbons instead of 392.
- `ui.run_javascript(...)` is called fire-and-forget in `page.py`. That is
  correct on NiceGUI ≥1.4. Older versions want `respond=False`. I do not know
  which version is in your container.
- Colour and layout choices are only partly checked against a screenshot.
  Everything is in `config.py`.

### Ribbon density

At the shipped settings the 128->64 gap renders as a near-solid slab: 128
ribbons, each spanning a large vertical extent, additively composited. It
looks striking and it does bury the individual hidden neurons behind it. If
you want the neuron columns to read through, the knob is the ribbon alpha in
`canvas.js` -- `0.055 + 0.10 * v * v`. Halving both constants, or dividing
them by `sqrt(gA)` so a wide fan-out does not accumulate more ink than a
narrow one, are the two obvious options. Not changed unasked.

## Known limits

1. The tool is looked up by name every message; if it is absent (not yet
   spawned) that sample is dropped and the count is logged.
2. Playback loops at the end. No stop-at-end.
3. Bag is read fully into memory. Fine at 67 MB, not at 5.5 GB — filter first.
4. One bag at a time; loading a second replaces the first for every connected
   browser. There is no per-tab session, deliberately: this is a single-user
   tool.
5. Ribbon geometry overlaps slightly between adjacent bands. That is what
   makes the bundle read as continuous rather than as stripes — intentional.
