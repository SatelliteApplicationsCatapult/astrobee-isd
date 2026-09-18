# BEECORE NN visualiser

Standalone 2D visualiser for the behavioural-cloning policy. Reads a rosbag
directly, derives the 18-element observation and 7-element action, runs them
through a network, and draws the whole thing as an animated graph.

Entirely separate from `beecore_capture_gui`. No roscore, no ROS master, no
`rosbag play`, no `rospy`.

## Run

```bash
python3 run_vis.py          # http://localhost:8095
```

Port 8095 to stay clear of 8090/8091. Paste a bag path, set robot and tool
names, optionally a `policy.pt`, press Load.

Only dependency beyond the standard library is `nicegui` and `numpy`. **torch
is not needed at all**, not even to read a trained policy — `torchload.py`
parses a `state_dict` .pt directly, since it is a zip of a flat pickle plus
raw tensor buffers. Keeping a 2 GB dependency out of a tool that multiplies
three matrices was worth the ~120 lines.

## Loading a trained policy

Leave the policy field blank and you get the untrained stand-in, as before.
Point it at `policy.pt` and the visualiser draws the real behaviourally-cloned
network. `obs_stats.json` is picked up from the same directory automatically.

Both files come from `bc_first_test.py`'s `save_trained_policy()`:

```
bc_policy/
├── policy.pt         torch state_dict of the SB3 ActorCriticPolicy
└── obs_stats.json    obs_mean, obs_sigma, act_scale, activation, channel names
```

**The stats file is not optional metadata.** The network was fitted on
`(obs − train_mean) / train_sigma`; standardising with this bag's own
statistics instead is a different transform, so the predicted column becomes
the network's output on an input it never saw in that form — wrong in a way
that looks exactly like ordinary BC error. Without the file the visualiser
falls back to bag statistics and puts a red warning on the canvas.

Two things the `state_dict` cannot carry, both of which come from the JSON:

- **The activation.** An activation module has no parameters, so it leaves
  nothing in the file but a gap in the layer numbering (`policy_net.0`,
  `policy_net.2`). SB3 defaults to **tanh**, not ReLU. Absent from the stats,
  tanh is assumed and logged.
- **`act_scale`.** Training divides the recorded wrench by it, so the policy
  emits roughly ±1 per channel while the recorded action is in newtons and
  newton-metres. The predicted column is multiplied back into physical units
  before display; without that every torque channel reads 16× wrong.

Depth and width are read from the weights, so `[32, 32]`, `[256, 256, 128]`
and anything else all work with no config change. Column positions and canvas
width are computed from the layer count.

## Layout

```
nn_visualiser/
├── run_vis.py
├── README.md
└── nnvis/
    ├── config.py       every tunable: colours, layout, network shape, topics
    ├── bagread.py      pure-Python rosbag v2.0 reader + five deserialisers
    ├── geometry.py     rotations, the relative-twist derivation
    ├── dataset.py      bag -> observations/actions, resampled to 30 Hz
    ├── torchload.py    reads a torch state_dict .pt with no torch installed
    ├── network.py      the trained-policy loader + the stand-in MLP
    ├── scene.py        assembles the binary payload the browser fetches
    └── ui/
        ├── page.py     NiceGUI shell and the three data routes
        └── static/canvas.js   the entire renderer
```

## How it draws

**Playback is wall-clock locked, and honest about it.** The playhead advances
by real elapsed time with no clamp on `dt`, so a slow renderer drops frames
rather than slowing down. An earlier `Math.min(0.1, ...)` silently discarded
any time beyond 100 ms, which made playback run at `0.1 * render_fps` below
10 fps -- 0.5x at 5 fps -- while the clock readout still claimed 1x. The
backgrounded-tab leap that clamp was guarding against is handled by resetting
the frame clock on `visibilitychange`. Render fps is shown next to the frame
counter so a struggling renderer is visible rather than inferred.

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

**Colour.** Amber ← grey → electric blue for signed channels; grey →
electric blue for channels that can only be positive. Violet for the gripper.
Both ramps run through `pow(t, 0.65)` — a linear ramp makes almost everything
look dead.

**Which ramp a hidden layer gets follows its activation**, not a constant.
ReLU output is `[0, ∞)` and takes the one-sided ramp; tanh is `[−1, 1]` and
takes the diverging one. Drawing a tanh layer on the ReLU ramp clamps every
negative unit to flat grey and the layer looks half dead. A **bounded**
activation is also drawn against its own limit (±1 for tanh) rather than a
p97 percentile, so a saturated unit reads as saturated instead of being
renormalised back into the middle of the range.

**Small gaps are drawn in full.** Ribbons and top-K selection exist for
legibility at 128×64 — 10,304 edges is a solid grey rectangle at any usable
alpha — not for throughput. Below `DENSE_EDGE_MAX` connections a gap draws
every edge and skips the ribbon substrate entirely; pooling a 32×32 gap into
4×4 blocks is aggregation that does not aggregate, and the ribbons then
obscure edges that were perfectly legible. A trained `FeedForward32Policy` is
1,824 connections total, so it draws in full.

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

## The two network sources

`Network.from_policy(pt, stats)` reads a real trained policy:
`mlp_extractor.policy_net.*` for the hidden stack and `action_net` for the
output. `value_net` is ignored on both counts — BC does not meaningfully
train it. torch's `Linear` stores `(out, in)`, so the weights are transposed
on the way in.

**The gripper channel is NOT squashed on this path.** The stand-in put a
logistic on its last output unit so the value always read as a 0/1 flag. A
trained policy has a bare Linear output: that channel is a plain `Box`
dimension and can emit 1.3 or −0.2. Squashing it here would show a cleaned-up
number instead of the one the network produced, which is exactly what the
predicted-vs-actual gap exists to reveal. The colour clamps; the printed
number does not.

`Network.random()` is the original stand-in, kept so the tool still runs with
no policy file. Hidden layers are random He-initialised features and **only
the output layer is fitted**, by ridge regression onto the recorded
`/joy_wrench`. It is a random-features linear readout, not behavioural
cloning, and nothing about its behaviour predicts a real policy. The canvas
says so in red when it is in use.

Still true and still unverified on the SB3 side: `action_net` emits the *mean*
of a diagonal Gaussian, with `log_std` and the action-bound clip living
outside those weights; and SB3 has no mixed continuous/discrete action space,
so the gripper lives in the same `Box` and has to be thresholded.

## Observation

18 channels: 3 position, **9 rotation-matrix elements** (row-major, `r00`
`r01` `r02` `r10` …), 3 linear velocity, 3 angular velocity — all
tool-relative-to-`perch_cam`, expressed in `perch_cam` axes.

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

**Orientation is a rotation matrix, not a quaternion, and this was a
deliberate change.** `q` and `−q` are the same rotation, so a quaternion
channel jumps by up to 2.0 in a single step for an orientation that barely
moved — four such flips in the 46 s test bag. Canonicalising to `w ≥ 0` does
not remove the discontinuity, it only pins where it happens. Tracking sign
continuity across samples would fix the jump at the cost of making the
observation history-dependent, which breaks the Markov property BC relies on.
`R(q) = R(−q)`, so the matrix has no sign ambiguity at all.

The cost is redundancy: nine numbers for three degrees of freedom, with six
orthonormality constraints the network has to discover from data. The 6D form
of Zhou et al. 2019 — first two columns, third recovered by cross product — is
the non-redundant continuous alternative if input width ever matters.

**The training script and the visualiser must agree channel for channel.**
Training does `quaternion_matrix(q)[:3, :3].flatten()`; `geometry.R_to_mat9`
does the same row-major reshape. A permuted observation would render
perfectly and be entirely wrong. `scene.build` raises if the policy's input
width and the bag's observation width disagree, which catches a policy
trained on the old 13-channel quaternion observation.

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

With a policy loaded, observations are standardised with the **training**
statistics out of `obs_stats.json`. That is the transform the network was
fitted on and the only one under which its output means anything.

Without a policy, statistics come from a pre-pass over the whole bag. Not
hardcoded because the second test bag spins the tool 4× faster than the first
(0.90 vs 0.23 rad/s), so ranges tuned to one clip the other badly. The canvas
reports which source is in use on the status line, next to the network shape.

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

Verified for the trained-policy path:

- `torchload.py` reads the real `policy.pt` with no torch installed: 13 keys,
  shapes matching `18→32→32→7` plus the ignored `value_net`, all float32.
- The predicted block in the shipped payload equals an independent forward
  pass done separately and denormalised — **max abs error 0.00e+00**. Read
  back out of the payload rather than assumed, because a patch script that
  cannot fail is a patch script that cannot be trusted.
- Predicted torque lands near 0.05 Nm and force near 0.8 N, i.e. physical
  units, confirming the `act_scale` multiply is applied.
- Renderer at `18-32-32-7` with all three gaps dense: **1.9 ms/frame**, 1,775
  strokes (every connection above the floor), 96 neurons, headers reading
  `OBSERVATION | HIDDEN 32 | HIDDEN 32 | PREDICTED | ACTUAL`. The stand-in at
  `18-128-64-7` still pools its two wide gaps: 2.7 ms/frame.
- Rotation-matrix channels in `obs_stats.json` satisfy `E[‖row‖²] =
  E[‖col‖²] = 1.000` to six decimals, so they are proper orthonormal.

**Not verified, because it cannot be from here:**

- Nothing has ever been rendered in a browser. `nicegui` is not installed in
  the environment this was written in. The ms figures are the *maths* only —
  actual rasterisation of ~350 large translucent bezier quads per frame is the
  unmeasured part, and it is the part most likely to disappoint on Mesa/Intel.
  If it stutters, lower `RIBBON_TARGET_PER_GROUP` first. With a small policy
  this is moot: dense gaps draw no ribbons at all, only thin strokes.
- The scene builder was exercised against **synthetic** observations, not a
  real bag, because there is no bag in the environment this was written in.
  The plumbing, shapes and forward pass are checked; the numbers going into
  it are not real data.
- Whether the tanh display bound of ±1 reads better than a percentile has not
  been judged by eye. `ACTIVATIONS` in `network.py` is the knob.
- `ui.run_javascript(...)` is called fire-and-forget in `page.py`. That is
  correct on NiceGUI ≥1.4. Older versions want `respond=False`. I do not know
  which version is in your container.
- Colour and layout choices are only partly checked against a screenshot.
  Everything is in `config.py`.

### Ribbon density

Ribbon alpha is `(0.026 + 0.055 * v^2) * alphaK`, with
`alphaK = sqrt(16*8) / sqrt(gA*gB)`. The normalisation matters because
ribbons composite additively, so `gA * gB` of them overlapping in the same
space saturate to a solid slab -- and without it, ink density is a side effect
of how many groups a layer happens to have rather than of the data. Net effect
against the first drop: 0.41x on the input gap, 0.52x on the hidden gap, 0.79x
on the output gap. Turn it up in `canvas.js` if it now reads too faint.

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
