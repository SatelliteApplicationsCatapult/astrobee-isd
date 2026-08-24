"""Precompute everything the canvas needs, once, per bag.

The whole sequence is shipped to the browser up front rather than streamed a
frame at a time.  At 30 fps a 46 s bag is ~1400 frames x 232 floats = 1.3 MB,
which is a single fetch.  In exchange there is no per-frame websocket traffic
at all, playback cannot stutter because Python is busy, and scrubbing, pause
and rate are pure client-side operations.
"""
import numpy as np

from . import config as C
from . import dataset
from .network import Network


def build(path, robot, tool, log=print):
    d = dataset.load(path, robot, tool, log=log)
    obs, act = d['obs'], d['act']

    mean, std = dataset.standardise(obs)
    x = (obs - mean) / std

    net = Network.random()
    net.fit_readout(x, act)
    net.order_hidden(net.forward(x))
    acts = net.forward(x)

    n = len(x)
    pred = acts[-1]

    # per-layer display scale: p99 of |activation|, so one hot unit does not
    # flatten the rest of the layer to invisible grey
    scales = [float(max(np.percentile(np.abs(a), 97), 1e-6)) for a in acts]
    # inputs are standardised, so a fixed +/-3 sigma reads better than a
    # percentile -- it keeps the display range interpretable
    scales[0] = 3.0

    # Actions get a scale PER CHANNEL.  Force saturates at 0.8 N and torque at
    # 0.05 Nm in the test bags: one shared scale would render every torque
    # channel as flat grey.  Predicted and actual share the scale so the two
    # columns stay directly comparable.
    out_scales = np.maximum(np.abs(act).max(0), 1e-6)
    out_scales[-1] = 1.0                       # gripper is already 0..1
    scales[-1] = float(out_scales[0])

    frames = np.concatenate([
        x.astype(np.float32),                      # 0..12    standardised in
        acts[1].astype(np.float32),                # 13..140  h1
        acts[2].astype(np.float32),                # 141..204 h2
        pred.astype(np.float32),                   # 205..211 predicted
        act.astype(np.float32),                    # 212..218 actual
        obs.astype(np.float32),                    # 219..231 raw in
    ], 1)

    weights = np.concatenate([W.ravel() for W, _ in net.layers]).astype(np.float32)

    meta = {
        'sizes': net.sizes,
        'n_frames': n,
        'stride': frames.shape[1],
        'fps': C.FPS,
        'duration': float(d['duration']),
        'in_labels': C.IN_LABELS,
        'out_labels': C.OUT_LABELS,
        'scales': [float(s) for s in scales],
        'out_scales': out_scales.tolist(),
        'ribbon_groups': [C.RIBBON_GROUPS[i] for i in range(len(net.sizes))],
        'edge_counts': [int(min(C.EDGE_CAP, max(C.EDGE_MIN,
                        round(C.EDGE_FRACTION * a * b))))
                        for a, b in zip(net.sizes[:-1], net.sizes[1:])],
        'palette': C.PALETTE,
        'col_x': C.COL_X,
        'col_top': C.COL_TOP,
        'col_bottom': C.COL_BOTTOM,
        'canvas': [C.CANVAS_W, C.CANVAS_H],
        'offsets': {'in': 0, 'h1': 13, 'h2': 141,
                    'pred': 205, 'act': 212, 'raw': 219},
        'grip_t': d['grip_t'],
        'grip_source': d['grip_source'],
        'obs_mean': mean.tolist(),
        'obs_std': std.tolist(),
        'n_raw': d['n_raw'],
    }
    return meta, frames.tobytes(), weights.tobytes()
