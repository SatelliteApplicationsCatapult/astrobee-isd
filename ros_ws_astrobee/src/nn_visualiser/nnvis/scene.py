"""Precompute everything the canvas needs, once, per bag.

The whole sequence is shipped to the browser up front rather than streamed a
frame at a time.  At 30 fps a 46 s bag is ~1400 frames x a couple of hundred
floats, which is a single fetch.  In exchange there is no per-frame websocket
traffic at all, playback cannot stutter because Python is busy, and scrubbing,
pause and rate are pure client-side operations.

Depth is not fixed here.  The frame buffer is laid out from the network's own
`sizes`, and the offsets travel to the browser in the metadata, so a 2-layer
FeedForward32Policy and a 4-layer one both work without touching the renderer.
"""
import math
import os

import numpy as np

from . import config as C
from . import dataset
from .network import Network


def _dp(scale):
    """Decimal places giving ~3 significant figures across a channel's range.

    toFixed(2) on a torque channel whose full range is 0.05 Nm leaves five
    printable levels across the whole range -- a displayed 0.01 could be
    anything from 0.005 to 0.0149.
    """
    return int(min(4, max(2, 2 - math.floor(math.log10(max(scale, 1e-9))))))


def build(path, robot, tool, policy_path=None, stats_path=None, log=print):
    d = dataset.load(path, robot, tool, log=log)
    obs, act = d['obs'], d['act']

    warnings = []

    # ---- network ------------------------------------------------------
    if policy_path and policy_path.endswith('.onnx'):
        net = Network.from_onnx(policy_path, log=log)
        log('policy: %s -> %s, %s'
            % (os.path.basename(policy_path), net.sizes,
               '/'.join(net.activation)))
    elif policy_path:
        # Dormant .pt + obs_stats.json route.  The UI no longer offers it.
        if stats_path is None:
            guess = os.path.join(os.path.dirname(policy_path), 'obs_stats.json')
            stats_path = guess if os.path.isfile(guess) else None
        if stats_path is None:
            warnings.append('no obs_stats.json found beside the policy')
        net = Network.from_policy(policy_path, stats_path, log=log)
        log('policy: %s -> %s, %s'
            % (os.path.basename(policy_path), net.sizes,
               '/'.join(net.activation)))
    else:
        net = Network.random()
        warnings.append('no policy loaded -- showing the untrained stand-in')

    if net.sizes[0] != obs.shape[1]:
        raise ValueError(
            'the policy takes %d observation channels, this bag produces %d. '
            'A policy trained on the quaternion observation cannot be run on '
            'the rotation-matrix one.' % (net.sizes[0], obs.shape[1]))

    # ---- channel contract ----------------------------------------------
    # Width agreement passes any permutation.  A permuted observation renders
    # perfectly and is entirely wrong, so the names are compared in order.
    # C.IN_LABELS / C.OUT_LABELS are this side's declaration of what
    # dataset.py builds; the file's names are what training used.
    for got, want, what in ((net.in_labels, C.IN_LABELS, 'observation'),
                            (net.out_labels, C.OUT_LABELS, 'action')):
        if got is not None and list(got) != list(want):
            raise ValueError('%s channel names differ from the policy file.\n'
                             '  file:       %s\n  visualiser: %s'
                             % (what, got, want))

    # ---- standardisation ----------------------------------------------
    # The network was fitted on (obs - train_mean) / train_sigma.  Feeding it
    # this bag's own statistics instead is a different transform, so the
    # predicted column would be the network's output on an input it never saw
    # in that form -- wrong in a way that looks exactly like ordinary BC error.
    if net.obs_mean is not None:
        mean, std = net.obs_mean, net.obs_sigma
        stat_src = 'training'
    else:
        mean, std = dataset.standardise(obs)
        stat_src = 'this bag'
        if policy_path:
            warnings.append('standardising with this bag, not the training '
                            'statistics -- predicted column is unreliable')
    log('standardised with %s statistics' % stat_src)
    x = (obs - mean) / std

    # ---- forward ------------------------------------------------------
    if net.obs_mean is None and policy_path is None:
        net.fit_readout(x, act)
    net.order_hidden(net.forward(x))
    acts = net.forward(x)

    n = len(x)
    n_layers = len(net.sizes)
    pred = net.to_physical(acts[-1])

    # ---- display scales ------------------------------------------------
    # inputs are standardised, so a fixed +/-3 sigma reads better than a
    # percentile -- it keeps the display range interpretable
    scales = [3.0]
    for k, bound in enumerate(net.hidden_bound):
        if bound is not None:
            # A bounded activation is drawn against its own limit, so a
            # saturated tanh unit reads as saturated rather than being
            # renormalised back into the middle of the ramp.
            scales.append(float(bound))
        else:
            scales.append(float(max(np.percentile(np.abs(acts[k + 1]), 97), 1e-6)))

    # Actions get a scale PER CHANNEL.  Force saturates at 0.8 N and torque at
    # 0.05 Nm in the test bags: one shared scale would render every torque
    # channel as flat grey.  Predicted and actual share the scale so the two
    # columns stay directly comparable.  The predicted column is included in
    # the max because a policy is free to overshoot the recorded envelope.
    out_scales = np.maximum(np.abs(act).max(0),
                            np.percentile(np.abs(pred), 99, axis=0))
    out_scales = np.maximum(out_scales, 1e-6)
    out_scales[-1] = 1.0                       # gripper is a 0/1 flag
    scales.append(float(out_scales[0]))

    # ---- frame buffer ---------------------------------------------------
    blocks = [x.astype(np.float32)]
    offsets_layers = [0]
    cursor = net.sizes[0]
    for k in range(1, n_layers - 1):
        blocks.append(acts[k].astype(np.float32))
        offsets_layers.append(cursor)
        cursor += net.sizes[k]
    blocks.append(pred.astype(np.float32))
    offsets_layers.append(cursor)
    cursor += net.sizes[-1]

    off_act = cursor
    blocks.append(act.astype(np.float32))
    cursor += net.sizes[-1]

    off_raw = cursor
    blocks.append(obs.astype(np.float32))
    cursor += net.sizes[0]

    frames = np.concatenate(blocks, 1)
    assert frames.shape[1] == cursor, (frames.shape, cursor)

    weights = np.concatenate([W.ravel() for W, _ in net.layers]).astype(np.float32)

    # ---- captions -------------------------------------------------------
    # The printed number is in physical units; the colour is that number
    # divided by the channel's own scale.  Those are different quantities and
    # the scales differ by 16x between force and torque, so the denominator is
    # printed next to every channel rather than left implicit.
    in_caption = [u'\u03c3%.3g' % v for v in std]
    out_caption = [u'%s%.3g' % ('' if i == net.sizes[-1] - 1 else u'\u00b1', v)
                   for i, v in enumerate(out_scales)]
    out_caption[-1] = u'0\u20131'

    hidden_caption = []
    for k, bound in enumerate(net.hidden_bound):
        if bound is not None:
            hidden_caption.append(u'%s \u00b1%.2g' % (net.activation[k], bound))
        else:
            hidden_caption.append(u'%s p97 %.2f' % (net.activation[k], scales[k + 1]))

    sizes = list(net.sizes)
    ribbon = [C.ribbon_groups(s) for s in sizes]
    ribbon[0] = sizes[0]                       # endpoints are never pooled
    ribbon[-1] = sizes[-1]

    gaps = list(zip(sizes[:-1], sizes[1:]))
    dense = [a * b <= C.DENSE_EDGE_MAX for a, b in gaps]

    col_x, canvas_w = C.column_x(n_layers)

    in_labels = net.in_labels or C.IN_LABELS
    out_labels = net.out_labels or C.OUT_LABELS

    meta = {
        'in_dp': [_dp(v) for v in std],
        'out_dp': [_dp(v) for v in out_scales],
        'in_caption': in_caption,
        'out_caption': out_caption,
        'hidden_caption': hidden_caption,
        'sizes': sizes,
        'n_frames': n,
        'stride': int(frames.shape[1]),
        'fps': C.FPS,
        'duration': float(d['duration']),
        'in_labels': list(in_labels),
        'out_labels': list(out_labels),
        'scales': [float(s) for s in scales],
        'out_scales': out_scales.tolist(),
        'activation': list(net.activation),
        'hidden_signed': list(net.hidden_signed),
        'ribbon_groups': ribbon,
        'dense': dense,
        'dense_max': C.DENSE_EDGE_MAX,
        'edge_counts': [int(min(C.EDGE_CAP, max(C.EDGE_MIN,
                        round(C.EDGE_FRACTION * a * b))))
                        for a, b in gaps],
        'palette': C.PALETTE,
        'col_x': col_x,
        'col_top': C.COL_TOP,
        'col_bottom': C.COL_BOTTOM,
        'canvas': [canvas_w, C.CANVAS_H],
        'offsets': {'layers': offsets_layers, 'act': off_act, 'raw': off_raw},
        'grip_t': d['grip_t'],
        'grip_source': d['grip_source'],
        'obs_mean': np.asarray(mean, float).tolist(),
        'obs_std': np.asarray(std, float).tolist(),
        'stat_source': stat_src,
        'net_source': ('stand-in' if policy_path is None
                       else os.path.basename(policy_path)),
        'trained': policy_path is not None,
        'warnings': warnings,
        'n_raw': d['n_raw'],
    }
    return meta, frames.tobytes(), weights.tobytes()
