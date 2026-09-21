"""The network whose activations get drawn.

Three sources, and they behave differently in ways the display has to know
about:

``Network.from_onnx(path)`` reads the ONNX export written by
``bc_first_test.py``.  This is the current route and the only one the UI
offers.  Weights, standardisation statistics, act_scale and channel names all
come out of the one file.

``Network.from_policy(pt, stats)`` is the old route, dormant: nothing in the
UI reaches it, but ``scene.build`` still dispatches a ``.pt`` path to it.  It
reads a real behaviourally-cloned policy
out of an SB3 ``state_dict`` plus the statistics saved beside it.  Hidden
layers use whatever activation the training script recorded (SB3 defaults to
tanh, not ReLU); the output layer is bare Linear, so the action channels are
unbounded and the gripper channel is NOT squashed into 0..1.

``Network.random()`` is the original stand-in: random He-initialised hidden
features with only the output layer fitted by ridge regression.  It is not
trained, it is not behavioural cloning, and nothing about it predicts how a
real policy behaves.  Kept so the visualiser still runs with no policy file.

Depth is read from the weights, not from config -- the drawn network is
whatever shape the file says it is.
"""
import json

import numpy as np

from . import config as C
from .torchload import load_state_dict


def _relu(x):
    return np.maximum(x, 0.0)


def _elu(x):
    return np.where(x > 0, x, np.expm1(np.minimum(x, 0.0)))


def _leaky(x):
    return np.where(x > 0, x, 0.01 * x)


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def _gelu(x):
    return 0.5 * x * (1.0 + np.tanh(0.7978845608 * (x + 0.044715 * x ** 3)))


# ONNX op_type -> ACTIVATIONS key.  Only ops that can sit between two Gemms of
# an SB3 MLP.  Anything else raises rather than being drawn as something else.
ONNX_ACTIVATIONS = {
    'Tanh': 'tanh',
    'Relu': 'relu',
    'Elu': 'elu',
    'LeakyRelu': 'leakyrelu',
    'Sigmoid': 'sigmoid',
}


# name -> (function, signed, bound)
#   signed: can the output go negative?  Decides which colour ramp the layer
#           gets.  A tanh layer drawn on the ReLU ramp loses every negative
#           unit to flat grey.
#   bound:  the activation's own full scale, or None if unbounded.  A bounded
#           activation is drawn against its real limit so saturation reads as
#           saturation instead of being renormalised away.
ACTIVATIONS = {
    'tanh': (np.tanh, True, 1.0),
    'relu': (_relu, False, None),
    'elu': (_elu, True, None),
    'leakyrelu': (_leaky, True, None),
    'sigmoid': (_sigmoid, False, 1.0),
    'gelu': (_gelu, True, None),
    'identity': (lambda x: x, True, None),
}


class Network:
    def __init__(self, layers, activation='relu', grip_logistic=False):
        """layers: list of (W, b) with W shape (fan_in, fan_out).

        activation: one name for the whole hidden stack, or a list with one
        name per hidden layer.  The output layer is always linear.
        """
        self.layers = [(np.asarray(W, np.float32), np.asarray(b, np.float32))
                       for W, b in layers]
        self.sizes = [self.layers[0][0].shape[0]] + [W.shape[1] for W, _ in self.layers]
        self.n_hidden = len(self.layers) - 1

        if isinstance(activation, str):
            activation = [activation] * self.n_hidden
        activation = list(activation)
        if len(activation) != self.n_hidden:
            raise ValueError('%d activations for %d hidden layers'
                             % (len(activation), self.n_hidden))
        for a in activation:
            if a not in ACTIVATIONS:
                raise ValueError('unknown activation %r; known: %s'
                                 % (a, ', '.join(sorted(ACTIVATIONS))))
        self.activation = activation

        # The stand-in squashes its last output unit so the gripper reads 0..1.
        # A trained policy does not: that channel is a plain Box dimension and
        # can emit 1.3 or -0.2.  Squashing it here would show a cleaned-up
        # number rather than the one the network produced, which is exactly
        # what the predicted-vs-actual gap exists to reveal.
        self.grip_logistic = grip_logistic

        # Set by from_policy(); stay None for the stand-in.
        self.obs_mean = None
        self.obs_sigma = None
        self.act_scale = None
        self.in_labels = None
        self.out_labels = None
        self.source = 'stand-in (untrained)'

    # ---- description ---------------------------------------------------
    @property
    def hidden_signed(self):
        return [ACTIVATIONS[a][1] for a in self.activation]

    @property
    def hidden_bound(self):
        return [ACTIVATIONS[a][2] for a in self.activation]

    # ---- forward -------------------------------------------------------
    def forward(self, x):
        """x: (N, n_in).  Returns list of activations, one per layer boundary.

        activations[0] is the input, activations[-1] the output.  The output
        layer is linear, matching SB3's action_net.
        """
        acts = [np.asarray(x, np.float32)]
        h = acts[0]
        last = len(self.layers) - 1
        for k, (W, b) in enumerate(self.layers):
            z = h @ W + b
            if k < last:
                h = ACTIVATIONS[self.activation[k]][0](z).astype(np.float32)
            else:
                h = z.copy()
                if self.grip_logistic:
                    h[:, -1] = _sigmoid(z[:, -1])
            acts.append(h)
        return acts

    def to_physical(self, y):
        """Scale a raw network output into the units the recorded action uses.

        A trained policy was fitted against actions divided by act_scale, so
        its raw output is roughly +/-1 per channel while the recorded wrench
        is in newtons and newton-metres.  Plotting one against the other
        without this multiply makes every torque channel look 16x wrong.
        """
        if self.act_scale is None:
            return y
        return y * self.act_scale

    # ---- construction: ONNX export (current) ---------------------------
    @classmethod
    def from_onnx(cls, path, log=print):
        """Load the self-contained ONNX export of an SB3 ActorCriticPolicy.

        The graph is obs -> (obs - mean)/sigma -> policy_net -> action_net ->
        clip(-1, 1) -> * scale.  The weights are pulled out of the initializers
        by name and run by this class's own forward pass, because the drawing
        needs every hidden activation -- ONNX Runtime would only give the
        output.  That forward pass stops before the clip, so overshoot beyond
        +/-1 x act_scale stays visible.

        log_std is in the file and ignored: it feeds only a Shape node, so it
        cannot reach the deterministic action.  value_net is not in the file
        at all -- the exporter prunes it because 'values' is not an output.
        """
        import onnx  # here, not at module top, so the stand-in runs without it
        from onnx import helper, numpy_helper

        m = onnx.load(path)
        g = m.graph
        init = {t.name: numpy_helper.to_array(t) for t in g.initializer}

        pfx = 'policy.mlp_extractor.policy_net.'
        hidden = _ordered_linears(init, pfx)
        if not hidden:
            raise ValueError('no %s* initializers in %s; names are %s'
                             % (pfx, path, ', '.join(init)))
        linears = hidden + [('policy.action_net.weight', 'policy.action_net.bias')]
        for key in ('mean', 'sigma', 'scale') + linears[-1]:
            if key not in init:
                raise ValueError('no %r initializer in %s' % (key, path))

        # Each Linear must be a plain Gemm, weight transposed (torch stores
        # (out, in)).  If the exporter ever emits MatMul+Add or folds the
        # transpose, this raises instead of drawing a transposed network.
        by_input = {}
        for node in g.node:
            for name in node.input:
                by_input.setdefault(name, []).append(node)
        activation = []
        for k, (w, _) in enumerate(linears):
            gemm = [n for n in by_input.get(w, []) if n.op_type == 'Gemm']
            if len(gemm) != 1:
                raise ValueError('%s is not consumed by exactly one Gemm' % w)
            attrs = {a.name: helper.get_attribute_value(a)
                     for a in gemm[0].attribute}
            if (attrs.get('transB') != 1 or attrs.get('transA', 0) != 0
                    or attrs.get('alpha', 1.0) != 1.0
                    or attrs.get('beta', 1.0) != 1.0):
                raise ValueError('unexpected Gemm attributes on %s: %s'
                                 % (w, attrs))
            if k == len(linears) - 1:
                break
            # The activation is the op that consumes this hidden Gemm's output.
            nxt = by_input.get(gemm[0].output[0], [])
            if len(nxt) != 1 or nxt[0].op_type not in ONNX_ACTIVATIONS:
                raise ValueError('after %s expected one activation op, got %s'
                                 % (w, [n.op_type for n in nxt]))
            activation.append(ONNX_ACTIVATIONS[nxt[0].op_type])

        layers = [(init[w].T, init[b]) for w, b in linears]
        net = cls(layers, activation=activation, grip_logistic=False)
        net.source = path
        net.obs_mean = init['mean'].astype(np.float32)
        net.obs_sigma = init['sigma'].astype(np.float32)
        net.act_scale = init['scale'].astype(np.float32)

        meta = {p.key: p.value for p in m.metadata_props}
        net.in_labels = json.loads(meta['obs_names']) if 'obs_names' in meta else None
        net.out_labels = json.loads(meta['act_names']) if 'act_names' in meta else None

        if len(net.obs_mean) != net.sizes[0] or len(net.act_scale) != net.sizes[-1]:
            raise ValueError('mean/scale widths %d/%d do not match the network %s'
                             % (len(net.obs_mean), len(net.act_scale), net.sizes))
        return net

    # ---- construction: trained policy, .pt + json (dormant) ------------
    @classmethod
    def from_policy(cls, pt_path, stats_path=None, log=print):
        """Load an SB3 state_dict plus the statistics saved alongside it.

        Reads ``mlp_extractor.policy_net.*`` (the hidden stack) and
        ``action_net`` (the output layer).  ``value_net`` is ignored on both
        counts -- BC does not meaningfully train it.

        The state_dict cannot carry the activation: an activation module has
        no parameters, so it leaves nothing behind but a gap in the layer
        numbering.  It comes from the stats file, or defaults to tanh with a
        warning, because that is what SB3's ActorCriticPolicy uses.
        """
        sd = load_state_dict(pt_path)

        hidden = _ordered_linears(sd, 'mlp_extractor.policy_net.')
        if not hidden:
            raise ValueError(
                'no mlp_extractor.policy_net.* Linear layers in %s; keys are %s'
                % (pt_path, ', '.join(sd)))
        if 'action_net.weight' not in sd:
            raise ValueError('no action_net.weight in %s' % pt_path)

        # torch Linear stores (out_features, in_features); the forward pass and
        # the drawing both want (in, out), so transpose on the way in.
        layers = [(sd[w].T, sd[b]) for w, b in hidden]
        layers.append((sd['action_net.weight'].T, sd['action_net.bias']))

        stats = {}
        if stats_path:
            with open(stats_path) as f:
                stats = json.load(f)

        activation = stats.get('activation')
        if activation is None:
            activation = 'tanh'
            log('no activation in stats -- assuming tanh (the SB3 default). '
                'If the policy was built with ReLU, the drawing is wrong.')

        net = cls(layers, activation=activation, grip_logistic=False)
        net.source = pt_path

        if 'obs_mean' in stats:
            net.obs_mean = np.asarray(stats['obs_mean'], np.float32)
            net.obs_sigma = np.asarray(stats['obs_sigma'], np.float32)
        if 'act_scale' in stats:
            net.act_scale = np.asarray(stats['act_scale'], np.float32)
        net.in_labels = stats.get('obs_names')
        net.out_labels = stats.get('act_names')

        # Shape agreement between the two files is the one mismatch that would
        # otherwise render silently: the wrong statistics give a plausible
        # picture of a network fed a transform it was never trained on.
        if net.obs_mean is not None and len(net.obs_mean) != net.sizes[0]:
            raise ValueError(
                'stats file has %d observation channels, the policy takes %d '
                '-- these two files are not a pair'
                % (len(net.obs_mean), net.sizes[0]))
        if net.act_scale is not None and len(net.act_scale) != net.sizes[-1]:
            raise ValueError(
                'stats file has %d action channels, the policy emits %d'
                % (len(net.act_scale), net.sizes[-1]))

        return net

    # ---- construction: stand-in ----------------------------------------
    @classmethod
    def random(cls, n_in=C.N_IN, hidden=C.HIDDEN, n_out=C.N_OUT, seed=C.SEED):
        rng = np.random.default_rng(seed)
        sizes = [n_in] + list(hidden) + [n_out]
        layers = []
        for a, b in zip(sizes[:-1], sizes[1:]):
            # He init: inputs are standardised, so this keeps ReLU units in a
            # useful range instead of all-dead or all-saturated.
            W = rng.normal(0.0, np.sqrt(2.0 / a), (a, b))
            layers.append((W, np.zeros(b)))
        return cls(layers, activation='relu', grip_logistic=True)

    def fit_readout(self, x, y, ridge=1.0):
        """Least-squares fit of the final layer only, hidden layers frozen.

        Stand-in path only.  y[:, :6] are the recorded forces/torques, y[:, 6]
        the held gripper flag, fitted in logit space so the logistic in
        forward() lands near 0 and 1.
        """
        acts = self.forward(x)
        h = acts[-2]
        H = np.concatenate([h, np.ones((len(h), 1), np.float32)], 1)
        t = np.asarray(y, np.float64).copy()
        eps = 1e-3
        p = np.clip(t[:, -1], eps, 1 - eps)
        t[:, -1] = np.log(p / (1 - p))
        A = H.T @ H + ridge * np.eye(H.shape[1])
        sol = np.linalg.solve(A, H.T @ t)
        self.layers[-1] = (sol[:-1].astype(np.float32),
                           sol[-1].astype(np.float32))
        return self

    # ---- display ordering ---------------------------------------------
    def order_hidden(self, acts):
        """Sort each hidden layer by activation variance over the bag.

        Hidden-unit ordering is arbitrary -- nothing makes unit 7 adjacent to
        unit 8 -- so any grouping of them is decorative.  Sorting by variance
        at least makes the grouping stable across frames and puts the busy
        units together, so the ribbons show a gradient instead of static.
        Applied as a permutation of the weight matrices, so the drawn network
        stays numerically identical to the one that was fitted.
        """
        for k in range(1, len(acts) - 1):
            order = np.argsort(-acts[k].var(0))
            W_in, b_in = self.layers[k - 1]
            self.layers[k - 1] = (W_in[:, order], b_in[order])
            W_out, b_out = self.layers[k]
            self.layers[k] = (W_out[order, :], b_out)
        return self


def _ordered_linears(sd, prefix):
    """(weight_key, bias_key) pairs under prefix, in Sequential index order.

    SB3 builds the stack as Linear, activation, Linear, activation..., so the
    indices run 0, 2, 4...  The gaps are the activations, which have no
    parameters and so appear nowhere in the state_dict.
    """
    idx = []
    for k in sd:
        if k.startswith(prefix) and k.endswith('.weight'):
            middle = k[len(prefix):-len('.weight')]
            if middle.isdigit():
                idx.append(int(middle))
    return [('%s%d.weight' % (prefix, i), '%s%d.bias' % (prefix, i))
            for i in sorted(idx)]
