"""The network whose activations get drawn.

Right now this is a stand-in.  The real one will come out of
`imitation.algorithms.bc.BC`, whose `.policy` is a Stable Baselines3
`ActorCriticPolicy`.  The swap is `Network.from_sb3(bc_trainer.policy)` --
see that method for the field names it expects.

The stand-in is NOT trained.  Hidden layers are random He-initialised
features; only the output layer is fitted, by ridge regression onto the
recorded /joy_wrench.  That makes the predicted row track the actual row so
the display shows something, but it is a random-features linear readout, not
behavioural cloning, and it should not be read as evidence about how a real
policy will behave.
"""
import numpy as np

from . import config as C


class Network:
    def __init__(self, layers, out_labels=None):
        """layers: list of (W, b) with W shape (fan_in, fan_out)."""
        self.layers = [(np.asarray(W, np.float32), np.asarray(b, np.float32))
                       for W, b in layers]
        self.sizes = [self.layers[0][0].shape[0]] + [W.shape[1] for W, _ in self.layers]
        self.out_labels = out_labels or C.OUT_LABELS

    # ---- forward -------------------------------------------------------
    def forward(self, x):
        """x: (N, n_in).  Returns list of activations, one per layer boundary.

        activations[0] is the input, activations[-1] the output.  Hidden
        layers are ReLU; the last layer is linear except for the final
        (gripper) unit, which is passed through a logistic so it reads 0..1.
        """
        acts = [np.asarray(x, np.float32)]
        h = acts[0]
        last = len(self.layers) - 1
        for k, (W, b) in enumerate(self.layers):
            z = h @ W + b
            if k < last:
                h = np.maximum(z, 0.0)
            else:
                h = z.copy()
                h[:, -1] = 1.0 / (1.0 + np.exp(-z[:, -1]))
            acts.append(h)
        return acts

    # ---- construction --------------------------------------------------
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
        return cls(layers)

    def fit_readout(self, x, y, ridge=1.0):
        """Least-squares fit of the final layer only, hidden layers frozen.

        y[:, :6] are the recorded forces/torques, y[:, 6] the held gripper
        flag.  The gripper column is fitted in logit space so the logistic in
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

    @classmethod
    def from_sb3(cls, policy):
        """Pull weights out of a Stable Baselines3 ActorCriticPolicy.

        `policy.mlp_extractor.policy_net` is the hidden stack (an
        nn.Sequential of Linear/activation pairs); `policy.action_net` is the
        output layer.  `policy.mlp_extractor.value_net` is a parallel MLP that
        BC does not meaningfully train -- deliberately ignored.

        Caveats to check when this is first used, not assumed:
          * SB3 defaults ActorCriticPolicy to tanh, not ReLU.  forward() here
            applies ReLU.  Either pass activation_fn=nn.ReLU when building the
            policy, or teach forward() about the activation.
          * For a Box action space, action_net emits the MEAN of a diagonal
            Gaussian.  A separate log_std parameter and a clip to the action
            bounds sit outside these weights.
          * SB3 has no mixed continuous/discrete action space, so the gripper
            has to live in the same Box and be thresholded.
        """
        import torch  # noqa: F401  (only needed on this path)
        layers = []
        for m in policy.mlp_extractor.policy_net:
            if hasattr(m, 'weight'):
                layers.append((m.weight.detach().cpu().numpy().T,
                               m.bias.detach().cpu().numpy()))
        a = policy.action_net
        layers.append((a.weight.detach().cpu().numpy().T,
                       a.bias.detach().cpu().numpy()))
        return cls(layers)

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
