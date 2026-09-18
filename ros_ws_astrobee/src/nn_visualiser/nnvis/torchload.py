"""Read a PyTorch ``state_dict`` .pt file without importing torch.

A .pt written by ``torch.save`` is a zip archive: one pickle describing the
tensors, plus one raw little/big-endian buffer per storage.  A ``state_dict``
pickle is trivially flat -- string keys, tensor values -- and the only
non-builtin thing in its object graph is ``torch._utils._rebuild_tensor_v2``.
Stubbing that is enough to read the whole file.

This exists so the visualiser keeps numpy as its only hard dependency.  torch
is ~2 GB and is needed on the training machine anyway; requiring it here just
to multiply three matrices would invert the design.

Deliberately NOT supported:

  * whole-object saves (``torch.save(model, path)``).  Those pickle the
    policy class, the gym spaces, the optimizer and an lr_schedule callable,
    so reading them without torch and SB3 installed means stubbing a list of
    classes that changes with every library version.  Save a state_dict.
  * the pre-1.6 tar format.  Detected and reported rather than mis-parsed.
  * non-contiguous tensors.  Linear weights are always contiguous; anything
    else raises rather than being silently reshaped wrong.
"""
import io
import pickle
import zipfile
from collections import OrderedDict

import numpy as np

# torch storage class name -> (numpy dtype character code, bytes per element)
_DTYPES = {
    'FloatStorage': 'f4',
    'DoubleStorage': 'f8',
    'HalfStorage': 'f2',
    'BFloat16Storage': None,      # no numpy equivalent; handled explicitly
    'LongStorage': 'i8',
    'IntStorage': 'i4',
    'ShortStorage': 'i2',
    'CharStorage': 'i1',
    'ByteStorage': 'u1',
    'BoolStorage': 'b1',
}


class TorchLoadError(RuntimeError):
    pass


class _Stub:
    """Stands in for any torch.* name the pickle references."""

    def __init__(self, name):
        self.name = name

    def __call__(self, *a, **k):
        if self.name == '_rebuild_tensor_v2':
            # (storage, storage_offset, size, stride, requires_grad, hooks, ...)
            return {'storage': a[0], 'offset': a[1],
                    'size': tuple(a[2]), 'stride': tuple(a[3])}
        if self.name == 'OrderedDict':
            return OrderedDict(*a, **k)
        raise TorchLoadError(
            'unsupported torch object %r in the pickle -- this looks like a '
            'whole-object save rather than a state_dict' % self.name)


class _Unpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith('torch'):
            return _Stub(name)
        if module == 'collections' and name == 'OrderedDict':
            return OrderedDict
        if module == 'numpy' or module.startswith('numpy.'):
            return super().find_class(module, name)
        raise TorchLoadError(
            'pickle references %s.%s; only a plain state_dict is supported'
            % (module, name))

    def persistent_load(self, pid):
        # ('storage', storage_type, key, location, numel)
        if not (isinstance(pid, tuple) and pid and pid[0] == 'storage'):
            raise TorchLoadError('unexpected persistent id %r' % (pid,))
        kind = pid[1].name if isinstance(pid[1], _Stub) else str(pid[1])
        return {'kind': kind.split('.')[-1], 'key': str(pid[2]), 'numel': pid[4]}


def load_state_dict(path):
    """Return ``{name: numpy array}`` from a torch state_dict .pt file."""
    try:
        z = zipfile.ZipFile(path)
    except zipfile.BadZipFile:
        raise TorchLoadError(
            '%s is not a zip archive.  Files written by torch < 1.6, or with '
            '_use_new_zipfile_serialization=False, use a tar layout that is '
            'not supported here.' % path)

    with z:
        names = z.namelist()
        pkls = [n for n in names if n.endswith('/data.pkl') or n == 'data.pkl']
        if len(pkls) != 1:
            raise TorchLoadError('expected one data.pkl, found %d' % len(pkls))
        prefix = pkls[0][:-len('data.pkl')]

        big_endian = False
        if prefix + 'byteorder' in names:
            big_endian = z.read(prefix + 'byteorder').decode().strip() == 'big'

        raw = _Unpickler(io.BytesIO(z.read(pkls[0]))).load()
        if not isinstance(raw, dict):
            raise TorchLoadError(
                'top level of the pickle is %s, not a dict -- a whole-object '
                'save cannot be read here' % type(raw).__name__)

        out = OrderedDict()
        for key, t in raw.items():
            if not isinstance(t, dict) or 'storage' not in t:
                raise TorchLoadError('entry %r is not a tensor' % key)
            st = t['storage']
            code = _DTYPES.get(st['kind'])
            if code is None:
                raise TorchLoadError(
                    'unsupported storage type %s for %r' % (st['kind'], key))
            dt = np.dtype(('>' if big_endian else '<') + code)

            buf = z.read(prefix + 'data/' + st['key'])
            flat = np.frombuffer(buf, dtype=dt)

            size = t['size']
            n = int(np.prod(size)) if size else 1
            expected = _contiguous_stride(size)
            if t['stride'] != expected:
                raise TorchLoadError(
                    '%r is not contiguous (stride %s, expected %s)'
                    % (key, t['stride'], expected))

            off = t['offset']
            if off + n > flat.size:
                raise TorchLoadError('%r runs past the end of its storage' % key)
            out[key] = flat[off:off + n].reshape(size).astype(np.float32)

        return out


def _contiguous_stride(size):
    stride = []
    acc = 1
    for s in reversed(size):
        stride.append(acc)
        acc *= s
    return tuple(reversed(stride))
