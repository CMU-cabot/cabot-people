# Copyright (c) 2026  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""torch.load compatibility for the JetPack 6.2 image.

JetPack 6.0 builds torch 2.1.1, JetPack 6.2 builds torch 2.6.0. torch 2.6
flipped the default of ``torch.load`` from ``weights_only=False`` to
``weights_only=True``, which only unpickles plain tensors and a small allow
list of types.

Every checkpoint this package ships is a full pickle, not a bare state dict:

* ``sgan/models/*_with_model.pt`` stores ``checkpoint['args']`` next to the
  weights, and ``sgan/inference.py`` reads it back.
* ``group_rl`` (HiCrowd) SAC checkpoints are loaded by ``rl/rl_agent.py``.
* ``crowdattn/trained_models/.../41665.pt``.

Under torch 2.6 all of these raise ``_pickle.UnpicklingError`` at startup.
The checkpoints are shipped with the package, so restoring the old behaviour
is safe here; we do not load checkpoints from untrusted sources.

The two call sites inside this package pass ``weights_only=False``
explicitly. ``patch_torch_load()`` exists for the vendored third-party code
under ``crowdattn/`` and ``group_rl/``, which we do not modify.
"""

import functools

import torch

_patched = False


def patch_torch_load():
    """Make ``torch.load`` default to ``weights_only=False`` again.

    No-op on torch < 2.6, where that is already the default, and no-op if
    called more than once.
    """
    global _patched
    if _patched:
        return
    _patched = True

    try:
        major, minor = (int(part) for part in torch.__version__.split('.')[:2])
    except ValueError:
        return
    if (major, minor) < (2, 6):
        return

    original_load = torch.load

    @functools.wraps(original_load)
    def load(*args, **kwargs):
        kwargs.setdefault('weights_only', False)
        return original_load(*args, **kwargs)

    torch.load = load
