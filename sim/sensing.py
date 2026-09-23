"""Encoder quantization and the bounded unknown disturbance d(t)."""
import math

import numpy as np


class Encoder:
    def __init__(self, bits, offset=0.0):
        self.lsb = 2 * math.pi / 2 ** bits
        self.offset = offset

    def read(self, q):
        """Absolute angle, floor-quantized to one count (plus calibration offset)."""
        return math.floor((q + self.offset) / self.lsb) * self.lsb


def bounded_disturbance(rng, duration, amp, bw_hz, fs=1000.0):
    """Band-limited random torque, sampled at fs, scaled so max|d| == amp.

    White noise -> 2nd-order low-pass (bw_hz) -> normalised to the bound. The
    bound is met exactly (worst case of the stated |d| <= 0.05 N m), so every
    run carries the full permitted disturbance somewhere.
    """
    n = int(round(duration * fs)) + 2
    if amp <= 0:
        return np.zeros(n)
    x = rng.standard_normal(n + 2000)
    w = 2 * math.pi * bw_hz / fs
    a = math.exp(-w)
    y = np.zeros_like(x)
    z = 0.0
    for k in range(len(x)):          # two cascaded 1st-order sections
        z = a * z + (1 - a) * x[k]
        y[k] = a * y[k - 1] + (1 - a) * z if k else z
    y = y[2000:]                      # discard filter start-up
    return amp * y / np.max(np.abs(y))
