"""Experimental decoder inferred from the user's INAIR 64-byte capture.

Offsets refer to HIDAPI read() payload, without an unnumbered-report zero ID.
Quaternion xyzw, body axes, and microsecond clock are hypotheses supported by
the capture, not manufacturer protocol guarantees. Labeled trials map +Z to left.
"""
import math
import struct

INAIR_YAW_DOMINANCE = 1.5  # Require clear horizontal dominance on both strokes.


def configure_inair(config):
    """Hardware-specific motion tolerances; leave XREAL defaults unchanged."""
    from dataclasses import replace
    return replace(config, yaw_dominance=INAIR_YAW_DOMINANCE, still_rate=12.0,
                   still_grace=.060, still_spike_rate=35.0,
                   preparation_angle=min(12.0, config.min_angle))


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


class PoseDecoder:
    def __init__(self):
        self.previous = None
        self.elapsed = 0.0

    def decode(self, raw):
        try:
            return self._decode(raw)
        except ValueError:
            self.previous = None
            raise

    def _decode(self, raw):
        if len(raw) != 64 or raw[0] != 0x20:
            raise ValueError("unexpected packet length/header")
        quat = struct.unpack_from("<4f", raw, 28)
        norm = math.sqrt(sum(x*x for x in quat))
        if not math.isfinite(norm) or not 0.98 <= norm <= 1.02:
            raise ValueError("invalid quaternion norm")
        quat = tuple(x/norm for x in quat)
        tick = struct.unpack_from("<I", raw, 16)[0]
        counter = raw[13]
        previous = self.previous
        self.previous = (tick, counter, quat)
        x, y, z, w = quat
        heading = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
        if previous is None:
            return None
        old_tick, old_counter, old_q = previous
        dt = ((tick-old_tick) & 0xffffffff) * 1e-6
        if not 0 < dt <= 0.1 or (counter-old_counter) % 256 != 1:
            raise ValueError("timestamp/counter discontinuity")
        delta = multiply(tuple(-v for v in old_q[:3])+(old_q[3],), quat)
        if delta[3] < 0:  # q and -q encode the same orientation
            delta = tuple(-v for v in delta)
        n = math.sqrt(sum(v*v for v in delta[:3]))
        factor = math.degrees(2*math.atan2(n, delta[3])) / (n*dt) if n else 0
        rates = tuple(v*factor for v in delta[:3])
        if math.hypot(*rates) > 2000:
            raise ValueError("implausible angular velocity")
        self.elapsed += dt
        return dict(t=self.elapsed, dt=dt, tick=tick, counter=counter,
                    quaternion=quat, norm=norm, heading=heading, rates=rates)
