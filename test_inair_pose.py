import contextlib
import io
import math
import struct
import unittest
from unittest.mock import patch

from inair_pose import PoseDecoder
from hardware_options import parse_hardware_args


def packet(tick, counter, angle=0, sign=1, axis=2):
    raw = bytearray(64)
    raw[0], raw[13] = 32, counter
    struct.pack_into("<I", raw, 16, tick)
    q = [0, 0, 0, math.cos(math.radians(angle)/2)]
    q[axis] = math.sin(math.radians(angle)/2)
    struct.pack_into("<4f", raw, 28, *(x*sign for x in q))
    return bytes(raw)


class PoseTests(unittest.TestCase):
    def test_coupled_outward_motion_rejected_by_inair_profile(self):
        from head_flick import HeadFlickDetector, FlickConfig
        from inair_pose import INAIR_YAW_DOMINANCE
        def run(ratio):
            f = HeadFlickDetector(FlickConfig(yaw_dominance=ratio))
            t = 0
            events = []
            for duration, yaw, other in ((.4,0,0),(.22,120,125),(.08,0,0),(.22,-120,0),(.8,0,0)):
                for _ in range(round(duration/.005)):
                    t += .005
                    event = f.update(t, yaw, other, 0)
                    if event:
                        events.append(event)
            return events
        self.assertEqual(run(.9), ['left'])
        self.assertEqual(run(INAIR_YAW_DOMINANCE), [])

    def test_rate_axes_and_clock_counter_wrap(self):
        for axis in range(3):
            d = PoseDecoder()
            self.assertIsNone(d.decode(packet(0xffffffff-2499, 255, axis=axis)))
            p = d.decode(packet(2500, 0, 0.5, axis=axis))
            self.assertAlmostEqual(p['dt'], .005)
            self.assertAlmostEqual(p['rates'][axis], 100, places=3)
            self.assertAlmostEqual(sum(abs(x) for x in p['rates']), 100, places=3)

    def test_quaternion_sign_flip_is_not_motion(self):
        d = PoseDecoder()
        d.decode(packet(10000, 0, 170))
        p = d.decode(packet(15000, 1, 170, sign=-1))
        self.assertLess(math.hypot(*p['rates']), 1e-6)

    def test_heading_wrap_does_not_spike(self):
        d = PoseDecoder()
        d.decode(packet(10000, 0, 179.8))
        p = d.decode(packet(15000, 1, -179.7))
        self.assertAlmostEqual(p['rates'][2], 100, places=2)

    def test_bad_packets_reset_history(self):
        invalid_norm = bytearray(packet(15000, 1))
        struct.pack_into('<f', invalid_norm, 28, float('nan'))
        for bad in (b'bad', bytes(invalid_norm), packet(10000, 1),
                    packet(15000, 2), packet(200000, 1), packet(15000, 1, 90)):
            d = PoseDecoder()
            d.decode(packet(10000, 0))
            with self.assertRaises(ValueError):
                d.decode(bad)
            self.assertIsNone(d.decode(packet(205000, 3)))

    def test_debug_dispatch_does_not_touch_desktops_or_xreal(self):
        import main_udp_yaw_desktop_switcher as app
        with patch('inair_diagnostics.run_live') as live, \
                patch.object(app, 'load_vda') as dll, \
                patch.object(app, 'connect_glasses') as tcp:
            app.main(['-Inair', '--imu-debug'])
        live.assert_called_once_with()
        dll.assert_not_called()
        tcp.assert_not_called()

    def test_debug_incompatible_flags(self):
        for flags in (['--imu-debug'], ['-Inair', '--imu-debug', '--diagnose']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_hardware_args(flags)


if __name__ == '__main__':
    unittest.main()
