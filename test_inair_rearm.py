import unittest

from head_flick import HeadFlickDetector, FlickConfig
from inair_pose import configure_inair
from test_head_flick import Motion


class InairRearmTests(unittest.TestCase):
    def motion(self):
        m = Motion(dt=.005)
        m.d = HeadFlickDetector(configure_inair(FlickConfig()))
        m.hold(.4)
        return m

    def test_brief_other_axis_noise_does_not_restart_quiet_timer(self):
        m = self.motion()
        m.d.reset()
        for _ in range(3):
            m.feed(.08, 0, 10)
            m.feed(.005, 0, 24)
        m.feed(.015, 0, 10)
        self.assertEqual(m.d.state, 'ready')

    def test_persistent_motion_cannot_accumulate_quiet_time(self):
        for yaw, other in ((0,16),(20,0),(0,60),(8,0)):
            m = self.motion()
            m.d.reset()
            m.feed(2, yaw, other)
            self.assertEqual(m.d.state, 'settling')
            self.assertEqual(m.events, [])

    def test_recovery_accepts_noisy_stopped_pose_then_next_pair(self):
        m = self.motion()
        m.turn(25); m.turn(-25)
        self.assertEqual(len(m.events), 1)
        for _ in range(6):
            m.feed(.075, 0, 10)
            m.feed(.005, 0, 24)
        m.feed(.05,0,10)
        self.assertEqual(m.d.state, 'ready')
        m.turn(-25);m.turn(25)
        self.assertEqual([e[1] for e in m.events], ['left','right'])

    def test_small_slow_opposite_preparation_allows_right_pair(self):
        m = self.motion()
        m.feed(.20,40)  # 8 degrees, never a fast stroke
        m.turn(-30)
        self.assertEqual(m.events, [])
        m.turn(30)
        self.assertEqual([e[1] for e in m.events], ['right'])

    def test_large_slow_prelude_is_not_reclassified_as_preparation(self):
        m = self.motion()
        m.feed(.36,40)
        m.turn(-30);m.turn(30)
        self.assertEqual(m.events, [])

    def test_fast_opposite_prelude_is_not_ignored(self):
        m = self.motion()
        m.feed(.07,120)  # small but deliberately fast opposite stroke
        m.turn(-30); m.turn(30)
        self.assertEqual(m.events, [])

    def test_no_repeat_without_stop(self):
        m = self.motion()
        for _ in range(5):
            m.turn(25);m.turn(-25)
        self.assertEqual(len(m.events),1)

    def test_single_direction_and_slow_return_still_rejected(self):
        for back in (False, True):
            m = self.motion()
            m.turn(-25)
            if back:m.turn(25, 1.1)
            m.hold(1)
            self.assertEqual(m.events, [])

    def test_noise_tolerance_does_not_change_xreal_defaults(self):
        m = Motion(dt=.005)
        m.d.reset()
        m.feed(.5,0,10)
        self.assertEqual(m.d.state,'settling')


if __name__ == '__main__':
    unittest.main()
