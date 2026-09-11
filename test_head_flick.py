"""Offline regression tests: no glasses, network, DLL calls or desktop switches."""
import math
import contextlib
import io
import struct
import unittest
from unittest.mock import patch

from head_flick import HeadFlickDetector, FlickConfig, AxisProbe
from imu_stream import FrameParser, HEADER, batch_times


class Motion:
    def __init__(self, dt=0.001):
        self.d = HeadFlickDetector()
        self.t = 0.0
        self.dt = dt
        self.events = []
        self.hold(0.35)

    def feed(self, duration, rate, other=0):
        for i in range(round(duration / self.dt)):
            self.t += self.dt
            r = rate(i * self.dt / duration) if callable(rate) else rate
            event = self.d.update(self.t, r, other)
            if event:
                self.events.append((self.t, event))

    def hold(self, duration):
        self.feed(duration, 0)

    def turn(self, angle, duration=0.22, other=0):
        self.feed(duration, lambda u: angle * math.pi / (2 * duration)
                  * math.sin(math.pi * u), other)


class GestureTests(unittest.TestCase):
    def pair(self, m, angle=25, outward=0.22, back=0.22):
        m.turn(angle, outward)
        self.assertEqual(m.events, [])
        m.turn(-angle, back)

    def test_fast_out_fast_back_both_directions(self):
        for a, direction in [(25, "left"), (-25, "right")]:
            m = Motion()
            self.pair(m, a)
            self.assertEqual([e[1] for e in m.events], [direction])

    def test_single_direction_never_switches_even_after_pause(self):
        for a in (25, -25):
            m = Motion()
            m.turn(a)
            m.hold(2)
            self.assertEqual(m.events, [])
            self.assertEqual(m.d.state, "ready")

    def test_slow_out_fast_back_rejected(self):
        m = Motion()
        self.pair(m, outward=1.1)
        m.hold(1)
        self.assertEqual(m.events, [])

    def test_fast_out_slow_back_rejected(self):
        m = Motion()
        self.pair(m, back=1.1)
        m.hold(1)
        self.assertEqual(m.events, [])

    def test_both_slow_rejected(self):
        m = Motion()
        self.pair(m, outward=1.1, back=1.1)
        self.assertEqual(m.events, [])

    def test_old_log_58dps_motion_cannot_pass_new_gate(self):
        m = Motion()
        m.feed(0.35, 58)
        m.feed(0.35, -58)
        self.assertEqual(m.events, [])

    def test_fast_return_without_enough_displacement_rejected(self):
        m = Motion()
        m.turn(25)
        m.turn(-3, 0.045)
        m.hold(1)
        self.assertEqual(m.events, [])

    def test_short_return_spike_cannot_trigger(self):
        m = Motion()
        m.turn(25)
        m.feed(0.001, -1800)
        m.hold(1)
        self.assertEqual(m.events, [])

    def test_delayed_return_cannot_complete_old_gesture(self):
        m = Motion()
        m.turn(25)
        m.hold(2)
        m.turn(-25)
        m.hold(1)
        self.assertEqual(m.events, [])

    def test_return_axis_is_checked_independently(self):
        m = Motion()
        m.turn(25)
        m.turn(-25, other=300)
        self.assertEqual(m.events, [])
        self.assertIn("return_axis", m.d.diagnostics)

    def test_outward_axis_interference_rejected(self):
        m = Motion()
        m.turn(25, other=300)
        m.turn(-25)
        self.assertEqual(m.events, [])

    def test_rebound_after_pair_does_not_switch_again(self):
        m = Motion()
        self.pair(m)
        m.turn(15, 0.12)
        m.turn(-15, 0.12)
        m.hold(0.4)
        self.assertEqual(len(m.events), 1)
        self.assertEqual(m.d.state, "ready")

    def test_xreal_rearms_when_yaw_stops_despite_other_axis_motion(self):
        m = Motion()
        m.d.cfg.recovery_yaw_only = True
        self.pair(m)
        # Reproduce the new XREAL log: yaw has stopped, while pitch/roll stays
        # above the ordinary all-axis 8 dps stillness threshold.
        m.feed(0.45, 0, other=20)
        self.assertEqual(m.d.state, "ready")
        self.assertEqual(len(m.events), 1)
        m.turn(-25)
        self.assertEqual(len(m.events), 1)
        m.turn(25)
        self.assertEqual([event for _, event in m.events], ["left", "right"])

    def test_initial_settling_still_requires_all_axes_to_stop(self):
        d = HeadFlickDetector(FlickConfig(recovery_yaw_only=True))
        t = 0.0
        for _ in range(500):
            t += .001
            d.update(t, 0, 20)
        self.assertEqual(d.state, "settling")

    def test_repeated_pairs_without_stop_consumed_once(self):
        m = Motion()
        for _ in range(4):
            m.turn(25)
            m.turn(-25)
        self.assertEqual(len(m.events), 1)

    def test_next_pair_can_start_from_different_stopped_angle(self):
        m = Motion()
        m.turn(35)
        m.turn(-18, 0.18)
        m.hold(0.45)
        self.assertEqual(len(m.events), 1)
        self.assertEqual(m.d.state, "ready")
        m.turn(-25)
        self.assertEqual(len(m.events), 1)
        m.turn(25)
        self.assertEqual([e[1] for e in m.events], ["left", "right"])

    def test_timeout_does_not_split_continuous_motion(self):
        m = Motion()
        self.pair(m)
        m.feed(2, 30)
        self.assertNotEqual(m.d.state, "recovering")
        self.assertEqual(len(m.events), 1)
        m.hold(0.4)
        self.assertEqual(m.d.state, "ready")

    def test_high_speed_pair_under_new_sensor_guard(self):
        m = Motion()
        m.feed(0.07, 850)
        self.assertEqual(m.events, [])
        m.feed(0.09, -850)
        self.assertEqual(len(m.events), 1)
        self.assertEqual(m.d.fault_count, 0)

    def test_gap_between_strokes_invalidates_pair(self):
        m = Motion()
        m.turn(25)
        m.t += 0.5
        m.d.update(m.t, 0)
        m.turn(-25)
        self.assertEqual(m.events, [])

    def test_faults_and_nonfinite_data(self):
        for r in (2500, float("nan"), float("inf")):
            m = Motion()
            m.feed(0.001, r)
            m.hold(0.4)
            self.assertTrue(math.isfinite(m.d.yaw))
            self.assertEqual(m.events, [])
            self.assertGreater(m.d.fault_count, 0)

    def test_sampling_rates_preserve_pair_result(self):
        for dt in (0.001, 0.002, 0.005, 0.01):
            m = Motion(dt)
            self.pair(m)
            self.assertEqual(len(m.events), 1)

    def test_reset_between_strokes_clears_old_pair(self):
        m = Motion()
        m.turn(25)
        m.d.reset()
        m.hold(0.4)
        m.turn(-25)
        self.assertEqual(m.events, [])

    def test_small_twitch_rejected(self):
        m = Motion()
        m.turn(4, 0.05)
        m.turn(-4, 0.05)
        self.assertEqual(m.events, [])


def frame(values=(1., 2., 3., 4., 5., 6.)):
    data = bytearray(134)
    data[:len(HEADER)] = HEADER
    struct.pack_into("<6f", data, 34, *values)
    return bytes(data)


class StreamTests(unittest.TestCase):
    def test_split_headers_and_frames(self):
        data = b"junk" + frame() * 3
        for chunk in (1, 5, 67, 134, 1024):
            parser = FrameParser()
            out = []
            for i in range(0, len(data), chunk):
                out.extend(parser.feed(data[i:i + chunk]))
            self.assertEqual(out, [(1., 2., 3., 4., 5., 6.)] * 3)

    def test_nan_placeholder(self):
        parser = FrameParser()
        self.assertEqual(len(parser.feed(frame((float("nan"),) * 6) + frame())), 1)

    def test_batch_interpolation(self):
        times = batch_times(1.0, 1.02, 20)
        self.assertAlmostEqual(times[0], 1.001)
        self.assertAlmostEqual(times[-1], 1.02)
        self.assertTrue(all(a < b for a, b in zip(times, times[1:])))

    def test_repeated_windows_clock_values_still_advance(self):
        previous = 1.0
        emitted = []
        for received in (1.0, 1.0, 1.0, 1.004):
            times = batch_times(previous, received, 1)
            emitted.extend(times)
            previous = times[-1]
        self.assertTrue(all(a < b for a, b in zip(emitted, emitted[1:])))
        self.assertEqual([round(t, 3) for t in emitted],
                         [1.001, 1.002, 1.003, 1.004])

    def test_full_gesture_survives_16ms_host_clock_quantization(self):
        detector = HeadFlickDetector()
        previous = None
        real_t = 0.0
        events = []
        for duration, angle in ((.35, 0), (.22, 25), (.08, 0),
                                (.22, -25), (.4, 0)):
            for i in range(round(duration * 1000)):
                real_t += .001
                # Reproduce the XPS log: about 16 IMU reports share one host
                # timestamp even though their sensor values keep changing.
                received = math.floor(real_t / .016) * .016
                times = batch_times(previous, received, 1)
                previous = times[-1]
                rate = angle * math.pi / (2 * duration) * math.sin(
                    math.pi * i * .001 / duration)
                event = detector.update(previous, rate)
                if event:
                    events.append(event)
        self.assertEqual(events, ["left"])
        self.assertEqual(detector.fault_count, 0)

    def test_long_gap_not_hidden(self):
        self.assertGreater(batch_times(1.0, 2.0, 20)[0] - 1.0, 0.1)


class AxisProbeTests(unittest.TestCase):
    def test_each_axis_can_be_identified(self):
        for axis in range(3):
            probe = AxisProbe(0)
            for i in range(1, 601):
                rates = [2.0] * 3
                rates[axis] = 60 * math.sin(i * 0.02)
                probe.update(i * 0.01, rates)
            self.assertEqual(probe.result()[0], axis)

    def test_ambiguous_or_stationary_motion_has_no_recommendation(self):
        for rates in [(1, 1, 1), (40, 40, 2)]:
            probe = AxisProbe(0)
            for i in range(1, 601):
                probe.update(i * 0.01, rates)
            self.assertIsNone(probe.result()[0])

    def test_missing_samples_have_no_recommendation(self):
        probe = AxisProbe(0)
        for i in range(1, 7):
            probe.update(i, (100, 2, 2))
        self.assertIsNone(probe.result()[0])


class MainLoopTests(unittest.TestCase):
    def run_simulation(self, desktop=2, switch_works=True, turn_axis=0, commands=(), pose_source=False, manual_change=False):
        import main_udp_yaw_desktop_switcher as app
        clock = [0.0]
        actual = [desktop]
        requests = []
        keys = list(commands)

        def read_key():
            if keys and clock[0] >= keys[0][0]:
                return keys.pop(0)[1]
            return None

        def samples():
            for duration, angle in [(0.4, 0), (0.22, 25), (0.08, 0),
                                    (0.22, -25), (6.4 if commands else 0.9, 0)]:
                for i in range(round(duration * 1000)):
                    clock[0] += 0.001
                    if manual_change and .1995 < clock[0] < .2005:
                        actual[0] = 3
                    rate = angle * math.pi / (2 * duration) * math.sin(math.pi * i * 0.001 / duration)
                    values = [0.0] * 6
                    values[turn_axis] = math.radians(rate)
                    values[5] = 9.81
                    if pose_source:
                        # Device clock and Windows clock intentionally have different epochs.
                        values.append(clock[0] + 1000)
                    yield clock[0], tuple(values)

        def goto(_, target):
            requests.append(target)
            if switch_works:
                actual[0] = target

        output = io.StringIO()
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(app, "calibrate_bias", return_value=(0, 0, 0)))
            stack.enter_context(patch.object(app, "CENTER_ON_FIRST_PACKET", False))
            stack.enter_context(patch.object(app, "DEBUG_PRINTS", False))
            stack.enter_context(patch.object(app, "YAW_INVERT", False))
            stack.enter_context(patch.object(app, "YAW_AXIS", 0))
            stack.enter_context(patch.object(app, "YAW_PROJECTION", None))
            stack.enter_context(patch.object(app, "desktop_count", return_value=3))
            stack.enter_context(patch.object(app, "check_keys", side_effect=read_key))
            stack.enter_context(patch.object(app.time, "monotonic", side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(app, "get_current_desktop_1based", side_effect=lambda _: actual[0]))
            stack.enter_context(patch.object(app, "goto_desktop_1based", side_effect=goto))
            stack.enter_context(contextlib.redirect_stdout(output))
            app.run_loop(samples(), object(), pose_source=pose_source)
        return requests, output.getvalue()

    def test_axis_probe_suppresses_switch_and_reports_result(self):
        requests, output = self.run_simulation(commands=[(0.04, "axis_probe")])
        self.assertEqual(requests, [])
        self.assertIn("RMS X/Y/Z", output)
        self.assertIn("建议=X", output)
        self.assertIn("诊断结束", output)

    def test_repeated_a_does_not_restart_or_reenter_probe(self):
        commands = [(0.04 + i * 0.03, "axis_probe") for i in range(230)]
        requests, output = self.run_simulation(commands=commands)
        self.assertEqual(requests, [])
        self.assertEqual(output.count("[AXIS] 接下来6秒"), 1)
        self.assertEqual(output.count("诊断结束"), 1)

    def test_manual_axis_selection_routes_gyro_correctly(self):
        requests, _ = self.run_simulation(turn_axis=1)
        self.assertEqual(requests, [])
        requests, output = self.run_simulation(turn_axis=1, commands=[(0.04, "axis_2")])
        self.assertEqual(requests, [1])
        self.assertIn("AXIS=Y", output)

    def test_actual_desktop_is_used(self):
        requests, output = self.run_simulation(desktop=3)
        self.assertEqual(requests, [2])
        self.assertIn("已确认桌面 2", output)

    def test_pose_source_uses_device_clock_and_confirms_single_switch(self):
        requests, output = self.run_simulation(desktop=3, pose_source=True)
        self.assertEqual(requests, [2])
        self.assertIn("已确认桌面 2", output)
        self.assertNotIn("duration=-", output)

    def test_manual_desktop_change_is_logged_and_used(self):
        requests, output = self.run_simulation(manual_change=True, pose_source=True)
        self.assertEqual(requests, [2])
        self.assertIn("实际桌面变化 2->3", output)

    def test_failed_switch_is_not_reported_successful_or_retried(self):
        requests, output = self.run_simulation(switch_works=False)
        self.assertEqual(requests, [1])
        self.assertIn("切换未确认", output)
        self.assertNotIn("已确认桌面", output)

    def test_compact_output_tracks_gesture_states(self):
        requests, output = self.run_simulation()
        self.assertEqual(requests, [1])
        self.assertIn("[状态] 已就绪", output)
        self.assertIn("[跟踪] 检测到外甩", output)
        self.assertIn("[跟踪] 外甩已达标", output)

    def test_xreal_calibration_waits_past_three_moving_windows(self):
        import main_udp_yaw_desktop_switcher as app
        moving = [(0.10 if i % 2 else -0.10, 0.08, -0.06) for i in range(1500)]
        quiet = [(0.01, -0.02, 0.005)] * app.CALIB_SAMPLES
        source = iter((i * .001, values + (0, 0, 0))
                      for i, values in enumerate(moving + quiet))
        with contextlib.redirect_stdout(io.StringIO()):
            bias = app.calibrate_bias(source)
        self.assertEqual(bias, (0.01, -0.02, 0.005))

    def test_xreal_detail_log_contains_axes_states_and_switch(self):
        class Log:
            def __init__(self):
                self.lines = []
            def write(self, line):
                self.lines.append(line)
        log = Log()
        # Exercise logging separately because run_simulation intentionally uses
        # the ordinary compact-output call signature.
        import main_udp_yaw_desktop_switcher as app
        clock = [0.0]
        def source():
            for duration, angle in ((.4,0),(.22,25),(.08,0),(.22,-25),(.5,0)):
                for i in range(round(duration*1000)):
                    clock[0] += .001
                    rate = angle*math.pi/(2*duration)*math.sin(math.pi*i*.001/duration)
                    yield clock[0], (math.radians(rate),0,0,0,0,0)
        with patch.object(app, 'calibrate_bias', return_value=(0,0,0)), \
                patch.object(app, 'YAW_AXIS', 0), patch.object(app, 'YAW_INVERT', False), \
                patch.object(app, 'YAW_PROJECTION', None), \
                patch.object(app, 'CENTER_ON_FIRST_PACKET', False), \
                patch.object(app, 'check_keys', return_value=None), \
                patch.object(app.time, 'monotonic', side_effect=lambda:clock[0]), \
                contextlib.redirect_stdout(io.StringIO()):
            app.run_loop(source(), None, detail_log=log)
        joined = '\n'.join(log.lines)
        self.assertIn('[AXIS_WINDOW]', joined)
        self.assertIn('raw_rad=', joined)
        self.assertIn('state=returning', joined)
        self.assertIn('[FLICK]', joined)

    def test_xreal_yz_projection_keeps_turn_and_removes_mounting_coupling(self):
        import main_udp_yaw_desktop_switcher as app
        selected, others, label = app.select_motion_components(
            (60.0, -180.0, 200.0), 1, True, "yz_difference")
        self.assertEqual(label, "Y-Z")
        self.assertGreaterEqual(selected, 190)
        self.assertLess(math.hypot(*others), 65)
        self.assertGreater(abs(selected) / math.hypot(*others), 3)

    def test_boundary_consumes_return(self):
        requests, output = self.run_simulation(desktop=1)
        self.assertEqual(requests, [])
        self.assertIn("已到边界", output)

    def test_q_does_not_reconnect(self):
        import main_udp_yaw_desktop_switcher as app
        with patch.object(app, "ENABLE_DESKTOP_SWITCH", False), \
                patch.object(app, "run_xreal") as run:
            app.main(['-Xreal'])
        run.assert_called_once()


if __name__ == "__main__":
    unittest.main()
