import contextlib
import io
import unittest
from unittest.mock import MagicMock, Mock, patch

from hardware_options import parse_hardware_args


class HardwareOptionsTests(unittest.TestCase):
    def test_no_arguments_require_manual_selection(self):
        self.assertIsNone(parse_hardware_args([]).hardware)

    def test_menu_selection_and_quit_dispatch(self):
        import main_udp_yaw_desktop_switcher as app
        for choice in ('inair', None):
            with patch.object(app, 'select_hardware', return_value=choice) as menu, \
                    patch.object(app, 'run_inair') as run, \
                    patch.object(app, 'connect_glasses') as tcp:
                app.main([])
            menu.assert_called_once_with()
            self.assertEqual(run.call_count, int(choice == 'inair'))
            tcp.assert_not_called()

    def test_menu_xreal_restores_verified_profile(self):
        import main_udp_yaw_desktop_switcher as app
        with patch.object(app, 'select_hardware', return_value='xreal'), \
                patch.object(app, 'run_xreal') as run, \
                patch.object(app, 'YAW_AXIS', 2), \
                patch.object(app, 'YAW_INVERT', False), \
                patch.object(app, 'YAW_DOMINANCE', 1.5), \
                patch.object(app, 'YAW_PROJECTION', None), \
                contextlib.redirect_stdout(io.StringIO()):
            app.main([])
            self.assertEqual(app.YAW_AXIS, app.XREAL_YAW_AXIS)
            self.assertEqual(app.YAW_INVERT, app.XREAL_YAW_INVERT)
            self.assertEqual(app.YAW_DOMINANCE, app.XREAL_YAW_DOMINANCE)
            self.assertEqual(app.YAW_PROJECTION, app.XREAL_YAW_PROJECTION)
        run.assert_called_once_with()

    def test_menu_accepts_1_2_and_q(self):
        from hardware_options import select_hardware
        for key, expected in [('1','inair'), ('2','xreal'), ('q',None)]:
            with patch('sys.stdin.isatty', return_value=False), \
                    patch('builtins.input', side_effect=['invalid',key]), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(select_hardware(), expected)

    def test_inair_run_does_not_create_logs(self):
        import main_udp_yaw_desktop_switcher as app
        with patch.object(app, 'load_vda', return_value=None), \
                patch('inair_stream.open_device'), patch('inair_stream.InairStream'), \
                patch.object(app, 'run_loop'), \
                patch('pathlib.Path.open', side_effect=AssertionError('file write')), \
                patch('builtins.open', side_effect=AssertionError('file write')):
            app.run_inair()

    def test_xreal_run_does_not_create_logs(self):
        import main_udp_yaw_desktop_switcher as app
        sock = MagicMock()
        stream = MagicMock()
        stream.samples.return_value = iter(())
        stream_type = MagicMock()
        stream_type.return_value.__enter__.return_value = stream
        with patch.object(app, 'load_vda', return_value=None), \
                patch.object(app, 'connect_glasses', return_value=sock), \
                patch.object(app, 'IMUStream', stream_type), \
                patch.object(app, 'run_loop') as run_loop, \
                patch('diagnostic_log.DiagnosticLog', side_effect=AssertionError('log created')), \
                contextlib.redirect_stdout(io.StringIO()):
            app.run_xreal()
        run_loop.assert_called_once_with(stream.samples.return_value, None)

    def test_requested_switches(self):
        for name, value in [("-Xreal", "xreal"), ("-Inair", "inair")]:
            self.assertEqual(parse_hardware_args([name]).hardware, value)

    def test_mutually_exclusive(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_hardware_args(["-Xreal", "-Inair"])

    def test_inair_dispatches_with_validated_mapping_not_xreal(self):
        import main_udp_yaw_desktop_switcher as app
        with patch.object(app, "run_inair") as inair, \
                patch.object(app, "connect_glasses") as connect, \
                patch.object(app, "YAW_AXIS", 1), patch.object(app, "YAW_INVERT", True), \
                patch.object(app, "YAW_DOMINANCE", .9), \
                patch.object(app, "YAW_PROJECTION", app.XREAL_YAW_PROJECTION):
            app.main(["-Inair"])
            self.assertEqual(app.YAW_AXIS, 2)
            self.assertFalse(app.YAW_INVERT)
            self.assertEqual(app.YAW_DOMINANCE, 1.5)
            self.assertIsNone(app.YAW_PROJECTION)
        inair.assert_called_once_with()
        connect.assert_not_called()

    def test_diagnostics_never_switch_desktops_or_connect(self):
        import main_udp_yaw_desktop_switcher as app
        with patch.object(app, "load_vda") as dll, \
                patch.object(app, "connect_glasses") as connect, \
                patch.object(app, "hardware_diagnostics", return_value="diagnostic") as diagnose, \
                contextlib.redirect_stdout(io.StringIO()):
            app.main(["-Inair", "--diagnose"])
        diagnose.assert_called_once_with("inair")
        dll.assert_not_called()
        connect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
