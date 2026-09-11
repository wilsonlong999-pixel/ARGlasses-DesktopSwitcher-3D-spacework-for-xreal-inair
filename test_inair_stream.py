import unittest
from unittest.mock import patch
import math

from inair_stream import InairStream, open_device
from test_inair_pose import packet


class StreamTests(unittest.TestCase):
    def test_receive_uses_sensor_time_with_identical_host_arrivals(self):
        class Device:
            def __init__(self):
                self.index = 0
                self.closed = False

            def read(self, *args, **kwargs):
                self.index += 1
                if self.index == 3:
                    stream.stop.set()
                return packet(self.index*5000, self.index, self.index*.5)

            def close(self):
                self.closed = True

        device = Device()
        stream = InairStream(device)
        with patch('inair_stream.time.monotonic', return_value=100):
            stream._receive()
        self.assertIsNone(stream.error)
        a, b = stream.queue.get_nowait(), stream.queue.get_nowait()
        self.assertEqual(a[0], b[0])
        self.assertAlmostEqual(b[1][6]-a[1][6], .005)
        self.assertAlmostEqual(math.degrees(a[1][2]), 100, places=3)
        self.assertTrue(device.closed)

    def test_invalid_data_closes_handle_and_stops(self):
        from unittest.mock import Mock
        device = Mock()
        device.read.return_value = b'wrong'
        stream = InairStream(device)
        stream._receive()
        self.assertIsInstance(stream.error, ConnectionError)
        device.close.assert_called_once()
        with self.assertRaises(ConnectionError):
            next(stream.samples())

    def test_only_target_interface_is_opened(self):
        from unittest.mock import Mock
        hid = Mock()
        hid.enumerate.return_value = [dict(path=b'xxvid_0483&pid_5750&mi_00#x'),
                                      dict(path=b'xxvid_0483&pid_5750&mi_02#x')]
        with patch.dict('sys.modules', hid=hid):
            device = open_device()
        device.open_path.assert_called_once_with(b'xxvid_0483&pid_5750&mi_02#x')
        device.write.assert_not_called()


if __name__ == '__main__':
    unittest.main()
