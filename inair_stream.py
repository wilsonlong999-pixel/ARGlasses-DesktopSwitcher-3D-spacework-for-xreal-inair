"""Passive HID receiver for validated INAIR firmware; independent of desktop I/O."""
import math
import queue
import threading
import time

from inair_pose import PoseDecoder


def open_device():
    try:
        import hid
    except ImportError as exc:
        raise SystemExit("[INAIR] 缺少 hidapi，请用当前 Python 执行 -m pip install hidapi") from exc
    devices = hid.enumerate(0x0483, 0x5750)
    targets = []
    for item in devices:
        path = item['path']
        name = path.decode() if isinstance(path, bytes) else path
        if 'vid_0483&pid_5750&mi_02#' in name.lower():
            targets.append(item)
    if len(targets) != 1:
        raise ConnectionError(f"需要一个 INAIR MI_02 接口，实际找到 {len(targets)} 个")
    device = hid.device()
    try:
        device.open_path(targets[0]['path'])
    except Exception:
        device.close()
        raise
    return device


class InairStream:
    def __init__(self, device):
        self.device = device
        self.stop = threading.Event()
        self.queue = queue.Queue(maxsize=128)
        self.error = None
        self.thread = threading.Thread(target=self._receive, daemon=True, name='inair-hid')

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *args):
        self.stop.set()
        self.thread.join(timeout=1)
        # HID reads have a 100 ms timeout; handle ownership stays in the reader.

    def _receive(self):
        decoder = PoseDecoder()
        last_received = time.monotonic()
        try:
            while not self.stop.is_set():
                data = self.device.read(65536, timeout_ms=100)
                now = time.monotonic()
                if not data:
                    if now-last_received > 3:
                        raise ConnectionError('3秒无HID输入')
                    continue
                if now-last_received > .1:
                    raise ConnectionError('HID接收中断超过100ms，丢弃残留手势')
                last_received = now
                try:
                    pose = decoder.decode(bytes(data))
                except ValueError as exc:
                    raise ConnectionError(f'INAIR数据异常：{exc}') from exc
                if pose is None:
                    continue
                # Arrival time is for freshness; device time is for integration.
                # These rates come from fused orientation, not raw gyro bias.
                values = tuple(math.radians(v) for v in pose['rates']) + (0., 0., 0., pose['t'])
                try:
                    self.queue.put_nowait((now, values))
                except queue.Full as exc:
                    raise ConnectionError('INAIR消费积压，丢弃历史动作并重新就绪') from exc
        except Exception as exc:
            self.error = ConnectionError(str(exc))
        finally:
            self.stop.set()
            self.device.close()

    def samples(self):
        while True:
            if self.error is not None:
                raise self.error
            try:
                sample = self.queue.get(timeout=.1)
            except queue.Empty:
                if self.stop.is_set():
                    raise ConnectionError('INAIR接收已停止')
                continue
            yield sample
