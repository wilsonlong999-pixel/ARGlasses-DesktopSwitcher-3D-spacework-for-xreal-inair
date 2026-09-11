"""Receive IMU frames independently of console/DLL work.

The known protocol has no verified device timestamp here. Receive intervals are
distributed over valid samples in each batch (an approximation, not sensor time).
"""
import math
import queue
import socket
import struct
import threading
import time

HEADER = bytes.fromhex("283600000080")
FRAME_SIZE = 134
FLOAT_OFFSET = 34
NOMINAL_SAMPLE_PERIOD = 0.001  # XREAL stream is approximately 1 kHz


class FrameParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        samples = []
        while True:
            pos = self.buf.find(HEADER)
            if pos < 0:
                self.buf = self.buf[-(len(HEADER) - 1):]
                break
            if pos:
                del self.buf[:pos]
            if len(self.buf) < FRAME_SIZE:
                break
            # Require a plausible next frame boundary when available.
            if (len(self.buf) >= FRAME_SIZE + len(HEADER)
                    and self.buf[FRAME_SIZE:FRAME_SIZE + len(HEADER)] != HEADER):
                del self.buf[:len(HEADER)]
                continue
            vals = struct.unpack_from("<6f", self.buf, FLOAT_OFFSET)
            del self.buf[:FRAME_SIZE]
            if all(math.isfinite(v) for v in vals):
                samples.append(vals)
        return samples


def batch_times(previous, received, count):
    """Build a strictly increasing sample timeline from packet arrival times.

    Some Windows systems expose the arrival clock in coarse 15.6 ms steps even
    though XREAL delivers about one sample per millisecond. Several consecutive
    recv calls can therefore have the exact same ``received`` value. Feeding
    those duplicates to the gesture detector resets it on almost every sample.
    ``previous`` is the last emitted sample time, not the previous recv time.
    """
    if not count:
        return []
    if previous is None:
        return [received - (count - 1 - i) * NOMINAL_SAMPLE_PERIOD
                for i in range(count)]
    span = received - previous
    if span <= 0:
        # The host clock has not advanced. Continue from the last emitted
        # sample so every real IMU report still contributes to the gesture.
        return [previous + (i + 1) * NOMINAL_SAMPLE_PERIOD
                for i in range(count)]
    if span > 0.100:
        # Preserve a real receive interruption. The detector must see the gap
        # and discard a gesture that straddles it.
        return [received - (count - 1 - i) * NOMINAL_SAMPLE_PERIOD
                for i in range(count)]
    step = span / count
    return [previous + (i + 1) * step for i in range(count)]


class IMUStream:
    def __init__(self, sock):
        self.sock = sock
        self.stop = threading.Event()
        self.batches = queue.Queue(maxsize=128)
        self.error = None
        self.thread = threading.Thread(target=self._receive, daemon=True,
                                       name="xreal-imu-receive")

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *args):
        self.stop.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.thread.join(timeout=1.0)

    def _receive(self):
        parser = FrameParser()
        previous_sample = None
        try:
            while not self.stop.is_set():
                data = self.sock.recv(65536)
                received = time.monotonic()
                if not data:
                    raise ConnectionError("glasses closed the connection")
                vals = parser.feed(data)
                if not vals:
                    continue
                times = batch_times(previous_sample, received, len(vals))
                batch = list(zip(times, vals))
                previous_sample = times[-1]
                if self.batches.full():
                    # Consumer stalled: discard old data; timestamp gap resets
                    # the detector. Never switch desktops using queued history.
                    try:
                        self.batches.get_nowait()
                    except queue.Empty:
                        pass
                self.batches.put_nowait(batch)
        except (OSError, ConnectionError) as exc:
            self.error = exc
        finally:
            self.stop.set()

    def samples(self):
        while True:
            try:
                batch = self.batches.get(timeout=0.1)
            except queue.Empty:
                if self.stop.is_set():
                    raise self.error or ConnectionError("IMU stream stopped")
                continue
            yield from batch
