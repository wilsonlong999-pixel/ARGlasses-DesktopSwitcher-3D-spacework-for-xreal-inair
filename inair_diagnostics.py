"""INAIR passive live diagnosis or offline replay; no desktop/network imports."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import sys
import time

from head_flick import FlickConfig, HeadFlickDetector
from inair_pose import PoseDecoder, configure_inair


class Session:
    def __init__(self, emit, verbose=True):
        self.emit, self.verbose = emit, verbose
        self.decoder = PoseDecoder()
        self.detector = HeadFlickDetector(configure_inair(FlickConfig()))
        self.label = "unlabeled"
        self.last_print = -1.0
        self.counts = Counter()

    def set_label(self, label):
        self.label = label
        self.decoder = PoseDecoder()
        self.detector.reset()
        self.last_print = -1.0
        self.emit("label", label=label)

    def feed(self, raw, host_time):
        self.counts["packets"] += 1
        try:
            pose = self.decoder.decode(raw)
        except ValueError as exc:
            self.detector.invalidate(str(exc))
            self.counts["invalid"] += 1
            self.emit("invalid", label=self.label, error=str(exc), hex=raw.hex())
            return
        if pose is None:
            self.emit("seed", label=self.label, hex=raw.hex())
            return
        before = (self.detector.state, self.detector.rejections)
        x, y, z = pose["rates"]
        event = self.detector.update(pose["t"], z, x, y)
        if self.detector.rejections != before[1]:
            self.counts["reject:" + self.label + ":" + self.detector.reason] += 1
        # Keep raw-sign labels for comparison with earlier diagnostic captures.
        candidate = {"left": "positive_z", "right": "negative_z"}.get(event)
        if candidate:
            self.counts["candidates"] += 1
            self.counts[self.label + ":" + candidate] += 1
        row = dict(label=self.label, host_time=host_time, hex=raw.hex(), **pose,
                   state=self.detector.state, reason=self.detector.reason,
                   excursion=self.detector.yaw-self.detector.rest,
                   filtered_rate=self.detector.rate, candidate=candidate,
                   checks=self.detector.diagnostics)
        self.emit("pose", **row)
        changed = before != (self.detector.state, self.detector.rejections)
        if self.verbose and (changed or candidate or pose["t"]-self.last_print >= 0.5):
            self.last_print = pose["t"]
            print(f"[INAIR] {self.label} heading={pose['heading']:+.1f}deg "
                  f"exc={row['excursion']:+.1f}deg ratesXYZ=({x:+.1f},{y:+.1f},{z:+.1f})dps "
                  f"state={row['state']} ({row['reason']}) candidate={candidate}", flush=True)
            if changed or candidate:
                print("[CHECK] " + row["checks"], flush=True)


def run_live():
    print("[MODE] 姿态诊断；不切换桌面，不生成日志文件。", flush=True)
    device = None
    def emit(kind, **data):
        pass  # Console diagnostics only; no file output.
    session = Session(emit)
    emit("config", config=asdict(session.detector.cfg),
         assumptions="xyzw@28, u32 microseconds@16, counter@13, body Z yaw; labeled +Z left")
    try:
        if sys.platform != "win32":
            raise RuntimeError("live HID diagnosis requires Windows")
        import hid
        import msvcrt
        devices = hid.enumerate(0x0483, 0x5750)
        emit("enumeration", devices=devices, hid_version=getattr(hid, "__version__", "unknown"))
        targets = [d for d in devices if "vid_0483&pid_5750&mi_02#" in
                   (d["path"].decode() if isinstance(d["path"], bytes) else d["path"]).lower()]
        if len(targets) != 1:
            raise RuntimeError(f"expected one MI_02 collection, found {len(targets)}")
        device = hid.device()
        device.open_path(targets[0]["path"])
        emit("opened", device=targets[0])
        print("按键标记：0静止 1向左返回 2向右返回 3点头 4歪头 5单向转头 6慢速往返 q退出", flush=True)
        labels = dict(zip("0123456", ("still", "left_return", "right_return", "pitch",
                                      "roll", "one_way", "slow_return")))
        deadline = time.monotonic() + 300
        last_data = time.monotonic()
        while time.monotonic() < deadline:
            if msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                if key == "q":
                    break
                if key in labels:
                    session.set_label(labels[key])
                    print("[LABEL] " + labels[key] + "；先停稳1秒再动作", flush=True)
            raw = device.read(65536, timeout_ms=100)
            now = time.monotonic()
            if raw:
                if now-last_data > 0.1:
                    session.decoder = PoseDecoder()
                    session.detector.invalidate("host input interruption")
                    emit("host_gap", seconds=now-last_data)
                session.feed(bytes(raw), now)
                last_data = now
            elif now-last_data > 3:
                raise RuntimeError("3 seconds without HID input; check connection")
    except (Exception, KeyboardInterrupt) as exc:
        emit("error", error_type=type(exc).__name__, error=str(exc))
        print(f"[STOP] {type(exc).__name__}: {exc}", flush=True)
    finally:
        if device is not None:
            device.close()
        emit("summary", counts=dict(session.counts))
        print(f"[SUMMARY] {dict(session.counts)}", flush=True)



def replay(path):
    session = Session(lambda *a, **kw: None, verbose=False)
    with open(path, encoding="utf-8-sig") as source:
        for line in source:
            row = json.loads(line)
            if row["kind"] == "phase_start":
                session.set_label(row["phase"])
            elif row["kind"] == "input":
                session.feed(bytes.fromhex(row["hex"]), row["host_time"])
    return dict(session.counts)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", help="Replay capture JSONL without HID or desktop access")
    args = parser.parse_args()
    if args.replay:
        print(json.dumps(replay(args.replay), ensure_ascii=False))
    else:
        run_live()
