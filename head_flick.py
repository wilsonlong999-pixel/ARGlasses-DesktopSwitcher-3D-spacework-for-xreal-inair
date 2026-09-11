"""Pure, replayable head-flick detector. Angles in degrees, time in seconds.

Positive yaw means left; the caller applies the hardware axis/sign mapping.
Switch only after fast outward AND fast return strokes; rearm at any stopped pose.
"""
from dataclasses import dataclass
from collections import deque
import math


class AxisProbe:
    """Measure labeled left/right-only motion; never silently change the axis."""
    def __init__(self, start, duration=6.0):
        self.start = self.last_t = start
        self.duration = duration
        self.seconds = 0.0
        self.energy = [0.0] * 3
        self.peaks = [0.0] * 3

    def update(self, t, rates):
        dt = t - self.last_t
        if dt <= 0:
            return
        self.last_t = t
        if dt > 0.1 or not all(math.isfinite(r) for r in rates):
            return
        self.seconds += dt
        for i, r in enumerate(rates):
            self.energy[i] += r * r * dt
            self.peaks[i] = max(self.peaks[i], abs(r))

    def result(self):
        rms = [math.sqrt(e / max(self.seconds, 1e-6)) for e in self.energy]
        ranked = sorted(range(3), key=lambda i: rms[i], reverse=True)
        best, second = ranked[:2]
        clear = (self.seconds >= self.duration * 0.7 and rms[best] >= 10
                 and rms[best] >= rms[second] * 1.5)
        return (best if clear else None), rms, self.peaks[:]


@dataclass
class FlickConfig:
    min_angle: float = 12.0
    start_rate: float = 18.0
    peak_rate: float = 80.0
    return_rate: float = 80.0
    return_angle: float = 6.0
    return_fraction: float = 0.35
    return_window: float = 0.450
    gesture_window: float = 1.200
    min_duration: float = 0.060
    fast_duration: float = 0.035
    window: float = 0.650
    max_angle: float = 75.0
    filter_tau: float = 0.018
    still_rate: float = 8.0
    still_span: float = 1.5
    still_hold: float = 0.220
    cooldown: float = 0.350
    recovery_timeout: float = 1.000
    yaw_dominance: float = 0.90
    max_rate: float = 2000.0  # engineering outlier guard, not a sensor range claim
    max_gap: float = 0.100
    still_grace: float = 0.0  # Optional noisy-time allowance in a bounded window.
    still_spike_rate: float = 35.0
    preparation_angle: float = 0.0
    # Some devices report harmless pitch/roll motion after a desktop switch.
    # The consumed yaw gesture may rearm from yaw stillness while the next
    # gesture still has to pass the ordinary three-axis dominance checks.
    recovery_yaw_only: bool = False


class HeadFlickDetector:
    def __init__(self, config=None):
        self.cfg = config or FlickConfig()
        self.reset()

    def reset(self):
        self.state = "settling"
        self.reason = "等待静止"
        self.last_t = None
        self.yaw = self.rest = self.rate = 0.0
        self.still_since = None
        self.still_min = self.still_max = 0.0
        self.filtered = [0.0] * 3
        self.lock_until = 0.0
        self.origin = self.peak = 0.0
        self.home = 0.0
        self.diagnostics = "尚未追踪"
        self.rejections = 0
        self.recovery_deadline = 0.0
        self.fault_count = 0
        self.last_fault = "none"
        self.still_window = deque()
        self.still_diagnostics = "尚未采样"

    def _settled(self, t, total_rate, dt=0.0):
        c = self.cfg
        if c.still_grace > 0:
            if total_rate > c.still_spike_rate:
                self.still_window.clear()
                self.still_since = None
                self.still_diagnostics = f"blocked=motion total={total_rate:.1f}/{c.still_spike_rate:.1f}dps quiet=0ms"
                return False
            quiet = total_rate <= c.still_rate
            self.still_window.append((t, dt, quiet, self.yaw))
            cutoff = t - c.still_hold - c.still_grace
            while self.still_window and self.still_window[0][0] <= cutoff:
                self.still_window.popleft()
            quiet_time = sum(max(0, end-max(end-span, cutoff))
                             for end, span, ok, _ in self.still_window if ok)
            span = max(v[3] for v in self.still_window)-min(v[3] for v in self.still_window)
            blocked = ('rate' if not quiet else 'yaw_span' if span > c.still_span
                       else 'quiet_time' if quiet_time+1e-9 < c.still_hold else 'none')
            self.still_diagnostics = (f"blocked={blocked} total={total_rate:.1f}/{c.still_rate:.1f}dps "
                                      f"quiet={quiet_time*1000:.0f}/{c.still_hold*1000:.0f}ms "
                                      f"window={(c.still_hold+c.still_grace)*1000:.0f}ms "
                                      f"yaw_span={span:.2f}/{c.still_span:.2f}deg")
            return blocked == 'none'
        if total_rate > self.cfg.still_rate:
            self.still_since = None
            self.still_diagnostics = f"blocked=rate total={total_rate:.1f}/{c.still_rate:.1f}dps"
            return False
        if self.still_since is None:
            self.still_since = t
            self.still_min = self.still_max = self.yaw
        self.still_min = min(self.still_min, self.yaw)
        self.still_max = max(self.still_max, self.yaw)
        if self.still_max - self.still_min > self.cfg.still_span:
            self.still_since = t
            self.still_min = self.still_max = self.yaw
        self.still_diagnostics = f"quiet={(t-self.still_since)*1000:.0f}/{c.still_hold*1000:.0f}ms"
        return t - self.still_since >= self.cfg.still_hold

    def _cancel(self, reason, state="settling"):
        # Do not recycle a rejected half-gesture as the next outward stroke.
        # A stable pause anywhere starts a new, independent gesture.
        self.state, self.reason = state, reason
        self.still_since = None
        self.still_window.clear()
        self.rejections += 1

    def invalidate(self, reason="采样中断或异常角速度"):
        # Discard missing time, not the consumed-gesture lock. Otherwise a
        # console/DLL stall while looking sideways can rearm the return stroke.
        self.last_t = None
        self.filtered = [0.0] * 3
        self.rate = 0.0
        self.still_since = None
        self.still_window.clear()
        if self.state != "recovering":
            self.state = "settling"
        self.reason = reason
        self.fault_count += 1
        self.last_fault = reason

    def update(self, t, yaw_rate, other_rate_1=0.0, other_rate_2=0.0):
        """Return 'left'/'right' once per gesture, or None. No I/O or clocks."""
        c = self.cfg
        raw = (yaw_rate, other_rate_1, other_rate_2)
        if not math.isfinite(t) or not all(math.isfinite(x) for x in raw):
            self.invalidate("非有限数据")
            return None
        if self.last_t is None:
            self.last_t = t
            return None
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0 or dt > c.max_gap or math.hypot(*raw) > c.max_rate:
            why = (f"时间非递增 dt={dt*1000:.3f}ms" if dt <= 0 else
                   f"采样间隔 dt={dt*1000:.1f}ms>{c.max_gap*1000:.0f}ms" if dt > c.max_gap else
                   f"三轴速度={math.hypot(*raw):.1f}dps>{c.max_rate:.0f}dps")
            self.invalidate(why)
            self.last_t = t
            return None
        alpha = -math.expm1(-dt / c.filter_tau)
        self.filtered = [f + alpha * (r - f) for f, r in zip(self.filtered, raw)]
        previous_rate = self.rate
        self.rate = self.filtered[0]
        # Suppress sub-degree bias drift only when all raw axes are quiet.
        if not (abs(self.rate) < 0.5 and math.hypot(*raw) < 1.0):
            self.yaw += (previous_rate + self.rate) * dt / 2
        total = math.hypot(*self.filtered)
        settle_rate = max(total, math.hypot(*raw))
        if self.state == "recovering" and c.recovery_yaw_only:
            settle_rate = max(abs(self.rate), abs(yaw_rate))
        settled = self._settled(t, settle_rate, dt)

        if self.state == "recovering":
            self.diagnostics = (f"rearm_at=current_pose "
                                f"remaining={max(0,self.recovery_deadline-t)*1000:.0f}ms "
                                f"faults={self.fault_count} {self.still_diagnostics}")
            self.reason = "等待当前朝向停稳，不要求回原点"
            if settled and t >= self.lock_until:
                self.rest = self.home = self.origin = self.yaw
                self.state, self.reason = "ready", "已停稳，当前朝向为新起点"
            elif t >= self.recovery_deadline:
                # Bound the post-switch state, but never create repeated
                # switches from one uninterrupted movement on a timer.
                self.state, self.reason = "settling", "恢复计时结束；停止即可开始下一次，无位置要求"
            return None

        if self.state == "settling":
            self.diagnostics = f"rearm_at=current_pose {self.still_diagnostics}"
            if settled and t >= self.lock_until:
                self.rest = self.home = self.origin = self.yaw
                self.state, self.reason = "ready", "就绪"
            return None

        if self.state == "returning":
            # Direction is frozen from the qualified outward stroke.
            projected = self.out_sign * (self.yaw - self.origin)
            self.peak = max(self.peak, projected)
            if self.return_t0 is None and self.out_sign * self.rate <= -c.start_rate:
                self.return_t0 = t
            back = self.peak - projected
            if self.return_t0 is not None:
                speed = -self.out_sign * self.rate
                self.return_peak_rate = max(self.return_peak_rate, speed)
                self.return_fast += dt if speed >= c.return_rate else 0.0
                self.return_travel += abs(self.rate) * dt
                self.return_other += math.hypot(*self.filtered[1:]) * dt
            need_back = max(c.return_angle, self.peak * c.return_fraction)
            checks = {
                "return_started": self.return_t0 is not None,
                "return_angle": back >= need_back,
                "return_fast": self.return_fast >= c.fast_duration,
                "return_direction": -self.out_sign * self.rate >= c.start_rate,
                "return_axis": self.return_travel >= c.yaw_dominance * self.return_other,
            }
            elapsed = t - self.return_t0 if self.return_t0 is not None else 0.0
            self.diagnostics = (
                f"phase=return out_dir={'left' if self.out_sign==1 else 'right'} "
                f"out_peak_rate={self.out_peak_rate:.1f}dps out_fast={self.out_fast*1000:.0f}ms "
                f"out_angle={self.peak:.1f}deg back={back:.1f}/{need_back:.1f}deg "
                f"back_peak_rate={self.return_peak_rate:.1f}dps "
                f"back_fast={self.return_fast*1000:.0f}/{c.fast_duration*1000:.0f}ms "
                f"back_axis={self.return_travel/max(self.return_other,1e-6):.2f}/{c.yaw_dominance:.2f} "
                f"back_elapsed={elapsed*1000:.0f}/{c.return_window*1000:.0f}ms "
                f"blocked={','.join(k for k,v in checks.items() if not v) or 'none'}")
            if self.peak > c.max_angle:
                self._cancel("外甩角度过大")
            elif t - self.t0 > c.gesture_window or elapsed > c.return_window:
                self._cancel("回向/完整手势超时")
            elif settled:
                self._cancel("已停稳但未完成快速回向")
            elif all(checks.values()):
                self.state, self.reason = "recovering", "快速外甩+快速回向完成，等待停稳"
                self.lock_until = t + c.cooldown
                self.recovery_deadline = t + c.recovery_timeout
                self.still_since = None
                self.still_window.clear()
                return "left" if self.out_sign == 1 else "right"
            return None

        if self.state in ("ready", "watching"):
            if settled:
                self.rest = self.yaw
                self.state, self.reason = "ready", "就绪"
            if abs(self.rate) < c.start_rate:
                return None
            self.origin = self.yaw - previous_rate * dt
            # Preserve the start of an unconsumed motion across time windows,
            # so a later valid outward stroke still consumes its full return.
            if self.state == "ready":
                self.home = self.origin
            self.t0 = t
            self.left_peak = self.right_peak = 0.0
            self.left_fast = self.right_fast = 0.0
            self.yaw_travel = self.other_travel = 0.0
            self.left_rate_peak = self.right_rate_peak = 0.0
            self.state, self.reason = "tracking", "追踪外甩"

        exc = self.yaw - self.origin
        self.left_peak = max(self.left_peak, exc)
        self.right_peak = max(self.right_peak, -exc)
        self.peak = max(self.left_peak, self.right_peak)
        self.yaw_travel += abs(self.rate) * dt
        self.other_travel += math.hypot(*self.filtered[1:]) * dt
        self.left_fast += dt if self.rate >= c.peak_rate else 0.0
        self.right_fast += dt if self.rate <= -c.peak_rate else 0.0
        self.left_rate_peak = max(self.left_rate_peak, self.rate)
        self.right_rate_peak = max(self.right_rate_peak, -self.rate)
        sign = 1 if self.left_peak >= self.right_peak else -1
        fast = self.left_fast if sign == 1 else self.right_fast
        opposite_peak = self.right_peak if sign == 1 else self.left_peak
        opposite_fast = self.right_fast if sign == 1 else self.left_fast
        preparation = (0 < c.preparation_angle and opposite_peak < c.preparation_angle
                       and opposite_fast < c.fast_duration)
        speed_peak = self.left_rate_peak if sign == 1 else self.right_rate_peak
        checks = {
            "angle": sign * exc >= c.min_angle,
            "home_angle": sign * (self.yaw - self.home) >= c.min_angle,
            "outward": sign * self.rate >= c.start_rate,
            "duration": t - self.t0 >= c.min_duration,
            "fast_hold": fast >= c.fast_duration,
            "opposite": opposite_peak < c.min_angle * 0.5 or preparation,
            "axis_ratio": self.yaw_travel >= c.yaw_dominance * self.other_travel,
        }
        ratio = self.yaw_travel / max(self.other_travel, 1e-6)
        self.diagnostics = (
            f"phase=out gesture_exc={exc:+.1f}deg peak={self.peak:.1f}deg "
            f"speed_peak={speed_peak:.1f}dps fast={fast*1000:.0f}/{c.fast_duration*1000:.0f}ms "
            f"axis_ratio={ratio:.2f}/{c.yaw_dominance:.2f} "
            f"elapsed={(t-self.t0)*1000:.0f}/{c.window*1000:.0f}ms "
            f"opposite_peak={opposite_peak:.1f}deg opposite_fast={opposite_fast*1000:.0f}ms "
            f"prep={'yes' if preparation and opposite_peak>=c.min_angle*.5 else 'no'} "
            f"blocked={','.join(k for k,v in checks.items() if not v) or 'none'}")
        if self.peak > c.max_angle:
            self._cancel("转角过大", "settling")
            return None
        if t - self.t0 > c.window:
            self._cancel("外甩未达标/超时，停稳后接受新手势")
            return None
        if settled:
            self._cancel("动作未达到阈值")
            return None

        if all(checks.values()):
            self.state, self.reason = "returning", "外甩已达标，等待快速反向回头；尚未切换"
            self.out_sign = sign
            self.out_peak_rate = speed_peak
            self.out_fast = fast
            self.return_t0 = None
            self.return_peak_rate = self.return_fast = 0.0
            self.return_travel = self.return_other = 0.0
            self.still_since = None
        return None
