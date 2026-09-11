# -*- coding: utf-8 -*-
"""
XREAL One Pro 修复版 (原: main_udp_yaw_desktop_switcher.py)
===========================================================
原版问题: 依赖 PhoenixHeadTracker.exe (AirAPI_Windows.dll), 仅支持 XREAL Air 系列,
XREAL One Pro 不走 Air 的 HID 协议 => 收不到任何数据 ("数据没取到" 的根源)。

数据源:
XREAL One Pro 内置 IMU 服务 (TCP 169.254.2.1:52998, 需在眼镜开发者菜单中
保持以太网开启, 默认开启)。
启动后按1选择INAIR，按2选择XREAL；也可用 -Xreal / -Inair 参数直接选择。
INAIR 使用姿态四元数差分和设备时间戳；--imu-debug 仅诊断，不切换。

切换语义 (快速外甩 + 快速回向确认 + 任意朝向停稳):
    - XREAL 使用 Y-Z 合成横转轴；转角、速度、持续时间、横向占比同时达标才切换
    - 快速外甩+快速回向确认才切换；未停稳前的连续回头/回弹不再次切换
    - 任意朝向停稳后接受下一次，当前位置就是新的动作起点
    - TCP 接收独立线程；按接收批次分配采样时间，避免切换/打印污染积分
    - 具体参数和测试方法见 FLICK_TUNING.md

运行键位 (可直接调参, 无需改代码):
    = / -    : 最小转角 MIN_EXCURSION ±5° (甩头至少转多少度才算)
    [ / ]    : 外甩确认窗口 ±100ms (外甩达标只记录，回向达标才切换)
    z        : 清除手势状态，静止后将当前朝向设为新的参考点
    r        : 重新校准陀螺仪零偏 (保持眼镜静止)
    i        : 左右方向反转开关
    d        : 调试输出开关
    a        : 6秒三轴诊断（此期间不切换，请只左右转头）
    1/2/3    : 手动选择 gx/gy/gz；选择后须重新静止
    q        : 退出
    (桌面1 开关 EXCLUDE_DESKTOP_1 在文件顶部 CONFIG 区, 改代码切换)
"""
import socket, time, math, os, ctypes
from head_flick import FlickConfig, HeadFlickDetector, AxisProbe
from imu_stream import IMUStream
from hardware_options import parse_hardware_args, ensure_hardware_supported, hardware_diagnostics, select_hardware

# =========================
# CONFIG
# =========================
# --- XREAL One Pro IMU (替代 PhoenixHeadTracker) ---
GLASSES_IP = "169.254.2.1"
GLASSES_PORT = 52998
RECV_TIMEOUT = 5.0
YAW_AXIS = 1        # 本轮佩戴日志主轴诊断建议 Y；1/2/3 可重新选择 X/Y/Z
YAW_INVERT = True    # Y轴正负方向尚需佩戴确认；方向反了按 i
XREAL_YAW_AXIS = 1
XREAL_YAW_INVERT = True
XREAL_YAW_DOMINANCE = 0.90
XREAL_YAW_PROJECTION = "yz_difference"
YAW_PROJECTION = XREAL_YAW_PROJECTION
CALIB_SAMPLES = 500     # 陀螺仪零偏校准采样数 (~0.4秒)
CAL_MAX_STD = 0.015     # 校准期间允许的最大波动 rad/s, 超过自动重采
CAL_MAX_ATTEMPTS = 20   # 保持当前连接等待静止；约10秒后才交给重连流程

ENABLE_DESKTOP_SWITCH = True  # True => troca desktop via VirtualDesktopAccessor.dll
SWITCH_COOLDOWN_MS = 350      # 与任意朝向停稳条件共同生效

# --- 桌面1 参与开关 ---
# EXCLUDE_DESKTOP_1 = True  => 禁止甩头切换到桌面1 (下限=2)
# EXCLUDE_DESKTOP_1 = False => 允许切换到桌面1 (下限=1, 可从桌面2左甩到桌面1)
EXCLUDE_DESKTOP_1 = False
MIN_DESKTOP = 2 if EXCLUDE_DESKTOP_1 else 1

# --- 甩头手势：起始值需佩戴实测；单位 °、°/s、ms ---
MIN_EXCURSION = 12.0     # 相对动作起点，最小外甩转角
START_RATE = 18.0        # 开始追踪；单凭这个速度不会切换
PEAK_RATE = 80.0         # 同方向超过此速度累计至少 FAST_HOLD_MS
RETURN_RATE = 80.0       # 回向速度门槛，与外甩分开检验
RETURN_ANGLE = 6.0       # 至少回退6度，且不低于完整外甩幅度的35%
RETURN_WINDOW_MS = 450   # 开始反向后必须在此时间内完成
GESTURE_WINDOW_MS = 1200 # 外甩开始到快速回向确认的总时限
FAST_HOLD_MS = 35
MIN_FLICK_MS = 60        # 排除短脉冲
GLANCE_WINDOW_MS = 650   # 外甩阶段窗口；失败后停稳开始新动作
MAX_FLICK_EXC = 75.0
RECOVERY_TIMEOUT_MS = 1000  # 恢复状态上限；停稳且冷却结束可提前恢复，仍在运动则等待停止
MAX_GYRO_RATE = 2000.0   # 三轴速度异常保护工程值，容纳日志700～855°/s；不代表设备量程
STILL_VEL = 8.0          # 三轴总角速度（不是只判断 yaw）
STILL_HOLD_MS = 220      # 任意朝向连续稳定时长
STILL_SPAN = 1.5         # 稳定期间 yaw 最大摆幅
FILTER_TAU_MS = 18       # 与实际 dt 相关的低通滤波
YAW_DOMINANCE = 0.90     # 日志中 Y/Z 耦合明显，允许两者接近；仍排除其他轴明显占优
MAX_SAMPLE_AGE_MS = 100  # 旧数据不用于触发桌面切换

# --- centraliza só quando chegar o primeiro pacote ---
CENTER_ON_FIRST_PACKET = True
CENTER_DESKTOP = 2            # desktop central
CENTER_SWITCH_RETRIES = 3      # tentativas pra centralizar
CENTER_SWITCH_VERIFY_DELAY = 0.03  # seg entre tentativa e verificação

DEBUG_PRINTS = False

RAD2DEG = 180.0 / math.pi
BUILD_VERSION = "20260911-final-no-log"

# IMU 帧解析位于 imu_stream.py；参考 SamiMitwalli/One-Pro-IMU-Retriever-Demo。

def update_thresholds():
    axis_label = "Y-Z" if YAW_PROJECTION == "yz_difference" else "XYZ"[YAW_AXIS]
    print(f"[CFG] MIN_EXC={MIN_EXCURSION:.0f}°  MODE=fast_out_fast_back REARM=current_pose RECOVERY_MAX={RECOVERY_TIMEOUT_MS}ms MAX_RATE={MAX_GYRO_RATE:.0f}dps  "
          f"WIN={GLANCE_WINDOW_MS}ms  STILL={STILL_HOLD_MS}ms  "
          f"START_RATE={START_RATE:.0f}°/s OUT_RATE={PEAK_RATE:.0f}°/s BACK_RATE={RETURN_RATE:.0f}°/s  MIN_DESKTOP={MIN_DESKTOP}  "
          f"AXIS={axis_label} INVERT={'ON' if YAW_INVERT else 'OFF'}")


def select_motion_components(rates, axis, invert, projection=None):
    """Return selected yaw, two orthogonal rates, and a diagnostic label."""
    if projection == "yz_difference":
        # XREAL's fitted frame is rotated around X: actual horizontal turning
        # appears on Y and Z with opposite signs. Rotate that plane by 45° so
        # the matching component is yaw and Y+Z remains an interference check.
        # Average rather than length-normalize, keeping existing angle/speed
        # thresholds on roughly the same scale as the formerly selected Y axis.
        scale = 0.5
        selected = (rates[1] - rates[2]) * scale
        others = (rates[0], (rates[1] + rates[2]) * scale)
        label = "Y-Z"
    else:
        selected = rates[axis]
        other_axes = [i for i in range(3) if i != axis]
        others = (rates[other_axes[0]], rates[other_axes[1]])
        label = "XYZ"[axis]
    if invert:
        selected = -selected
    return selected, others, label


# =========================
# Win32 helpers (anti "taskbar flashing") — 原版未改动
# =========================
user32 = ctypes.windll.user32
kernel32 = ctypes.windll.kernel32

SW_MINIMIZE = 6

user32.FindWindowW.argtypes = [ctypes.c_wchar_p, ctypes.c_wchar_p]
user32.FindWindowW.restype = ctypes.c_void_p

user32.GetForegroundWindow.argtypes = []
user32.GetForegroundWindow.restype = ctypes.c_void_p

user32.GetWindowThreadProcessId.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32)]
user32.GetWindowThreadProcessId.restype = ctypes.c_uint32

kernel32.GetCurrentThreadId.argtypes = []
kernel32.GetCurrentThreadId.restype = ctypes.c_uint32

user32.AttachThreadInput.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_bool]
user32.AttachThreadInput.restype = ctypes.c_bool

user32.SetForegroundWindow.argtypes = [ctypes.c_void_p]
user32.SetForegroundWindow.restype = ctypes.c_bool

user32.ShowWindow.argtypes = [ctypes.c_void_p, ctypes.c_int]
user32.ShowWindow.restype = ctypes.c_bool


def _activate_progman_desktop():
    """
    Foca o 'desktop window' (Progman / 'Program Manager') antes de trocar.
    """
    hwnd = user32.FindWindowW("Progman", "Program Manager")
    if not hwnd:
        return None

    dummy = ctypes.c_uint32(0)
    desktop_tid = user32.GetWindowThreadProcessId(hwnd, ctypes.byref(dummy))

    fg = user32.GetForegroundWindow()
    fg_tid = user32.GetWindowThreadProcessId(fg, ctypes.byref(dummy)) if fg else 0

    cur_tid = kernel32.GetCurrentThreadId()

    if desktop_tid and fg_tid and (fg_tid != cur_tid):
        user32.AttachThreadInput(desktop_tid, cur_tid, True)
        user32.AttachThreadInput(fg_tid, cur_tid, True)
        user32.SetForegroundWindow(hwnd)
        user32.AttachThreadInput(fg_tid, cur_tid, False)
        user32.AttachThreadInput(desktop_tid, cur_tid, False)
    else:
        user32.SetForegroundWindow(hwnd)

    time.sleep(0.005)
    return hwnd


def _minimize_progman(hwnd):
    if hwnd:
        user32.ShowWindow(hwnd, SW_MINIMIZE)
        time.sleep(0.005)

# =========================
# VirtualDesktopAccessor (opcional) — 原版未改动
# =========================
HERE = os.path.dirname(os.path.abspath(__file__))
VDA_DLL_PATH = os.path.join(HERE, "VirtualDesktopAccessor.dll")


def load_vda():
    if not os.path.exists(VDA_DLL_PATH):
        raise SystemExit("❌ VirtualDesktopAccessor.dll não encontrada na pasta do script.")
    os.add_dll_directory(HERE)

    vda = ctypes.WinDLL(VDA_DLL_PATH)

    vda.GetCurrentDesktopNumber.argtypes = []
    vda.GetCurrentDesktopNumber.restype = ctypes.c_int

    vda.GoToDesktopNumber.argtypes = [ctypes.c_int]
    vda.GoToDesktopNumber.restype = None

    return vda


def get_current_desktop_1based(vda) -> int:
    return int(vda.GetCurrentDesktopNumber()) + 1


def goto_desktop_1based(vda, target: int):
    """
    Switch desktop via VDA (rápido) + workaround anti-flash:
      1) foca Progman (desktop)
      2) troca desktop
      3) minimiza Progman
    """
    progman = _activate_progman_desktop()
    vda.GoToDesktopNumber(int(target) - 1)  # API é 0-based
    time.sleep(0.01)
    _minimize_progman(progman)

# =========================
# XREAL One Pro IMU: TCP 读取 + 解析 (替代 PhoenixHeadTracker/UDP)
# =========================
def connect_glasses():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(RECV_TIMEOUT)
    try:
        s.connect((GLASSES_IP, GLASSES_PORT))
    except OSError:
        s.close()
        raise
    return s


def calibrate_bias(samples, detail_log=None):
    """采集样本估计三轴陀螺仪零偏；运动时持续重采，最多等待约10秒。"""
    print("[启动] 请保持 XREAL 眼镜静止，正在校准……")
    for attempt in range(1, CAL_MAX_ATTEMPTS + 1):
        cal = [[], [], []]
        for _, v in samples:
            for i in range(3):
                cal[i].append(v[i])
            if len(cal[0]) >= CALIB_SAMPLES:
                break
        meds = []
        std = 0.0
        stds = []
        for i in range(3):
            c = sorted(cal[i])
            n = len(c)
            med = c[n // 2]
            meds.append(med)
            axis_std = (sum((x - med) ** 2 for x in c) / n) ** 0.5
            stds.append(axis_std)
            std = max(std, axis_std)
        if std <= CAL_MAX_STD:
            if detail_log:
                detail_log.write(f"[CAL] attempt={attempt} accepted=yes "
                                 f"bias_rad={meds[0]:+.7f}/{meds[1]:+.7f}/{meds[2]:+.7f} "
                                 f"std_xyz={stds[0]:.7f}/{stds[1]:.7f}/{stds[2]:.7f} "
                                 f"max_std={std:.7f} limit={CAL_MAX_STD:.7f}")
            return tuple(meds)
        if detail_log:
            detail_log.write(f"[CAL] attempt={attempt} accepted=no "
                             f"bias_rad={meds[0]:+.7f}/{meds[1]:+.7f}/{meds[2]:+.7f} "
                             f"std_xyz={stds[0]:.7f}/{stds[1]:.7f}/{stds[2]:.7f} "
                             f"max_std={std:.7f} limit={CAL_MAX_STD:.7f}")
        if attempt == 1 or attempt % 4 == 0:
            print(f"[启动] 眼镜仍在运动，继续等待静止（{attempt}/{CAL_MAX_ATTEMPTS}）")
    raise ConnectionError("校准等待超时：请保持眼镜静止")

# =========================
# 甩头手势 (glance) 判定
# =========================
# MIN_DESKTOP 在 CONFIG 区由 EXCLUDE_DESKTOP_1 决定 (1 或 2)

def glance_target(current: int, direction: str, max_desktop: int) -> int:
    """右甩 => 当前桌面+1, 左甩 => 当前桌面-1, 边界处夹紧 (下限 MIN_DESKTOP)。返回 0 表示无需切换。"""
    if direction == "right":
        target = current + 1
    else:
        target = current - 1
    if max_desktop < MIN_DESKTOP:
        return 0
    target = max(MIN_DESKTOP, min(max_desktop, target))
    return target if target != current else 0

# =========================
# 键盘: 运行时调参 (非阻塞)
# =========================
def check_keys():
    try:
        import msvcrt
    except ImportError:
        return None
    try:
        cmd = None
        while msvcrt.kbhit():
            ch = msvcrt.getwch().lower()
            if ch in ("q",):
                cmd = "quit"
            elif ch in ("=", "+"):        # 阈值 +5°
                cmd = "angle_up"
            elif ch in ("-", "_"):        # 阈值 -5°
                cmd = "angle_dn"
            elif ch in ("[", "{"):        # 甩头窗口 -100ms
                cmd = "window_dn"
            elif ch in ("]", "}"):        # 甩头窗口 +100ms
                cmd = "window_up"
            elif ch in ("z", "t"):
                cmd = "zero"
            elif ch in ("r",):
                cmd = "recal"
            elif ch in ("i",):
                cmd = "invert"
            elif ch in ("d",):
                cmd = "debug"
            elif ch == "a":
                cmd = "axis_probe"
            elif ch in ("1", "2", "3"):
                cmd = "axis_" + ch
        return cmd
    except OSError:
        return None  # 无控制台时忽略键盘


# =========================
# main
# =========================
def main(argv=None):
    global YAW_AXIS, YAW_INVERT, YAW_DOMINANCE, YAW_PROJECTION
    args = parse_hardware_args(argv)
    if args.hardware is None:
        args.hardware = select_hardware()
        if args.hardware is None:
            return
    if args.imu_debug:
        from inair_diagnostics import run_live
        return run_live()
    if args.diagnose:
        print(hardware_diagnostics(args.hardware))
        return
    ensure_hardware_supported(args.hardware)
    if args.hardware == "inair":
        from inair_pose import INAIR_YAW_DOMINANCE
        YAW_AXIS, YAW_INVERT = 2, False
        YAW_DOMINANCE = INAIR_YAW_DOMINANCE
        YAW_PROJECTION = None
        return run_inair()
    # Restore the verified XREAL profile explicitly on every XREAL start.
    # Do not inherit mutable values left by INAIR or an earlier invocation.
    YAW_AXIS, YAW_INVERT = XREAL_YAW_AXIS, XREAL_YAW_INVERT
    YAW_DOMINANCE = XREAL_YAW_DOMINANCE
    YAW_PROJECTION = XREAL_YAW_PROJECTION
    return run_xreal()


def run_xreal():
    vda = load_vda() if ENABLE_DESKTOP_SWITCH else None
    print(f"[版本] {BUILD_VERSION}")
    while True:
        sock = None
        try:
            sock = connect_glasses()
            print(f"[状态] XREAL 已连接 {GLASSES_IP}:{GLASSES_PORT}")
            with IMUStream(sock) as stream:
                run_loop(stream.samples(), vda)
            return
        except (ConnectionError, OSError) as exc:
            print(f"[连接中断] {exc}；2秒后重连。")
            time.sleep(2)
        finally:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass


def run_inair():
    from inair_stream import InairStream, open_device
    vda = load_vda() if ENABLE_DESKTOP_SWITCH else None
    try:
        while True:
            try:
                device = open_device()
                with InairStream(device) as stream:
                    run_loop(stream.samples(), vda, pose_source=True)
                return
            except (OSError, ConnectionError) as exc:
                print(f'[连接异常] {exc}；2秒后重新连接，连接后需停稳；q退出')
                until = time.monotonic()+2
                while time.monotonic() < until:
                    if check_keys() == 'quit':
                        return
                    time.sleep(.05)
    except KeyboardInterrupt:
        print('[MAIN] 已退出')


def detector_config(pose_source=False):
    config = FlickConfig(
        min_angle=MIN_EXCURSION, start_rate=START_RATE, peak_rate=PEAK_RATE,
        min_duration=MIN_FLICK_MS / 1000, fast_duration=FAST_HOLD_MS / 1000,
        window=GLANCE_WINDOW_MS / 1000, max_angle=MAX_FLICK_EXC,
        filter_tau=FILTER_TAU_MS / 1000, still_rate=STILL_VEL,
        still_span=STILL_SPAN, still_hold=STILL_HOLD_MS / 1000,
        cooldown=SWITCH_COOLDOWN_MS / 1000, recovery_timeout=RECOVERY_TIMEOUT_MS / 1000,
        max_rate=MAX_GYRO_RATE, return_rate=RETURN_RATE, return_angle=RETURN_ANGLE,
        return_window=RETURN_WINDOW_MS / 1000, gesture_window=GESTURE_WINDOW_MS / 1000,
        yaw_dominance=YAW_DOMINANCE, recovery_yaw_only=not pose_source)
    if pose_source:
        from inair_pose import configure_inair
        config = configure_inair(config)
    return config


def desktop_count(vda):
    vda.GetDesktopCount.argtypes = []
    vda.GetDesktopCount.restype = ctypes.c_int
    count = int(vda.GetDesktopCount())
    if count < 1:
        raise OSError("VirtualDesktopAccessor 返回无效桌面数量")
    return count


def run_loop(samples, vda, pose_source=False, detail_log=None):
    global MIN_EXCURSION, GLANCE_WINDOW_MS, YAW_INVERT, DEBUG_PRINTS, YAW_AXIS, YAW_PROJECTION
    gyro_bias = (0., 0., 0.) if pose_source else calibrate_bias(samples, detail_log)
    if not pose_source:
        print("[状态] XREAL 校准完成；保持静止，等待手势就绪")
    detector = HeadFlickDetector(detector_config(pose_source))
    max_desktop = desktop_count(vda) if vda else 3
    current_desktop = get_current_desktop_1based(vda) if vda else CENTER_DESKTOP
    # 启动定位先完成，之后的积压数据会按时间戳丢弃，不拿来积分。
    center = max(1, min(max_desktop, CENTER_DESKTOP))
    if vda and not pose_source and CENTER_ON_FIRST_PACKET and current_desktop != center:
        for _ in range(CENTER_SWITCH_RETRIES):
            goto_desktop_1based(vda, center)
            time.sleep(CENTER_SWITCH_VERIFY_DELAY)
            current_desktop = get_current_desktop_1based(vda)
            if current_desktop == center:
                break
        if current_desktop != center:
            print(f"[WARN] 启动定位未成功，实际桌面={current_desktop}")
    print(f"[启动] {max_desktop} 个桌面，当前={current_desktop}；请先保持静止片刻")
    print("[操作] 快速外甩并快速返回切换桌面；q退出，i反转方向。")
    if DEBUG_PRINTS:
        update_thresholds()
    if pose_source and DEBUG_PRINTS:
        print(f"[INAIR-CFG] STILL=12dps QUIET=220/280ms SPIKE_MAX=35dps PREP_MAX={detector.cfg.preparation_angle:.0f}deg; RECOVERY_MAX=1000ms")
    other_axes = [i for i in range(3) if i != YAW_AXIS]
    accept_after = time.monotonic()
    session_started = accept_after
    last_dbg = last_keys = last_desktop_poll = 0.0
    pending_target = None
    pending_until = 0.0
    previous_state = detector.state
    axis_probe = None
    last_rejections = 0
    last_file_rejections = 0
    last_fault_count = 0
    last_axis_key = float("-inf")
    compact_state = None
    last_file_log = float("-inf")
    sample_count = 0
    axis_window_start = None
    axis_window_count = 0
    axis_window_squares = [0.0, 0.0, 0.0]
    axis_window_peaks = [0.0, 0.0, 0.0]

    for sample_t, v in samples:
        sample_count += 1
        now = time.monotonic()
        cmd = None
        if now - last_keys >= 0.02:
            last_keys = now
            cmd = check_keys()
        if cmd == "quit":
            print("[MAIN] 已退出")
            return
        if cmd == "recal":
            gyro_bias = (0., 0., 0.) if pose_source else calibrate_bias(samples, detail_log)
            if pose_source:
                print("[INAIR] 使用融合姿态，不扣除原始陀螺仪零偏；已重置，请停稳")
            detector.reset()
            axis_probe = None
            accept_after = time.monotonic()
            continue  # 丢弃校准前取出的旧样本
        if cmd == "zero":
            detector.reset()
            axis_probe = None
            accept_after = now
            print("[KEY] 已重置，请在新的正前方保持静止")
        if cmd == "axis_probe":
            fresh_press = now - last_axis_key > 0.4
            last_axis_key = now
            if axis_probe is None and fresh_press:
                axis_probe = AxisProbe(now)
                detector.reset()
                accept_after = now
                print("[AXIS] 接下来6秒只做左右转头再回正；按a一次即可，重复按键不重启计时")
        if cmd in ("axis_1", "axis_2", "axis_3"):
            YAW_AXIS = int(cmd[-1]) - 1
            YAW_PROJECTION = None
            other_axes = [i for i in range(3) if i != YAW_AXIS]
            axis_probe = None
            detector.reset()
            accept_after = now
            update_thresholds()
        if cmd == "angle_up":
            MIN_EXCURSION = min(60.0, MIN_EXCURSION + 5.0)
        if cmd == "angle_dn":
            MIN_EXCURSION = max(5.0, MIN_EXCURSION - 5.0)
        if cmd == "window_up":
            GLANCE_WINDOW_MS = min(1000, GLANCE_WINDOW_MS + 100)
        if cmd == "window_dn":
            GLANCE_WINDOW_MS = max(150, GLANCE_WINDOW_MS - 100)
        if cmd == "invert":
            YAW_INVERT = not YAW_INVERT
        if cmd in ("angle_up", "angle_dn", "window_up", "window_dn", "invert"):
            detector.cfg = detector_config(pose_source)
            detector.reset()  # 不允许半途变更阈值/方向触发残留动作
            accept_after = now
            update_thresholds()
        if cmd == "debug":
            DEBUG_PRINTS = not DEBUG_PRINTS

        if sample_t <= accept_after:
            continue
        if now - sample_t > MAX_SAMPLE_AGE_MS / 1000:
            detector.invalidate("丢弃积压数据；回正锁保留")
            if detail_log:
                detail_log.write(f"[STALE] sample={sample_count} age_ms={(now-sample_t)*1000:.2f}")
            continue
        motion_t = v[6] if pose_source else sample_t
        rates = [(v[i] - gyro_bias[i]) * RAD2DEG for i in range(3)]
        if detail_log:
            if axis_window_start is None:
                axis_window_start = motion_t if pose_source else sample_t
            axis_window_count += 1
            for i, value in enumerate(rates):
                axis_window_squares[i] += value * value
                axis_window_peaks[i] = max(axis_window_peaks[i], abs(value))
            axis_now = motion_t if pose_source else sample_t
            if axis_now-axis_window_start >= .5:
                rms = [(value/max(axis_window_count, 1)) ** .5 for value in axis_window_squares]
                detail_log.write(f"[AXIS_WINDOW] samples={axis_window_count} "
                                 f"rms_dps={rms[0]:.2f}/{rms[1]:.2f}/{rms[2]:.2f} "
                                 f"peak_dps={axis_window_peaks[0]:.2f}/{axis_window_peaks[1]:.2f}/{axis_window_peaks[2]:.2f}")
                axis_window_start = axis_now
                axis_window_count = 0
                axis_window_squares = [0.0, 0.0, 0.0]
                axis_window_peaks = [0.0, 0.0, 0.0]
        if axis_probe is not None:
            axis_probe.update(sample_t, rates)
            if sample_t - axis_probe.start >= axis_probe.duration:
                suggested, rms, peaks = axis_probe.result()
                print(f"[AXIS] RMS X/Y/Z={rms[0]:.1f}/{rms[1]:.1f}/{rms[2]:.1f}dps "
                      f"PEAK={peaks[0]:.1f}/{peaks[1]:.1f}/{peaks[2]:.1f}dps")
                if suggested is None:
                    print("[AXIS] 运动不足、缺样或轴不明确；保持当前轴，可按a重测")
                else:
                    print(f"[AXIS] 若刚才只做左右转头，建议={'XYZ'[suggested]}，"
                          f"当前={'XYZ'[YAW_AXIS]}；按{suggested+1}可选择，未自动改轴")
                axis_probe = None
                detector.reset()
                accept_after = now
                print("[AXIS] 诊断结束，请保持静止等待ready；方向反了按i")
            continue
        rate, motion_others, axis_label = select_motion_components(
            rates, YAW_AXIS, YAW_INVERT, None if pose_source else YAW_PROJECTION)
        direction = detector.update(motion_t, rate, *motion_others)

        if detail_log and (motion_t-last_file_log >= .100 or detector.state != previous_state or direction):
            last_file_log = motion_t
            detail_log.write(
                f"[SAMPLE] n={sample_count} t={motion_t-session_started:.6f} "
                f"age_ms={(now-sample_t)*1000:.2f} axis={axis_label} invert={YAW_INVERT} "
                f"raw_rad={v[0]:+.7f}/{v[1]:+.7f}/{v[2]:+.7f} "
                f"rate_dps={rates[0]:+.2f}/{rates[1]:+.2f}/{rates[2]:+.2f} "
                f"selected={rate:+.2f} filtered={detector.rate:+.2f} "
                f"exc={detector.yaw-detector.rest:+.2f} state={detector.state} "
                f"reason={detector.reason} desktop={current_desktop} checks={detector.diagnostics}")

        # Minimal switching trace. This remains visible when detailed debug is off.
        if not DEBUG_PRINTS and detector.state != compact_state:
            if detector.state == "ready":
                print("[状态] 已就绪")
            elif detector.state == "tracking":
                print("[跟踪] 检测到外甩，正在确认速度和幅度")
            elif detector.state == "returning":
                print("[跟踪] 外甩已达标，等待快速反向回甩")
            elif detector.state == "settling" and compact_state in ("tracking", "returning"):
                print(f"[跟踪] 本次动作未通过：{detector.reason}")
            compact_state = detector.state
        if detector.fault_count != last_fault_count:
            if (detail_log and detector.fault_count > last_fault_count
                    and (detector.fault_count <= 5 or detector.fault_count % 100 == 0)):
                detail_log.write(f"[IMU_FAULT] count={detector.fault_count} "
                                 f"n={sample_count} {detector.last_fault}")
            if detector.fault_count > last_fault_count and DEBUG_PRINTS:
                print(f"[IMU-WARN] t={sample_t-session_started:.3f}s {detector.last_fault}")
            last_fault_count = detector.fault_count

        # Windows 切换是异步的；读取真实结果，禁止把请求当成成功。
        if vda and pending_target is None and now-last_desktop_poll >= .5:
            last_desktop_poll = now
            actual = get_current_desktop_1based(vda)
            if actual != current_desktop:
                if detail_log:
                    detail_log.write(f"[DESKTOP_EXTERNAL] {current_desktop}->{actual}")
                print(f"[DESKTOP] 检测到实际桌面变化 {current_desktop}->{actual}（无本程序待确认请求）")
                current_desktop = actual
        if pending_target is not None:
            actual = get_current_desktop_1based(vda)
            if actual == pending_target:
                if detail_log:
                    detail_log.write(f"[SWITCH_CONFIRMED] desktop={actual}")
                current_desktop = actual
                print(f"[SWITCH] 已确认桌面 {actual}")
                pending_target = None
            elif now >= pending_until:
                if detail_log:
                    detail_log.write(f"[SWITCH_FAILED] requested={pending_target} actual={actual}")
                current_desktop = actual
                print(f"[WARN] 切换未确认：请求={pending_target}，实际={actual}")
                pending_target = None
            # 等待期间出现的新手势也已消费，避免稍后补发。
            direction = None

        if DEBUG_PRINTS and (detector.state != previous_state or now - last_dbg >= 0.5):
            print(f"[YAW] t={sample_t-session_started:.3f}s axis={axis_label} "
                  f"rest_exc={detector.yaw - detector.rest:+6.1f}° "
                  f"rate={detector.rate:+6.1f}°/s state={detector.state} "
                  f"({detector.reason}) desktop={current_desktop} "
                  f"gyroXYZ={rates[0]:+.1f}/{rates[1]:+.1f}/{rates[2]:+.1f}dps "
                  f"age={(now-sample_t)*1000:.1f}ms")
            if detector.state in ("settling", "tracking", "returning", "recovering", "watching"):
                tag = "RECOVER" if detector.state == "recovering" and direction is None else "CHECK"
                print(f"[{tag}] {detector.diagnostics}")
            last_dbg = now
        if DEBUG_PRINTS and detector.rejections != last_rejections:
            if detector.rejections > last_rejections:
                print(f"[REJECT] t={sample_t-session_started:.3f}s "
                      f"{detector.reason} | {detector.diagnostics}")
            last_rejections = detector.rejections
        if detail_log and detector.rejections != last_file_rejections:
            detail_log.write(f"[REJECT] n={sample_count} reason={detector.reason} checks={detector.diagnostics}")
            last_file_rejections = detector.rejections
        previous_state = detector.state
        if not direction:
            continue
        if vda:
            current_desktop = get_current_desktop_1based(vda)
            max_desktop = desktop_count(vda)  # 兼容手动切换/增删桌面
        target = glance_target(current_desktop, direction, max_desktop)
        if not target:
            if detail_log:
                detail_log.write(f"[BOUNDARY] direction={direction} desktop={current_desktop}")
            print(f"[FLICK] {direction} 已到边界，保持桌面 {current_desktop}")
            continue
        print(f"[FLICK] {'向左' if direction == 'left' else '向右'}：桌面 {current_desktop} → {target}")
        if detail_log:
            detail_log.write(f"[FLICK] direction={direction} desktop={current_desktop}->{target} "
                             f"peak={detector.peak:.2f} checks={detector.diagnostics}")
        if vda:
            goto_desktop_1based(vda, target)
            pending_target = target
            pending_until = time.monotonic() + 0.6
        else:
            current_desktop = target
            print(f"[DRY-RUN] 模拟桌面 {target}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[MAIN] 已退出")
