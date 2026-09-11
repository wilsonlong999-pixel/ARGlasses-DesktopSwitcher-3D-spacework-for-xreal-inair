"""Hardware selection and read-only connection diagnostics.

INAIR uses USB HID, independently of XREAL's TCP transport and axis mapping.
"""
import argparse
import ctypes
import os
import subprocess


def parse_hardware_args(argv=None):
    parser = argparse.ArgumentParser(
        description="快速往返甩头切换桌面；无参数启动时手动选择 INAIR 或 XREAL。",
        allow_abbrev=False)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-Xreal", "-xreal", "--xreal", dest="hardware",
                       action="store_const", const="xreal", help="XREAL One Pro TCP IMU")
    group.add_argument("-Inair", "-inair", "--inair", dest="hardware",
                       action="store_const", const="inair", help="INAIR Pro USB HID 姿态控制")
    parser.set_defaults(hardware=None)
    parser.add_argument("--diagnose", action="store_true",
                        help="仅打印网卡、USB/HID、显示器和串口信息，不连接IMU或切换桌面")
    parser.add_argument("--imu-debug", action="store_true",
                        help="INAIR HID 实时姿态/手势诊断，仅显示，不保存日志或切换桌面")
    args = parser.parse_args(argv)
    if args.imu_debug and (args.hardware != "inair" or args.diagnose):
        parser.error("--imu-debug 需配合 -Inair，且不能与 --diagnose 同用")
    return args


def select_hardware():
    """Read a single console key; redirected input falls back to input()."""
    import sys
    print("请选择眼镜：\n  1 — INAIR Pro\n  2 — XREAL One Pro\n  q — 退出", flush=True)
    while True:
        if os.name == 'nt' and sys.stdin.isatty():
            import msvcrt
            key = msvcrt.getwch().lower()
        else:
            try:
                key = input("选择：").strip().lower()
            except EOFError:
                raise SystemExit("未选择眼镜；可使用 -Inair 或 -Xreal 参数启动。")
        if key in ('1', '2'):
            hardware = 'inair' if key == '1' else 'xreal'
            print("已选择 " + ('INAIR Pro' if hardware == 'inair' else 'XREAL One Pro'), flush=True)
            return hardware
        if key == 'q':
            return None
        print("请按 1、2 或 q。", flush=True)


def ensure_hardware_supported(hardware):
    if hardware not in ("xreal", "inair"):
        raise ValueError("Unknown hardware")


def hardware_diagnostics(hardware):
    """Read device inventory only; no HID open/write, port scans or drivers."""
    lines = [f"[HARDWARE] selected={hardware}; diagnostic only",
             "[INFO] 下列接口名称并不自动证明它属于眼镜。可比较插拔前后差异。"]
    if os.name != "nt":
        return "\n".join(lines + ["Windows device diagnostics required."])
    encoding = f"cp{ctypes.windll.kernel32.GetOEMCP()}"
    commands = [["ipconfig"]] + [
        ["pnputil", "/enum-devices", "/connected", "/class", cls]
        for cls in ("USB", "HIDClass", "Monitor", "Ports")]
    for command in commands:
        lines.append("\n[DIAG] " + " ".join(command))
        try:
            result = subprocess.run(command, capture_output=True, timeout=15,
                                    creationflags=subprocess.CREATE_NO_WINDOW)
            for data in (result.stdout, result.stderr):
                if data:
                    lines.append(data.decode(encoding, errors="replace"))
            if result.returncode:
                lines.append(f"[DIAG] exit_code={result.returncode}")
        except (OSError, subprocess.TimeoutExpired) as exc:
            lines.append(f"[DIAG] unavailable: {exc}")
    return "\n".join(lines)
