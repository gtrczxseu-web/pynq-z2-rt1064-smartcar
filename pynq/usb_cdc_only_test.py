"""Temporary PYNQ test: authorize OpenART CDC interfaces without mass storage.

Run in PYNQ Jupyter; an already attached, unmounted OpenART is supported:
    %run usb_cdc_only_test.py
Then plug OpenART into PYNQ when prompted. The host's default policy is
restored in finally. The per-device interface state lasts until replugging.
Reference: https://docs.kernel.org/usb/authorization.html
"""

import errno
import math
import os
import subprocess
import sys
import time
from pathlib import Path


TARGET_ID = ("1209", "abd1")  # Verified on the user's actual OpenART COM7.


def read_attribute(path):
    return path.read_text().strip().lower()


def write_attribute(path, value):
    path.write_text(str(value) + "\n")


def find_target(root):
    matches = []
    for device in root.iterdir():
        if not (device / "idVendor").is_file():
            continue
        try:
            identity = (read_attribute(device / "idVendor"),
                        read_attribute(device / "idProduct"))
        except OSError as exc:
            if exc.errno in (errno.ENOENT, errno.ENODEV, errno.ENXIO, errno.EIO):
                continue
            raise
        if identity == TARGET_ID:
            matches.append(device)
    if len(matches) > 1:
        raise RuntimeError('发现多个 OpenART；请只连接一个后再试，未修改设备接口。')
    return matches[0] if matches else None


def interfaces_of(root, device):
    interfaces = []
    for interface in sorted(root.glob(device.name + ":*")):
        try:
            interface_class = int(read_attribute(interface / "bInterfaceClass"), 16)
        except (FileNotFoundError, ValueError):
            # sysfs may expose an interface before its attributes are readable.
            continue
        interfaces.append((interface, interface_class))
    return interfaces


def ensure_storage_unmounted(interface, mountinfo=Path('/proc/self/mountinfo')):
    """Match major:minor, including partitions; never detach mounted storage."""
    mounted = {line.split()[2] for line in mountinfo.read_text().splitlines() if len(line.split()) >= 3}
    for block_root in interface.rglob('block'):
        for device in block_root.iterdir():
            for attribute in [device/'dev', *device.glob('*/dev')]:
                if attribute.is_file() and read_attribute(attribute) in mounted:
                    raise RuntimeError('OpenART 磁盘或分区已挂载；请先正常卸载，再重新连接。接口保持原样。')


def authorize_cdc(root, device, interfaces, expected_number):
    """Only this verified device; never reset the bus or detach CDC to recover."""
    if read_attribute(device/'devnum') != expected_number:
        raise RuntimeError('OpenART 在连接期间重新枚举，请保留输出后再试。')
    storage = [interface for interface, kind in interfaces if kind == 0x08]
    for interface in storage:
        ensure_storage_unmounted(interface)
    for interface in storage:
        write_attribute(interface/'authorized', 0)
    for interface, kind in interfaces:
        if kind in (0x02, 0x0A):
            write_attribute(interface/'authorized', 1)
    for interface, kind in interfaces:
        if kind == 0x02 and not (interface/'driver').exists():
            write_attribute(root.parent/'drivers_probe', interface.name)
    if read_attribute(device/'devnum') != expected_number:
        raise RuntimeError('OpenART 在接口启用期间重新枚举，请保留输出后再试。')


def connect_cdc_only(root=Path("/sys/bus/usb/devices"), wait_seconds=90):
    if not math.isfinite(wait_seconds) or wait_seconds <= 0:
        raise ValueError('wait_seconds must be finite and positive')
    policy = root / "usb1" / "interface_authorized_default"
    if not policy.is_file():
        raise RuntimeError("此内核没有所需的 USB 接口授权属性。请把输出发给我。")
    original = read_attribute(policy)
    existing = find_target(root)
    changed_policy = False
    try:
        if existing is None:
            changed_policy = True  # restore even if a sysfs write is interrupted
            write_attribute(policy, 0)
            print("现在把 OpenART 接到 PYNQ USB-A；%g 秒内完成，插上后保持连接。" % wait_seconds, flush=True)
        else:
            print('检测到已插入的 OpenART；检查磁盘未挂载后直接连接，无需拔插。', flush=True)
        deadline = time.monotonic() + wait_seconds
        while time.monotonic() < deadline:
            device = find_target(root)
            if device is not None:
                interfaces = interfaces_of(root, device)
                count_attribute = device / "bNumInterfaces"
                try:
                    expected_count = int(read_attribute(count_attribute))
                    device_number = read_attribute(device/'devnum')
                    if int(device_number) <= 0:
                        raise ValueError('Incomplete USB device number')
                except (FileNotFoundError, ValueError):
                    # Enumeration is not an atomic sysfs snapshot. Missing or
                    # empty attributes are retried within the existing deadline.
                    time.sleep(0.05)
                    continue
                if expected_count <= 0:
                    time.sleep(0.05)
                    continue
                classes = {value for _, value in interfaces}
                if len(interfaces) >= expected_count and {0x02, 0x0A}.issubset(classes):
                    authorize_cdc(root, device, interfaces, device_number)
                    print("已启用 OpenART 的 CDC 串口接口，USB 磁盘接口保持禁用。", flush=True)
                    return device.name
            time.sleep(0.05)
        raise RuntimeError("等待超时：没有找到完整的 OpenART USB 接口。")
    finally:
        if changed_policy:
            write_attribute(policy, original)
            print("USB 总线的默认授权设置已恢复。", flush=True)


def main():
    if sys.platform != "linux" or os.geteuid() != 0:
        raise RuntimeError("本测试需要在 PYNQ 上以 root 权限运行。请把这条输出发给我。")
    try:
        from serial.tools import list_ports
    except ImportError:
        sys.path.insert(0, str(Path(__file__).parent / "pyserial-3.5-py2.py3-none-any.whl"))
        from serial.tools import list_ports
    from openart_usb import OpenARTUSB

    device_name = connect_cdc_only()
    root = Path("/sys/bus/usb/devices")
    initial_device_number = read_attribute(root / device_name / "devnum")
    serial_port = None
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        if not (root / device_name).exists():
            raise RuntimeError("模块仍自动断开：临时禁用磁盘接口没有解决掉线。请发完整输出。")
        if read_attribute(root / device_name / "devnum") != initial_device_number:
            raise RuntimeError("模块发生了 USB 重新枚举，连接仍不稳定。请发完整输出。")
        for item in list_ports.comports():
            if (item.vid, item.pid) == (0x1209, 0xABD1):
                serial_port = item.device
        time.sleep(0.5)
    if serial_port is None:
        raise RuntimeError("设备持续存在，但没有找到对应串口。请发完整输出。")
    print("PYNQ 侧连接观察 30 秒通过。", flush=True)
    print("摄像头串口:", serial_port, flush=True)
    camera = OpenARTUSB(serial_port, timeout=5.0)
    try:
        # The successful PC probe allowed CDC setup to settle before USBDBG.
        # This is a timing check, not a confirmed explanation of missing replies.
        time.sleep(0.5)
        firmware = camera.handshake()
    finally:
        camera.close()
    print("实际固件版本:", ".".join(str(part) for part in firmware))
    print("保持模块连接，可回到原 Notebook 第 4 步继续。重新插拔后此接口设置失效。")


if __name__ == "__main__":
    try:
        main()
    finally:
        if sys.platform == "linux":
            log = subprocess.run(["dmesg"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 universal_newlines=True)
            print("最近的系统日志：")
            print("\n".join(log.stdout.splitlines()[-25:]))
