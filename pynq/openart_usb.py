"""Minimal legacy OpenMV USBDBG client for an OpenART Plus firmware probe.

Protocol references (compatibility must be checked on the actual module):
https://github.com/nxp-mcuxpresso/OpenART/blob/master/bsp/imxrt/components/openmv-nxp/omv/usbdbg.h
https://github.com/openmv/openmv/blob/v4.5.2/tools/pyopenmv.py
No flash, reset, or persistent script-save commands are implemented.
"""

import io
import struct
import sys
import time

import numpy as np


class ProtocolError(RuntimeError):
    pass


def decode_frame(width, height, kind, payload, rgb565_byteorder="big"):
    """Return a contiguous uint8 BGR image for PYNQ's default HDMI pixel format."""
    if kind == 1:
        gray = np.frombuffer(payload, dtype=np.uint8).reshape(height, width)
        return np.repeat(gray[:, :, None], 3, axis=2)
    if kind == 2:
        if rgb565_byteorder not in ("big", "little"):
            raise ValueError("rgb565_byteorder must be big or little")
        pixels = np.frombuffer(payload, dtype=">u2" if rgb565_byteorder == "big" else "<u2")
        pixels = pixels.astype(np.uint16).reshape(height, width)
        red = ((pixels >> 11) * 255 // 31).astype(np.uint8)
        green = (((pixels >> 5) & 63) * 255 // 63).astype(np.uint8)
        blue = ((pixels & 31) * 255 // 31).astype(np.uint8)
        return np.ascontiguousarray(np.stack((blue, green, red), axis=2))
    if kind > 2:
        try:
            import cv2
        except ImportError:
            from PIL import Image
            with Image.open(io.BytesIO(payload)) as image:
                bgr = np.asarray(image.convert("RGB"))[:, :, ::-1].copy()
        else:
            bgr = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ProtocolError("收到完整帧，但 JPEG 解码失败。请核对固件的帧格式。")
        if bgr.shape != (height, width, 3):
            raise ProtocolError("JPEG 尺寸与 USB 帧头不一致。请核对固件协议。")
        return bgr
    raise ProtocolError("不支持的帧格式：{}".format(kind))


class OpenARTUSB:
    def __init__(self, port=None, baudrate=921600, timeout=5.0,
                 frame_timeout=3.0, rgb565_byteorder="big", transport=None,trace_callback=None):
        if transport is None:
            import serial
            if sys.platform.startswith("linux"):
                if baudrate not in (921600, 12000000):
                    raise ValueError("OpenART USBDBG 调试波特率需为 921600 或 12000000。")
                transport = serial.Serial(port=None, baudrate=baudrate,
                                          timeout=0.1, write_timeout=1.0)
                transport.dtr = True
                transport.rts = False
                transport.port = port
                try:
                    transport.open()
                    # Validated on this PYNQ/OpenART pair: open with RTS inactive,
                    # then apply both debug rates after control-line setup.
                    transport.baudrate = 12000000 if baudrate == 921600 else 921600
                    time.sleep(0.2)
                    transport.baudrate = baudrate
                    time.sleep(0.5)
                except BaseException:
                    transport.close()
                    raise
            else:
                transport = serial.Serial(port, baudrate=baudrate, timeout=0.1,
                                          write_timeout=1.0)
        self.serial = transport
        self.timeout = timeout
        self.frame_timeout = frame_timeout
        self.rgb565_byteorder = rgb565_byteorder
        self.version = None
        self.closed = False
        self.last_command = None
        self.last_frame_header = None
        self.trace_callback=trace_callback
        if hasattr(transport, "reset_input_buffer"):
            transport.reset_input_buffer()

    def close(self):
        if not self.closed:
            self.closed = True
            self.serial.close()

    def _trace(self,stage,**values):
        if self.trace_callback is not None:
            self.trace_callback(dict(stage=stage,command=self.last_command,
                frame_header=self.last_frame_header,monotonic=time.monotonic(),**values))

    def _write_all(self, data, deadline):
        offset = 0
        while offset < len(data):
            if time.monotonic() >= deadline:
                raise ProtocolError("USB 写入超时；连接已关闭，请重新插拔模块再连接。")
            count = self.serial.write(data[offset:])
            if count:
                offset += count
            else:
                time.sleep(0.005)

    def _read_exact(self, size, deadline):
        chunks = []
        remaining = size
        while remaining:
            if time.monotonic() >= deadline:
                partial = b"".join(chunks)
                error = ProtocolError(
                    "USB 接收超时：命令 {}，已收 {} / {} 字节，还缺 {} 字节；"
                    "已收数据前 32 字节（十六进制）：{}。最近帧头：{}。连接已关闭。".format(
                        "0x{:02X}".format(self.last_command) if self.last_command is not None else "unknown",
                        len(partial), size, remaining, partial[:32].hex() or "无",
                        self.last_frame_header))
                error.command = self.last_command
                error.expected_bytes = size
                error.received_bytes = len(partial)
                error.partial_data = partial
                raise error
            chunk = self.serial.read(remaining)
            if chunk:
                chunks.append(chunk)
                remaining -= len(chunk)
                if remaining:
                    self._trace('read_partial',expected_bytes=size,received_bytes=size-remaining)
            else:
                time.sleep(0.005)
        return b"".join(chunks)

    def _request(self, command, size=0, payload=None, timeout=None):
        if self.closed:
            raise ProtocolError("连接已关闭。请重新插拔 OpenART，并重新运行连接单元格。")
        if payload is not None and len(payload)!=size:
            raise ValueError("payload length does not match command length")
        self.last_command = command
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        try:
            self._trace('request_write',expected_bytes=size)
            self._write_all(struct.pack("<BBI", 0x30, command, size), deadline)
            if payload is not None:
                if len(payload) != size:
                    raise ValueError("payload length does not match command length")
                self._write_all(payload, deadline)
                return None
            if command & 0x80:
                self._trace('request_read',expected_bytes=size)
                result=self._read_exact(size, deadline)
                self._trace('request_complete',expected_bytes=size,received_bytes=len(result))
                return result
            return None
        except BaseException as exc:
            # An incomplete frame or command would desynchronize future requests.
            try:self._trace('request_error',error_type=type(exc).__name__,message=str(exc)[:200])
            finally:self.close()
            raise

    def handshake(self):
        version = struct.unpack("<III", self._request(0x80, 12))
        if not (0 < version[0] < 100 and version[1] < 1000 and version[2] < 1000):
            self.close()
            raise ProtocolError(
                "固件版本响应异常：{}。可能是上次未完成传输的残留响应或协议不匹配；"
                "请给 OpenART 模块重新上电后再连接。".format(version))
        self.version = version
        return version

    def _require_handshake(self):
        if self.version is None:
            raise ProtocolError("必须先成功执行 handshake()，再运行摄像头命令。")

    def script_running(self):
        self._require_handshake()
        return bool(struct.unpack("<I", self._request(0x87, 4))[0])

    def stop_script(self):
        self._require_handshake()
        self._request(0x06)

    def exec_script(self, source):
        self._require_handshake()
        payload = source.encode("utf-8")
        self._request(0x05, len(payload), payload=payload)

    def enable_framebuffer(self, enabled):
        self._require_handshake()
        self._request(0x0D, 4, payload=struct.pack("<I", int(bool(enabled))))

    def read_console(self):
        self._require_handshake()
        size = struct.unpack("<I", self._request(0x8E, 4))[0]
        if size > 1024 * 1024:
            self.close()
            raise ProtocolError("日志长度异常。请核对固件协议。")
        if not size:
            return ""
        return self._request(0x8F, size, timeout=self.frame_timeout).decode("utf-8", errors="replace")

    def read_frame(self):
        self._require_handshake()
        width, height, kind = struct.unpack("<III", self._request(0x81, 12))
        self.last_frame_header = (width, height, kind)
        if width == 0:
            return None
        size = kind if kind > 2 else width * height * kind
        if not (0 < width <= 1920 and 0 < height <= 1200 and kind > 0
                and 0 < size <= 8 * 1024 * 1024):
            self.close()
            raise ProtocolError("帧头异常：{}。请重新插拔模块并核对固件协议。".format((width, height, kind)))
        payload = self._request(0x82, size, timeout=self.frame_timeout)
        return decode_frame(width, height, kind, payload, self.rgb565_byteorder)
