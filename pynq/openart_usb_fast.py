"""Packet acquisition separated from decoding, using the verified USBDBG client."""
import io
import struct
import time
from dataclasses import dataclass

import numpy as np
from openart_usb import OpenARTUSB as _Camera, ProtocolError, decode_frame

__version__ = '1.0.0'


@dataclass(frozen=True)
class FramePacket:
    sequence: int
    width: int
    height: int
    kind: int
    payload: bytes
    requested_at: float
    received_at: float
    header_ms: float
    transfer_ms: float


class Decoder:
    def __init__(self, backend='auto'):
        if backend not in ('auto', 'opencv', 'pillow'):
            raise ValueError('decoder must be auto, opencv or pillow')
        self.cv2 = None
        if backend != 'pillow':
            try:
                import cv2
                self.cv2 = cv2
            except ImportError:
                if backend == 'opencv':
                    raise
        self.backend = 'opencv' if self.cv2 is not None else 'pillow'
        if self.cv2 is None:
            from PIL import Image
            self.image = Image

    def decode(self, packet, rgb565_byteorder='big'):
        if packet.kind <= 2:
            return decode_frame(packet.width, packet.height, packet.kind,
                                packet.payload, rgb565_byteorder)
        if self.cv2 is not None:
            result = self.cv2.imdecode(np.frombuffer(packet.payload, np.uint8), self.cv2.IMREAD_COLOR)
            if result is None:
                raise ProtocolError('收到完整帧，但 JPEG 解码失败。')
        else:
            with self.image.open(io.BytesIO(packet.payload)) as image:
                result = np.asarray(image.convert('RGB'))[:, :, ::-1].copy()
        if result.shape != (packet.height, packet.width, 3):
            raise ProtocolError('JPEG 尺寸与 USB 帧头不一致。')
        return result


class OpenARTUSB(_Camera):
    """Only the acquisition worker may use USB while the pipeline is running."""
    def __init__(self, *args, **kwargs):
        self.decoder = Decoder(kwargs.pop('decoder', 'auto'))
        super().__init__(*args, **kwargs)
        self.frame_sequence = 0
        self.empty_headers = 0
        self.last_packet = None

    def read_packet(self):
        self._require_handshake()
        started = time.monotonic()
        width, height, kind = struct.unpack('<III', self._request(0x81, 12))
        header_done = time.monotonic()
        self.last_frame_header = (width, height, kind)
        if width == 0:
            self.empty_headers += 1
            return None
        size = kind if kind > 2 else width * height * kind
        if not (0 < width <= 1920 and 0 < height <= 1200 and kind > 0
                and 0 < size <= 8 * 1024 * 1024):
            self.close()
            raise ProtocolError('帧头异常：{}。请给模块重新上电后再连接。'.format((width, height, kind)))
        payload = self._request(0x82, size, timeout=self.frame_timeout)
        received = time.monotonic()
        self.frame_sequence += 1
        packet = FramePacket(self.frame_sequence, width, height, kind, payload,
            started, received, (header_done - started) * 1000, (received - header_done) * 1000)
        self.last_packet = packet
        return packet

    def read_frame(self):
        packet = self.read_packet()
        return None if packet is None else self.decoder.decode(packet, self.rgb565_byteorder)
