"""AXI-Lite HUD2 driver. Formats numbers; all glyph/pixel drawing remains in PL.

No MMIO accesses or device connections occur on import. Samples are coalesced
while a frame-boundary commit is pending; an unchanged sample is never replayed
as fresh telemetry. The PL has its own one-second no-update watchdog.
"""
from decimal import Decimal, ROUND_HALF_UP
import math
from telemetry_reference import VEHICLE_INITIAL,validate_reference

HUD_PATH = 'video/hdmi_out/pl_hud_live_0'
HUD_ADDRESS = 0x43C80000
VERSION = 0x48554432


def decimal_value(value):
    result = Decimal(str(value))
    if not result.is_finite():
        raise ValueError('Non-finite telemetry value')
    return result


def packed_bcd(value, places, digits, signed=False):
    value = decimal_value(value)
    integer = int((value * (Decimal(10) ** places)).quantize(Decimal(1), rounding=ROUND_HALF_UP))
    negative = integer < 0
    if negative and not signed:
        raise ValueError('Negative unsigned telemetry value')
    magnitude = abs(integer)
    if magnitude >= 10 ** digits:
        raise ValueError('Telemetry value exceeds HUD display range')
    packed = 0
    for index, char in enumerate(reversed(str(magnitude).zfill(digits))):
        packed |= int(char) << (4 * index)
    if signed and negative:
        packed |= 0x80000000
    return packed


def encode_sample(frame, grid_size_m=None, self_test=False):
    metric = grid_size_m is not None
    if metric and (not math.isfinite(grid_size_m) or grid_size_m <= 0):
        raise ValueError('Grid size must be confirmed, finite and positive')
    sequence = frame['sequence']
    if isinstance(sequence, bool) or not isinstance(sequence, int) or not 0 <= sequence <= 255:
        raise ValueError('Telemetry sequence must be an 8-bit integer')
    if not frame.get('valid') or frame.get('flags') != 0:
        return [0, 0, 0, 0, 0, 2 if metric else 0, sequence]
    speed = decimal_value(frame['speed_grid_s'])
    distance = decimal_value(frame['distance_grid'])
    x, y = decimal_value(frame['x_grid']), decimal_value(frame['y_grid'])
    if metric:
        size = decimal_value(grid_size_m)
        speed *= size * Decimal('3.6')
        distance *= size
        x *= size
        y *= size
    flags = 1 | (2 if metric else 0) | (4 if self_test else 0)
    return [packed_bcd(speed, 3, 6), packed_bcd(distance, 3, 8),
            packed_bcd(x, 3, 5, True), packed_bcd(y, 3, 5, True),
            packed_bcd(frame['heading_deg'], 2, 5, True), flags, sequence]


class HudRegisters:
    def __init__(self, mmio, grid_size_m=None, reference_frame=VEHICLE_INITIAL):
        if grid_size_m is not None and (not math.isfinite(grid_size_m) or grid_size_m <= 0):
            raise ValueError('Invalid grid size')
        self.mmio = mmio
        self.grid_size_m = grid_size_m
        self.reference_frame = validate_reference(reference_frame)
        if int(mmio.read(0x00)) != VERSION:
            raise RuntimeError('HUD2 version mismatch; use this package bit/HWH together')
        self.desired = None
        self.last_sample_key = None
        self.last_submitted_tag = None
        self.last_acked_tag = None
        self.last_source_sequence = None
        self.last_format_error = None
        self.stats = dict(submitted=0, acknowledged=0, busy_polls=0, coalesced=0,
                          invalid_inputs=0, format_errors=0)

    def offer(self, frame, self_test=False):
        key = (frame.get('received_monotonic'), frame.get('sequence'), frame.get('flags'), bool(self_test))
        if key == self.last_sample_key:
            return False
        self.last_sample_key = key
        if self.desired is not None:
            self.stats['coalesced'] += 1
        try:
            if not self_test and self.reference_frame != VEHICLE_INITIAL and frame.get('reference_frame') != self.reference_frame:
                raise ValueError('Telemetry reference does not match HUD reference')
            words = encode_sample(frame, self.grid_size_m, self_test)
        except (ValueError, KeyError, TypeError, ArithmeticError) as exc:
            self.stats['format_errors'] += 1
            self.last_format_error = str(exc)
            words = [0, 0, 0, 0, 0, 0, 0]
        if not words[5] & 1:
            self.stats['invalid_inputs'] += 1
        self.desired = words
        return True

    def invalidate(self):
        self.desired = [0, 0, 0, 0, 0, 0, 0]

    def pump(self):
        """Non-blocking: never wait for a video frame inside the UART worker."""
        if self.desired is None and self.last_submitted_tag==self.last_acked_tag:
            return False
        status = int(self.mmio.read(0x08))
        applied = int(self.mmio.read(0x0C))
        if (self.last_submitted_tag is not None and applied == self.last_submitted_tag
                and applied != self.last_acked_tag):
            self.stats['acknowledged'] += 1
            self.last_acked_tag = applied
        if self.desired is None:
            return False
        if status & 1:
            self.stats['busy_polls'] += 1
            return False
        words = self.desired
        tag = (applied + 1) & 0xFFFFFFFF
        for index, word in enumerate(words):
            self.mmio.write(0x10 + index * 4, int(word))
        self.mmio.write(0x04, tag)
        self.last_submitted_tag = tag
        self.last_source_sequence = words[6]
        self.stats['submitted'] += 1
        self.desired = None
        return True

    def status(self):
        status = int(self.mmio.read(0x08))
        return dict(pending=bool(status & 1), timed_out=bool(status & 2),
                    active_valid=bool(status & 4), applied_tag=int(self.mmio.read(0x0C)),
                    active_source_sequence=int(self.mmio.read(0x2C)),
                    frame_counter=int(self.mmio.read(0x30)))
