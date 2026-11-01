"""Auxiliary ARM planar pose composition of PL displacement and PL yaw.

Fixed axes at stationary calibration: +X vehicle right, +Y vehicle forward.
Positive yaw is anticlockwise viewed from above. This is dead reckoning,
not absolute positioning or a new FPGA computation.
"""
import math
import uuid


def wrap(value):
    return (value + 180.0) % 360.0 - 180.0


class SessionPose:
    def __init__(self):
        self.epoch = uuid.uuid4().hex[:12]
        self.x = self.y = self.heading = 0.0
        self.continuous = True
        self.gaps = 0

    def mark_gap(self):
        # PL rebases after an invalid interval: missing movement is unknown.
        self.continuous = False
        self.gaps += 1

    def update(self, result):
        yaw = result['heading_deg']
        dx, dy = result['dx_body_m'], result['dy_body_m']
        if not all(math.isfinite(v) for v in (yaw, dx, dy)):
            raise ValueError('Non-finite PL pose input')
        midpoint = math.radians(self.heading + wrap(yaw - self.heading) / 2.0)
        self.x += math.cos(midpoint) * dx - math.sin(midpoint) * dy
        self.y += math.sin(midpoint) * dx + math.cos(midpoint) * dy
        self.heading = yaw
        return dict(x_m=self.x, y_m=self.y, pose_valid=self.continuous,
                    pose_epoch=self.epoch, pose_gap_count=self.gaps,
                    pose_source='ARM midpoint composition of PL dx/dy and PL yaw',
                    pose_origin='stationary calibration; persists across capture runs')
