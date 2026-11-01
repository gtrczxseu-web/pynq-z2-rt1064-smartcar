"""White-reference correction and contiguous BGRX output, without image wrapping."""
from pathlib import Path
import hashlib
import json
import math
from numbers import Real
import numpy as np
try:
    import cv2 as _cv2
except ImportError:
    _cv2=None

OUTPUT_SHIFT_PIXELS = 0
_active = None


class CameraQuality:
    def __init__(self, settings):
        gains = settings.get('gains_bgr', [1, 1, 1])
        if not isinstance(gains,(list,tuple)) or len(gains) != 3 or any(isinstance(v, bool) or not isinstance(v,Real) or not math.isfinite(v) or not .35 <= v <= 3.2 for v in gains):
            raise ValueError('White-reference gains must be finite and within 0.35..3.2')
        if settings.get('schema') != 1 or settings.get('output_bpp') != 32 or settings.get('shift_pixels') != OUTPUT_SHIFT_PIXELS:
            raise ValueError('Unsupported display calibration profile')
        self.settings = dict(settings)
        self.gains = np.asarray(gains, dtype=np.float32)
        values = np.arange(256, dtype=np.float32)[:, None] * self.gains[None, :]
        # Preserve midtones, gently compress highlights. No clipped sensor detail
        # can be reconstructed by this curve.
        excess = np.maximum(values-200, 0)
        values = np.where(values <= 200, values, 200+45*(1-np.exp(-excess/60)))
        self.lut = np.clip(np.rint(values), 0, 255).astype(np.uint8)

    def apply(self, bgr):
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('Expected uint8 BGR image')
        if _cv2 is not None:
            return _cv2.LUT(bgr,self.lut.reshape(256,1,3))
        return self.lut[bgr, np.arange(3)]


def calibrate_neutral(frames, roi=(100, 60, 140, 120), target=180):
    """Only call after the user places a neutral white/grey reference in ROI."""
    if not frames or not math.isfinite(target) or not 100 <= target <= 210:
        raise ValueError('Reference frames and a sensible white target are required')
    x,y,w,h = roi
    if any(isinstance(v,bool) or not isinstance(v,int) for v in roi) or min(x,y) < 0 or min(w,h) < 16:
        raise ValueError('Reference ROI is invalid')
    samples=[]
    for frame in frames:
        if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3 or x+w > frame.shape[1] or y+h > frame.shape[0]:
            raise ValueError('Reference ROI does not fit a uint8 BGR frame')
        samples.append(frame[y:y+h,x:x+w].reshape(-1,3))
    pixels=np.concatenate(samples,axis=0)
    saturated=float(np.mean(pixels.max(axis=1)>=245))
    valid=(pixels.min(axis=1)>=20)&(pixels.max(axis=1)<245)
    if saturated > .05 or float(valid.mean()) < .85:
        raise ValueError('White reference is too dark or saturated; adjust the camera first')
    pixels=pixels[valid]
    median=np.median(pixels,axis=0)
    spread=(np.percentile(pixels,90,axis=0)-np.percentile(pixels,10,axis=0))/median
    if np.max(spread) > .65:
        raise ValueError('Reference ROI is not sufficiently uniform')
    gains=target/median
    if np.min(gains) < .35 or np.max(gains) > 3.2:
        raise ValueError('Reference would require excessive colour gain; adjust lighting first')
    return dict(schema=1,output_bpp=32,shift_pixels=OUTPUT_SHIFT_PIXELS,
        gains_bgr=gains.tolist(),reference_median_bgr=median.tolist(),reference_roi=list(roi),
        target_white=target,reference_saturation_pct=round(saturated*100,4),
        native_brightness=255,firmware=[4,5,2],
        note='Calibrated using the user-supplied white paper under current lighting; redo when lighting changes.')


def configure_quality(folder):
    global _active
    _active=None
    folder=Path(folder)
    path=folder/'camera_calibration.json'
    if not path.is_file():
        _active=None
        return None
    settings=json.loads(path.read_text(encoding='utf-8'))
    expected=settings.get('hardware_sha256')
    if not expected or hashlib.sha256((folder/'hud_live.bit').read_bytes()).hexdigest()!=expected:
        raise ValueError('Display compensation belongs to a different FPGA bitstream')
    _active=CameraQuality(settings)
    return _active.settings


def correct_camera(bgr):
    return bgr if _active is None else _active.apply(bgr)


def pack_background(output, bgr):
    """Keep all geometry intact and copy contiguous 32-bit pixels to DMA memory."""
    if output.ndim!=3 or bgr.ndim!=3 or output.dtype!=np.uint8 or output.shape[:2]!=(480,640) or bgr.shape[1:]!=(640,3) or bgr.dtype!=np.uint8 or bgr.shape[0] not in (1,480):
        raise ValueError('Expected a 640x480 frame or one-row background')
    if output.shape[2]==3:
        output[:]=bgr
    elif output.shape[2]==4:
        if _cv2 is not None:
            packed=_cv2.cvtColor(bgr,_cv2.COLOR_BGR2BGRA)
            packed[:,:,3]=0
        else:
            packed=np.zeros((bgr.shape[0],640,4),dtype=np.uint8)
            packed[:,:,:3]=bgr
        np.copyto(output,packed)
    else:
        raise ValueError('Expected 24-bit BGR or 32-bit BGRX output')
