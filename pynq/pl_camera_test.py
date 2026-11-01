"""OpenART connection/frame utilities for live_board; no overlay reload or software HUD drawing."""
import time
from pathlib import Path

import numpy as np
from openart_usb_fast import OpenARTUSB
from usb_cdc_only_test import connect_cdc_only
from camera_quality import correct_camera,pack_background


def connect_camera(wait_seconds=90,progress=None):
    """Connect one verified OpenART; a safely unmounted existing device is allowed."""
    from serial.tools import list_ports
    from usb_cdc_only_test import TARGET_ID, read_attribute
    if progress is not None:progress(dict(stage='cdc_setup',monotonic=time.monotonic()))
    device_name = connect_cdc_only(wait_seconds=wait_seconds)
    device = Path('/sys/bus/usb/devices')/device_name
    number = read_attribute(device/'devnum')
    def same_device():
        try:
            return (read_attribute(device/'idVendor'), read_attribute(device/'idProduct')) == TARGET_ID and read_attribute(device/'devnum') == number
        except OSError:
            return False
    deadline = time.monotonic() + 8
    matches = []
    while time.monotonic() < deadline:
        matches = [p.device for p in list_ports.comports()
                   if (p.vid, p.pid) == (0x1209, 0xABD1)]
        if not same_device():
            raise RuntimeError('OpenART 在等待串口时已断开或重新枚举；请重新连接。')
        if matches:
            break
        time.sleep(0.1)
    if len(matches) != 1:
        raise RuntimeError('Expected one verified OpenART USB port; found: {}'.format(matches))
    time.sleep(2)
    current = [p.device for p in list_ports.comports()
               if (p.vid, p.pid) == (0x1209, 0xABD1)]
    if matches != current or not same_device():
        raise RuntimeError('OpenART USB disconnected during startup. Keep the output; check power/connection before replugging.')
    if progress is not None:progress(dict(stage='serial_open',port=matches[0],device=device_name,
        devnum=number,monotonic=time.monotonic()))
    cam = OpenARTUSB(matches[0], timeout=5.0, frame_timeout=10.0, decoder='auto',trace_callback=progress)
    try:
        version = cam.handshake()
    except BaseException:
        cam.close()
        raise
    print('CAMERA_CONNECTED:', matches[0], 'firmware=', version,
          'decoder=', cam.decoder.backend, flush=True)
    return cam


def start_preview(cam):
    """Run the previously used QVGA script in RAM and verify one decoded frame."""
    if cam is None or cam.closed:
        raise RuntimeError('Run connect_camera() successfully first.')
    if tuple(cam.version or ()) != (4,5,2):
        raise RuntimeError('This camera brightness profile was verified only on OpenART firmware 4.5.2.')
    print('CAMERA_SCRIPT_BEGIN', flush=True)
    cam.enable_framebuffer(False)
    cam.stop_script()
    deadline = time.monotonic() + 3
    while cam.script_running():
        if time.monotonic() >= deadline:
            raise RuntimeError('The previous OpenART script has not stopped.')
        time.sleep(0.05)
    previous_log = cam.read_console()
    if previous_log:
        print('PREVIOUS_CAMERA_LOG:', previous_log[-2000:], flush=True)
    cam.enable_framebuffer(True)
    cam.exec_script(Path(__file__).with_name('camera_preview.py').read_text(encoding='utf-8'))
    time.sleep(2.2)
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        frame = cam.read_frame()
        if frame is not None:
            print('CAMERA_FRAME_OK:', tuple(frame.shape), flush=True)
            return frame
        time.sleep(0.02)
    print('CAMERA_LOG:', cam.read_console()[-4000:], flush=True)
    raise RuntimeError('No camera frame received. Keep the complete output for diagnosis.')


def copy_camera_frame_reference(output, bgr):
    """Copy BGR into 640x480, preserving aspect ratio; never draw a software HUD."""
    if output.shape not in ((480,640,3),(480,640,4)) or output.dtype != np.uint8:
        raise ValueError('Keep the HDMI mode at 640x480, BGR or BGRX.')
    if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
        raise ValueError('Camera data must be uint8 BGR with three channels.')
    height, width = bgr.shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError('Camera image is empty.')
    bgr=correct_camera(bgr)
    if (height, width) == (480, 640):
        pack_background(output,bgr)
        return
    try:
        import cv2
    except ImportError:
        # Pillow works in RGB; convert back to BGR after resizing.
        from PIL import Image
        scale = min(640 / width, 480 / height)
        w, h = max(1, round(width * scale)), max(1, round(height * scale))
        rgb = Image.fromarray(bgr[:, :, ::-1])
        resampling = Image.Resampling.BILINEAR if hasattr(Image, 'Resampling') else Image.BILINEAR
        resized = np.asarray(rgb.resize((w, h), resampling))[:, :, ::-1]
    else:
        scale = min(640 / width, 480 / height)
        w, h = max(1, round(width * scale)), max(1, round(height * scale))
        resized = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_LINEAR)
    canvas=np.zeros((480,640,3),dtype=np.uint8)
    x, y = (640 - w) // 2, (480 - h) // 2
    canvas[y:y+h,x:x+w]=resized
    pack_background(output,canvas)


def copy_camera_frame(output,bgr):
    """Use cached BGRX scaling when OpenCV is available; retain reference fallback."""
    if output.shape==(480,640,4):
        try:
            from video_renderer import renderer
        except ImportError:
            pass
        else:
            renderer.copy(output,bgr,correct_camera)
            return
    copy_camera_frame_reference(output,bgr)


def close_camera(cam):
    """Stop only the temporary capture script; HDMI retains the last submitted frame."""
    if cam is None or cam.closed:
        return
    if getattr(cam,'supports_deadline',False):
        cam.shutdown()
        print('CAMERA_WORKER_CLOSED',flush=True)
        return
    try:
        cam.enable_framebuffer(False)
        cam.stop_script()
    finally:
        cam.close()
    print('CAMERA_CLOSED', flush=True)
