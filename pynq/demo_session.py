"""Repeatable PYNQ demo lifecycle. Import is free of hardware side effects."""
from datetime import datetime
from pathlib import Path
import platform
import uuid
import xml.etree.ElementTree as ET
import zipfile

from live_board import verify_package, initialize_mmio, hdmi_start, run_live, checkpoint
from live_hud import HudRegisters, HUD_PATH, HUD_ADDRESS
from telemetry_reference import VEHICLE_INITIAL,validate_reference


class DemoSession:
    def __init__(self, folder='.', grid_size_m=0.20, reference_frame=VEHICLE_INITIAL):
        if grid_size_m != 0.20:
            raise ValueError('This release uses the confirmed competition grid: 0.20 m.')
        self.folder = Path(folder).resolve()
        self.grid_size_m = grid_size_m
        self.reference_frame = validate_reference(reference_frame)
        self.overlay = self.hdmi = self.cam = self.hud = self.uart = None
        self.ready = self.running = self.attempted = self.closed = False
        self.run_number = 0
        self.last_result = None
        self.tracker=None
        self.idle_traces=[]
        self.log_dir = self.folder/'logs'/('session_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])

    def setup(self):
        if self.closed:
            raise RuntimeError('Session closed. Restart the kernel before a fresh setup.')
        if self.ready:
            checkpoint('SESSION_ALREADY_READY', action='continue camera connection or capture; no overlay reload')
            return
        if self.attempted:
            raise RuntimeError('Setup was interrupted/failed; save the output and restart the kernel before retrying.')
        if platform.system() != 'Linux' or platform.machine().lower() != 'armv7l':
            raise RuntimeError('Run this Notebook on the PYNQ-Z2 Jupyter server.')
        verify_package(self.folder)
        from camera_quality import configure_quality
        quality=configure_quality(self.folder)
        from pynq import Overlay, Clocks
        from live_board import require_idle_video_dma
        require_idle_video_dma()
        from pynq.lib.video import VideoMode  # register the standard video drivers
        bit = self.folder/'hud_live.bit'
        modules=list(ET.parse(bit.with_suffix('.hwh')).getroot().iter('MODULE'))
        assert [m.get('FULLNAME') for m in modules if m.get('VLNV')=='local:user:pl_sensor_hud:1.0']==['/'+HUD_PATH]
        overlay=Overlay(str(bit),download=False)
        for name,address in {HUD_PATH:HUD_ADDRESS,'axi_uartlite_0':0x42c00000,'rx_level_0':0x41260000}.items():
            assert overlay.ip_dict[name]['phys_addr']==address,name
        self.attempted=True
        self.overlay=overlay
        checkpoint('DEMO_DOWNLOAD_BEGIN')
        overlay.download()
        assert abs(Clocks.fclk0_mhz-100)<0.1 and abs(Clocks.fclk1_mhz-142.857143)<0.2
        assert Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()=='operating'
        checkpoint('DEMO_DOWNLOAD_OK',fclk0_mhz=Clocks.fclk0_mhz,fclk1_mhz=Clocks.fclk1_mhz)
        self.uart, _, mmio=initialize_mmio(overlay)
        import sensor_driver
        sensor_driver.configure(mmio)
        self.hud=HudRegisters(mmio,grid_size_m=self.grid_size_m,reference_frame=self.reference_frame)
        self.hdmi=hdmi_start(overlay)
        self.ready=True
        checkpoint('DEMO_READY',grid_size_m=self.grid_size_m,speed_unit='km/h',distance_unit='m',
                   heading_unit='relative degrees',log_dir=str(self.log_dir))
        checkpoint('CAMERA_QUALITY_READY',output_bpp=32,shift_pixels=0,calibrated=quality is not None)
        checkpoint('HUD_REFERENCE_READY',reference_frame=self.reference_frame)

    def _require_idle(self):
        if not self.ready or self.closed:
            raise RuntimeError('Complete setup first; a closed session requires a new kernel.')
        if self.running:
            raise RuntimeError('Capture is still running. Use Kernel -> Interrupt and wait for the result first.')

    def _stop_tracking(self):
        if self.tracker is not None:
            self.tracker.stop()
            self.idle_traces.append(self.tracker.snapshot())
            if self.tracker.error:
                checkpoint('IDLE_TRACKER_ERROR',error=self.tracker.error)
            self.tracker=None

    def _save_idle_traces(self):
        if self.idle_traces:
            import json
            self.log_dir.mkdir(parents=True,exist_ok=True)
            (self.log_dir/'idle_tracking.json').write_text(json.dumps(self.idle_traces,indent=2),encoding='utf8')

    def _start_tracking(self):
        import sensor_driver
        if self.tracker is not None or not self.ready or self.closed or not sensor_driver.processor.calibrated:
            return
        from live_board import LiveWorker
        tracker=LiveWorker(self.uart,self.hud,camera_association=True,hud_enabled=False)
        tracker.start()
        self.tracker=tracker

    def calibrate(self,seconds=3,stationary_limits=None):
        """Caller confirms physical stillness; keep tracking between captures."""
        self._require_idle()
        self._stop_tracking()
        import sensor_driver,json
        result=sensor_driver.calibrate(self.uart,seconds,stationary_limits=stationary_limits)
        self._start_tracking()
        self._save_idle_traces()
        self.log_dir.mkdir(parents=True,exist_ok=True)
        (self.log_dir/('calibration_'+result['pose_epoch']+'.json')).write_text(json.dumps(result,indent=2),encoding='utf8')
        return result

    def self_test(self):
        self._require_idle()
        self._stop_tracking()
        import sensor_driver
        return sensor_driver.register_self_test()

    def connect_camera(self, wait_seconds=90, stop_event=None):
        self._require_idle()
        if self.cam is not None and not self.cam.closed:
            checkpoint('CAMERA_REUSED',action='use capture, do not replug a connected camera')
            return
        from camera_guard import IsolatedCamera
        camera=None
        try:
            camera=IsolatedCamera(wait_seconds=wait_seconds,stop_event=stop_event)
        except BaseException as exc:
            if camera is not None:
                camera.close()
            self.cam=None
            from live_board import save_result
            import time
            save_result(self.log_dir/('camera_connect_error_'+str(time.monotonic_ns())+'.json'),
                dict(error=type(exc).__name__,message=str(exc),diagnostics=getattr(exc,'diagnostics',None)))
            raise
        self.cam=camera
        checkpoint('CAMERA_GUARD_READY',port=camera.port,firmware=camera.version,
                   pid=camera.process.pid,frame_deadline_s=camera.frame_timeout)

    def reconnect_camera(self, wait_seconds=90):
        """Release a stale descriptor without writing USB, then explicitly reconnect."""
        self._require_idle()
        if self.cam is not None:
            try:
                self.cam.close()
            finally:
                self.cam=None
        self.connect_camera(wait_seconds=wait_seconds)

    def disconnect_camera(self):
        self._require_idle()
        if self.cam is not None:
            try:
                from pl_camera_test import close_camera
                close_camera(self.cam)
            finally:
                self.cam.close()
                self.cam=None
        checkpoint('CAMERA_RELEASED',action='unplug OpenART; then connect_camera when ready')

    def capture(self, seconds=30, camera=True, stop_event=None, on_progress=None,frame_overlay=None):
        self._require_idle()
        import sensor_driver
        if not sensor_driver.processor.calibrated:
            raise RuntimeError('Keep the vehicle still and run stationary calibration first')
        if camera and (self.cam is None or self.cam.closed):
            raise RuntimeError('Camera is not ready. Connect it first, or use camera=False for colorbars.')
        self._stop_tracking()
        self.run_number+=1
        self.running=True
        self.last_result=None
        try:
            result=run_live(self.hdmi,self.uart,self.hud,cam=self.cam if camera else None,
                seconds=seconds,log_path=str(self.log_dir/('{:03d}_{}.json'.format(self.run_number,'camera' if camera else 'colorbars'))),
                camera_fallback=True,stop_event=stop_event,on_progress=on_progress,
                on_receiver_stopped=self._start_tracking,frame_overlay=frame_overlay)
            self.last_result=result
            if camera and (result['video_error'] is not None or result['camera_degraded'] or result.get('interrupted')
                           or result.get('camera_cancelled')):
                if self.cam is not None:
                    self.cam.close()
                self.cam=None
                checkpoint('CAMERA_HANDLE_RELEASED', action='run session.connect_camera() before another camera capture')
            return result
        except RuntimeError as exc:
            if 'MMIO' in str(exc):
                self.ready=False  # an unjoined hardware thread must not be reused
            raise
        finally:
            self.running=False
            if self.ready:
                self._start_tracking()
            self._save_idle_traces()

    def timeout_status(self):
        self._require_idle()
        state=self.hud.status()
        checkpoint('DEMO_HUD_STATUS',**state)
        return state

    def export_logs(self):
        if self.running:
            raise RuntimeError('Wait for capture to stop before exporting logs.')
        if not self.log_dir.is_dir():
            raise RuntimeError('No capture logs exist yet.')
        self._save_idle_traces()
        archive=self.log_dir.with_suffix('.zip')
        with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as bundle:
            for path in sorted(self.log_dir.iterdir()):
                if path.is_file() and path.suffix in ('.json','.jsonl','.csv'):
                    bundle.write(path,self.log_dir.name+'/'+path.name)
        checkpoint('LOG_ARCHIVE_READY',path=str(archive))
        return archive

    def close(self):
        if self.running:
            raise RuntimeError('Interrupt capture and wait until it stops before cleanup.')
        if self.closed:
            return
        self._stop_tracking()
        self._save_idle_traces()
        errors=[]
        try:
            if self.cam is not None:
                from pl_camera_test import close_camera
                close_camera(self.cam)
        except Exception as exc:
            errors.append(str(exc))
        finally:
            if self.cam is not None:
                self.cam.close()
            self.cam=None
            if self.hdmi is not None:
                try:
                    self.hdmi.stop()
                except Exception as exc:
                    errors.append(str(exc))
            self.ready=False
            self.closed=True
        checkpoint('DEMO_CLOSED',cleanup_errors=errors,logs=str(self.log_dir))
