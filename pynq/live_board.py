"""Staged board tests: actual UART telemetry -> HUD2 registers -> PL pixels."""
import hashlib
import json
import math
import threading
import sys
import time
import csv
from collections import deque
from pathlib import Path

from live_hud import HudRegisters, HUD_ADDRESS, HUD_PATH
from vehicle_telemetry import UartTelemetry
from camera_quality import pack_background
from telemetry_reference import VEHICLE_INITIAL,reference_description
from sample_alignment import SampleHistory,alignment_summary


def checkpoint(stage, **values):
    print(stage, json.dumps(values, ensure_ascii=False), flush=True)


def save_result(path, result):
    if path is not None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def verify_package(folder='.'):
    folder = Path(folder)
    manifest = json.loads((folder / 'manifest.json').read_text(encoding='utf-8'))
    for name, expected in manifest['upload_files'].items():
        path = folder / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected['sha256']:
            raise RuntimeError('文件缺失或校验失败，请重新上传：' + name)
    checkpoint('PACKAGE_FILES_OK', files=len(manifest['upload_files']))
    return manifest


def require_idle_video_dma():
    """Read known resident DMA status before any FPGA reconfiguration."""
    from pynq import PL, MMIO
    checked=0
    for name,description in (PL.ip_dict or {}).items():
        if description.get('type') != 'xilinx.com:ip:axi_vdma:6.3':
            continue
        mmio=MMIO(description['phys_addr'],0x10000)
        for control,status in ((0x00,0x04),(0x30,0x34)):
            if mmio.read(control)&1 or not(mmio.read(status)&1):
                raise RuntimeError('已有视频DMA仍在运行，禁止直接重新下载FPGA。先在原Notebook正常结束；若内核已异常退出，请正常重启PYNQ后再运行A/B。')
        checked+=1
    checkpoint('VIDEO_STARTUP_GUARD_OK',known_dma_channels=checked*2)


def initialize_mmio(overlay):
    from pynq import MMIO
    expected = {'axi_uartlite_0': 0x42C00000, 'rx_level_0': 0x41260000, HUD_PATH: HUD_ADDRESS}
    for name, address in expected.items():
        assert overlay.ip_dict[name]['phys_addr'] == address, 'HWH 地址不匹配：' + name
    uart = MMIO(expected['axi_uartlite_0'], 0x10000)
    gpio = MMIO(expected['rx_level_0'], 0x10000)
    checkpoint('UART_STATUS_READ_BEGIN')
    old_status = int(uart.read(0x08))
    uart.write(0x0C, 0x03)
    uart.read(0x08)  # clear pre-test error latches, not part of the capture window
    checkpoint('UART_READY', previous_status=hex(old_status), rx_level=int(gpio.read(0)) & 1)
    checkpoint('HUD_VERSION_READ_BEGIN')
    hud_mmio = MMIO(expected[HUD_PATH], 0x1000)
    HudRegisters(hud_mmio)  # version read only
    checkpoint('HUD_VERSION_OK', version=hex(int(hud_mmio.read(0))), address=hex(HUD_ADDRESS))
    return uart, gpio, hud_mmio


def hdmi_start(overlay):
    from pynq.lib.video import VideoMode,PixelFormat,PIXEL_BGR
    checkpoint('HDMI_BIND_BEGIN')
    hdmi = overlay.video.hdmi_out
    hdmi.configure(VideoMode(640,480,32),pixelformat=PixelFormat(32,PIXEL_BGR.in_color,PIXEL_BGR.out_color))
    hdmi.start()
    checkpoint('HDMI_START_OK')
    return hdmi


def colorbars():
    import numpy as np
    colors = np.array([(255,255,255),(0,255,255),(255,255,0),(0,255,0),
                       (255,0,255),(0,0,255),(255,0,0),(0,0,0)], dtype=np.uint8)
    return np.repeat(colors, 80, axis=0)[None, :, :]


def register_display_test(hdmi, hud, seconds=8, log_path='logs/03_register_test.json'):
    """Explicitly labelled TEST input; this function is never called by run_live."""
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('seconds must be finite and positive')
    background = colorbars()
    begin_frame = hud.status()['frame_counter']
    before = dict(hud.stats)
    started = time.monotonic()
    next_update = started
    checkpoint('HUD_REGISTER_TEST_BEGIN', seconds=seconds, source='explicit TEST input')
    while time.monotonic() - started < seconds:
        now = time.monotonic()
        if now >= next_update:
            hud.offer(dict(sequence=0, flags=0, valid=True, speed_grid_s=1.93,
                           distance_grid=31.141, x_grid=-15.442, y_grid=14.629,
                           heading_deg=-178.35, received_monotonic=now), self_test=True)
            next_update = now + 0.1
        hud.pump()
        output = hdmi.newframe()
        try:
            pack_background(output,background)
        except BaseException:
            output.freebuffer()
            raise
        hdmi.writeframe(output)
    flush_hud(hud)
    state = hud.status()
    delta = {key: hud.stats[key] - before[key] for key in before}
    result = dict(hud=delta, hardware=state, frame_counter_delta=(state['frame_counter']-begin_frame)&0xFFFFFFFF,
                  software_check_pass=delta['acknowledged'] >= 3 and not state['pending'] and not state['timed_out'],
                  screen_values_confirmed=None, source='explicit TEST input', software_hud_drawn=False)
    save_result(log_path, result)
    checkpoint('HUD_REGISTER_TEST_RESULT', **result)
    return result


def flush_hud(hud):
    deadline = time.monotonic() + 0.25
    while time.monotonic() < deadline:
        hud.pump()
        if hud.desired is None and (hud.last_submitted_tag is None or hud.last_acked_tag == hud.last_submitted_tag):
            return
        time.sleep(0.001)


class LiveWorker:
    def __init__(self, uart_mmio, hud, camera_association=False, hud_enabled=True):
        self.receiver = UartTelemetry(uart_mmio, hud.grid_size_m,hud.reference_frame)
        self.hud = hud
        self.camera_association=camera_association
        self.hud_enabled=hud_enabled
        self.history=SampleHistory()
        self.last_paired_sequence=None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.previous_switch_interval = None
        self.error = None
        self.polls = 0
        self.max_gap = 0.0
        self.records = deque(maxlen=2048)
        self.record_drops = 0
        self.poll_gap_samples = deque(maxlen=32)
        self.poll_gap_sample_drops = 0

    def _run(self):
        previous = time.monotonic()
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                with self.lock:
                    self.max_gap = max(self.max_gap, now-previous)
                    if now-previous >= 0.020:
                        if len(self.poll_gap_samples) == self.poll_gap_samples.maxlen:
                            self.poll_gap_sample_drops += 1
                        self.poll_gap_samples.append(dict(monotonic=now, gap_ms=round((now-previous)*1000,3),
                            last_sequence=self.receiver.decoder.last_sequence))
                    frames = self.receiver.poll(max_bytes=512)
                    for frame in frames:
                        self.history.add(frame)
                        if len(self.records) == self.records.maxlen:
                            self.record_drops += 1
                        self.records.append(dict(frame))
                    if self.hud_enabled and frames and not self.camera_association:
                        self.hud.offer(frames[-1], self_test=False)
                    if self.hud_enabled:
                        self.hud.pump()
                    self.polls += 1
                previous = now
                # Native receiver services the FIFO independently. Python only
                # parses the buffered 10 Hz source and commits HUD registers.
                self.stop_event.wait(0.002)
        except Exception as exc:
            with self.lock:
                self.error = '{}: {}'.format(type(exc).__name__, exc)
            self.stop_event.set()

    def start(self):
        if self.thread is not None:
            raise RuntimeError('Worker already started')
        self.receiver.mmio.write(0x0C, 0x03)
        old = int(self.receiver.mmio.read(0x08))
        checkpoint('UART_CAPTURE_HISTORY_CLEARED', previous_status=hex(old))
        self.receiver.start_background()
        checkpoint('UART_NATIVE_RECEIVER_STARTED',buffer_events=65535)
        # RAW33 bursts exceed one UARTLite FIFO. Give the UART worker frequent
        # GIL access while camera/CSV code shares this Python interpreter.
        self.previous_switch_interval = sys.getswitchinterval()
        sys.setswitchinterval(min(self.previous_switch_interval, 0.0005))
        self.thread = threading.Thread(target=self._run, name='live-uart-hud', daemon=True)
        try:
            self.thread.start()
        except BaseException:
            self.receiver.stop_background()
            raise

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(2)
            if self.thread.is_alive():
                raise RuntimeError('MMIO 线程未停止，停止本内核，勿重载电路或继续访问硬件。')
        self.receiver.stop_background()
        if self.previous_switch_interval is not None:
            sys.setswitchinterval(self.previous_switch_interval)
            self.previous_switch_interval = None

    def snapshot(self):
        with self.lock:
            return dict(uart=dict(self.receiver.stats), decoder=dict(self.receiver.decoder.stats),
                        native_receiver=dict(self.receiver.native_metrics),
                        latest=None if self.receiver.latest is None else dict(self.receiver.latest),
                        latest_observed=None if self.receiver.latest_observed is None else dict(self.receiver.latest_observed),
                        hud=dict(self.hud.stats), worker_error=self.error,
                        max_poll_gap_ms=round(self.max_gap*1000, 3), polls=self.polls,
                        uart_diagnostics=list(self.receiver.decoder.diagnostics),
                        uart_diagnostic_drops=self.receiver.decoder.diagnostic_drops,
                        poll_gap_samples=list(self.poll_gap_samples), poll_gap_sample_drops=self.poll_gap_sample_drops,
                        last_format_error=self.hud.last_format_error)

    def drain_records(self):
        with self.lock:
            records = list(self.records)
            self.records.clear()
            return records

    def pair_camera(self, packet):
        with self.lock:
            sample,row=self.history.select(packet.requested_at,packet.received_at,packet.sequence)
            row.update(width=packet.width,height=packet.height,kind=packet.kind,payload_bytes=len(packet.payload),
                       header_ms=packet.header_ms,usb_transfer_ms=packet.transfer_ms)
            if sample is not None:
                self.hud.offer(sample)
                self.hud.pump()
                self.last_paired_sequence=sample['sequence']
            elif row['status']!='startup':
                self.hud.invalidate()
                self.hud.pump()
            return row


def _run_live(hdmi, uart_mmio, hud, cam=None, seconds=30, log_path='logs/05_live_camera.json',
             camera_fallback=False, camera_stall_seconds=3, stop_event=None, on_progress=None,
             on_receiver_stopped=None,frame_overlay=None):
    from pl_camera_test import copy_camera_frame
    native_renderer=getattr(getattr(frame_overlay,'__self__',None),'render_camera',None)
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('seconds must be finite and positive')
    if not math.isfinite(camera_stall_seconds) or camera_stall_seconds <= 0:
        raise ValueError('camera_stall_seconds must be finite and positive')
    if cam is not None and cam.closed:
        raise RuntimeError('摄像头已断开，请清理旧对象后重新连接')
    # Initialise image-library kernels before the FIFO reset / capture window.
    # Cold resize/LUT setup must not hold the GIL while the UART worker is active.
    import numpy as np
    bpp=getattr(getattr(hdmi,'mode',None),'bits_per_pixel',24)
    warm_frame=np.empty((480,640,4 if bpp==32 else 3),dtype=np.uint8)
    if cam is not None:
        copy_camera_frame(warm_frame,np.zeros((240,320,3),dtype=np.uint8))
    else:
        pack_background(warm_frame,colorbars())
    worker = LiveWorker(uart_mmio, hud, camera_association=cam is not None)
    alignment_rows=[]
    before_hud = dict(hud.stats)
    before_hw = hud.status()
    background = colorbars() if cam is None else None
    frames, empty, interrupted, error = 0, 0, False, None
    camera_requested = cam is not None
    camera_frames = 0
    camera_error = None
    camera_cancelled=False
    camera_fault=None
    camera_profile=cam
    camera_source=None if cam is None else getattr(cam,'source_kind','real_openart_usb_legacy')
    first_sample = None
    last_sample = None
    record_count = 0
    csv_file = None
    writer = None
    csv_path = None
    if log_path is not None:
        csv_path = str(Path(log_path).with_suffix('.csv'))
        Path(csv_path).parent.mkdir(parents=True, exist_ok=True)
        csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
        writer = csv.DictWriter(csv_file, fieldnames=['received_monotonic', 'sequence', 'flags',
            'valid', 'speed_grid_s', 'distance_grid', 'x_grid', 'y_grid', 'heading_deg',
            'speed_m_s', 'speed_km_h', 'distance_m', 'x_m', 'y_m',
            'source','pose_source','source_t20','source_t5','source_elapsed_s','source_clock_epoch','source_counter_reset','source_tick_consistent','source_interval_s','decode_interval_s','timestamp_kind','pl_samples','pl_rejected','dx_body_m','dy_body_m','reference_frame','pose_epoch','pose_valid','pose_gap_count','vehicle_x_grid','vehicle_y_grid','vehicle_heading_deg',
            'vehicle_x_m','vehicle_y_m','camera_heading_vehicle_deg'], extrasaction='ignore')
        writer.writeheader()
    def drain():
        nonlocal first_sample, last_sample, record_count
        for record in worker.drain_records():
            if writer is not None:
                writer.writerow(record)
            record_count += 1
            if record['valid']:
                if first_sample is None:
                    first_sample = record
                last_sample = record
    checkpoint('LIVE_HUD_BEGIN', seconds=seconds, camera=cam is not None, source='RT1064 UART telemetry')
    checkpoint('TELEMETRY_REFERENCE',**reference_description(hud.reference_frame))
    print('本轮车板应已上电稳定；HUD 应显示 LIVE。停止更新约 1 秒后 PL 会显示 STALE 和横线。', flush=True)
    start = time.monotonic()
    events = [dict(kind='capture_started',monotonic=start,elapsed_s=0.0,camera=camera_requested)]
    last_camera_frame = start
    next_report = start + 5
    next_ui=start
    try:
        worker.start()
        while time.monotonic()-start < seconds:
            if worker.stop_event.is_set() or (stop_event is not None and stop_event.is_set()):
                break
            drain()
            try:
                if cam is None:
                    bgr=None
                elif getattr(cam,'supports_deadline',False):
                    bgr=cam.read_frame(stop_event=stop_event)
                else:
                    bgr=cam.read_frame()
                if cam is not None:
                    if bgr is not None:
                        last_camera_frame = time.monotonic()
                    elif time.monotonic()-last_camera_frame >= camera_stall_seconds:
                        raise TimeoutError('Camera returned no frame within the configured stall interval')
            except Exception as exc:
                if getattr(cam,'supports_deadline',False):
                    from camera_guard import CameraCancelled
                    if isinstance(exc,CameraCancelled):
                        camera_cancelled=True
                        camera_fault=exc.diagnostics
                        break
                if not camera_fallback:
                    raise
                camera_error = '{}: {}'.format(type(exc).__name__, exc)
                camera_fault=getattr(exc,'diagnostics',None)
                if camera_fault is None and hasattr(cam,'diagnostics'):
                    camera_fault=cam.diagnostics(camera_error)
                try:
                    cam.close()  # close descriptor only; do not write USB after a failure
                except Exception:
                    pass
                cam = None
                with worker.lock:
                    worker.camera_association=False
                background = colorbars()
                event_time = time.monotonic()
                events.append(dict(kind='camera_lost',monotonic=event_time,elapsed_s=round(event_time-start,6),
                                   error=camera_error,diagnostics=camera_fault))
                checkpoint('CAMERA_LOST_USING_COLORBARS', error=camera_error,
                           monotonic=event_time, elapsed_s=round(event_time-start,6),
                           action='UART and PL HUD continue; reconnect camera explicitly after this run')
                bgr = None
            if cam is not None and bgr is None:
                empty += 1
                time.sleep(0.005)
            else:
                output = hdmi.newframe()
                try:
                    if cam is None:
                        pack_background(output,background)
                    else:
                        if native_renderer is None:copy_camera_frame(output, bgr)
                        alignment_rows.append(worker.pair_camera(cam.last_packet))
                    if cam is not None and native_renderer is not None:
                        native_renderer(output,bgr)
                    elif frame_overlay is not None:
                        frame_overlay(output,video_frame=cam is not None)
                except BaseException:
                    output.freebuffer()
                    raise
                hdmi.writeframe(output)
                frames += 1
                if cam is not None:
                    camera_frames += 1
            now = time.monotonic()
            if on_progress is not None and now>=next_ui:
                on_progress(dict(worker.snapshot(),elapsed_s=now-start,camera_frames=camera_frames,
                                 last_alignment=alignment_rows[-1] if alignment_rows else None,
                                 camera_requested=camera_requested,camera_degraded=camera_error is not None,
                                 camera_error=camera_error,camera_source=camera_source))
                next_ui=now+0.5
            if now >= next_report:
                snapshot = worker.snapshot()
                checkpoint('LIVE_HUD_PROGRESS', submitted_video_frames=frames,
                           uart_frames=snapshot['decoder']['frames'], latest=snapshot['latest'],
                           hud_commits=snapshot['hud']['submitted']-before_hud['submitted'],
                           overrun=snapshot['uart']['overrun'])
                next_report = now + 5
                if csv_file is not None:
                    csv_file.flush()
    except KeyboardInterrupt:
        interrupted = True
        event_time = time.monotonic()
        events.append(dict(kind='capture_interrupted',monotonic=event_time,elapsed_s=round(event_time-start,6)))
    except Exception as exc:
        error = '{}: {}'.format(type(exc).__name__, exc)
    finally:
        try:
            worker.stop()
            # Restore session tracking before formatting/writing potentially
            # large alignment logs or Jupyter output. The stopped capture owns
            # no MMIO thread now; the idle tracker never accesses HUD registers.
            if on_receiver_stopped is not None:
                on_receiver_stopped()
        finally:
            try:
                # A stalled MMIO owner may still hold the lock: never wait for it here.
                if worker.thread is None or not worker.thread.is_alive():
                    drain()
            finally:
                if csv_file is not None:
                    csv_file.close()
    flush_hud(hud)
    result = worker.snapshot()
    state = hud.status()
    now = time.monotonic()
    events.append(dict(kind='capture_finished',monotonic=now,elapsed_s=round(now-start,6)))
    result['hud'] = {key: hud.stats[key]-before_hud[key] for key in before_hud}
    result.update(seconds=round(now-start, 3), submitted_video_frames=frames,
                  camera_frames=camera_frames if camera_requested else None,
                  average_camera_fps=round(camera_frames/max(now-start, 1e-9), 3) if camera_requested else None,
                  empty_reads=empty, interrupted=interrupted, video_error=error,
                  hardware=state, frame_counter_delta=(state['frame_counter']-before_hw['frame_counter'])&0xFFFFFFFF,
                  latest_age_s=None if result['latest'] is None else round(now-result['latest']['received_monotonic'], 3),
                  grid_size_m=hud.grid_size_m, screen_values_confirmed=None,
                  hud_data_source='RT1064 UART telemetry', software_hud_drawn=False,
                  note='Submitted camera FPS is not physical HDMI refresh or latency. Registers are acknowledged at SOF; visible values still need manual comparison. Updates stop when this function returns; STALE is then expected after about one second.')
    result.update(first=first_sample, last=last_sample, recorded_frames=record_count,
                  csv_path=csv_path, record_drops=worker.record_drops,
                  camera_error=camera_error, camera_degraded=camera_error is not None,
                  stop_reason='user_interrupt' if interrupted else ('error' if error or worker.error else 'duration'))
    result.update(camera_source=camera_source,camera_cancelled=camera_cancelled,camera_fault=camera_fault,
                  camera_worker=None if camera_profile is None or not hasattr(camera_profile,'diagnostics')
                        else camera_profile.diagnostics())
    result.update(started_monotonic=start, events=events)
    try:
        import cv2
        result['opencv_threads_used']=cv2.getNumThreads()
    except ImportError:
        result['opencv_threads_used']=None
    result['camera_prefetch']=False
    result['video_render_profile']='native_panel_ram_v4' if native_renderer is not None else 'cached_bgrx_v3'
    result['alignment']=alignment_summary(alignment_rows) if camera_requested else None
    result['expected_hud_sequence']=worker.last_paired_sequence if cam is not None else (
        result['latest']['sequence'] if result['latest'] else None)
    if log_path is not None and camera_requested:
        alignment_path=Path(log_path).with_name(Path(log_path).stem+'_alignment.json')
        save_result(alignment_path,dict(summary=result['alignment'],frames=alignment_rows))
        result['alignment_path']=str(alignment_path)
    if stop_event is not None and stop_event.is_set():
        result['stop_reason']='requested'
    result['reference_frame']=reference_description(hud.reference_frame)
    result['validation'] = dict(
        telemetry_integrity_pass=bool(result['decoder']['frames'] >= 3 and result['latest'] is not None
            and result['latest_age_s'] <= 1.0 and result['worker_error'] is None
            and not any(result['uart'][key] for key in ('overrun','frame_error','parity_error','receive_buffer_drops'))
            and not any(result['decoder'][key] for key in ('crc_errors','skipped_bytes','missing_frames','duplicates','invalid_measurements'))),
        hud_delivery_pass=bool(result['hud']['acknowledged'] >= 3 and not state['pending'] and not state['timed_out']
            and state['applied_tag'] == hud.last_submitted_tag and result['latest'] is not None
            and state['active_source_sequence'] == result['expected_hud_sequence']
            and not result['hud']['invalid_inputs'] and not result['hud']['format_errors']),
        camera_capture_pass=(bool(camera_frames > 0 and camera_error is None and error is None and not interrupted
                                  and not camera_cancelled and camera_source=='real_openart_usb')
                             if camera_requested else None),
        recording_pass=bool(record_count == result['decoder']['frames'] and worker.record_drops == 0))
    if first_sample is not None and last_sample is not None:
        result['motion_delta'] = {key: last_sample[key]-first_sample[key] for key in
                                  ('distance_grid', 'x_grid', 'y_grid')}
        result['motion_delta']['heading_shortest_deg'] = (last_sample['heading_deg']-first_sample['heading_deg']+180)%360-180
        if hud.grid_size_m is not None:
            result['motion_delta'].update({key: last_sample[key]-first_sample[key]
                                          for key in ('distance_m', 'x_m', 'y_m')})
    result['software_check_pass'] = bool(frames > 0 and result['decoder']['frames'] >= 3
        and result['latest'] is not None and result['latest_age_s'] <= 1.0
        and result['hud']['acknowledged'] >= 3 and not state['pending'] and not state['timed_out']
        and state['applied_tag'] == hud.last_submitted_tag
        and state['active_source_sequence'] == result['expected_hud_sequence']
        and not interrupted and error is None and result['worker_error'] is None
        and camera_error is None and worker.record_drops == 0
        and not any(result['uart'][k] for k in ('overrun','frame_error','parity_error','receive_buffer_drops'))
        and not any(result['decoder'][k] for k in ('crc_errors','skipped_bytes','missing_frames','duplicates','invalid_measurements'))
        and not result['hud']['invalid_inputs'] and not result['hud']['format_errors'])
    if camera_requested:
        result['software_check_pass']=bool(result['software_check_pass'] and result['validation']['camera_capture_pass'])
    if camera_requested:
        result['validation']['software_association_pass']=result['alignment']['association_pass']
        result['software_check_pass']=result['software_check_pass'] and result['alignment']['association_pass']
    save_result(log_path, result)
    print('LIVE_HUD_RESULT:', json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return result


def run_live(hdmi,uart_mmio,hud,cam=None,seconds=30,log_path='logs/05_live_camera.json',
             camera_fallback=False,camera_stall_seconds=3,stop_event=None,on_progress=None,
             on_receiver_stopped=None,frame_overlay=None):
    """Scope OpenCV's CPU policy to this capture and restore it on all exits."""
    try:
        import cv2
    except ImportError:
        return _run_live(hdmi,uart_mmio,hud,cam,seconds,log_path,camera_fallback,camera_stall_seconds,stop_event,on_progress,on_receiver_stopped,frame_overlay)
    previous=cv2.getNumThreads()
    try:
        cv2.setNumThreads(1)
        return _run_live(hdmi,uart_mmio,hud,cam,seconds,log_path,camera_fallback,camera_stall_seconds,stop_event,on_progress,on_receiver_stopped,frame_overlay)
    finally:
        cv2.setNumThreads(previous)
