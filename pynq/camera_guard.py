"""Finite parent waits for USB-only spawned process; no automatic bus/board reset."""
from collections import deque
import ctypes
import json
import math
import multiprocessing as mp
import threading
import time
import zlib
from camera_worker import usb_camera_server,send_message,MAX_MESSAGE,MAX_PAYLOAD

_quarantined=[]


class CameraGuardError(RuntimeError):
    def __init__(self,message,diagnostics):
        super().__init__(message)
        self.diagnostics=diagnostics


class CameraCancelled(CameraGuardError):
    pass


def quarantined_workers():
    global _quarantined
    active=[]
    for camera in _quarantined:
        if camera.process.is_alive():
            active.append(camera)
        else:
            camera.process.join(0)
    _quarantined=active
    return [dict(pid=c.process.pid,alive=True) for c in active]


class IsolatedCamera:
    supports_deadline=True
    source_kind='real_openart_usb'

    def __init__(self,wait_seconds=15,startup_timeout=40,frame_timeout=2.0,
                 _server=None,_test_options=None,stop_event=None):
        for value in (wait_seconds,startup_timeout,frame_timeout):
            if not math.isfinite(value) or value<=0:
                raise ValueError('Camera deadlines must be finite and positive')
        blocked=quarantined_workers()
        if blocked:
            raise CameraGuardError('Previous camera worker has not exited; no second USB owner will be started',
                                   dict(quarantined=blocked))
        self.closed=False
        self.last_packet=None
        self.last_diagnostic=None
        self.last_cleanup=None
        self.version=None
        self.port=None
        self.rgb565_byteorder='big'
        self.frame_timeout=frame_timeout
        self.events=deque(maxlen=24)
        self.event_drops=0
        self.request_id=0
        self._lock=threading.Lock()
        self.decoder=None
        context=mp.get_context('spawn')  # no inherited live DMA/MMIO/UART threads
        self.arena=context.RawArray(ctypes.c_ubyte,MAX_PAYLOAD)
        self.conn,child_conn=context.Pipe(duplex=True)
        options=dict(wait_seconds=wait_seconds)
        if _test_options is not None:
            if _server is None:raise ValueError('Test options require an explicit test server')
            options.update(_test_options)
        self.source_kind='synthetic_usb_worker' if _server is not None else 'real_openart_usb'
        self.process=context.Process(target=_server or usb_camera_server,args=(child_conn,self.arena,options),
                                     name='openart-usb-only',daemon=True)
        self.process.start()
        child_conn.close()
        try:
            ready=self._wait(startup_timeout,stop_event,expected=('ready',))
            self.version=tuple(ready['version'])
            self.port=ready['port']
        except BaseException:
            self.close()
            raise

    def _remember(self,event):
        if len(self.events)==self.events.maxlen:self.event_drops+=1
        self.events.append(event)

    def diagnostics(self,reason=None):
        packet=self.last_packet
        return dict(reason=reason,pid=self.process.pid,worker_alive=self.process.is_alive(),
            source_kind=self.source_kind,port=self.port,request_id=self.request_id,
            event_drops=self.event_drops,events=list(self.events),cleanup=self.last_cleanup,
            last_packet=None if packet is None else dict(sequence=packet.sequence,width=packet.width,
                height=packet.height,kind=packet.kind,bytes=len(packet.payload),
                requested_at=packet.requested_at,received_at=packet.received_at,
                header_ms=packet.header_ms,transfer_ms=packet.transfer_ms))

    def _fail(self,message,cancelled=False):
        self.close()
        self.last_diagnostic=self.diagnostics(message)
        error=CameraCancelled if cancelled else CameraGuardError
        raise error(message,self.last_diagnostic)

    def _wait(self,timeout,stop_event,expected):
        deadline=time.monotonic()+timeout
        while True:
            if stop_event is not None and stop_event.is_set():
                self._fail('Capture stop requested while waiting for camera; USB connection released',True)
            remaining=deadline-time.monotonic()
            if remaining<=0:
                self._fail('Camera worker deadline exceeded (%.3fs)'%timeout)
            if not self.conn.poll(min(.02,remaining)):
                if not self.process.is_alive():self._fail('Camera worker exited without a complete reply')
                continue
            try:
                message=json.loads(self.conn.recv_bytes(MAX_MESSAGE))
            except (OSError,EOFError,ValueError) as exc:
                self._fail('Invalid camera IPC reply: %s'%type(exc).__name__)
            if message.get('id')!=self.request_id:
                self._fail('Camera IPC request id mismatch')
            if message['type']=='stage':
                self._remember(message['event'])
                continue
            if message['type']=='error':
                self._remember(message)
                self._fail('%s: %s'%(message['error_type'],message['message']))
            if message['type'] not in expected:
                self._fail('Unexpected camera reply type')
            return message

    def read_frame(self,stop_event=None):
        if self.closed:raise CameraGuardError('Camera is closed; reconnect explicitly',self.diagnostics())
        if not self._lock.acquire(False):raise RuntimeError('Concurrent camera operations are forbidden')
        try:
            if stop_event is not None and stop_event.is_set():
                self._fail('Capture stop requested before next camera request',True)
            self.request_id+=1
            try:send_message(self.conn,dict(command='frame',id=self.request_id))
            except (OSError,EOFError):self._fail('Cannot send camera frame request')
            message=self._wait(self.frame_timeout,stop_event,('packet','empty'))
            if message['type']=='empty':return None
            size=message['size']
            if not isinstance(size,int) or not 0<size<=len(self.arena):self._fail('Invalid shared payload size')
            payload=bytes(memoryview(self.arena).cast('B')[:size])
            if zlib.crc32(payload)!=message['crc32']:self._fail('Shared camera payload CRC mismatch')
            from openart_usb_fast import FramePacket,Decoder
            packet=FramePacket(message['sequence'],message['width'],message['height'],message['kind'],payload,
                message['requested_at'],message['received_at'],message['header_ms'],message['transfer_ms'])
            if self.decoder is None:self.decoder=Decoder('auto')
            self.last_packet=packet
            return self.decoder.decode(packet,self.rgb565_byteorder)
        finally:
            self._lock.release()

    def close(self):
        """Bounded abort. Kernel-blocked survivors are quarantined, never reopened."""
        if self.closed:return
        self.closed=True
        began=time.monotonic()
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(.35)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(.35)
        alive=self.process.is_alive()
        if not alive:self.process.join(0)
        self.conn.close()
        self.last_cleanup=dict(pid=self.process.pid,reaped=not alive,exitcode=self.process.exitcode,
                               elapsed_s=time.monotonic()-began)
        if alive and self not in _quarantined:_quarantined.append(self)

    def shutdown(self):
        """Only while idle: release framebuffer/script before terminating child."""
        if self.closed:return self.last_cleanup
        try:
            self.request_id+=1
            send_message(self.conn,dict(command='close',id=self.request_id))
            self._wait(2.0,None,('closed',))
            self.process.join(.35)
        finally:
            self.close()
        if not self.last_cleanup['reaped']:
            raise CameraGuardError('Camera worker remains quarantined',self.diagnostics())
        return self.last_cleanup
