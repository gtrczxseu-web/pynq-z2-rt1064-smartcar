"""USB-only child process. Never imports PYNQ or accesses FPGA/MMIO."""
import ctypes
import json
import os
import time
import zlib

MAX_MESSAGE=2048
MAX_PAYLOAD=8*1024*1024


def send_message(conn,message):
    data=json.dumps(message,separators=(',',':')).encode('utf8')
    if len(data)>MAX_MESSAGE:
        raise ValueError('Camera control message exceeds atomic-message limit')
    conn.send_bytes(data)


def usb_camera_server(conn,arena,options):
    camera=None
    owner_lock=None
    request_id=0
    def trace(event):
        send_message(conn,dict(type='stage',id=request_id,event=event))
    try:
        import fcntl
        import tempfile
        lock_path=os.path.join(tempfile.gettempdir(),'hud_openart_1209_abd1.lock')
        fd=os.open(lock_path,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
        owner_lock=os.fdopen(fd,'r+')
        fcntl.flock(owner_lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        owner_lock.seek(0);owner_lock.truncate();owner_lock.write(str(os.getpid()));owner_lock.flush()
        from pl_camera_test import connect_camera,start_preview,close_camera
        trace(dict(stage='child_imported',pid=os.getpid(),monotonic=time.monotonic()))
        camera=connect_camera(wait_seconds=options['wait_seconds'],progress=trace)
        # One startup decode, then only compressed acquisition in this process.
        if camera.decoder.cv2 is not None:
            camera.decoder.cv2.setNumThreads(1)
        trace(dict(stage='preview_start',monotonic=time.monotonic()))
        start_preview(camera)
        send_message(conn,dict(type='ready',id=0,pid=os.getpid(),version=camera.version,
                               port=camera.serial.port))
        while True:
            message=json.loads(conn.recv_bytes(MAX_MESSAGE))
            request_id=message['id']
            if message['command']=='close':
                trace(dict(stage='graceful_close',monotonic=time.monotonic()))
                close_camera(camera)
                send_message(conn,dict(type='closed',id=request_id))
                break
            if message['command']!='frame':
                raise ValueError('Unknown camera worker command')
            packet=camera.read_packet()
            if packet is None:
                send_message(conn,dict(type='empty',id=request_id))
                continue
            size=len(packet.payload)
            if size>len(arena):
                raise ValueError('Payload exceeds shared arena')
            ctypes.memmove(ctypes.addressof(arena),packet.payload,size)
            send_message(conn,dict(type='packet',id=request_id,sequence=packet.sequence,
                width=packet.width,height=packet.height,kind=packet.kind,size=size,
                crc32=zlib.crc32(packet.payload),requested_at=packet.requested_at,
                received_at=packet.received_at,header_ms=packet.header_ms,transfer_ms=packet.transfer_ms))
    except (EOFError,BrokenPipeError):
        pass
    except BaseException as exc:
        try:
            send_message(conn,dict(type='error',id=request_id,error_type=type(exc).__name__,
                message=str(exc)[:300],monotonic=time.monotonic()))
        except (EOFError,BrokenPipeError,OSError):
            pass
    finally:
        if camera is not None:
            try:camera.close()  # descriptor only; no new USB requests on error
            except Exception:pass
        if owner_lock is not None:owner_lock.close()
        conn.close()
