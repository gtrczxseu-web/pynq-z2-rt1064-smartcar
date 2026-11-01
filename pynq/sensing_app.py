"""Offline PYNQ sensing/display application. No motor command is implemented."""
import argparse,json,math,os,queue,signal,subprocess,sys,threading,time
from pathlib import Path

def atomic_json(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    os.replace(str(temporary),str(path))

def read_text(path):
    try:return Path(path).read_text().strip()
    except OSError:return None

def load_config(path):
    c=json.loads(Path(path).read_text(encoding='utf8'))
    if c.get('grid_size_m')!=.2 or type(c.get('camera')) is not bool:raise ValueError('Invalid grid/camera configuration')
    for key,lo,hi in (('segment_seconds',5,300),('calibration_seconds',3,10),('startup_delay_s',0,60),('minimum_free_mb',64,4096)):
        value=c.get(key)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi:
            raise ValueError('Invalid '+key)
    if not isinstance(c.get('stationary_limits'),dict):raise ValueError('Stationary limits required')
    return c

class HardwareOwner:
    def __enter__(self):
        import fcntl
        fd=os.open('/run/lock/hud_autostart_v6.lock',os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o644)
        self.file=os.fdopen(fd,'r+')
        try:fcntl.flock(self.file.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BaseException:self.file.close();raise RuntimeError('Another standalone hardware owner is active')
        self.file.seek(0);self.file.truncate();self.file.write(str(os.getpid()));self.file.flush()
        return self
    def __exit__(self,*args):self.file.close()

class Application:
    def __init__(self,folder,config,session=None,display=None,controls_factory=None):
        self.folder=Path(folder).resolve();self.config=config
        self.stop_event=threading.Event();self.segment_stop=threading.Event();self.actions=queue.Queue(maxsize=16)
        self.phase='BOOTING';self.message='Keep vehicle and all wheels still'
        self.paused=False;self.needs_calibration=True;self.calibration_requested=True
        self.camera_retry=False;self.camera_error=None;self.poweroff=False;self.fault=None;self.last_camera_attempt=-1e9
        self.session=session;self.display=display;self.controls=None;self.controls_factory=controls_factory
        self.last_status=0;self.latest=None;self.segment=0;self.last_validation=None
        self.boot_id=read_text('/proc/sys/kernel/random/boot_id') or 'host-test'
        self.status_path=self.folder/'state/status.json';self.exit_code=0
    def request(self,action):
        try:self.actions.put_nowait(action)
        except queue.Full:
            self.fault='Local action queue overflow';self.stop_event.set()
        self.segment_stop.set()
    def stop(self,*unused):self.stop_event.set();self.segment_stop.set()
    def status(self,force=False):
        now=time.monotonic()
        if not force and now-self.last_status<2:return
        self.last_status=now
        record=dict(phase=self.phase,message=self.message,pid=os.getpid(),boot_id=self.boot_id,
            monotonic=now,ethernet_carrier=read_text('/sys/class/net/eth0/carrier'),network_required=False,
            motor_control=True,motion=None if getattr(self,'motion',None) is None else self.motion.snapshot(),paused=self.paused,needs_calibration=self.needs_calibration,
            camera_expected=self.config['camera'],camera_connected=bool(self.session and self.session.cam is not None and not self.session.cam.closed),
            camera_error=self.camera_error,segment=self.segment,latest=self.latest,last_validation=self.last_validation,
            log_dir=None if self.session is None else str(self.session.log_dir),fault=self.fault,
            trajectory=None if getattr(self,'report',None) is None else self.report.snapshot(),
            vision=None if getattr(self,'vision',None) is None else self.vision.snapshot(),
            obstacle=None if getattr(self,'obstacle',None) is None else self.obstacle.snapshot())
        atomic_json(self.status_path,record)
        if self.session is not None:
            self.session.log_dir.mkdir(parents=True,exist_ok=True)
            heartbeat={key:record[key] for key in ('phase','pid','boot_id','monotonic','ethernet_carrier','camera_connected','segment','needs_calibration')}
            heartbeat['pose_epoch']=None if self.latest is None else self.latest.get('pose_epoch')
            with (self.session.log_dir/'offline_heartbeat.jsonl').open('a',encoding='utf8') as handle:
                handle.write(json.dumps(heartbeat)+'\n')
    def set_phase(self,phase,message,card=False):
        changed=phase!=self.phase
        self.phase,self.message=phase,str(message)
        if self.display is not None:
            self.display.update(phase,message)
            if card and self.session is not None and self.session.ready:self.display.card(self.session.hdmi,phase,message)
        self.status(force=changed)
    def process_actions(self):
        while True:
            try:action=self.actions.get_nowait()
            except queue.Empty:break
            if action=='toggle_pause':
                self.paused=not self.paused
                if not self.paused and self.config['camera'] and self.camera_error is None:self.camera_retry=True
            elif action=='retry_camera':self.camera_retry=True
            elif action=='recalibrate':self.calibration_requested=True;self.needs_calibration=True;self.paused=False
            elif action=='shutdown':self.poweroff=True;self.stop()
            elif action=='control_error':raise RuntimeError('Local GPIO control failed')
    def connect_camera(self):
        if not self.config['camera']:return
        self.last_camera_attempt=time.monotonic()
        self.camera_retry=False
        self.set_phase('CONNECTING CAMERA','Keep still. Camera startup has a finite deadline.',card=True)
        try:
            if self.session.cam is not None:
                self.session.disconnect_camera()
            self.session.connect_camera(wait_seconds=10,stop_event=self.stop_event)
            self.camera_error=None
        except Exception as exc:
            if self.stop_event.is_set():return
            self.camera_error=type(exc).__name__+': '+str(exc)
            print('OFFLINE_CAMERA_UNAVAILABLE',self.camera_error,flush=True)
            self.set_phase('CAMERA UNAVAILABLE','Sensors can run on colour bars. BTN1 retries camera.',card=True)
    def retry_missing_camera(self):
        motion=getattr(self,'motion',None)
        missing=self.session.cam is None or self.session.cam.closed
        if (self.config['camera'] and missing and time.monotonic()-self.last_camera_attempt>=10
                and not (motion is not None and (motion.active or motion.state=='PAUSED'))):
            self.connect_camera()
    def progress(self,snapshot):
        latest=snapshot.get('latest_observed') or snapshot.get('latest')
        self.latest=latest
        if snapshot.get('camera_degraded'):self.camera_error=snapshot.get('camera_error') or 'Camera lost'
        if self.needs_calibration:
            self.set_phase('FAULT','Tracking lost. Stop movement; hold BTN2 to recalibrate.')
        else:
            phase='RUNNING' if not self.config['camera'] or self.camera_error is None else 'RUNNING NO CAMERA'
            self.set_phase(phase,'RAW %d | CAM %d | SEG %d'%(snapshot['decoder']['raw_frames'],snapshot['camera_frames'],self.segment))
    def calibrate(self):
        self.set_phase('WAIT STILL','Keep vehicle and every wheel stationary. Checking fresh RAW33.',card=True)
        try:
            result=self.session.calibrate(self.config['calibration_seconds'],stationary_limits=self.config['stationary_limits'])
        except RuntimeError as exc:
            self.set_phase('WAIT STILL',str(exc)[:120],card=True)
            self.stop_event.wait(1)
            return False
        self.needs_calibration=False;self.calibration_requested=False;self.fault=None
        print('OFFLINE_CALIBRATED',json.dumps(result),flush=True)
        return True
    def run(self):
        try:
            if self.session is None:
                from demo_session import DemoSession
                from telemetry_reference import CAMERA_INITIAL
                self.session=DemoSession(self.folder,grid_size_m=.2,reference_frame=CAMERA_INITIAL)
            self.status(True)
            self.session.setup()
            if self.display is None:
                from boot_display import BootDisplay
                self.display=BootDisplay()
            if self.controls_factory is None:
                from boot_controls import LocalControls
                self.controls_factory=LocalControls
            self.controls=self.controls_factory(self.session.overlay,self.request,lambda:self.phase)
            self.controls.start()
            self.set_phase('BOOTING','Keep still. Automatic sensor calibration will establish the origin.',card=True)
            if self.stop_event.wait(self.config['startup_delay_s']):return 0
            self.connect_camera()
            while not self.stop_event.is_set():
                self.process_actions()
                if self.stop_event.is_set():break
                if self.controls.error:raise RuntimeError(self.controls.error)
                if self.paused:
                    self.set_phase('PAUSED',self.fault or 'UART tracking continues. BTN0 resumes; BTN1 retries camera.',card=True)
                    self.stop_event.wait(.25);continue
                if self.camera_retry:self.connect_camera()
                else:self.retry_missing_camera()
                if self.needs_calibration:
                    if not self.calibration_requested:
                        self.set_phase('FAULT','Keep still and hold BTN2 for 2 seconds to recalibrate.',card=True)
                        self.stop_event.wait(.25);continue
                    if not self.calibrate():continue
                if self.stop_event.is_set():break
                space=os.statvfs(self.folder)
                if space.f_bavail*space.f_frsize<self.config['minimum_free_mb']*1024**2:
                    self.paused=True;self.fault='Low disk space; recording paused';continue
                self.segment+=1;self.segment_stop.clear()
                use_camera=bool(self.config['camera'] and self.session.cam is not None and not self.session.cam.closed)
                self.set_phase('RUNNING' if use_camera or not self.config['camera'] else 'RUNNING NO CAMERA',
                    'Real sensor processing; logs saved in one-minute segments.')
                result=self.session.capture(self.config['segment_seconds'],camera=use_camera,stop_event=self.segment_stop,
                    on_progress=self.progress,frame_overlay=self.display.overlay)
                if result.get('video_error') or result.get('worker_error'):raise RuntimeError(result.get('video_error') or result.get('worker_error'))
                if result.get('camera_degraded'):self.camera_error=result.get('camera_error') or 'Camera lost'
                self.last_validation=dict(result['validation'],expected_camera=self.config['camera'],
                    expected_camera_pass=(result['validation'].get('camera_capture_pass') is True) if self.config['camera'] else None)
                atomic_json(self.session.log_dir/('offline_%03d.json'%self.segment),dict(phase=self.phase,
                    boot_id=self.boot_id,validation=self.last_validation,camera_error=self.camera_error,
                    software_check_pass=bool(result['software_check_pass'] and (not self.config['camera'] or use_camera)),
                    result_file='%03d_%s.json'%(self.session.run_number,'camera' if use_camera else 'colorbars')))
                self.status(True)
        except BaseException as exc:
            self.exit_code=1;self.fault=type(exc).__name__+': '+str(exc)
            print('OFFLINE_FATAL',self.fault,flush=True)
            try:self.set_phase('FAULT',self.fault[:120],card=True)
            except Exception:pass
        finally:
            if getattr(self,'motion',None) is not None:self.motion.close()
            self.segment_stop.set()
            if self.controls is not None:
                try:self.controls.stop()
                except Exception as exc:self.exit_code=1;self.fault=str(exc)
            if self.session is not None:
                try:
                    if self.session.ready:self.set_phase('STOPPING','Saving logs and releasing camera / HDMI.',card=True)
                    self.session.close()
                    if self.session.log_dir.is_dir():self.session.export_logs()
                except Exception as exc:self.exit_code=1;self.fault=str(exc)
            self.phase='STOPPED' if not self.exit_code else 'FAILED';self.status(True)
            print('OFFLINE_EXIT',json.dumps(dict(code=self.exit_code,poweroff=self.poweroff,fault=self.fault)),flush=True)
        if self.poweroff and self.exit_code==0:
            os.sync()
            try:subprocess.run(['/usr/sbin/shutdown','-h','now'],check=True,timeout=8)
            except Exception as exc:
                self.exit_code=1;self.fault='Poweroff request failed: '+str(exc);self.phase='FAILED';self.status(True)
        return self.exit_code

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',default=str(Path(__file__).with_name('autostart.json')))
    args=parser.parse_args();folder=Path(__file__).resolve().parent
    app=Application(folder,load_config(args.config))
    signal.signal(signal.SIGTERM,app.stop);signal.signal(signal.SIGINT,app.stop)
    with HardwareOwner():return app.run()

if __name__=='__main__':sys.exit(main())
