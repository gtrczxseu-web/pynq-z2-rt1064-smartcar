"""V7 sensing display plus manually armed two-lap route control."""
import argparse,signal,sys
from pathlib import Path
from sensing_app import Application as SensingApplication,HardwareOwner,load_config
from motion_link import MotionLink
from trajectory_report import RouteReport
import route_settings
from settings_menu import SettingsMenu
from light_vision import VisionOverlay
from telemetry_reference import CAMERA_INITIAL
from visual_obstacle import ObstacleOverlay,forward_step

class Application(SensingApplication):
    def __init__(self,*args,motion_factory=MotionLink,**kw):
        super().__init__(*args,**kw);self.motion=None;self.motion_factory=motion_factory
        self.report=RouteReport(self.folder/'reports',rect=(348,88,280,240),minimal=True,reference_frame=CAMERA_INITIAL)
        self.settings_menu=SettingsMenu(self.folder)
        self.vision=VisionOverlay(self.folder,observer=self.vision_observed,digit_observer=self.digit_observed,
            policy_provider=self.vision_policy,camera_view='forward')
        self.obstacle=ObstacleOverlay(self.folder,self.obstacle_policy,self.obstacle_observed)
    def obstacle_policy(self):
        motion=self.motion
        if motion is None or self.settings_menu.open:return 'off',None
        m=motion.snapshot();status=m.get('mcu') or {};latest=motion.latest or {}
        token=(m.get('session'),status.get('step'),latest.get('pose_epoch'),latest.get('source_clock_epoch'))
        phase=status.get('bypass_phase') or m.get('bypass_phase')
        if m.get('bypass_pending') or phase in ('BYPASS_RIGHT','BYPASS_LEFT') or status.get('state') in ('BYPASS_RIGHT','BYPASS_LEFT'):return 'off',None
        if m.get('active'):
            if status.get('session')==m.get('session') and status.get('protocol_version',0)>=6 and forward_step(motion.recipe,status.get('step',-1)) and status.get('state') in ('COUNTDOWN','MOVE','HOLDING','HELD','BYPASS_FORWARD'):
                return 'monitor',token
            return 'off',None
        if latest.get('pose_valid') and latest.get('speed_m_s',1)<.02:return 'learn',token
        return 'off',None
    def obstacle_observed(self,stamp,result,sample,session,index):
        if self.motion is not None:self.motion.receive_obstacle(stamp,result,sample,session,index)
    def vision_policy(self):
        motion=self.motion
        if motion is None or self.settings_menu.open:return 'off',None
        m=motion.snapshot();latest=motion.latest or {}
        token=(m.get('session'),latest.get('pose_epoch'),latest.get('source_clock_epoch'))
        if motion.recipe['mode']=='continuous':return 'colors',token
        status=m.get('mcu') or {}
        if m.get('obstacle_hold') and status.get('state')=='HELD' and status.get('session')==m.get('session'):return 'colors',token
        if m.get('state')=='PAUSED' and status.get('state')=='PAUSED' and status.get('session')==m.get('session'):
            return 'step_wait',token+(status['step'],)
        return 'off',None
    def vision_observed(self,stamp,detections,sample=None,session=0):
        if self.vision_policy()[0]=='off':return
        self.report.vision(stamp,detections,sample,session,channel='colors')
        motion=self.motion
        if motion is not None:motion.receive_vision(stamp,detections,sample,session)
    def digit_observed(self,stamp,detections,sample=None,session=0):
        if self.vision_policy()[0]!='step_wait':return
        self.report.vision(stamp,detections,sample,session,channel='digits')
        self.motion.receive_digits(stamp,detections,sample,session)
    def camera_overlay(self,frame):
        motion=self.motion
        self.obstacle.submit(frame,None if motion is None else motion.latest,0 if motion is None else motion.session)
        self.vision.overlay(frame,None if motion is None else motion.latest,0 if motion is None else motion.session)
        self.obstacle.draw(frame)
        if not (motion is not None and getattr(motion,'bypass_active',False)):self.report.camera_overlay(frame)
    def request(self,action):
        menu=self.settings_menu
        if menu.open and action in ('toggle_pause','retry_camera','settings','settings_cancel'):
            try:
                if action=='toggle_pause':menu.next()
                elif action=='retry_camera':menu.change()
                elif action=='settings_cancel':menu.cancel()
                else:
                    recipe=menu.confirm()
                    if self.motion is not None:self.motion.recipe=recipe
            except Exception as exc:menu.error=str(exc)
            return
        if action=='settings':
            if self.motion is not None and (self.motion.active or self.motion.state=='PAUSED'):return
            try:menu.begin()
            except Exception as exc:menu.error=str(exc)
            return
        if action=='settings_cancel':return
        if action=='toggle_pause':
            if self.motion is not None:self.motion.toggle()
            return
        if self.motion is not None:self.motion.stop('Local action: '+action)
        menu.cancel()
        if action=='retry_camera':
            camera=getattr(self.session,'cam',None)
            if camera is not None and not camera.closed and self.camera_error is None:return
        super().request(action)
    def stop(self,*args):
        if self.motion is not None:self.motion.stop('Application stop')
        super().stop(*args)
    def calibrate(self):
        if self.motion is not None:self.motion.stop('Stationary recalibration')
        ok=super().calibrate()
        if ok and self.motion is None:
            self.motion=self.motion_factory(self.session.uart)
            self.motion.reference_frame=CAMERA_INITIAL
            self.motion.settings_loader=lambda:route_settings.load(self.folder)
            try:self.motion.recipe=self.motion.settings_loader()
            except Exception as exc:print('ROUTE_SETTINGS_UNAVAILABLE',str(exc),flush=True)
            self.motion.pose_sink=self.report.pose
            self.report.event_log_path=self.session.log_dir/'motion_events.jsonl'
            self.motion.log_sink=lambda record:self.report.event(record,self.motion.latest)
            self.motion.start()
        return ok
    def progress(self,snapshot):
        super().progress(snapshot)
        self.report.poll()
    def set_phase(self,phase,message,card=False):
        status=None;m={}
        if self.motion is not None and phase.startswith('RUNNING'):
            m=self.motion.snapshot();state=m['state'];status=m['mcu']
            if status is not None and not m['compatible']:phase='UPDATE CAR';message='Install V7 turn-speed firmware for ROTATE.' if m.get('route')=='rotate' else 'Install step-pause car firmware before BTN0 can start.'
            elif state=='FAULT' or m['fault']:phase='MOTION FAULT';message=m['fault']
            elif state=='PAUSED':phase='STEP PAUSED';message='LAP %d LEG %d DONE | BTN0 NEXT STEP'%(status['lap'],status['leg'])
            elif state=='FINISHED':phase='FINISHED';message='Route complete. Motors OFF. BTN0 starts a new run.'
            elif state in ('BYPASS','BYPASS_STARTING'):
                phase=(m.get('bypass_phase') or 'BYPASS_STARTING').replace('_',' ');message='Right 0.5m / forward 1.0m / left 0.5m; side clearance assumed'
            elif state in ('SIGNAL_HOLDING','SIGNAL_WAIT','SIGNAL_RELEASING'):
                if m.get('obstacle_hold'):
                    phase='OBSTACLE WAIT';message='CLEAR: BTN0 / GREEN' if m.get('obstacle_clear') and m.get('traffic_reason') is None else 'Obstacle / signal: keep stopped'
                else:
                    phase='TRAFFIC WAIT';message='RED: WAIT GREEN' if m.get('traffic_reason')=='RED' else ('YELLOW: WAIT %.1fs'%m['yellow_remaining_s'] if m.get('yellow_remaining_s') is not None else 'STOPPING / RESUMING')
            elif m['active']:
                phase=state
                message=status['action'] if status else state
            else:
                phase='RUN STOPPED' if self.report.latest_report is not None else ('READY' if m['ready'] else 'WAIT LINK')
                message='BTN0 starts a new run'
        if self.display is not None:
            self.display.route_overlay=self.report.overlay
            self.display.camera_overlay=self.camera_overlay
            self.display.context=dict(camera=bool(self.session and self.session.cam is not None and not self.session.cam.closed),
                mcu=status,summary=self.report.latest_report,report_error=self.report.error,
                mode=self.motion.recipe['mode'] if self.motion is not None else 'step',
                total_steps=len(self.motion.recipe['steps']) if self.motion is not None else 12,
                vision_control=False if self.motion is None else m.get('vision_control',False),
                traffic_message=message if phase=='TRAFFIC WAIT' else None,
                next_speed_percent=None if self.motion is None else m.get('next_speed_percent'),
                obstacle_clear=m.get('obstacle_clear',False) and m.get('traffic_reason') is None,
                bypass_note=m.get('bypass_note'),
                countdown_held=m.get('countdown_held',False),
                settings_menu=self.settings_menu.snapshot())
        if self.settings_menu.open:phase='SETTINGS';message='Button menu; motors remain stopped'
        super().set_phase(phase,message,card)
    def run(self):
        try:return super().run()
        finally:
            self.report.close()
            self.vision.close()
            self.obstacle.close()

def main():
    import cv2,faulthandler
    faulthandler.enable()
    cv2.setNumThreads(1)  # keep the thread pool fixed while vision is active
    parser=argparse.ArgumentParser();parser.add_argument('--config',default=str(Path(__file__).with_name('autostart.json')))
    args=parser.parse_args();folder=Path(__file__).resolve().parent
    config=load_config(args.config)
    if config.get('motion_route')!='two_laps_10grid_step_pause':raise ValueError('Explicit step-pause route configuration required')
    app=Application(folder,config)
    signal.signal(signal.SIGTERM,app.stop);signal.signal(signal.SIGINT,app.stop)
    with HardwareOwner():return app.run()

if __name__=='__main__':sys.exit(main())
