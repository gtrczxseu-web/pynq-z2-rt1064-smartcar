"""Cached ARM operation menu. Numeric sensor HUD remains FPGA-generated."""
from pathlib import Path
import math,time
from PIL import Image,ImageDraw,ImageFont
import numpy as np

def font(size):
    for name in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf','C:/Windows/Fonts/arial.ttf'):
        if Path(name).exists():return ImageFont.truetype(name,size)
    return ImageFont.load_default()

class BootDisplay:
    def __init__(self,clock=time.monotonic,hint_seconds=4):
        self.clock=clock;self.phase=None;self.changed_at=clock();self.message=''
        self.small=font(14);self.menu=font(18);self.title=font(25);self.value=font(29)
        self.context={};self.route_overlay=None;self.camera_overlay=None;self.last=None;self.layer=None;self.canvas=None;self.base=None;self.native_key=None
        self.update('BOOTING','Keep vehicle and all wheels still')
    def update(self,phase,message):
        if phase!=self.phase:self.changed_at=self.clock()
        self.phase=phase;self.message=str(message)
    def content(self):
        p=self.phase;m=self.context.get('mcu') or {};step=m.get('step',0)+1;total=self.context.get('total_steps',12)
        next_speed=self.context.get('next_speed_percent');speed_note='' if next_speed is None else ' | NEXT %d%%'%next_speed
        if p=='SETTINGS':return dict(kind='settings',heading='RUN SETTINGS',hint='BTN2 SAVE')
        if p=='COUNTDOWN':return dict(kind='scene',heading='RESUMING START' if self.context.get('countdown_held') else 'START IN %d'%max(1,math.ceil(3-(self.clock()-self.changed_at))),hint='BTN1 STOP')
        if p in ('DRIVING','TURNING','GAP'):
            return dict(kind='scene',heading='%s   %d/%d'%(m.get('action',p),step,total)+speed_note,hint='BTN1 STOP')
        if p.startswith('BYPASS '):return dict(kind='scene',heading=p,hint='BTN1 STOP')
        if p=='TRAFFIC WAIT':return dict(kind='scene',heading=self.context.get('traffic_message') or 'TRAFFIC WAIT',hint='BTN1 STOP')
        if p=='OBSTACLE WAIT':return dict(kind='scene',heading=self.context.get('bypass_note') or 'OBSTACLE PAUSED',hint='BTN0 / GREEN' if self.context.get('obstacle_clear') else 'BTN1 STOP')
        if p=='STEP PAUSED':return dict(kind='scene',heading='STEP %d COMPLETE'%step+speed_note,hint='AUTO NEXT' if self.context.get('mode')=='continuous' else ('BTN0 / GREEN' if self.context.get('vision_control') else 'BTN0 NEXT'))
        if p=='READY':return dict(kind='scene',heading='READY | '+('AUTO' if self.context.get('mode')=='continuous' else 'STEP'),hint='BTN0 START')
        if p in ('ARMING','CONFIGURING','STARTING','RESUMING'):return dict(kind='scene',heading='PREPARING NEXT ACTION',hint='BTN1 STOP')
        if p in ('FINISHED','RUN STOPPED'):
            return dict(kind='result',heading='RUN COMPLETE' if p=='FINISHED' else 'RUN STOPPED',hint='BTN0 NEW RUN')
        if p in ('WAIT STILL','WAIT LINK'):
            missing=p=='WAIT LINK' or ('frames=0' in self.message)
            return dict(kind='notice',heading='WAITING FOR CONTROLLER' if missing else 'CALIBRATING',hint='',
                detail='Power on car controller; check UART.' if missing else 'Keep the car and every wheel still.')
        if p=='CONNECTING CAMERA':return dict(kind='notice',heading='CHECKING CAMERA',hint='',detail='Optional camera input; please wait.')
        if p=='CAMERA UNAVAILABLE':return dict(kind='notice',heading='CAMERA NOT CONNECTED',hint='',detail='Trajectory and sensors remain available.')
        if 'FAULT' in p or p in ('FAILED','UPDATE CAR'):
            detail=self.message.replace('RuntimeError: ','')[:100]
            return dict(kind='notice',heading='CHECK CONTROLLER' if p=='UPDATE CAR' else 'RUN STOPPED',hint='BTN0 RETRY' if p=='MOTION FAULT' else '',detail=detail)
        return dict(kind='notice',heading='STOPPING' if p in ('STOPPING','STOPPED') else 'STARTING',hint='',detail='Saving report and releasing devices.' if p in ('STOPPING','STOPPED') else 'Keep the car still; preparing display.')
    def _fit(self,draw,text,width,typeface):
        while text and draw.textbbox((0,0),text,font=typeface)[2]>width:text=text[:-1]
        return text
    def _render(self,content,video):
        r=self.context.get('summary') or {}
        key=(tuple(content.items()),video,r.get('measured_distance_m'),r.get('action_duration_s'),r.get('completed_steps'),len(r.get('recognitions',[])),self.context.get('report_error'),str(self.context.get('settings_menu')))
        if key==self.last:return
        self.last=key
        image=Image.new('RGB',(640,296),(8,16,26));draw=ImageDraw.Draw(image)
        heading='SYSTEM STATUS' if content['kind']=='notice' else content['heading']
        hint=content['hint'];colour=(103,224,151) if self.phase in ('READY','FINISHED') else (246,202,119)
        hint_width=draw.textbbox((0,0),hint,font=self.menu)[2] if hint else 0
        heading=self._fit(draw,heading,600-hint_width,self.menu)
        draw.text((12,7),heading,font=self.menu,fill=colour)
        if hint:draw.text((628-hint_width,7),hint,font=self.menu,fill='white')
        draw.line((12,37,628,37),fill=(45,66,85))
        if content['kind']=='settings':
            menu=self.context.get('settings_menu') or {}
            for i,(label,value) in enumerate(menu.get('rows',[])):
                y=48+i*31;selected=i==menu.get('cursor')
                if selected:draw.rounded_rectangle((12,y,628,y+29),radius=4,fill=(31,71,92))
                draw.text((24,y+4),('> ' if selected else '  ')+label,font=self.menu,fill=(103,224,151) if selected else (173,194,213))
                draw.text((340,y+4),value,font=self.menu,fill='white')
            draw.text((16,260),'BTN0 SELECT   BTN1 CHANGE   BTN3 CANCEL',font=self.small,fill=(246,202,119))
            detail=str(menu['error'])[:70] if menu.get('error') else menu.get('hint','')
            draw.text((16,280),detail,font=self.small,fill=(255,112,93) if menu.get('error') else (173,194,213))
        elif content['kind']=='scene' and not video:
            draw.text((92,140),'NO CAMERA',font=self.menu,fill=(156,181,201))
            draw.text((61,171),'Trajectory remains available',font=self.small,fill=(121,148,171))
        elif content['kind']=='result':
            draw.text((26,66),'RESULT',font=self.title,fill='white')
            if r:
                count=len((r.get('recipe') or {}).get('steps',[])) or 12
                rows=[('Distance','%.2f m'%r.get('measured_distance_m',0)),('Active time','--' if r.get('action_duration_s') is None else '%.1f s'%r['action_duration_s']),('Steps','%d / %d'%(r.get('completed_steps',0),count))]
                for i,(label,value) in enumerate(rows):
                    y=115+i*43;draw.text((26,y),label,font=self.menu,fill=(156,181,201));draw.text((176,y-3),value,font=self.title,fill='white')
                draw.text((26,237),'Seen %d events'%len(r.get('recognitions',[])),font=self.small,fill=(156,181,201))
                note='Report save failed' if self.context.get('report_error') else 'Report saved on board'
            else:note='Saving report...'
            draw.text((26,258),note,font=self.small,fill=(103,224,151))
        elif content['kind']=='notice':
            draw.text((24,100),content['heading'],font=self.title,fill=colour)
            words=content.get('detail','').split();line='';y=150
            for word in words:
                if draw.textbbox((0,0),line+' '+word,font=self.menu)[2]>570:
                    draw.text((24,y),line,font=self.menu,fill=(190,206,219));y+=27;line=word
                else:line=(line+' '+word).strip()
            draw.text((24,y),line,font=self.menu,fill=(190,206,219))
        self.layer=np.asarray(image)[:,:,::-1].copy()
    def render_camera(self,output,bgr):
        """Compose the native camera panel in RAM, then make one contiguous DMA copy."""
        import cv2
        from camera_quality import correct_camera
        content=self.content();video=content['kind']=='scene'
        self._render(content,video)
        if self.base is None or self.base.shape!=output.shape or self.native_key!=self.last:
            self.base=np.zeros(output.shape,dtype=np.uint8);self.base[:,:,:3]=(26,16,8)
            self.base[40:336,:,:3]=self.layer;self.canvas=np.empty(output.shape,dtype=np.uint8)
            self.native_key=self.last
        np.copyto(self.canvas,self.base)
        if video:
            camera=correct_camera(bgr)
            if camera.shape[:2]!=(240,320):camera=cv2.resize(camera,(320,240),interpolation=cv2.INTER_LINEAR)
            camera=camera.copy()
            if self.camera_overlay is not None:self.camera_overlay(camera)
            target=self.canvas[88:328,12:332]
            if output.shape[2]==4:
                cv2.cvtColor(camera,cv2.COLOR_BGR2BGRA,dst=target);target[:,:,3]=0
            else:np.copyto(target,camera)
        if content['kind'] in ('scene','result') and self.route_overlay is not None:self.route_overlay(self.canvas)
        np.copyto(output,self.canvas)
    def overlay(self,output,video_frame=None):
        available=self.context.get('camera') if video_frame is None else video_frame
        content=self.content();video=bool(content['kind']=='scene' and available)
        camera=output[::2,::2,:3].copy() if video else None
        if camera is not None and self.camera_overlay is not None:self.camera_overlay(camera)
        self._render(content,video)
        output[:40,:,:3]=(26,16,8)
        output[40:336,:,:3]=self.layer
        # Only the background changes here; the numeric HUD is drawn downstream in PL.
        output[336:,:,:3]=(26,16,8)
        if camera is not None:output[88:328,12:332,:3]=camera
        if content['kind'] in ('scene','result') and self.route_overlay is not None:self.route_overlay(output)
    def card(self,hdmi,phase,message):
        self.update(phase,message)
        output=hdmi.newframe()
        try:
            output[:,:,:3]=(26,16,8)
            if output.shape[2]==4:output[:,:,3]=255
            self.overlay(output,video_frame=False)
        except BaseException:output.freebuffer();raise
        hdmi.writeframe(output)
