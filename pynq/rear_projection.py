"""Ground-plane rear-camera projection of encoder/IMU estimates; no control IO."""
import json,math,time
from pathlib import Path
import numpy as np
import cv2
from PIL import Image,ImageDraw
from trajectory_report import text_font,path_lines
from telemetry_reference import VEHICLE_INITIAL,CAMERA_INITIAL,validate_reference


def defaults():
    return dict(schema=1,enabled=True,mode='approximate',height_m=.15,pitch_down_deg=10,
        horizontal_fov_deg=70,camera_right_m=0,camera_forward_m=-.15,mirror=False,
        ground_polygon=[[0,.62],[1,.62],[1,1],[0,1]],homography=None)


def validate(c):
    c=dict(c)
    if c.get('schema')!=1 or type(c.get('enabled')) is not bool or type(c.get('mirror')) is not bool:raise ValueError('Invalid rear-camera profile')
    for k,lo,hi in [('height_m',.02,1),('pitch_down_deg',-15,80),('horizontal_fov_deg',20,140),('camera_right_m',-.5,.5),('camera_forward_m',-.8,.5)]:
        v=c[k]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError('Invalid '+k)
    polygon=np.asarray(c['ground_polygon'],dtype=float)
    if polygon.shape!=(4,2) or not np.isfinite(polygon).all() or (polygon<0).any() or (polygon>1).any():raise ValueError('Ground polygon needs four normalized image corners')
    if c.get('mode') not in ('approximate','homography'):raise ValueError('Invalid projection mode')
    if c['mode']=='homography':
        h=np.asarray(c['homography'],dtype=float)
        if h.shape!=(3,3) or not np.isfinite(h).all() or abs(np.linalg.det(h))<1e-12:raise ValueError('Invalid ground homography')
    return c


def fit_ground(points):
    """Measured camera-plane metres (right, backwards) -> normalized pixels."""
    if len(points)<4:raise ValueError('At least four measured ground correspondences required')
    a=[];b=[]
    for x,y,u,v in points:
        if not all(math.isfinite(t) for t in (x,y,u,v)) or not 0<=u<=1 or not 0<=v<=1:raise ValueError('Invalid measured calibration point')
        a.extend([[x,y,1,0,0,0,-u*x,-u*y],[0,0,0,x,y,1,-v*x,-v*y]]);b.extend([u,v])
    a=np.asarray(a);b=np.asarray(b)
    if np.linalg.matrix_rank(a)<8:raise ValueError('Calibration points must not be collinear')
    h=np.append(np.linalg.lstsq(a,b,rcond=None)[0],1).reshape(3,3)
    return h.tolist()


def save(folder,config):
    c=validate(config);p=Path(folder)/'rear_camera.json';t=p.with_suffix('.tmp')
    t.write_text(json.dumps(c,ensure_ascii=False,indent=2)+'\n',encoding='utf8');t.replace(p)
    return c


def project(point,current,heading,c,size=(320,240),reference_frame=VEHICLE_INITIAL):
    sign=-1 if validate_reference(reference_frame)==CAMERA_INITIAL else 1
    dx=sign*(point[0]-current[0]);dy=sign*(point[1]-current[1]);a=math.radians(heading)
    bx=math.cos(a)*dx+math.sin(a)*dy-c['camera_right_m']
    by=-math.sin(a)*dx+math.cos(a)*dy-c['camera_forward_m']
    right=-bx;back=-by
    if back<.05:return None
    if c['mode']=='homography':
        q=np.asarray(c['homography']).dot([right,back,1])
        if abs(q[2])<1e-8:return None
        u,v=q[0]/q[2],q[1]/q[2]
    else:
        pitch=math.radians(c['pitch_down_deg']);z=back*math.cos(pitch)+c['height_m']*math.sin(pitch)
        if z<.05:return None
        f=.5/math.tan(math.radians(c['horizontal_fov_deg'])/2)
        u=.5+f*right/z;v=.5+f*(size[0]/size[1])*(c['height_m']*math.cos(pitch)-back*math.sin(pitch))/z
        # The estimated optical horizon can lie above the selected floor.
        # Anchor this illustrative projection to that floor; measured homography stays untouched.
        horizon=.5-f*(size[0]/size[1])*math.tan(pitch)
        v+=max(0,min(y for x,y in c['ground_polygon'])-horizon)
    if c['mirror']:u=1-u
    if not all(math.isfinite(t) for t in (u,v)) or max(abs(u),abs(v))>20:return None
    return (round(u*(size[0]-1)),round(v*(size[1]-1)))


def project_many(points,current,heading,c,size=(320,240),reference_frame=VEHICLE_INITIAL):
    """Same projection as project(), sharing the transform across a whole path."""
    sign=-1 if validate_reference(reference_frame)==CAMERA_INITIAL else 1
    p=np.asarray(points,dtype=np.float64)
    if not len(p):return np.empty((0,2),np.int32),np.empty(0,bool)
    dx=sign*(p[:,0]-current[0]);dy=sign*(p[:,1]-current[1]);a=math.radians(heading)
    right=-(math.cos(a)*dx+math.sin(a)*dy-c['camera_right_m'])
    back=-(-math.sin(a)*dx+math.cos(a)*dy-c['camera_forward_m']);valid=back>=.05
    if c['mode']=='homography':
        q=np.column_stack((right,back,np.ones(len(p)))).dot(np.asarray(c['homography']).T)
        valid&=np.abs(q[:,2])>=1e-8;den=np.where(np.abs(q[:,2])>=1e-8,q[:,2],1)
        u=q[:,0]/den;v=q[:,1]/den
    else:
        pitch=math.radians(c['pitch_down_deg']);z=back*math.cos(pitch)+c['height_m']*math.sin(pitch)
        valid&=z>=.05;den=np.where(z>=.05,z,1);f=.5/math.tan(math.radians(c['horizontal_fov_deg'])/2)
        u=.5+f*right/den;v=.5+f*(size[0]/size[1])*(c['height_m']*math.cos(pitch)-back*math.sin(pitch))/den
        horizon=.5-f*(size[0]/size[1])*math.tan(pitch)
        v+=max(0,min(y for x,y in c['ground_polygon'])-horizon)
    if c['mirror']:u=1-u
    valid&=np.isfinite(u)&np.isfinite(v)&(np.abs(u)<=20)&(np.abs(v)<=20)
    pixels=np.zeros((len(p),2),np.int32)
    pixels[valid]=np.rint(np.column_stack((u[valid]*(size[0]-1),v[valid]*(size[1]-1)))).astype(np.int32)
    return pixels,valid


class RearProjection:
    def __init__(self,folder,clock=time.monotonic,reference_frame=VEHICLE_INITIAL):
        self.folder=Path(folder);self.clock=clock;self.config=defaults();self.last_load=-1e9
        self.reference_frame=validate_reference(reference_frame)
        self.last_render=-1e9;self.error=None;self.rgba=None;self.bgr=None;self.mask=None
        self.last_caption=None
        self.floor_key=None;self.floor_mask=None;self.banner_key=None;self.banner=None
    def load(self):
        if self.clock()-self.last_load<1:return
        self.last_load=self.clock();p=self.folder/'rear_camera.json'
        try:self.config=validate(json.loads(p.read_text('utf8'))) if p.exists() else defaults();self.error=None
        except Exception as exc:self.error=str(exc)
    def overlay(self,frame,points,current,heading,markers=(),fresh=True,caption=None):
        self.load()
        if not self.config['enabled']:return
        if self.clock()-self.last_render>=(.2 if caption is not None else .5) or caption!=self.last_caption:
            self.last_caption=caption
            self.last_render=self.clock();w,h=frame.shape[1],frame.shape[0];c=self.config
            rgba=np.zeros((h,w,4),np.uint8)
            if fresh and not self.error:
                recent=points[-800:]
                pixels,valid=project_many(recent,current,heading,c,(w,h),self.reference_frame)
                if len(recent)>1:
                    data=np.asarray(recent)
                    visible=valid[:-1]&valid[1:]&~data[1:,4].astype(bool)
                    for good,colour in ((True,(255,221,82,255) if caption is not None else (80,245,200,255)),(False,(255,189,88,255))):
                        lines=path_lines(pixels,visible&(data[1:,3].astype(bool)==good))
                        if lines:cv2.polylines(rgba,lines,False,colour,3)
            if fresh and not self.error and markers:
                image=Image.fromarray(rgba);draw=ImageDraw.Draw(image)
                for i,m in enumerate(markers):
                    if m['x_m'] is None:continue
                    p=project((m['x_m'],m['y_m']),current,heading,c,(w,h),self.reference_frame)
                    if p is not None:
                        x,y=p;draw.ellipse((x-5,y-5,x+5,y+5),outline=(255,221,82,255),width=2)
                        draw.text((x+6,y-10),'T%d'%(i+1),font=text_font(11),fill=(255,221,82,255))
                rgba=np.asarray(image).copy()
            # Mask every projected pixel to the user-adjustable visible floor.
            floor_key=(w,h,tuple(tuple(p) for p in c['ground_polygon']))
            if floor_key!=self.floor_key:
                mask=Image.new('L',(w,h));md=ImageDraw.Draw(mask)
                md.polygon([(round(x*(w-1)),round(y*(h-1))) for x,y in c['ground_polygon']],fill=255)
                self.floor_mask=np.asarray(mask);self.floor_key=floor_key
            np.minimum(rgba[:,:,3],self.floor_mask,out=rgba[:,:,3])
            label=(('PLAN PROFILE ERROR' if self.error else caption) if fresh or caption=='PLAN WAIT START' else 'PLAN WAIT DATA') if caption is not None else ('AR WAIT DATA' if not fresh else ('AR PROFILE ERROR' if self.error else ('AR CALIBRATED*' if c['mode']=='homography' else 'AR APPROX')))
            if (label,w,h)!=self.banner_key:
                banner=Image.new('RGBA',(123,18),(5,15,25,255));ImageDraw.Draw(banner).text((2,1),label,font=text_font(11),fill=(255,221,82,255))
                self.banner=np.asarray(banner);self.banner_key=(label,w,h)
            rgba[h-19:h-1,3:126]=self.banner
            self.rgba=rgba
            self.bgr=self.rgba[:,:,:3][:,:,::-1].copy();self.mask=self.rgba[:,:,3].copy()
        if self.rgba is not None:
            cv2.copyTo(self.bgr,self.mask,frame[:,:,:3])
