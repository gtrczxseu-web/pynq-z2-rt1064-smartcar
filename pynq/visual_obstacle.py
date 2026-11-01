"""Low-resolution floor-appearance obstacle demo; no range or motor ownership."""
import json,queue,threading,time
from pathlib import Path
import cv2,numpy as np

NEAR_BOUNDARY_Y=179/239  # Measured 50 cm cup contact, 320x240 capture, 2026-11-01.

def forward_step(recipe,index):
    if not 0<=index<len(recipe['steps']):return False
    step=recipe['steps'][index]
    if step['kind']!='move':return False
    direction=dict(FORWARD=0,RIGHT=-90,BACK=180,LEFT=90)[step['direction']]
    return abs((direction-step['heading_deg']+180)%360-180)<=20


class GroundDetector:
    """Learn the visible side-floor appearance while stationary, then freeze it."""
    def __init__(self,polygon):
        self.polygon=np.asarray(polygon,dtype=float);self.reference=None;self.samples=[]
        self.floor=np.zeros((120,160),np.uint8)
        cv2.fillConvexPoly(self.floor,np.rint(self.polygon*[159,119]).astype(np.int32),255)
        self.top=max(0,min(110,int(np.min(self.polygon[:,1])*119)))
        self.corridor=np.array([[66,self.top],[94,self.top],[125,116],[35,116]],np.int32)
        self.lane=np.zeros_like(self.floor);cv2.fillConvexPoly(self.lane,self.corridor,255)
        self.lane=cv2.bitwise_and(self.lane,self.floor)
        # Fixed camera, flat floor; contact position is an approximate range gate.
        self.near_top=max(self.top,round(NEAR_BOUNDARY_Y*119))
        left=round(66+(35-66)*(self.near_top-self.top)/(116-self.top))
        self.corridor=np.array([[left,self.near_top],[160-left,self.near_top],[125,119],[35,119]],np.int32)
        self.near_lane=np.zeros_like(self.floor);cv2.fillConvexPoly(self.near_lane,self.corridor,255)
        self.near_lane=cv2.bitwise_and(self.near_lane,self.floor)
        self.side=(self.floor>0)&(self.lane==0);self.side[:, :5]=False;self.side[:,155:]=False
        self.side[:self.near_top]=False
        self.kernel=np.ones((3,3),np.uint8)
        rows=np.arange(120)[:,None];split=(self.near_top+120)//2
        self.bands=[self.side&(rows<split),self.side&(rows>=split)]
        self.bands_valid=all(np.count_nonzero(b)>=30 for b in self.bands)
        far=(self.near_top+split)/2;near=(split+120)/2
        self.blend=np.clip((rows-far)/(near-far),0,1).astype(np.float32)[:,:,None]
        self.shift_limit=np.asarray([40,8,8],dtype=np.float32)
        self.row_samples=[]
    def detect(self,frame,learn=False):
        image=cv2.resize(frame,(160,120),interpolation=cv2.INTER_AREA)
        lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB)
        floor,lane,corridor,top=self.floor,self.near_lane,self.corridor,self.top
        current=None
        if self.bands_valid:
            ends=[np.median(lab[b],axis=0).astype(np.float32) for b in self.bands]
            current=ends[0]*(1-self.blend)+ends[1]*self.blend
        if learn:
            pixels=lab[self.side]
            if len(pixels)>=100 and self.bands_valid:
                median=(ends[0]+ends[1])*.5
                deviation=np.percentile(np.abs(lab-current)[self.side],75,axis=0)
                if deviation[0]<=30 and max(deviation[1:])<=13:
                    if self.samples and np.max(np.abs(median-self.samples[-1]))>20:self.samples=[];self.row_samples=[]
                    self.samples=(self.samples+[median])[-3:]
                    self.row_samples=(self.row_samples+[current])[-3:]
                    if len(self.samples)>=3:self.reference=np.median(self.row_samples,axis=0)
        if self.reference is None:return dict(state='UNKNOWN',rect=None,learned_frames=len(self.samples),corridor=corridor.tolist())
        # Side-floor exposure changes after translation; never relearn the lane
        # while moving. Bound this frame's correction to retain dark/wide blocks.
        reference=self.reference
        if current is not None:reference=reference+np.clip(current-reference,-self.shift_limit,self.shift_limit)
        delta=lab-reference;difference=np.abs(delta)
        vertical=cv2.Sobel(delta,cv2.CV_32F,0,1,ksize=3)
        chroma_squared=difference[:,:,1]*difference[:,:,1]+difference[:,:,2]*difference[:,:,2]
        mask=np.uint8((chroma_squared>=18*18)|(difference[:,:,0]>=45))*255
        mask=cv2.bitwise_and(mask,floor)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,self.kernel)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,self.kernel)
        candidates=[]
        for contour in cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]:
            x,y,w,h=cv2.boundingRect(contour);area=cv2.contourArea(contour)
            if area<30 or w<6 or h<7 or y+h<top+7 or area/(w*h)<.4:continue
            bottom=y+h-1
            component=np.zeros_like(mask);cv2.drawContours(component,[contour],-1,255,-1)
            # Real body -> floor is a sharp contrast drop; its dim mirrored
            # continuation must not move the contact point towards the camera.
            contrast=np.mean(delta[component>0],axis=0)
            contrast/=max(float(np.linalg.norm(contrast)),1)
            start=max(0,self.top-12);end=min(120,bottom+3)
            scores=-np.mean(vertical[start:end,x+w//4:x+w-w//4].dot(contrast),axis=1)
            peak=float(np.max(scores))
            if peak<20:
                if bottom<118:continue
                contact=bottom  # Opaque objects clipped at the near image edge.
            else:
                strong=np.flatnonzero(scores>=max(20,peak*.75))
                contact=start+int(strong[-1])
            if contact<self.near_top:continue
            foot=component[max(y,min(contact,bottom)-2):min(contact,bottom)+1]
            columns=np.flatnonzero(np.any(foot,axis=0))
            if np.count_nonzero(lane[contact,columns])>=max(3,len(columns)*.5):candidates.append((area,(x,y,w,max(1,contact-y+1))))
        rect=max(candidates)[1] if candidates else None
        if rect is not None:
            x,y,w,h=rect;rect=(round(x*frame.shape[1]/160),round(y*frame.shape[0]/120),round(w*frame.shape[1]/160),round(h*frame.shape[0]/120))
        return dict(state='BLOCKED' if rect else 'CLEAR',rect=rect,learned_frames=3,corridor=corridor.tolist())


class ObstacleOverlay:
    """One latest-frame worker, 5 Hz maximum; drawing never enters inference."""
    def __init__(self,folder,policy_provider,observer,clock=time.monotonic):
        profile=Path(folder)/'rear_camera.json'
        polygon=json.loads(profile.read_text('utf8'))['ground_polygon'] if profile.exists() else [[0,.67],[1,.67],[1,1],[0,1]]
        self.detector=GroundDetector(polygon);self.policy_provider=policy_provider;self.observer=observer;self.clock=clock
        self.work=queue.Queue(maxsize=1);self.closed=threading.Event();self.thread=None
        self.last_submit=-1e9;self.result=None;self.error=None;self.frames=0;self.processing_ms=None;self.epoch=None
    def submit(self,frame,sample=None,session=0):
        policy=self.policy_provider();now=self.clock()
        if policy[0]=='off' or now-self.last_submit<.2:return
        self.last_submit=now
        if self.thread is None:
            self.thread=threading.Thread(target=self._run,name='vision-obstacles',daemon=True);self.thread.start()
        try:self.work.get_nowait()
        except queue.Empty:pass
        try:self.work.put_nowait((now,frame.copy(),None if sample is None else dict(sample),session,policy))
        except queue.Full:pass
    def _run(self):
        try:
            cv2.setNumThreads(1)
            while not self.closed.is_set():
                try:stamp,frame,sample,session,policy=self.work.get(timeout=.2)
                except queue.Empty:continue
                if self.closed.is_set() or policy!=self.policy_provider() or not 0<=self.clock()-stamp<=.5:continue
                epoch=None if sample is None else (sample.get('pose_epoch'),sample.get('source_clock_epoch'))
                if epoch!=self.epoch:
                    self.detector=GroundDetector(self.detector.polygon);self.epoch=epoch
                start=self.clock()
                try:result=self.detector.detect(frame,learn=policy[0]=='learn')
                except Exception as exc:
                    self.error=type(exc).__name__+': '+str(exc);result=dict(state='UNKNOWN',rect=None,corridor=[])
                if self.closed.is_set() or policy!=self.policy_provider() or not 0<=self.clock()-stamp<=.5:continue
                self.processing_ms=round((self.clock()-start)*1000,2);self.frames+=1
                self.result=(stamp,result,policy)
                if policy[0]=='monitor':self.observer(stamp,result,sample,session,policy[1][1])
                if self.error:return
        except Exception as exc:self.error=type(exc).__name__+': '+str(exc)
    def draw(self,frame):
        policy=self.policy_provider();result=self.result
        fresh=result is not None and result[2]==policy and 0<=self.clock()-result[0]<=.5
        state=result[1]['state'] if fresh else 'UNKNOWN'
        text='OBS FRONT ONLY' if policy[0]=='off' else ('OBS '+state+' ~0.5m')
        if self.error:text='OBS OFFLINE'
        color=(40,80,255) if state=='BLOCKED' and policy[0]!='off' else (150,170,150)
        cv2.putText(frame,text,(4,25),cv2.FONT_HERSHEY_SIMPLEX,.32,color,1,cv2.LINE_AA)
        boundary=round(NEAR_BOUNDARY_Y*(frame.shape[0]-1))
        cv2.line(frame,(0,boundary),(frame.shape[1]-1,boundary),(0,210,255),1)
        cv2.putText(frame,'50 cm',(4,boundary-4),cv2.FONT_HERSHEY_SIMPLEX,.32,(0,210,255),1,cv2.LINE_AA)
        polygon=self.detector.corridor.astype(float)*[frame.shape[1]/160,frame.shape[0]/120]
        cv2.polylines(frame,[np.rint(polygon).astype(np.int32)],True,color,1)
        if fresh and policy[0]!='off' and result[1]['rect'] is not None:
            x,y,w,h=result[1]['rect'];cv2.rectangle(frame,(x,y),(x+w-1,y+h-1),color,1)
    def snapshot(self):
        result=self.result;policy=self.policy_provider()
        fresh=result is not None and result[2]==policy and 0<=self.clock()-result[0]<=.5
        return dict(policy=policy[0],state=result[1]['state'] if fresh else 'UNKNOWN',learned=self.detector.reference is not None,error=self.error,frames=self.frames,processing_ms=self.processing_ms,range_estimated=False,approximate_max_distance_m=.5,boundary_calibrated=True,metric_range_calibrated=False,reflection_filter='contact_edge')
    def close(self):
        self.closed.set()
        if self.thread is not None:self.thread.join(2)
