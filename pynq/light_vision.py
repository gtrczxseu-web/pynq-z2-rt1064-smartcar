"""Pretrained digit CNN and bounded color cards; ARM display only, no motor IO."""
from pathlib import Path
import queue,threading,time
import numpy as np


def color_masks(bgr):
    """Same integer rule as axis_color_mask.v: RGB >=100 and dominance >=40."""
    b,g,r=np.moveaxis(bgr.astype(np.int16),-1,0)
    return [(r>=100)&(r-g>=40)&(r-b>=40),
            (g>=100)&(g-r>=40)&(g-b>=40),
            (b>=100)&(b-r>=40)&(b-g>=40)]


def digit_region(bgr):
    h,w=bgr.shape[:2]
    return w//16,h//12,15*w//16,11*h//12


def digit_input(bgr,cv):
    """Single black digit on a white card, excluding the image border."""
    left,top,right,bottom=digit_region(bgr);crop=bgr[top:bottom,left:right]
    gray=cv.cvtColor(crop,cv.COLOR_BGR2GRAY)
    if int(gray.max())-int(gray.min())<60:return None
    # Local contrast preserves grey strokes on a shaded card beside dark scenery.
    ink=cv.adaptiveThreshold(gray,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY_INV,31,8)
    # A dark scene around a small white card encloses the digit contour.
    contours=cv.findContours(ink,cv.RETR_LIST,cv.CHAIN_APPROX_SIMPLE)[-2]
    boxes=[]
    hsv=cv.cvtColor(crop,cv.COLOR_BGR2HSV)
    for c in contours:
        x,y,bw,bh=cv.boundingRect(c)
        if not (x>3 and y>3 and x+bw<crop.shape[1]-3 and y+bh<crop.shape[0]-3 and bh>=18 and bw>=3 and .08<=bw/bh<=1.3):continue
        mask=ink[y:y+bh,x:x+bw]>0
        # Ink must be dark and surrounded by a brighter card, not a colored marker.
        if not .06<=float(mask.mean())<=(1 if bw/bh<.22 else .85):continue
        ink_gray=float(np.median(gray[y:y+bh,x:x+bw][mask]))
        if np.median(hsv[y:y+bh,x:x+bw,1][mask])>95 and np.median(hsv[y:y+bh,x:x+bw,2][mask])>70:continue
        pad=max(4,round(bh*.12));x0,y0=max(0,x-pad),max(0,y-pad)
        x1,y1=min(crop.shape[1],x+bw+pad),min(crop.shape[0],y+bh+pad)
        surround=np.ones((y1-y0,x1-x0),bool);surround[y-y0:y-y0+bh,x-x0:x-x0+bw]=False
        ring=gray[y0:y1,x0:x1][surround]
        if np.mean(ring>ink_gray+12)<.8 or np.median(ring)<100 or np.median(ring)-ink_gray<20:continue
        boxes.append((x,y,bw,bh))
    if not boxes:return None
    x,y,bw,bh=max(boxes,key=lambda q:q[2]*q[3]);digit=ink[y:y+bh,x:x+bw]
    scale=20/max(bw,bh);nw,nh=max(1,round(bw*scale)),max(1,round(bh*scale))
    canvas=np.zeros((28,28),dtype=np.uint8);ox,oy=(28-nw)//2,(28-nh)//2
    canvas[oy:oy+nh,ox:ox+nw]=cv.resize(digit,(nw,nh),interpolation=cv.INTER_AREA)
    # MNIST digits are centered by ink mass rather than the bounding rectangle.
    moments=cv.moments(canvas)
    if moments['m00']:
        dx=13.5-moments['m10']/moments['m00'];dy=13.5-moments['m01']/moments['m00']
        canvas=cv.warpAffine(canvas,np.float32([[1,0,dx],[0,1,dy]]),(28,28))
    return canvas.astype(np.float32).reshape(1,1,28,28)/255,(left+x,top+y,bw,bh)


def detect_colors(bgr,cv):
    found=[]
    hsv=cv.cvtColor(bgr,cv.COLOR_BGR2HSV);height,width=bgr.shape[:2]
    kernel=np.ones((3,3),np.uint8)
    for name,color,(lo,hi) in zip(('RED','GREEN','BLUE','YELLOW'),((0,0,255),(0,255,0),(255,120,0),(0,220,255)),((0,16),(35,85),(100,135),(20,32))):
        # Keep the calibrated HSV ranges, but do pixel operations inside OpenCV.
        mask=cv.inRange(hsv,(lo,110,60),(hi,255,255))
        if name=='RED':mask=cv.bitwise_or(mask,cv.inRange(hsv,(170,110,60),(179,255,255)))
        # Join small gaps from cover lettering and shading without merging
        # distant objects or relaxing the wall/size rejection below.
        mask=cv.morphologyEx(mask,cv.MORPH_CLOSE,kernel)
        contours=cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[-2]
        cards=[]
        for c in contours:
            x,y,w,h=cv.boundingRect(c);area=cv.contourArea(c)
            if x<2 or y<2 or x+w>width-2 or y+h>height-2:continue
            # Yellow needs a presented card, not a small distant chair/label.
            minimum=max(120,height*width*.008) if name=='YELLOW' else 120
            maximum=height*width*.20
            # A nearby card can exceed 20%; require a central, filled convex
            # quadrilateral before raising its limit. Frame-edge walls stay out.
            if area>maximum and .3*width<=x+w/2<=.7*width and .3*height<=y+h/2<=.7*height and area/(w*h)>=.8:
                polygon=cv.approxPolyDP(c,.03*cv.arcLength(c,True),True)
                if len(polygon)==4 and cv.isContourConvex(polygon):maximum=height*width*.45
            if minimum<=area<=maximum and .4<=w/h<=2.5 and area/(w*h)>=.65:cards.append(c)
        if cards:found.append(dict(label=name,rect=cv.boundingRect(max(cards,key=cv.contourArea)),color=color))
    return found


class Recognizer:
    def __init__(self,folder):
        import cv2
        cv2.setNumThreads(1)
        model=np.frombuffer((Path(folder)/'models/mnist-8.onnx').read_bytes(),dtype=np.uint8)
        self.cv=cv2;self.net=cv2.dnn.readNetFromONNX(model)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV);self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.previous=None;self.stable=0
        self.digit_status='WAIT CARD';self.confidence=None
    def detect(self,bgr):
        return detect_colors(bgr,self.cv)+self.detect_digits(bgr)
    def detect_digits(self,bgr):
        cv=self.cv;found=[]
        prepared=digit_input(bgr,cv);digit=None;self.confidence=None;self.digit_status='WAIT CARD'
        if prepared is not None:
            tensor,rect=prepared;self.net.setInput(tensor);scores=self.net.forward().reshape(-1)
            scores=np.exp(scores-scores.max());scores/=scores.sum();index=int(scores.argmax())
            self.confidence=round(float(scores[index]),3);self.digit_status='HOLD STILL'
            if scores[index]>=.7 and np.partition(scores,-2)[-1]-np.partition(scores,-2)[-2]>=.25:digit=index
        self.stable=self.stable+1 if digit is not None and digit==self.previous else 1
        self.previous=digit
        if digit is not None and self.stable>=2:
            self.digit_status='DIGIT %d'%digit
            found.append(dict(label='DIGIT %d %d%%'%(digit,round(float(scores[digit])*100)),rect=rect,color=(255,220,80)))
        return found


class VisionOverlay:
    """Independent latest-frame color and digit workers, gated by route state."""
    def __init__(self,folder,clock=time.monotonic,observer=None,camera_view='rear',digit_observer=None,policy_provider=None):
        self.folder=Path(folder);self.clock=clock;self.work=queue.Queue(maxsize=1);self.digit_work=queue.Queue(maxsize=1)
        self.observer=observer;self.digit_observer=digit_observer;self.policy_provider=policy_provider;self.camera_view=camera_view
        self.closed=threading.Event();self.thread=None;self.digit_thread=None;self.cv=None
        self.result=None;self.color_result=None;self.error=None;self.digit_error=None
        self.last_submit=self.last_digit_submit=-1e9;self.digit_status='WAIT CARD';self.confidence=None
        self.confirmed=None;self.current_context=None;self.current_policy=None
        self.color_processing_ms=None;self.digit_processing_ms=None;self.color_frames=0;self.digit_frames=0
    def _policy(self):return self.policy_provider() if self.policy_provider is not None else ('step_wait',None)
    @staticmethod
    def _replace(work,item):
        try:work.get_nowait()
        except queue.Empty:pass
        if item is not None:
            try:work.put_nowait(item)
            except queue.Full:pass
    def _current(self,stamp,context,policy):
        return (not self.closed.is_set() and 0<=self.clock()-stamp<=.5 and context==self.current_context
            and policy==self.current_policy and policy==self._policy())
    def _run(self):
        try:
            import cv2
            cv2.setNumThreads(1);self.cv=cv2
            while not self.closed.is_set():
                try:stamp,frame,sample,session,policy=self.work.get(timeout=.2)
                except queue.Empty:continue
                context=(session,None if sample is None else sample.get('pose_epoch'),None if sample is None else sample.get('source_clock_epoch'))
                if not self._current(stamp,context,policy):continue
                start=self.clock();found=detect_colors(frame,cv2)
                if not self._current(stamp,context,policy):continue
                self.color_processing_ms=round((self.clock()-start)*1000,2);self.color_frames+=1
                self.color_result=(stamp,found,context)
                if self.observer is not None:self.observer(stamp,found,sample,session)
        except Exception as exc:
            self.error=type(exc).__name__+': '+str(exc);print('COLOR DISPLAY UNAVAILABLE: '+self.error,flush=True)
    def _run_digits(self):
        recognizer=None;previous=None
        try:
            while not self.closed.is_set():
                try:stamp,frame,sample,session,policy=self.digit_work.get(timeout=.2)
                except queue.Empty:continue
                context=(session,None if sample is None else sample.get('pose_epoch'),None if sample is None else sample.get('source_clock_epoch'))
                if policy[0]!='step_wait' or not self._current(stamp,context,policy):continue
                if recognizer is None:recognizer=Recognizer(self.folder)
                if not self._current(stamp,context,policy):continue
                if (context,policy)!=previous:
                    recognizer.previous=None;recognizer.stable=0;previous=(context,policy)
                start=self.clock();found=recognizer.detect_digits(frame)
                if not self._current(stamp,context,policy):continue
                self.digit_processing_ms=round((self.clock()-start)*1000,2);self.digit_frames+=1
                self.result=(stamp,found,context)
                self.digit_status=recognizer.digit_status;self.confidence=recognizer.confidence
                digit=next((d['label'].split()[1] for d in found if d['label'].startswith('DIGIT ')),None)
                if digit is not None:self.confirmed=(stamp,digit,context)
                if self.digit_observer is not None:self.digit_observer(stamp,found,sample,session)
        except Exception as exc:
            self.digit_error=type(exc).__name__+': '+str(exc);print('DIGIT DISPLAY UNAVAILABLE: '+self.digit_error,flush=True)
    def overlay(self,frame,sample=None,session=0):
        now=self.clock();policy=self._policy()
        context=(session,None if sample is None else sample.get('pose_epoch'),None if sample is None else sample.get('source_clock_epoch'))
        self.current_context=context
        if self.current_policy is not None and policy!=self.current_policy:
            self.result=self.color_result=self.confirmed=None;self.confidence=None
            self.digit_status='WAIT CARD' if policy[0]=='step_wait' else 'OFF'
            self.last_submit=self.last_digit_submit=-1e9
            self._replace(self.work,None);self._replace(self.digit_work,None)
        self.current_policy=policy
        if policy[0]=='off':self.digit_status='OFF';return
        if self.thread is None:
            self.thread=threading.Thread(target=self._run,name='vision-colors',daemon=True);self.thread.start()
        if not self.error and now-self.last_submit>=.125:
            self.last_submit=now;image=frame.copy();pose=None if sample is None else dict(sample)
            self._replace(self.work,(now,image,pose,session,policy))
            if policy[0]=='step_wait' and not self.digit_error and now-self.last_digit_submit>=.5:
                self.last_digit_submit=now
                if self.digit_thread is None:
                    self.digit_thread=threading.Thread(target=self._run_digits,name='vision-digits',daemon=True);self.digit_thread.start()
                self._replace(self.digit_work,(now,image,pose,session,policy))
        result=self.result;colors=self.color_result;cv=self.cv
        if cv is None:return
        fresh=result is not None and result[2]==context and 0<=now-result[0]<=.5
        fresh_colors=colors is not None and colors[2]==context and 0<=now-colors[0]<=.5
        guide=(155,155,155);hint='COLOR'
        if policy[0]=='step_wait':
            left,top,right,bottom=digit_region(frame)
            for x,y,dx,dy in ((left,top,1,1),(right-1,top,-1,1),(left,bottom-1,1,-1),(right-1,bottom-1,-1,-1)):
                cv.line(frame,(x,y),(x+dx*8,y),guide,1);cv.line(frame,(x,y),(x,y+dy*8),guide,1)
            hint=self.digit_status if fresh and self.digit_status.startswith('DIGIT') else 'DIGIT: WAIT'
            confirmed=self.confirmed
            if confirmed is not None and confirmed[2]==context and 0<=now-confirmed[0]<=2:
                current=fresh and any(d['label'].startswith('DIGIT '+confirmed[1]+' ') for d in result[1])
                hint=('DIGIT ' if current else 'LAST ')+confirmed[1]
            if self.digit_error:hint='DIGIT OFFLINE'
        if self.error:hint='COLOR OFFLINE'
        cv.putText(frame,hint,(4,12),cv.FONT_HERSHEY_SIMPLEX,.34,guide,1,cv.LINE_AA)
        cv.putText(frame,'FRONT' if self.camera_view=='forward' else 'REAR',(frame.shape[1]-43,12),cv.FONT_HERSHEY_SIMPLEX,.34,guide,1,cv.LINE_AA)
        detections=(colors[1] if fresh_colors else [])+(result[1] if fresh and policy[0]=='step_wait' else [])
        for detection in detections:
            x,y,w,h=detection['rect'];color=detection['color']
            cv.rectangle(frame,(x,y),(x+w-1,y+h-1),color,1)
            cv.putText(frame,detection['label'],(x,max(12,y-4)),cv.FONT_HERSHEY_SIMPLEX,.35,color,1,cv.LINE_AA)
    def close(self):
        self.closed.set()
        for worker in (self.thread,self.digit_thread):
            if worker is not None:worker.join(timeout=2)
    def snapshot(self):
        result=self.result;colors=self.color_result;confirmed=self.confirmed;now=self.clock();policy=self._policy()
        held=policy[0]=='step_wait' and not self.digit_error and confirmed is not None and confirmed[2]==self.current_context and 0<=now-confirmed[0]<=2
        latest=colors or result
        return dict(backend='ARM HSV / OpenCV DNN',camera_view=self.camera_view,error=self.error,digit_error=self.digit_error,
            policy=policy[0],digit_enabled=policy[0]=='step_wait',digit_status=self.digit_status if policy[0]=='step_wait' else 'OFF',digit_confidence=self.confidence if policy[0]=='step_wait' else None,
            display_digit=confirmed[1] if held else None,display_digit_age_s=round(now-confirmed[0],2) if held else None,
            colors=[] if colors is None or policy[0]=='off' or now-colors[0]>.5 else [d['label'] for d in colors[1]],
            color_processing_ms=self.color_processing_ms,digit_processing_ms=self.digit_processing_ms,color_frames=self.color_frames,digit_frames=self.digit_frames,
            result_age_s=None if latest is None else round(now-latest[0],2))
