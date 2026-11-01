"""Cached 32-bit video conversion. Geometry/LUT/interpolation match the reference."""
import cv2
import numpy as np

def to_bgrx(source,destination):
    cv2.cvtColor(source,cv2.COLOR_BGR2BGRA,dst=destination)
    destination[:,:,3]=0

def copy_to_dma(output,cached):
    np.copyto(output,cached)

class FrameRenderer:
    def __init__(self):
        self.shape=None
        self.native=None
        self.canvas=None
        self.target=None
        self.reallocations=0

    def copy(self,output,bgr,correct):
        if output.shape!=(480,640,4) or output.dtype!=np.uint8:
            raise ValueError('Expected a 640x480 BGRX output')
        if bgr.ndim!=3 or bgr.shape[2]!=3 or bgr.dtype!=np.uint8:
            raise ValueError('Camera data must be uint8 BGR with three channels')
        h,w=bgr.shape[:2]
        if h<=0 or w<=0:raise ValueError('Camera image is empty')
        if self.shape!=(h,w):
            scale=min(640/w,480/h)
            rw,rh=max(1,round(w*scale)),max(1,round(h*scale))
            x,y=(640-rw)//2,(480-rh)//2
            self.native=np.empty((h,w,4),dtype=np.uint8)
            self.canvas=np.zeros((480,640,4),dtype=np.uint8)
            self.target=self.canvas[y:y+rh,x:x+rw]
            self.target_size=(rw,rh)
            self.shape=(h,w)
            self.reallocations+=1
        corrected=correct(bgr)
        to_bgrx(corrected,self.native)
        if self.target_size==(w,h):
            np.copyto(self.target,self.native)
        else:
            cv2.resize(self.native,self.target_size,dst=self.target,interpolation=cv2.INTER_LINEAR)
        copy_to_dma(output,self.canvas)

renderer=FrameRenderer()
