import sys,unittest
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
from light_vision import Recognizer,VisionOverlay
from boot_display import BootDisplay

class MovingDigitTests(unittest.TestCase):
    def test_native_panel_preserves_camera_geometry_and_does_not_mutate_input(self):
        d=BootDisplay();d.update('DRIVING','');frame=np.zeros((240,320,3),np.uint8)
        frame[:,:,0]=np.arange(320,dtype=np.uint16).astype(np.uint8)
        original=frame.copy();output=np.empty((480,640,4),np.uint8)
        d.render_camera(output,frame)
        np.testing.assert_array_equal(output[88:328,12:332,:3],original)
        np.testing.assert_array_equal(frame,original)
        self.assertTrue((output[:,:,3]==0).all())

    def test_real_digit_card_remains_detectable_as_it_crosses_the_view(self):
        image=cv2.imdecode(np.frombuffer((PACKAGE/'verification/data/digit_card_2_20261027.png').read_bytes(),np.uint8),1)
        for offset in (-80,-60,-40,0,40,60,80):
            frame=cv2.warpAffine(image,np.float32([[1,0,offset],[0,1,0]]),(320,240),borderMode=cv2.BORDER_REPLICATE)
            r=Recognizer(PACKAGE);r.detect(frame)
            digits=[d['label'].split()[1] for d in r.detect(frame) if d['label'].startswith('DIGIT')]
            self.assertEqual(digits,['2'],offset)

    def test_waiting_frame_is_replaced_with_latest_image_and_pose(self):
        now=[0.0];v=VisionOverlay(PACKAGE,lambda:now[0]);v.thread=object();v.digit_thread=object()
        frame=np.zeros((240,320,3),np.uint8)
        v.overlay(frame,{'x_m':1},11)
        now[0]=.21;frame[:]=80;v.overlay(frame,{'x_m':2},12)
        self.assertEqual(v.work.qsize(),1)
        stamp,image,sample,session,policy=v.work.get_nowait()
        self.assertEqual((stamp,sample,session),(.21,{'x_m':2},12))
        self.assertTrue((image==80).all())

    def test_old_detection_box_is_not_drawn_on_a_new_camera_view(self):
        v=VisionOverlay(PACKAGE,lambda:.6);v.thread=object();v.digit_thread=object();v.cv=cv2
        v.result=(0,[dict(label='DIGIT 2',rect=(100,70,20,30),color=(0,255,0))],(0,None,None))
        frame=np.zeros((240,320,3),np.uint8);v.overlay(frame)
        self.assertEqual(frame[70:100,100:120].sum(),0)

if __name__=='__main__':unittest.main()
