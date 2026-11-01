"""Recorded cup and synthetic exposure/blur regressions; no hardware access."""
import sys,unittest
from unittest.mock import patch
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
from visual_obstacle import GroundDetector
POLYGON=[[0,.67],[1,.67],[1,1],[0,1]]

def image(name):
    return cv2.imdecode(np.frombuffer((ROOT/'verification/data'/name).read_bytes(),np.uint8),cv2.IMREAD_COLOR)

class CupTests(unittest.TestCase):
    def detector(self):
        # Appearance regressions retain their original full-floor field of view;
        # measured 50 cm range/reflection checks have separate captured fixtures.
        with patch('visual_obstacle.NEAR_BOUNDARY_Y',.67):d=GroundDetector(POLYGON)
        for unused in range(3):d.detect(image('obstacle_empty_floor_20261101.png'),True)
        return d
    def test_weaker_cup_base_survives_noise_filtering(self):
        d=self.detector();frame=image('obstacle_cup_00_20261101.png')
        frame=np.uint8(np.clip(frame.astype(np.float32)*.9,0,255))
        self.assertEqual(d.detect(frame,True)['state'],'BLOCKED')
    def test_recorded_cups_under_brightness_and_light_blur_variations(self):
        for gain in (.9,1.,1.1):
            for blur in (0,3):
                d=self.detector()
                for index in (0,4,20):
                    frame=image('obstacle_cup_%02d_20261101.png'%index)
                    frame=np.uint8(np.clip(frame.astype(np.float32)*gain,0,255))
                    if blur:frame=cv2.GaussianBlur(frame,(blur,blur),0)
                    self.assertEqual(d.detect(frame,True)['state'],'BLOCKED',(gain,blur,index))
    def test_empty_floor_after_cup_and_thin_floor_marks_remain_clear(self):
        d=self.detector();self.assertEqual(d.detect(image('obstacle_cup_20_20261101.png'),True)['state'],'BLOCKED')
        empty=image('obstacle_empty_floor_20261101.png')
        for gain in (.9,1.):
            frame=np.uint8(np.clip(empty.astype(np.float32)*gain,0,255))
            for unused in range(3):result=d.detect(frame,True)
            self.assertEqual(result['state'],'CLEAR')
        d=GroundDetector(POLYGON);empty=np.full((240,320,3),170,np.uint8)
        for unused in range(3):d.detect(empty,True)
        empty[201:203]=(20,20,20);empty[180:185,150:155]=(20,20,20)
        self.assertEqual(d.detect(empty)['state'],'CLEAR')
    def test_bright_far_floor_is_not_mistaken_for_an_obstacle(self):
        d=self.detector();frame=image('obstacle_empty_floor_20261101.png')
        frame=np.uint8(np.clip(frame.astype(np.float32)*1.1,0,255))
        for unused in range(3):result=d.detect(frame,True)
        self.assertEqual(result['state'],'CLEAR')

if __name__=='__main__':unittest.main()
