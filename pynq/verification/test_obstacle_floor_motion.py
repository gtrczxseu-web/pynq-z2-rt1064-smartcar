"""Captured bypass-side floor and bounded exposure regressions; no hardware."""
from pathlib import Path
import sys,unittest
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
from visual_obstacle import GroundDetector
POLYGON=[[0,.67],[1,.67],[1,1],[0,1]]

def image(name):
    return cv2.imdecode(np.frombuffer((ROOT/'verification/data'/name).read_bytes(),np.uint8),cv2.IMREAD_COLOR)

class MovingFloorTests(unittest.TestCase):
    def detector(self):
        d=GroundDetector(POLYGON)
        for unused in range(3):d.detect(image('obstacle_empty_floor_20261101.png'),True)
        self.assertIsNotNone(d.reference);return d
    def test_shifted_camera_scene_cup_at_left_does_not_block_empty_central_lane(self):
        d=self.detector();reference=d.reference.copy()
        for index in range(5):
            self.assertEqual(d.detect(image('bypass_side_floor_%d_20261101.png'%index))['state'],'CLEAR',index)
        np.testing.assert_array_equal(d.reference,reference)
    def test_spatial_floor_consistency_allows_three_consecutive_idle_samples(self):
        d=GroundDetector(POLYGON)
        for index in range(3):result=d.detect(image('bypass_side_floor_%d_20261101.png'%index),True)
        self.assertEqual(result['state'],'CLEAR');self.assertIsNotNone(d.reference)
    def test_captured_shifted_floor_under_previous_brighter_exposure_is_clear(self):
        d=GroundDetector(POLYGON)
        for unused in range(3):d.detect(np.full((240,320,3),170,np.uint8),True)
        reference=d.reference.copy()
        for index in range(5):
            self.assertEqual(d.detect(image('bypass_side_floor_%d_20261101.png'%index))['state'],'CLEAR',index)
        np.testing.assert_array_equal(d.reference,reference)
    def test_central_obstacle_still_blocks_on_shifted_floor_without_reference_learning(self):
        for color in ((20,20,20),(200,0,0),(0,0,200),(0,180,0)):
            d=self.detector();reference=d.reference.copy()
            frame=image('bypass_side_floor_4_20261101.png');frame[165:220,130:190]=color
            self.assertEqual(d.detect(frame)['state'],'BLOCKED',color)
            np.testing.assert_array_equal(d.reference,reference)
    def test_compensation_is_bounded_and_full_width_dark_block_is_not_learned_away(self):
        d=GroundDetector(POLYGON);floor=np.full((240,320,3),170,np.uint8)
        for unused in range(3):d.detect(floor,True)
        reference=d.reference.copy();frame=floor.copy();frame[160:]=(20,20,20)
        for unused in range(5):self.assertEqual(d.detect(frame)['state'],'BLOCKED')
        np.testing.assert_array_equal(d.reference,reference)

if __name__=='__main__':unittest.main()
