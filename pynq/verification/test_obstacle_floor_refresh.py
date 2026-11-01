"""Idle reference refresh must not learn away a central obstacle or motion."""
import sys,unittest
from unittest.mock import patch
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
from visual_obstacle import GroundDetector
POLYGON=[[0,.67],[1,.67],[1,1],[0,1]]

class FloorRefreshTests(unittest.TestCase):
    def detector(self):
        d=GroundDetector(POLYGON)
        for unused in range(3):d.detect(np.full((240,320,3),170,np.uint8),learn=True)
        return d
    def test_idle_brightness_drift_recovers_after_three_stable_floor_samples(self):
        d=self.detector();frame=np.full((240,320,3),100,np.uint8)
        for unused in range(3):result=d.detect(frame,learn=True)
        self.assertEqual(result['state'],'CLEAR')
    def test_motion_compensates_floor_exposure_without_updating_reference(self):
        d=self.detector();reference=d.reference.copy()
        for unused in range(8):result=d.detect(np.full((240,320,3),100,np.uint8))
        np.testing.assert_array_equal(d.reference,reference)
        self.assertEqual(result['state'],'CLEAR')
    def test_idle_refresh_does_not_learn_central_obstacle_and_storage_is_bounded(self):
        d=self.detector();frame=np.full((240,320,3),170,np.uint8);frame[165:220,130:190]=(25,25,25)
        for unused in range(20):self.assertEqual(d.detect(frame,learn=True)['state'],'BLOCKED')
        self.assertLessEqual(len(d.samples),3)
    def test_actual_empty_floor_recovers_from_a_previous_different_reference(self):
        image=cv2.imdecode(np.frombuffer((ROOT/'verification/data/obstacle_empty_floor_20261101.png').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        d=self.detector()
        for unused in range(3):result=d.detect(image,learn=True)
        self.assertEqual(result['state'],'CLEAR')
    def test_actual_bottle_base_is_detected_without_learning_it_as_floor(self):
        # Check appearance learning independently of the new measured range gate.
        with patch('visual_obstacle.NEAR_BOUNDARY_Y',.67):d=GroundDetector(POLYGON)
        empty=cv2.imdecode(np.frombuffer((ROOT/'verification/data/obstacle_empty_floor_20261101.png').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        for unused in range(3):d.detect(empty,learn=True)
        for i in (2,3,4):
            frame=cv2.imdecode(np.frombuffer((ROOT/('verification/data/obstacle_bottle_%d_20261101.png'%i)).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
            self.assertEqual(d.detect(frame,learn=True)['state'],'BLOCKED',i)
        self.assertEqual(d.detect(empty,learn=True)['state'],'CLEAR')

if __name__=='__main__':unittest.main()
