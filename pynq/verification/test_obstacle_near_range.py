"""Measured 50 cm boundary, far reflections and near objects; no hardware."""
from pathlib import Path
import json,sys,tempfile,unittest
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
from visual_obstacle import GroundDetector,ObstacleOverlay,NEAR_BOUNDARY_Y
POLYGON=[[0,.67],[1,.67],[1,1],[0,1]]

def image(kind,index):
    path=ROOT/'verification/data'/('near50_%s_%d_20261101.png'%(kind,index))
    return cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)

def reflected_cup(bottom,color):
    frame=np.full((240,320,3),170,np.uint8);top=bottom-75
    frame[top:bottom,140:180]=color
    # A sharp contact edge and a dimmer, blurred mirror continuation on the floor.
    reflection=frame[top:bottom,140:180][::-1].copy()
    reflection=np.uint8(reflection.astype(np.float32)*.45+170*.55)
    reflection=cv2.GaussianBlur(reflection,(5,5),0)
    length=min(240-bottom-2,len(reflection))
    frame[bottom+2:bottom+2+length,140:180]=reflection[:length]
    return frame

class NearRangeTests(unittest.TestCase):
    def detector(self):
        d=GroundDetector(POLYGON)
        for index in range(3):d.detect(image('empty',index),True)
        self.assertIsNotNone(d.reference);return d
    def test_measured_contact_is_stable_and_marker_matches_saved_calibration(self):
        c=json.loads((ROOT/'verification/data/near50_calibration_20261101.json').read_text('utf8'))
        rows=[]
        for index in range(8):
            gray=cv2.cvtColor(image('cup50',index),cv2.COLOR_BGR2GRAY)
            edge=np.mean(np.abs(cv2.Sobel(gray,cv2.CV_32F,0,1,ksize=3))[:,154:175],axis=1)
            rows.append(170+int(np.argmax(edge[170:186])))
        self.assertEqual(rows,c['contact_rows']);self.assertEqual(round(NEAR_BOUNDARY_Y*239),int(np.median(rows)))
        self.assertEqual(c['measured_distance_m'],.5)
    def test_all_measured_50cm_cup_frames_block_without_box_following_reflection_tail(self):
        d=self.detector()
        for index in range(8):
            result=d.detect(image('cup50',index));self.assertEqual(result['state'],'BLOCKED',index)
            x,y,w,h=result['rect'];self.assertTrue(176<=y+h<=186,(index,result))
    def test_all_one_metre_cup_and_reflection_frames_are_outside_detection_range(self):
        d=self.detector()
        for index in range(8):self.assertEqual(d.detect(image('cup100',index))['state'],'CLEAR',index)
    def test_captured_empty_central_floor_remains_clear(self):
        d=self.detector()
        for index in range(5):self.assertEqual(d.detect(image('empty',index))['state'],'CLEAR',index)
    def test_far_object_reflection_extending_inside_near_zone_does_not_trigger(self):
        for color in ((20,20,20),(50,110,200),(0,0,220)):
            d=GroundDetector(POLYGON)
            for unused in range(3):d.detect(np.full((240,320,3),170,np.uint8),True)
            self.assertEqual(d.detect(reflected_cup(168,color))['state'],'CLEAR',color)
    def test_near_body_still_blocks_and_box_ends_at_contact_instead_of_reflection(self):
        for color in ((20,20,20),(50,110,200),(0,0,220)):
            d=GroundDetector(POLYGON)
            for unused in range(3):d.detect(np.full((240,320,3),170,np.uint8),True)
            result=d.detect(reflected_cup(196,color));self.assertEqual(result['state'],'BLOCKED',color)
            x,y,w,h=result['rect'];self.assertLessEqual(y+h,200)
    def test_body_above_lane_with_foot_outside_the_travel_corridor_is_ignored(self):
        d=GroundDetector(POLYGON);frame=np.full((240,320,3),170,np.uint8)
        for unused in range(3):d.detect(frame,True)
        cv2.fillConvexPoly(frame,np.array([[130,158],[210,158],[100,220],[20,220]],np.int32),(20,20,20))
        self.assertEqual(d.detect(frame)['state'],'CLEAR')
    def test_boundary_marker_is_visible_even_without_a_fresh_detection(self):
        with tempfile.TemporaryDirectory() as folder:
            policy=[('learn',(0,0,1,0))]
            overlay=ObstacleOverlay(folder,lambda:policy[0],lambda *args:None)
            try:
                for mode in (('learn',(0,0,1,0)),('off',None)):
                    policy[0]=mode;frame=np.full((240,320,3),170,np.uint8);overlay.draw(frame)
                    self.assertGreater(np.count_nonzero(np.any(frame[179]!=(170,170,170),axis=1)),300)
                self.assertFalse(overlay.snapshot()['range_estimated']);self.assertTrue(overlay.snapshot()['boundary_calibrated'])
            finally:overlay.close()

if __name__=='__main__':unittest.main()
