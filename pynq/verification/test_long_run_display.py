"""Projection parity and display caching; full recording stays unchanged."""
import json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import cv2
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
from rear_projection import project,project_many,defaults,RearProjection
from trajectory_report import RouteReport,map_image
from telemetry_reference import VEHICLE_INITIAL,CAMERA_INITIAL
from test_trajectory_report import pose
from route_settings import preset
from vision_signals import SignalCards
from light_vision import Recognizer

class BatchProjectionTests(unittest.TestCase):
    def test_vector_projection_matches_scalar_for_both_frames_calibrations_and_mirror(self):
        rng=np.random.default_rng(4);points=rng.uniform(-3,3,(800,2));current=(.3,-.2)
        for ref in (VEHICLE_INITIAL,CAMERA_INITIAL):
            for heading in (-180,-90,0,45,90,180):
                for mode in ('approximate','homography'):
                    for mirror in (False,True):
                        c=defaults();c.update(mode=mode,mirror=mirror,camera_right_m=.04)
                        if mode=='homography':c['homography']=[[.3,.02,.5],[0,.4,.3],[.01,.3,1]]
                        pixels,valid=project_many(points,current,heading,c,reference_frame=ref)
                        expected=[project(p,current,heading,c,reference_frame=ref) for p in points]
                        np.testing.assert_array_equal(valid,[p is not None for p in expected])
                        np.testing.assert_array_equal(pixels[valid],np.asarray([p for p in expected if p is not None]))
    def test_empty_points_and_homography_zero_denominator(self):
        c=defaults();pixels,valid=project_many([], (0,0),0,c);self.assertEqual(pixels.shape,(0,2))
        c.update(mode='homography',homography=[[1,0,0],[0,1,0],[0,0,0]])
        pixels,valid=project_many([(0,-1)],(0,0),0,c);self.assertFalse(valid[0])
    def test_batch_projection_does_not_bridge_data_gaps_or_camera_invisible_points(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=RearProjection(tmp,clock=lambda:1)
            points=[(-.1,-1,0,True,False),(.1,-1,0,True,True)]
            frame=np.zeros((240,320,3),np.uint8);p.overlay(frame,points,(0,0),0)
            self.assertFalse(frame[180:210,140:180].any())
            p.last_render=-1e9;points[1]=(.1,-1,0,True,False);frame[:]=0;p.overlay(frame,points,(0,0),0)
            self.assertTrue(frame[180:210,140:180].any())

class MapCacheTests(unittest.TestCase):
    def test_unchanged_finished_scene_is_cached_but_new_samples_still_render(self):
        with tempfile.TemporaryDirectory() as tmp:
            now=[1.];r=RouteReport(tmp,clock=lambda:now[0],minimal=True)
            try:
                with patch('trajectory_report.map_image',wraps=map_image) as render:
                    r.poll();self.assertEqual(render.call_count,1)
                    for unused in range(4):now[0]+=.6;r.poll()
                    self.assertEqual(render.call_count,1)
                    r.event(dict(kind='start_requested',monotonic=now[0],session=12),pose(now[0]));r.poll()
                    now[0]+=.6;r.pose(pose(now[0],y=.2,d=.2));r.poll()
                    self.assertGreater(render.call_count,1)
                    r.event(dict(kind='stop',monotonic=now[0],reason='Offline cache test'));r.poll()
                    now[0]+=.6;r.poll();count=render.call_count
                    for unused in range(4):now[0]+=.6;r.poll()
                    self.assertEqual(render.call_count,count)
                    self.assertEqual(r.snapshot()['pending_items'],0)
            finally:r.close()
    def test_large_map_does_not_modify_recorded_points_and_keeps_validity_colors(self):
        points=[(i*.002,.5,0,True,False) for i in range(2000)];before=list(points)
        frame=np.asarray(map_image(points,minimal=True,size=(280,240)))
        self.assertEqual(points,before);self.assertTrue(np.any(np.all(frame==(80,194,255),axis=2)))
        amber=np.asarray(map_image([(0,0,0,True,False),(1,1,0,False,False)],minimal=True,size=(280,240)))
        self.assertTrue(np.any(np.all(amber==(255,189,88),axis=2)))

class PlannedPathTests(unittest.TestCase):
    def report(self,folder,recipe):
        r=RouteReport(folder,clock=lambda:1,reference_frame=CAMERA_INITIAL)
        r.run=dict(recipe=recipe,steps=[],initial_pose=dict(heading_deg=0));r.step=dict(index=0)
        self.addCleanup(lambda:setattr(r,'run',None))
        return r
    def test_remaining_current_move_is_bounded_and_independent_of_past_points(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=self.report(tmp,preset('rectangle',4,4));points,label=r.planned_path((.1,.4),0)
            self.assertEqual(label,'PLAN FORWARD');self.assertLessEqual(len(points),26)
            self.assertAlmostEqual(points[-1][0],.1);self.assertAlmostEqual(points[-1][1],2.9)
            r.points=[(i*.1,i*.2,0,True,False) for i in range(2000)]
            self.assertEqual(r.planned_path((.1,.4),0),(points,label))
    def test_camera_hint_follows_actual_body_direction_instead_of_always_forward(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=self.report(tmp,preset('rectangle',1,1))
            for index,position,heading,label in ((0,(0,0),0,'PLAN FORWARD'),(1,(0,1),0,'PLAN RIGHT'),
                    (2,(1,1),0,'PLAN REVERSE'),(3,(1,0),0,'PLAN LEFT'),(1,(0,1),-90,'PLAN FORWARD')):
                r.step['index']=index;points,actual=r.planned_path(position,heading)
                self.assertEqual(actual,label);self.assertTrue(points)
    def test_start_heading_rotates_the_plan_and_turns_have_no_fake_translation(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=self.report(tmp,preset('competition',1));r.run['initial_pose']['heading_deg']=90
            points,label=r.planned_path((0,0),90);self.assertAlmostEqual(points[-1][0],-1)
            self.assertAlmostEqual(points[-1][1],0);self.assertEqual(label,'PLAN FORWARD')
            r.step['index']=5;self.assertEqual(r.planned_path((0,0),0),([],'PLAN TURN'))
    def test_camera_overlay_uses_plan_without_turn_history_and_leaves_measurements_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=self.report(tmp,preset('rectangle',1,1));r.points=[(0,.2,0,True,False)];r.previous=dict(t=1,heading=0)
            r.turn_markers=[dict(x_m=.2,y_m=.2)];before=list(r.points)
            with patch.object(r.projection,'overlay') as draw:r.camera_overlay(np.zeros((240,320,3),np.uint8))
            self.assertEqual(draw.call_args.args[4],());self.assertEqual(draw.call_args.kwargs['caption'],'PLAN FORWARD')
            self.assertEqual(r.points,before)
    def test_waiting_state_previews_next_move_and_idle_state_has_no_plan(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=self.report(tmp,preset('rectangle',1,1));r.step=None;r.run['steps']=[dict(index=0)]
            self.assertEqual(r.planned_path((0,1),0)[1],'PLAN RIGHT')
            r.run=None;self.assertEqual(r.planned_path((0,0),0),([],'PLAN WAIT START'))

class SlowCameraTests(unittest.TestCase):
    def test_two_fresh_observations_survive_slow_camera_interval_but_duplicates_do_not(self):
        cards=SignalCards();red=[dict(label='RED')]
        self.assertEqual(cards.update(0,red)[0],set());self.assertEqual(cards.update(.8,red)[0],{'RED'})
        self.assertEqual(cards.update(.8,red)[0],set())
        cards=SignalCards();cards.update(0,red);self.assertEqual(cards.update(1.1,red)[0],set())

class CapturedColorTests(unittest.TestCase):
    def test_four_actual_color_samples_with_blur_and_lower_brightness(self):
        for color in ('red','green','blue','yellow'):
            frame=cv2.imdecode(np.frombuffer((PACKAGE/('verification/data/traffic_'+color+'_20261028.png')).read_bytes(),np.uint8),1)
            for image in (frame,cv2.GaussianBlur(frame,(9,9),0),(frame*.65).astype(np.uint8)):
                found=Recognizer(PACKAGE).detect(image)
                self.assertEqual([d['label'] for d in found if d['label'] in ('RED','GREEN','BLUE','YELLOW')],[color.upper()])
    def test_actual_room_background_is_still_rejected(self):
        frame=cv2.imdecode(np.frombuffer((PACKAGE/'verification/data/traffic_background_20261028.png').read_bytes(),np.uint8),1)
        self.assertEqual([d['label'] for d in Recognizer(PACKAGE).detect(frame) if d['label'] in ('RED','GREEN','BLUE','YELLOW')],[])

if __name__=='__main__':unittest.main()
