"""Synthetic camera/controller tests. No camera device, GPIO, UART or motors."""
import sys,tempfile,threading,unittest
from pathlib import Path
from unittest.mock import patch
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
from visual_obstacle import GroundDetector,ObstacleOverlay,forward_step
from motion_link import MotionLink,decode_status
from route_settings import preset
from test_motion import Clock,Writer,status_packet
from standalone import Application
from boot_display import BootDisplay
from light_vision import detect_colors

POLYGON=[[0,.67],[1,.67],[1,1],[0,1]]
def floor():return np.full((240,320,3),170,np.uint8)

class DetectorTests(unittest.TestCase):
    def learned(self):
        detector=GroundDetector(POLYGON)
        for unused in range(3):result=detector.detect(floor(),True)
        self.assertEqual(result['state'],'CLEAR');return detector
    def test_no_reference_is_unknown_until_three_stationary_frames(self):
        d=GroundDetector(POLYGON)
        for unused in range(5):self.assertEqual(d.detect(floor())['state'],'UNKNOWN')
        self.assertEqual(d.detect(floor(),True)['state'],'UNKNOWN')
        self.assertEqual(d.detect(floor(),True)['state'],'UNKNOWN')
        self.assertEqual(d.detect(floor(),True)['state'],'CLEAR')
    def test_visible_colored_and_dark_obstacles_inside_lane(self):
        for color in ((255,0,0),(0,0,200),(35,35,35),(70,110,160)):
            d=self.learned();frame=floor();frame[165:220,115:205]=color
            result=d.detect(frame);self.assertEqual(result['state'],'BLOCKED',color)
            self.assertIsNotNone(result['rect']);self.assertEqual(d.detect(floor())['state'],'CLEAR')
    def test_object_above_floor_or_outside_lane_and_thin_tile_seams_do_not_block(self):
        d=self.learned()
        for box in ((110,30,210,140),(5,165,40,220)):
            frame=floor();x,y,right,bottom=box;frame[y:bottom,x:right]=(255,0,0)
            self.assertEqual(d.detect(frame)['state'],'CLEAR')
        frame=floor();frame[201:203]=(20,20,20);self.assertEqual(d.detect(frame)['state'],'CLEAR')
    def test_small_illumination_change_and_same_color_object_are_not_claimed_as_depth(self):
        d=self.learned();self.assertEqual(d.detect(np.full((240,320,3),180,np.uint8))['state'],'CLEAR')
        # A camera appearance detector cannot distinguish an identical surface.
        frame=floor();frame[165:220,115:205]=(170,170,170)
        self.assertEqual(d.detect(frame)['state'],'CLEAR')
    def test_only_actions_facing_camera_are_eligible(self):
        r=preset('competition');self.assertEqual([i for i in range(12) if forward_step(r,i)],[0,4,6,8,10])
        self.assertFalse(forward_step(preset('rotate'),0));self.assertFalse(forward_step(r,12))
    def test_recorded_empty_floor_and_synthetic_obstacle_replay(self):
        for name in ('traffic_background_20261028.png','stationary_background_20261027.png'):
            frame=cv2.imdecode(np.frombuffer((PACKAGE/'verification/data'/name).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
            d=GroundDetector(POLYGON)
            for unused in range(3):result=d.detect(frame,True)
            self.assertEqual(result['state'],'CLEAR',name)
            for color in ((255,0,0),(35,35,35),(70,110,160)):
                obstacle=frame.copy();obstacle[165:220,115:205]=color
                self.assertEqual(d.detect(obstacle)['state'],'BLOCKED',(name,color))
            self.assertEqual(d.detect(frame)['state'],'CLEAR')

class ControlTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.writer=Writer();self.m=MotionLink(writer=self.writer,clock=self.clock)
        self.m.recipe=preset('translate',speed=40);self.m.session=123;self.m.sequence=100;self.m.run_started=90
        self.state=3;self.index=0;self.ticks=0;self.m.state='DRIVING';self.fresh()
    def fresh(self,state=None,index=None):
        if state is not None:self.state=state
        if index is not None:self.index=index
        self.ticks+=5;self.m.receive(decode_status(status_packet(self.state,123,self.m.sequence,self.index,ticks=self.ticks,version=7),self.clock()))
        self.m.receive_pose(dict(received_monotonic=self.clock(),pose_epoch=1,source_clock_epoch=0,pose_valid=True,speed_m_s=0))
    def obstacle(self,state,age=0,session=123,index=None,epoch=1):
        self.clock.now+=.2;self.fresh()
        self.m.receive_obstacle(self.clock()-age,dict(state=state,rect=None),dict(pose_epoch=epoch,source_clock_epoch=0),session,self.index if index is None else index);self.m.step()
    def green(self,age=0):
        self.clock.now+=.2;self.fresh();self.m.receive_vision(self.clock()-age,[dict(label='GREEN')],session=123);self.m.step()
    def held(self):
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.assertTrue(self.m.obstacle_hold)
        self.fresh(11);self.m.step();self.assertEqual(self.m.state,'SIGNAL_WAIT')
    def clear(self):
        for unused in range(3):self.obstacle('CLEAR')
    def test_two_distinct_frames_hold_and_button_releases_same_action(self):
        self.held();self.assertEqual(sum(p[3]==5 for p in self.writer.packets),1)
        self.clear();self.assertTrue(self.m.toggle());self.m.step();self.assertEqual(self.writer.packets[-1][3],6)
        self.fresh(3);self.m.step();self.assertEqual(self.m.state,'DRIVING')
        self.assertFalse(any(p[3] in (1,2,4) for p in self.writer.packets));self.assertEqual(self.m.mcu['step'],0)
    def test_clear_alone_never_auto_releases_and_two_clear_frames_are_not_enough(self):
        self.held();self.obstacle('CLEAR');self.obstacle('CLEAR');self.assertFalse(self.m.toggle())
        self.obstacle('CLEAR');self.assertTrue(self.m.snapshot()['obstacle_clear'])
        for unused in range(5):self.obstacle('CLEAR')
        self.assertTrue(self.m.obstacle_hold);self.assertNotIn(6,[p[3] for p in self.writer.packets])
    def test_green_cannot_bypass_obstacle_but_same_card_can_confirm_after_clear(self):
        self.held();self.green();self.green();self.assertTrue(self.m.obstacle_hold)
        self.clear();self.green();self.assertTrue(self.m.obstacle_hold);self.green()
        self.assertFalse(self.m.obstacle_hold);self.assertEqual(self.writer.packets[-1][3],6)
    def test_stale_clear_unknown_new_obstacle_or_stale_mcu_cannot_resume(self):
        for condition in ('stale','unknown','blocked','mcu','epoch'):
            self.setUp();self.held();self.clear()
            if condition=='stale':self.clock.now+=.6
            elif condition=='mcu':self.m.mcu['received_monotonic']=self.clock()-1
            elif condition=='epoch':self.m.latest['pose_epoch']=2
            else:self.obstacle(condition.upper())
            self.assertFalse(self.m.toggle(),condition);self.assertTrue(self.m.obstacle_hold)
    def test_duplicate_stamps_wrong_session_step_epoch_and_stale_frames_are_ignored(self):
        for kw in (dict(age=.6),dict(session=124),dict(index=1),dict(epoch=2)):
            self.setUp();self.obstacle('BLOCKED',**kw);self.obstacle('BLOCKED',**kw);self.assertFalse(self.m.obstacle_hold)
        self.setUp();self.obstacle('BLOCKED');stamp=self.clock()
        self.m.receive_obstacle(stamp,dict(state='BLOCKED'),None,123,0);self.m.step();self.assertFalse(self.m.obstacle_hold)
    def test_ready_paused_turn_side_and_done_frames_cannot_trigger_hold(self):
        for state in (0,9,5,6):
            self.setUp();self.fresh(state);self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.assertFalse(self.m.obstacle_hold)
        self.setUp();self.m.recipe=preset('rectangle');self.index=1
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.assertFalse(self.m.obstacle_hold)
    def test_continuous_green_releases_red_only_after_obstacle_is_clear(self):
        self.m.recipe['mode']='continuous';self.held();self.m.traffic_reason='RED'
        self.green();self.green();self.assertEqual(self.m.traffic_reason,'RED')
        self.clear();self.green();self.green();self.assertIsNone(self.m.traffic_reason);self.assertFalse(self.m.obstacle_hold)
        self.assertEqual(self.writer.packets[-1][3],6)
    def test_button_cannot_override_red_or_yellow_and_yellow_still_waits_five_seconds(self):
        self.m.recipe['mode']='continuous';self.held();self.clear();self.m.traffic_reason='RED'
        self.assertFalse(self.m.toggle());self.m.traffic_reason='YELLOW';self.m.yellow_at=self.clock()
        self.assertFalse(self.m.toggle());self.green();self.green();self.assertEqual(self.m.traffic_reason,'YELLOW')
    def test_stop_and_completed_run_clear_latch_without_any_visual_restart(self):
        self.held();self.m.stop('BTN1');self.fresh(7);self.clear();self.green();self.green()
        self.assertFalse(self.m.obstacle_hold);self.assertEqual(self.m.state,'STOPPED')
        self.assertFalse(any(p[3] in (1,2,4,6) for p in self.writer.packets))
    def test_new_obstacle_during_release_reasserts_hold(self):
        self.held();self.clear();self.assertTrue(self.m.toggle());self.m.step()
        self.obstacle('BLOCKED');self.obstacle('BLOCKED')
        self.assertTrue(self.m.obstacle_hold);self.assertEqual(self.writer.packets[-1][3],5)
    def test_green_card_above_floor_region_can_release_after_actual_clear_detection(self):
        self.held();detector=GroundDetector(POLYGON)
        for unused in range(3):detector.detect(floor(),True)
        frame=floor();frame[40:95,130:190]=(0,220,0)
        result=detector.detect(frame);self.assertEqual(result['state'],'CLEAR')
        for unused in range(3):self.obstacle(result['state'])
        colors=detect_colors(frame,cv2);self.assertEqual([d['label'] for d in colors],['GREEN'])
        for unused in range(2):
            self.clock.now+=.15;self.fresh();self.m.receive_vision(self.clock(),colors,session=123);self.m.step()
        self.assertEqual(self.writer.packets[-1][3],6)
    def test_late_green_from_before_clear_cannot_resume(self):
        self.held();self.clear();self.green(age=.4);self.assertTrue(self.m.obstacle_hold)
        self.assertNotIn(6,[p[3] for p in self.writer.packets])
    def test_interrupted_countdown_does_not_display_a_new_three_second_promise(self):
        self.fresh(2);self.m.state='COUNTDOWN';self.obstacle('BLOCKED');self.obstacle('BLOCKED')
        self.assertTrue(self.m.countdown_held)
        display=BootDisplay();display.context=dict(countdown_held=True);display.update('COUNTDOWN','')
        self.assertEqual(display.content()['heading'],'RESUMING START')

class IntegrationTests(unittest.TestCase):
    def test_no_graphics_are_submitted_as_obstacles_and_queue_is_latest_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            v=ObstacleOverlay(tmp,lambda:('learn',(0,0,1,0)),lambda *a:None,clock=lambda:1);v.thread=object()
            frame=floor();v.submit(frame);item=v.work.get_nowait();v.draw(frame)
            np.testing.assert_array_equal(item[1],floor());self.assertFalse(np.array_equal(frame,floor()))
    def test_application_does_not_feed_obstacle_lane_drawing_into_color_detection(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=Application(tmp,{});m=ControlTests();m.setUp();m.held();a.motion=m.m
            a.obstacle.thread=object();a.vision.thread=object();a.vision.cv=cv2
            try:
                frame=floor();a.camera_overlay(frame)
                np.testing.assert_array_equal(a.obstacle.work.get_nowait()[1],floor())
                np.testing.assert_array_equal(a.vision.work.get_nowait()[1],floor())
            finally:
                a.obstacle.thread=a.vision.thread=None;a.obstacle.close();a.vision.close();a.report.close()
    def test_slow_old_policy_result_cannot_reach_controller(self):
        with tempfile.TemporaryDirectory() as tmp:
            entered=threading.Event();release=threading.Event();seen=[];policy=[('monitor',(123,0,1,0))]
            v=ObstacleOverlay(tmp,lambda:policy[0],lambda *a:seen.append(a))
            def slow(*a,**kw):entered.set();release.wait(2);return dict(state='BLOCKED')
            try:
                with patch.object(v.detector,'detect',side_effect=slow):
                    v.submit(floor());self.assertTrue(entered.wait(1));policy[0]=('off',None);release.set();v.close()
                self.assertEqual(seen,[])
            finally:release.set();v.close()
    def test_step_obstacle_wait_runs_colors_without_digits_and_keeps_camera_scene(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=Application(tmp,{});m=ControlTests();m.setUp();a.motion=m.m;m.held()
            try:
                self.assertEqual(a.vision_policy()[0],'colors');self.assertEqual(a.obstacle_policy()[0],'monitor')
                a.display=BootDisplay()
                with patch.object(a,'status'):a.set_phase('RUNNING','')
                self.assertEqual(a.phase,'OBSTACLE WAIT');self.assertEqual(a.display.content()['kind'],'scene')
                m.clear()
                with patch.object(a,'status'):a.set_phase('RUNNING','')
                self.assertEqual(a.display.content()['hint'],'BTN0 / GREEN')
            finally:a.vision.close();a.obstacle.close();a.report.close()
    def test_detector_exception_publishes_unknown_and_close_discards_inflight_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            seen=[];arrived=threading.Event()
            def observer(*args):seen.append(args);arrived.set()
            v=ObstacleOverlay(tmp,lambda:('monitor',(123,0,None,None)),observer)
            with patch.object(v.detector,'detect',side_effect=RuntimeError('test failure')):
                try:
                    v.submit(floor());self.assertTrue(arrived.wait(1));self.assertEqual(seen[0][1]['state'],'UNKNOWN')
                    self.assertIsNotNone(v.error)
                finally:v.close()
            entered=threading.Event();release=threading.Event();seen=[]
            v=ObstacleOverlay(tmp,lambda:('monitor',(123,0,None,None)),observer)
            def slow(*a,**kw):entered.set();release.wait(1);return dict(state='BLOCKED')
            with patch.object(v.detector,'detect',side_effect=slow):
                v.submit(floor());self.assertTrue(entered.wait(1));v.closed.set();release.set();v.close()
            self.assertEqual(seen,[])


if __name__=='__main__':unittest.main()
