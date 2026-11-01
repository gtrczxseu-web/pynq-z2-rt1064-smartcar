"""Mode gating, independent workers and stale-result boundaries; no hardware."""
import sys,tempfile,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
from light_vision import VisionOverlay
from standalone import Application
from motion_link import MotionLink,decode_status
from route_settings import preset
from trajectory_report import RouteReport
from test_motion import Clock,Writer,status_packet
from test_trajectory_report import pose

def red_frame():
    frame=np.full((240,320,3),180,np.uint8);frame[60:110,100:150]=(0,0,255);return frame

class WorkerPolicyTests(unittest.TestCase):
    def test_continuous_colors_never_construct_digit_model(self):
        seen=threading.Event();v=VisionOverlay(PACKAGE,observer=lambda *a:seen.set(),policy_provider=lambda:('colors',(12,)))
        with patch('light_vision.Recognizer',side_effect=AssertionError('No digit model in continuous')):
            try:
                v.overlay(red_frame(),session=12);self.assertTrue(seen.wait(2))
                self.assertIsNone(v.digit_thread);self.assertEqual(v.digit_frames,0)
                self.assertIsNone(v.error);self.assertIsNone(v.digit_error);self.assertFalse(v.snapshot()['digit_enabled'])
            finally:v.close()
    def test_step_off_does_not_queue_or_start_any_inference(self):
        policy=[('off',None)];v=VisionOverlay(PACKAGE,policy_provider=lambda:policy[0])
        try:
            v.overlay(red_frame());self.assertIsNone(v.thread);self.assertIsNone(v.digit_thread)
            self.assertTrue(v.work.empty());self.assertTrue(v.digit_work.empty())
            self.assertEqual(v.snapshot()['policy'],'off')
        finally:v.close()
    def test_colors_publish_while_digit_worker_is_blocked(self):
        entered=threading.Event();release=threading.Event();color=threading.Event()
        class Blocked:
            digit_status='WAIT CARD';confidence=None
            def __init__(self,folder):pass
            def detect_digits(self,frame):entered.set();release.wait(2);return []
        v=VisionOverlay(PACKAGE,observer=lambda *a:color.set(),policy_provider=lambda:('step_wait',(12,0)))
        with patch('light_vision.Recognizer',Blocked):
            try:
                v.overlay(red_frame(),session=12);self.assertTrue(entered.wait(1));self.assertTrue(color.wait(1))
                self.assertFalse(release.is_set());self.assertIsNotNone(v.color_result)
            finally:release.set();v.close()
    def test_inflight_digit_is_discarded_after_resuming_or_next_wait(self):
        for replacement in (('off',None),('step_wait',(12,1))):
            entered=threading.Event();release=threading.Event();done=threading.Event();seen=[];policy=[('step_wait',(12,0))]
            class Blocked:
                digit_status='DIGIT 6';confidence=.95
                def __init__(self,folder):pass
                def detect_digits(self,frame):
                    entered.set();release.wait(2);done.set();return [dict(label='DIGIT 6 95%',rect=(100,70,20,30),color=(0,255,0))]
            v=VisionOverlay(PACKAGE,digit_observer=lambda *a:seen.append(a),policy_provider=lambda:policy[0])
            with patch('light_vision.Recognizer',Blocked):
                try:
                    v.overlay(red_frame(),session=12);self.assertTrue(entered.wait(1))
                    policy[0]=replacement;release.set();self.assertTrue(done.wait(1));v.close()
                    self.assertEqual(seen,[]);self.assertIsNone(v.confirmed)
                finally:release.set();v.close()
    def test_switching_mode_clears_old_text_boxes_and_waiting_images(self):
        policy=[('step_wait',(12,0))];v=VisionOverlay(PACKAGE,clock=lambda:1,policy_provider=lambda:policy[0])
        v.thread=object();v.digit_thread=object();v.cv=cv2
        frame=red_frame();v.overlay(frame,session=12);v.confirmed=(1,'6',(12,None,None))
        policy[0]=('off',None);v.overlay(frame,session=12)
        self.assertTrue(v.work.empty());self.assertTrue(v.digit_work.empty());self.assertIsNone(v.confirmed)
        self.assertIsNone(v.result);self.assertIsNone(v.color_result)
    def test_latest_frame_queues_are_bounded_and_digit_rate_is_two_per_second(self):
        now=[1.];v=VisionOverlay(PACKAGE,clock=lambda:now[0]);v.thread=object();v.digit_thread=object()
        v.overlay(red_frame());self.assertEqual(v.last_digit_submit,1.)
        for t in (1.13,1.26,1.39):now[0]=t;v.overlay(red_frame())
        self.assertEqual(v.last_digit_submit,1.);self.assertEqual(v.work.qsize(),1);self.assertEqual(v.digit_work.qsize(),1)
        now[0]=1.52;v.overlay(red_frame());self.assertEqual(v.last_digit_submit,1.52)
    def test_digit_model_failure_does_not_disable_color_worker(self):
        color=threading.Event();attempted=threading.Event();v=VisionOverlay(PACKAGE,observer=lambda *a:color.set())
        def missing(folder):attempted.set();raise RuntimeError('Missing digit model')
        with patch('light_vision.Recognizer',side_effect=missing):
            try:
                v.overlay(red_frame());self.assertTrue(color.wait(2));self.assertTrue(attempted.wait(2))
                v.close();self.assertIsNotNone(v.digit_error);self.assertIsNone(v.error)
            finally:v.close()

class StateBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.writer=Writer();self.motion=MotionLink(writer=self.writer,clock=self.clock)
        self.motion.session=123;self.motion.run_started=90;self.motion.recipe=preset('rectangle',.4,.4,mode='step')
        self.ticks=0;self.paused(0)
    def paused(self,step):
        self.ticks+=5;self.motion.receive(decode_status(status_packet(9,123,self.motion.sequence,step,ticks=self.ticks,version=6),self.clock()))
        self.motion.state='PAUSED';self.motion.paused_at=self.clock()
    def digit(self,label='DIGIT 6 95%',stamp=None):
        self.motion.receive_digits(self.clock() if stamp is None else stamp,[dict(label=label)],session=123);self.motion.step()
    def test_digit_channel_sets_next_speed_only_in_step_wait(self):
        for unused in range(2):self.clock.now+=.2;self.digit()
        self.assertEqual(self.motion.next_digit,6)
        self.motion.next_digit=None;self.motion.recipe['mode']='continuous'
        for unused in range(2):self.clock.now+=.2;self.digit('DIGIT 9 95%')
        self.assertIsNone(self.motion.next_digit)
    def test_motion_and_stale_previous_pause_digit_results_are_ignored(self):
        old=self.clock();self.motion.state='DRIVING';self.ticks+=5
        self.motion.receive(decode_status(status_packet(3,123,self.motion.sequence,0,ticks=self.ticks,version=6),self.clock()))
        for unused in range(2):self.clock.now+=.1;self.digit()
        self.assertIsNone(self.motion.next_digit)
        self.paused(1)
        for unused in range(2):self.clock.now+=.1;self.digit(stamp=old)
        self.assertIsNone(self.motion.next_digit)
    def test_digit_queue_is_consumed_before_green_continues(self):
        for unused in range(2):
            self.clock.now+=.2
            self.motion.receive_digits(self.clock(),[dict(label='DIGIT 6 95%')],session=123)
            self.motion.receive_vision(self.clock(),[dict(label='GREEN')],session=123);self.motion.step()
        self.assertEqual(self.motion.state,'RESUMING');self.assertEqual(self.writer.packets[-1][3],0x66)
    def test_application_policy_follows_mcu_stopped_boundary_and_settings(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=Application(tmp,dict(camera=True));a.motion=self.motion
            self.assertEqual(a.vision_policy()[0],'step_wait');first=a.vision_policy()
            self.motion.state='RESUMING';self.assertEqual(a.vision_policy()[0],'off')
            self.paused(1);self.assertNotEqual(a.vision_policy(),first)
            a.settings_menu.open=True;self.assertEqual(a.vision_policy()[0],'off')
            a.settings_menu.open=False;self.motion.recipe['mode']='continuous'
            self.assertEqual(a.vision_policy()[0],'colors')
    def test_independent_color_and_digit_observers_do_not_reset_report_tracks(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=RouteReport(tmp,clock=lambda:2)
            try:
                r.event(dict(kind='start_requested',monotonic=1,session=12),pose(1))
                for t in (1.1,1.3):
                    r.vision(t,[dict(label='GREEN')],pose(t),12,channel='colors')
                    r.vision(t,[dict(label='DIGIT 3 95%')],pose(t),12,channel='digits')
                r.poll();self.assertEqual({d['label'] for d in r.run['recognitions']},{'GREEN','DIGIT 3'})
            finally:r.close()

if __name__=='__main__':unittest.main()
