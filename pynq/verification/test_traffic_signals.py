"""Offline card/state-machine tests; no connection to the vehicle."""
import json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_motion import Clock,Writer,status_packet
from motion_link import MotionLink,decode_status
from vision_signals import SignalCards
from light_vision import Recognizer
from route_settings import preset
from boot_display import BootDisplay
from standalone import Application
from trajectory_report import RouteReport

class TrafficTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.writer=Writer();self.link=MotionLink(writer=self.writer,clock=self.clock)
        self.link.session=123;self.link.sequence=100;self.link.run_started=90
        self.link.recipe=preset('rectangle',.4,.2,'continuous',80);self.ticks=0
        self.status(3);self.link.state='DRIVING'
    def status(self,state,step=0,seq=None,version=6,session=123):
        self.ticks+=5
        self.link.receive(decode_status(status_packet(state,session,self.link.sequence if seq is None else seq,step,ticks=self.ticks,version=version),self.clock()))
    def frame(self,*labels,age=0,session=123,sample=None):
        self.link.receive_vision(self.clock()-age,[dict(label=label) for label in labels],sample,session);self.link.step()
    def card(self,*labels):
        self.clock.now+=.2;self.frame(*labels);self.clock.now+=.2;self.frame(*labels)
    def held(self):
        self.status(10);self.link.step();self.clock.now+=.1;self.status(11);self.link.step()
    def test_red_holds_current_action_retries_one_packet_and_waits_after_disappearing(self):
        self.card('RED');p=self.writer.packets[-1];self.assertEqual(p[3],5)
        for unused in range(3):self.clock.now+=.11;self.link.step();self.assertEqual(self.writer.packets[-1],p)
        self.held();self.assertEqual(self.link.state,'SIGNAL_WAIT')
        self.clock.now+=20;self.frame();self.assertEqual(self.link.traffic_reason,'RED')
        self.assertNotIn(6,[p[3] for p in self.writer.packets]);self.assertEqual(self.link.session,123)
    def test_green_releases_red_and_rejoins_same_action(self):
        self.card('RED');self.held();self.card('GREEN');self.assertEqual(self.writer.packets[-1][3],6)
        self.status(3);self.link.step();self.assertEqual(self.link.state,'DRIVING')
        self.assertNotIn(2,[p[3] for p in self.writer.packets]);self.assertEqual(self.link.mcu['step'],0)
    def test_yellow_waits_five_seconds_after_mcu_settled_and_does_not_repeat(self):
        self.card('YELLOW');self.status(10);self.link.step()
        self.clock.now+=7;self.status(10);self.frame('YELLOW');self.assertIsNone(self.link.yellow_at)
        self.status(11);self.link.step();start=self.clock()
        self.clock.now=start+4.99;self.frame('YELLOW');self.assertEqual(self.link.traffic_reason,'YELLOW')
        self.clock.now=start+5.01;self.frame('YELLOW');self.assertEqual(self.writer.packets[-1][3],6)
        self.status(3);self.link.step();self.card('YELLOW');self.assertIsNone(self.link.traffic_reason)
        self.assertEqual(sum(p[3]==5 for p in self.writer.packets),1)
    def test_red_overrides_yellow_timer(self):
        self.card('YELLOW');self.held();self.card('RED');self.assertEqual(self.link.traffic_reason,'RED')
        self.clock.now+=10;self.frame();self.assertNotEqual(self.writer.packets[-1][3],6)
    def test_green_cannot_release_when_red_is_still_present(self):
        self.card('RED');self.held();self.card('RED','GREEN')
        self.assertEqual(self.link.traffic_reason,'RED');self.assertNotIn(6,[p[3] for p in self.writer.packets])
    def test_green_does_not_shorten_yellow_wait(self):
        self.card('YELLOW');self.held();self.card('GREEN');self.assertEqual(self.link.traffic_reason,'YELLOW')
    def test_red_between_steps_prevents_continuous_auto_next(self):
        self.status(9);self.link.state='PAUSED';self.link.paused_at=self.clock()
        self.card('RED');self.assertEqual(self.writer.packets[-1][3],5)
        self.held();self.clock.now+=20;self.link.step();self.assertNotIn(4,[p[3] for p in self.writer.packets])
    def test_step_green_runs_only_once_until_removed_and_presented_again(self):
        self.link.recipe['mode']='step';self.status(9);self.link.state='PAUSED';self.link.paused_at=self.clock()
        self.card('GREEN');self.assertEqual(self.writer.packets[-1][3],4)
        self.status(3,step=1);self.link.step();self.status(9,step=1);self.link.step()
        n=len(self.writer.packets);self.card('GREEN');self.assertEqual(len(self.writer.packets),n)
        self.clock.now+=.6;self.frame();self.card('GREEN');self.assertEqual(len(self.writer.packets),n+1)
    def test_step_red_yellow_ignored_and_green_seen_while_moving_not_queued(self):
        self.link.recipe['mode']='step';self.card('RED');self.card('YELLOW');self.card('GREEN')
        self.assertIsNone(self.link.traffic_reason);self.assertTrue(self.link.active)
        self.status(9);self.link.step();self.assertEqual(self.link.state,'PAUSED')
        self.card('GREEN');self.assertEqual(self.link.state,'RESUMING')
    def test_completed_status_before_green_result_uses_actual_pause_boundary(self):
        self.link.recipe['mode']='step';self.clock.now+=.2;self.frame('GREEN')
        self.clock.now+=.1;self.status(9)
        self.clock.now+=.1;self.frame('GREEN')
        self.assertEqual(self.link.state,'PAUSED')
        self.clock.now+=.2;self.frame('GREEN')
        self.assertEqual(self.link.state,'RESUMING');self.assertEqual(self.writer.packets[-1][3],4)
    def test_stale_before_pause_green_cannot_start_next_step(self):
        self.link.recipe['mode']='step';self.status(9);self.link.state='PAUSED';self.link.paused_at=self.clock()+.5
        self.card('GREEN');self.assertEqual(self.link.state,'PAUSED')
    def test_digits_zero_through_nine_are_absolute_and_consumed_once(self):
        for digit in range(10):
            self.setUp();self.link.recipe['mode']='step';self.status(9);self.link.state='PAUSED';self.link.paused_at=self.clock()
            self.card('DIGIT %d 95%%'%digit)
            self.assertEqual(self.link.snapshot()['next_speed_percent'],60+digit*10)
            self.status(9);self.link.state='PAUSED';self.link.toggle();self.link.step()
            self.assertEqual(self.writer.packets[-1][3],0x60+digit);self.assertIsNone(self.link.next_digit)
            packet=self.writer.packets[-1];self.clock.now+=.11;self.link.step();self.assertEqual(self.writer.packets[-1],packet)
            self.status(9,step=1);self.link.step();self.link.toggle();self.link.step()
            self.assertEqual(self.writer.packets[-1][3],4)
    def test_digit_waits_across_turn_until_next_translation(self):
        self.link.recipe=preset('competition',.4,mode='step');self.status(9,step=4);self.link.state='PAUSED';self.link.paused_at=self.clock()
        self.card('DIGIT 6 90%')
        self.status(9,step=4);self.link.state='PAUSED';self.link.toggle();self.link.step()
        self.assertEqual(self.writer.packets[-1][3],4);self.assertEqual(self.link.next_digit,6)
        self.status(9,step=5);self.link.step();self.link.toggle();self.link.step()
        self.assertEqual(self.writer.packets[-1][3],0x66);self.assertIsNone(self.link.next_digit)
    def test_green_and_digit_in_same_frames_use_speed_for_that_step(self):
        self.link.recipe['mode']='step';self.status(9);self.link.state='PAUSED';self.link.paused_at=self.clock()
        self.card('GREEN','DIGIT 9 90%');self.assertEqual(self.writer.packets[-1][3],0x69)
    def test_latest_distinct_digit_replaces_pending_value(self):
        self.link.recipe['mode']='step';self.status(9);self.link.state='PAUSED';self.link.paused_at=self.clock()
        self.card('DIGIT 2 90%');self.card('DIGIT 7 90%');self.assertEqual(self.link.next_digit,7)
    def test_old_frames_wrong_session_epoch_and_display_last_do_not_control(self):
        self.link.latest=dict(pose_epoch=2,source_clock_epoch=0)
        for unused in range(3):
            self.clock.now+=.2;self.frame('RED',age=.6);self.frame('RED',session=124)
            self.frame('RED',sample=dict(pose_epoch=1,source_clock_epoch=0));self.frame('LAST 6')
        self.assertIsNone(self.link.traffic_reason);self.assertIsNone(self.link.next_digit)
    def test_duplicate_timestamp_is_not_two_confirmations(self):
        self.frame('RED');self.frame('RED');self.assertIsNone(self.link.traffic_reason)
    def test_no_visual_start_in_ready_finished_stopped_or_menu_preparation(self):
        for state in ('WAITING','STOPPED','FINISHED','ARMING','CONFIGURING','STARTING'):
            self.link.state=state;self.status(7)
            for unused in range(2):
                self.clock.now+=.2;self.link.receive_vision(self.clock(),[dict(label='GREEN'),dict(label='DIGIT 9 90%')],session=123)
                self.link._vision_step(self.clock())
            self.assertEqual(self.link.state,state);self.assertIsNone(self.link.next_digit)
    def test_old_protocol_keeps_basic_route_without_visual_motor_commands(self):
        self.status(3,version=5);self.card('RED','DIGIT 9 90%')
        self.assertFalse(self.link.snapshot()['vision_control']);self.assertIsNone(self.link.traffic_reason)
        self.assertTrue(all(p[3]==3 for p in self.writer.packets))
    def test_stop_clears_pending_light_speed_and_cannot_be_undone_by_green(self):
        self.card('RED','DIGIT 9 90%');self.held();self.link.stop('BTN1');self.status(7)
        self.card('GREEN');self.assertEqual(self.link.state,'STOPPED');self.assertEqual(self.writer.packets[-1][3],0)
        self.assertIsNone(self.link.next_digit);self.assertIsNone(self.link.traffic_reason)
    def test_red_during_release_cancels_release_retry(self):
        self.card('RED');self.held();self.card('GREEN');self.assertEqual(self.writer.packets[-1][3],6)
        self.clock.now+=.6;self.frame();self.card('RED');self.assertEqual(self.writer.packets[-1][3],5)
    def test_queue_contains_only_latest_frame(self):
        for i in range(100):self.link.receive_vision(self.clock()+i,[],session=123)
        self.assertEqual(self.link.vision_frames.qsize(),1)

class CardTests(unittest.TestCase):
    def test_rearm_requires_observed_absence_not_a_camera_gap(self):
        c=SignalCards();red=[dict(label='RED')]
        self.assertEqual(c.update(0,red)[0],set());self.assertEqual(c.update(.2,red)[0],{'RED'})
        self.assertEqual(c.update(10,red)[0],set());self.assertEqual(c.update(10.2,red)[0],set())
        c.update(10.8,[]);c.update(11,red);self.assertEqual(c.update(11.2,red)[0],{'RED'})
    def test_red_priority_yellow_region_and_background_fixture(self):
        c=SignalCards();ds=[dict(label=x) for x in ('RED','YELLOW','GREEN')]
        c.update(0,ds);self.assertEqual(c.update(.2,ds)[1],'RED')
        r=Recognizer(PACKAGE);frame=np.full((240,320,3),180,np.uint8)
        frame[60:120,100:160]=(0,255,255)
        self.assertEqual([d['label'] for d in r.detect(frame)],['YELLOW'])
        frame[:]=180;frame[60:82,100:120]=(0,255,255)
        self.assertEqual(r.detect(frame),[])
    def test_wait_display_preserves_scene_and_controls(self):
        d=BootDisplay();d.context=dict(traffic_message='RED: WAIT GREEN',vision_control=True)
        d.update('TRAFFIC WAIT','');self.assertEqual(d.content()['kind'],'scene');self.assertEqual(d.content()['hint'],'BTN1 STOP')
        d.update('STEP PAUSED','');self.assertEqual(d.content()['hint'],'BTN0 / GREEN')
    def test_observer_fans_out_once_and_nonrunning_phase_is_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=Application(tmp,dict(camera=True),display=BootDisplay());seen=[]
            a.report.vision=lambda *v,**kw:seen.append(('report',v));a.motion=SimpleNamespace(receive_vision=lambda *v:seen.append(('motion',v)),recipe=preset(mode='continuous'),latest=None,snapshot=lambda:{})
            a.vision_observed(1,[],None,123);self.assertEqual([v[0] for v in seen],['report','motion'])
            a.set_phase('CONNECTING CAMERA','test')
    def test_report_preserves_step_across_hold_and_excludes_wait_from_active_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=RouteReport(tmp,clock=lambda:20)
            try:
                r.event(dict(kind='start_requested',monotonic=0,session=12))
                for state,t in (('MOVE',1),('HOLDING',2),('HELD',2.1),('MOVE',7),('DONE',9)):
                    r.event(dict(kind='mcu_state',monotonic=t,session=12,state=state,step=0,lap=1,leg=1,action='FORWARD'))
                r.event(dict(kind='completed',monotonic=9));r.poll();result=r.latest_report
                self.assertEqual(len(result['steps']),1);self.assertEqual(result['completed_steps'],1)
                self.assertEqual(result['steps'][0]['traffic_wait_s'],5);self.assertEqual(result['action_duration_s'],3)
            finally:r.close()

if __name__=='__main__':unittest.main()
