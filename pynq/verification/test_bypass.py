"""Bypass protocol, retry, traffic arbitration and report continuity; no hardware."""
from pathlib import Path
import sys,tempfile,unittest
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
from motion_link import MotionLink,decode_status
from route_settings import preset
from standalone import Application
from trajectory_report import RouteReport
from test_motion import Clock,Writer,status_packet
from test_trajectory_report import pose

class BypassTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.writer=Writer();self.m=MotionLink(writer=self.writer,clock=self.clock)
        self.m.recipe=preset('rectangle',mode='continuous',speed=80);self.m.session=123;self.m.sequence=100;self.m.run_started=90;self.m.state='DRIVING'
        self.native=3;self.ticks=0;self.version=8;self.fresh()
    def fresh(self,state=None,seq=None,fault=0):
        if state is not None:self.native=state
        self.ticks+=5
        self.m.receive(decode_status(status_packet(self.native,123,self.m.sequence if seq is None else seq,0,fault,self.ticks,self.version),self.clock()))
        self.m.receive_pose(dict(pose_valid=True,pose_epoch=1,source_clock_epoch=0,speed_m_s=0,received_monotonic=self.clock()))
    def obstacle(self,state):
        self.clock.now+=.2;self.fresh();self.m.receive_obstacle(self.clock(),dict(state=state),self.m.latest,123,0);self.m.step()
    def green(self):
        self.clock.now+=.2;self.fresh();self.m.receive_vision(self.clock(),[dict(label='GREEN')],self.m.latest,123);self.m.step()
    def request(self):
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.assertEqual(self.writer.packets[-1][3],5)
        self.clock.now+=.1;self.fresh(11);self.m.step();self.assertEqual(self.writer.packets[-1][3],7)
    def test_v8_status_and_held_phase_are_decoded_without_hiding_real_faults(self):
        for state in (12,13,14):self.assertEqual(decode_status(status_packet(state=state,version=8))['state'],('BYPASS_RIGHT','BYPASS_FORWARD','BYPASS_LEFT')[state-12])
        self.assertEqual(decode_status(status_packet(state=11,fault=0x11,version=8))['bypass_phase'],'BYPASS_FORWARD')
        self.assertEqual(decode_status(status_packet(state=11,fault=8,version=8))['fault'],'BYPASS TOO SHORT')
        for packet in (status_packet(state=12,version=7),status_packet(state=11,fault=0x13,version=8),status_packet(state=11,fault=8,version=7)):
            with self.assertRaises(ValueError):decode_status(packet)
    def test_stopped_or_unknown_or_stale_frames_cannot_start_a_bypass(self):
        self.m.state='STOPPED';self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.assertEqual(self.writer.packets,[])
        self.m.state='SIGNAL_WAIT';self.fresh(11);self.m.obstacle_hold=True
        self.m.obstacle_state='UNKNOWN';self.m.step();self.assertFalse(any(p[3]==7 for p in self.writer.packets))
        self.m.obstacle_state='BLOCKED';self.m.obstacle_stamp=self.clock()-1;self.m.step();self.assertFalse(any(p[3]==7 for p in self.writer.packets))
    def test_v7_keeps_existing_obstacle_stop_without_sending_new_opcode(self):
        self.version=7;self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.fresh(11);self.m.step()
        self.assertTrue(self.m.obstacle_hold);self.assertFalse(any(p[3]==7 for p in self.writer.packets))
    def test_confirmed_obstacle_clearing_during_braking_still_starts_bypass(self):
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.fresh(10)
        for unused in range(3):self.obstacle('CLEAR')
        self.assertFalse(any(p[3]==7 for p in self.writer.packets))
        self.fresh(11);self.m.step();self.assertEqual(self.writer.packets[-1][3],7)
        self.assertFalse(any(p[3] in (2,4,6) for p in self.writer.packets))
    def test_confirmed_intent_survives_unknown_and_frame_gap_while_stopping(self):
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.fresh(10);self.obstacle('UNKNOWN')
        self.clock.now+=1;self.fresh(11);self.m.step()
        self.assertEqual(self.writer.packets[-1][3],7)
    def test_confirmed_intent_cannot_cross_pose_epoch(self):
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.fresh(11)
        self.m.latest=dict(self.m.latest,pose_epoch=2);self.m.step()
        self.assertFalse(any(p[3]==7 for p in self.writer.packets))
    def test_confirmed_intent_cannot_cross_motion_step(self):
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.fresh(11)
        self.m.mcu['step']=1;self.m.step()
        self.assertFalse(any(p[3]==7 for p in self.writer.packets))
    def test_stop_before_hold_ack_discards_intent_and_green_cannot_restart(self):
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.m.stop('BTN1');self.fresh(11)
        self.green();self.green();self.m.step()
        self.assertEqual(self.m.state,'STOPPED');self.assertFalse(any(p[3]==7 for p in self.writer.packets))
    def test_red_delays_committed_bypass_and_green_does_not_release_straight_ahead(self):
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.m.traffic_reason='RED';self.fresh(10)
        for unused in range(3):self.obstacle('CLEAR')
        self.fresh(11);self.m.step();self.assertFalse(any(p[3]==7 for p in self.writer.packets))
        self.green();self.green();self.assertEqual(self.writer.packets[-1][3],7)
        self.assertFalse(any(p[3]==6 for p in self.writer.packets))
    def test_bypass_retry_is_identical_and_keeps_original_route_and_step(self):
        recipe=self.m.recipe.copy();self.request();packet=self.writer.packets[-1];old_seq=(self.m.sequence-1)&65535
        self.clock.now+=.11;self.fresh(11,seq=old_seq);self.m.step();self.assertEqual(self.writer.packets[-1],packet)
        self.fresh(12);self.m.step();self.assertTrue(self.m.bypass_active);self.assertEqual(self.m.state,'BYPASS')
        self.assertFalse(self.m.obstacle_hold);self.assertEqual(self.m.recipe,recipe);self.assertEqual(self.m.mcu['step'],0)
        self.fresh(13);self.m.step();self.fresh(14);self.m.step();self.fresh(3);self.m.step()
        self.assertEqual(self.m.state,'DRIVING');self.assertFalse(self.m.bypass_active)
        self.assertEqual(sum(e['kind']=='bypass_started' for e in self.m.log),1)
        self.assertEqual(sum(e['kind']=='bypass_completed' for e in self.m.log),1)
        self.assertFalse(any(p[3] in (1,2,4) for p in self.writer.packets))
    def test_short_remaining_distance_stays_held_without_retry_loop(self):
        self.request();self.fresh(11,fault=8);self.m.step();self.assertTrue(self.m.obstacle_hold)
        count=sum(p[3]==7 for p in self.writer.packets)
        for unused in range(4):self.obstacle('BLOCKED')
        self.assertEqual(sum(p[3]==7 for p in self.writer.packets),count);self.assertIn('1m',self.m.bypass_note)
    def test_bypass_can_finish_at_pause_or_done_without_an_intermediate_move_status(self):
        for state in (9,6):
            self.setUp();self.request();self.fresh(12);self.m.step();self.fresh(14);self.m.step()
            self.fresh(state);self.m.step();self.assertFalse(self.m.bypass_active)
            self.assertEqual(self.m.state,'PAUSED' if state==9 else 'FINISHED')
            self.assertEqual(sum(e['kind']=='bypass_completed' for e in self.m.log),1)
    def test_red_preempts_unacknowledged_bypass_and_restores_original_hold_if_not_accepted(self):
        self.request();self.m.traffic_reason='RED';self.m.step();self.assertEqual(self.writer.packets[-1][3],5)
        self.fresh(11);self.m.step();self.assertTrue(self.m.obstacle_hold);self.assertFalse(self.m.bypass_active)
        self.green();self.green();self.assertEqual(self.writer.packets[-1][3],7)
        self.assertFalse(any(p[3]==6 for p in self.writer.packets))
    def test_red_after_native_acceptance_resumes_the_held_bypass_phase(self):
        self.request();self.m.traffic_reason='RED';self.m.step();self.fresh(11,fault=0x10);self.m.step()
        self.assertTrue(self.m.bypass_seen);self.assertFalse(self.m.obstacle_hold)
        self.green();self.green();self.assertEqual(self.writer.packets[-1][3],6)
        self.fresh(12);self.m.step();self.assertEqual(self.m.state,'BYPASS')
        self.assertEqual(sum(p[3]==7 for p in self.writer.packets),1)
    def test_forward_obstacle_pauses_without_nested_bypass_and_green_resumes_it(self):
        self.request();self.fresh(12);self.m.step();self.fresh(13);self.m.step()
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.assertEqual(self.writer.packets[-1][3],5)
        self.fresh(11,fault=0x11);self.m.step()
        for unused in range(3):self.obstacle('CLEAR')
        self.green();self.green();self.assertEqual(self.writer.packets[-1][3],6)
        self.assertEqual(sum(p[3]==7 for p in self.writer.packets),1)
    def test_lateral_phases_disable_detection_and_operator_stop_never_restarts(self):
        self.request();self.fresh(12);self.m.step()
        app=SimpleNamespace(motion=self.m,settings_menu=SimpleNamespace(open=False))
        self.assertEqual(Application.obstacle_policy(app)[0],'off')
        self.obstacle('BLOCKED');self.obstacle('BLOCKED');self.assertFalse(self.m.obstacle_hold)
        self.fresh(13);self.m.step();self.assertEqual(Application.obstacle_policy(app)[0],'monitor')
        self.m.stop('BTN1');count=len(self.writer.packets);self.green();self.green()
        self.assertEqual(self.m.state,'STOPPED');self.assertEqual(len(self.writer.packets),count)
    def test_bypass_motion_time_is_not_counted_as_traffic_wait_or_extra_route_steps(self):
        with tempfile.TemporaryDirectory() as folder:
            r=RouteReport(folder,clock=lambda:10)
            try:
                r.event(dict(kind='start_requested',monotonic=0,session=123,recipe=preset('translate')),pose(0))
                for t,state in ((1,'MOVE'),(2,'HELD'),(3,'BYPASS_RIGHT'),(4,'BYPASS_FORWARD'),(5,'BYPASS_LEFT'),(6,'MOVE'),(7,'DONE')):
                    r.event(dict(kind='mcu_state',monotonic=t,session=123,state=state,step=0,lap=1,leg=1,action='FORWARD'),pose(t,y=t*.1,d=t*.1))
                r.event(dict(kind='completed',monotonic=7),pose(7,y=.7,d=.7));r.poll()
                self.assertEqual(r.latest_report['completed_steps'],1);self.assertEqual(len(r.latest_report['steps']),1)
                self.assertAlmostEqual(r.latest_report['steps'][0]['duration_s'],5)
            finally:r.close()

if __name__=='__main__':unittest.main()
