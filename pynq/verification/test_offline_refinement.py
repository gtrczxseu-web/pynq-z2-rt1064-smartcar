"""Recorded/synthetic images and deterministic timing; no hardware access."""
import json,sys,tempfile,unittest
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
from light_vision import detect_colors
from test_motion import Clock,Writer,status_packet
from motion_link import MotionLink,decode_status
from trajectory_report import RouteReport
from test_trajectory_report import pose
from route_settings import preset


class ColorTests(unittest.TestCase):
    def test_large_central_cards_are_not_rejected_at_twenty_percent(self):
        for label,color in [('RED',(0,0,255)),('GREEN',(0,255,0)),('BLUE',(255,0,0)),('YELLOW',(0,255,255))]:
            frame=np.full((240,320,3),180,np.uint8);frame[50:190,70:250]=color
            self.assertEqual([(d['label'],d['rect']) for d in detect_colors(frame,cv2)],[(label,(70,50,180,140))])

    def test_large_card_exception_does_not_admit_wall_border_offcenter_or_irregular_patch(self):
        frame=np.full((240,320,3),180,np.uint8)
        for box in ((0,0,320,240),(60,20,260,220),(4,4,144,154)):
            frame[:]=180;x,y,right,bottom=box;frame[y:bottom,x:right]=(255,0,0)
            self.assertEqual(detect_colors(frame,cv2),[])
        frame[:]=180;cv2.fillPoly(frame,[np.array([(60,50),(260,50),(260,190),(180,190),(180,130),(60,130)])],(255,0,0))
        self.assertEqual(detect_colors(frame,cv2),[])

    def test_recorded_four_cards_and_two_backgrounds_remain_correct(self):
        for name,expected in [('red',['RED']),('green',['GREEN']),('blue',['BLUE']),('yellow',['YELLOW']),('background',[])]:
            image=cv2.imdecode(np.frombuffer((PACKAGE/('verification/data/traffic_'+name+'_20261028.png')).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
            self.assertEqual([d['label'] for d in detect_colors(image,cv2)],expected,name)


class ContinuityTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.writer=Writer();self.m=MotionLink(writer=self.writer,clock=self.clock)
        self.m.session=123;self.m.sequence=100;self.m.run_started=90;self.m.recipe=preset('rectangle',mode='continuous')
        self.m.receive(decode_status(status_packet(9,123,100,ticks=5,version=7),self.clock()))
        self.m.state='PAUSED';self.m.paused_at=self.clock()
    def test_auto_resume_after_point_three_five_seconds_not_before(self):
        self.clock.now+=.349;self.m.step();self.assertEqual(self.writer.packets,[])
        self.clock.now+=.002;self.m.step();self.assertEqual(self.writer.packets[-1][3],4)
        self.assertEqual(self.m.session,123)
    def test_red_confirmed_at_resume_boundary_takes_priority(self):
        for dt in (.2,.151):
            self.clock.now+=dt;self.m.receive_vision(self.clock(),[dict(label='RED')],session=123);self.m.step()
        self.assertEqual(self.writer.packets[-1][3],5);self.assertFalse(any(p[3]==4 for p in self.writer.packets))
    def test_step_mode_keeps_waiting_without_new_button_or_green(self):
        self.m.recipe['mode']='step';self.clock.now+=100;self.m.step()
        self.assertEqual(self.m.state,'PAUSED');self.assertEqual(self.writer.packets,[])


class ReportMetricsTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.r=RouteReport(self.tmp.name,clock=lambda:20)
    def tearDown(self):self.r.close();self.tmp.cleanup()
    def event(self,kind,t,sample=None,**kw):self.r.event(dict(kind=kind,monotonic=t,**kw),sample)
    def state(self,state,t,sample=None,step=0):self.event('mcu_state',t,sample,session=123,state=state,step=step,lap=1,leg=step+1,action='TURN' if state=='TURN' else 'FORWARD')
    def begin(self,recipe):self.event('start_requested',0,pose(0),session=123,recipe=recipe)
    def finish(self,t,sample=None):self.state('DONE',t,sample);self.event('completed',t);self.r.poll();return self.r.latest_report
    def test_multi_turn_measurement_unwraps_720_degrees(self):
        self.begin(preset('rotate',turn_degrees=720,speed=120));self.state('TURN',1,pose(1))
        for i in range(1,25):
            sample=pose(1+i*.1);sample['heading_deg']=sample['vehicle_heading_deg']=(-30*i+180)%360-180
            self.r.pose(sample)
        result=self.finish(sample['received_monotonic']);s=result['steps'][0]
        self.assertEqual((s['command_rotation_deg'],s['measured_rotation_deg'],s['speed_limit_percent']),(720,720,120))
        self.assertIn('720.00 deg CW',(Path(result['report_dir'])/'report.html').read_text('utf8'))
    def test_discontinuous_yaw_is_unknown_not_a_fabricated_angle(self):
        self.begin(preset('rotate',turn_degrees=360));self.state('TURN',1,pose(1))
        sample=pose(1.1);sample['vehicle_heading_deg']=-30;self.r.pose(sample)
        sample=pose(1.2,epoch=2);sample['vehicle_heading_deg']=-60;self.r.pose(sample)
        self.r.pose(pose(1.3,epoch=2));s=self.finish(1.3)['steps'][0]
        self.assertIsNone(s['measured_rotation_deg']);self.assertEqual(s['command_rotation_deg'],360)
    def test_missing_pose_cannot_report_zero_degree_rotation(self):
        self.event('start_requested',0,session=123,recipe=preset('rotate'));self.state('TURN',1)
        self.assertIsNone(self.finish(3)['steps'][0]['measured_rotation_deg'])
    def test_commanded_distance_and_digit_speed_are_separate_from_measured_speed(self):
        self.begin(preset('outback',length=2,speed=80));self.state('MOVE',1,pose(1))
        self.state('PAUSED',3,pose(3,y=1.9,d=1.9));self.event('continue_requested',4,speed_percent=150)
        self.state('MOVE',4,pose(4,y=1.9,d=1.9),step=1);self.state('DONE',6,pose(6,d=3.8),step=1);self.event('completed',6);self.r.poll()
        first,second=self.r.latest_report['steps'];self.assertEqual((first['command_distance_m'],first['distance_m'],first['speed_limit_percent']),(2,1.9,80))
        self.assertEqual(second['speed_limit_percent'],150);self.assertAlmostEqual(second['average_speed_m_s'],.95)
    def test_signal_response_uses_confirmed_frame_and_held_ack(self):
        self.begin(preset('rectangle',mode='continuous'));self.state('MOVE',1,pose(1))
        self.event('signal_card',2,label='RED',captured_monotonic=1.8);self.event('signal_hold',2.01,reason='RED')
        self.state('HOLDING',2.1);self.state('HELD',2.3);self.state('MOVE',3);result=self.finish(4)
        response=result['traffic_responses'][0]
        self.assertAlmostEqual(response['confirmed_frame_to_held_s'],.5);self.assertAlmostEqual(response['command_to_held_s'],.29)
    def test_unacknowledged_hold_remains_unknown_and_cannot_contaminate_next_run(self):
        self.begin(preset('rectangle',mode='continuous'));self.event('signal_hold',2,reason='YELLOW')
        self.event('stop',3,reason='BTN1');self.r.poll();self.assertIsNone(self.r.latest_report['traffic_responses'][0]['held_monotonic'])
        self.begin(preset('translate'));self.assertEqual(self.finish(4)['traffic_responses'],[])


if __name__=='__main__':unittest.main()
