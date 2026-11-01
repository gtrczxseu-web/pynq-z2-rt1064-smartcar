"""Synthetic lifecycle and captured-image replay; no vehicle/motor operation."""
import csv,json,math,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
from trajectory_report import RouteReport,map_image
from test_trajectory_report import pose
from rear_projection import defaults,project
from light_vision import Recognizer,VisionOverlay
from standalone import Application
from boot_display import BootDisplay


class PatrolTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.now=1.
        self.report=RouteReport(self.tmp.name,clock=lambda:self.now,minimal=True,rect=(348,88,280,240))
    def tearDown(self):self.report.close();self.tmp.cleanup()
    def begin(self,session=12,t=1):
        self.now=t;self.report.event(dict(kind='start_requested',monotonic=t,session=session),pose(t,x=3,y=4));self.report.poll()
    def see(self,t,labels=('BLUE',),sample=True,session=12,poll=True):
        self.now=t
        p=pose(t,x=3,y=4) if sample is True else sample
        self.report.vision(t,[dict(label=v) for v in labels],p,session)
        if poll:self.report.poll()
    def test_continuous_sighting_is_one_event_and_reappearing_target_is_another(self):
        self.begin()
        for t in (1.1,1.6,2.1,2.6):self.see(t)
        self.assertEqual(len(self.report.run['recognitions']),1)
        self.see(3,());self.see(4.7,());self.see(5.2);self.see(5.7)
        events=self.report.run['recognitions'];self.assertEqual([e['id'] for e in events],['V1','V2'])
        self.assertEqual(events[0]['camera_view'],'rear')
    def test_single_frame_false_positive_and_flicker_are_not_repeated_events(self):
        self.begin();self.see(1.1);self.see(1.6,());self.see(2.1)
        self.assertEqual(self.report.run['recognitions'],[])
        self.see(2.6);self.see(3,());self.see(3.5)
        self.assertEqual(len(self.report.run['recognitions']),1)
    def test_no_run_old_session_and_prestart_frames_do_not_enter_the_report(self):
        self.see(1.1);self.assertIsNone(self.report.run)
        self.begin(t=3)
        self.see(3.1,session=11);self.see(3.6,session=11)
        self.report.vision(2.9,[dict(label='RED')],pose(2.9),12);self.report.poll()
        self.assertEqual(self.report.run['recognitions'],[])
        self.see(4.1);self.see(4.6);self.assertEqual(len(self.report.run['recognitions']),1)
    def test_location_uses_capture_pose_without_rear_view_sign_flip(self):
        self.begin();self.see(1.1,sample=pose(1.1,x=3.2,y=3.8))
        self.now=2.3;self.report.pose(pose(2.2,x=3.8,y=3.2))
        self.report.vision(1.6,[dict(label='BLUE')],pose(1.5,x=3.4,y=3.6),12);self.report.poll()
        event=self.report.run['recognitions'][0]
        self.assertTrue(event['position_available'])
        self.assertAlmostEqual(event['x_m'],.4);self.assertAlmostEqual(event['y_m'],-.4)
        self.assertIn('not target coordinates',self.report.run['recognition_position_basis'])
    def test_missing_stale_invalid_or_reset_pose_keeps_event_without_invented_position(self):
        self.begin()
        for n,sample in enumerate((None,pose(1,valid=True),pose(4,valid=False),pose(5.5,epoch=2))):
            t=1.1+n*1.5;label=('RED','GREEN','BLUE','DIGIT 2 99%')[n]
            self.see(t,(label,),sample=sample);self.see(t+.5,(label,),sample=sample)
        events=self.report.run['recognitions'];self.assertEqual(len(events),4)
        self.assertTrue(all(not e['position_available'] and e['x_m'] is None and e['y_m'] is None for e in events))
    def test_delayed_inference_duplicate_frame_and_late_callback_are_ignored(self):
        self.begin();self.see(1.1);self.see(1.1)
        self.assertEqual(self.report.run['recognitions'],[])
        self.now=4;self.report.vision(1.6,[dict(label='BLUE')],pose(1.6),12);self.report.poll()
        self.assertEqual(self.report.run['recognitions'],[])
        self.report.event(dict(kind='stop',monotonic=4,reason='test'));self.report.poll()
        self.see(4.5);self.see(5);self.assertEqual(self.report.latest_report['recognitions'],[])
    def test_pose_reset_during_the_run_does_not_create_a_false_relative_location(self):
        self.begin();self.report.pose(pose(1.1,x=0,y=0,epoch=2));self.report.poll()
        self.see(1.2,sample=pose(1.2,x=0,y=0,epoch=2));self.see(1.7,sample=pose(1.7,x=.1,y=0,epoch=2))
        event=self.report.run['recognitions'][0]
        self.assertFalse(event['position_available']);self.assertIsNone(event['x_m'])
    def test_step_is_at_capture_time_not_at_delayed_processing_time(self):
        self.begin()
        self.report.event(dict(kind='mcu_state',monotonic=1.05,session=12,state='MOVE',step=0,lap=1,leg=1,action='FORWARD'));self.report.poll()
        self.see(1.1)
        self.report.event(dict(kind='mcu_state',monotonic=1.7,session=12,state='PAUSED',step=0,lap=1,leg=1,action='FORWARD'));self.report.poll()
        self.now=2;self.report.vision(1.6,[dict(label='BLUE')],pose(1.6),12);self.report.poll()
        self.assertEqual(self.report.run['recognitions'][0]['step'],1)
    def test_report_exports_zero_and_unknown_coordinates_honestly(self):
        self.begin();self.see(1.1);self.see(1.6)
        self.see(2.1,('DIGIT 2 99%',),sample=None);self.see(2.6,('DIGIT 2 99%',),sample=None)
        self.report.event(dict(kind='stop',monotonic=3,reason='synthetic test'));self.report.poll()
        r=self.report.latest_report;target=Path(r['report_dir'])
        saved=json.loads((target/'summary.json').read_text('utf8'))
        self.assertEqual(len(saved['recognitions']),2)
        self.assertEqual(saved['recognitions'][0]['x_m'],0)
        self.assertIsNone(saved['recognitions'][1]['x_m'])
        with (target/'recognitions.csv').open(encoding='utf8',newline='') as f:rows=list(csv.DictReader(f))
        self.assertEqual(rows[0]['x_m'],'0.0');self.assertEqual(rows[1]['x_m'],'')
        html=(target/'report.html').read_text('utf8')
        self.assertIn('不是目标实际坐标',html);self.assertIn('recognitions.csv',html);self.assertIn('DIGIT 2',html)
        self.assertIsNone(self.report.error)
    def test_new_run_clears_tracks_and_late_old_session_cannot_leak(self):
        self.begin();self.see(1.1);self.see(1.6)
        self.begin(session=13,t=2)
        self.see(2.1,session=12);self.see(2.6,session=12)
        self.see(3.1,session=13);self.see(3.6,session=13)
        self.assertEqual([e['id'] for e in self.report.run['recognitions']],['V1'])
    def test_vision_callback_is_queue_only_and_does_not_touch_control(self):
        self.begin()
        with patch.object(Path,'open',side_effect=AssertionError('IO in inference callback')):
            self.see(1.1,poll=False);self.see(1.6,poll=False)
        self.report.poll();self.assertEqual(len(self.report.run['recognitions']),1)
    def test_rear_projection_axes_remain_correct_after_quarter_turns(self):
        c=defaults();c['camera_forward_m']=0
        for h in (0,90,180,270):
            a=math.radians(h)
            rear=(math.sin(a),-math.cos(a));rear_right=(.1*math.cos(a)+math.sin(a),.1*math.sin(a)-math.cos(a))
            center=project(rear,(0,0),h,c);right=project(rear_right,(0,0),h,c)
            self.assertLess(right[0],center[0]);self.assertIsNone(project((-rear[0],-rear[1]),(0,0),h,c))
    def test_map_markers_do_not_cover_numeric_hud_and_result_displays_count(self):
        self.begin();self.see(1.1);self.see(1.6)
        output=np.zeros((480,640,4),np.uint8);output[:,:,3]=77;self.report.overlay(output)
        self.assertFalse(output[336:,:,:3].any());self.assertTrue((output[:,:,3]==77).all())
        self.report.event(dict(kind='stop',monotonic=2,reason='test'));self.report.poll()
        display=BootDisplay();display.context=dict(summary=self.report.latest_report);display.update('RUN STOPPED','')
        display.overlay(output,False);self.assertEqual(output.shape,(480,640,4))


class VisionDeliveryTests(unittest.TestCase):
    def test_capture_context_and_session_are_copied_before_async_inference(self):
        import cv2
        received=[]
        def slow_colors(frame,cv):
            time.sleep(.1);return [dict(label='BLUE',rect=(10,20,30,40),color=(255,0,0))]
        with patch('light_vision.detect_colors',slow_colors):
            v=VisionOverlay(PACKAGE,clock=lambda:1,observer=lambda *args:received.append(args))
            try:
                sample=pose(1,x=2);v.overlay(np.zeros((240,320,3),np.uint8),sample,12);sample['vehicle_x_m']=99
                deadline=time.monotonic()+2
                while not received and time.monotonic()<deadline:time.sleep(.01)
                self.assertTrue(received);self.assertEqual(received[0][2]['vehicle_x_m'],2);self.assertEqual(received[0][3],12)
                self.assertEqual(v.snapshot()['camera_view'],'rear')
            finally:v.close()
    def test_real_captured_digit_replay_can_complete_report_without_vehicle_io(self):
        import cv2
        image=cv2.imdecode(np.frombuffer((PACKAGE/'verification/data/digit_card_2_20261027.png').read_bytes(),np.uint8),1)
        recognizer=Recognizer(PACKAGE)
        with tempfile.TemporaryDirectory() as tmp:
            clock=[1.];r=RouteReport(tmp,clock=lambda:clock[0]);r.event(dict(kind='start_requested',monotonic=1,session=12),pose(1));r.poll()
            for t in (1.1,1.6,2.1,2.6):
                clock[0]=t;r.vision(t,recognizer.detect(image),pose(t),12);r.poll()
            r.event(dict(kind='stop',monotonic=3,reason='Captured-image replay, no real vehicle run'));r.poll()
            self.assertEqual([e['label'] for e in r.latest_report['recognitions']],['DIGIT 2'])
            self.assertIsNone(r.error);r.close()


if __name__=='__main__':unittest.main()
