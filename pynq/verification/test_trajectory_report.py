import csv,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
from trajectory_report import RouteReport,vehicle_pose,map_image
from motion_link import MotionLink,STEPS


def pose(t,x=0,y=0,d=0,speed=1,valid=True,epoch=1,gaps=0):
    return dict(received_monotonic=t,x_m=-x,y_m=-y,vehicle_x_m=x,vehicle_y_m=y,
        heading_deg=0,vehicle_heading_deg=0,distance_m=d,speed_m_s=speed,
        reference_frame='camera_initial_y_negative',pose_valid=valid,pose_epoch=epoch,
        source_clock_epoch=0,pose_gap_count=gaps,source='SYNTHETIC_TEST')


class ReportTests(unittest.TestCase):
    def test_completed_actions_without_move_status_do_not_disappear_from_report(self):
        import route_settings as routes
        self.event('start_requested',0,pose(0),session=12,recipe=routes.preset('outback',.4))
        self.state('PAUSED',0,5,pose(5,y=.4,d=.4))
        self.state('DONE',1,8,pose(8,y=0,d=.8));self.event('completed',8);self.r.poll()
        r=self.r.latest_report
        self.assertEqual(r['completed_steps'],2);self.assertEqual(r['measured_distance_m'],.8)
        self.assertIsNone(r['action_duration_s']);self.assertIsNone(r['waiting_duration_s'])
        self.assertIsNone(r['average_action_speed_m_s'])
        self.assertTrue(all(s['started_monotonic'] is None and s['distance_m'] is None for s in r['steps']))
        self.assertIn('动作开始状态缺失',(Path(r['report_dir'])/'report.html').read_text('utf8'))

    def test_event_log_failure_does_not_block_report_or_other_events(self):
        self.r.event_log_path=Path(self.tmp.name)/'missing_parent/events.jsonl'
        self.begin();self.event('stop',1,reason='test');self.r.poll()
        self.assertEqual(self.r.latest_report['outcome'],'stopped')
        self.assertIsNotNone(self.r.snapshot()['event_log_error'])

    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.r=RouteReport(self.tmp.name,clock=lambda:20)
    def tearDown(self):self.r.close();self.tmp.cleanup()
    def event(self,kind,t,sample=None,**kw):self.r.event(dict(kind=kind,monotonic=t,**kw),sample)
    def begin(self):self.event('start_requested',0,pose(0),session=12)
    def state(self,state,index,t,sample):
        lap,leg,action=STEPS[index]
        self.event('mcu_state',t,sample,session=12,state=state,step=index,lap=lap,leg=leg,action=action)
    def test_vehicle_axes_never_flip_twice(self):
        p=pose(1,x=2,y=3);self.assertEqual((vehicle_pose(p)['x'],vehicle_pose(p)['y']),(2,3))
        del p['vehicle_x_m'];del p['vehicle_y_m']
        self.assertEqual((vehicle_pose(p)['x'],vehicle_pose(p)['y']),(2,3))
    def test_two_steps_exclude_pause_from_action_speed(self):
        self.begin();self.state('MOVE',0,3,pose(3))
        self.r.pose(pose(4,y=1,d=1));self.r.pose(pose(5,y=2,d=2))
        self.state('PAUSED',0,5,pose(5,y=2,d=2));self.r.poll()
        self.assertIsNone(self.r.latest_report);self.assertIsNotNone(self.r.run)
        self.state('MOVE',1,8,pose(8,y=2,d=2))
        self.r.pose(pose(9,x=1,y=2,d=3));self.r.pose(pose(10,x=2,y=2,d=4))
        self.state('DONE',1,10,pose(10,x=2,y=2,d=4));self.event('completed',10);self.r.poll()
        r=self.r.latest_report
        self.assertEqual((r['completed_steps'],r['action_duration_s'],r['waiting_duration_s']),(2,4,6))
        self.assertEqual(r['measured_distance_m'],4);self.assertEqual(r['average_action_speed_m_s'],1)
        self.assertEqual([s['distance_m'] for s in r['steps']],[2,2])
        self.assertEqual((self.r.points[0][:2],self.r.points[-1][:2]),((0,0),(2,2)))
        target=Path(r['report_dir'])
        for name in ('trajectory.csv','summary.json','report.html','trajectory.png'):self.assertTrue((target/name).is_file(),name)
        self.assertIn('Measured speed over time',(target/'report.html').read_text('utf8'))
        with (target/'trajectory.csv').open(encoding='utf8',newline='') as f:self.assertEqual(len(list(csv.DictReader(f))),r['sample_count'])
        self.r.close();self.assertEqual(self.r.latest_report['outcome'],'completed')
    def test_all_twelve_steps_close_once(self):
        self.begin()
        for i in range(12):
            start=3+2*i;sample=pose(start,d=i*.2)
            self.state('TURN' if 'TURN' in STEPS[i][2] or i==11 else 'MOVE',i,start,sample)
            end=pose(start+1,x=(i+1)*.2,d=(i+1)*.2)
            self.r.pose(end);self.state('DONE' if i==11 else 'PAUSED',i,start+1,end)
        self.event('completed',26);self.r.poll()
        self.assertEqual(self.r.latest_report['completed_steps'],12)
        self.assertEqual(len(self.r.latest_report['steps']),12)
    def test_stop_partial_and_new_start_create_separate_reports(self):
        self.begin();self.state('MOVE',0,3,pose(3));self.event('stop',4,pose(4,y=.5,d=.5),reason='Operator stop',fault=False)
        self.r.poll();first=self.r.latest_report
        self.assertEqual(first['outcome'],'stopped');self.assertFalse(first['steps'][0]['completed'])
        self.event('start_requested',6,pose(6,y=.5,d=.5),session=13);self.r.poll()
        self.assertNotEqual(first['run_id'],self.r.run['run_id']);self.assertEqual(self.r.run['measured_distance_m'],0)
    def test_counter_reset_and_invalid_pose_are_recorded_without_invented_distance(self):
        self.begin();self.r.pose(pose(.1,x=1,d=1));self.r.pose(pose(.2,x=1,d=0,epoch=2,valid=False,gaps=1))
        self.r.pose(pose(.3,x=1.5,d=.5,epoch=2,valid=False,gaps=1));self.event('stop',.4,reason='test',fault=False);self.r.poll()
        self.assertEqual(self.r.latest_report['measured_distance_m'],1.5)
        self.assertEqual(self.r.latest_report['data_breaks'],1);self.assertEqual(self.r.latest_report['invalid_pose_samples'],2)
        self.assertTrue(self.r.points[2][4]);self.assertEqual(self.r.latest_report['outcome'],'stopped')
    def test_no_start_creates_no_report_and_no_data_is_labeled(self):
        self.r.pose(pose(1));self.r.poll();self.assertIsNone(self.r.run);self.assertEqual(list(Path(self.tmp.name).iterdir()),[])
        self.event('start_requested',2,session=12);self.event('stop',3,reason='test');self.r.poll()
        self.assertFalse(self.r.latest_report['measurement_available'])
        self.assertIn('遥测样本不足',(Path(self.r.latest_report['report_dir'])/'report.html').read_text('utf8'))
    def test_save_failure_does_not_propagate_to_motor_control(self):
        self.begin();self.event('stop',1,reason='test')
        with patch.object(self.r,'_html',side_effect=OSError('disk full')):self.r.poll()
        self.assertIn('disk full',self.r.error)
    def test_raw_pose_callback_keeps_more_samples_than_ui_and_copies_input(self):
        class Writer:
            def write(self,p):pass
        link=MotionLink(writer=Writer());link.pose_sink=self.r.pose
        self.begin()
        for i in range(1,11):link.receive_pose(pose(i*.1,x=i*.1,d=i*.1))
        self.r.poll();self.assertEqual(self.r.run['sample_count'],11)
        self.assertAlmostEqual(self.r.run['measured_distance_m'],1)
    def test_overlay_preserves_menu_pl_numeric_region_and_alpha(self):
        a=np.zeros((480,640,4),dtype=np.uint8);a[:,:,3]=77;self.r.overlay(a)
        self.assertTrue(a[140:328,388:632,:3].any());self.assertFalse(a[:128,:,:3].any())
        self.assertFalse(a[336:,:,:3].any());self.assertTrue((a[:,:,3]==77).all())


if __name__=='__main__':unittest.main()
