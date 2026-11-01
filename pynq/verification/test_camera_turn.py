import json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
from boot_display import BootDisplay
from sensing_app import Application as SensorApplication
from trajectory_report import RouteReport,map_image
from test_trajectory_report import pose


class CameraTests(unittest.TestCase):
    def test_actual_camera_frame_is_visible_despite_old_context(self):
        source=np.full((480,640,4),91,dtype=np.uint8);d=BootDisplay()
        d.context=dict(camera=False);d.update('DRIVING','');d.overlay(source,video_frame=True)
        self.assertTrue((source[88:328,12:332,:3]==91).all())
    def test_missing_actual_frame_never_displays_colorbars_as_video(self):
        source=np.full((480,640,4),91,dtype=np.uint8);d=BootDisplay()
        d.context=dict(camera=True);d.update('DRIVING','');d.overlay(source,video_frame=False)
        self.assertFalse((source[88:328,12:332,:3]==91).all())
    def app(self):
        config=json.loads((PACKAGE/'autostart.json').read_text('utf8'))
        a=SensorApplication(PACKAGE,config,session=SimpleNamespace(cam=None));a.motion=SimpleNamespace(active=False,state='STOPPED')
        return a
    def test_idle_missing_camera_retries_without_motor_commands(self):
        a=self.app()
        with patch.object(a,'connect_camera') as connect:a.retry_missing_camera();connect.assert_called_once()
        self.assertFalse(a.motion.active);self.assertEqual(a.motion.state,'STOPPED')
    def test_active_or_step_paused_does_not_block_video_on_reconnect_attempt(self):
        a=self.app()
        with patch.object(a,'connect_camera') as connect:
            a.motion.active=True;a.retry_missing_camera()
            a.motion.active=False;a.motion.state='PAUSED';a.retry_missing_camera()
            connect.assert_not_called()
    def test_healthy_camera_and_cooldown_prevent_repeated_reconnects(self):
        a=self.app()
        with patch.object(a,'connect_camera') as connect,patch('sensing_app.time.monotonic',return_value=100):
            a.session.cam=SimpleNamespace(closed=False);a.retry_missing_camera()
            a.session.cam=None;a.last_camera_attempt=95;a.retry_missing_camera();connect.assert_not_called()
            a.last_camera_attempt=89;a.retry_missing_camera();connect.assert_called_once()


class TurnTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.r=RouteReport(self.tmp.name,clock=lambda:10)
        self.r.event(dict(kind='start_requested',monotonic=0,session=12),pose(0))
    def tearDown(self):self.r.close();self.tmp.cleanup()
    def state(self,state,t,sample,step=5):
        self.r.event(dict(kind='mcu_state',monotonic=t,session=12,state=state,step=step,lap=2,leg=2,action='TURN CW'),sample)
    def test_turn_highlight_end_marker_and_saved_report(self):
        self.state('TURN',1,pose(1,x=1,y=2,d=2));self.r.poll();self.assertTrue(self.r.turning)
        sample=pose(2,x=1.02,y=2,d=2);sample['vehicle_heading_deg']=-90
        self.state('PAUSED',2,sample);self.r.poll()
        self.assertFalse(self.r.turning);self.assertEqual(len(self.r.turn_markers),1)
        marker=self.r.turn_markers[0]
        self.assertEqual((marker['x_m'],marker['y_m'],marker['heading_deg'],marker['step']),(1.02,2,-90,6))
        self.state('PAUSED',2.1,sample);self.r.poll();self.assertEqual(len(self.r.turn_markers),1)
        self.r.event(dict(kind='stop',monotonic=2.2,reason='test',fault=False));self.r.poll()
        target=Path(self.r.latest_report['report_dir'])
        data=json.loads((target/'summary.json').read_text('utf8'));self.assertEqual(data['turn_markers'][0],marker)
        self.assertIn('T1',(target/'report.html').read_text('utf8'))
    def test_final_turn_done_creates_marker(self):
        self.state('TURN',1,pose(1),step=11);self.state('DONE',2,pose(2,x=.1),step=11)
        self.r.event(dict(kind='completed',monotonic=2));self.r.poll()
        self.assertEqual(len(self.r.latest_report['turn_markers']),1);self.assertFalse(self.r.turning)
    def test_translation_or_aborted_turn_does_not_get_completed_marker(self):
        self.state('MOVE',1,pose(1),step=0);self.state('PAUSED',2,pose(2),step=0)
        self.state('TURN',3,pose(3));self.r.event(dict(kind='stop',monotonic=3.2,reason='Operator stop',fault=False));self.r.poll()
        self.assertEqual(self.r.turn_markers,[]);self.assertFalse(self.r.turning)
    def test_no_fresh_position_records_event_without_fabricating_point(self):
        self.state('TURN',1,pose(1));self.state('PAUSED',2,None);self.r.poll()
        self.assertEqual(len(self.r.turn_markers),1);self.assertIsNone(self.r.turn_markers[0]['x_m'])
        self.assertFalse(self.r.turn_markers[0]['position_available'])
    def test_icon_colors_and_end_ring_are_distinct(self):
        points=[(0,0,0,True,False),(1,1,0,True,False)]
        a=np.asarray(map_image(points,turning=False));b=np.asarray(map_image(points,turning=True))
        self.assertTrue((a==(255,112,93)).all(axis=2).any());self.assertTrue((b==(196,128,255)).all(axis=2).any())
        marker=dict(x_m=0,y_m=0)
        c=np.asarray(map_image(points,turn_markers=[marker]))
        self.assertTrue((c==(255,221,82)).all(axis=2).any())
    def test_new_run_clears_old_turn_marks(self):
        self.state('TURN',1,pose(1));self.state('PAUSED',2,pose(2));self.r.poll()
        self.r.event(dict(kind='start_requested',monotonic=3,session=13),pose(3));self.r.poll()
        self.assertEqual(self.r.turn_markers,[]);self.assertFalse(self.r.turning)


if __name__=='__main__':unittest.main()
