import sys,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
from boot_display import BootDisplay
from trajectory_report import RouteReport


class DisplayTests(unittest.TestCase):
    def test_video_is_exact_four_by_three_and_alpha_is_preserved(self):
        source=np.arange(480*640*4,dtype=np.uint8).reshape((480,640,4));output=source.copy()
        d=BootDisplay();d.context=dict(camera=True,mcu=dict(lap=1,step=0,action='FORWARD'))
        d.update('DRIVING','Long technical message should not become a menu');d.overlay(output)
        self.assertTrue(np.array_equal(output[88:328,12:332,:3],source[::2,::2,:3]))
        self.assertTrue(np.array_equal(output[:,:,3],source[:,:,3]))
        self.assertTrue((output[336:,:,:3]==(26,16,8)).all())
    def test_notice_hides_camera_and_trace_and_shortens_raw_calibration_error(self):
        called=[];d=BootDisplay();d.route_overlay=lambda output:called.append(True)
        d.context=dict(camera=True);d.update('WAIT STILL','Insufficient RAW33 frames: frames=0, uart={lots of debug fields}')
        a=np.ones((480,640,4),dtype=np.uint8);d.overlay(a)
        self.assertEqual(called,[]);self.assertIn('CONTROLLER',d.content()['heading'])
        self.assertNotIn('debug',str(d.content()));self.assertNotIn('RAW33',str(d.content()))
    def test_run_pause_finish_use_different_primary_actions(self):
        d=BootDisplay();d.context=dict(camera=True,mcu=dict(step=1,lap=1,action='RIGHT'))
        for phase,kind,hint in [('READY','scene','BTN0 START'),('DRIVING','scene','BTN1 STOP'),('STEP PAUSED','scene','BTN0 NEXT'),('FINISHED','result','BTN0 NEW RUN')]:
            d.update(phase,'');c=d.content();self.assertEqual((c['kind'],c['hint']),(kind,hint))
        self.assertNotIn('BTN3',str(d.content()))
    def test_countdown_updates_without_changing_motion_state(self):
        now=[0];d=BootDisplay(clock=lambda:now[0]);d.update('COUNTDOWN','')
        self.assertEqual(d.content()['heading'],'START IN 3');now[0]=1.2
        self.assertEqual(d.content()['heading'],'START IN 2');self.assertEqual(d.phase,'COUNTDOWN')
    def test_map_and_video_have_separate_regions(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            report=RouteReport(tmp,rect=(348,88,280,240),minimal=True)
            source=np.full((480,640,4),100,dtype=np.uint8);d=BootDisplay()
            d.route_overlay=report.overlay;d.context=dict(camera=True);d.update('READY','')
            d.overlay(source)
            self.assertTrue((source[88:328,12:332,:3]==100).all())
            self.assertFalse((source[88:328,348:628,:3]==100).all());report.close()
    def test_finished_frame_uses_results_instead_of_live_camera(self):
        d=BootDisplay();d.context=dict(camera=True,summary=dict(measured_distance_m=8.04,action_duration_s=17.3,completed_steps=12))
        d.update('FINISHED','Both laps complete');a=np.full((480,640,4),100,dtype=np.uint8);d.overlay(a)
        self.assertFalse((a[88:328,12:332,:3]==100).all());self.assertEqual(d.content()['kind'],'result')


if __name__=='__main__':unittest.main()
