"""Logical camera-front reference and actual UART payload checks; no vehicle IO."""
import csv,json,math,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
from telemetry_reference import CAMERA_INITIAL,VEHICLE_INITIAL
import route_settings as routes
from trajectory_report import RouteReport,vehicle_pose
from rear_projection import defaults,project
from motion_link import MotionLink,decode_status
from standalone import Application
from test_motion import Clock,Writer,status_packet
from test_trajectory_report import pose


class CameraForwardTests(unittest.TestCase):
    def test_each_logical_direction_maps_to_the_opposite_original_vehicle_direction(self):
        for direction,wire in [('FORWARD',1),('BACK',0),('RIGHT',3),('LEFT',2)]:
            recipe=dict(routes.default_settings(),steps=[dict(kind='move',direction=direction,distance_m=.4,heading_deg=0)])
            config=dict(routes.configuration(recipe,CAMERA_INITIAL))
            self.assertEqual(config[0x20]>>14,wire)
            self.assertEqual(config[0x20]&0x3fff,200)
            self.assertEqual(config[0x30],0)
    def test_turn_direction_full_turn_count_speed_and_distances_are_not_flipped(self):
        for angle in (90,180,360,720):
            recipe=routes.preset(turn_degrees=angle,speed=100)
            original=dict(routes.configuration(recipe,VEHICLE_INITIAL));camera=dict(routes.configuration(recipe,CAMERA_INITIAL))
            for op,value in original.items():
                if 0x20<=op<0x2c:self.assertEqual(camera[op],value^(1<<14))
                else:self.assertEqual(camera[op],value)
    def test_default_route_requires_configuration_and_wire_handshake_uses_camera_mapping(self):
        clock=Clock();writer=Writer();link=MotionLink(writer=writer,clock=clock,session_factory=lambda:123)
        link.reference_frame=CAMERA_INITIAL
        link.receive(decode_status(status_packet(version=2),clock()))
        self.assertFalse(link.toggle());self.assertEqual(writer.packets,[])
        ticks=5
        link.receive(decode_status(status_packet(version=4,ticks=ticks),clock()));self.assertTrue(link.toggle());link.step()
        self.assertEqual(writer.packets[-1][3],1)
        ticks+=5;link.receive(decode_status(status_packet(1,123,link.sequence,ticks=ticks,version=4),clock()));clock.now+=.11;link.step()
        for op,value in routes.configuration(link.recipe,CAMERA_INITIAL):
            packet=writer.packets[-1]
            self.assertEqual(packet[3],op);self.assertEqual(int.from_bytes(packet[8:10],'little'),value)
            ticks+=5;s=decode_status(status_packet(1,123,value,ticks=ticks,version=4),clock());s['config_ack']=op
            link.receive(s);clock.now+=.11;link.step()
        self.assertEqual(writer.packets[-1][3],2)
        self.assertEqual(link.snapshot()['reference_frame'],CAMERA_INITIAL)
    def test_actual_application_selects_the_same_reference_for_control_map_projection_and_camera(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer=Writer();link=MotionLink(writer=writer)
            app=Application(tmp,{},session=SimpleNamespace(uart=None,log_dir=Path(tmp)),motion_factory=lambda uart:link)
            with patch('standalone.SensingApplication.calibrate',return_value=True),patch.object(link,'start'):
                self.assertTrue(app.calibrate())
            self.assertEqual(link.reference_frame,CAMERA_INITIAL)
            self.assertEqual(app.report.reference_frame,CAMERA_INITIAL)
            self.assertEqual(app.report.projection.reference_frame,CAMERA_INITIAL)
            self.assertEqual(app.vision.snapshot()['camera_view'],'forward')
            self.assertEqual(writer.packets,[]);app.report.close();app.vision.close()
    def test_raw_original_backward_becomes_positive_camera_forward_once(self):
        sample=pose(1,x=-.2,y=-.4);before=dict(sample)
        p=vehicle_pose(sample,CAMERA_INITIAL)
        self.assertEqual((p['x'],p['y']),(.2,.4));self.assertEqual(sample,before)
        del sample['vehicle_x_m'];del sample['vehicle_y_m']
        self.assertEqual((vehicle_pose(sample,CAMERA_INITIAL)['x'],vehicle_pose(sample,CAMERA_INITIAL)['y']),(.2,.4))
    def test_camera_front_projection_is_visible_and_old_trail_behind_camera_is_not(self):
        c=defaults()
        self.assertEqual(project((0,-1),(0,0),0,c),project((0,1),(0,0),0,c,reference_frame=CAMERA_INITIAL))
        self.assertIsNone(project((0,-1),(0,0),0,c,reference_frame=CAMERA_INITIAL))
        for heading in (0,90,180,270):
            a=math.radians(heading);front=(-math.sin(a),math.cos(a));front_right=(.1*math.cos(a)-math.sin(a),.1*math.sin(a)+math.cos(a))
            center=project(front,(0,0),heading,c,reference_frame=CAMERA_INITIAL)
            right=project(front_right,(0,0),heading,c,reference_frame=CAMERA_INITIAL)
            self.assertGreater(right[0],center[0])
    def test_new_report_map_markers_and_csv_use_positive_camera_forward(self):
        with tempfile.TemporaryDirectory() as tmp:
            clock=[1.];r=RouteReport(tmp,clock=lambda:clock[0],reference_frame=CAMERA_INITIAL)
            r.event(dict(kind='start_requested',monotonic=1,session=12),pose(1));r.poll()
            for t,d in ((1.1,.2),(1.6,.4)):
                clock[0]=t;s=pose(t,y=-d,d=d);r.pose(s);r.vision(t,[dict(label='BLUE')],s,12);r.poll()
            self.assertEqual(r.points[-1][:2],(0,.4));event=r.run['recognitions'][0]
            self.assertEqual(event['y_m'],.4);self.assertEqual(event['camera_view'],'forward')
            r.event(dict(kind='stop',monotonic=2,reason='Offline direction test'));r.poll();self.assertIsNone(r.error)
            report=r.latest_report;target=Path(report['report_dir'])
            self.assertEqual(report['final_pose']['y_m'],.4);self.assertEqual(report['reference_frame'],CAMERA_INITIAL)
            self.assertIn('camera forward',report['axes'])
            with (target/'trajectory.csv').open(encoding='utf8',newline='') as f:
                rows=list(csv.DictReader(f));self.assertEqual(float(rows[-1]['camera_y_m']),.4)
            self.assertIn('镜头朝向为新车头',(target/'report.html').read_text('utf8'));r.close()


if __name__=='__main__':unittest.main()
