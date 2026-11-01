"""Single STEP actions and the V7 angular speed configuration contract."""
import sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
import route_settings as routes
from settings_menu import SettingsMenu
from motion_link import MotionLink,decode_status
from test_motion import Clock,Writer,status_packet
from telemetry_reference import CAMERA_INITIAL


class SingleActionTests(unittest.TestCase):
    def test_translate_has_one_forward_move_with_camera_front_axes(self):
        for distance in (2,2.2,4):
            r=routes.preset('translate',length=distance,speed=120)
            self.assertEqual(r['steps'],[dict(kind='move',direction='FORWARD',distance_m=distance,heading_deg=0)])
            cfg=routes.configuration(r,CAMERA_INITIAL)
            self.assertEqual(dict(cfg)[0x20],(1<<14)+round(distance/.002))
            self.assertNotIn(0x13,dict(cfg));self.assertEqual(dict(cfg)[0x10],1)

    def test_rotate_fixed_angles_and_angular_caps(self):
        for angle in (90,180,360,720):
            for speed in (20,100,200):
                r=routes.preset('rotate',turn_degrees=angle,speed=speed)
                self.assertEqual(r['steps'],[dict(kind='turn',heading_deg=0,rotation_deg=angle)])
                cfg=dict(routes.configuration(r))
                self.assertEqual(cfg[0x50],angle*10);self.assertEqual(cfg[0x13],round(.4*speed))
                self.assertEqual(cfg[0x10],1)

    def test_single_routes_are_step_only(self):
        for name in ('translate','rotate'):
            with self.assertRaises(ValueError):routes.preset(name,mode='continuous')
        with self.assertRaises(ValueError):routes.preset('rotate',speed=10)
        with self.assertRaises(ValueError):routes.preset('rotate',turn_degrees=270)

    def test_legacy_routes_do_not_change_the_angular_cap(self):
        for name in ('competition','rectangle','outback'):
            for speed in (20,100,200):self.assertNotIn(0x13,dict(routes.configuration(routes.preset(name,speed=speed))))

    def test_menu_cycles_new_routes_only_in_step(self):
        with tempfile.TemporaryDirectory() as tmp:
            m=SettingsMenu(tmp);m.begin();m.cursor=1
            for name in ('rectangle','outback','translate','rotate','competition'):
                m.change();self.assertEqual(m.name,name)
            m.cursor=0;m.change();m.cursor=1
            for name in ('rectangle','outback','competition'):
                m.change();self.assertEqual(m.name,name)

    def test_rotate_menu_saves_angle_and_speed_and_skips_distances(self):
        with tempfile.TemporaryDirectory() as tmp:
            routes.save(tmp,routes.preset('rotate',speed=200,turn_degrees=720))
            m=SettingsMenu(tmp);m.begin();self.assertEqual(m.fields(),[0,1,4,5])
            self.assertEqual(m.snapshot()['rows'][2][1],'--')
            self.assertEqual(m.snapshot()['rows'][4][0],'TURN SPEED')
            m.cursor=4;m.change();self.assertEqual(m.speed,20)
            m.next();m.change();self.assertEqual(m.angle,90)
            self.assertEqual(m.confirm(),routes.preset('rotate',speed=20,turn_degrees=90))
            m.begin();self.assertEqual((m.speed,m.angle),(20,90))
            m.cursor=0;m.change();self.assertEqual(m.name,'competition')

    def test_translate_menu_wraps_and_saves_only_distance(self):
        with tempfile.TemporaryDirectory() as tmp:
            routes.save(tmp,routes.preset('translate',length=4))
            m=SettingsMenu(tmp);m.begin();self.assertEqual(m.fields(),[0,1,2,4])
            m.cursor=2;m.change();m.change()
            self.assertEqual(m.confirm(),routes.preset('translate',length=2.2))

    def test_v6_rotate_is_rejected_before_any_uart_command(self):
        clock=Clock();writer=Writer();m=MotionLink(writer=writer,clock=clock,settings_loader=lambda:routes.preset('rotate'))
        m.receive(decode_status(status_packet(version=6),clock()))
        self.assertFalse(m.toggle());self.assertIn('V7',m.fault);self.assertEqual(writer.packets,[])
        self.assertFalse(m.snapshot()['compatible'])

    def test_v6_translate_and_v7_rotate_can_arm(self):
        for version,name in ((6,'translate'),(7,'rotate')):
            clock=Clock();writer=Writer();m=MotionLink(writer=writer,clock=clock,session_factory=lambda:123,settings_loader=lambda:routes.preset(name))
            m.receive(decode_status(status_packet(version=version),clock()))
            self.assertTrue(m.toggle());m.step()
            self.assertEqual(writer.packets[-1][2:4],bytes([version,1]));self.assertTrue(m.snapshot()['compatible'])


if __name__=='__main__':unittest.main()
