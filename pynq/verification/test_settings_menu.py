import json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
from boot_controls import ButtonEvents
from boot_display import BootDisplay
from settings_menu import SettingsMenu
from standalone import Application
import route_settings as routes


class MenuTests(unittest.TestCase):
    def test_selection_is_only_saved_on_confirm(self):
        with tempfile.TemporaryDirectory() as tmp:
            m=SettingsMenu(tmp);m.begin();m.change();self.assertEqual(m.mode,'continuous')
            self.assertFalse((Path(tmp)/'route_settings.json').exists())
            m.cancel();m.begin();self.assertEqual(m.mode,'step')
            m.change();m.confirm();self.assertEqual(routes.load(tmp)['mode'],'continuous')
    def test_visible_values_and_selected_row_update(self):
        with tempfile.TemporaryDirectory() as tmp:
            m=SettingsMenu(tmp);m.begin();d=BootDisplay();d.update('SETTINGS','')
            d.context=dict(settings_menu=m.snapshot());a=np.zeros((480,640,4),dtype=np.uint8);d.overlay(a)
            m.next();m.change();d.context=dict(settings_menu=m.snapshot());b=np.zeros_like(a);d.overlay(b)
            self.assertFalse(np.array_equal(a,b));self.assertEqual(m.snapshot()['rows'][1][1],'RECTANGLE')
            self.assertEqual(d.content()['kind'],'settings')
    def test_distance_speed_and_heading_recipe(self):
        with tempfile.TemporaryDirectory() as tmp:
            m=SettingsMenu(tmp);m.begin();m.cursor=2;m.change();self.assertEqual(m.length,2.2)
            m.speed=70;m.cursor=4;m.change();self.assertEqual(m.speed,90)
            m.cursor=5;m.change();self.assertEqual(m.angle,180)
            r=m.confirm();self.assertEqual(r['steps'][0]['distance_m'],2.2)
            self.assertEqual(r['steps'][5]['rotation_deg'],180);self.assertEqual(r['speed_percent'],90)
    def test_distance_limits_reach_four_then_wrap_to_two(self):
        with tempfile.TemporaryDirectory() as tmp:
            m=SettingsMenu(tmp);m.begin()
            for field,attribute,limit,near in [(2,'length',4,3.95),(3,'width',4,3.99)]:
                m.cursor=field;setattr(m,attribute,.8);m.change()
                self.assertEqual(getattr(m,attribute),2.0)
                m.cursor=field;setattr(m,attribute,near)
                m.change()
                self.assertEqual(getattr(m,attribute),limit)
                if field!=3:self.assertIn('MAX',m.snapshot()['rows'][field][1])
                self.assertIn('4.00 -> 2.00',m.snapshot()['hint'])
                m.change();self.assertEqual(getattr(m,attribute),2.0)
                for expected in [round(2+.2*i,1) for i in range(1,11)]+[2.0]:
                    m.change();self.assertEqual(getattr(m,attribute),expected)
    def test_wrapped_rectangle_dimensions_are_saved_and_reopened(self):
        with tempfile.TemporaryDirectory() as tmp:
            routes.save(tmp,routes.preset('rectangle',4,4,speed=40))
            m=SettingsMenu(tmp);m.begin();m.cursor=2;m.change();m.cursor=3;m.change();m.change()
            saved=m.confirm();self.assertEqual(saved,routes.preset('rectangle',2.0,2.2,speed=40))
            m.begin();self.assertEqual((m.length,m.width,m.speed),(2.0,2.2,40))
    def test_speed_cycles_above_original_limit_and_can_be_lowered_and_saved(self):
        with tempfile.TemporaryDirectory() as tmp:
            m=SettingsMenu(tmp);m.begin();m.cursor=4;self.assertEqual(m.speed,100)
            for expected in (120,140,160,180,200,20,40,60,80,100,120):
                m.change();self.assertEqual(m.speed,expected)
            self.assertEqual(m.confirm()['speed_percent'],120)
            m.begin();self.assertEqual(m.speed,120)
    def test_turn_selection_is_only_four_fixed_options(self):
        with tempfile.TemporaryDirectory() as tmp:
            m=SettingsMenu(tmp);m.begin();m.cursor=5
            for expected in (180,360,720,90):m.change();self.assertEqual(m.angle,expected)
    def test_continuous_competition_keeps_angle_visible_and_saves_fixed_route(self):
        with tempfile.TemporaryDirectory() as tmp:
            routes.save(tmp,routes.preset('competition',2.4,speed=80,turn_degrees=720))
            m=SettingsMenu(tmp);m.begin();m.change()
            self.assertEqual(m.mode,'continuous');self.assertEqual(m.snapshot()['rows'][5],('TURN ANGLE','90 deg FIXED'))
            self.assertNotIn(5,m.fields());m.cursor=4;m.next();self.assertEqual(m.cursor,0)
            r=m.confirm();self.assertEqual(r,routes.preset('competition',2.4,mode='continuous',speed=80))
            self.assertEqual(routes.load(tmp),r)
    def test_continuous_angle_change_is_ignored_and_step_selection_is_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            m=SettingsMenu(tmp);m.begin();m.cursor=5;m.change();self.assertEqual(m.angle,180)
            m.cursor=0;m.change();m.cursor=5;m.change();self.assertEqual(m.angle,180)
            m.cursor=0;m.change();self.assertIn(5,m.fields())
            self.assertEqual(m.snapshot()['rows'][5][1],'180 deg CW')
    def test_custom_actions_are_preserved_when_only_mode_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=routes.preset('outback',.8);r['preset']='custom';routes.save(tmp,r)
            m=SettingsMenu(tmp);m.begin();self.assertEqual(m.fields(),[0,1,4]);m.change();s=m.confirm()
            self.assertEqual(s['steps'],r['steps']);self.assertEqual(s['mode'],'continuous')
    def test_menu_does_not_start_or_stop_motion_and_busy_entry_is_ignored(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=Application(tmp,json.loads((PACKAGE/'autostart.json').read_text('utf8')))
            class Motion:
                active=False;state='STOPPED';recipe=routes.default_settings()
                def toggle(self):raise AssertionError('Menu must not ARM/START')
                def stop(self,reason):raise AssertionError('Menu must not change idle controller')
            a.motion=Motion();a.request('settings');self.assertTrue(a.settings_menu.open)
            a.request('toggle_pause');a.request('retry_camera');a.request('settings_cancel')
            self.assertFalse(a.settings_menu.open);self.assertFalse((Path(tmp)/'route_settings.json').exists())
            a.motion.active=True;a.request('settings');self.assertFalse(a.settings_menu.open)
            a.motion.active=False;a.motion.state='PAUSED';a.request('settings');self.assertFalse(a.settings_menu.open)
    def test_short_menu_keys_do_not_change_legacy_long_press_actions(self):
        def press(bit,hold):
            b=ButtonEvents();events=[]
            for mask,t in [(1<<bit,0),(1<<bit,.06),(1<<bit,hold),(0,hold+.1),(0,hold+.16)]:events+=b.feed(mask,t)
            return events
        self.assertEqual(press(2,.2),['settings']);self.assertEqual(press(2,2.2),['recalibrate'])
        self.assertEqual(press(3,.2),['settings_cancel']);self.assertEqual(press(3,3.2),['shutdown'])
        self.assertEqual(press(0,4),['toggle_pause']);self.assertEqual(press(1,4),['retry_camera'])


if __name__=='__main__':unittest.main()
