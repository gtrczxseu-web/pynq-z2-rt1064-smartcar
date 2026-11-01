import json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
import route_settings as routes
from motion_link import MotionLink,decode_status
from test_motion import Clock,Writer,status_packet
from rear_projection import defaults,project,fit_ground,RearProjection,save
from standalone import Application
from boot_display import BootDisplay


class RecipeTests(unittest.TestCase):
    def paused_link(self,mode='step'):
        recipe=routes.preset('rectangle',.4,.2,mode)
        link,clock,writer=self.link(recipe);link.recipe=recipe;link.session=123;link.sequence=100
        self.status(link,clock,9,step=0,seq=100,version=5);link.state='PAUSED';link.paused_at=clock()
        return link,clock,writer
    def test_continue_retries_reuse_one_sequence(self):
        link,clock,writer=self.paused_link();self.assertTrue(link.toggle());link.step()
        first=writer.packets[-1];self.assertEqual(first[3],4)
        for unused in range(4):clock.now+=.11;link.step()
        self.assertEqual(writer.packets,[first]*5)
    def test_unobserved_move_between_pauses_needs_a_new_manual_press(self):
        link,clock,writer=self.paused_link();link.toggle();link.step();count=len(writer.packets)
        self.status(link,clock,9,step=1,seq=link.sequence,version=5);clock.now+=.11;link.step()
        self.assertEqual(link.state,'PAUSED');self.assertEqual(len(writer.packets),count)
        clock.now+=5;link.step();self.assertEqual(len(writer.packets),count)
        self.assertTrue(link.toggle());link.step();self.assertEqual(len(writer.packets),count+1)
        self.assertNotEqual(writer.packets[-1],writer.packets[0])
    def test_continuous_wait_restarts_for_each_completed_step(self):
        link,clock,writer=self.paused_link('continuous');clock.now+=.36;link.step()
        self.assertEqual(len(writer.packets),1)
        self.status(link,clock,9,step=1,seq=link.sequence,version=5);link.step()
        self.assertEqual(link.state,'PAUSED');clock.now+=.34;link.step();self.assertEqual(len(writer.packets),1)
        clock.now+=.02;link.step();self.assertEqual(len(writer.packets),2)

    def test_rejected_extended_speed_has_a_visible_reason(self):
        link,clock,writer=self.link(routes.preset('outback',2,speed=200))
        self.status(link,clock,0,version=4);self.assertFalse(link.toggle())
        self.assertIn('V5',link.fault or '')
        with tempfile.TemporaryDirectory() as tmp:
            a=Application(tmp,{},display=BootDisplay());a.motion=link
            with patch.object(a,'status'):a.set_phase('RUNNING','')
            self.assertEqual(a.phase,'MOTION FAULT');self.assertIn('V5',a.display.content()['detail'])
            a.report.close();a.vision.close()
        self.assertEqual(writer.packets,[])

    def test_initialization_loads_saved_continuous_mode_before_first_press(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as tmp:
            recipe=routes.preset('outback',.8,mode='continuous',speed=140);routes.save(tmp,recipe)
            link,clock,writer=self.link(routes.default_settings())
            a=Application(tmp,{},session=SimpleNamespace(uart=None,log_dir=Path(tmp)),motion_factory=lambda uart:link)
            with patch('standalone.SensingApplication.calibrate',return_value=True),patch.object(link,'start'):
                self.assertTrue(a.calibrate())
            self.assertEqual(link.recipe,recipe);self.assertEqual(writer.packets,[])
            a.report.close();a.vision.close()

    def test_motion_event_callback_does_not_write_to_disk(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as tmp:
            link,clock,writer=self.link(routes.default_settings())
            a=Application(tmp,{},session=SimpleNamespace(uart=None,log_dir=Path(tmp)),motion_factory=lambda uart:link)
            with patch('standalone.SensingApplication.calibrate',return_value=True),patch.object(link,'start'):
                a.calibrate()
            record=dict(kind='command',monotonic=clock(),op=4)
            with patch.object(Path,'open',side_effect=AssertionError('Disk IO on motion/RX callback')):
                link.log_sink(record)
            a.report.poll();self.assertTrue((Path(tmp)/'motion_events.jsonl').exists())
            self.assertEqual(json.loads((Path(tmp)/'motion_events.jsonl').read_text())['op'],4)
            a.report.close();a.vision.close()

    def test_extended_speed_requires_v5_and_full_configuration_preserves_value(self):
        recipe=routes.preset('outback',2,speed=200)
        self.assertEqual(routes.configuration(recipe)[1],(0x11,160))
        link,clock,writer=self.link(recipe);self.status(link,clock,0,version=4)
        self.assertFalse(link.toggle());self.assertEqual(writer.packets,[])
        self.status(link,clock,0,version=5);self.assertTrue(link.toggle());link.step()
        self.assertEqual(writer.packets[-1][2:4],bytes([5,1]))
        with self.assertRaises(ValueError):routes.preset(speed=201)
    def test_fixed_full_turn_encoding_is_not_zero_heading(self):
        for degrees in (180,360,720):
            recipe=routes.preset(turn_degrees=degrees);cfg=routes.configuration(recipe)
            self.assertEqual(recipe['steps'][5]['rotation_deg'],degrees)
            self.assertIn((0x55,degrees*10),cfg);self.assertIn((0x5b,degrees*10),cfg)
        self.assertFalse(routes.requires_v3(routes.preset(turn_degrees=90)))
    def test_full_turn_recipe_requires_v4_before_arming(self):
        link,clock,writer=self.link(routes.preset(turn_degrees=720));self.status(link,clock,0,version=3)
        self.assertFalse(link.toggle());self.assertEqual(writer.packets,[])
        self.status(link,clock,0,version=4);self.assertTrue(link.toggle());link.step()
        self.assertEqual(writer.packets[-1][2:4],bytes([4,1]))
    def test_presets_and_wire_units(self):
        rectangle=routes.preset('rectangle',1,.6,'continuous',50)
        self.assertEqual(len(rectangle['steps']),4)
        cfg=routes.configuration(rectangle)
        self.assertEqual(cfg[:2],[(0x10,4),(0x11,40)])
        self.assertIn((0x20,500),cfg);self.assertIn((0x21,(2<<14)|300),cfg)
        self.assertEqual(cfg[-1],(0x12,0x1234))
    def test_validate_and_atomic_save(self):
        for change in [dict(mode='auto'),dict(speed_percent=0),dict(steps=[]),dict(steps=[dict(kind='turn',heading_deg=float('nan'))])]:
            with self.assertRaises(ValueError):routes.validate(dict(routes.default_settings(),**change))
        with tempfile.TemporaryDirectory() as tmp:
            recipe=routes.preset('outback',.8);routes.save(tmp,recipe);self.assertEqual(routes.load(tmp),recipe)
    def link(self,recipe):
        clock=Clock();writer=Writer();link=MotionLink(writer=writer,clock=clock,session_factory=lambda:123,settings_loader=lambda:recipe)
        return link,clock,writer
    def status(self,link,clock,state,seq=0,ack=0,step=0,version=3):
        self.tick=getattr(self,'tick',0)+5
        link.receive(decode_status(status_packet(state,link.session,seq,step,ack,self.tick,version),clock()))
    def test_full_configuration_handshake_then_start(self):
        recipe=routes.preset('rectangle',1,.8,'step',50);link,clock,writer=self.link(recipe)
        self.status(link,clock,0);self.assertTrue(link.toggle());link.step()
        self.assertEqual(writer.packets[-1][3],1)
        self.status(link,clock,1,link.sequence);clock.now+=.11;link.step()
        for op,value in routes.configuration(recipe):
            p=writer.packets[-1];self.assertEqual(p[2:4],bytes([3,op]));self.assertEqual(int.from_bytes(p[8:10],'little'),value)
            self.status(link,clock,1,value,op);clock.now+=.11;link.step()
        self.assertEqual(writer.packets[-1][3],2);self.assertEqual(link.state,'STARTING')
    def test_config_waits_for_opcode_and_value_ack(self):
        link,clock,writer=self.link(routes.default_settings());self.status(link,clock,0);link.toggle();link.step()
        self.status(link,clock,1,1);clock.now+=.11;link.step()
        self.status(link,clock,1,12,0x11);clock.now+=.11;link.step()
        self.assertEqual(link.config_index,0);self.assertEqual(writer.packets[-1][3],0x10)
        link.stop();clock.now+=10;link.step();self.assertEqual(writer.packets[-1][3],0)
    def test_continuous_requires_explicit_start_and_stop_cancels_auto_next(self):
        recipe=routes.default_settings();recipe['mode']='continuous';link,clock,writer=self.link(recipe)
        self.status(link,clock,0,version=2);link.step();self.assertEqual(writer.packets,[])
        link.toggle();link.step();self.status(link,clock,1,link.sequence,version=2);clock.now+=.11;link.step()
        self.status(link,clock,9,link.sequence,version=2);link.step();self.assertEqual(link.state,'PAUSED')
        clock.now+=.9;link.step();self.assertEqual(writer.packets[-1][3],4);self.assertEqual(link.state,'RESUMING')
        link.stop();n=len(writer.packets);clock.now+=10;link.step();self.assertEqual(len(writer.packets),n)
    def test_old_car_default_compatible_custom_rejected_before_arm(self):
        link,clock,writer=self.link(routes.preset('outback',1));self.status(link,clock,0,version=2)
        self.assertFalse(link.toggle());self.assertEqual(writer.packets,[])
        link.settings_loader=routes.default_settings;self.assertTrue(link.toggle())
    def test_custom_action_description_uses_current_recipe(self):
        recipe=routes.preset('outback',1);link,clock,writer=self.link(recipe)
        self.status(link,clock,0);link.toggle();self.status(link,clock,3,step=1)
        self.assertEqual(link.mcu['action'],'BACK');self.assertEqual(link.mcu['total_steps'],2)


class ProjectionTests(unittest.TestCase):
    def test_estimated_trail_at_two_metres_is_inside_selected_floor(self):
        c=defaults();c['ground_polygon']=[[0,.67],[1,.67],[1,1],[0,1]]
        near=project((0,-1),(0,0),0,c);far=project((0,-2),(0,0),0,c)
        self.assertGreaterEqual(far[1],round(.67*239))
        self.assertLess(far[1],near[1]);self.assertLess(near[1],240)
        with tempfile.TemporaryDirectory() as tmp:
            save(tmp,c);r=RearProjection(tmp,clock=lambda:10)
            frame=np.zeros((240,320,3),dtype=np.uint8)
            r.overlay(frame,[(0,-2,0,True,False),(0,-1,0,True,False)],(0,0),0)
            self.assertTrue(frame[160:220,150:170].any())
            self.assertFalse(frame[:160].any())
    def test_rear_view_swaps_vehicle_right_and_excludes_front(self):
        c=defaults();c['camera_forward_m']=0
        self.assertIsNone(project((0,1),(0,0),0,c))
        left=project((.1,-1),(0,0),0,c);center=project((0,-1),(0,0),0,c)
        self.assertLess(left[0],center[0]);c['mirror']=True
        self.assertGreater(project((.1,-1),(0,0),0,c)[0],center[0])
    def test_global_rotation_preserves_rear_projection(self):
        c=defaults();c['camera_forward_m']=0
        self.assertEqual(project((0,-1),(0,0),0,c),project((-1,0),(0,0),-90,c))
    def test_measured_homography_and_degenerate_rejection(self):
        points=[[-.5,.5,.1,.9],[.5,.5,.9,.9],[.5,2,.7,.6],[-.5,2,.3,.6]]
        h=np.asarray(fit_ground(points))
        for x,y,u,v in points:
            p=h.dot([x,y,1]);self.assertTrue(np.allclose(p[:2]/p[2],[u,v]))
        with self.assertRaises(ValueError):fit_ground([[0,y,.5,.8] for y in range(4)])
    def test_projection_pixels_only_on_visible_floor_and_stale_data_no_trail(self):
        with tempfile.TemporaryDirectory() as tmp:
            c=defaults();c.update(pitch_down_deg=0,camera_forward_m=0);save(tmp,c)
            r=RearProjection(tmp,clock=lambda:10);frame=np.zeros((240,320,3),dtype=np.uint8)
            points=[(0,-.2,0,True,False),(0,-1,0,True,False)]
            r.overlay(frame,points,(0,0),0,fresh=True)
            # Camera ground polygon starts at 62%; label also stays at the bottom.
            self.assertFalse(frame[:145].any());self.assertTrue(frame[150:].any())
            r.last_render=-1e9;frame[:]=0;r.overlay(frame,points,(0,0),0,fresh=False)
            self.assertFalse(frame[:220].any())
    def test_gap_does_not_draw_a_false_connector(self):
        with tempfile.TemporaryDirectory() as tmp:
            c=defaults();c.update(pitch_down_deg=0,camera_forward_m=0);save(tmp,c)
            r=RearProjection(tmp,clock=lambda:10);frame=np.zeros((240,320,3),dtype=np.uint8)
            r.overlay(frame,[(0,-.2,0,True,False),(0,-1,0,True,True)],(0,0),0,fresh=True)
            self.assertFalse(frame[:220].any())


if __name__=='__main__':unittest.main()
