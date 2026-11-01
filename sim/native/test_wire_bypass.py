"""Python UART to actual C route/position/PID; ideal wheel/IMU model only."""
from pathlib import Path
import ctypes,math,sys,unittest
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'pynq'))
from motion_link import MotionLink,decode_status
from route_settings import preset
from telemetry_reference import CAMERA_INITIAL

class WireBypassTests(unittest.TestCase):
    def setUp(self):
        self.lib=ctypes.CDLL(str(OUT/('adapter_bridge.dll' if sys.platform=='win32' else 'adapter_bridge.so')));self.lib.bridge_init()
        for name in ('bridge_x','bridge_y','bridge_goal_x','bridge_goal_y','bridge_motor_peak'):getattr(self.lib,name).restype=ctypes.c_float
        self.now=100.;self.ticks=0;self.sent=[];self.drop={};self.obstacle=None;self.positions=[]
        class Writer:
            def write(writer,p):
                self.sent.append(bytes(p))
                if self.drop.get(p[3],0):self.drop[p[3]]-=1;return
                self.lib.bridge_feed(p,len(p))
        self.link=MotionLink(writer=Writer(),clock=lambda:self.now,session_factory=lambda:123)
        self.link.reference_frame=CAMERA_INITIAL;self.recipe=preset('translate',length=2,speed=80)
        self.link.settings_loader=lambda:self.recipe;self.receive()
    def receive(self):
        p=(ctypes.c_ubyte*18)();self.lib.bridge_status(p);self.link.receive(decode_status(bytes(p),self.now))
        self.link.receive_pose(dict(received_monotonic=self.now,pose_epoch=1,source_clock_epoch=0,pose_valid=True,speed_m_s=0))
    def pump(self,count=1,labels=()):
        for unused in range(count):
            self.now+=.02;self.ticks+=1
            if self.ticks%10==0:
                self.link.receive_vision(self.now,[dict(label=s) for s in labels],self.link.latest,self.link.session)
                if self.obstacle is not None:self.link.receive_obstacle(self.now,dict(state=self.obstacle),self.link.latest,self.link.session,self.link.mcu['step'])
            self.link.step();self.lib.bridge_tick();self.positions.append((self.lib.bridge_x()*.2,self.lib.bridge_y()*.2))
            if self.ticks%5==0:self.receive()
    def until(self,predicate,limit=10000,labels=()):
        for unused in range(limit):
            if predicate():return
            self.pump(labels=labels)
        self.fail(str(self.link.snapshot()))
    def start(self):
        self.assertTrue(self.link.toggle());self.until(lambda:self.link.state=='DRIVING')
    def bypass(self):
        self.obstacle='BLOCKED';self.until(lambda:self.link.state=='BYPASS');self.obstacle=None
    def assert_final_two_metres(self):
        self.until(lambda:self.link.state=='FINISHED');self.assertAlmostEqual(self.lib.bridge_x()*.2,0,delta=.05)
        self.assertAlmostEqual(self.lib.bridge_y()*.2,-2,delta=.05)
        self.assertEqual(self.lib.bridge_motor_peak(),0);self.assertEqual(self.link.mcu['step'],0)
        self.assertEqual(sum(p[3]==2 for p in self.sent),1)
    def test_camera_forward_bypass_goes_to_camera_right_and_credits_forward_progress(self):
        self.start();self.pump(20);self.bypass();self.assert_final_two_metres()
        self.assertLess(min(x for x,y in self.positions),-.45)
        travelled=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(self.positions,self.positions[1:]))
        self.assertAlmostEqual(travelled,3,delta=.1)
        self.assertEqual(sum(p[3]==7 for p in self.sent),1);self.assertFalse(any(p[3]==4 for p in self.sent))
    def test_continuous_route_recovers_from_dropped_packets_and_red_mid_bypass(self):
        self.recipe=preset('rectangle',length=2,width=2,mode='continuous',speed=80);self.drop={5:1,7:1,6:1}
        self.start();self.bypass();self.until(lambda:self.link.mcu['state']=='BYPASS_FORWARD')
        self.until(lambda:self.link.state=='SIGNAL_WAIT',labels=('RED',));goal=(self.lib.bridge_goal_x(),self.lib.bridge_goal_y())
        self.pump(100);self.assertEqual(self.lib.bridge_motor_peak(),0)
        self.until(lambda:self.link.state=='BYPASS',labels=('GREEN',))
        self.assertEqual((self.lib.bridge_goal_x(),self.lib.bridge_goal_y()),goal)
        self.until(lambda:self.link.state=='FINISHED')
        self.assertAlmostEqual(self.lib.bridge_x()*.2,0,delta=.05);self.assertAlmostEqual(self.lib.bridge_y()*.2,0,delta=.05)
        self.assertEqual(self.link.mcu['step'],3);self.assertEqual(sum(p[3]==2 for p in self.sent),1)
        self.assertGreaterEqual(sum(p[3]==7 for p in self.sent),2);self.assertGreaterEqual(sum(p[3]==6 for p in self.sent),2)
        self.assertEqual(sum(e['kind']=='bypass_started' for e in self.link.log),1)
        self.assertEqual(sum(e['kind']=='continue_requested' for e in self.link.log),3)
    def test_confirmed_obstacle_clears_during_braking_but_detour_still_finishes_original_step(self):
        self.start();self.pump(20);self.obstacle='BLOCKED'
        self.until(lambda:self.link.mcu['state']=='HOLDING');self.obstacle='CLEAR'
        self.now+=.001
        self.link.receive_obstacle(self.now,dict(state='CLEAR'),self.link.latest,self.link.session,self.link.mcu['step']);self.link.step()
        self.assertEqual(self.link.obstacle_state,'CLEAR')
        self.until(lambda:self.link.state=='BYPASS');self.obstacle=None
        self.assert_final_two_metres();self.assertEqual(sum(p[3]==7 for p in self.sent),1)
        self.assertFalse(any(p[3]==6 for p in self.sent))
    def test_operator_stop_in_each_bypass_phase_never_restarts_from_green(self):
        for phase in ('BYPASS_RIGHT','BYPASS_FORWARD','BYPASS_LEFT'):
            self.setUp();self.start();self.bypass();self.until(lambda:self.link.mcu['state']==phase)
            self.link.stop('BTN1');self.pump(10);self.assertEqual(self.lib.bridge_motor_peak(),0)
            count=sum(p[3]==7 for p in self.sent);self.pump(150,labels=('GREEN',))
            self.assertEqual(self.link.state,'STOPPED');self.assertEqual(self.link.mcu['state'],'STOPPED')
            self.assertEqual(sum(p[3]==7 for p in self.sent),count)
    def test_forward_obstacle_during_bypass_requires_clear_and_green_without_nested_route(self):
        self.start();self.bypass();self.until(lambda:self.link.mcu['state']=='BYPASS_FORWARD')
        self.obstacle='BLOCKED';self.until(lambda:self.link.state=='SIGNAL_WAIT')
        self.pump(100,labels=('GREEN',));self.assertEqual(self.lib.bridge_motor_peak(),0)
        self.assertEqual(sum(p[3]==7 for p in self.sent),1);self.obstacle='CLEAR'
        self.until(lambda:self.link.snapshot()['obstacle_clear'])
        self.until(lambda:self.link.state=='BYPASS',labels=('GREEN',));self.obstacle=None
        self.assert_final_two_metres();self.assertEqual(sum(p[3]==7 for p in self.sent),1)
    def test_short_remaining_leg_waits_for_clear_then_continues_original_goal(self):
        self.start();self.until(lambda:self.lib.bridge_y()*.2< -1.25);self.obstacle='BLOCKED'
        self.until(lambda:self.link.bypass_note is not None);self.assertEqual(self.link.mcu['state'],'HELD')
        self.assertEqual(self.lib.bridge_motor_peak(),0);self.pump(100)
        self.assertEqual(sum(p[3]==7 for p in self.sent),1);self.obstacle='CLEAR'
        self.until(lambda:self.link.snapshot()['obstacle_clear']);self.assertTrue(self.link.toggle());self.obstacle=None
        self.assert_final_two_metres();self.assertEqual(self.link.mcu['fault'],'OK')
    def test_countdown_red_pause_does_not_try_bypass_before_move_target_exists(self):
        self.recipe=preset('translate');self.recipe.update(mode='continuous',preset='custom')
        self.assertTrue(self.link.toggle());self.until(lambda:self.link.state=='COUNTDOWN');self.obstacle='BLOCKED'
        self.until(lambda:self.link.state=='SIGNAL_WAIT',labels=('RED',));self.pump(100)
        self.assertFalse(any(p[3]==7 for p in self.sent));self.assertEqual(self.lib.bridge_y(),0)
        self.until(lambda:self.link.state=='BYPASS',labels=('GREEN',));self.obstacle=None
        self.assert_final_two_metres();self.assertEqual(sum(p[3]==7 for p in self.sent),1)
    def test_hold_at_completed_step_rejects_bypass_then_recovers_normal_next_step(self):
        self.recipe=preset('outback',length=2,speed=80);self.start();self.until(lambda:self.link.state=='PAUSED')
        self.link.obstacle_hold=True;self.link._traffic_command(5);self.until(lambda:self.link.mcu['state']=='HELD')
        self.obstacle='BLOCKED';self.until(lambda:self.link.bypass_note is not None)
        self.assertEqual(self.link.mcu['fault'],'BYPASS NOT MOVE');self.assertEqual(self.lib.bridge_motor_peak(),0)
        count=sum(p[3]==7 for p in self.sent);self.pump(100);self.assertEqual(sum(p[3]==7 for p in self.sent),count)
        self.obstacle='CLEAR';self.until(lambda:self.link.snapshot()['obstacle_clear'])
        self.until(lambda:self.link.state=='PAUSED',labels=('GREEN',));self.obstacle=None
        self.assertEqual(self.link.mcu['step'],0);self.assertTrue(self.link.toggle());self.until(lambda:self.link.state=='FINISHED')
        self.assertEqual(self.link.mcu['step'],1);self.assertAlmostEqual(self.lib.bridge_y()*.2,0,delta=.05)

if __name__=='__main__':unittest.main(verbosity=2)
