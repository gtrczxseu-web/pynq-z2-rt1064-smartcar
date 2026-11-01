import importlib.util,json,struct,sys,time,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
import motion_link as ml
from motion_link import MotionLink,crc16,decode_status,UartTx
from raw_decoder import RawDecoder
from vehicle_telemetry import TelemetryDecoder,crc8
from boot_display import BootDisplay
from standalone import Application

class Clock:
    def __init__(self):self.now=100.
    def __call__(self):return self.now
    def sleep(self,s):self.now+=s
class Writer:
    def __init__(self):self.packets=[]
    def write(self,p):self.packets.append(bytes(p))
def status_packet(state=0,session=0,seq=0,step=0,fault=0,ticks=1,version=2):
    p=struct.pack('<2sBBIHBBI',b'\xa7\x7a',version,state,session,seq,step,fault,ticks)
    return p+struct.pack('<H',crc16(p))

class LinkTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.writer=Writer();self.link=MotionLink(writer=self.writer,clock=self.clock,session_factory=lambda:123)
        self.ticks=0
    def fresh(self,state=0,session=0,step=0,fault=0):
        self.ticks+=5
        self.link.receive(decode_status(status_packet(state,session,self.link.sequence,step,fault,self.ticks),self.clock()))
        self.link.receive_pose(dict(pose_valid=True,speed_m_s=0,received_monotonic=self.clock()))
    def start(self):
        self.fresh();self.assertTrue(self.link.toggle());self.link.step();self.assertEqual(self.writer.packets[-1][3],1)
        self.clock.now+=.11;self.fresh(1,123);self.link.step();self.assertEqual(self.writer.packets[-1][3],2)
        self.clock.now+=.11;self.fresh(2,123);self.link.step();self.assertEqual(self.writer.packets[-1][3],3)
    def test_crc_vectors_and_command(self):
        self.assertEqual(crc16(b'123456789'),0x29b1)
        p=ml.command(2,0x12345678,65535);self.assertEqual(len(p),12);self.assertEqual(p[:4],b'\xc7\x7c\x02\x02')
        self.assertEqual(int.from_bytes(p[-2:],'little'),crc16(p[:-2]))
    def test_no_automatic_start(self):
        self.fresh()
        for unused in range(30):self.link.step()
        self.assertEqual(self.writer.packets,[]);self.assertFalse(self.link.active)
    def test_button_start_handshake_countdown_and_completion(self):
        self.start();self.assertEqual(self.link.state,'COUNTDOWN')
        self.clock.now+=.11;self.fresh(3,123,0);self.link.step();self.assertEqual(self.link.state,'DRIVING')
        self.clock.now+=.11;self.fresh(5,123,5);self.link.step();self.assertEqual(self.link.state,'TURNING')
        self.clock.now+=.11;self.fresh(6,123,11);self.link.step();self.assertEqual(self.link.state,'FINISHED')
        before=len(self.writer.packets)
        for unused in range(20):self.link.step()
        self.assertEqual(before,len(self.writer.packets))
    def test_btn0_during_action_does_not_abort_or_restart(self):
        self.start();before=len(self.writer.packets);session=self.link.session
        self.assertFalse(self.link.toggle());self.assertEqual(len(self.writer.packets),before)
        self.assertTrue(self.link.active);self.assertEqual(self.link.session,session)
    def test_emergency_stop_still_sends_stop(self):
        self.start();self.link.stop('BTN1');self.assertEqual(self.writer.packets[-1][3],0);self.assertEqual(self.link.state,'STOPPED')
    def test_step_pause_waits_and_resume_preserves_session_and_progress(self):
        self.start();self.clock.now+=.11;self.fresh(9,123,0);self.link.step()
        self.assertEqual(self.link.state,'PAUSED');before=len(self.writer.packets)
        self.clock.now+=60;self.link.step();self.assertEqual(len(self.writer.packets),before)
        self.assertEqual(self.link.state,'PAUSED');self.fresh(9,123,0)
        self.assertTrue(self.link.toggle());self.link.step()
        self.assertEqual(self.writer.packets[-1][3],4);self.assertEqual(self.link.session,123)
        self.clock.now+=.11;self.fresh(3,123,1);self.link.step();self.assertEqual(self.link.state,'DRIVING')
        self.assertEqual(sum(p[3]==2 for p in self.writer.packets),1)
    def test_old_continuous_firmware_cannot_start(self):
        self.fresh();self.link.mcu['protocol_version']=1
        self.assertFalse(self.link.toggle());self.assertEqual(self.writer.packets,[])
    def test_stale_application_does_not_abort_basic_motion(self):
        self.start();self.clock.now+=1.26;self.link.step();self.assertTrue(self.link.active);self.assertEqual(self.writer.packets[-1][3],3)
    def test_invalid_display_pose_does_not_abort_basic_motion(self):
        self.start();self.link.receive_pose(dict(pose_valid=False,received_monotonic=self.clock()))
        self.link.step();self.assertTrue(self.link.active)
    def test_no_mcu_status_is_rejected(self):self.assertFalse(self.link.toggle());self.assertEqual(self.writer.packets,[])
    def test_display_speed_does_not_reject_start(self):
        self.fresh();self.link.latest['speed_m_s']=.1;self.assertTrue(self.link.toggle())
    def test_wrong_session_and_mcu_fault_stop(self):
        self.start();self.fresh(3,124);self.link.step();self.assertEqual(self.link.state,'FAULT')
    def test_mcu_fault_can_retry_with_explicit_press(self):
        self.start();self.fresh(8,123,2,3);self.link.step();self.assertEqual(self.link.state,'FAULT')
        self.assertIn('STALL',self.link.fault)
        self.fresh(7,123,2,3);self.link.session_factory=lambda:124
        before=len(self.writer.packets);self.link.step();self.assertEqual(len(self.writer.packets),before)
        self.assertTrue(self.link.toggle());self.link.step();self.assertEqual(self.writer.packets[-1][3],1)
    def test_source_clock_reset_alone_does_not_latch_fault(self):
        self.start();self.link.receive(decode_status(status_packet(ticks=1),self.clock()));self.assertNotEqual(self.link.state,'FAULT')
    def test_duplicate_status_is_ignored_without_time_gate(self):
        self.start();old=self.link.mcu['received_monotonic'];self.clock.now+=.6
        p=dict(self.link.mcu,received_monotonic=self.clock());self.link.receive(p)
        self.assertEqual(self.link.mcu['received_monotonic'],old);self.link.step();self.assertTrue(self.link.active)
    def test_tx_failure_disables_heartbeats(self):
        self.fresh();self.link.toggle()
        def bad(p):raise OSError('unmapped')
        self.writer.write=bad;self.link.step();self.assertEqual(self.link.state,'FAULT');self.assertFalse(self.link.active)
    def test_log_write_failure_cannot_prevent_stop(self):
        self.start()
        def disk_full(record):raise OSError('disk full')
        self.link.log_sink=disk_full;self.link.stop('operator')
        self.assertEqual(self.writer.packets[-1][3],0);self.assertEqual(self.link.log_error,'disk full')
        self.assertFalse(self.link.toggle())
    def test_10hz_sensor_feed_survives_2hz_ui_callbacks(self):
        self.start()
        for i in range(1,251):
            self.clock.now+=.02
            if i%5==0:
                self.ticks+=5
                self.link.receive(decode_status(status_packet(3,123,self.link.sequence,0,ticks=self.ticks),self.clock()))
                self.link.receive_pose(dict(pose_valid=True,speed_m_s=0,received_monotonic=self.clock()))
            if i%25==0:self.link.receive_pose(self.link.latest)
            self.link.step();self.assertTrue(self.link.active)
        # Sensor display loss does not abort the basic MCU motion path.
        for i in range(31):
            self.clock.now+=.02;self.ticks+=1
            self.link.receive(decode_status(status_packet(3,123,self.link.sequence,0,ticks=self.ticks),self.clock()))
            self.link.receive_pose(None);self.link.step()
        self.assertTrue(self.link.active)
    def test_ui_snapshot_cannot_replace_newer_decoded_pose(self):
        self.fresh();new=dict(pose_valid=True,speed_m_s=0,received_monotonic=self.clock()+.1)
        self.link.receive_pose(new);self.link.receive_pose(dict(new,received_monotonic=self.clock()-.2))
        self.assertEqual(self.link.latest['received_monotonic'],new['received_monotonic'])
    def test_handshake_retries_without_extra_deadline(self):
        self.fresh();self.link.toggle()
        for unused in range(17):
            self.clock.now+=.1;self.fresh();self.link.step()
        self.assertEqual(self.link.state,'ARMING');self.assertEqual(self.writer.packets[-1][3],1)
    def test_old_session_is_not_reused(self):
        self.fresh(7,123);self.assertFalse(self.link.toggle())
    def test_tx_never_reads_rx_status_and_respects_fifo_spacing(self):
        writes=[]
        class MMIO:
            def read(self,offset):raise AssertionError('RX clear-on-read status belongs to native receiver')
            def write(self,offset,b):writes.append((self_clock(),offset,b))
        class Native:
            def __init__(self,mmio):self.mmio=mmio;self.calls=0
            def write(self,packet):
                self.calls+=1
                for b in packet:self.mmio.write(4,b)
        self_clock=self.clock;tx=UartTx(MMIO(),self.clock,self.clock.sleep,native_factory=Native)
        tx.write(ml.command(3,123,1));tx.write(ml.command(0,123,2))
        self.assertEqual(len(writes),24);self.assertGreaterEqual(writes[12][0]-writes[11][0],.00299)
        self.assertTrue(all(offset==4 for unused,offset,b in writes))
        self.assertEqual(tx.native.calls,2)
    def test_native_call_delay_does_not_abort_a_partial_packet(self):
        c=self.clock;packets=[]
        class Native:
            def __init__(self,unused):pass
            def write(self,packet):c.now+=.070;packets.append(bytes(packet))
        tx=UartTx(None,c,c.sleep,native_factory=Native);p=ml.command(3,123,4);tx.write(p)
        self.assertEqual(packets,[p]);self.assertEqual(tx.stats['bursts'],1);self.assertEqual(tx.stats['calls_over_30ms'],1)
    def test_native_failure_still_propagates(self):
        class Native:
            def __init__(self,unused):pass
            def write(self,packet):raise OSError('native mapping lost')
        tx=UartTx(None,self.clock,self.clock.sleep,native_factory=Native)
        with self.assertRaises(OSError):tx.write(ml.command(3,123,4))
    def test_status_validation(self):
        for p in (status_packet(state=10),status_packet(state=12,version=6),status_packet(step=12),status_packet(fault=9),status_packet(version=9),status_packet()[:-1],status_packet()[:-1]+b'\x00'):
            with self.assertRaises(ValueError):decode_status(p)

class ParserTests(unittest.TestCase):
    def test_motion_status_is_decoded_only_once(self):
        d=RawDecoder(TelemetryDecoder(.2),False)
        with patch.object(ml,'decode_status',wraps=ml.decode_status) as decode:d.feed(status_packet())
        self.assertEqual(decode.call_count,1)
    def test_chunked_status_legacy_raw_and_noise(self):
        d=RawDecoder(TelemetryDecoder(.2),process_sensor=False);seen=[]
        class Sink:
            def receive(self,s):seen.append(s)
        raw=bytearray(b'\xa6\x6a\x00\x00')+struct.pack('<7I',0,0,0,0,0,5,20);raw.append(crc8(raw))
        legacy=bytearray(16);legacy[:2]=b'\xa5\x5a';legacy[15]=crc8(legacy[:15])
        stream=b'noise'+status_packet()+legacy+raw+status_packet(5,123,4,5,ticks=6)
        with patch.object(ml,'_sink',Sink()):
            for b in stream:d.feed(bytes([b]))
        self.assertEqual(len(seen),2);self.assertEqual(seen[-1]['action'],'TURN CW')
        self.assertEqual(d.stats['raw_frames'],1);self.assertEqual(d.stats['crc_errors'],0);self.assertEqual(d.legacy.stats['frames'],1)
    def test_corrupted_status_recovers(self):
        d=RawDecoder(TelemetryDecoder(.2),False);p=bytearray(status_packet());p[8]^=1
        d.feed(p+status_packet(ticks=2));self.assertGreater(d.stats['crc_errors'],0)

class MenuTests(unittest.TestCase):
    def test_active_view_has_only_stop_hint(self):
        c=Clock();d=BootDisplay(clock=c);d.update('DRIVING','LAP 1/2 LEG 1/4 FORWARD | 10 GRID')
        a=np.zeros((480,640,4),dtype=np.uint8);d.overlay(a);first=a.copy()
        c.now+=5;a[:]=0;d.overlay(a)
        self.assertEqual(d.content()['hint'],'BTN1 STOP');self.assertTrue(np.array_equal(first,a))
        self.assertNotIn('BTN2',str(d.content()))
    def test_countdown_and_fault_keep_controls(self):
        c=Clock();d=BootDisplay(clock=c)
        for phase,hint in (('COUNTDOWN','BTN1 STOP'),('MOTION FAULT','BTN0 RETRY'),('READY','BTN0 START')):
            d.update(phase,'Keep still');a=np.zeros((480,640,4),dtype=np.uint8);d.overlay(a)
            self.assertEqual(d.content()['hint'],hint);self.assertTrue(a[40:78,:,:3].any())
    def test_turns_do_not_reopen_large_menu(self):
        c=Clock();d=BootDisplay(clock=c);d.update('COUNTDOWN','3 seconds');c.now+=5
        for phase in ('DRIVING','GAP','TURNING','DRIVING'):
            d.update(phase,'LAP 2');a=np.zeros((480,640,4),dtype=np.uint8);d.overlay(a)
            self.assertEqual(d.content()['hint'],'BTN1 STOP');self.assertNotIn('BTN2',str(d.content()))

class SessionTests(unittest.TestCase):
    def test_tracking_gap_does_not_stop_later_display_segments(self):
        import demo_session,sensor_driver
        session=demo_session.DemoSession(PACKAGE);session.ready=True
        processor=SimpleNamespace(calibrated=True,pose=SimpleNamespace(continuous=False))
        result={'video_error':None,'camera_degraded':False}
        with patch.object(sensor_driver,'processor',processor), \
             patch.object(session,'_stop_tracking'),patch.object(session,'_start_tracking'), \
             patch.object(demo_session,'run_live',return_value=result) as capture:
            for unused in range(2):self.assertIs(session.capture(camera=False),result)
        self.assertEqual(capture.call_count,2);self.assertEqual(session.run_number,2)
        self.assertFalse(session.running);self.assertFalse(processor.pose.continuous)

class ApplicationTests(unittest.TestCase):
    def app(self):
        config=json.loads((PACKAGE/'autostart.json').read_text())
        a=Application(ROOT/'verification',config)
        class Motion:
            def __init__(self):self.toggles=0;self.stops=[]
            def toggle(self):self.toggles+=1
            def stop(self,reason):self.stops.append(reason)
        a.motion=Motion();return a
    def test_btn0_is_motion_not_video_pause(self):
        a=self.app();a.request('toggle_pause');self.assertEqual(a.motion.toggles,1);self.assertFalse(a.paused);self.assertTrue(a.actions.empty())
    def test_stop_does_not_restart_a_healthy_camera_capture(self):
        a=self.app();a.session=SimpleNamespace(cam=SimpleNamespace(closed=False))
        a.request('retry_camera')
        self.assertEqual(len(a.motion.stops),1);self.assertTrue(a.actions.empty())
        self.assertFalse(a.segment_stop.is_set());self.assertFalse(a.camera_retry)
    def test_other_local_operations_stop_first(self):
        for action in ('retry_camera','recalibrate','shutdown','control_error'):
            a=self.app();a.request(action);self.assertEqual(len(a.motion.stops),1);self.assertEqual(a.actions.get_nowait(),action)
    def test_signal_stops_motors(self):
        a=self.app();a.stop();self.assertTrue(a.stop_event.is_set());self.assertEqual(a.motion.stops,['Application stop'])

if __name__=='__main__':unittest.main()
