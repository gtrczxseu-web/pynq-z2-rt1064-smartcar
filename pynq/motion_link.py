"""V7 high-level UART commands. RT1064 owns the 5/20 ms motor loops.

No read of the RX FIFO occurs here: the existing single telemetry receiver
publishes CRC-checked status frames. Only this module writes the TX FIFO.
"""
import queue,secrets,struct,threading,time
from route_settings import default_settings,validate,configuration,requires_v3,step_status
from telemetry_reference import VEHICLE_INITIAL,CAMERA_INITIAL
from vision_signals import SignalCards
from visual_obstacle import forward_step

STATES=('WAIT','ARMED','COUNTDOWN','MOVE','GAP','TURN','DONE','STOPPED','FAULT','PAUSED','HOLDING','HELD','BYPASS_RIGHT','BYPASS_FORWARD','BYPASS_LEFT')
FAULTS=('OK','LINK LOST','TIMEOUT','STALL','POSE ERROR','HEADING ERROR','NOT STILL','BAD SENSOR','BYPASS TOO SHORT','BYPASS NOT MOVE')
BYPASS_STATES=STATES[12:]
STEPS=((1,1,'FORWARD'),(1,2,'RIGHT'),(1,3,'BACK'),(1,4,'LEFT'),
       (2,1,'FORWARD'),(2,2,'TURN CW'),(2,2,'FORWARD'),(2,3,'TURN CW'),
       (2,3,'FORWARD'),(2,4,'TURN CW'),(2,4,'FORWARD'),(2,4,'RESTORE HEADING'))
_sink=None

def crc16(data):
    c=0xffff
    for b in data:
        c^=b<<8
        for unused in range(8):c=((c<<1)^0x1021 if c&0x8000 else c<<1)&0xffff
    return c

def command(op,session,sequence,version=2):
    p=struct.pack('<2sBBIH',b'\xc7\x7c',version,op,session,sequence)
    return p+struct.pack('<H',crc16(p))

def decode_status(packet,now=None):
    if len(packet)!=18 or packet[:2]!=b'\xa7\x7a' or packet[2] not in (1,2,3,4,5,6,7,8) or crc16(packet[:-2])!=int.from_bytes(packet[-2:],'little'):
        raise ValueError('Invalid V7 route status CRC/version')
    state=packet[3];session,seq,step,fault,ticks=struct.unpack_from('<IHBBI',packet,4)
    config_ack=fault if packet[2]>=3 and state==1 else 0
    if config_ack:fault=0
    bypass_phase=None
    if packet[2]>=8 and state in (10,11) and fault in (0x10,0x11,0x12):bypass_phase=BYPASS_STATES[fault-0x10];fault=0
    if state>=len(STATES) or (state>=10 and packet[2]<6) or (state>=12 and packet[2]<8) or step>=12 or fault>=len(FAULTS) or (fault>=8 and packet[2]<8):raise ValueError('Invalid V7 state')
    lap,leg,action=STEPS[step]
    return dict(protocol_version=packet[2],state=STATES[state],session=session,sequence=seq,step=step,fault=FAULTS[fault],config_ack=config_ack,
                ticks=ticks,lap=lap,leg=leg,action=action,bypass_phase=bypass_phase,received_monotonic=time.monotonic() if now is None else now)

def publish_status(packet):
    status=decode_status(packet)
    sink=_sink
    if sink is not None:sink.receive(status)

def publish_pose(sample):
    """Publish at RAW33 decode cadence, independently of the 0.5 s UI callback."""
    sink=_sink
    if sink is not None:sink.receive_pose(sample)

class UartTx:
    def __init__(self,mmio,clock=time.monotonic,sleep=time.sleep,native_factory=None):
        self.mmio=mmio;self.clock=clock;self.sleep=sleep;self.last_write=-1e9
        if native_factory is None:
            from uart_tx_native import NativeUartTx
            native_factory=NativeUartTx
        self.native=native_factory(mmio)
        self.stats=dict(bursts=0,maximum_call_ms=0.0,calls_over_30ms=0)
    def write(self,packet):
        if len(packet)!=12:raise ValueError('One 12-byte command per TX burst')
        # UARTLite has a 16-byte TX FIFO. At the fixed 115200 8N1, 12 bytes
        # drain in 1.05 ms; serialize bursts with at least 3 ms quiet time.
        # Do NOT read STATUS here: RX error bits are clear-on-read and belong
        # to the existing sole native RX reader. Commands use status ACKs.
        wait=.003-(self.clock()-self.last_write)
        if wait>0:self.sleep(wait)
        began=self.clock()
        try:
            # CDLL releases the GIL. The volatile C loop completes all 12
            # stores without Python scheduling between bytes. Do not abort a
            # partially sent frame because a non-realtime thread was delayed.
            self.native.write(packet)
            self.stats['bursts']+=1
        finally:
            self.last_write=self.clock();elapsed=(self.last_write-began)*1000
            self.stats['maximum_call_ms']=max(self.stats['maximum_call_ms'],elapsed)
            self.stats['calls_over_30ms']+=int(elapsed>30)

class MotionLink:
    def __init__(self,uart=None,writer=None,clock=time.monotonic,session_factory=lambda:secrets.randbits(32) or 1,settings_loader=None):
        self.clock=clock;self.writer=writer or UartTx(uart,clock);self.session_factory=session_factory
        self.lock=threading.RLock();self.thread=None;self.closed=threading.Event()
        self.mcu=None;self.latest=None;self.pose_sink=None
        self.state='WAITING';self.fault=None;self.session=0;self.sequence=0
        self.last_send=-1e9;self.log=[];self.log_drops=0;self.log_sink=None;self.log_error=None
        self.settings_loader=settings_loader;self.recipe=default_settings();self.config_items=[];self.config_index=0;self.paused_at=0
        self.resume_step=None;self.resume_packet=None
        self.reference_frame=VEHICLE_INITIAL
        self.vision_frames=queue.Queue(maxsize=1);self.cards=SignalCards();self.run_started=0
        self.digit_frames=queue.Queue(maxsize=1);self.digit_cards=SignalCards()
        self.next_digit=None;self.traffic_reason=None;self.yellow_at=None;self.traffic_packet=None
        self.traffic_op=None;self.traffic_sequence=None
        self.countdown_held=False
        self.obstacle_frames=queue.Queue(maxsize=1);self._reset_obstacle()
        self._reset_bypass()
    def _reset_bypass(self):
        self.bypass_packet=None;self.bypass_sequence=None;self.bypass_active=False
        self.bypass_seen=False;self.bypass_phase=None;self.bypass_tried=None;self.bypass_note=None
    def _reset_obstacle(self):
        self.obstacle_hold=False;self.obstacle_stamp=-1e9;self.obstacle_state='UNKNOWN'
        self.obstacle_count=0;self.obstacle_clear_count=0;self.obstacle_clear_at=None
        self.obstacle_epoch=None;self.obstacle_context=None
        self.obstacle_confirmed_context=None
    def receive_obstacle(self,stamp,result,sample,session,step):
        item=(stamp,dict(result),None if sample is None else dict(sample),session,step)
        try:self.obstacle_frames.get_nowait()
        except queue.Empty:pass
        try:self.obstacle_frames.put_nowait(item)
        except queue.Full:pass
    def _obstacle_ready(self,stamp=None):
        m=self.mcu;now=self.clock()
        return bool(self.obstacle_hold and self.obstacle_clear_count>=3 and self.obstacle_state=='CLEAR'
            and 0<=now-self.obstacle_stamp<=.5 and m is not None and m['session']==self.session and m['state']=='HELD'
            and 0<=now-m['received_monotonic']<=.5
            and self.obstacle_context==(self.session,m['step'],self.obstacle_epoch)
            and (self.obstacle_epoch is None or self.latest is not None and self.obstacle_epoch==tuple(self.latest.get(k) for k in ('pose_epoch','source_clock_epoch')))
            and (stamp is None or stamp>=self.obstacle_clear_at))
    def _resume_obstacle(self,automatic=False,stamp=None):
        if not self._obstacle_ready(stamp) or self.traffic_reason is not None:return False
        if (self.mcu['protocol_version']>=8 and not self.bypass_active
                and self.obstacle_confirmed_context==self.obstacle_context
                and self.bypass_tried!=self.obstacle_context):return False
        self.obstacle_hold=False;self._traffic_command(6)
        self._event('obstacle_resume_requested',automatic=automatic);return True
    def _obstacle_step(self,now):
        try:stamp,result,sample,session,index=self.obstacle_frames.get_nowait()
        except queue.Empty:return
        m=self.mcu
        if (not self.active or m is None or m['protocol_version']<6 or m['session']!=self.session
                or session!=self.session or index!=m['step'] or not forward_step(self.recipe,index)
                or self.bypass_packet is not None
                or m['protocol_version']>=8 and (m['state']=='COUNTDOWN' or self.countdown_held)
                or self.bypass_active and self.bypass_phase in ('BYPASS_RIGHT','BYPASS_LEFT')
                or m['state'] not in ('COUNTDOWN','MOVE','HOLDING','HELD','BYPASS_FORWARD')
                or stamp<self.run_started or not 0<=now-stamp<=.5 or stamp<=self.obstacle_stamp):return
        if sample is not None and self.latest is not None:
            if any(sample.get(k)!=self.latest.get(k) for k in ('pose_epoch','source_clock_epoch')):return
        epoch=None if sample is None else tuple(sample.get(k) for k in ('pose_epoch','source_clock_epoch'))
        context=(session,index,epoch)
        if context!=self.obstacle_context:self.obstacle_confirmed_context=None
        if context!=self.obstacle_context or stamp-self.obstacle_stamp>.75:
            self.obstacle_count=self.obstacle_clear_count=0;self.obstacle_clear_at=None
        self.obstacle_stamp=stamp;self.obstacle_state=result['state']
        self.obstacle_epoch=epoch;self.obstacle_context=context
        if result['state']=='BLOCKED':
            self.obstacle_count+=1;self.obstacle_clear_count=0;self.obstacle_clear_at=None
            if self.obstacle_count>=2:self.obstacle_confirmed_context=context
            if self.obstacle_count>=2 and not self.obstacle_hold:
                self.obstacle_hold=True;self.cards=SignalCards()
                self._event('signal_card',label='OBSTACLE',captured_monotonic=stamp)
        elif result['state']=='CLEAR':
            self.obstacle_count=0;self.obstacle_clear_count+=1
            if self.obstacle_clear_count==3:self.obstacle_clear_at=stamp
        else:self.obstacle_count=self.obstacle_clear_count=0;self.obstacle_clear_at=None
    def _bypass_step(self,now):
        m=self.mcu
        if m is None or m['protocol_version']<8 or m['session']!=self.session:return False
        if self.bypass_seen and m['state']=='PAUSED':
            self._event('bypass_completed');self._reset_bypass();self._reset_obstacle()
        if not self.active:return False
        phase=m['state'] if m['state'] in BYPASS_STATES else m.get('bypass_phase')
        if phase is not None:
            if not self.bypass_seen:
                self._reset_obstacle();self._event('bypass_started',side_m=.5,forward_m=1.0)
            self.bypass_active=self.bypass_seen=True;self.bypass_phase=phase;self.bypass_packet=None
        elif self.bypass_seen and m['state']=='MOVE':
            self._event('bypass_completed');self._reset_bypass();self._reset_obstacle()
        if self.bypass_packet is not None:
            if self.traffic_reason is not None:
                self.bypass_packet=None;self._traffic_command(5);return False
            if m['sequence']==self.bypass_sequence and m['state']=='HELD' and m['fault'] in ('BYPASS TOO SHORT','BYPASS NOT MOVE'):
                self.bypass_packet=None;self.bypass_active=False;self.obstacle_hold=True
                self.bypass_note='BYPASS NEEDS 1m REMAINING' if m['fault']=='BYPASS TOO SHORT' else 'NO BYPASS AT THIS STAGE';self.state='SIGNAL_WAIT'
                self._event('bypass_unavailable',reason=self.bypass_note);return False
            if now-self.last_send>=.1:
                self.writer.write(self.bypass_packet);self.last_send=now
            self.state='BYPASS_STARTING';return True
        if (self.bypass_active and not self.bypass_seen and m['state']=='HELD'
                and (self.traffic_packet is None or self.traffic_op==5 and m['sequence']==self.traffic_sequence)):
            # A traffic signal preempted a request before the MCU acknowledged it.
            self.bypass_active=False;self.bypass_tried=None;self.obstacle_hold=True
        if (self.obstacle_hold and not self.bypass_active and self.traffic_reason is None
                and self.bypass_tried!=self.obstacle_context and self.active
                and m['state']=='HELD' and not m.get('bypass_phase')
                and not self.countdown_held
                and self.obstacle_confirmed_context is not None
                and self.obstacle_confirmed_context==self.obstacle_context==(self.session,m['step'],self.obstacle_epoch)
                and (self.obstacle_epoch is None or self.latest is not None and self.obstacle_epoch==tuple(self.latest.get(k) for k in ('pose_epoch','source_clock_epoch')))
                and 0<=now-m['received_monotonic']<=.5):
            self.bypass_tried=self.obstacle_context;self.bypass_active=True;self.bypass_note=None
            self.sequence=(self.sequence+1)&65535;self.bypass_sequence=self.sequence
            self.bypass_packet=command(7,self.session,self.sequence,8);self.last_send=-1e9
            self.obstacle_hold=False;self.traffic_packet=None;self.state='BYPASS_STARTING'
            self._event('bypass_requested',trigger='confirmed_obstacle');return self._bypass_step(now)
        return False
    def _event(self,kind,**values):
        record=dict(monotonic=self.clock(),kind=kind,**values)
        if len(self.log)>=256:self.log.pop(0);self.log_drops+=1
        self.log.append(record)
        if self.log_sink is not None:
            try:self.log_sink(record)
            except Exception as exc:self.log_error=str(exc)
    def receive(self,status):
        with self.lock:
            status=dict(status)
            if status['session']==self.session and status['step']<len(self.recipe['steps']):
                status.update(step_status(self.recipe,status['step']))
            if status['state'] in BYPASS_STATES:status['action']=status['state'].replace('_',' ')
            if self.mcu is not None:
                delta=(status['ticks']-self.mcu['ticks'])&0xffffffff
                if not delta:return  # ignore duplicate status
            if self.mcu is None or (self.mcu['state'],self.mcu['step'],self.mcu['fault'])!=(status['state'],status['step'],status['fault']):
                self._event('mcu_state',**status)
            self.mcu=dict(status)
    @property
    def active(self):return self.state in ('ARMING','CONFIGURING','STARTING','RESUMING','COUNTDOWN','DRIVING','TURNING','GAP','SIGNAL_HOLDING','SIGNAL_WAIT','SIGNAL_RELEASING','BYPASS_STARTING','BYPASS')
    def receive_vision(self,stamp,detections,sample=None,session=0):
        # Called by inference; UART writes remain on the existing motion thread.
        item=(stamp,[dict(d) for d in detections],None if sample is None else dict(sample),session)
        try:self.vision_frames.get_nowait()
        except queue.Empty:pass
        try:self.vision_frames.put_nowait(item)
        except queue.Full:pass
    def receive_digits(self,stamp,detections,sample=None,session=0):
        item=(stamp,[dict(d) for d in detections],None if sample is None else dict(sample),session)
        try:self.digit_frames.get_nowait()
        except queue.Empty:pass
        try:self.digit_frames.put_nowait(item)
        except queue.Full:pass
    def _digit_step(self,now):
        try:stamp,found,sample,session=self.digit_frames.get_nowait()
        except queue.Empty:return
        m=self.mcu
        if (self.recipe['mode']!='step' or self.state!='PAUSED' or m is None or m['state']!='PAUSED'
                or m['protocol_version']<6 or session!=self.session or m['session']!=self.session
                or stamp<max(self.run_started,self.paused_at) or not 0<=now-stamp<=.5):return
        if sample is not None and self.latest is not None:
            if any(sample.get(k)!=self.latest.get(k) for k in ('pose_epoch','source_clock_epoch')):return
        events,unused=self.digit_cards.update(stamp,[d for d in found if d.get('label','').startswith('DIGIT ')])
        for label in sorted(events):
            self.next_digit=int(label.split()[1]);self._event('next_speed',digit=self.next_digit,speed_percent=60+10*self.next_digit)
    def _vision_step(self,now):
        try:stamp,found,sample,session=self.vision_frames.get_nowait()
        except queue.Empty:return
        m=self.mcu
        if (m is None or m['protocol_version']<6 or session!=self.session or m['session']!=self.session
                or stamp<self.run_started or not 0<=now-stamp<=.5
                or self.state not in ('RESUMING','COUNTDOWN','DRIVING','TURNING','GAP','PAUSED','SIGNAL_HOLDING','SIGNAL_WAIT','SIGNAL_RELEASING','BYPASS_STARTING','BYPASS')):return
        if sample is not None and self.latest is not None:
            if any(sample.get(k)!=self.latest.get(k) for k in ('pose_epoch','source_clock_epoch')):return
        obstacle_wait=self.obstacle_hold and m['state']=='HELD'
        if self.recipe['mode']=='step' and not obstacle_wait and (self.state!='PAUSED' or m['state']!='PAUSED' or stamp<self.paused_at):return
        if self.obstacle_hold and not self._obstacle_ready(stamp) and m['protocol_version']<8:found=[d for d in found if d.get('label')!='GREEN']
        if self.recipe['mode']=='continuous':
            found=[d for d in found if not d.get('label','').startswith('DIGIT ')]
            colors=('RED','YELLOW','GREEN');labels={d.get('label') for d in found}
            chosen=next((c for c in colors if c in labels),None)
            found=[d for d in found if d.get('label') not in colors or d.get('label')==chosen]
            # Defer the GREEN edge until the mandatory yellow wait has elapsed.
            if self.traffic_reason=='YELLOW':found=[d for d in found if d.get('label')!='GREEN']
        events,color=self.cards.update(stamp,found)
        for label in sorted(events):
            self._event('signal_card',label=label,captured_monotonic=stamp)
            if label.startswith('DIGIT '):
                self.next_digit=int(label.split()[1]);self._event('next_speed',digit=self.next_digit,speed_percent=60+10*self.next_digit)
        if self.recipe['mode']=='step':
            if 'GREEN' in events and obstacle_wait:self._resume_obstacle(automatic=True,stamp=stamp)
            elif 'GREEN' in events and self.state=='PAUSED' and m['state']=='PAUSED' and stamp>=self.paused_at:
                self._resume(automatic=True)
            return
        if color=='RED' and 'RED' in events:
            self.traffic_reason='RED';self.yellow_at=None
        elif color=='YELLOW' and 'YELLOW' in events and self.traffic_reason is None:
            self.traffic_reason='YELLOW';self.yellow_at=None
        elif color=='GREEN' and 'GREEN' in events and self.traffic_reason=='RED':
            self.traffic_reason=None;self._event('signal_green_release')
        if color=='GREEN' and 'GREEN' in events and self.obstacle_hold:self._resume_obstacle(automatic=True,stamp=stamp)
    def _traffic_command(self,op):
        if op==5 and self.mcu['state']=='COUNTDOWN':self.countdown_held=True
        self.sequence=(self.sequence+1)&65535;self.traffic_op=op;self.traffic_sequence=self.sequence
        self.traffic_packet=command(op,self.session,self.sequence,self.mcu['protocol_version']);self.last_send=-1e9
        self.state='SIGNAL_HOLDING' if op==5 else 'SIGNAL_RELEASING'
        self._event('signal_hold' if op==5 else 'signal_release',reason='OBSTACLE' if self.obstacle_hold else self.traffic_reason)
    def _traffic_step(self,now):
        m=self.mcu
        if m is None or m['protocol_version']<6 or m['session']!=self.session:return False
        if not (self.active or self.state=='PAUSED'):return False
        if m['state'] not in ('COUNTDOWN','MOVE','TURN','GAP','PAUSED','HOLDING','HELD')+BYPASS_STATES:return False
        if self.traffic_packet is not None:
            if (self.traffic_reason is not None or self.obstacle_hold) and self.traffic_op==6:self._traffic_command(5)
            if m['sequence']==self.traffic_sequence:self.traffic_packet=None
        if self.traffic_packet is None:
            if self.traffic_reason=='YELLOW' and m['state']=='HELD':
                if self.yellow_at is None:self.yellow_at=now
                if now-self.yellow_at>=5:
                    self.traffic_reason=None;self._event('signal_yellow_elapsed')
            if (self.traffic_reason is not None or self.obstacle_hold) and m['state'] not in ('HOLDING','HELD'):
                self._traffic_command(5)
            elif self.traffic_reason is None and not self.obstacle_hold and m['state']=='HELD':self._traffic_command(6)
        if self.traffic_packet is not None:
            if now-self.last_send>=.1:
                self.writer.write(self.traffic_packet);self.last_send=now
            return True
        if m['state'] in ('HOLDING','HELD'):
            self.state='SIGNAL_HOLDING' if m['state']=='HOLDING' else 'SIGNAL_WAIT'
            if now-self.last_send>=.1:self._send(3)
            return True
        return False
    def receive_pose(self,latest):
        with self.lock:
            if latest is not None and (self.latest is None or
                    latest.get('received_monotonic',-1e9)>=self.latest.get('received_monotonic',-1e9)):
                self.latest=dict(latest)
                if self.pose_sink is not None:self.pose_sink(self.latest)
    def _ready(self):
        return bool(self.mcu is not None and self.mcu.get('protocol_version') in (2,3,4,5,6,7,8))
    def _send(self,op):
        self.sequence=(self.sequence+1)&65535
        version=self.mcu.get('protocol_version',2) if self.mcu is not None else 2
        self.writer.write(command(op,self.session,self.sequence,version));self.last_send=self.clock()
        if op!=3:self._event('command',op=op,session=self.session,sequence=self.sequence)
    def _stop(self,reason,fault=False):
        self._reset_obstacle()
        self._reset_bypass()
        self.next_digit=None;self.traffic_reason=None;self.yellow_at=None;self.traffic_packet=None
        self.state='FAULT' if fault else 'STOPPED';self.fault=str(reason) if fault else None
        self._event('stop',reason=str(reason),fault=fault)
        try:self._send(0)
        except Exception as exc:
            self.state='FAULT';self.fault='STOP TX failed: '+str(exc)
    def stop(self,reason='Operator stop'):
        with self.lock:self._stop(reason)
    def _reject(self,reason):
        self.fault=reason;self._event('start_rejected',reason=reason)
        return False
    def _resume(self,automatic=False):
        self.state='RESUMING';self.last_send=-1e9;self.resume_step=self.mcu['step']
        self.sequence=(self.sequence+1)&65535
        op=4;digit=None;next_step=self.resume_step+1
        if self.next_digit is not None and self.mcu['protocol_version']>=6 and next_step<len(self.recipe['steps']) and self.recipe['steps'][next_step]['kind']=='move':
            digit=self.next_digit;op=0x60+digit;self.next_digit=None
        self.resume_packet=command(op,self.session,self.sequence,self.mcu['protocol_version'])
        self._event('continue_requested',session=self.session,completed_step=self.resume_step,automatic=automatic,
            speed_percent=self.recipe['speed_percent'] if digit is None else 60+10*digit)
    def toggle(self):
        with self.lock:
            if self.obstacle_hold and self.active:return self._resume_obstacle()
            if self.active:self._event('busy_press_ignored',reason='Current action will pause when complete');return False
            if not self._ready():return self._reject('Waiting for supported MCU status')
            if self.state=='PAUSED' and self.mcu['state']=='PAUSED' and self.mcu['session']==self.session:
                self._resume();return True
            if self.mcu['state'] not in ('WAIT','DONE','STOPPED','FAULT'):
                self._stop('Unexpected active MCU state',True);return False
            try:self.recipe=validate(self.settings_loader()) if self.settings_loader is not None else self.recipe
            except Exception as exc:return self._reject('Route settings: '+str(exc))
            if (requires_v3(self.recipe) or self.reference_frame==CAMERA_INITIAL) and self.mcu.get('protocol_version') not in (3,4,5,6,7,8):
                return self._reject('Flash configurable car firmware before this recipe')
            if any(s.get('rotation_deg') for s in self.recipe['steps']) and self.mcu.get('protocol_version') not in (4,5,6,7,8):
                return self._reject('Flash V4 full-turn car firmware before fixed multi-turn actions')
            if self.recipe.get('preset')=='rotate' and self.mcu.get('protocol_version',0)<7:
                return self._reject('Flash V7 car firmware for ROTATE speed control')
            if self.recipe['speed_percent']>100 and self.mcu.get('protocol_version') not in (5,6,7,8):
                return self._reject('Flash V5 extended-speed car firmware before speeds above 100%')
            candidate=int(self.session_factory())&0xffffffff
            if not candidate or candidate==self.session or candidate==self.mcu['session']:
                return self._reject('Session must be fresh and nonzero')
            self.session=candidate;self.sequence=0;self.state='ARMING';self.fault=None
            self.cards=SignalCards();self.run_started=self.clock();self.next_digit=None
            self._reset_obstacle()
            self._reset_bypass()
            self.countdown_held=False
            self.digit_cards=SignalCards()
            self.traffic_reason=None;self.yellow_at=None;self.traffic_packet=None
            self.config_items=configuration(self.recipe,self.reference_frame);self.config_index=0
            self.last_send=-1e9;self._event('start_requested',session=self.session,recipe=self.recipe)
            return True
    def step(self):
        with self.lock:
            now=self.clock()
            m=self.mcu
            try:
                if m is not None and m['protocol_version']>=8 and m['state']=='MOVE':self.countdown_held=False
                if m is not None and m['session']==self.session and m['state'] in ('DONE','STOPPED','FAULT'):
                    if m['state']=='DONE' and self.bypass_seen:self._event('bypass_completed')
                    self._reset_obstacle();self._reset_bypass()
                # A completed MCU action can arrive just before a vision result.
                # Publish that boundary before deciding whether GREEN is eligible.
                if (m is not None and m['session']==self.session and m['state']=='PAUSED'
                        and self.state in ('COUNTDOWN','DRIVING','TURNING','GAP','BYPASS','BYPASS_STARTING')):
                    self.state='PAUSED';self.paused_at=m['received_monotonic']
                    self._event('step_paused',completed_step=m['step'])
                self._obstacle_step(now)
                self._digit_step(now)
                self._vision_step(now)
                if self._bypass_step(now):return
                if self._traffic_step(now):return
                # A short display/card window follows the MCU's completed-action
                # acknowledgement; light events above take priority over resume.
                if self.state=='PAUSED' and self.recipe['mode']=='continuous' and now-self.paused_at>=.35:
                    self._resume(automatic=True)
                if not self.active:return
                if self.state=='ARMING':
                    if m['session']==self.session and m['state']=='ARMED':self.state='CONFIGURING' if m.get('protocol_version',0)>=3 else 'STARTING'
                    elif now-self.last_send>=.1:self._send(1)
                    if self.state=='ARMING':return
                if m['session']!=self.session:
                    self._stop('MCU session mismatch',True);return
                if self.state=='CONFIGURING':
                    op,value=self.config_items[self.config_index]
                    if m.get('config_ack')==op and m['sequence']==value:
                        self.sequence=value;self.config_index+=1;self.last_send=-1e9
                        if self.config_index==len(self.config_items):self.state='STARTING'
                    if self.state=='CONFIGURING':
                        op,value=self.config_items[self.config_index]
                        if now-self.last_send>=.1:
                            self.writer.write(command(op,self.session,value,m['protocol_version']));self.last_send=now
                        return
                if self.state=='RESUMING' and m['state']=='PAUSED' and m['step']==self.resume_step:
                    if now-self.last_send>=.1:
                        self.writer.write(self.resume_packet);self.last_send=now
                        self._event('command',op=self.resume_packet[3],session=self.session,sequence=self.sequence)
                    return
                if self.state=='STARTING' and m['state']=='ARMED':
                    if now-self.last_send>=.1:self._send(2)
                    return
                if m['state']=='DONE':self.state='FINISHED';self._event('completed');return
                if m['state']=='PAUSED':
                    self.state='PAUSED';self.paused_at=now;self._event('step_paused',completed_step=m['step']);return
                if m['state'] in ('FAULT','WAIT','STOPPED'):
                    self._stop('MCU '+m['state']+': '+m['fault'],True);return
                self.state='BYPASS' if m['state'] in BYPASS_STATES else dict(COUNTDOWN='COUNTDOWN',MOVE='DRIVING',GAP='GAP',TURN='TURNING').get(m['state'],self.state)
                if now-self.last_send>=.1:self._send(3)
            except Exception as exc:self._stop('UART command failure: '+str(exc),True)
    def snapshot(self):
        with self.lock:
            ready=self._ready()
            return dict(state=self.state,ready=ready,active=self.active,fault=self.fault,session=self.session,
                        mcu=None if self.mcu is None else dict(self.mcu),log_drops=self.log_drops,log_error=self.log_error,
                        tx=None if not hasattr(self.writer,'stats') else dict(self.writer.stats),
                        route=self.recipe.get('preset'),mode=self.recipe['mode'],reference_frame=self.reference_frame,total_steps=len(self.recipe['steps']),
                        vision_control=ready and self.mcu['protocol_version']>=6,traffic_reason=self.traffic_reason,
                        yellow_remaining_s=None if self.yellow_at is None or self.traffic_reason!='YELLOW' else round(max(0,5-(self.clock()-self.yellow_at)),1),
                        next_speed_percent=None if self.next_digit is None else 60+10*self.next_digit,
                        obstacle_hold=self.obstacle_hold,obstacle_clear=self._obstacle_ready(),
                        bypass_supported=ready and self.mcu['protocol_version']>=8,bypass_active=self.bypass_active,
                        bypass_pending=self.bypass_packet is not None,bypass_phase=self.bypass_phase,bypass_note=self.bypass_note,
                        countdown_held=self.countdown_held,
                        compatible=ready and (self.recipe.get('preset')!='rotate' or self.mcu['protocol_version']>=7),hardware_qualified=False)
    def start(self):
        global _sink
        if _sink is not None:raise RuntimeError('Another motion TX owner exists')
        _sink=self
        # A previous MCU run must be stopped before accepting any new input.
        self.stop('PYNQ application startup')
        def work():
            try:
                while not self.closed.wait(.02):self.step()
            except BaseException as exc:
                with self.lock:self._stop('Motion thread failed: '+str(exc),True)
        self.thread=threading.Thread(target=work,name='route-command-lease',daemon=True);self.thread.start()
    def close(self):
        global _sink
        if self.closed.is_set():return
        self.closed.set()
        if self.thread is not None:self.thread.join(.5)
        self.stop('Application exit')
        if _sink is self:_sink=None
