"""Debounced local buttons; no hardware access on import."""
import queue,threading,time

class ButtonEvents:
    def __init__(self,debounce=.05):
        self.debounce=debounce;self.candidate=self.stable=0;self.changed_at=0
        self.held={};self.emitted=set()
    def feed(self,mask,now):
        out=[];mask&=15
        if mask!=self.candidate:self.candidate=mask;self.changed_at=now
        if mask!=self.stable and now-self.changed_at>=self.debounce:
            rising=mask&~self.stable;falling=self.stable&~mask;self.stable=mask
            for bit in range(4):
                if rising&(1<<bit):self.held[bit]=now;self.emitted.discard(bit)
                if falling&(1<<bit):
                    if bit in (2,3) and bit not in self.emitted:
                        out.append('settings' if bit==2 else 'settings_cancel')
                    self.held.pop(bit,None);self.emitted.discard(bit)
            if rising&1:out.append('toggle_pause')
            if rising&2:out.append('retry_camera')
        for bit,seconds,action in ((2,2,'recalibrate'),(3,3,'shutdown')):
            if bit in self.held and bit not in self.emitted and now-self.held[bit]>=seconds:
                self.emitted.add(bit);out.append(action)
        return out

class LocalControls:
    def __init__(self,overlay,on_action,phase):
        from pynq import MMIO
        self.buttons=MMIO(overlay.ip_dict['btns_gpio']['phys_addr'],0x10000)
        self.leds=MMIO(overlay.ip_dict['leds_gpio']['phys_addr'],0x10000)
        self.on_action=on_action;self.phase=phase;self.stop_event=threading.Event();self.error=None
        self.thread=threading.Thread(target=self._run,name='local-buttons',daemon=True)
    def start(self):self.thread.start()
    def _run(self):
        decoder=ButtonEvents()
        try:
            while not self.stop_event.wait(.025):
                now=time.monotonic()
                for action in decoder.feed(self.buttons.read(0),now):self.on_action(action)
                phase=self.phase()
                value=4 if phase in ('READY','DRIVING','TURNING','GAP','COUNTDOWN','FINISHED') or phase.startswith('RUNNING') else 8 if phase in ('PAUSED','STEP PAUSED') else 10 if 'FAULT' in phase else (2 if int(now*2)%2 else 0)
                self.leds.write(0,value)
        except Exception as exc:self.error=str(exc);self.on_action('control_error')
    def stop(self):
        self.stop_event.set();self.thread.join(2)
        if self.thread.is_alive():raise RuntimeError('Local GPIO thread did not stop')
        self.leds.write(0,0)
