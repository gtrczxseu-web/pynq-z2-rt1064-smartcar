"""SP21 MMIO driver; all odometry magnitude and yaw integration run in PL."""
import time,math

def signed32(v):
    return v - (1 << 32) if v & 0x80000000 else v

class SensorProcessor:
    def __init__(self, mmio, bias=0):
        self.mmio=mmio
        if mmio.read(0x34)!=0x53503231:
            raise RuntimeError('SP21 ABI mismatch: upload the new bit/HWH pair')
        self.bias=bias
        self.calibrated=False
        self.clear()

    def wait(self):
        end=time.monotonic()+0.05
        while self.mmio.read(0x64)&1:
            if time.monotonic()>end: raise RuntimeError('PL processing timeout')

    def clear(self):
        self.wait();self.mmio.write(0x60,2);self.wait()
        from sensor_fusion import SessionPose
        self.pose=SessionPose()

    def process(self, counters):
        self.wait()
        before=self.mmio.read(0x68)
        rejected=self.mmio.read(0x6c)
        for i,v in enumerate(tuple(counters)+(self.bias & 0xffff,)):
            self.mmio.write(0x40+4*i,int(v)&0xffffffff)
        self.mmio.write(0x60,1);self.wait()
        count=self.mmio.read(0x68)
        if count==before:
            if self.mmio.read(0x6c)!=rejected:
                self.pose.mark_gap()
            return None
        result=dict(pl_samples=count,pl_rejected=self.mmio.read(0x6c),
          speed_m_s=self.mmio.read(0x70)/1e6,
          distance_m=(self.mmio.read(0x74)+(self.mmio.read(0x78)<<32))/1e6,
          heading_deg=signed32(self.mmio.read(0x7c))/(65536*100),
          dx_body_m=signed32(self.mmio.read(0x80))/1e6,
          dy_body_m=signed32(self.mmio.read(0x84))/1e6)
        result.update(self.pose.update(result))
        return result

processor=None

def configure(mmio):
    global processor
    processor=SensorProcessor(mmio)
    return processor

def calibrate(uart, seconds=3, stationary_limits=None):
    """Caller must keep vehicle still. Only bias estimation is performed by ARM."""
    from vehicle_telemetry import UartTelemetry
    if processor is None: raise RuntimeError('Run setup first')
    processor.calibrated=False
    if not math.isfinite(seconds) or seconds<2: raise ValueError('Use at least 2 finite seconds of stationary data')
    rx=UartTelemetry(uart,0.20,process_sensor=False)
    # Discard idle-time FIFO history before timing a fresh stationary window.
    uart.write(0x0c,3)
    uart.read(8)
    start=time.monotonic();samples=[]
    while time.monotonic()-start<seconds:
        rx.poll()
        samples.extend(rx.decoder.calibration_samples)
        rx.decoder.calibration_samples.clear()
        time.sleep(0.0001)
    if len(samples)<10:
        raise RuntimeError('Insufficient RAW33 frames: frames=%d, uart=%s, decoder=%s; inspect link and receive timing' % (len(samples),rx.stats,rx.decoder.stats))
    guard=None
    if stationary_limits is not None:
        if any(rx.stats.get(k,0) for k in ('overrun','frame_error','parity_error','receive_buffer_drops')) or any(rx.decoder.stats.get(k,0) for k in ('crc_errors','missing_frames','duplicates')):
            raise RuntimeError('WAIT STILL: UART/protocol errors during calibration')
        from stationary_guard import check_stationary
        guard=check_stationary(samples,**stationary_limits)
    pairs=[]
    for a,b in zip(samples,samples[1:]):
        dt=(b[6]-a[6])&0xffffffff
        if 0<dt<=200: pairs.append((signed32((b[4]-a[4])&0xffffffff),dt))
    if len(pairs)<8: raise RuntimeError('Insufficient valid calibration intervals')
    encoder_step_max=max(abs(signed32((b[i]-a[i])&0xffffffff))
                         for a,b in zip(samples,samples[1:]) for i in range(4))
    if encoder_step_max>50:
        raise RuntimeError('Encoder movement during calibration; keep all wheels still and repeat')
    means=[g/n for g,n in pairs]
    if max(means)-min(means)>50: raise RuntimeError('Vehicle moved during calibration; keep still and repeat')
    bias=round(sum(g for g,n in pairs)/sum(n for g,n in pairs))
    if abs(bias)>2000: raise RuntimeError('Abnormal gyro bias; inspect IMU and movement')
    processor.bias=int(bias);processor.clear()
    # Anchor the shared origin to the last stationary sample, not a later run.
    assert processor.process(samples[-1]) is None
    processor.calibrated=True
    return dict(bias_lsb=bias,frames=len(samples),spread_lsb=max(means)-min(means),
                encoder_step_max=encoder_step_max,pose_epoch=processor.pose.epoch,
                source='stationary RAW33; PL subtracts bias and integrates yaw',stationary_guard=guard)


def register_self_test():
    """Explicit synthetic register inputs; this is not a real sensor test."""
    if processor is None: raise RuntimeError('Run setup first')
    processor.calibrated=False
    processor.bias=0
    processor.clear()
    try:
        assert processor.process((0,0,0,0,0,0,0)) is None
        a=processor.process((2430,2430,2430,2430,0,5,20))
        assert a is not None and a['speed_m_s']==2.0 and a['distance_m']==0.2,a
        b=processor.process((2430,2430,2430,2430,28600,10,40))
        assert b is not None and b['speed_m_s']==0 and abs(b['heading_deg']-9.997955322265625)<1e-9,b
        return dict(test='PL_REGISTER_SELF_TEST',synthetic_input=True,real_sensor_test=False,
                    passed=True,encoder=a,imu=b)
    finally:
        processor.clear()
