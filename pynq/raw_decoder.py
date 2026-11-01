"""RAW33 -> PL speed/path/yaw; auxiliary ARM pose composition in sensor_driver."""
import struct,time
from collections import deque
import sensor_driver
from sample_alignment import CounterTimeline

class RawDecoder:
    def __init__(self, legacy, process_sensor=True):
        self.legacy=legacy
        self.process_sensor=process_sensor
        self.grid_size_m=legacy.grid_size_m
        self.reference_frame=legacy.reference_frame
        self.buffer=bytearray()
        self.stats=dict(frames=0,crc_errors=0,skipped_bytes=0,missing_frames=0,duplicates=0,invalid_measurements=0,raw_frames=0)
        self.last_sequence=None;self.latest_pose=None
        self.diagnostics=legacy.diagnostics;self.calibration_samples=deque(maxlen=100)
        self.timeline=CounterTimeline()

    @property
    def diagnostic_drops(self): return self.legacy.diagnostic_drops

    def diagnostic(self,*args,**kw): self.legacy.diagnostic(*args,**kw)

    def feed(self,data):
        from vehicle_telemetry import crc8
        from telemetry_reference import camera_initial_sample,VEHICLE_INITIAL
        self.buffer.extend(data);out=[]
        while len(self.buffer)>=2:
            header=bytes(self.buffer[:2]);n=16 if header==b'\xa5\x5a' else 33 if header==b'\xa6\x6a' else 18 if header==b'\xa7\x7a' else 0
            if not n:
                del self.buffer[0];self.stats['skipped_bytes']+=1;continue
            if len(self.buffer)<n: break
            packet=bytes(self.buffer[:n])
            if n==18:
                from motion_link import publish_status
                try:publish_status(packet)
                except ValueError:
                    del self.buffer[0];self.stats['crc_errors']+=1
                    self.diagnostic('motion_crc_error',packet_hex=packet.hex())
                    continue
                del self.buffer[:18]
                continue
            if crc8(packet[:-1])!=packet[-1]:
                del self.buffer[0];self.stats['crc_errors']+=1
                self.diagnostic('crc_error',packet_hex=packet.hex(),frame_bytes=n)
                continue
            del self.buffer[:n]
            if n==16:
                poses=self.legacy.feed(packet)
                if poses and poses[-1]['valid']: self.latest_pose=poses[-1]
                continue
            self.stats['raw_frames']+=1
            if packet[3]: self.stats['invalid_measurements']+=1;continue
            seq=packet[2]
            if self.last_sequence is not None:
                delta=(seq-self.last_sequence)&255
                if not delta: self.stats['duplicates']+=1;continue
                self.stats['missing_frames']+=delta-1
                if delta>1:
                    self.diagnostic('raw_sequence_gap',previous=self.last_sequence,current=seq,
                        missing=delta-1,source_counters=list(struct.unpack_from('<7I',packet,4)))
            self.last_sequence=seq
            counters=struct.unpack_from('<7I',packet,4)
            received=time.monotonic()
            timing=self.timeline.observe(counters[5],counters[6],received)
            if timing['source_counter_reset'] or not timing['source_tick_consistent']:
                self.diagnostic('source_counter_anomaly',counters=list(counters),**timing)
            self.calibration_samples.append(counters)
            if not self.process_sensor: continue
            if sensor_driver.processor is None: raise RuntimeError('Configure PL processor before receiving')
            if not timing['source_tick_consistent'] and hasattr(sensor_driver.processor,'pose'):
                sensor_driver.processor.pose.mark_gap()
            result=sensor_driver.processor.process(counters)
            if result is None: continue
            self.stats['frames']+=1
            # One calibration epoch for X/Y/path/yaw. MCU pose never supplies X/Y.
            fresh=result['pose_valid']
            if not fresh:
                self.stats['invalid_measurements']+=1
            x=result['x_m']/0.20;y=result['y_m']/0.20
            result.update(sequence=seq,flags=0 if fresh else 0x40,valid=fresh,
              speed_grid_s=result['speed_m_s']/0.20,distance_grid=result['distance_m']/0.20,
              x_grid=x,y_grid=y,received_monotonic=time.monotonic(),
              source_t20=counters[5],source_t5=counters[6],source='PL_ENCODER_IMU',
              reference_frame=VEHICLE_INITIAL)
            if self.grid_size_m is not None:
                result.update(speed_km_h=result['speed_m_s']*3.6,
                              x_m=x*self.grid_size_m,y_m=y*self.grid_size_m)
            result.update(timing)
            frame=camera_initial_sample(result,self.reference_frame)
            from motion_link import publish_pose
            publish_pose(frame)
            out.append(frame)
        return out
