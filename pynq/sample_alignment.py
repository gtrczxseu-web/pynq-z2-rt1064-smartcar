"""Causal software association; no claim of camera exposure synchronization."""
from collections import deque
import math


class CounterTimeline:
    """Unwrap nominal MCU PIT counters without equating them to the PS clock."""
    def __init__(self):
        self.previous=None
        self.elapsed=0.0
        self.epoch=0

    def observe(self,t20,t5,received):
        dt20=dt5=arrival_delta=None
        reset=False
        consistent=True
        if self.previous is not None:
            a,b,stamp=self.previous
            dt20=(t20-a)&0xffffffff
            dt5=(t5-b)&0xffffffff
            reset=dt20>0x7fffffff or dt5>0x7fffffff
            arrival_delta=received-stamp
            if reset:
                self.epoch+=1
                self.elapsed=0.0
                consistent=False
            else:
                self.elapsed+=dt5*.005
                consistent=abs(dt20*.020-dt5*.005)<=.025000001
        self.previous=(t20,t5,received)
        return dict(source_elapsed_s=self.elapsed,source_clock_epoch=self.epoch,
                    source_counter_reset=reset,source_tick_consistent=consistent,
                    source_interval_s=None if dt5 is None or reset else dt5*.005,
                    decode_interval_s=arrival_delta,
                    timestamp_kind='PS decode completion; nominal source ticks logged separately')


class SampleHistory:
    def __init__(self, capacity=128, max_age_s=0.25):
        if capacity < 2 or not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError('Invalid history configuration')
        self.samples = deque(maxlen=capacity)
        self.max_age_s = max_age_s
        self.first_time = None
        self.drops = 0

    def add(self, frame):
        stamp = frame['received_monotonic']
        if not math.isfinite(stamp):
            raise ValueError('Invalid sample timestamp')
        if self.samples and stamp < self.samples[-1]['received_monotonic']:
            raise ValueError('Non-monotonic sample order')
        if self.first_time is None:
            self.first_time = stamp
        if len(self.samples) == self.samples.maxlen:
            self.drops += 1  # intentional rolling history, not UART loss
        self.samples.append(dict(frame))

    def select(self, requested_at, received_at, camera_sequence):
        if not all(math.isfinite(v) for v in (requested_at, received_at)) or received_at < requested_at:
            raise ValueError('Invalid camera transfer timestamps')
        row = dict(camera_sequence=camera_sequence, camera_requested_monotonic=requested_at,
                   camera_received_monotonic=received_at,
                   camera_transfer_ms=(received_at-requested_at)*1000,
                   association='latest decoded sensor at or before camera USB request',
                   hardware_synchronized=False, exposure_timestamp_available=False)
        sample = next((s for s in reversed(self.samples)
                       if s['received_monotonic'] <= requested_at), None)
        if sample is None:
            row['status'] = 'startup' if self.first_time is None or requested_at < self.first_time else 'history_expired'
            return None, row
        age = requested_at-sample['received_monotonic']
        row.update(sensor_sequence=sample['sequence'], sensor_received_monotonic=sample['received_monotonic'],
                   sensor_age_ms=age*1000, source_t20=sample['source_t20'], source_t5=sample['source_t5'],
                   pose_epoch=sample.get('pose_epoch'))
        row['status'] = 'invalid_sensor' if not sample['valid'] else 'stale' if age > self.max_age_s else 'paired'
        return (dict(sample) if row['status']=='paired' else None), row


def alignment_summary(rows):
    statuses = {}
    ages = []
    for row in rows:
        key = row['status']
        statuses[key] = statuses.get(key, 0) + 1
        if key == 'paired':
            ages.append(row['sensor_age_ms'])
    ages.sort()
    return dict(frames=len(rows), statuses=statuses,
                paired_age_max_ms=max(ages) if ages else None,
                paired_age_p95_ms=ages[max(0, math.ceil(len(ages)*.95)-1)] if ages else None,
                association_pass=bool(ages and not any(statuses.get(k, 0) for k in
                    ('stale', 'invalid_sensor', 'history_expired'))),
                hardware_synchronized=False, exposure_timestamp_available=False,
                timestamp_note='PS Python decode/request/receive times; native ring and wire delay are unmeasured. Source ticks are nominal PIT counts, not PS timestamps.')
