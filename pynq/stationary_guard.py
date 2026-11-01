"""Conservative startup check; low-rate constant rotation still needs physical stillness."""
import math

def signed_delta(a,b):
    value=(b-a)&0xffffffff
    return value-(1<<32) if value&(1<<31) else value

def check_stationary(samples,expected_bias_lsb=-5,max_bias_deviation_lsb=15,
                     max_gyro_spread_lsb=8,max_encoder_span=2,min_samples=25):
    if len(samples)<min_samples:raise RuntimeError('WAIT STILL: insufficient fresh sensor frames')
    if any(len(row)!=7 for row in samples):raise ValueError('Invalid RAW33 counter row')
    values=(expected_bias_lsb,max_bias_deviation_lsb,max_gyro_spread_lsb,max_encoder_span)
    if not all(math.isfinite(v) for v in values) or min(values[1:])<0:raise ValueError('Invalid stationary limits')
    paths=[[0] for _ in range(4)];means=[];ticks=0
    for a,b in zip(samples,samples[1:]):
        dt20=(b[5]-a[5])&0xffffffff;dt5=(b[6]-a[6])&0xffffffff
        if not 0<dt20<=10 or not 0<dt5<=40 or abs(dt20*.02-dt5*.005)>.0250001:
            raise RuntimeError('WAIT STILL: source counter discontinuity')
        ticks+=dt5;means.append(signed_delta(a[4],b[4])/dt5)
        for i in range(4):paths[i].append(paths[i][-1]+signed_delta(a[i],b[i]))
    span=max(max(path)-min(path) for path in paths)
    if span>max_encoder_span:raise RuntimeError('WAIT STILL: encoder movement detected')
    mean=signed_delta(samples[0][4],samples[-1][4])/ticks
    spread=max(means)-min(means)
    if abs(mean-expected_bias_lsb)>max_bias_deviation_lsb or spread>max_gyro_spread_lsb:
        raise RuntimeError('WAIT STILL: gyro motion or unexpected bias')
    if ticks*.005<2.3:raise RuntimeError('WAIT STILL: stationary window too short')
    return dict(frames=len(samples),encoder_span=span,gyro_mean_lsb=mean,gyro_spread_lsb=spread,
                source_window_s=ticks*.005,expected_bias_lsb=expected_bias_lsb,
                note='Encoder and gyro gate; physical stillness required, not an absolute motion detector')
