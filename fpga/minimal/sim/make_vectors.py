"""Independent integer reference for counter-delta sensor processing."""
from pathlib import Path
import math,random
ROOT=Path(__file__).resolve().parent
rng=random.Random(20261101)
rows=[];previous=None;count=rejected=distance=yaw=0;speed=dx=dy=0
def signed32(n):return ((n+2**31)%2**32)-2**31
def trunc(n,d):return (-1 if n<0 else 1)*(abs(n)//d)
def feed(values):
    global previous,count,rejected,distance,yaw,speed,dx,dy
    s=[v&0xffffffff for v in values]
    if previous is not None:
        dt20=(s[5]-previous[5])&0xffffffff;dt5=(s[6]-previous[6])&0xffffffff
        if not (0<dt20<=50 and 0<dt5<=200 and abs(4*dt20-dt5)<=5):rejected+=1
        else:
            a,b,c,d=[signed32(s[i]-previous[i]) for i in range(4)]
            dx=trunc((a-b-c+d)*200000,10140)
            dy=trunc((a+b+c+d)*200000,9720)
            step=math.isqrt(dx*dx+dy*dy);distance+=step;speed=step*50//dt20
            bias=((s[7]+32768)%65536)-32768
            delta=(signed32(s[4]-previous[4])-bias*dt5)*2291
            yaw=(yaw+delta+1179648000)%2359296000-1179648000
            count+=1
    previous=s[:7]
    rows.append(s+[count,rejected,dx&0xffffffff,dy&0xffffffff,speed,distance&0xffffffff,distance>>32,yaw&0xffffffff])
feed([0]*8)
feed([2430]*4+[0,5,20,0])
feed([2430]*4+[28600,10,40,0])
assert rows[1][10:16]==[0,200000,2000000,200000,0,0]
assert rows[2][15]==65522600
cur=[2430]*4+[28600,10,40,0]
for index in range(35):
    dt=rng.randint(1,5)
    for i in range(4):cur[i]=(cur[i]+rng.randint(-2500,2500))&0xffffffff
    cur[4]=(cur[4]+rng.randint(-30000,30000))&0xffffffff
    cur[5]=(cur[5]+dt)&0xffffffff;cur[6]=(cur[6]+4*dt)&0xffffffff
    cur[7]=rng.choice([-5,0,7])
    feed(cur)
# A duplicate must reject/rebase without adding path or publishing a sample.
feed(cur)
cur[5]=(cur[5]+1)&0xffffffff;cur[6]=(cur[6]+4)&0xffffffff
for i in range(4):cur[i]=(cur[i]-243)&0xffffffff
feed(cur)
# Counter restart, then valid recovery from the new baseline.
feed([0]*8);feed([243,243,243,243,0,1,4,0])
# Large positive/negative yaw steps exercise normalization across +/-180 deg.
feed([243,243,243,243,600000,6,24,0])
feed([243,243,243,243,-300000,11,44,0])
# Rebase then wrap both timestamp counters and signed encoder accumulators.
feed([0xfffffff0]*4+[0xfffffff0,0xfffffffe,0xfffffff8,0])
feed([0x96e]*4+[0x100,3,12,0])
(ROOT/'vectors.mem').write_text('\n'.join(' '.join(f'{v:08x}' for v in row) for row in rows)+'\n',encoding='ascii')
(ROOT/'vectors_count.vh').write_text(f'localparam VECTOR_COUNT={len(rows)};\n',encoding='ascii')
print('REFERENCE_VECTORS',len(rows))
