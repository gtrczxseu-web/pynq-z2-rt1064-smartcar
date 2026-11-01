"""Saved logical routes; the transmitter selects the fixed initial wire axes."""
import json,math
from pathlib import Path
from telemetry_reference import VEHICLE_INITIAL,CAMERA_INITIAL,validate_reference

NAMES=('FORWARD','BACK','RIGHT','LEFT')
LEGACY=((0,0),(2,0),(1,0),(3,0),(0,0),(None,-90),(2,-90),(None,-180),(1,-180),(None,90),(3,90),(None,0))


def default_settings():
    return dict(schema=1,mode='step',speed_percent=100,preset='competition',
        steps=[dict(kind='turn',heading_deg=h) if d is None else dict(kind='move',direction=NAMES[d],distance_m=2.0,heading_deg=h) for d,h in LEGACY])


def number(v,lo,hi):
    if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError('Value outside %.2f..%.2f'%(lo,hi))
    return float(v)


def validate(data):
    if data.get('schema')!=1 or data.get('mode') not in ('step','continuous'):raise ValueError('Invalid route schema/mode')
    speed=number(data.get('speed_percent'),10,200)
    if data.get('preset') in ('translate','rotate') and data['mode']!='step':raise ValueError('Single-action routes require STEP mode')
    if data.get('preset')=='rotate' and speed<20:raise ValueError('ROTATE speed must be 20..200 percent')
    if speed!=int(speed):raise ValueError('Speed percentage must be an integer')
    steps=data.get('steps')
    if not isinstance(steps,list) or not 1<=len(steps)<=12:raise ValueError('Use 1 to 12 route actions')
    result=[]
    for s in steps:
        kind=s.get('kind');h=number(s.get('heading_deg'),-180,180)
        if kind=='turn':
            action=dict(kind=kind,heading_deg=round(h,2))
            if 'rotation_deg' in s:
                rotation=number(s['rotation_deg'],90,720)
                if rotation not in (90,180,360,720):raise ValueError('Use fixed CW turns: 90, 180, 360 or 720 degrees')
                action['rotation_deg']=int(rotation)
            result.append(action)
        elif kind=='move' and s.get('direction') in NAMES:
            distance=number(s.get('distance_m'),.02,4.0)
            # Wire distance resolution is 0.01 grid = 2 mm.
            result.append(dict(kind=kind,direction=s['direction'],distance_m=round(distance/.002)*.002,heading_deg=round(h,2)))
        else:raise ValueError('Unknown route action/direction')
    return dict(schema=1,mode=data['mode'],speed_percent=int(speed),preset=str(data.get('preset','custom'))[:32],steps=result)


def load(folder):
    p=Path(folder)/'route_settings.json'
    return validate(json.loads(p.read_text('utf8'))) if p.exists() else default_settings()


def save(folder,data):
    data=validate(data);p=Path(folder)/'route_settings.json';t=p.with_suffix('.tmp')
    t.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8');t.replace(p)
    return data


def preset(name='competition',length=2.0,width=2.0,mode='step',speed=100,turn_degrees=90):
    data=default_settings();data.update(preset=name,mode=mode,speed_percent=speed)
    if name=='competition':
        for s in data['steps']:
            if s['kind']=='move':s['distance_m']=length
        for index,factor in ((5,1),(6,1),(7,2),(8,2),(9,3),(10,3)):
            data['steps'][index]['heading_deg']=(-turn_degrees*factor+180)%360-180
        if turn_degrees!=90:
            for index in (5,7,9,11):data['steps'][index]['rotation_deg']=turn_degrees
    elif name=='rectangle':
        data['steps']=[dict(kind='move',direction=d,distance_m=v,heading_deg=0) for d,v in [('FORWARD',length),('RIGHT',width),('BACK',length),('LEFT',width)]]
    elif name=='outback':data['steps']=[dict(kind='move',direction=d,distance_m=length,heading_deg=0) for d in ('FORWARD','BACK')]
    elif name=='translate':data['steps']=[dict(kind='move',direction='FORWARD',distance_m=length,heading_deg=0)]
    elif name=='rotate':data['steps']=[dict(kind='turn',heading_deg=0,rotation_deg=turn_degrees)]
    else:raise ValueError('Unknown preset')
    return validate(data)


def configuration(data,reference_frame=VEHICLE_INITIAL):
    validate_reference(reference_frame)
    data=validate(data);items=[(0x10,len(data['steps'])),(0x11,round(.8*data['speed_percent']))]
    if data.get('preset')=='rotate':items.append((0x13,round(.4*data['speed_percent'])))
    for i,s in enumerate(data['steps']):
        heading=round(s['heading_deg']*100)&65535
        if s['kind']=='turn':items.append((0x50+i,s['rotation_deg']*10) if 'rotation_deg' in s else (0x40+i,heading))
        else:
            direction=NAMES.index(s['direction'])^(1 if reference_frame==CAMERA_INITIAL else 0)
            value=(direction<<14)|round(s['distance_m']/.002)
            items.extend([(0x20+i,value),(0x30+i,heading)])
    return items+[(0x12,0x1234)]


def requires_v3(data):
    default=default_settings();return data['steps']!=default['steps'] or data['speed_percent']!=100


def step_status(data,index):
    s=data['steps'][index]
    return dict(action=s['direction'] if s['kind']=='move' else 'TURN',lap=1 if data.get('preset')!='competition' or index<4 else 2,leg=index+1,total_steps=len(data['steps']))
