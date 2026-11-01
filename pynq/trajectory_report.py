"""ARM trajectory overlay and measured route reports; never commands motors."""
import csv,html,json,math,queue,time,uuid
from pathlib import Path
from functools import lru_cache
import numpy as np
import cv2
from PIL import Image,ImageDraw,ImageFont
from telemetry_reference import VEHICLE_INITIAL,CAMERA_INITIAL,validate_reference,reference_description


def vehicle_pose(sample,reference_frame=VEHICLE_INITIAL):
    """Resolve raw vehicle coordinates once, then select the fixed run axes."""
    validate_reference(reference_frame)
    if sample is None:return None
    flip=-1 if sample.get('reference_frame')=='camera_initial_y_negative' else 1
    try:
        x=float(sample['vehicle_x_m'] if 'vehicle_x_m' in sample else flip*sample['x_m'])
        y=float(sample['vehicle_y_m'] if 'vehicle_y_m' in sample else flip*sample['y_m'])
        yaw=float(sample.get('vehicle_heading_deg',sample['heading_deg']))
        t=float(sample['received_monotonic']);distance=float(sample['distance_m'])
        speed=float(sample['speed_m_s'])
    except (KeyError,TypeError,ValueError):return None
    if not all(math.isfinite(v) for v in (x,y,yaw,t,distance,speed)):return None
    if reference_frame==CAMERA_INITIAL:x,y=-x,-y
    return dict(t=t,x=x,y=y,heading=yaw,distance=distance,speed=speed,
        valid=bool(sample.get('pose_valid',sample.get('valid',False))),
        epoch=(sample.get('pose_epoch'),sample.get('source_clock_epoch')),
        gaps=int(sample.get('pose_gap_count',0)),source=sample.get('source','unknown'))


@lru_cache(maxsize=4)
def text_font(size):
    for name in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf','C:/Windows/Fonts/arial.ttf'):
        if Path(name).exists():return ImageFont.truetype(name,size)
    return ImageFont.load_default()


def path_lines(pixels,selected):
    """Join consecutive segments without bridging gaps or validity changes."""
    indices=np.flatnonzero(selected)
    if not len(indices):return []
    groups=np.split(indices,np.flatnonzero(np.diff(indices)>1)+1)
    return [np.ascontiguousarray(pixels[g[0]:g[-1]+2]) for g in groups]


def map_image(points,heading=0,size=(244,188),caption='WAIT FOR BTN0',distance=0,elapsed=0,minimal=False,turning=False,turn_markers=(),vision_markers=()):
    image=Image.new('RGB',size,(10,22,35));draw=ImageDraw.Draw(image)
    small=text_font(11);title=text_font(13)
    draw.rounded_rectangle((0,0,size[0]-1,size[1]-1),radius=7,outline=(80,110,140))
    draw.text((9,5),'TRAJECTORY' if minimal else 'LIVE TRAJECTORY',font=title,fill=(228,239,251))
    # This 188-pixel panel stays above the existing PL numeric HUD at y=336.
    left,top,right,bottom=(12,32,size[0]-12,size[1]-32) if minimal else (12,30,164,145)
    xs=[0]+[p[0] for p in points];ys=[0]+[p[1] for p in points]
    cx=(min(xs)+max(xs))/2;cy=(min(ys)+max(ys))/2
    span=max(1.0,max(xs)-min(xs),max(ys)-min(ys))*1.25
    scale=min(right-left,bottom-top)/span
    def xy(x,y):return ((left+right)/2+(x-cx)*scale,(top+bottom)/2-(y-cy)*scale)
    for k in range(5):
        x=left+(right-left)*k/4;y=top+(bottom-top)*k/4
        draw.line((x,top,x,bottom),fill=(32,49,66));draw.line((left,y,right,y),fill=(32,49,66))
    ox,oy=xy(0,0);draw.ellipse((ox-3,oy-3,ox+3,oy+3),fill=(103,224,151))
    if len(points)>1:
        data=np.asarray(points);pixels=np.rint(np.column_stack(((left+right)/2+(data[:,0]-cx)*scale,
            (top+bottom)/2-(data[:,1]-cy)*scale))).astype(np.int32)
        layer=np.asarray(image).copy()
        for good,colour in ((True,(80,194,255)),(False,(255,189,88))):
            lines=path_lines(pixels,~data[1:,4].astype(bool)&(data[1:,3].astype(bool)==good))
            if lines:cv2.polylines(layer,lines,False,colour,2)
        image=Image.fromarray(layer);draw=ImageDraw.Draw(image)
    for i,marker in enumerate(turn_markers):
        if marker['x_m'] is None:continue
        mx,my=xy(marker['x_m'],marker['y_m'])
        draw.ellipse((mx-7,my-7,mx+7,my+7),outline=(255,221,82),width=2)
        draw.text((min(mx+8,right-18),max(top,my-15)),'T%d'%(i+1),font=small,fill=(255,221,82))
    for marker in vision_markers[-8:]:
        if not marker['position_available']:continue
        mx,my=xy(marker['x_m'],marker['y_m']);colour=tuple(marker['color_rgb'])
        draw.rectangle((mx-4,my-4,mx+4,my+4),fill=colour,outline=(240,245,250))
        draw.text((min(mx+6,right-25),max(top,my-13)),marker['id'],font=small,fill=colour)
    px,py=xy(*(points[-1][:2] if points else (0,0)))
    angle=math.radians(heading);dx=-math.sin(angle);dy=-math.cos(angle)
    tip=(px+10*dx,py+10*dy)
    draw.polygon((tip,(px-5*dx+5*dy,py-5*dy-5*dx),(px-5*dx-5*dy,py-5*dy+5*dx)),fill=(196,128,255) if turning else (255,112,93))
    if minimal:
        draw.text((size[0]-100,7),'+X RIGHT / +Y UP',font=small,fill=(173,194,213))
        note='TURNING' if turning else ('DATA GAP' if points and not points[-1][3] else ('T = TURN END' if turn_markers else 'ENCODER + IMU'))
        if vision_markers and not turning and note!='DATA GAP':note='V = CAR AT SIGHTING'
        draw.text((12,size[1]-22),'%.1f m span | %s'%(span,note),font=small,fill=(255,189,88) if note=='DATA GAP' else (173,194,213))
    else:
        for pos,text in [((174,32),'+Y UP'),((174,49),'+X RIGHT'),((174,73),'%.2f m'%distance),((174,90),'%.1f s'%elapsed),((174,115),'START'),((174,132),'green'),((12,170),'SPAN %.1fm | ENCODER + IMU'%span)]:
            draw.text(pos,text,font=small,fill=(173,194,213))
        draw.text((12,153),caption[:23],font=small,fill=(255,203,111))
    return image


class RouteReport:
    """One explicit BTN0 start = one report; all recorder work is off the RX path."""
    def __init__(self,folder,clock=time.monotonic,rect=(388,140,244,188),minimal=False,reference_frame=VEHICLE_INITIAL):
        from rear_projection import RearProjection
        self.folder=Path(folder);self.clock=clock;self.pending=queue.Queue()
        self.reference_frame=validate_reference(reference_frame)
        self.camera_view='forward' if reference_frame==CAMERA_INITIAL else 'rear'
        self.projection=RearProjection(self.folder.parent,clock,reference_frame=reference_frame)
        self.run=None;self.step=None;self.previous=None;self.points=[];self.origin=None
        self.turning=False;self.turn_markers=[]
        self.vision_tracks={}
        self.csv_file=None;self.csv_writer=None;self.latest_report=None;self.error=None
        self.event_log_path=None;self.event_log_error=None
        self.rect=rect;self.minimal=minimal
        self.panel=np.asarray(map_image([],size=rect[2:],minimal=minimal))[:,:,::-1].copy();self.last_panel=-1e9
        self.panel_key=None
    def pose(self,sample):self.pending.put(('pose',dict(sample)))
    def event(self,event,sample=None):self.pending.put(('event',(dict(event),None if sample is None else dict(sample))))
    def vision(self,stamp,detections,sample=None,session=0,channel='all'):
        # Copy capture-time telemetry; inference completion is later than the image.
        self.pending.put(('vision',(stamp,[dict(d) for d in detections],None if sample is None else dict(sample),session,channel)))
    def _begin(self,event):
        if self.run is not None:self._finish('interrupted','New explicit run',event['monotonic'])
        ident='run_%08x_%s'%(event['session'],uuid.uuid4().hex[:8])
        target=self.folder/ident;target.mkdir(parents=True)
        self.run=dict(schema=1,run_id=ident,session=event['session'],started_monotonic=event['monotonic'],
            board_wall_time=time.strftime('%Y-%m-%d %H:%M:%S'),time_basis='monotonic seconds; board wall clock is informational',
            source='PL encoder displacement and IMU yaw; ARM dead reckoning',
            axes=reference_description(self.reference_frame)['axes']+'; positive yaw anticlockwise',reference_frame=self.reference_frame,
            recipe=event.get('recipe'),steps=[],turn_markers=[],recognitions=[],recognitions_dropped=0,
            traffic_responses=[],traffic_responses_dropped=0,
            recognition_position_basis='Observer vehicle position at image submission, relative to run origin; not target coordinates',
            camera_view=self.camera_view,sample_count=0,invalid_pose_samples=0,data_breaks=0,measured_distance_m=0.0,
            peak_speed_m_s=0.0,report_dir=str(target),raw_capture_logs=str(self.folder.parent/'logs'))
        self.step=None;self.previous=None;self.points=[];self.origin=None
        self.next_step_speed=(event.get('recipe') or {}).get('speed_percent')
        self.last_signal=None;self.traffic_response=None
        self.turning=False;self.turn_markers=self.run['turn_markers']
        self.vision_tracks={}
        self.csv_file=(target/'trajectory.csv').open('w',newline='',encoding='utf8')
        self.csv_writer=csv.writer(self.csv_file)
        axes=['camera_x_m','camera_y_m'] if self.reference_frame==CAMERA_INITIAL else ['vehicle_x_m','vehicle_y_m']
        self.csv_writer.writerow(['monotonic','elapsed_s']+axes+['relative_x_m','relative_y_m','heading_deg','distance_counter_m','speed_m_s','pose_valid','line_break','step'])
    def _sample(self,sample):
        p=vehicle_pose(sample,self.reference_frame)
        if p is None:return
        if self.run is None:
            if not self.points:self.preview=p
            return
        a=self.previous
        if a is not None and p['t']<=a['t']:return
        broken=bool(a and (p['epoch']!=a['epoch'] or p['gaps']>a['gaps'] or p['t']-a['t']>.5 or p['distance']<a['distance']))
        increment=p['distance']-a['distance'] if a is not None and p['epoch']==a['epoch'] and p['distance']>=a['distance'] else 0.0
        self.run['measured_distance_m']+=increment
        self.run['sample_count']+=1;self.run['invalid_pose_samples']+=int(not p['valid'])
        self.run['data_breaks']+=int(broken)
        self.run['peak_speed_m_s']=max(self.run['peak_speed_m_s'],p['speed'])
        if self.step is not None:
            self.step['distance_m']+=increment
            self.step['peak_speed_m_s']=max(self.step['peak_speed_m_s'],p['speed'])
            if self.step.get('measured_rotation_deg') is not None:
                if a is None or broken or not (a['valid'] and p['valid']):self.step['measured_rotation_deg']=None
                else:
                    self.step['measured_rotation_deg']-=((p['heading']-a['heading']+180)%360)-180
                    self.step['rotation_samples']+=1
        if self.origin is None:
            self.origin=(p['x'],p['y']);self.run['initial_pose']=dict(x_m=p['x'],y_m=p['y'],heading_deg=p['heading'],pose_epoch=p['epoch'])
        x=p['x']-self.origin[0];y=p['y']-self.origin[1]
        self.points.append((x,y,p['heading'],p['valid'],broken))
        if len(self.points)>2000:
            # Keep the origin and endpoint, and never bridge a discarded gap.
            reduced=[self.points[0]]
            for i in range(2,len(self.points),2):
                q=list(self.points[i]);q[4]=any(v[4] for v in self.points[i-1:i+1]);reduced.append(tuple(q))
            self.points=reduced
        self.run['final_pose']=dict(x_m=p['x'],y_m=p['y'],heading_deg=p['heading'])
        self.csv_writer.writerow([p['t'],p['t']-self.run['started_monotonic'],p['x'],p['y'],x,y,p['heading'],p['distance'],p['speed'],p['valid'],broken,None if self.step is None else self.step['index']+1])
        self.previous=p
    def _vision(self,stamp,detections,sample,session,channel='all'):
        if self.run is None or session!=self.run['session'] or stamp<self.run['started_monotonic'] or not 0<=self.clock()-stamp<=1.5:return
        labels={}
        colors={'RED':(255,112,93),'GREEN':(103,224,151),'BLUE':(83,161,255),'YELLOW':(246,202,119)}
        for d in detections:
            parts=d.get('label','').split()
            if len(parts)==1 and parts[0] in colors:labels[parts[0]]=dict(kind='color',value=parts[0],color_rgb=colors[parts[0]])
            elif len(parts)>=2 and parts[0]=='DIGIT' and parts[1] in tuple(str(i) for i in range(10)):
                labels['DIGIT '+parts[1]]=dict(kind='digit',value=parts[1],color_rgb=(215,174,255))
        for label,track in list(self.vision_tracks.items()):
            if channel=='colors' and label.startswith('DIGIT '):continue
            if channel=='digits' and not label.startswith('DIGIT '):continue
            if label not in labels:track['count']=0
            if stamp-track['last_seen']>=2:self.vision_tracks.pop(label)
        for label,description in labels.items():
            track=self.vision_tracks.setdefault(label,dict(last_seen=stamp,count=0,recorded=False))
            if stamp<=track['last_seen'] and track['count']:continue
            track['count']=track['count']+1 if stamp-track['last_seen']<=1 else 1;track['last_seen']=stamp
            if track['recorded'] or track['count']<2:continue
            track['recorded']=True
            if len(self.run['recognitions'])>=128:self.run['recognitions_dropped']+=1;continue
            p=vehicle_pose(sample,self.reference_frame)
            located=bool(p and p['valid'] and 0<=stamp-p['t']<=.5 and self.origin is not None and self.previous is not None and p['epoch']==self.previous['epoch']==self.run['initial_pose']['pose_epoch'])
            steps=self.run['steps']+([self.step] if self.step is not None else [])
            capture_step=next((s['index']+1 for s in steps if s['started_monotonic'] is not None and s['started_monotonic']<=stamp<s.get('ended_monotonic',math.inf)),None)
            self.run['recognitions'].append(dict(description,id='V%d'%(len(self.run['recognitions'])+1),
                monotonic=stamp,elapsed_s=stamp-self.run['started_monotonic'],label=label,camera_view=self.camera_view,
                position_available=located,x_m=p['x']-self.origin[0] if located else None,
                y_m=p['y']-self.origin[1] if located else None,heading_deg=p['heading'] if located else None,
                step=capture_step))
            self.last_panel=-1e9
    def _close_step(self,t,completed,turn_completed=False):
        if self.step is None:return
        if turn_completed and self.step.get('kind')=='TURN':
            p=self.previous;located=p is not None and 0<=t-p['t']<=.5 and self.origin is not None
            self.turn_markers.append(dict(step=self.step['index']+1,monotonic=t,
                x_m=p['x']-self.origin[0] if located else None,y_m=p['y']-self.origin[1] if located else None,
                heading_deg=p['heading'] if located else None,pose_valid=bool(located and p['valid']),
                position_available=located))
        self.turning=False
        start=self.step['started_monotonic']
        waiting=self.step.get('traffic_wait_s',0)
        if self.step.get('hold_started_monotonic') is not None:waiting+=max(0,t-self.step.pop('hold_started_monotonic'))
        self.step.update(ended_monotonic=t,duration_s=None if start is None else max(0,t-start-waiting),traffic_wait_s=waiting,completed=completed)
        dt=self.step['duration_s'];self.step['average_speed_m_s']=self.step['distance_m']/dt if dt else None
        recipe=(self.run.get('recipe') or {}).get('steps',[])
        command=recipe[self.step['index']] if self.step['index']<len(recipe) else {}
        self.step['command_distance_m']=command.get('distance_m')
        rotation=command.get('rotation_deg')
        if command.get('kind')=='turn' and rotation is None and self.step.get('initial_heading_deg') is not None:
            rotation=-((command['heading_deg']-self.step['initial_heading_deg']+180)%360-180)
        self.step['command_rotation_deg']=rotation
        if (not self.step.get('rotation_samples') or self.previous is None or not 0<=t-self.previous['t']<=.5):self.step['measured_rotation_deg']=None
        self.run['steps'].append(self.step);self.step=None
    def _event(self,event,sample):
        kind=event['kind'];t=event['monotonic']
        if kind=='start_requested':self._begin(event)
        if self.run is None:return
        self._sample(sample)
        if kind=='continue_requested':self.next_step_speed=event.get('speed_percent')
        elif kind=='signal_card' and event.get('label') in ('RED','YELLOW','OBSTACLE'):
            self.last_signal=(event['label'],event.get('captured_monotonic'))
        elif kind=='signal_hold':
            captured=self.last_signal[1] if self.last_signal and self.last_signal[0]==event.get('reason') else None
            self.traffic_response=dict(reason=event.get('reason'),confirmed_frame_monotonic=captured,command_monotonic=t,held_monotonic=None,confirmed_frame_to_held_s=None,command_to_held_s=None)
            if len(self.run['traffic_responses'])<128:self.run['traffic_responses'].append(self.traffic_response)
            else:self.run['traffic_responses_dropped']+=1
        if kind=='mcu_state' and event.get('session')==self.run['session']:
            state=event['state']
            if state=='HELD' and self.traffic_response is not None:
                response=self.traffic_response;response['held_monotonic']=t
                response['command_to_held_s']=max(0,t-response['command_monotonic'])
                captured=response['confirmed_frame_monotonic']
                if captured is not None and captured<=t:response['confirmed_frame_to_held_s']=t-captured
                self.traffic_response=None
            if self.step is not None and self.step['index']==event['step']:
                if state in ('HOLDING','HELD'):
                    self.step.setdefault('hold_started_monotonic',t);self.turning=False;self.last_panel=-1e9
                elif state in ('MOVE','TURN','BYPASS_RIGHT','BYPASS_FORWARD','BYPASS_LEFT') and self.step.get('hold_started_monotonic') is not None:
                    self.step['traffic_wait_s']=self.step.get('traffic_wait_s',0)+max(0,t-self.step.pop('hold_started_monotonic'))
                    self.turning=state=='TURN';self.last_panel=-1e9
            if state in ('MOVE','TURN') and (self.step is None or self.step['index']!=event['step']):
                self._close_step(t,True)
                self.step=dict(index=event['step'],lap=event['lap'],leg=event['leg'],action=event['action'],kind=state,
                    started_monotonic=t,distance_m=0.0,peak_speed_m_s=0.0,speed_limit_percent=self.next_step_speed,
                    initial_heading_deg=self.previous['heading'] if self.previous is not None and self.previous['valid'] and 0<=t-self.previous['t']<=.5 else None,
                    measured_rotation_deg=0.0 if state=='TURN' and self.previous is not None and self.previous['valid'] and 0<=t-self.previous['t']<=.5 else None,rotation_samples=0)
                self.turning=state=='TURN';self.last_panel=-1e9
            elif state in ('PAUSED','DONE'):
                if self.step is None and not any(s['index']==event['step'] for s in self.run['steps']):
                    recipe=self.run.get('recipe') or {};steps=recipe.get('steps',[])
                    action=steps[event['step']]['kind'].upper() if event['step']<len(steps) else 'UNKNOWN'
                    self.step=dict(index=event['step'],lap=event['lap'],leg=event['leg'],action=event['action'],kind=action,
                        started_monotonic=None,distance_m=None,peak_speed_m_s=None,speed_limit_percent=None,measured_rotation_deg=None)
                self._close_step(t,True,turn_completed=True);self.last_panel=-1e9
        if kind=='completed':self._finish('completed','All route actions completed',t)
        elif kind=='stop':self._finish('fault' if event.get('fault') else 'stopped',event.get('reason','Operator stop'),t)
    def _finish(self,outcome,reason,t):
        if self.run is None:return
        self._close_step(t,False);r=self.run
        r.update(outcome=outcome,reason=reason,ended_monotonic=t,wall_duration_s=max(0,t-r['started_monotonic']))
        r['unobserved_action_starts']=sum(s['started_monotonic'] is None for s in r['steps'])
        complete=r['unobserved_action_starts']==0
        r['action_duration_s']=sum(s['duration_s'] for s in r['steps']) if complete else None
        r['waiting_duration_s']=max(0,r['wall_duration_s']-r['action_duration_s']) if complete else None
        r['action_distance_m']=sum(s['distance_m'] for s in r['steps']) if complete else None
        r['average_action_speed_m_s']=r['action_distance_m']/r['action_duration_s'] if r['action_duration_s'] else None
        r['completed_steps']=sum(s['completed'] for s in r['steps'])
        r['measurement_available']=r['sample_count']>=2
        r['last_sample_age_s']=None if self.previous is None else max(0,t-self.previous['t'])
        r['quality_note']='Encoder/IMU estimate; no absolute positioning. Missing intervals are not reconstructed. Distance uses nondecreasing counter intervals within the same epoch.'
        r['sample_period_note']='Raw pose callbacks; screen refresh limited to 2 Hz. Boundary metrics use the latest received telemetry.'
        target=Path(r['report_dir']);self.csv_file.close();self.csv_file=None
        self.latest_report=r;self.run=None
        (target/'summary.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        heading=self.points[-1][2] if self.points else 0
        with (target/'recognitions.csv').open('w',newline='',encoding='utf8') as f:
            fields=['id','elapsed_s','kind','value','camera_view','position_available','x_m','y_m','heading_deg','step']
            writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(r['recognitions'])
        map_image(self.points,heading,caption=outcome.upper(),distance=r['measured_distance_m'],elapsed=r['wall_duration_s'],turn_markers=self.turn_markers,vision_markers=r['recognitions']).save(target/'trajectory.png')
        self._html(r,target)
        (self.folder/'latest.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        (self.folder/'index.html').write_text('<meta charset="utf-8"><meta http-equiv="refresh" content="0;url='+r['run_id']+'/report.html">',encoding='utf8')
    def _html(self,r,target):
        def number(value,places):return '--' if value is None else '%.*f'%(places,value)
        points=self.points;span=max([1.0]+[abs(v) for p in points for v in p[:2]])*1.15
        def xy(p):return (300+250*p[0]/span,215-175*p[1]/span)
        lines=[]
        for a,b in zip(points,points[1:]):
            if b[4]:continue
            ax,ay=xy(a);bx,by=xy(b)
            lines.append('<path d="M%.2f %.2f L%.2f %.2f" stroke="%s"/>'%(ax,ay,bx,by,'#177bb1' if b[3] else '#ba7100'))
        for i,marker in enumerate(self.turn_markers):
            if marker['x_m'] is None:continue
            mx,my=xy((marker['x_m'],marker['y_m']))
            lines.append('<circle cx="%.2f" cy="%.2f" r="7" stroke="#b08700"/><text x="%.2f" y="%.2f" fill="#8d6800" stroke="none">T%d</text>'%(mx,my,mx+10,my-8,i+1))
        for marker in r['recognitions']:
            if not marker['position_available']:continue
            mx,my=xy((marker['x_m'],marker['y_m']));colour='#%02x%02x%02x'%tuple(marker['color_rgb'])
            lines.append('<rect x="%.2f" y="%.2f" width="8" height="8" fill="%s"/><text x="%.2f" y="%.2f" fill="%s" stroke="none">%s</text>'%(mx-4,my-4,colour,mx+7,my-8,colour,marker['id']))
        chart='<svg viewBox="0 0 600 430" role="img" aria-label="Measured trajectory"><rect width="600" height="430" fill="#f1f6fa"/><path d="M30 215H570M300 30V400" stroke="#a5b7c5"/><g fill="none" stroke-width="2">'+''.join(lines)+'</g><circle cx="300" cy="215" r="4" fill="#0a9b6b"/><text x="310" y="25">+Y forward</text><text x="470" y="232">+X right</text><text x="16" y="417">Run origin; range +/- %.2f m; gaps are disconnected</text></svg>'%span
        rows=''
        for s in r['steps']:
            turning=s['kind']=='TURN';unit=' deg CW' if turning else ' m'
            configured=s.get('command_rotation_deg') if turning else s.get('command_distance_m')
            measured=s.get('measured_rotation_deg') if turning else s['distance_m']
            values=(s['index']+1,s['action'],number(configured,2)+unit,number(measured,2)+unit,number(s.get('speed_limit_percent'),0)+'%',number(s['duration_s'],2),number(s['average_speed_m_s'],3),'yes' if s['completed'] else 'partial')
            rows+='<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in values)+'</tr>'
        speed=[]
        with (target/'trajectory.csv').open(encoding='utf8',newline='') as f:
            for row in csv.DictReader(f):speed.append((float(row['elapsed_s']),float(row['speed_m_s'])))
        maximum=max([.1]+[v for t,v in speed]);duration=max(1,r['wall_duration_s'])
        poly=' '.join('%.1f,%.1f'%(40+520*t/duration,170-140*v/maximum) for t,v in speed)
        speed_svg='<svg viewBox="0 0 600 210" role="img" aria-label="Measured speed over time"><path d="M40 20V170H570" stroke="#789" fill="none"/><polyline points="'+poly+'" stroke="#177bb1" stroke-width="2" fill="none"/><text x="40" y="198">Elapsed seconds (0 to %.1f); speed m/s (0 to %.2f)</text></svg>'%(duration,maximum)
        page='''<!doctype html><meta charset="utf-8"><title>小车运行报告</title><style>body{font:16px system-ui,sans-serif;color:#18344a;max-width:920px;margin:24px auto;padding:16px}h1{font-size:28px}table{border-collapse:collapse;width:100%}td,th{padding:9px;border-bottom:1px solid #d7e3eb;text-align:left}svg{width:100%;max-width:650px}.note{color:#6c511a;background:#fff4dc;padding:14px}a{color:#086caa}</style><h1>小车运行报告 / Route report</h1>'''
        page+='<p>'+html.escape(r['run_id'])+' | '+html.escape(r['outcome'])+' | '+html.escape(r['reason'])+'</p>'
        page+='<p>记录累计距离 %.3f m · 动作用时 %s s · 倒计时及等待 %s s · 完成 %d 步</p>'%(r['measured_distance_m'],number(r['action_duration_s'],2),number(r['waiting_duration_s'],2),r['completed_steps'])
        if r['unobserved_action_starts']:page+='<p class="note">%d步动作开始状态缺失；完成确认保留，分步距离、用时和均速未知的部分显示--，不补造数据。</p>'%r['unobserved_action_starts']
        if not r['measurement_available']:page+='<p class="note">遥测样本不足，距离统计不可用于判断实际行驶距离。</p>'
        final=r.get('final_pose');initial=r.get('initial_pose')
        if final and initial:page+='<p>起点 (%.3f, %.3f) m；终点 (%.3f, %.3f) m；相对位移 (%.3f, %.3f) m；终点航向 %.2f°。</p>'%(initial['x_m'],initial['y_m'],final['x_m'],final['y_m'],final['x_m']-initial['x_m'],final['y_m']-initial['y_m'],final['heading_deg'])
        page+='<p class="note">编码器与航向推算轨迹，并非绝对定位。数据缺口 %d 次，连续性未确认的样本 %d 个；缺失路程不补算。橙线表示连续性未确认。时间按单调时钟计算，板端日期仅供参考。</p>'%(r['data_breaks'],r['invalid_pose_samples'])
        page+='<p>黄色圆环 T1、T2… 标记转向完成位置；缺少新鲜位置数据时只在 JSON 记录事件，不补造位置。</p>'
        page+='<h2>动作设定与测量</h2><p>速度百分比为指令上限；实测均速单位为 m/s，二者不直接等同。旋转按连续 IMU 航向累加，顺时针为正；缺失开始状态或采样断开时显示 --。差值仅用于观察，不限制小车运动。</p>'
        page+='<table><tr><th>步骤</th><th>动作</th><th>设定距离/角度</th><th>实测距离/角度</th><th>速度上限</th><th>用时/s</th><th>均速/(m/s)</th><th>完成</th></tr>'+rows+'</table>'
        responses=''.join('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in (s['reason'],number(s['confirmed_frame_to_held_s'],3),number(s['command_to_held_s'],3)))+'</tr>' for s in r['traffic_responses'])
        page+='<h2>信号与视觉障碍暂停响应</h2><p>从触发确认的摄像头帧提交时间到车板 HELD 静止确认回传时间；包含处理、通信和停车过程，不是从目标首次出现开始计时。缺少确认显示 --。</p><table><tr><th>信号</th><th>确认帧至静止确认/s</th><th>发命令至静止确认/s</th></tr>'+responses+'</table>'
        sightings=r['recognitions']
        page+='<h2>摄像头巡检记录 / Camera sightings</h2><p>'+('镜头朝向为新车头，+Y为镜头前方、+X为镜头右方；物理安装仍在原车尾。' if self.camera_view=='forward' else '后视摄像头，坐标使用原车身方向。')+'</p><p>共 %d 次识别事件。V 标记代表发现时小车的位置，不是目标实际坐标；实时地图仅显示最近8次事件。同一类别连续出现只记一次，消失至少2秒后重新确认可再次记录，次数不代表独立物体数量。</p>'%len(sightings)
        if r['recognitions_dropped']:page+='<p>超过128条的事件未继续保存：%d条。</p>'%r['recognitions_dropped']
        seen_rows=''.join('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in (s['id'],number(s['elapsed_s'],2),s['label'],number(s['x_m'],3),number(s['y_m'],3),s['camera_view']))+'</tr>' for s in sightings)
        page+='<table><tr><th>标记</th><th>时间/s</th><th>识别</th><th>小车相对X/m</th><th>小车相对Y/m</th><th>视角</th></tr>'+seen_rows+'</table><p><a href="recognitions.csv">识别事件 CSV</a> · 未知位置显示 --；未记录到事件不代表沿途没有目标。</p>'
        page+=chart+speed_svg+'<p><a href="trajectory.csv">轨迹 CSV</a> · <a href="summary.json">报告 JSON</a> · <a href="trajectory.png">小地图 PNG</a></p>'
        (target/'report.html').write_text(page,encoding='utf8')
    def poll(self):
        try:
            while True:
                try:kind,value=self.pending.get_nowait()
                except queue.Empty:break
                if kind=='pose':self._sample(value)
                elif kind=='vision':self._vision(*value)
                else:
                    self._event(*value)
                    if self.event_log_path is not None:
                        try:
                            with self.event_log_path.open('a',encoding='utf8') as f:f.write(json.dumps(value[0],ensure_ascii=False)+'\n')
                        except Exception as exc:self.event_log_error=type(exc).__name__+': '+str(exc)
            if self.csv_file is not None:self.csv_file.flush()
            now=self.clock()
            if now-self.last_panel>=.5:
                self.last_panel=now;r=self.run or self.latest_report
                heading=self.points[-1][2] if self.points else getattr(self,'preview',{}).get('heading',0)
                caption='RECORDING' if self.run else (r['outcome'].upper() if r else 'WAIT FOR BTN0')
                if self.points and not self.points[-1][3]:caption+=' | DATA GAP'
                elapsed=max(0,now-r['started_monotonic']) if self.run else (r['wall_duration_s'] if r else 0)
                key=(r.get('run_id') if r else None,r.get('sample_count') if r else None,len(self.points),tuple(self.points[-1]) if self.points else None,heading,caption,self.turning,
                    len(self.turn_markers),len(r.get('recognitions',[])) if r else 0,None if self.minimal else int(elapsed*2))
                if key!=self.panel_key:
                    self.panel=np.asarray(map_image(self.points,heading,size=self.rect[2:],minimal=self.minimal,caption=caption,distance=r['measured_distance_m'] if r else 0,elapsed=elapsed,turning=self.turning,turn_markers=self.turn_markers,vision_markers=r.get('recognitions',[]) if r else ()))[:,:,::-1].copy()
                    self.panel_key=key
        except Exception as exc:
            # Recording/display failures are reported, never turned into motor gates.
            self.error=type(exc).__name__+': '+str(exc)
    def overlay(self,output):
        x,y,w,h=self.rect
        cv2.mixChannels([self.panel],[output[y:y+h,x:x+w]],[0,0,1,1,2,2])
    def planned_path(self,current,heading):
        """Only the remaining current translation, in the recorded map axes."""
        r=self.run
        if r is None:return [],'PLAN WAIT START'
        steps=(r.get('recipe') or {}).get('steps',[])
        index=self.step['index'] if self.step is not None else (r['steps'][-1]['index']+1 if r['steps'] else 0)
        if index>=len(steps):return [],'PLAN COMPLETE'
        if steps[index]['kind']=='turn':return [],'PLAN TURN'
        a=math.radians((r.get('initial_pose') or {}).get('heading_deg',0));c,s=math.cos(a),math.sin(a)
        vectors={'FORWARD':(0,1),'BACK':(0,-1),'RIGHT':(1,0),'LEFT':(-1,0)};target=np.zeros(2)
        for step in steps[:index+1]:
            if step['kind']=='move':
                x,y=vectors[step['direction']];unit=np.array((c*x-s*y,s*x+c*y))
                target+=unit*step['distance_m']
        remaining=float(np.dot(target-np.asarray(current),unit))
        if remaining<=.01:return [],'PLAN ARRIVAL'
        a=math.radians(heading);right=math.cos(a)*unit[0]+math.sin(a)*unit[1];forward=-math.sin(a)*unit[0]+math.cos(a)*unit[1]
        caption='PLAN FORWARD' if forward>.5 else ('PLAN REVERSE' if forward<-.5 else ('PLAN RIGHT' if right>0 else 'PLAN LEFT'))
        distance=min(remaining,2.5);count=max(2,math.ceil(distance/.1)+1)
        points=[(current[0]+unit[0]*d,current[1]+unit[1]*d,heading,True,False) for d in np.linspace(0,distance,count)]
        return points,caption
    def camera_overlay(self,frame):
        p=self.previous
        current=self.points[-1][:2] if self.points else (0,0)
        fresh=p is not None and 0<=self.clock()-p['t']<=.5
        heading=p['heading'] if p else 0;planned,caption=self.planned_path(current,heading)
        self.projection.overlay(frame,planned,current,heading,(),fresh,caption=caption)
    def snapshot(self):
        r=self.run or self.latest_report or {}
        return dict(recording=self.run is not None,error=self.error,event_log_error=self.event_log_error,turning=self.turning,turn_markers=len(self.turn_markers),projection_error=self.projection.error,pending_items=self.pending.qsize(),
            recognition_events=len(r.get('recognitions',[])),recognitions_dropped=r.get('recognitions_dropped',0),camera_view=self.camera_view,reference_frame=self.reference_frame,
            latest_report=None if self.latest_report is None else str(Path(self.latest_report['report_dir'])/'report.html'),
            sample_count=0 if self.run is None else self.run['sample_count'])
    def close(self):
        self.poll()
        if self.run is not None:
            try:self._finish('stopped','Application exit',self.clock())
            except Exception as exc:self.error=type(exc).__name__+': '+str(exc)
        if self.csv_file is not None:self.csv_file.close();self.csv_file=None
