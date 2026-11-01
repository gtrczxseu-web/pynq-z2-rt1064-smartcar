"""Small on-screen menu. Edits a next-run file; never commands a motor."""
import route_settings as routes


class SettingsMenu:
    def __init__(self,folder):
        self.folder=folder;self.open=False;self.cursor=0;self.error=None
    def begin(self):
        self.original=routes.load(self.folder);self.mode=self.original['mode'];self.name=self.original.get('preset','custom')
        if self.name not in ('competition','rectangle','outback','translate','rotate'):self.name='custom'
        moves=[s for s in self.original['steps'] if s['kind']=='move'];turns=[s for s in self.original['steps'] if s['kind']=='turn']
        self.length=moves[0]['distance_m'] if moves else 2;self.width=moves[1]['distance_m'] if len(moves)>1 else 2
        self.speed=self.original['speed_percent'];self.angle=turns[0].get('rotation_deg',abs(turns[0]['heading_deg'])) if turns else 90
        self.open=True;self.cursor=0;self.error=None
    def fields(self):
        if self.name=='custom':return [0,1,4]
        if self.name=='rotate':return [0,1,4,5]
        if self.name=='rectangle':return [0,1,2,3,4]
        if self.name=='competition' and self.mode=='step':return [0,1,2,4,5]
        return [0,1,2,4]
    def next(self):
        options=self.fields();self.cursor=options[(options.index(self.cursor)+1)%len(options)]
    def change(self):
        def after(value,options):return options[(options.index(value)+1)%len(options)] if value in options else options[0]
        if self.cursor==0:
            self.mode=after(self.mode,['step','continuous'])
            if self.mode=='continuous' and self.name in ('translate','rotate'):self.name='competition'
        elif self.cursor==1:
            choices=['competition','rectangle','outback']+(['translate','rotate'] if self.mode=='step' else [])+(['custom'] if self.original.get('preset')=='custom' else [])
            self.name=after(self.name,choices)
            if self.name=='rotate' and self.speed<20:self.speed=20
        elif self.cursor==2:self.length=2.0 if self.length<2 or self.length>=4 else min(4.0,round(self.length+.2,3))
        elif self.cursor==3:self.width=2.0 if self.width<2 or self.width>=4 else min(4.0,round(self.width+.2,3))
        elif self.cursor==4:self.speed=20 if self.speed>=200 else min(200,self.speed+20)
        elif self.cursor==5 and self.mode=='step':self.angle=after(self.angle,[90,180,360,720])
    def confirm(self):
        if self.name=='custom':data=dict(self.original,mode=self.mode,speed_percent=self.speed)
        else:data=routes.preset(self.name,self.length,self.width,self.mode,self.speed,90 if self.mode=='continuous' else self.angle)
        data=routes.save(self.folder,data);self.open=False;return data
    def cancel(self):self.open=False
    def snapshot(self):
        if not self.open:return None
        values=['STEP' if self.mode=='step' else 'CONTINUOUS',self.name.upper(),
            '%.2f m'%self.length if self.name not in ('custom','rotate') else '--',
            '%.2f m'%self.width if self.name=='rectangle' else '--','%d %%'%self.speed,
            ('90 deg FIXED' if self.mode=='continuous' else '%g deg CW'%self.angle) if self.name in ('competition','rotate') else '--']
        numeric={2:(self.length,4.0,'+0.20 m / 4.00 -> 2.00 m'),3:(self.width,4.0,'+0.20 m / 4.00 -> 2.00 m'),
                 4:(self.speed,200,'+20% / 200% -> 20%')}
        for i,(value,limit,hint) in numeric.items():
            if i in self.fields() and value>=limit:values[i]+='  MAX'
        hint=numeric[self.cursor][2] if self.cursor in numeric else ('90 / 180 / 360 / 720 deg' if self.cursor==5 else 'BTN1 SWITCH OPTION')
        return dict(cursor=self.cursor,rows=list(zip(('MODE','ROUTE','LENGTH','WIDTH','TURN SPEED' if self.name=='rotate' else 'SPEED LIMIT','TURN ANGLE'),values)),hint=hint,error=self.error)
