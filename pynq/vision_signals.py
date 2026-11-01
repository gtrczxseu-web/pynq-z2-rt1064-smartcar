"""Fresh card edges for route control; no camera, UART or motor ownership."""

class SignalCards:
    def __init__(self):
        self.tracks={};self.last_stamp=-1e9

    def update(self,stamp,detections):
        if stamp<=self.last_stamp:return set(),None
        self.last_stamp=stamp;labels=set()
        for d in detections:
            parts=d.get('label','').split()
            if parts and parts[0] in ('RED','GREEN','YELLOW'):labels.add(parts[0])
            elif len(parts)>=2 and parts[0]=='DIGIT' and parts[1] in tuple(str(i) for i in range(10)):
                labels.add('DIGIT '+parts[1])
        events=set();stable=set()
        for label,t in self.tracks.items():
            if label not in labels:
                t['count']=0
                if stamp-t['seen']>=.5:t['used']=False
        for label in labels:
            t=self.tracks.setdefault(label,dict(seen=-1e9,count=0,used=False))
            # Each observation is freshness-checked by MotionLink. Allow a slow
            # camera interval without confusing it with an observed disappearance.
            t['count']=t['count']+1 if stamp-t['seen']<=1 else 1;t['seen']=stamp
            if t['count']>=2:
                stable.add(label)
                if not t['used']:events.add(label);t['used']=True
        color=next((c for c in ('RED','YELLOW','GREEN') if c in stable),None)
        return events,color
