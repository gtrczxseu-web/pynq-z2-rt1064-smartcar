"""Minimal next-run recipe editor; saving never sends motor commands."""
import json
from pathlib import Path
import route_settings as routes


def show(folder=None):
    import ipywidgets as w
    from IPython.display import display
    folder=Path(folder or Path(__file__).resolve().parent)
    current=routes.load(folder)
    status=w.HTML()
    choice=w.Dropdown(options=[('原两圈','competition'),('矩形平移','rectangle'),('往返','outback'),('自定义','custom')],description='路线')
    choice.value=current.get('preset') if current.get('preset') in ('competition','rectangle','outback') else 'custom'
    mode=w.Dropdown(options=[('分步暂停','step'),('连续运行','continuous')],value=current['mode'],description='方式')
    moves=[s for s in current['steps'] if s['kind']=='move']
    turns=[s for s in current['steps'] if s['kind']=='turn']
    angle=turns[0].get('rotation_deg',abs(turns[0]['heading_deg'])) if turns and current.get('preset')=='competition' else 90
    length=w.BoundedFloatText(value=moves[0]['distance_m'] if moves else 2,min=.02,max=4,step=.2,description='长度/m')
    width=w.BoundedFloatText(value=moves[1]['distance_m'] if len(moves)>1 else 2,min=.02,max=4,step=.2,description='宽度/m')
    speed=w.IntSlider(value=current['speed_percent'],min=min(20,current['speed_percent']),max=200,step=20,description='速度上限/%')
    editor=w.Textarea(value=json.dumps(current,ensure_ascii=False,indent=2),layout=w.Layout(width='100%',height='240px'))
    details=w.Accordion(children=[editor]);details.set_title(0,'自定义动作：距离、方向、目标航向');details.selected_index=None
    def save_route(unused):
        try:
            data=json.loads(editor.value) if choice.value=='custom' else routes.preset(choice.value,length.value,width.value,
                mode.value,speed.value,90 if mode.value=='continuous' else angle)
            data.update(mode=mode.value,speed_percent=speed.value)
            routes.save(folder,data);status.value='已保存。下一轮 BTN0 启动时使用；不会立即运动。'
        except Exception as exc:status.value='未保存：'+str(exc)
    store=w.Button(description='保存下一轮设置',button_style='success');store.on_click(save_route)
    panel=w.VBox([w.HTML('<h3>运行设置</h3><p>只保存，不启动电机。超过100%的速度需要 V5 车板固件。百分比相对原速度上限，实际车速取决于车辆。</p>'),
        w.HBox([choice,mode]),w.HBox([length,width]),speed,details,store,status,
        w.HTML('<p>镜头朝向是新的前进方向（原车尾成为新车头），地图+Y与此一致。地面分界线沿用原标定。AR APPROX 为近似投影；镜头背后的已走轨迹仅在小地图显示。</p>')])
    display(panel);return panel
