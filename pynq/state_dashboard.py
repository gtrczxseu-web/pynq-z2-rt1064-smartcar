"""Jupyter operation panel; graphics are a separate ARM/browser view, not PL AR."""
import html
import json
import math
import threading
import time
from collections import deque


def state_svg(points, heading=0.0):
    """Fixed calibration axes, auto-scaled metres, bounded display history."""
    points=list(points)
    span=max([0.5]+[abs(v)*1.15 for p in points for v in p])
    scale=120/span
    def screen(p):
        return 170+p[0]*scale,160-p[1]*scale
    poly=' '.join('%.1f,%.1f'%screen(p) for p in points)
    x,y=screen(points[-1] if points else (0,0))
    angle=math.radians(heading)
    ax,ay=x-28*math.sin(angle),y-28*math.cos(angle)
    ticks=''.join('<text x="%d" y="%d" fill="#50627a" font-size="12">%s</text>'%t
                  for t in [(175,30,'+Y'),(305,155,'+X'),(175,177,'0')])
    return '''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 340 360" style="width:100%%;max-width:480px;height:auto" role="img" aria-label="轨迹和相对航向">
    <rect width="340" height="360" rx="12" fill="#f3f7fc"/>
    <defs><marker id="tip" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto"><path d="M0,0 L0,6 L6,3 z" fill="#e87926"/></marker></defs>
    <path d="M40 160 H310 M170 30 V290" stroke="#a2afc0"/>
    <circle cx="170" cy="160" r="3" fill="#50627a"/>%s
    <polyline points="%s" fill="none" stroke="#1d6ca7" stroke-width="2.5"/>
    <circle cx="%.1f" cy="%.1f" r="5" fill="#1d6ca7"/>
    <path d="M%.1f %.1f L%.1f %.1f" stroke="#e87926" stroke-width="3" marker-end="url(#tip)"/>
    <text x="15" y="20" font-size="17" fill="#17324d">相对航向 %.2f° · 逆时针为正</text>
    <text x="15" y="318" font-size="13" fill="#50627a">蓝线：轨迹　橙色：相机相对朝向</text>
    <text x="15" y="337" font-size="12" fill="#50627a">范围 ±%.3f m · 原点：静止校准位置</text>
    <text x="15" y="354" font-size="11" fill="#50627a">最近最多 512 个显示点；完整采集样本保存在 CSV</text></svg>'''%(ticks,poly,x,y,x,y,ax,ay,heading,span)


class StateDashboard:
    def __init__(self, session):
        import ipywidgets as w
        self.session=session
        self.thread=None
        self.stop_event=threading.Event()
        self.points=deque(maxlen=512)
        self.epoch=None
        self.last_key=None
        self.last_result=None
        self.status=w.HTML()
        self.graph=w.HTML(value=state_svg([]))
        self.log=w.Output(layout=w.Layout(max_height='180px',overflow='auto'))
        self.stationary=w.Checkbox(description='确认静止且驱动已关闭',indent=False,
                                   layout=w.Layout(width='auto'))
        self.duration=w.Dropdown(options=[('15 秒',15),('30 秒',30),('60 秒',60),
            ('180 秒',180),('300 秒',300),('600 秒',600)],value=30,description='时长',layout=w.Layout(width='200px'))
        self.camera=w.Checkbox(value=True,description='使用摄像头',indent=False)
        self.buttons={}
        for name,label,callback in [('setup','1 初始化',self._setup),('camera','2 连接摄像头',self._camera),
            ('calibrate','3 静止校准 / 新起点',self._calibrate),('start','4 开始',self._start),
            ('stop','停止',self._stop),('export','导出日志',self._export),('close','结束并释放',self._close)]:
            b=w.Button(description=label,layout=w.Layout(width='auto',min_width='150px',height='36px',flex='1 0 150px'))
            b.on_click(callback)
            self.buttons[name]=b
        self.buttons['stop'].button_style='warning'
        self.buttons['start'].button_style='success'
        note=w.HTML('<b>第五版 · USB 摄像头故障隔离</b><br>上方为操作面板；HDMI 仍由 FPGA 叠加数值。'
            '软件时间配对不代表摄像头曝光硬同步。相机初始右方为 +X，前方为 +Y；相机位置近似取车体中心。')
        def wrap(items):
            return w.Box(items,layout=w.Layout(display='flex',flex_flow='row wrap',width='100%'))
        self.widget=w.VBox([note,self.stationary,wrap([self.duration,self.camera]),
            wrap([self.buttons[k] for k in ('setup','camera','calibrate')]),
            wrap([self.buttons[k] for k in ('start','stop','export','close')]),self.status,self.graph,self.log],
            layout=w.Layout(width='100%'))
        self.refresh('等待初始化')

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def refresh(self,message):
        ready=self.session.ready and not self.session.closed
        active=self.busy() or self.session.running
        for name,b in self.buttons.items():
            b.disabled=active if name!='stop' else not active
        self.buttons['setup'].disabled=active or ready or self.session.closed or self.session.attempted
        for k in ('camera','calibrate','start','export','close'):
            self.buttons[k].disabled=self.buttons[k].disabled or not ready
        self.stationary.disabled=active
        self.duration.disabled=self.camera.disabled=active
        self.status.value='<div style="padding:10px;background:#eaf1f8;border-radius:8px">%s</div>'%html.escape(message)

    def _action(self,action,message):
        if self.busy() or self.session.running:
            return
        try:
            with self.log:
                action()
            self.refresh(message)
        except Exception as exc:
            self.refresh('操作失败：%s'%exc)

    def _setup(self,_):
        self._action(self.session.setup,'初始化完成；连接摄像头，再静止校准')

    def _camera(self,_):
        self._action(lambda:self.session.connect_camera(wait_seconds=10),'摄像头已连接；可进行静止校准')

    def _calibrate(self,_):
        if not self.stationary.value:
            self.refresh('请先确认并勾选：主控上电、驱动板关闭、车体与车轮静止')
            return
        def action():
            import sensor_driver
            self.session._require_idle()
            result=self.session.calibrate(seconds=3)
            self.points.clear()
            self.graph.value=state_svg([])
            self.stationary.value=False
            print('CALIBRATION_READY',json.dumps(result))
        self._action(action,'校准成功，当前位置为新起点；可以开始采集')

    def _start(self,_):
        if self.busy() or self.session.running:
            return
        # Check before spawning, so a failed start reports immediately.
        try:
            self.session._require_idle()
            import sensor_driver
            if not sensor_driver.processor.calibrated:
                raise RuntimeError('请先完成静止校准')
            if self.camera.value and (self.session.cam is None or self.session.cam.closed):
                raise RuntimeError('请先连接摄像头')
        except Exception as exc:
            self.refresh(str(exc))
            return
        self.stop_event.clear()
        seconds,camera=self.duration.value,self.camera.value
        def run():
            try:
                with self.log:
                    self.last_result=self.session.capture(seconds,camera,self.stop_event,self.update)
                passed=self.last_result['software_check_pass']
                message='本轮已停止；检查通过' if passed else '本轮已停止；检查未通过，请导出日志'
                if self.last_result.get('camera_cancelled'):
                    message='本轮已停止；USB 等待已取消，请重新连接摄像头后再采集'
                elif self.last_result.get('camera_degraded'):
                    message='本轮已停止；摄像头异常记录已保存，请重新连接摄像头'
            except Exception as exc:
                message='采集失败：%s'%exc
            # The thread is still alive here, so finish state updates after exit.
            self.thread=None
            self.refresh(message)
        self.thread=threading.Thread(target=run,name='dashboard-capture',daemon=True)
        self.thread.start()
        if self.busy():
            self.refresh('正在采集；可点“停止”，请勿关闭内核')

    def _stop(self,_):
        self.stop_event.set()
        self.status.value='正在停止，等待释放采集线程……'

    def _export(self,_):
        def action():
            from IPython.display import display,FileLink
            path=self.session.export_logs()
            display(FileLink(str(path.relative_to(self.session.folder))))
        self._action(action,'日志已打包；点击输出区链接下载')

    def _close(self,_):
        self._action(self.session.close,'已释放摄像头和 HDMI；再次使用请重启内核')

    def update(self,snapshot):
        sample=snapshot.get('latest_observed') or snapshot.get('latest')
        if sample is None:
            self.status.value='等待 RAW33 数据；请检查车板电源与 UART'
            return
        key=(sample.get('pose_epoch'),sample.get('pl_samples'))
        if key!=self.last_key and sample['valid']:
            if sample.get('pose_epoch')!=self.epoch:
                self.points.clear()
                self.epoch=sample.get('pose_epoch')
            self.points.append((sample['x_m'],sample['y_m']))
            self.last_key=key
            self.graph.value=state_svg(self.points,sample['heading_deg'])
        age=max(0,time.monotonic()-sample['received_monotonic'])
        alignment=snapshot.get('last_alignment') or {}
        warnings=[]
        if snapshot.get('camera_degraded'):
            warnings.append('摄像头异常，已切换色条；本轮停止后可重新连接')
        if age>0.3:
            warnings.append('遥测过期')
        if not sample['valid']:
            warnings.append('轨迹不连续，请停止并重新校准')
        if any(snapshot['uart'][k] for k in ('overrun','frame_error','parity_error','receive_buffer_drops')):
            warnings.append('UART 错误')
        if any(snapshot['decoder'][k] for k in ('crc_errors','missing_frames','skipped_bytes')):
            warnings.append('协议缺帧或校验错误')
        message='；'.join(warnings) if warnings else '实时采集中'
        self.status.value=('<div style="padding:10px;background:%s;border-radius:8px">%s | %.2f km/h | 路程 %.3f m | '
            'X %.3f / Y %.3f m<br>遥测年龄 %.0f ms | 帧配对 %s | RAW33 %d | 缺帧 %d</div>')%(
                '#fff0df' if warnings else '#e5f3eb',html.escape(message),sample['speed_km_h'],sample['distance_m'],
                sample['x_m'],sample['y_m'],age*1000,html.escape(alignment.get('status','无摄像头')),
                snapshot['decoder']['raw_frames'],snapshot['decoder']['missing_frames'])
