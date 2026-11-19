"""Live gateway monitoring with the original first-tab route image and renderer."""
import json,time
from pathlib import Path
from datetime import datetime
from PyQt5.QtCore import Qt,QThread,QTimer,QProcess,pyqtSignal
from PyQt5.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QLabel,QSplitter,QPlainTextEdit,QPushButton
from PyQt5.QtGui import QFontDatabase,QPixmap
from brain_monitor_client import sample
from route_image_widget import RouteImage
ROOT=Path(__file__).resolve().parents[1]

class ReadWorker(QThread):
    received=pyqtSignal(object)
    def run(self):
        try:self.received.emit(sample())
        except Exception as exc:self.received.emit(dict(error=str(exc)))
class CancelWorker(QThread):
    received=pyqtSignal(object)
    def __init__(self,expected,parent):super().__init__(parent);self.expected=expected
    def run(self):
        try:
            from brain_cancel_client import cancel
            self.received.emit(cancel(self.expected))
        except Exception as exc:self.received.emit(dict(error=str(exc)))

class BrainMonitorPage(QWidget):
    def __init__(self,owner):
        super().__init__();self.owner=owner;self.worker=None;self.cancel_worker=None;self.last_log=None;self.closed=False;self.last_data=None;self.last_error='';self.live_image_key=None;self.fetch_key=None;self.fetch_output=''
        layout=QVBoxLayout(self);top=QHBoxLayout()
        self.summary=QLabel('天机大脑监控 · 等待连接');self.summary.setWordWrap(True);top.addWidget(self.summary,1)
        self.cancel_button=self.owner.stop_real;layout.addLayout(top)
        self.task=QLabel();self.task.setWordWrap(True);layout.addWidget(self.task)
        self.splitter=QSplitter(Qt.Horizontal);self.splitter.setChildrenCollapsible(False);layout.addWidget(self.splitter,1)
        page=QWidget();left=QVBoxLayout(page);left.setContentsMargins(0,0,0,0)
        self.map_note=QLabel('全局规划 · 与第一页使用相同规划图');self.map_note.setWordWrap(True);left.addWidget(self.map_note)
        self.image=RouteImage();self.image.setMinimumSize(280,180);left.addWidget(self.image,1);self.splitter.addWidget(page)
        log_page=QWidget();log_layout=QVBoxLayout(log_page);tools=QHBoxLayout();tools.addWidget(QLabel('大脑任务日志 / 本机导航日志'))
        self.logs=QPlainTextEdit();self.logs.setReadOnly(True);self.logs.setMaximumBlockCount(1200)
        font=QFontDatabase.systemFont(QFontDatabase.FixedFont);font.setPointSize(13);self.logs.setFont(font)
        for label,method in [('放大',self.logs.zoomIn),('缩小',self.logs.zoomOut)]:
            button=QPushButton(label);button.clicked.connect(lambda _,fn=method:fn(1));tools.addWidget(button)
        log_layout.addLayout(tools);log_layout.addWidget(self.logs,1);self.splitter.addWidget(log_page);self.splitter.setSizes([1120,720])
        self.fetch=QProcess(self);self.fetch.setProcessChannelMode(QProcess.MergedChannels);self.fetch.readyReadStandardOutput.connect(self.read_fetch);self.fetch.finished.connect(self.fetched)
        self.timer=QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(2000)
    def showEvent(self,event):super().showEvent(event);self.tick()
    def tick(self):
        if self.closed or not self.isVisible():return
        self.refresh_header()
        if self.worker is not None and self.worker.isRunning():return
        self.worker=ReadWorker(self);self.worker.received.connect(self.render);self.worker.finished.connect(self.worker_finished);self.worker.start()
    def worker_finished(self):
        worker=self.worker;self.worker=None
        if worker is not None:worker.deleteLater()
    def refresh_header(self):
        if self.owner.tabs.currentWidget() is not self:return
        self.owner._monitor_header_active=True;self.owner.stop_real.setText('强制取消当前天机任务')
        if not self.last_data:
            self.owner.status.setText('天机任务：等待网关实时状态');return
        packet=self.last_data['monitor'];task=packet.get('task',{});link=packet.get('link',{});nav=link.get('navigation_status',{});ready=nav.get('readiness',{})
        labels={'running':'执行中','verifying':'核验中','succeeded':'已完成','failed':'失败','cancelled':'已取消','recovery_required':'失败，需处理','waiting_human':'等待处理'}
        state=task.get('state');when=datetime.fromtimestamp(self.last_data.get('task_updated_at',self.last_data.get('received_at',time.time()))).strftime('%H:%M:%S');age=time.time()-self.last_data.get('received_at',0)
        failures=task.get('failure_history')or[];reason=(failures[-1].get('message','')if failures and state in ('failed','recovery_required')else task.get('blocked_reason',''))
        from navigation_outcome import explain
        reason=explain(task)or reason
        text='天机实时任务：'+labels.get(state,str(state or '待命'))+' · 目标：'+str(task.get('goal')or '无')+'号位 · 停止确认：'+str(task.get('stop_confirmation','未知'))+'\n任务ID：'+str(task.get('task_id')or '无')+' · 更新：'+when
        if reason:text+='\n'+str(reason).split('\n',1)[0][:180]
        if self.last_error or age>6:text+='\n状态查询异常/过期：'+(self.last_error or str(round(age,1))+'秒')
        self.owner.status.setText(text)
        blockers=['等待本轮路径守卫检查'if x=='LIVE_ADMISSION_NOT_CONFIRMED'else x for x in nav.get('motion_blockers',[])]
        self.owner.filter_version_status.setText('Jetson→S100：'+('状态已过期'if self.last_error or age>6 else'在线'if link.get('online')else'异常')+' · 机械臂反馈：'+('新鲜'if ready.get('arm_fresh')else'未就绪')+' · 准入：'+('通过'if nav.get('motion_ready')else'；'.join(blockers))+' · 采样：'+when)
    def render_route(self,packet):
        preview=packet.get('link',{}).get('planning_preview')
        if preview:
            key=(preview.get('task_id'),preview.get('generated_at'));folder=preview.get('artifact_folder')
            if self.live_image_key==key:
                self.map_note.setText('当前任务全局规划 · '+str(preview.get('task_id'))+' · '+datetime.fromtimestamp(preview['generated_at']).strftime('%H:%M:%S'));return
            if folder and self.fetch.state()==QProcess.NotRunning and self.fetch_key!=key:
                self.fetch_key=key;self.fetch_output='';self.fetch.setProgram('/usr/bin/python3');self.fetch.setArguments([str(ROOT/'modules/brain_plan_fetch.py'),'--folder',folder]);self.fetch.start()
            self.map_note.setText('正在读取当前规划产物；沿用第一页原渲染方式。下方已有图片为历史规划。')
        else:self.map_note.setText('当前任务尚无规划图；下方为第一页已保存的历史规划图。')
        if not self.owner.image.original.isNull():self.image.setPixmap(self.owner.image.original)
    def read_fetch(self):self.fetch_output+=bytes(self.fetch.readAllStandardOutput()).decode(errors='replace')
    def fetched(self,code,*_):
        self.read_fetch()
        if self.closed:return
        for line in reversed(self.fetch_output.splitlines()):
            if code==0 and line.startswith('GUI_RESULT '):
                folder=Path(line[len('GUI_RESULT '):]);pixmap=QPixmap(str(folder/'route.png'))
                if not pixmap.isNull():
                    self.image.setPixmap(pixmap);self.owner.image.setPixmap(pixmap);self.owner.route_image=folder/'route.png';self.live_image_key=self.fetch_key
                    if self.last_data:self.render_route(self.last_data['monitor'])
                    return
        self.map_note.setText('当前规划图读取失败，保留历史图；详情见日志。');self.owner.log.appendPlainText('[监控规划图]\n'+self.fetch_output[-2500:])
    def render(self,data):
        if self.closed:return
        if 'error'in data:
            self.last_error=data['error'];self.summary.setText('天机网关查询异常：'+self.last_error);self.refresh_header();return
        self.last_data=data;self.last_error='';packet=data['monitor'];link=packet.get('link',{});task=packet.get('task',{})
        self.summary.setText('大脑网关：在线 · Jetson→S100：'+('在线'if link.get('online')else'异常')+' · 耗时：'+str(link.get('latency_ms','?'))+'ms · 每2秒更新')
        self.task.setText('任务：'+str(task.get('task_id')or'无')+' · 状态：'+str(task.get('state',''))+' · 停止确认：'+str(task.get('stop_confirmation','')))
        self.render_route(packet);self.refresh_header()
        text='[大脑网关日志]\n'+'\n'.join(json.dumps(row,ensure_ascii=False)for row in data.get('logs',[]))+'\n\n[本机导航日志]\n'+self.owner.log.toPlainText()[-32000:]
        if text!=self.last_log:self.logs.setPlainText(text);self.logs.verticalScrollBar().setValue(self.logs.verticalScrollBar().maximum());self.last_log=text
    def cancel_task(self):
        if self.cancel_worker is not None and self.cancel_worker.isRunning():return
        expected=(self.last_data or{}).get('monitor',{}).get('task',{}).get('task_id')
        self.cancel_button.setEnabled(False);self.cancel_worker=CancelWorker(expected,self);self.cancel_worker.received.connect(self.cancelled);self.cancel_worker.finished.connect(self.cancel_finished);self.cancel_worker.start();self.owner.status.setText('正在向原天机任务发送取消请求；停止尚未确认。')
    def cancel_finished(self):
        worker=self.cancel_worker;self.cancel_worker=None;self.cancel_button.setEnabled(True)
        if worker is not None:worker.deleteLater()
    def cancelled(self,result):
        if self.closed:return
        if 'error'in result:
            text='取消回执未确认：'+result['error'];self.owner.log.appendPlainText(text);self.summary.setText(text);return
        status=result.get('status',{});text='取消已受理'if result.get('accepted')else'取消未受理：'+str(result.get('reason',''))
        text+=' · 停止确认：'+str(status.get('stop_confirmation','未知'));self.owner.log.appendPlainText(text);self.summary.setText(text)
        if self.last_data:self.last_data['monitor']['task']=status;self.last_data['task_updated_at']=time.time();self.refresh_header()
        self.tick()
    def shutdown(self):
        self.closed=True;self.timer.stop()
        for worker in (self.worker,self.cancel_worker):
            if worker is not None and worker.isRunning():worker.wait(17000)
        if self.fetch.state()!=QProcess.NotRunning:self.fetch.kill();self.fetch.waitForFinished(2000)
