"""Navigation configuration and planning GUI. No motion transport exists here."""
import os,sys,json,math
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
from paths import ROOT,NAV
import targets
from display_map_path import local_map
import preview_options
import task_backend
from release_status import explanation,arrival_text
sys.path.insert(0,str(NAV/'modules'))
from map_gui import Window as MapWindow
from PyQt5.QtCore import Qt,QProcess,QSettings,QTimer
from PyQt5.QtGui import QSurfaceFormat,QPixmap,QFontDatabase
from PyQt5.QtWidgets import QApplication,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QComboBox,QPushButton,QPlainTextEdit,QCheckBox,QDoubleSpinBox,QGridLayout,QScrollArea,QSizePolicy,QTabWidget,QSplitter,QGroupBox

from route_image_widget import RouteImage

class NavigationWindow(MapWindow):
 def __init__(self):
  self.closing=False
  saved_scene=ROOT/'runtime/gui_scene.json'
  try:selected=json.loads(saved_scene.read_text())['scene_id'] if saved_scene.exists() else 'office'
  except (OSError,ValueError,KeyError):selected='office'
  targets.select_scene(selected);self.goals=targets.load();super().__init__(local_map(self.goals['targets']['1']));self.setWindowTitle('S100导航控制');self.resize(1880,1000)
  self.setStyleSheet('QPushButton{min-height:30px;padding:3px 8px} QComboBox,QDoubleSpinBox{min-height:30px}')
  map_panel=self.takeCentralWidget()
  # The inherited point-cloud widget forced a 550px minimum even on hidden
  # tabs, making the whole console taller than the 1080p available desktop.
  self.scene.setMinimumSize(480,300)
  root=QWidget();self.setCentralWidget(root);outer=QVBoxLayout(root);outer.setContentsMargins(10,8,10,8)
  header=QHBoxLayout();self.status=QLabel('待选择目标');self.status.setWordWrap(True)
  header.addWidget(self.status,1)
  self.stop_real=QPushButton('停止底盘导航');self.stop_real.setStyleSheet('background:#b42323;color:white;padding:9px 18px;font-weight:bold');self.stop_real.clicked.connect(self.stop_navigation);header.addWidget(self.stop_real)
  outer.addLayout(header)
  self.filter_version_status=QLabel('点云滤除：等待S100确认配置');self.filter_version_status.setWordWrap(True);outer.addWidget(self.filter_version_status)
  self.tabs=QTabWidget();self.tabs.setDocumentMode(True);outer.addWidget(self.tabs,1)
  self.tabs.setStyleSheet('QTabBar::tab{padding:9px 22px;font-weight:bold}')
  panel=QWidget();scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(panel)
  scroll.setMinimumWidth(420)
  self.dashboard=QSplitter(Qt.Horizontal)
  self.dashboard.addWidget(scroll)
  self.detail_splitter=QSplitter(Qt.Vertical)
  route_page=QWidget();route_layout=QVBoxLayout(route_page)
  route_layout.setContentsMargins(6,4,6,4);route_layout.addWidget(QLabel('规划路径'))
  self.image=RouteImage();self.image.setMinimumSize(280,180);route_layout.addWidget(self.image,1)
  self.detail_splitter.addWidget(route_page)
  log_page=QWidget();log_layout=QVBoxLayout(log_page);log_layout.setContentsMargins(6,4,6,4)
  self.log=QPlainTextEdit();self.log.setReadOnly(True);self.log.setMaximumBlockCount(2000)
  log_font=QFontDatabase.systemFont(QFontDatabase.FixedFont)
  log_font.setPointSize(QApplication.font().pointSize()+1);self.log.setFont(log_font)
  log_tools=QHBoxLayout();log_tools.addWidget(QLabel('导航日志'))
  for label,handler in [('放大字体',self.log.zoomIn),('缩小字体',self.log.zoomOut)]:
   button=QPushButton(label);button.clicked.connect(lambda _,fn=handler:fn(2));log_tools.addWidget(button)
  log_tools.addStretch();log_layout.addLayout(log_tools);log_layout.addWidget(self.log,1)
  self.detail_splitter.addWidget(log_page)
  self.dashboard.addWidget(self.detail_splitter)
  self.dashboard.setStretchFactor(0,0);self.dashboard.setStretchFactor(1,1)
  self.dashboard.setChildrenCollapsible(False);self.detail_splitter.setChildrenCollapsible(False)
  self.dashboard.setSizes([540,1250]);self.detail_splitter.setSizes([500,350])
  self.tabs.addTab(self.dashboard,'导航总览')
  self.tabs.addTab(map_panel,'三维点云')
  from brain_monitor_page import BrainMonitorPage
  self.brain_monitor=BrainMonitorPage(self);self.tabs.addTab(self.brain_monitor,'大脑与导航监控')
  control=QVBoxLayout(panel);control.setContentsMargins(12,10,12,10);control.setSpacing(10)
  self.advanced=QGroupBox('高级设置与滤除状态');self.advanced.setCheckable(True);self.advanced.setChecked(False)
  advanced_layout=QVBoxLayout(self.advanced)
  self.advanced_content=QWidget();settings=QVBoxLayout(self.advanced_content)
  advanced_layout.addWidget(self.advanced_content);self.advanced_content.hide()
  self.advanced.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Maximum)
  self.advanced.toggled.connect(self.advanced_content.setVisible)
  layout=control
  layout.addWidget(QLabel('导航场景（地图和点位一起切换）'))
  self.scene_choice=QComboBox()
  for key,name in targets.SCENES.items():self.scene_choice.addItem(name,key)
  self.scene_choice.setCurrentIndex(self.scene_choice.findData(targets.scene_id()));layout.addWidget(self.scene_choice)
  self.backend_choice=QComboBox();self.backend_choice.addItem('CLI调试（现有入口）','legacy');self.backend_choice.addItem('S100任务API（与Agent共用）','api');self.backend_choice.addItem('模块化总线（重构版测试）','bus');self.backend_choice.setCurrentIndex(self.backend_choice.findData(task_backend.load()['backend']));layout.addWidget(self.backend_choice)
  self.backend_choice.currentIndexChanged.connect(self.change_task_backend)
  self.scene_note=QLabel('');self.scene_note.setWordWrap(True);layout.addWidget(self.scene_note)
  self.choice=QComboBox()
  for row in self.goals['targets'].values():self.choice.addItem(f"{row['name']}  X {row['pose'][0]:.3f}, Y {row['pose'][1]:.3f} m",row['id'])
  layout.addWidget(self.choice)
  cfg=json.loads((ROOT/'config/motion_policy.json').read_text())
  text=QLabel('S100静态地图规划 · 实时障碍停车等待，净空后继续')
  text.setWordWrap(True);layout.addWidget(text)
  speed_row=QHBoxLayout();speed_row.addWidget(QLabel('直线平移速度上限'))
  self.speed_settings=QSettings('Tianji','S100Navigation')
  self.speed=QDoubleSpinBox();self.speed.setRange(.5,10.);self.speed.setDecimals(1);self.speed.setSingleStep(.5);self.speed.setSuffix(' cm/s')
  self.load_speed();self.speed.valueChanged.connect(self.save_speed);speed_row.addWidget(self.speed);speed_row.addStretch();layout.addLayout(speed_row)
  speed_note=QLabel('按场景和点位保存，下次实机启动或并行预览生效。转向速度按原控制配置。');speed_note.setWordWrap(True);layout.addWidget(speed_note)
  self.cable_precheck=QCheckBox();self.cable_precheck.setChecked(False);self.cable_precheck.hide()
  layout=settings
  self.cleanup_status=QLabel('近体滤除：等待S100确认');self.cleanup_status.setWordWrap(True);layout.addWidget(self.cleanup_status)
  fine=cfg.get('fine_goal',{})
  if fine.get('enabled'):
   label=QLabel(f"终点微调：位置≤{fine['position_tolerance_m']*100:g} cm、朝向≤{fine['heading_tolerance_deg']:g}°")
   label.setWordWrap(True);layout.addWidget(label)
  self.floor_enabled=QCheckBox('手动补地面（青色，仅补二维候选区）');layout.addWidget(self.floor_enabled)
  fields=QGridLayout();self.floor_bounds=[]
  for i,(label,value) in enumerate(zip(['X最小','X最大','Y最小','Y最大'],[-1.55,3.,-3.25,1.65])):
   fields.addWidget(QLabel(label+' / m'),i//2,2*(i%2));spin=QDoubleSpinBox();spin.setRange(-100,100);spin.setDecimals(2);spin.setValue(value);fields.addWidget(spin,i//2,2*(i%2)+1);self.floor_bounds.append(spin)
  layout.addLayout(fields);self.floor_apply=QPushButton('保存补地面区域到S100');self.floor_apply.clicked.connect(self.save_floor);layout.addWidget(self.floor_apply)
  layout.addWidget(QLabel('只读路径规划开关（不改变实机导航联锁）'))
  self.preview_camera=QCheckBox('规划时忽略头部相机 / ShapeMask');self.preview_grippers=QCheckBox('夹爪按开启状态规划（关闭则读实测）')
  self.preview_saved_arms=QCheckBox('仅规划预览：忽略双臂实时反馈，使用已保存姿态')
  options=preview_options.load();self.preview_saved_arms.setChecked(options['use_saved_arms']);self.preview_camera.setChecked(options['ignore_head_camera']);self.preview_grippers.setChecked(options['open_grippers'])
  self.preview_saved_arms.toggled.connect(self.save_preview_options);layout.addWidget(self.preview_saved_arms);self.preview_camera.toggled.connect(self.save_preview_options);self.preview_grippers.toggled.connect(self.save_preview_options);layout.addWidget(self.preview_camera);layout.addWidget(self.preview_grippers)
  self.preview_note=QLabel(preview_options.caption(options));self.preview_note.setWordWrap(True);layout.addWidget(self.preview_note)
  settings.addStretch();layout=control
  self.start=QPushButton('规划当前点位');self.start.clicked.connect(self.plan)
  self.plan_both=QPushButton('预览 1 / 2 号路径');self.plan_both.clicked.connect(lambda:self.onboard_task('--plan-both'))
  self.open_route=QPushButton('查看规划图');self.open_route.clicked.connect(self.show_route_image)
  live=QPushButton('实时拦截点诊断');live.clicked.connect(lambda:QProcess.startDetached('/usr/bin/python3',[str(ROOT/'modules/live_guard_gui.py')]))
  self.cancel=QPushButton('取消当前操作');self.cancel.setEnabled(False);self.cancel.clicked.connect(self.stop_plan)
  check=QPushButton('检查S100导航状态');check.clicked.connect(lambda:self.onboard_task('--check'))
  dry=QPushButton('并行预览（不移动）');dry.clicked.connect(lambda:self.onboard_task('--execute-preview'))
  right_micro=QPushButton('向右微移预检');right_micro.clicked.connect(lambda:self.onboard_task('--preview-right-micro'))
  self.force_release=QCheckBox('测试：净空/ShapeMask强制放行');self.force_release.toggled.connect(self.set_forced_release);layout.addWidget(self.force_release)
  self.actual=QPushButton('启动实机导航');self.actual.clicked.connect(self.start_real)
  actions=QGridLayout()
  for index,button in enumerate((self.start,self.actual,self.plan_both,dry,self.open_route,check,right_micro,live,self.cancel)):
   actions.addWidget(button,index//2,index%2)
  layout.addLayout(actions)
  reason=QLabel('实车启动会重新规划。实时数据过期或碰撞拦截时，底盘停车等待复检。')
  reason.setWordWrap(True);layout.addWidget(reason)
  layout.addWidget(self.advanced);layout.addStretch()
  self.process=QProcess(self)
  self.process.stateChanged.connect(lambda state:self.backend_choice.setEnabled(state==QProcess.NotRunning));self.process.setProcessChannelMode(QProcess.MergedChannels);self.process.readyReadStandardOutput.connect(self.read_output);self.process.finished.connect(self.finished);self.process.errorOccurred.connect(lambda *_:self.finished(-1))
  self.once_autostart_pending=False;self.once_execution_seconds=None;self.previous_folder=None;self.route_image=None;self.action='plan';self.scene_choice.currentIndexChanged.connect(self.switch_scene);self.choice.currentIndexChanged.connect(self.goal_changed);self.update_scene_note();self.load_floor();self.show_latest();self.execution_timer=QTimer(self);self.execution_timer.timeout.connect(self.refresh_execution);self.execution_timer.start(250);QTimer.singleShot(800,lambda:self.onboard_task('--check'))
 def speed_key(self):
  return f'translation_speed/{targets.scene_id()}/{self.choice.currentData()}'
 def load_speed(self):
  default=10.
  self.speed.blockSignals(True);self.speed.setValue(self.speed_settings.value(self.speed_key(),default,type=float));self.speed.blockSignals(False)
 def save_speed(self,value):
  self.speed_settings.setValue(self.speed_key(),value)
 def speed_args(self):
  return ['--translation-cap-mps',f'{self.speed.value()/100:.3f}']
 def show_latest(self):
  try:
   cache=ROOT/'runtime'/f'latest_plan_{targets.scene_id()}_{self.choice.currentData()}.json';pointer=json.loads((cache if cache.exists() else ROOT/'runtime/latest_plan.json').read_text());folder=Path(pointer['folder']);result=json.loads((folder/'planning/plan.json').read_text());goal=result['target']
   if goal['id']!=self.choice.currentData() or not targets.unchanged(goal):return
   self.route_image=folder/'route.png'
   from common import atomic_json
   atomic_json(ROOT/'runtime/latest_plan.json',pointer)
   self.image.setPixmap(QPixmap(str(folder/'route.png')));selected=result.get('selected')
   self.status.setText(f"{targets.SCENES[targets.scene_id()]} · 保存的规划快照 → {goal['name']}\n"+(f"路径 {selected['length_m']:.2f}m，横移 {selected['lateral_distance_m']:.2f}m；三维碰撞复检通过。" if selected else ('起步候选动作全部被碰撞/未知区域拒绝；底盘未启动。' if result['attempts'] and all(r.get('expanded_nodes',99)<=1 for r in result['attempts']) else '无合规路径：'+str([r.get('reason') for r in result['attempts']])))+'\n保存结果不代表当前可执行。')
   if result.get('cable_precheck') and selected:self.status.setText(f"线缆待验证的候选路线 → {goal['name']}\n路长 {selected['length_m']:.2f}m；暂缓疑似线缆 {result['cable_precheck']['count']} 点。\n实机先局部检查和微移5cm；检测后再复检剩余路线。")
   if result.get('intent_prefix_only') and selected:self.status.setText('仅用于5cm检测的意图轨迹；终点/剩余路径尚未通过真实点云检查。\n先检查局部起步段，满5cm后再判断线材并重新规划。')
   if result.get('preview_posture_assumptions'):self.status.setText(self.status.text()+'\n保存结果采用：'+preview_options.caption(result.get('preview_options',dict(ignore_head_camera=True,open_grippers=True)))+'；头部实体保持URDF碰撞范围。')
   if result.get('frozen_frame_preview'):self.status.setText(self.status.text()+'\n历史冻结帧预览，不代表当前姿态或当前净空。')
   report=folder/'scene/near_body_cleanup.json'
   if report.exists():self.show_cleanup_report(json.loads(report.read_text()))
  except (OSError,ValueError,KeyError):pass
 def save_preview_options(self,*_):
  options=preview_options.save(dict(ignore_head_camera=self.preview_camera.isChecked(),open_grippers=self.preview_grippers.isChecked(),use_saved_arms=self.preview_saved_arms.isChecked()));self.preview_note.setText(preview_options.caption(options)+'；设置已保存，下次只读规划生效。')
 def select_map(self,index):
  if hasattr(self,'scene_choice') and 0<=index<len(self.paths):
   selected=self.paths[index].resolve()
   for key in targets.SCENES:
    row=json.loads(targets.scene_file(key).read_text())['targets']['1']
    if local_map(row).resolve()==selected:
     if key!=targets.scene_id():self.scene_choice.setCurrentIndex(self.scene_choice.findData(key));return
  super().select_map(index)
 def update_scene_note(self):
  note='办公室：原1/2号位及新记录3号位；1号位是沿地图Y轴平移保存的参考点。' if targets.scene_id()=='office' else '外侧：使用刚刚导航成功的地图和1/2号位。'
  self.scene_note.setToolTip(note)
  self.scene_note.setText('办公室 · 10月28日清理地图 · 独立1/2/3号点位' if targets.scene_id()=='office' else '外侧 · 清理地图 · 独立1/2号点位')
 def goal_changed(self,*_):
  self.load_speed();self.route_image=None;self.image.clear();self.status.setText('选择点位后可只读规划；尚未生成此场景下的新路径。');self.show_latest()
 def switch_scene(self,*_):
  if self.process.state()!=QProcess.NotRunning:return
  targets.select_scene(self.scene_choice.currentData());self.goals=targets.load();self.choice.blockSignals(True);self.choice.clear()
  for row in self.goals['targets'].values():self.choice.addItem(f"{row['name']}  X {row['pose'][0]:.3f}, Y {row['pose'][1]:.3f} m",row['id'])
  self.choice.blockSignals(False);self.previous_folder=None;self.route_image=None;self.image.clear();self.log.clear()
  folder=local_map(self.goals['targets']['1']);index=self.maps.findData(str(folder));self.maps.blockSignals(True);self.maps.setCurrentIndex(index);self.maps.blockSignals(False);self.load(folder)
  from common import atomic_json
  atomic_json(ROOT/'runtime/gui_scene.json',dict(scene_id=targets.scene_id()));self.update_scene_note();self.goal_changed();self.onboard_task('--check')
 def show_route_image(self):
  self.tabs.setCurrentIndex(0);self.image.setFocus()
 def change_task_backend(self):
  value=task_backend.save(self.backend_choice.currentData());self.status.setText('任务后端已切换：'+{'api':'S100任务API。','bus':'模块化总线与重构执行器；点击按钮后才启动。','legacy':'现有CLI调试入口。'}[value])
 def show_cleanup_report(self,report,verified=False):
  cfg=report.get('configuration',{});details=report.get('voxel_report',{})
  state='S100已确认启用' if verified and cfg.get('enabled') else '保存的滤除快照' if cfg.get('enabled') else '未启用'
  if verified and cfg.get('enabled') and cfg.get('model_sha256')!=report.get('model_hash'):state='模型不匹配，滤除未生效'
  text='近体滤除：'+state+'；5cm步骤：'+('已取消' if cfg.get('micro_probe_enabled') is False else '状态未确认')
  if details:text+=f"\n最近规划：额外滤除 {details.get('filtered_points',0)} 点，连续物体保护 {details.get('continuous_points',0)} 点（静态快照）。"
  self.cleanup_status.setText(text)
 def plan(self):
  if self.process.state()!=QProcess.NotRunning:return
  self.action='plan';self.actual.setEnabled(False);self.force_release.setEnabled(False);self.start.setEnabled(False);self.cancel.setEnabled(True);self.choice.setEnabled(False);self.log.clear();self.status.setText('正在只读采集雷达、重新定位并规划…');self.image.clear()
  arguments=[str(ROOT/'modules/onboard_client.py'),'--scene',targets.scene_id(),'--goal',str(self.choice.currentData())]
  if self.cable_precheck.isChecked():arguments+=['--plan-cable']
  self.process.setProgram('/usr/bin/python3');self.process.setArguments(arguments);self.process.start()
 def onboard_task(self,option):
  if self.closing:return
  if self.process.state()!=QProcess.NotRunning:return
  self.action=option;self.actual.setEnabled(False);self.force_release.setEnabled(False);self.start.setEnabled(False);self.cancel.setEnabled(True);self.log.clear();self.status.setText('S100正在检查/联调，运动输出保持禁用。')
  arguments=[str(ROOT/'modules/onboard_client.py'),'--scene',targets.scene_id(),option]
  if option=='--plan-both':self.choice.setEnabled(False)
  if option=='--execute-preview':arguments+=['--goal',str(self.choice.currentData()),*self.speed_args()];self.choice.setEnabled(False)
  self.process.setProgram('/usr/bin/python3');self.process.setArguments(arguments);self.process.start()
 def start_real(self):
  if self.process.state()!=QProcess.NotRunning:return
  run_seconds=self.once_execution_seconds;self.once_execution_seconds=None
  self.action='--execute-cable' if self.cable_precheck.isChecked() else '--execute';self.actual.setEnabled(False);self.start.setEnabled(False);self.force_release.setEnabled(False);self.choice.setEnabled(False);self.cancel.setEnabled(True);self.log.clear();self.image.clear();self.status.setText('你已启动：S100正在生成候选轨迹；微移检测后复检剩余路径。' if self.action=='--execute-cable' else '你已启动：S100正在重新定位和规划；尚未输出速度。')
  self.process.setProgram('/usr/bin/python3');self.process.setArguments([str(ROOT/'modules/onboard_client.py'),'--scene',targets.scene_id(),self.action,'--goal',str(self.choice.currentData()),*self.speed_args()]+(['--execution-seconds',str(run_seconds)]if run_seconds is not None else []));self.process.start()
 def stop_navigation(self):
  if self.tabs.currentWidget() is self.brain_monitor:
   self.brain_monitor.cancel_task();return
  self.once_autostart_pending=False
  self.stop_process=QProcess(self);self.stop_process.setProgram('/usr/bin/python3');self.stop_process.setArguments([str(ROOT/'modules/onboard_client.py'),'--scene',targets.scene_id(),'--stop']);self.stop_process.start()
  if self.action in ['--execute','--execute-cable'] and self.process.state()!=QProcess.NotRunning:self.process.terminate()
  self.status.setText('已请求S100停车，执行会话将退出。')
 def refresh_execution(self):
  if self.tabs.currentWidget() is self.brain_monitor:
   self.brain_monitor.refresh_header();return
  self.stop_real.setText('停止底盘导航')
  if getattr(self,'_monitor_header_active',False):
   self._monitor_header_active=False
   if self.process.state()==QProcess.NotRunning:self.show_latest()
  try:
   from filter_status_text import status_text
   import time
   def read_status(name):
    try:return json.loads((ROOT/'runtime'/name).read_text())
    except (OSError,ValueError):return {}
   self.filter_version_status.setText(status_text(read_status('onboard_readiness.json'),read_status('onboard_execution_status.json'),time.time()))
  except (ValueError,KeyError,TypeError):self.filter_version_status.setText('点云滤除：状态不可用')
  busy=self.process.state()!=QProcess.NotRunning
  self.backend_choice.setEnabled(not busy)
  self.speed.setEnabled(not busy);self.scene_choice.setEnabled(not busy);self.plan_both.setEnabled(not busy);self.maps.setEnabled(not busy);self.preview_camera.setEnabled(not busy);self.preview_grippers.setEnabled(not busy);self.preview_saved_arms.setEnabled(not busy)
  for widget in [self.floor_enabled,self.floor_apply,self.cable_precheck,*self.floor_bounds]:widget.setEnabled(not busy)
  if self.action not in ['--execute','--execute-cable','--execute-preview'] or not busy:return
  try:
   d=json.loads((ROOT/'runtime/onboard_execution_status.json').read_text())
   if __import__('time').time()-d['updated_wall_time']>2:return
   names={'WAITING_GUARDS':'等待检查放行，输出为零','FOLLOWING':'S100导航执行中','CABLE_PROBE_MOVING':'沿候选轨迹微移5cm，采集原始线缆点云','MICRO_STOPPED_SAMPLING':'已停止微移，比较随车/固定环境证据','CABLE_PRECHECK_COMPLETE':'线缆检测完成，复检剩余路线','CABLE_EVIDENCE_UNRESOLVED':'线缆证据不足，保持停车','LOCALIZING_AND_PLANNING':'正在定位和规划，未输出速度','ARRIVED':'到达目标，已请求停车','STOP_GUARD_LOST':'检查失效，停车退出','STOP_OPERATOR_LINK':'GUI连接过期，停车退出','PLAN_BLOCKED':'候选路径未找到，未启动底盘'}
   names['CABLE_PRECHECK_FOUND_STATIC_OBSTACLE']='已完成5cm，检测到固定环境点，保持停车'
   names.update(PREVIEW_READY='零输出预览：并行跟踪与碰撞复核中',TEST_COMPLETE='零输出预览完成',WAIT_CONTROL_READY='等待速度桥就绪，输出为零',ARMING_GUARDS='等待连续新数据复检，输出为零',PAUSED_FOR_FRESH_DATA='数据短暂过期，零速度等待新数据后复检')
   names.update(WAITING_FOR_OBSTACLE_CLEAR='障碍拦截，已停车；等待净空后自动继续',CONFIRMING_OBSTACLE_CLEAR='已检测净空，停车确认稳定后继续原任务')
   names.update(FINE_ADJUSTING='终点微调：低速修正朝向与XY位置',GOAL_SETTLING='终点误差达标，停车确认稳定后完成')
   names.update(FINE_OBSERVING='终点微调：停车观察新读数，暂不修正',FINE_ADJUSTMENT_PAUSED='终点微调已暂停，保持停车；请查看原因后重新启动')
   names.update(WAITING_FOR_SENSOR_RECOVERY='数据恢复等待中：保持停车，任务保留',CONFIRMING_SENSOR_RECOVERY='连续新数据复检中：保持停车，通过后继续')
   names.update(PREPARING_CONTROLLER_EXTENSION='核验斜向平移控制器与驻车状态，尚未输出速度',STOP_TEST_DURATION='单次实机测试到时，已停车',STOP_CONTROLLER_EXTENSION='控制器进程异常，已请求停车')
   names.update(WAITING_FOR_POSE_CERTIFICATE='运动位姿复核中，保持停车；通过后继续原任务')
   self.status.setText(names.get(d['phase'],d['phase'])+'\n'+explanation(d)+('\n已确认线缆特征：'+str(d['confirmed_cable_features']) if 'confirmed_cable_features' in d else ''))
   guard=d.get('guard') or {}
   if guard.get('perception_mode')=='BACKGROUND_SCAN_FILTER_AND_COLLISION':
    counts=d.get('command_gate_stats',{});n=counts.get('output_nonzero',0);total=n+counts.get('output_zero',0)
    kind='模拟门控' if self.action=='--execute-preview' else '速度门控'
    self.cleanup_status.setText(f'前后雷达独立后台处理；本轮导航计算 {d.get("loop_compute_s",0)*1000:.0f}ms。\n{kind}：非零{n}/{total}包（含启动等待）；5cm步骤已取消。')
   pending=d.get('probe_wire_deferral')
   if pending:self.status.setText(self.status.text()+f"\n线材候选区域已固定；当前暂缓 {pending['temporarily_deferred_points']} 点，满5cm后判定。")
   pointer=json.loads((ROOT/'runtime/latest_plan.json').read_text());folder=pointer['folder']
   if folder!=getattr(self,'last_execution_image',None):
    self.last_execution_image=folder;self.image.setPixmap(QPixmap(str(Path(folder)/'route.png')))
  except (OSError,ValueError,KeyError):pass
 def set_forced_release(self,checked):
  if self.process.state()!=QProcess.NotRunning:return
  self.action='--force-release';self.actual.setEnabled(False);self.force_release.setEnabled(False);self.start.setEnabled(False);self.cancel.setEnabled(True);self.log.clear()
  self.process.setProgram('/usr/bin/python3');self.process.setArguments([str(ROOT/'modules/onboard_client.py'),'--scene',targets.scene_id(),'--force-release','on' if checked else 'off']);self.process.start()
 def read_output(self):self.log.appendPlainText(bytes(self.process.readAllStandardOutput()).decode(errors='replace').rstrip())
 def finished(self,code,*_):
  if self.closing:return
  self.read_output();self.actual.setEnabled(True);self.force_release.setEnabled(True);self.start.setEnabled(True);self.cancel.setEnabled(False);self.choice.setEnabled(True)
  if code==0:
   if self.action=='--check':
    self.load_floor();self.show_latest()
    try:
     d=json.loads((ROOT/'runtime/onboard_readiness.json').read_text());self.force_release.blockSignals(True);self.force_release.setChecked(d.get('forced_release_active',False));self.force_release.blockSignals(False);self.status.setText('计算主机：S100\n关节接收：'+('正常' if d.get('arm_fresh') else '失效')+'；头部反馈：'+('正常' if d.get('head_fresh') else '无效')+'\n相机净空：'+('通过' if d.get('clearance_clear') else '未通过')+('（测试强制放行；原始状态保留）' if d.get('forced_release_active') else '')+'；实机未启动（需点击启动按钮）。')
     if d.get('fixed_gripper_policy'):self.status.setText(self.status.text()+'\n夹爪几何：固定开启（未接电机，使用配置值）')
     report=dict(d.get('cleanup_last_plan',{}));report.update(configuration=d.get('near_body_cleanup',{}),model_hash=d.get('model_hash'));self.show_cleanup_report(report,verified=True)
     try:
      last=json.loads((ROOT/'runtime/onboard_execution_status.json').read_text());message=arrival_text(last)
      if message:self.status.setText(self.status.text()+'\n最近记录：'+message+'\n'+explanation(last))
     except (OSError,ValueError):pass
    except (OSError,ValueError):pass
   elif self.action in ['--execute-preview','--preview-right-micro']:
    try:
     d=json.loads((ROOT/'runtime/onboard_execution_status.json').read_text());self.status.setText((arrival_text(d) or ('并行预览完成（零运动输出）。' if d.get('phase')=='TEST_COMPLETE' else '并行预览结束，请查看拦截原因（零运动输出）。'))+'\n'+explanation(d))
    except (OSError,ValueError):self.status.setText('零运动输出预览已结束，详情见日志。')
   elif self.action in ['--execute','--execute-cable']:
    try:
     d=json.loads((ROOT/'runtime/onboard_execution_status.json').read_text());self.status.setText(('导航任务完成：已到达目标，停车流程已结束。' if d.get('phase')=='ARRIVED' and d.get('finished') else '路径检查未通过，底盘未启动。' if d.get('phase')=='PLAN_BLOCKED' else '起步检查未能持续满足，底盘未启动。' if d.get('phase')=='STOP_GUARDS_NOT_READY' else '已完成5cm，检测到固定环境点，保持停车。' if d.get('phase')=='CABLE_PRECHECK_FOUND_STATIC_OBSTACLE' else '已完成5cm，线缆证据不足，保持停车。' if d.get('phase')=='CABLE_EVIDENCE_UNRESOLVED' else 'S100执行会话已结束。')+'\n'+explanation(d))
    except (OSError,ValueError):self.status.setText('执行会话已结束，详情见日志。')
   elif self.action=='--force-release':QTimer.singleShot(0,lambda:self.onboard_task('--check'))
   elif self.action=='--manual-floor':
    self.load_floor();self.status.setText('补地面设置已保存到S100；请重新规划。原占据点和三维碰撞检查保留。')
   elif self.action=='--pipeline':self.status.setText('S100定位、跟踪和实时碰撞只读联调已完成，未发送运动指令。详情见下方日志。')
   else:self.show_latest()
  else:
   self.status.setText('规划未完成或已取消；请查看日志。')
   if self.action=='--check':self.cleanup_status.setText('近体滤除：无法向S100确认，请检查连接。')
 def stop_plan(self):
  self.once_autostart_pending=False
  if self.action in ['--execute','--execute-cable']:self.stop_navigation()
  if self.process.state()!=QProcess.NotRunning:self.process.terminate();QTimer.singleShot(2000,lambda:self.process.kill() if self.process.state()!=QProcess.NotRunning else None)
 def closeEvent(self,event):
  self.closing=True
  self.brain_monitor.shutdown()
  if self.action in ['--execute','--execute-cable']:self.stop_navigation()
  if self.process.state()!=QProcess.NotRunning:
   self.process.terminate()
   if not self.process.waitForFinished(2000):self.process.kill();self.process.waitForFinished(1000)
  super().closeEvent(event)
 def save_floor(self):
  if self.process.state()!=QProcess.NotRunning:return
  self.action='--manual-floor';self.actual.setEnabled(False);self.start.setEnabled(False);self.cancel.setEnabled(True)
  self.process.setProgram('/usr/bin/python3');self.process.setArguments([str(ROOT/'modules/onboard_client.py'),'--scene',targets.scene_id(),'--manual-floor','on' if self.floor_enabled.isChecked() else 'off','--floor-bounds',*[str(w.value()) for w in self.floor_bounds]]);self.process.start()
 def load_floor(self):
  try:
   cfg=json.loads((ROOT/'runtime/manual_floor.json').read_text());self.floor_enabled.setChecked(cfg.get('enabled',False))
   for widget,value in zip(self.floor_bounds,cfg.get('bounds_map_m',[])):widget.setValue(value)
   self.update_cloud()
  except (OSError,ValueError):pass
 def update_cloud(self,*args):
  super().update_cloud(*args)
  try:
   import numpy as np
   from s100_cloud_gui import COLORS
   cfg=json.loads((ROOT/'runtime/manual_floor.json').read_text())
   if not cfg.get('enabled') or self.archive.meta.get('voxel_file_sha256')!=cfg['map_voxel_sha256']:return
   a,b,c,d=cfg['bounds_map_m'];x,y=np.meshgrid(np.arange(a+.05,b,.10),np.arange(c+.05,d,.10));p=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
   pointer=json.loads((ROOT/'runtime/latest_plan.json').read_text());scene=json.loads((Path(pointer['folder'])/'scene/frozen_input.json').read_text());p[:,2]=np.c_[p[:,:2],np.ones(len(p))]@np.asarray(scene['ground_plane'])
   COLORS[4]=[.05,.80,.85,.7];self.scene.frames['back']=dict(xyz=np.ascontiguousarray(p,np.float32),labels=np.full(len(p),4,np.uint8));self.scene.stale['back']=False;self.scene.cloud_dirty=True;self.scene.dirty=True
  except (OSError,ValueError,KeyError,AttributeError):pass

def main():
 fmt=QSurfaceFormat();fmt.setVersion(2,1);fmt.setProfile(QSurfaceFormat.CompatibilityProfile);QSurfaceFormat.setDefaultFormat(fmt)
 app=QApplication([])
 font=app.font();height=app.primaryScreen().availableGeometry().height()
 font.setPointSize(max(11,min(13,round(height/80))));app.setFont(font)
 window=NavigationWindow();window.showMaximized()
 # Keep the measured head reader alive between navigation sessions. Existing
 # arm/head GUI serial ownership is respected by the helper preflight.
 from onboard_client import head_feedback_reader
 try:head_reader=head_feedback_reader(duration=0)
 except Exception as exc:
  head_reader=None;window.log.appendPlainText('头部只读反馈未就绪：'+str(exc))
 def close_head_reader():
  if head_reader is not None and head_reader.poll() is None:
   head_reader.terminate()
   try:head_reader.wait(timeout=2)
   except __import__('subprocess').TimeoutExpired:head_reader.kill();head_reader.wait(timeout=2)
 app.aboutToQuit.connect(close_head_reader)
 if '--run-once-goal' in sys.argv:
  goal=int(sys.argv[sys.argv.index('--run-once-goal')+1]);assert str(goal) in targets.load()['targets']
  window.once_autostart_pending=True;window.once_execution_seconds=90.;deadline=__import__('time').monotonic()+25.
  def launch_authorized_once():
   if not window.once_autostart_pending:return
   if window.process.state()!=QProcess.NotRunning:
    if __import__('time').monotonic()<deadline:QTimer.singleShot(250,launch_authorized_once)
    else:window.once_autostart_pending=False;window.status.setText('启动检查尚未结束，本次自动测试已取消。')
    return
   window.once_autostart_pending=False;window.choice.setCurrentIndex(window.choice.findData(goal));window.log.appendPlainText('按本次明确授权执行一次实机测试，上限90秒。');window.start_real()
  QTimer.singleShot(4000,launch_authorized_once)
 if '--capture' in sys.argv:QTimer.singleShot(3000,lambda:window.grab().save(sys.argv[sys.argv.index('--capture')+1]))
 app.exec_()
if __name__=='__main__':main()
