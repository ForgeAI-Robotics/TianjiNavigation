#!/usr/bin/env python3
"""Jetson offline map viewer. No Receiver, ROS, SSH or control object is started."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import argparse,json,signal,sys,subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
from saved_map import MapArchive,base_transform
from cloud_library import scan_library,preferred_map,LIBRARY
from pcd_archive import PcdArchive
from pose_memory import read_current,caption
from map_waypoints import load as load_waypoints
import time
VIEWER=ROOT.parent/'navigation_link_v2';sys.path.insert(0,str(VIEWER))
from s100_cloud_gui import Scene,LocalRobotModel,default_urdf_path,COLORS
from link import JOINTS
from viewer_camera import basis
from PyQt5.QtCore import Qt,QTimer,QUrl
from PyQt5.QtGui import QSurfaceFormat,QDesktopServices,QPainter,QColor,QFont
from PyQt5.QtWidgets import QApplication,QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QComboBox,QPushButton,QSlider,QCheckBox,QFileDialog,QMessageBox,QInputDialog
from OpenGL.GL import glDisable,glEnable,glColor4f,glLineWidth,glBegin,glEnd,glVertex3f,GL_DEPTH_TEST,GL_LINE_STRIP,GL_LINES,GL_LINE_LOOP

class ArchiveScene(Scene):
    def __init__(self,model):
        super().__init__(model);self.enabled['distance']=False;self.mode=1;self.path=np.empty((0,3));self.point_size=2.;self.current_pose_matrix=None;self.waypoints=[]
        COLORS[1]=[.6,.7,.75,.65];COLORS[2]=[1.,.18,.12,1.];COLORS[3]=[1.,.7,.12,1.]
    def paintGL(self):
        super().paintGL()
        if len(self.path):
            glDisable(GL_DEPTH_TEST);glColor4f(1.,.6,.12,1.);glLineWidth(2.);glBegin(GL_LINE_STRIP)
            for x,y,_ in self.path:glVertex3f(float(x),float(y),.03)
            glEnd();glLineWidth(1.);glEnable(GL_DEPTH_TEST)
        if self.current_pose_matrix is not None:
            T=self.current_pose_matrix;x,y,z=T[:3,3];z=max(.05,z+.05)
            glDisable(GL_DEPTH_TEST);glColor4f(.15,1.,.35,1.);glLineWidth(3);glBegin(GL_LINE_LOOP)
            for a in np.linspace(0,2*np.pi,32):glVertex3f(float(x+.16*np.cos(a)),float(y+.16*np.sin(a)),float(z))
            glEnd();glBegin(GL_LINES);glVertex3f(float(x),float(y),float(z));glVertex3f(float(x+.5*T[0,0]),float(y+.5*T[1,0]),float(z));glEnd();glLineWidth(1);glEnable(GL_DEPTH_TEST)


        if self.waypoints:
            glDisable(GL_DEPTH_TEST);glLineWidth(3)
            for row in self.waypoints:
                x,y,yaw=row['pose'];z=.10;color=(.15,1.,.45,1.) if row['id']==1 else (.9,.35,1.,1.)
                glColor4f(*color);glBegin(GL_LINE_LOOP)
                for a in np.linspace(0,2*np.pi,32):glVertex3f(float(x+.15*np.cos(a)),float(y+.15*np.sin(a)),z)
                glEnd();glBegin(GL_LINES);glVertex3f(x,y,z);glVertex3f(float(x+.45*np.cos(yaw)),float(y+.45*np.sin(yaw)),z);glEnd()
            glLineWidth(1);glEnable(GL_DEPTH_TEST)
            painter=QPainter(self);painter.setFont(QFont('Sans',13,QFont.Bold))
            offset,_,right,vertical=basis(self.yaw,self.pitch,self.special_view);scale=self.height()/(2*np.tan(np.deg2rad(21)))
            for row in self.waypoints:
                q=np.r_[row['pose'][:2],.10]-self.target-self.distance*offset;depth=-q@offset
                if depth>.05:
                    painter.setPen(QColor(60,255,130) if row['id']==1 else QColor(235,115,255));painter.drawText(int(self.width()/2+scale*(q@right)/depth)+9,int(self.height()/2-scale*(q@vertical)/depth)-9,row['name'])
            painter.end()

class Window(QMainWindow):
    def __init__(self,folder,capture=None):
        super().__init__();self.setWindowTitle('已保存三维地图 · 离线查看（无底盘控制）');self.resize(1200,850);self.capture=capture;self.archive=None
        self.model=LocalRobotModel(default_urdf_path(),VIEWER/'model_manifest.json',VIEWER/'collision_geometry.json');self.scene=ArchiveScene(self.model)
        central=QWidget();self.setCentralWidget(central);layout=QVBoxLayout(central);top=QHBoxLayout();layout.addLayout(top)
        self.maps=QComboBox();self.paths=[]
        top.addWidget(self.maps,1);button=QPushButton('打开其他地图…');button.clicked.connect(self.open_folder);top.addWidget(button)
        history=QPushButton('历史备份…');history.clicked.connect(self.open_history);top.addWidget(history)
        refresh=QPushButton('刷新列表');refresh.clicked.connect(self.refresh_library);top.addWidget(refresh)
        directory=QPushButton('打开保存目录');directory.clicked.connect(lambda:QDesktopServices.openUrl(QUrl.fromLocalFile(str(LIBRARY))));top.addWidget(directory)
        navigation=QPushButton('导航配置…');navigation.clicked.connect(self.open_navigation);top.addWidget(navigation)
        anomalies=QPushButton('异常点云…');anomalies.clicked.connect(self.open_conflicts);top.addWidget(anomalies)
        self.mode=QComboBox();self.mode.addItems(['全部体素 / 滤除后','静态候选体素','原始关键帧','动态候选（红色）','原图 + 动态候选（红色）','证据不足（黄色）','原图 + 上一版恢复保留（黄）']);self.mode.currentIndexChanged.connect(self.update_cloud);top.addWidget(self.mode)
        self.warning=QLabel();self.warning.setWordWrap(True);layout.addWidget(self.warning);self.info=QLabel();self.info.setWordWrap(True);layout.addWidget(self.info)
        position_row=QHBoxLayout();self.position_check=QCheckBox('显示实时定位（绿圈/朝向）');self.position_check.setChecked(True);position_row.addWidget(self.position_check)
        self.position_label=QLabel();self.position_label.setWordWrap(True);position_row.addWidget(self.position_label,1);layout.addLayout(position_row);layout.addWidget(self.scene,1)
        row=QHBoxLayout();layout.addLayout(row);row.addWidget(QLabel('关键帧'));self.slider=QSlider(Qt.Horizontal);self.slider.valueChanged.connect(self.update_cloud);row.addWidget(self.slider,1)
        self.model_check=QCheckBox('显示记录的末帧URDF');self.model_check.setChecked(True);self.model_check.toggled.connect(self.update_cloud);row.addWidget(self.model_check)
        for text,name in [('俯视','top'),('正视','front'),('侧视','side')]:
            button=QPushButton(text);button.clicked.connect(lambda _,n=name:self.scene.view(n));row.addWidget(button)
        fit=QPushButton('适合地图');fit.clicked.connect(self.fit);row.addWidget(fit)
        layout.addWidget(QLabel('左键旋转 · 中键平移 · Shift+中键旋转 · 滚轮缩放 ｜ 橙线为记录轨迹；只有记录了匹配关节状态时显示URDF。此窗口不连接机器人。'))
        self.maps.currentIndexChanged.connect(self.select_map)
        self.refresh_library()
        index=next((i for i,path in enumerate(self.paths) if path.resolve()==folder.resolve()),0);self.maps.blockSignals(True);self.maps.setCurrentIndex(index);self.maps.blockSignals(False);self.load(folder)
        self.library_timer=QTimer(self);self.library_timer.timeout.connect(self.refresh_library);self.library_timer.start(10000)
        self.position_timer=QTimer(self);self.position_timer.timeout.connect(self.update_position);self.position_timer.start(500);self.position_check.toggled.connect(self.update_position);self.update_position()
        if capture:QTimer.singleShot(2000,self.save_capture)

    def open_navigation(self):
        self.navigation_process=subprocess.Popen([sys.executable,str(ROOT.parent/'navigation_waypoints_v6/modules/navigation_gui.py')],start_new_session=True)

    def open_conflicts(self):
        process=getattr(self,'conflict_process',None)
        if process is not None and process.poll() is None:
            QMessageBox.information(self,'异常点云','异常点云窗口已打开，可从任务栏切换。');return
        self.conflict_process=subprocess.Popen([sys.executable,str(ROOT/'modules/conflict_viewer.py')],start_new_session=True)

    def refresh_library(self):
        current=self.maps.currentData()
        try:
            catalogue=scan_library();entries=catalogue['entries'];self.history_entries=catalogue.get('archived_entries',[])
        except OSError as exc:
            self.statusBar().showMessage('点云目录暂不可写：'+str(exc));return
        self.maps.blockSignals(True);self.maps.clear();self.paths=[]
        for entry in entries:
            self.paths.append(Path(entry['path']));self.maps.addItem(entry['label'],entry['path'])
        index=self.maps.findData(current)
        self.maps.setCurrentIndex(max(0,index));self.maps.blockSignals(False)
        self.statusBar().showMessage(f'点云库：{LIBRARY} ｜ {len(entries)}个可用场景 · {len(self.history_entries)}项历史备份')

    def open_history(self):
        self.refresh_library();entries=self.history_entries
        if not entries:QMessageBox.information(self,'历史备份','没有其他历史点云。');return
        labels=[row['label'] for row in entries];label,ok=QInputDialog.getItem(self,'历史备份（仅查看，不切换导航场景）','原始点云和处理版本均保留：',labels,0,False)
        if ok:self.load(Path(entries[labels.index(label)]['path']));self.statusBar().showMessage('当前查看历史备份；导航场景保持原选择。')

    def update_position(self,*_):
        packet=read_current(ROOT/'runtime/current_pose.json',time.time());text=caption(packet)
        matches=self.archive is not None and packet.get('map_voxel_sha256')==self.archive.meta.get('voxel_file_sha256')
        self.scene.current_pose_matrix=None
        if packet.get('map_pose_valid') and not matches:text+=' · 当前所选点云不是定位参考地图，未叠加位置'
        if matches and packet.get('map_pose_valid') and self.position_check.isChecked():self.scene.current_pose_matrix=np.array(packet['T_map_base'])
        self.position_label.setText(text+'；黄色URDF/橙线仍为历史记录');self.scene.dirty=True

    def select_map(self,index):
        if 0<=index<len(self.paths):self.load(self.paths[index])
    def open_folder(self):
        filename,_=QFileDialog.getOpenFileName(self,'选择地图元数据',str(LIBRARY),'地图与点云 (map.json *.pcd)')
        if filename:self.load(Path(filename) if filename.lower().endswith('.pcd') else Path(filename).parent)
    def load(self,folder):
        try:archive=PcdArchive(folder) if folder.suffix.lower()=='.pcd' else MapArchive(folder)
        except Exception as exc:QMessageBox.warning(self,'不能加载地图',str(exc));return
        self.archive=archive;self.evidence=None
        self.reviewed_keep=np.ones(len(archive.points),bool)
        review=ROOT.parent/'navigation_s100_v7/config/scenes/office_fullmap_cleanup.json'
        if review.exists():
            cfg=json.loads(review.read_text())
            if cfg.get('map_voxel_sha256')==archive.meta.get('voxel_file_sha256'):
                reviewed={tuple(key) for key in cfg['voxel_keys']}
                keys=np.floor(archive.points/cfg['resolution_m']).astype(np.int64)
                self.reviewed_keep=np.array([tuple(key) not in reviewed for key in keys],bool)

        if archive.meta.get('dynamic_evidence'):
            with np.load(archive.folder/archive.meta['dynamic_evidence'],allow_pickle=False) as data:
                self.evidence=dict(points=np.array(data['points']),labels=np.array(data['labels']))
                if 'restored_from_v1' in data:self.evidence['restored_from_v1']=np.array(data['restored_from_v1'])
                for key in ('self_return_removed','previous_labels','manual_removed','history_self_removed','target_ray_removed','body_repetition_removed','padding_self_removed'):
                    if key in data:self.evidence[key]=np.array(data[key])
        self.slider.setEnabled(bool(archive.keyframes));self.mode.model().item(2).setEnabled(bool(archive.keyframes))
        if not archive.keyframes and self.mode.currentIndex()==2:self.mode.setCurrentIndex(0)
        self.mode.blockSignals(True)
        self_mode=self.evidence is not None and 'self_return_removed' in self.evidence
        self.mode.setItemText(3,'动态/自体候选（红色）' if self_mode else '动态候选（红色）');self.mode.setItemText(4,'原图 + 动态/自体候选（红色）' if self_mode else '原图 + 动态候选（红色）')
        if self.evidence is not None and 'manual_removed' in self.evidence:
            self.mode.setItemText(3,'已移除点（含人工，红色）');self.mode.setItemText(4,'编辑前原图 + 已移除点（红色）')
        for i in (3,4,5):self.mode.model().item(i).setEnabled(self.evidence is not None)
        restored_available=self.evidence is not None and 'restored_from_v1' in self.evidence
        self.mode.model().item(6).setEnabled(restored_available)
        if self.evidence is None and self.mode.currentIndex()>2:self.mode.setCurrentIndex(0)
        if self.mode.currentIndex()==6 and not restored_available:self.mode.setCurrentIndex(0)
        self.mode.blockSignals(False)
        self.slider.blockSignals(True);self.slider.setRange(0,max(0,len(archive.keyframes)-1));self.slider.setValue(max(0,len(archive.keyframes)-1));self.slider.blockSignals(False)
        meta=archive.meta;self.scene.waypoints=load_waypoints(meta)
        warning='本次记录中途失效：'+str(meta['invalid_reason'])+'。已保存点云可查看，不能作为有效导航地图。' if meta.get('invalid_reason') else '这是离线验证或派生地图，尚未完成导航验收。' if meta.get('validation_only') else '会话里程计地图；未进行闭环优化或完整覆盖认证。'
        if self.evidence is not None:
            removal_metadata=meta.get('dynamic_removal',{})
            method=removal_metadata.get('method','') if isinstance(removal_metadata,dict) else str(removal_metadata)
            if meta.get('dufomap_report'):warning+=' DUFOMap官方核心试验：未叠加本地自体算法或人工删除；通过历史空闲空间识别动态点。'
            else:warning+=(' 此结果为V1，尚无视角/细结构保护，仅供对照。' if method.endswith('_V1') else ' 已启用相近视角、邻近表面误差带和细结构保护。')
            warning+=' 红色仍只是动态候选，不是人员语义识别；不确定点保留。'
        if self.evidence is not None and 'self_return_removed' in self.evidence:warning+=' V3新增URDF原始三角面与部件局部重复证据；环境见证冲突保留。'
        if meta.get('history_filter_report'):warning+=' 当前含V4离线8cm表面带试验；仍需历史重复证据，不用于导航放行。'
        if meta.get('target_ray_report'):warning+=' V5对轨迹附近区域追加实际射线复核；未按轨迹直接删点。'
        if meta.get('body_repetition_report'):warning+=' V6追加近体部位重复观测试验（至少60秒及1米位移）；仅离线候选结果。'
        if meta.get('padding_filter_report'):warning+=' V7过滤模型表面膨胀2cm，加8cm匹配容差；原URDF未改。'
        if meta.get('view_only_crop'):
            warning+=' 当前为附近区域的裁剪视图；不是桌子语义分割，原始关键帧未裁剪。'
            if meta['view_only_crop'].get('lower_xyz_m',[0,0,1])[2]<.05:warning+=' 当前高度范围已包含地面。'
        if meta.get('user_confirmed_trail_cleanup'):warning+=' 已人工清理轨迹附近选定区域；这是人工编辑副本，不是自动动态识别，不能用于导航占用判断。'
        if meta.get('source_file'):warning+=' 独立PCD使用文件原坐标系，无历史关节/轨迹信息。'
        if self.scene.waypoints:warning+=' 导航标点：'+'；'.join(f"{w['name']} X={w['pose'][0]:.3f} Y={w['pose'][1]:.3f}m" for w in self.scene.waypoints)
        self.warning.setText(warning);self.warning.setStyleSheet('background:#5b2929;color:#fff;padding:8px;' if meta.get('invalid_reason') else 'background:#644b20;color:#fff;padding:8px;');self.scene.path=archive.trajectory;self.update_cloud();self.fit()
    def update_cloud(self,*_):
        if self.archive is None:return
        archive=self.archive;meta=archive.meta;mode=self.mode.currentIndex();index=self.slider.value();joints=(meta.get('latest') or {}).get('joints',{});pose=(meta.get('latest') or {}).get('base_xy_yaw',[0.,0.,0.])
        COLORS[2]=[1.,.75,.12,1.] if mode==6 else [1.,.18,.12,1.]
        labels=None
        if mode==2 and archive.keyframes:points,pose,joints,stamp=archive.frame(index)
        elif mode>=3 and self.evidence is not None:
            p=self.evidence['points'];classification=self.evidence['labels']
            if mode==3:points=p[classification==2];labels=np.full(len(points),2,np.uint8)
            elif mode==4:points=p;labels=np.where(classification==2,2,1).astype(np.uint8)
            elif mode==6:points=p;labels=np.where(self.evidence['restored_from_v1'],2,1).astype(np.uint8)
            else:points=p[classification==0];labels=np.full(len(points),3,np.uint8)
        else:points=archive.points[archive.static&self.reviewed_keep] if mode==1 else archive.points[self.reviewed_keep]
        if labels is None:labels=np.ones(len(points),np.uint8)
        self.scene.frames={'front':dict(xyz=np.ascontiguousarray(points,np.float32),labels=labels)};self.scene.stale={'front':False};self.scene.cloud_dirty=True
        self.scene.shapes=[];model_valid=set(JOINTS).issubset(joints.get('joint_positions_rad',{})) and joints.get('model_sha256')==self.model.sha256
        if self.model_check.isChecked() and model_valid:
            transform=base_transform(pose)
            if mode==2 and archive.keyframes:
                with np.load(archive.keyframes[index],allow_pickle=False) as recorded:
                    if 'pose_matrix' in recorded:transform=np.array(recorded['pose_matrix'],float)
            elif (meta.get('latest') or {}).get('base_transform_4x4') is not None:transform=np.array(meta['latest']['base_transform_4x4'],float)
            for shape in self.model.shapes(joints['joint_positions_rad']):
                if shape['mode']!='measured_or_fixed':continue
                shape=dict(shape,matrix=(transform@np.array(shape['matrix'])).tolist());self.scene.shapes.append(shape)
        self.scene.enabled['model']=self.model_check.isChecked();self.scene.dirty=True
        frame=(' · 原始帧 '+archive.keyframes[index].name) if mode==2 and archive.keyframes else ''
        removal_name='已移除点（含人工）' if self.evidence is not None and 'manual_removed' in self.evidence else '动态候选'
        reviewed_info=f' · 本轮复核清理 {int((~self.reviewed_keep).sum())} 点' if (~self.reviewed_keep).any() else ''
        dynamic_info=reviewed_info+(' · '+removal_name+' '+str(int(np.count_nonzero(self.evidence['labels']==2)))) if self.evidence is not None else ''
        if self.evidence is not None and 'self_return_removed' in self.evidence and 'previous_labels' in self.evidence:dynamic_info+=' · V3自身追加移除 '+str(int(np.count_nonzero(self.evidence['self_return_removed']&(self.evidence['previous_labels']!=2))))
        if self.evidence is not None and 'history_self_removed' in self.evidence:dynamic_info+=' · V4历史自体追加移除 '+str(int(np.count_nonzero(self.evidence['history_self_removed'])))
        if self.evidence is not None and 'target_ray_removed' in self.evidence:dynamic_info+=' · V5射线追加移除 '+str(int(np.count_nonzero(self.evidence['target_ray_removed'])))
        if self.evidence is not None and 'body_repetition_removed' in self.evidence:dynamic_info+=' · V6近体重复追加移除 '+str(int(np.count_nonzero(self.evidence['body_repetition_removed'])))
        if self.evidence is not None and 'padding_self_removed' in self.evidence:dynamic_info+=' · V7膨胀追加移除 '+str(int(np.count_nonzero(self.evidence['padding_self_removed'])))
        if self.evidence is not None and 'manual_removed' in self.evidence:dynamic_info+=' · 人工移除 '+str(int(np.count_nonzero(self.evidence['manual_removed'])))
        if meta.get('dufomap_processed_frames'):dynamic_info+=' · DUFOMap处理扫描 '+str(meta['dufomap_processed_frames'])+' 帧'
        if self.evidence is not None and 'restored_from_v1' in self.evidence:dynamic_info+=' · 上版恢复保留 '+str(int(np.count_nonzero(self.evidence['restored_from_v1'])))
        self.info.setText(f'{archive.folder}\n保存 {meta["frames"]} 帧 · 全部体素 {len(archive.points):,} · 静态候选 {int(archive.static.sum()):,} · 当前显示 {len(points):,} 点{frame}{dynamic_info} · URDF：'+('历史实测姿态' if self.scene.shapes else '无此帧完整匹配姿态，未显示'))
    def fit(self):
        if self.archive is None or not len(self.archive.points):return
        low,high=np.percentile(self.archive.points,[3,97],axis=0);self.scene.target=(low+high)*.5;self.scene.distance=float(np.clip(np.linalg.norm(high-low)*.9,2.,25.));self.scene.far_clip=max(50.,float(np.linalg.norm(self.archive.points-self.scene.target,axis=1).max()*2));self.scene.dirty=True
    def save_capture(self):
        self.scene.grabFramebuffer().save(str(self.capture));print('CAPTURE '+str(self.capture),flush=True)

def main():
    default=preferred_map()
    parser=argparse.ArgumentParser();parser.add_argument('map',type=Path,nargs='?',default=default);parser.add_argument('--capture',type=Path);parser.add_argument('--test-exit-seconds',type=int,default=0);parser.add_argument('--mode',choices=['clean','static','raw','dynamic','compare','uncertain','restored'],default='clean');args=parser.parse_args()
    fmt=QSurfaceFormat();fmt.setVersion(2,1);fmt.setProfile(QSurfaceFormat.CompatibilityProfile);fmt.setSamples(0);fmt.setSwapInterval(1);QSurfaceFormat.setDefaultFormat(fmt)
    app=QApplication([]);folder=args.map.parent if args.map.name=='map.json' else args.map;window=Window(folder,args.capture);window.mode.setCurrentIndex(['clean','static','raw','dynamic','compare','uncertain','restored'].index(args.mode));window.show()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:window.close())
    if args.test_exit_seconds:QTimer.singleShot(args.test_exit_seconds*1000,window.close)
    app.exec_()

if __name__=='__main__':main()
