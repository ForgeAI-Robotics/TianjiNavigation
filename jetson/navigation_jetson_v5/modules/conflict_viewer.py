"""Offline anomaly cloud and original URDF viewer. No receiver or robot I/O."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import argparse,json,signal,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
V2=ROOT.parent/'navigation_link_v2';sys.path.insert(0,str(V2))
from s100_cloud_gui import Scene,LocalRobotModel,default_urdf_path
from viewer_camera import basis
from preview_conflicts import load_preview
from PyQt5.QtCore import Qt,QTimer,QPoint
from PyQt5.QtGui import QSurfaceFormat,QPainter,QColor,QFont
from PyQt5.QtWidgets import QApplication,QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QCheckBox,QPushButton,QComboBox,QSplitter,QTableWidget,QTableWidgetItem,QAbstractItemView,QFileDialog,QMessageBox
from OpenGL.GL import *

DEFAULT_MANIFEST=ROOT.parent/'navigation_waypoints_v6/results/blocker_diagnosis/latest_saved.json'
LATEST_PREVIEW=ROOT.parent/'navigation_waypoints_v6/runtime/latest_plan.json'

def latest_folder():
    if LATEST_PREVIEW.exists():
        return Path(json.loads(LATEST_PREVIEW.read_text())['folder'])
    return Path(json.loads(DEFAULT_MANIFEST.read_text())['project_folder'])

EDGES=[(a,b) for a in range(8) for b in range(a+1,8) if a^b in (1,2,4)]

def load_snapshot(folder,model_hash):
    folder=Path(folder)
    meta=json.loads((folder/'逐点对应与URDF位置.json').read_text())
    if meta['source_urdf_sha256']!=model_hash:raise ValueError('标注URDF版本与本机原模型不一致')
    with np.load(folder/'全部标注数据.npz',allow_pickle=False) as data:points=np.array(data['conflict_points_body'],float)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():raise ValueError('异常点数据无效')
    groups={}
    for row in meta['pairs']:groups.setdefault(row['point_id'],[]).append(row)
    if len(groups)!=len(points):raise ValueError('标注编号与点云数量不一致')
    ordered=sorted(groups)
    if not np.allclose(points,np.array([groups[k][0]['point_body_m'] for k in ordered]).reshape(-1,3)):raise ValueError('标注坐标与点云不一致')
    return meta,points,groups

class ConflictScene(Scene):
    def __init__(self,model):
        super().__init__(model);self.enabled['distance']=False;self.enabled['model']=True
        self.self_extra=np.empty((0,3));self.show_self_extra=True;self.ignored=np.empty((0,3));self.show_ignored=True;self.points=np.empty((0,3));self.nearest=[];self.pairs=[];self.boxes=[];self.selected=None;self.show_boxes=True;self.show_lines=True;self.show_labels=False;self.show_nearest=True;self.point_size=6.;self.mode=1
    def projected(self,point):
        offset,_,right,vertical=basis(self.yaw,self.pitch,self.special_view)
        p=np.asarray(point)-self.target-self.distance*offset;depth=-p@offset
        if depth<=.05:return None
        scale=self.height()/(2*np.tan(np.deg2rad(21)))
        return self.width()/2+scale*(p@right)/depth,self.height()/2-scale*(p@vertical)/depth
    def paintGL(self):
        super().paintGL();glDisable(GL_DEPTH_TEST)
        if self.show_boxes:
            glColor4f(.97,.70,.18,.55);glLineWidth(1.);glBegin(GL_LINES)
            for box in self.boxes:
                vertices=np.array(box['body_corners_m'])
                for a,b in EDGES:glVertex3dv(vertices[a]);glVertex3dv(vertices[b])
            glEnd()
        if self.show_lines:
            glColor4f(.75,.43,1.,.85);glLineWidth(1.4);glBegin(GL_LINES)
            for row in self.pairs:glVertex3dv(row['point_body_m']);glVertex3dv(row['nearest_urdf_collision_body_m'])
            glEnd()
        if self.show_nearest:
            glPointSize(6.);glColor4f(.15,.65,1.,1.);glBegin(GL_POINTS)
            for row in self.pairs:glVertex3dv(row['nearest_urdf_collision_body_m'])
            glEnd()
        if self.show_self_extra:
            glPointSize(5.);glColor4f(.15,1.,.8,1.);glBegin(GL_POINTS)
            for point in self.self_extra:glVertex3dv(point)
            glEnd()
        if self.show_ignored:
            glPointSize(5.);glColor4f(1.,.48,.05,1.);glBegin(GL_POINTS)
            for point in self.ignored:glVertex3dv(point)
            glEnd()
        glPointSize(7.);glColor4f(1.,.12,.09,1.);glBegin(GL_POINTS)
        for point in self.points:glVertex3dv(point)
        glEnd()
        if self.selected is not None:
            glPointSize(13.);glColor4f(.1,1.,.3,1.);glBegin(GL_POINTS);glVertex3dv(self.selected[1]);glEnd()
        glPointSize(1.);glLineWidth(1.);glEnable(GL_DEPTH_TEST)
        if self.show_labels or self.selected is not None:
            painter=QPainter(self);painter.setFont(QFont('Sans',9));painter.setPen(QColor(255,115,110))
            if self.show_labels:
                for name,point in self.nearest:
                    p=self.projected(point)
                    if p:painter.drawText(int(p[0])+7,int(p[1])-6,name)
            if self.selected is not None:
                p=self.projected(self.selected[1]);painter.setPen(QColor(60,255,130));painter.setFont(QFont('Sans',11,QFont.Bold))
                if p:painter.drawText(int(p[0])+12,int(p[1])-15,self.selected[0])
            painter.end()

class Window(QMainWindow):
    def __init__(self,folder):
        super().__init__();self.resize(1450,930);self.setWindowTitle('异常点云与URDF · 保存的导航冲突快照 · 无运动控制')
        self.model=LocalRobotModel(default_urdf_path(),V2/'model_manifest.json',V2/'collision_geometry.json');self.scene=ConflictScene(self.model);self.folder=Path(folder)
        central=QWidget();self.setCentralWidget(central);layout=QVBoxLayout(central)
        self.title=QLabel();self.title.setWordWrap(True);self.title.setStyleSheet('font-size:16px;padding:9px;background:#253b4b');layout.addWidget(self.title)
        note=QLabel('红色：触发冲突的体素中心　蓝色：对应URDF碰撞体最近点　紫色：对应连线　黄色：原URDF网格/碰撞体。保存快照，非实时；落入保护范围不等于真实接触。');note.setWordWrap(True);layout.addWidget(note)
        row=QHBoxLayout();layout.addLayout(row);row.addWidget(QLabel('查看部位：'));self.filter=QComboBox();self.filter.currentIndexChanged.connect(self.update_layers);row.addWidget(self.filter,1)
        for label,view in [('三维',None),('俯视 XY','top'),('侧视 XZ','side'),('正视 YZ','front')]:
            button=QPushButton(label);button.clicked.connect(lambda _,v=view:self.scene.view(v));row.addWidget(button)
        fit=QPushButton('整机视角');fit.clicked.connect(self.fit_robot);row.addWidget(fit)
        fit=QPushButton('聚焦异常');fit.clicked.connect(self.fit_points);row.addWidget(fit)
        button=QPushButton('加载最新规划');button.clicked.connect(self.load_latest);row.addWidget(button)
        button=QPushButton('打开其他标注…');button.clicked.connect(self.open_folder);row.addWidget(button)
        layers=QHBoxLayout();layout.addLayout(layers);self.checks={}
        for label,key,checked in [('原URDF网格','model',True),('规划碰撞盒','boxes',True),('对应连线','lines',True),('蓝色最近点','nearest',True),('全部编号','labels',False),('灰色环境点','environment',False),('橙色临时忽略线材','ignored',True),('青色扩大自我滤除','self_extra',True)]:
            c=QCheckBox(label);c.setChecked(checked);c.toggled.connect(self.update_layers);layers.addWidget(c);self.checks[key]=c
        layers.addStretch();split=QSplitter();split.addWidget(self.scene);side=QWidget();side_layout=QVBoxLayout(side)
        side_layout.addWidget(QLabel('点击编号高亮；双击编号聚焦该点'))
        self.table=QTableWidget(0,4);self.table.setHorizontalHeaderLabels(['编号','对应部件','Z(cm)','中心距(cm)']);self.table.setSelectionBehavior(QAbstractItemView.SelectRows);self.table.setSelectionMode(QAbstractItemView.SingleSelection);self.table.setEditTriggers(QAbstractItemView.NoEditTriggers);self.table.itemSelectionChanged.connect(self.select_point);self.table.cellDoubleClicked.connect(self.focus_selected);self.table.verticalHeader().hide();side_layout.addWidget(self.table,1)
        self.detail=QLabel('选择一个异常点查看坐标');self.detail.setWordWrap(True);self.detail.setTextInteractionFlags(Qt.TextSelectableByMouse);side_layout.addWidget(self.detail)
        side.setMinimumWidth(375);split.addWidget(side);split.setSizes([1030,400]);layout.addWidget(split,1)
        layout.addWidget(QLabel('左键旋转 · 中键平移 · Shift+中键旋转 · 滚轮缩放 ｜ 仅查看保存的数据，不连接机器人，不发送控制指令。'))
        self.setStyleSheet('QMainWindow,QWidget{background:#101c26;color:#dae6ed}QPushButton,QComboBox{background:#263c4b;border:1px solid #466171;padding:6px;border-radius:3px}QTableWidget{gridline-color:#344a59}QHeaderView::section{background:#263c4b;padding:5px}QCheckBox{padding:4px}')
        self.load(folder)
    def load(self,folder):
        folder=Path(folder);cloud=None
        if (folder/'planning/plan.json').exists():
            meta,points,groups,cloud=load_preview(folder,self.model.sha256)
        else:
            meta,points,groups=load_snapshot(folder,self.model.sha256)
        self.folder=folder;self.meta=meta;self.groups=groups;self.all_points=points;self.selected_id=None
        self.all_shapes=[]
        if cloud is not None:
            world=json.loads((folder/'scene/immutable_world.json').read_text())
            self.all_shapes=[shape for shape in self.model.shapes(world['joint_positions_rad']) if shape['mode']=='measured_or_fixed']
        for item in meta['urdf_collision_poses']:
            if cloud is None and item['mode']=='measured_or_fixed':self.all_shapes.append(dict(link=item['link'],name=item['collision_name'],matrix=item['T_body_collision'],bounds=item['local_bounds_m'],mode=item['mode']))
        scene_folder=Path(meta['source_scene']);self.environment=np.empty((0,3));self.scene.ignored=np.empty((0,3))
        if (scene_folder/'scene/temporary_pillar_cables.npz').exists():
            with np.load(scene_folder/'scene/temporary_pillar_cables.npz') as data:self.scene.ignored=np.array(data['ignored_points_body']).reshape(-1,3)
        if cloud is not None:
            self.environment=cloud[(abs(cloud[:,0])<1.5)&(abs(cloud[:,1])<1.1)&(cloud[:,2]>-.1)&(cloud[:,2]<1.8)]
        elif (scene_folder/'scene/display_clouds.npz').exists():
            with np.load(scene_folder/'scene/display_clouds.npz') as data:p=data['occupied_local']
            self.environment=p[(abs(p[:,0])<1.5)&(abs(p[:,1])<1.1)&(p[:,2]>-.1)&(p[:,2]<1.8)]
        self.scene.self_extra=np.empty((0,3))
        if (scene_folder/'scene/self_filter_envelope.npz').exists():
            with np.load(scene_folder/'scene/self_filter_envelope.npz') as data:self.scene.self_extra=np.concatenate([data['extra_raw_points_body'],data['extra_voxel_points_body']])
        self.filter.blockSignals(True);self.filter.clear();self.filter.addItem(f'全部异常点（{len(groups)}）',None)
        for link in dict.fromkeys(p['link'] for p in meta['pairs']):self.filter.addItem(link,link)
        self.filter.blockSignals(False)
        budget=meta['thresholds'];self.title.setText(f'已加载 {len(groups)} 个异常体素 / {len(meta["pairs"])} 组URDF对应关系　｜　判定范围：{budget["safety_margin_m"]*100:g}cm安全间隔 + {budget["voxel_half_diagonal_m"]*100:.2f}cm体素保护\n{self.folder}')
        self.title.setText(self.title.text()+f'\n橙色：临时忽略的立柱线材候选，共{len(self.scene.ignored)}个原始点；红色仍参与碰撞判断。')
        self.title.setText(self.title.text()+f'\n青色：扩大自我滤除包络后额外排除的点（原始点及体素），共{len(self.scene.self_extra)}个；碰撞盒保持原值。')
        if cloud is not None:
            self.title.setText(meta['preview_label']+f' · {len(groups)} 个触发体素\n{self.folder}')
            self.detail.setText('表格距离为体素中心到碰撞盒的距离')
        self.update_layers();self.fit_robot()
        if cloud is not None and len(points):self.fit_points()
    def load_latest(self):
        try:self.load(latest_folder())
        except Exception as exc:QMessageBox.warning(self,'不能加载最新规划',str(exc))
    def open_folder(self):
        path=QFileDialog.getExistingDirectory(self,'选择标注目录',str(self.folder.parent))
        if not path:return
        try:self.load(Path(path))
        except Exception as exc:QMessageBox.warning(self,'不能加载',str(exc))
    def update_layers(self,*_):
        if not hasattr(self,'meta'):return
        link=self.filter.currentData();pairs=[p for p in self.meta['pairs'] if link is None or p['link']==link];names=sorted({p['point_id'] for p in pairs})
        self.visible_names=names;self.scene.points=np.array([self.groups[n][0]['point_body_m'] for n in names]).reshape(-1,3);self.scene.pairs=pairs;self.scene.nearest=[(n,self.groups[n][0]['point_body_m']) for n in names]
        self.scene.shapes=self.all_shapes;self.scene.enabled['model']=self.checks['model'].isChecked();self.scene.boxes=[b for b in self.meta['urdf_collision_poses'] if link is None or b['link']==link]
        self.scene.show_self_extra=self.checks['self_extra'].isChecked();self.scene.show_ignored=self.checks['ignored'].isChecked();self.scene.show_boxes=self.checks['boxes'].isChecked();self.scene.show_lines=self.checks['lines'].isChecked();self.scene.show_nearest=self.checks['nearest'].isChecked();self.scene.show_labels=self.checks['labels'].isChecked()
        self.scene.frames={'front':dict(xyz=np.ascontiguousarray(self.environment,np.float32),labels=np.ones(len(self.environment),np.uint8))} if self.checks['environment'].isChecked() else {};self.scene.stale={'front':False};self.scene.cloud_dirty=True;self.scene.dirty=True
        self.table.blockSignals(True);self.table.setRowCount(len(names))
        for i,name in enumerate(names):
            rows=[p for p in pairs if p['point_id']==name];closest=min(rows,key=lambda p:p['point_to_collision_distance_m']);values=[name,','.join(dict.fromkeys(p['link'] for p in rows)),f'{closest["point_body_m"][2]*100:.1f}',f'{closest["point_to_collision_distance_m"]*100:.2f}']
            for j,value in enumerate(values):self.table.setItem(i,j,QTableWidgetItem(value))
        self.table.resizeColumnsToContents();self.table.setColumnWidth(1,150);self.table.blockSignals(False)
        if self.selected_id in names:self.table.selectRow(names.index(self.selected_id))
        else:self.scene.selected=None;self.detail.setText('选择编号查看体素和对应URDF坐标（厘米）')
    def select_point(self):
        row=self.table.currentRow()
        if row<0 or row>=len(self.visible_names):return
        name=self.visible_names[row];self.selected_id=name;pairs=[p for p in self.scene.pairs if p['point_id']==name];closest=min(pairs,key=lambda p:p['point_to_collision_distance_m']);p=np.array(closest['point_body_m'])*100;q=np.array(closest['nearest_urdf_collision_body_m'])*100
        self.scene.selected=(name,np.array(closest['point_body_m']));self.scene.dirty=True
        self.detail.setText(f'{name} · {closest["link"]}\n异常点 X/Y/Z：{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f} cm\nURDF最近点：{q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f} cm\n距离：{closest["point_to_collision_distance_m"]*100:.2f} cm\n坐标系：X前 / Y左 / Z上\n初筛范围{self.meta["thresholds"]["effective_collision_distance_m"]*100:.2f}cm；未确认真实接触。')
    def focus_selected(self,*_):
        if self.scene.selected is not None:self.scene.target=np.array(self.scene.selected[1]);self.scene.distance=1.1;self.scene.dirty=True
    def fit_robot(self):self.scene.target=np.array([.15,0.,.72]);self.scene.distance=2.8;self.scene.dirty=True
    def fit_points(self):
        if len(self.scene.points):self.scene.target=(self.scene.points.min(0)+self.scene.points.max(0))/2;self.scene.distance=1.9;self.scene.dirty=True
    def capture(self,path):
        image=self.grab().toImage();p=QPainter(image);p.drawImage(self.scene.mapTo(self,QPoint(0,0)),self.scene.grabFramebuffer());p.end();image.save(str(path))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path,nargs='?');parser.add_argument('--capture',type=Path);parser.add_argument('--test-exit-seconds',type=int,default=0);args=parser.parse_args()
    folder=args.folder or latest_folder()
    fmt=QSurfaceFormat();fmt.setVersion(2,1);fmt.setProfile(QSurfaceFormat.CompatibilityProfile);fmt.setSamples(0);QSurfaceFormat.setDefaultFormat(fmt)
    app=QApplication([]);window=Window(folder);window.show()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:window.close())
    if args.capture:QTimer.singleShot(2200,lambda:window.capture(args.capture))
    if args.test_exit_seconds:QTimer.singleShot(args.test_exit_seconds*1000,window.close)
    app.exec_()

if __name__=='__main__':main()
