#!/usr/bin/env python3
"""Jetson-local hardware OpenGL viewer. No robot control publisher or SDK."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import argparse,json,hashlib,struct,threading,time,signal,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import paramiko
from PyQt5.QtCore import Qt,QTimer,QLockFile,QPoint
from PyQt5.QtGui import QSurfaceFormat,QPainter,QColor,QFont
from PyQt5.QtWidgets import QApplication,QMainWindow,QOpenGLWidget,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QCheckBox,QComboBox,QPushButton,QSplitter,QGridLayout,QMessageBox,QDoubleSpinBox,QScrollArea
from OpenGL.GL import *
from OpenGL.GLU import gluPerspective,gluLookAt
from local_model import LocalRobotModel,default_urdf_path
from navigation_pose import source_packet,save_current,store
from lidar_shapemask import box_mesh
from eight_direction import merge_directions,NAMES
from cloud_sampling import statistics,preview_indices
from viewer_camera import basis,pan_delta
from view_protocol import unpack,frame_age,frame_fresh,MAX_BYTES,MAGIC
ROOT=Path(__file__).resolve().parent
REMOTE='/dev/shm/tianji_s100_live_view'
UNIT='tianji-dev-shapemask-view.service'
DIRECTION_COLORS=['#ff9d54','#e5d455','#7fe083','#49d7cb','#73b9ff','#b59aff','#ee91da','#ff9292']
COLORS=np.array([[.4,.4,.4,.2],[.38,.49,.56,.48],[.1,.95,.82,1.],[1.,.65,.2,.85],[.65,.4,.9,.8],[.2,.57,1.,.9]],np.float32)


class Receiver(threading.Thread):
    def __init__(self,model_sha,config):
        super().__init__(daemon=True);self.sha=model_sha;self.config=config;self.lock=threading.Lock();self.frames={};self.info={};self.error='连接中';self.stop_event=threading.Event();self.client=None;self.sftp=None;self.owned=False;self.seen={};self.enabled=threading.Event();self.enabled.set();self.display_mode='recognition'
    def set_connected(self,value):
        if value:self.enabled.set()
        else:self.enabled.clear()
    def snapshot(self):
        with self.lock:return dict(self.frames),dict(self.info),self.error
    def command(self,command):
        i,o,e=self.client.exec_command(command,timeout=4);i.close();result=o.read().decode();error=e.read().decode();rc=o.channel.recv_exit_status()
        return rc,result,error
    def connect(self):
        c=paramiko.SSHClient();c.load_host_keys(str(ROOT/'viewer_known_hosts'));c.set_missing_host_key_policy(paramiko.RejectPolicy())
        c.connect(self.config['host'],username=self.config['username'],password=self.config['password'],look_for_keys=False,allow_agent=False,timeout=4,auth_timeout=4,banner_timeout=4)
        self.client=c
        rc,_,_=self.command('systemctl --user is-active --quiet '+UNIT)
        self.owned=self.owned or rc!=0
        if rc!=0:
            rc,_,error=self.command('systemctl --user start '+UNIT)
            if rc:raise RuntimeError('开发接收服务启动失败：'+error[:120])
        self.sftp=c.open_sftp();self.sftp.get_channel().settimeout(3.)
    def run(self):
        last_maintenance=0.
        try:
            while not self.stop_event.is_set():
                try:
                    if not self.enabled.is_set():
                        if self.client and self.owned:
                            self.command('systemctl --user stop '+UNIT);self.owned=False
                        self.disconnect()
                        with self.lock:self.frames={};self.info={};self.error='已断开 S100'
                        self.stop_event.wait(.2);continue
                    if self.client is None:self.connect()
                    if time.monotonic()-last_maintenance>=1.:
                        try:
                            with self.sftp.open(REMOTE+'/viewer.lease','wb') as f:f.write(b'Jetson read-only GUI\n')
                        except FileNotFoundError:
                            self.stop_event.wait(.5);continue
                        source=source_packet(self.sha);source['display_mode']=self.display_mode;payload=json.dumps(source,allow_nan=False).encode()
                        temporary=REMOTE+'/pose_source.json.upload'
                        with self.sftp.open(temporary,'wb') as f:f.write(payload)
                        self.sftp.posix_rename(temporary,REMOTE+'/pose_source.json')
                        for name in ['load','status']:
                            try:
                                with self.sftp.open(REMOTE+'/'+name+'.json','rb') as f:d=json.loads(f.read(64000))
                                with self.lock:self.info[name]=d
                            except FileNotFoundError:pass
                        last_maintenance=time.monotonic()
                    for sensor in ['front','back']:
                        try:
                            with self.sftp.open(REMOTE+'/'+sensor+'.view','rb') as f:
                                prefix=f.read(12)
                                if len(prefix)!=12 or prefix[:8]!=MAGIC:raise ValueError('Invalid live display prefix')
                                size=struct.unpack_from('<I',prefix,8)[0]
                                if not 0<size<=64000:raise ValueError('Invalid live display header length')
                                header=f.read(size);probe_header=json.loads(header);key=(probe_header.get('session_id'),probe_header.get('sequence'))
                                if self.seen.get(sensor)==key:continue
                                payload=prefix+header+f.read(MAX_BYTES-len(prefix)-len(header)+1)
                        except FileNotFoundError:continue
                        h,xyz,labels=unpack(payload,self.sha);key=(h.get('session_id'),h.get('sequence'))
                        if self.seen.get(sensor)!=key:
                            self.seen[sensor]=key
                            sampling=statistics(xyz,h.get('sensor_origin_model_m',[0.,0.,0.]))
                            with self.lock:self.frames[sensor]=dict(header=h,xyz=xyz,labels=labels,arrival=time.monotonic(),key=key,sampling=sampling)
                    with self.lock:self.error=''
                    self.stop_event.wait(.2)
                except Exception as exc:
                    with self.lock:self.error=str(exc)[:180]
                    self.disconnect();self.stop_event.wait(2.)
        finally:
            if self.client and self.owned:
                try:self.command('systemctl --user stop '+UNIT)
                except Exception:pass
            self.disconnect()
    def disconnect(self):
        if self.sftp:
            try:self.sftp.close()
            except Exception:pass
        if self.client:
            try:self.client.close()
            except Exception:pass
        self.sftp=None;self.client=None


class Scene(QOpenGLWidget):
    def __init__(self,model):
        super().__init__();self.model=model;self.mesh_data={};self.mesh_buffers={};self.frames={};self.stale={};self.enabled={'front':True,'back':True,'env':True,'self':True,'unknown':True,'model':True,'distance':True,'cables':True,'table':True,'zones':False};self.mode=1;self.distances=merge_directions({}, {})
        self.pose={};self.shapes=[];self.yaw=-.95;self.pitch=.42;self.distance=2.8;self.target=np.array([.12,0.,.74]);self.last_mouse=None;self.special_view=None;self.dirty=True;self.cloud_dirty=True;self.renderer='等待 OpenGL';self.paint_ms=0.;self.paint_count=0;self.point_buffer=None;self.color_buffer=None;self.count=0;self.self_count=0;self.saved_positions=None;self.full_cloud=True;self.point_size=1.;self.far_clip=40.
        self.setMinimumSize(700,550)
        self.timer=QTimer(self);self.timer.timeout.connect(self.render_if_dirty);self.timer.start(50)
        self.load_meshes()
    def load_meshes(self):
        xml=ET.parse(self.model.urdf).getroot();by_shape={s['link']+'|'+s['name']:s for s in self.model.geometry['shapes']}
        cache={}
        for link in xml.findall('link'):
            for i,c in enumerate(link.findall('collision')):
                key=(link.get('name'),c.get('name',str(i)));g=c.find('geometry')[0]
                if g.tag=='mesh':
                    prefix='package://tianji_robot_description/';uri=g.get('filename');assert uri.startswith(prefix)
                    file=self.model.urdf.parent.parent/uri[len(prefix):];raw=file.read_bytes();relative=str(file.relative_to(self.model.urdf.parents[3]))
                    if hashlib.sha256(raw).hexdigest()!=self.model.manifest['mesh_sha256'][relative]:raise ValueError('模型网格哈希不匹配')
                    scale=tuple(np.fromstring(g.get('scale','1 1 1'),sep=' '));cachekey=(str(file),scale)
                    if cachekey not in cache:
                        count=struct.unpack_from('<I',raw,80)[0];assert len(raw)==84+50*count
                        rec=np.frombuffer(raw,dtype=[('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')],offset=84,count=count)
                        cache[cachekey]=np.ascontiguousarray(rec['v'].reshape(-1,3)*np.array(scale),np.float32)
                elif g.tag=='box':
                    size=np.fromstring(g.get('size'),sep=' ');cachekey=('box',*size)
                    if cachekey not in cache:
                        v,f=box_mesh([-size/2,size/2]);cache[cachekey]=np.ascontiguousarray(v[f].reshape(-1,3),np.float32)
                else:raise ValueError('Unsupported geometry')
                self.mesh_data[key]=(cachekey,cache[cachekey])
        from chassis_proxy import profile,PROXY_KEY
        self.chassis_profile=profile(self.model)
        vertices,faces=box_mesh(self.chassis_profile['bounds_m'])
        self.mesh_data[PROXY_KEY]=(PROXY_KEY,np.ascontiguousarray(vertices[faces].reshape(-1,3),np.float32))
    def initializeGL(self):
        self.renderer=glGetString(GL_RENDERER).decode();self.gl_version=glGetString(GL_VERSION).decode()
        glClearColor(.045,.075,.10,1);glEnable(GL_BLEND);glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);glEnable(GL_DEPTH_TEST)
        cache={}
        for key,(ck,v) in self.mesh_data.items():
            if ck not in cache:
                bid=glGenBuffers(1);glBindBuffer(GL_ARRAY_BUFFER,bid);glBufferData(GL_ARRAY_BUFFER,v.nbytes,v,GL_STATIC_DRAW);cache[ck]=(bid,len(v))
            self.mesh_buffers[key]=cache[ck]
        self.mesh_data={};glBindBuffer(GL_ARRAY_BUFFER,0)
        self.point_buffer=glGenBuffers(1);self.color_buffer=glGenBuffers(1)
        print('OPENGL '+self.renderer,flush=True)
    def render_if_dirty(self):
        if self.dirty:self.dirty=False;self.update()
    def set_data(self,frames,stale):
        self.frames=frames;self.stale=stale
        candidates=[f for s,f in frames.items() if self.enabled[s] and not stale.get(s,True)]
        if self.saved_positions is not None:
            from chassis_proxy import replace_shapes
            self.pose=dict(self.saved_positions);self.shapes=replace_shapes([s for s in self.model.shapes(self.pose) if s['mode']=='measured_or_fixed'],self.chassis_profile)
        elif candidates:
            latest=max(candidates,key=lambda f:f['header']['observation_time']);self.pose=dict(latest['header']['positions_used'])
            self.shapes=[s for s in self.model.shapes(self.pose) if s['mode']=='measured_or_fixed']
            if latest['header'].get('chassis_collision'):
                if latest['header']['chassis_collision']!=self.chassis_profile:raise ValueError('CHASSIS_PROXY_MISMATCH')
                from chassis_proxy import replace_shapes
                self.shapes=replace_shapes(self.shapes,self.chassis_profile)
        else:self.shapes=[]
        self.distances=merge_directions({s:f for s,f in frames.items() if self.enabled[s]},stale) if self.saved_positions is None else merge_directions({},{})
        self.far_clip=max(40.,self.distance+max((np.linalg.norm(f['xyz']-self.target,axis=1).max() for f in frames.values() if len(f['xyz'])),default=0.)*2)
        self.cloud_dirty=True;self.dirty=True
    def upload_cloud(self):
        vertices=[];colors=[];kinds=[]
        for sensor,f in self.frames.items():
            if not self.enabled[sensor]:continue
            labels=f['labels'];keep=np.ones(len(labels),bool)
            overlay=f.get('header',{}).get('cable_overlay',{});cables=np.zeros(len(labels),bool);table=np.zeros(len(labels),bool)
            cables[overlay.get('ignored_indices',[])]=True;table[overlay.get('protected_indices',[])]=True
            if not self.full_cloud:
                keep[:]=False;keep[preview_indices(f['xyz'],labels)]=True
            if not self.enabled['env']:keep&=(labels!=1)|table
            if not self.enabled['self']:keep&=(labels!=2)|cables
            if self.mode==2:keep&=(labels!=2)&~cables
            if self.mode==3 and overlay.get('navigation_filter_source')=='S100_V7_NAVIGATION_SELF_FILTER' and not overlay.get('navigation_filtered_indices_truncated'):
                keep[overlay.get('navigation_filtered_indices',[])]=False
            if not self.enabled['cables']:keep&=~cables
            if not self.enabled['table']:keep&=~table
            if not self.enabled['unknown']:keep&=labels!=5
            points=f['xyz'][keep];lab=labels[keep];rgba=COLORS[lab].copy()
            if self.mode==0:rgba[:]=(.6,.7,.75,.65)
            if self.mode!=0:
                rgba[cables[keep]]=(1.,.40,.06,1.)
                rgba[table[keep]]=(.86,.4,1.,1.)
            if self.stale.get(sensor,True):rgba[:]=(.35,.38,.4,.3)
            vertices.append(points);colors.append(rgba);kinds.append(lab==2)
        if vertices:
            xyz=np.concatenate(vertices);rgba=np.concatenate(colors);is_self=np.concatenate(kinds);order=np.argsort(is_self,kind='stable');xyz=np.ascontiguousarray(xyz[order],np.float32);rgba=np.ascontiguousarray(rgba[order],np.float32);self.count=len(xyz);self.self_count=int(is_self.sum())
        else:xyz=np.empty((0,3),np.float32);rgba=np.empty((0,4),np.float32);self.count=self.self_count=0
        glBindBuffer(GL_ARRAY_BUFFER,self.point_buffer);glBufferData(GL_ARRAY_BUFFER,xyz.nbytes,xyz,GL_DYNAMIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER,self.color_buffer);glBufferData(GL_ARRAY_BUFFER,rgba.nbytes,rgba,GL_DYNAMIC_DRAW);glBindBuffer(GL_ARRAY_BUFFER,0);self.cloud_dirty=False
    def paintGL(self):
        begin=time.perf_counter();glEnable(GL_BLEND);glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);glEnable(GL_DEPTH_TEST);glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);glViewport(0,0,self.width(),self.height());glMatrixMode(GL_PROJECTION);glLoadIdentity();gluPerspective(42,self.width()/max(1,self.height()),.05,max(self.far_clip,self.distance+10.))
        glMatrixMode(GL_MODELVIEW);glLoadIdentity()
        offset,up,_,_=basis(self.yaw,self.pitch,self.special_view);eye=self.target+self.distance*offset
        gluLookAt(*eye,*self.target,*up);glDepthMask(GL_FALSE);glColor4f(.17,.26,.31,.55);glLineWidth(1);glBegin(GL_LINES)
        for i in range(-10,11):
            k=i*.2;glVertex3f(k,-2.,0);glVertex3f(k,2.,0);glVertex3f(-2.,k,0);glVertex3f(2.,k,0)
        for color,end in [((.85,.3,.3),( .3,0,0)),((.3,.8,.4),(0,.3,0)),((.3,.5,.9),(0,0,.3))]:
            glColor3f(*color);glVertex3f(0,0,0);glVertex3f(*end)
        glEnd()
        if self.enabled['model']:
            glEnableClientState(GL_VERTEX_ARRAY);glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);glColor4f(.96,.8,.37,.3)
            for shape in self.shapes:
                proxy=shape['name']=='development_chassis_cuboid';glColor4f(.96,.8,.37,.95 if proxy else .3);glLineWidth(2. if proxy else 1.)
                bid,count=self.mesh_buffers[(shape['link'],shape['name'])];glBindBuffer(GL_ARRAY_BUFFER,bid);glVertexPointer(3,GL_FLOAT,0,None);glPushMatrix();glMultMatrixf(np.ascontiguousarray(np.array(shape['matrix'],np.float32).T));glDrawArrays(GL_TRIANGLES,0,count);glPopMatrix()
            glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);glDisableClientState(GL_VERTEX_ARRAY);glBindBuffer(GL_ARRAY_BUFFER,0)
        if self.cloud_dirty:self.upload_cloud()
        glDepthMask(GL_TRUE);glEnableClientState(GL_VERTEX_ARRAY);glEnableClientState(GL_COLOR_ARRAY)
        glBindBuffer(GL_ARRAY_BUFFER,self.point_buffer);glVertexPointer(3,GL_FLOAT,0,None);glBindBuffer(GL_ARRAY_BUFFER,self.color_buffer);glColorPointer(4,GL_FLOAT,0,None)
        glPointSize(self.point_size);glDrawArrays(GL_POINTS,0,self.count-self.self_count);glDisable(GL_DEPTH_TEST);glPointSize(self.point_size if self.mode==0 else max(3.,self.point_size));glDrawArrays(GL_POINTS,self.count-self.self_count,self.self_count);glEnable(GL_DEPTH_TEST)
        glDisableClientState(GL_VERTEX_ARRAY);glDisableClientState(GL_COLOR_ARRAY);glBindBuffer(GL_ARRAY_BUFFER,0)
        if self.enabled.get('zones',True):
            fresh=[f for sensor,f in self.frames.items() if self.enabled.get(sensor,False) and not self.stale.get(sensor,True) and f.get('header',{}).get('pose_source','realtime')=='realtime']
            if fresh:
                header=max(fresh,key=lambda f:f.get('header',{}).get('observation_time',0)).get('header',{});overlay=header.get('cable_overlay',{})
                glDisable(GL_DEPTH_TEST);glColor4f(.1,1.,.65,.9);glLineWidth(1.3)
                for center in overlay.get('centers_body_m',[]):
                    for a,b in [(0,1),(0,2),(1,2)]:
                        glBegin(GL_LINE_LOOP)
                        for angle in np.linspace(0,2*np.pi,24,endpoint=False):
                            p=np.asarray(center).copy();p[a]+=overlay['radius_m']*np.cos(angle);p[b]+=overlay['radius_m']*np.sin(angle);glVertex3dv(p)
                        glEnd()
                glLineWidth(1.);glEnable(GL_DEPTH_TEST)
        annotations=[]
        if self.enabled['distance']:
            glDisable(GL_DEPTH_TEST);glLineWidth(2.5)
            forward=self.target-eye;forward/=np.linalg.norm(forward);right=np.cross(forward,up);right/=np.linalg.norm(right);vertical=np.cross(right,forward)
            scale=self.height()/(2*np.tan(np.deg2rad(21)))
            for row in self.distances:
                if row['status']!='observed':continue
                a=np.array(row['robot_point']);b=np.array(row['object_point']);color=QColor(DIRECTION_COLORS[row['index']])
                glColor4f(color.redF(),color.greenF(),color.blueF(),1.);glBegin(GL_LINES);glVertex3dv(a);glVertex3dv(b);glEnd()
                glPointSize(8);glBegin(GL_POINTS);glVertex3dv(a);glVertex3dv(b);glEnd()
                v=(a+b)*.5-eye;depth=np.dot(v,forward)
                if depth>.05:
                    x=self.width()/2+np.dot(v,right)/depth*scale;y=self.height()/2-np.dot(v,vertical)/depth*scale
                    annotations.append((x,y,row,color))
            glLineWidth(1);glEnable(GL_DEPTH_TEST)
        if annotations:
            painter=QPainter(self);painter.setRenderHint(QPainter.Antialiasing);painter.setFont(QFont('Sans',11,QFont.Bold));occupied=[]
            for x,y,row,color in annotations:
                text=f"{row['name']} {row['distance_cm']:.1f} cm";width=painter.fontMetrics().horizontalAdvance(text)+14
                x=int(np.clip(x,4,max(4,self.width()-width-4)));y=int(np.clip(y,24,max(24,self.height()-6)))
                while any(abs(y-oy)<25 and abs(x-ox)<max(width,ow) for ox,oy,ow in occupied) and y<self.height()-30:y+=25
                occupied.append((x,y,width));painter.fillRect(x,y-20,width,25,QColor(12,23,32,225));painter.setPen(color);painter.drawText(x+7,y-2,text)
            painter.end()
        self.paint_ms=(time.perf_counter()-begin)*1000;self.paint_count+=1
    def mousePressEvent(self,e):
        if e.button() not in (Qt.LeftButton,Qt.MiddleButton):e.ignore();return
        self.last_mouse=e.pos()
        if e.button()==Qt.MiddleButton:self.setCursor(Qt.ClosedHandCursor)
        e.accept()
    def mouseMoveEvent(self,e):
        if self.last_mouse is None:return
        delta=e.pos()-self.last_mouse;self.last_mouse=e.pos()
        orbit=(e.buttons()&Qt.LeftButton) or ((e.buttons()&Qt.MiddleButton) and (e.modifiers()&Qt.ShiftModifier))
        if orbit:
            self.special_view=None;self.yaw-=delta.x()*.008;self.pitch=np.clip(self.pitch+delta.y()*.008,-1.4,1.4)
        elif e.buttons()&Qt.MiddleButton:
            self.target+=pan_delta(delta.x(),delta.y(),self.height(),self.distance,self.yaw,self.pitch,self.special_view)
        else:return
        self.dirty=True;e.accept()
    def mouseReleaseEvent(self,e):self.last_mouse=None;self.unsetCursor();e.accept()
    def wheelEvent(self,e):self.distance=float(np.clip(self.distance*np.exp(-e.angleDelta().y()*.001),.5,300.));self.dirty=True
    def view(self,name):
        if name=='cloud':
            points=[f['xyz'] for sensor,f in self.frames.items() if self.enabled[sensor] and len(f['xyz'])]
            if points:
                points=np.concatenate(points);self.target=(points.min(0)+points.max(0))/2;radius=np.linalg.norm(points-self.target,axis=1).max();self.distance=float(np.clip(radius/np.sin(np.deg2rad(21))*1.12,1.,300.));self.far_clip=self.distance+radius*2+10.
            self.special_view=None
        elif name in ('all','robot'):
            points=[np.array(s['world_corners']) for s in self.shapes]
            if name=='all':points.extend(np.array([r['robot_point'],r['object_point']]) for r in self.distances if r['status']=='observed')
            if points:
                points=np.concatenate(points);self.target=(points.min(0)+points.max(0))/2
                radius=np.linalg.norm(points-self.target,axis=1).max();self.distance=float(np.clip(radius/np.sin(np.deg2rad(21))*1.12,1.,12.))
            elif name=='robot':
                self.target=np.array([.12,0.,.74]);self.distance=2.8
            self.special_view=None
        else:self.special_view=name
        self.dirty=True


class Window(QMainWindow):
    def __init__(self,model,config,capture=None):
        super().__init__();self.setWindowTitle('S100 实时三维自体识别 · 只读');self.resize(1380,900);self.scene=Scene(model);self.receiver=Receiver(model.sha256,config);self.receiver.start();self.keys={};self.last_stale={};self.history=[];self.capture_path=capture;self.force_scene_update=True;self.initial_robot_fit=False
        base=QWidget();self.setCentralWidget(base);layout=QVBoxLayout(base);self.top=QLabel('正在连接 S100…');self.top.setStyleSheet('font-size:18px;padding:10px;background:#253644');layout.addWidget(self.top)
        self.top.setFixedHeight(52)
        row=QHBoxLayout();self.mode=QComboBox();self.mode.addItems(['自体识别','原始点云','筛后预览（仅隐藏候选）','导航障碍层（与S100同规则）']);row.addWidget(self.mode);self.mode.currentIndexChanged.connect(self.change_mode)
        for title,name in [('三维',None),('俯视 XY','top'),('侧视 XZ','side'),('正视 YZ','front'),('整机视角','robot'),('八向全览','all'),('点云全览','cloud')]:
            b=QPushButton(title);b.clicked.connect(lambda _,n=name:self.scene.view(n));row.addWidget(b)
        history_button=QPushButton('历史点云查看器…');history_button.clicked.connect(self.open_cloud_library);row.addWidget(history_button)
        anomalies=QPushButton('异常点云…');anomalies.clicked.connect(self.open_conflicts);row.addWidget(anomalies)
        row.addStretch();layout.addLayout(row)
        source_row=QHBoxLayout();source_row.addWidget(QLabel('关节来源：'))
        self.pose_source=QComboBox();self.pose_source.addItems(['实时关节（允许控制器反馈连接）','保存姿态（停用只读反馈连接）']);self.pose_source.setCurrentIndex(0 if store.mode()=='realtime' else 1);source_row.addWidget(self.pose_source)
        self.pose_source.currentIndexChanged.connect(self.select_pose_source)
        save_button=QPushButton('保存当前关节角度');save_button.clicked.connect(self.save_pose);source_row.addWidget(save_button)
        self.pose_caption=QLabel('双臂14轴；保存值仅供静态查看');source_row.addWidget(self.pose_caption);source_row.addStretch()
        self.connection_button=QPushButton('断开 S100');self.connection_button.clicked.connect(self.toggle_connection);source_row.addWidget(self.connection_button);layout.addLayout(source_row)
        try:self.refresh_pose_source()
        except Exception as exc:self.pose_caption.setText(str(exc))
        density_row=QHBoxLayout();self.full_toggle=QCheckBox('完整单帧有效点云（不裁剪、不抽点）');self.full_toggle.setChecked(True);self.full_toggle.toggled.connect(self.set_full_cloud);density_row.addWidget(self.full_toggle)
        density_row.addWidget(QLabel('点大小（像素）：'));size=QDoubleSpinBox();size.setRange(1.,6.);size.setSingleStep(.5);size.setValue(1.);size.valueChanged.connect(self.set_point_size);density_row.addWidget(size);density_row.addWidget(QLabel('统计为单帧真实回波；点间距不是净空距离'));density_row.addStretch();layout.addLayout(density_row)
        split=QSplitter();split.addWidget(self.scene);panel=QWidget();side=QVBoxLayout(panel)
        for title,key in [('前雷达','front'),('后雷达','back'),('黄色 URDF 网格','model'),('灰色环境点','env'),('青色自体候选','self'),('蓝色未知点','unknown'),('橙色线缆候选','cables'),('紫色桌面/表面保留点','table'),('绿色局部线缆候选区','zones'),('八方向距离连线','distance')]:
            c=QCheckBox(title);c.setChecked(key!='zones');
            c.toggled.connect(lambda value,k=key:self.layer(k,value));side.addWidget(c)
        self.cable_status=QLabel('线缆候选区：等待 S100 实时结果');self.cable_status.setWordWrap(True);self.cable_status.setTextInteractionFlags(Qt.TextSelectableByMouse);side.addWidget(self.cable_status)
        self.raw_stats=QLabel('等待完整点云统计…');self.raw_stats.setWordWrap(True);self.raw_stats.setTextInteractionFlags(Qt.TextSelectableByMouse);side.addWidget(self.raw_stats)
        caption=QLabel('双臂及末端 · 水平八向距离（cm）');side.addWidget(caption)
        compass=QGridLayout();self.direction_labels={}
        for index,(r,c) in {1:(0,0),0:(0,1),7:(0,2),2:(1,0),6:(1,2),3:(2,0),4:(2,1),5:(2,2)}.items():
            label=QLabel(NAMES[index]+'\n未知');label.setAlignment(Qt.AlignCenter);label.setMinimumHeight(45);label.setStyleSheet('color:'+DIRECTION_COLORS[index]+';background:#1e303d;padding:3px;font-size:13px');compass.addWidget(label,r,c);self.direction_labels[index]=label
        center=QLabel('↑ 前\n双臂');center.setAlignment(Qt.AlignCenter);compass.addWidget(center,1,1);side.addLayout(compass)
        self.info=QLabel();self.info.setWordWrap(True);self.info.setTextInteractionFlags(Qt.TextSelectableByMouse);side.addWidget(self.info);side.addStretch()
        note=QLabel('鼠标中键拖动平移；Shift＋中键或左键旋转，滚轮缩放。\n\n每雷达 1 Hz 更新；界面只保留最新帧。过期或断线的数据灰显。\n\n底盘碰撞代理：68 × 64 × 39 cm 长方体。\n八向以底盘 X 前、Y 左划分。只测双臂所在高度的有效回波；无回波显示未知。当前外参下的估计值，不代表可通行距离。\n\n完整显示每帧所有有效回波；零距离和非有限值不显示。统计按完整单帧计算，界面每雷达1Hz更新。');note.setWordWrap(True);side.addWidget(note);panel.setMinimumWidth(320);scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(panel);scroll.setMinimumWidth(340);scroll.setMaximumWidth(400);split.addWidget(scroll);layout.addWidget(split,1);split.setSizes([1000,320])
        self.mode.setCurrentIndex(3)
        self.bottom=QLabel('准备加载模型…');self.bottom.setMaximumHeight(30);layout.addWidget(self.bottom);self.timer=QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(500);self.last_cpu=(time.monotonic(),time.process_time());self.cpu=0.;self.started=time.monotonic();self.captured=False
        self.setStyleSheet('QMainWindow,QWidget{background:#101c26;color:#dae6ed}QCheckBox{padding:4px}QPushButton,QComboBox{background:#263c4b;border:1px solid #466171;padding:7px;border-radius:3px}QLabel{line-height:1.5}')
    def open_conflicts(self):
        process=getattr(self,'conflict_process',None)
        if process is not None and process.poll() is None:
            QMessageBox.information(self,'异常点云','异常点云窗口已打开，可从任务栏切换。');return
        viewer=ROOT.parent/'navigation_jetson_v5/modules/conflict_viewer.py'
        self.conflict_process=subprocess.Popen([sys.executable,str(viewer)],start_new_session=True)

    def open_cloud_library(self):
        # Separate process avoids changing live scene colors or receiver state.
        process=getattr(self,'archive_process',None)
        if process is not None and process.poll() is None:
            QMessageBox.information(self,'历史点云','历史点云窗口已打开，可通过任务栏切换。');return
        viewer=ROOT.parent/'navigation_jetson_v5/modules/map_gui.py'
        self.archive_process=subprocess.Popen([sys.executable,str(viewer)],cwd=str(viewer.parent),start_new_session=True)

    def set_full_cloud(self,value):self.scene.full_cloud=value;self.scene.cloud_dirty=True;self.scene.dirty=True
    def set_point_size(self,value):self.scene.point_size=value;self.scene.dirty=True
    def refresh_pose_source(self):
        packet=source_packet(self.scene.model.sha256)
        if packet['mode']=='saved':
            p=packet['snapshot'];self.scene.saved_positions=dict(p['joint_positions_rad']);self.pose_caption.setText('保存于 '+time.strftime('%m-%d %H:%M:%S',time.localtime(p['observation_time']))+' · 非实时（头部/夹爪未知）')
        else:self.scene.saved_positions=None;self.pose_caption.setText('实时实测关节；运动由外部控制器负责')
        self.force_scene_update=True
    def select_pose_source(self,index):
        try:
            if index==1 and store.load_saved().get('model_sha256')!=self.scene.model.sha256:raise ValueError('保存姿态与当前URDF版本不匹配')
            store.set_mode('realtime' if index==0 else 'saved');self.refresh_pose_source()
        except Exception as exc:
            self.pose_source.blockSignals(True);self.pose_source.setCurrentIndex(0 if store.mode()=='realtime' else 1);self.pose_source.blockSignals(False);QMessageBox.warning(self,'关节来源未切换',str(exc))
    def save_pose(self):
        try:
            path,pose=save_current();self.refresh_pose_source();self.pose_caption.setText('已保存 '+time.strftime('%H:%M:%S',time.localtime(pose['observation_time']))+' · 双臂14轴')
        except Exception as exc:QMessageBox.warning(self,'姿态未保存',str(exc))
    def toggle_connection(self):
        connect=not self.receiver.enabled.is_set();self.receiver.set_connected(connect);self.connection_button.setText('断开 S100' if connect else '连接 S100');self.force_scene_update=True
    def layer(self,key,value):
        self.scene.enabled[key]=value;self.scene.set_data(self.scene.frames,self.scene.stale)
    def change_mode(self,index):
        self.scene.mode=[1,0,2,3][index];self.receiver.display_mode='navigation' if index==3 else 'recognition';self.scene.cloud_dirty=True;self.scene.dirty=True
    def tick(self):
        frames,info,error=self.receiver.snapshot();mono=time.monotonic();now=time.time();stale={};changed=False;lines=[]
        for s,title in [('front','前雷达'),('back','后雷达')]:
            f=frames.get(s)
            if not f:stale[s]=True;lines.append(title+'：等待数据');continue
            age=frame_age(f['header'],now);stale[s]=(not frame_fresh(f['header'],now,mono-f['arrival']) or f['header'].get('pose_source','realtime')!=('saved' if self.scene.saved_positions is not None else 'realtime'))
            if self.scene.mode==3 and self.scene.saved_positions is None:
                overlay=f['header'].get('cable_overlay',{});stale[s]|=overlay.get('navigation_filter_source')!='S100_V7_NAVIGATION_SELF_FILTER' or bool(overlay.get('navigation_filtered_indices_truncated'))
            changed|=self.keys.get(s)!=f['key'];self.keys[s]=f['key'];counts=f['header']['original_counts'];lines.append(f'{title}：'+('过期' if stale[s] else '实时')+f' {age:.1f}s\n  自体 {counts["2"]} / 未知 {counts["5"]}\n  显示 {len(f["xyz"])} 点')
        changed|=stale!=self.last_stale or self.force_scene_update;self.force_scene_update=False;self.last_stale=stale
        if changed:self.scene.set_data(frames,stale)
        cable_lines=[];cable_radii=set()
        states={'FROZEN_USER_CABLE_ZONES':'当前姿态局部候选区','POSTURE_CHANGED_KEEP_ALL':'姿态变化，局部区停用','ANCHOR_UNAVAILABLE_KEEP_ALL':'关节/部件缺失，局部区停用','MODEL_CHANGED_KEEP_ALL':'模型变化，局部区停用','SAVED_POSTURE_DISPLAY_ONLY':'保存姿态，局部区停用','DISABLED':'局部区未启用','ZONE_BUDGET_EXCEEDED_KEEP_ALL':'候选过多，局部区保留全部'}
        for sensor,title in [('front','前'),('back','后')]:
            h=frames.get(sensor,{}).get('header',{});overlay=h.get('cable_overlay')
            if not overlay:continue
            if overlay.get('radius_m',0)>0:cable_radii.add(round(overlay['radius_m']*100,2))
            state=states.get(overlay.get('status'),overlay.get('status','未知'))
            cable_lines.append(f"{title}：{state}"+('（旧数据）' if stale.get(sensor,True) else '')+f"\n  线缆 {overlay.get('candidate_count',0)} / 近表面自体候选 {overlay.get('surface_candidate_count',0)}\n  表面保留 {overlay.get('protected_count',0)} 点 · {overlay.get('compute_ms',0):.0f} ms")
        radius_text='/'.join(f'{radius:g}' for radius in sorted(cable_radii)) or '—'
        self.cable_status.setText('S100 线缆/桌面优先保留\n'+('\n'.join(cable_lines) if cable_lines else '等待实时结果')+f'\n绿圈半径{radius_text}cm，仅限标定姿态；桌面结构优先保留。\n近表面复核保留原始占用，不作放行依据。\n筛后为预览；不清除线缆背后的未知空间。')
        if not self.initial_robot_fit and len(frames)==2 and not any(stale.values()):self.scene.view('robot');self.initial_robot_fit=True
        density=[]
        for sensor,title in [('front','前雷达'),('back','后雷达')]:
            frame=frames.get(sensor)
            if not frame:continue
            h=frame['header'];ranges=h.get('measured_range_m',[]);title+='（过期）' if stale.get(sensor,True) else '（实时）'
            density.append(f"{title}：原始 {h.get('raw_point_count','?')} / 有效 {h.get('valid_return_count',len(frame['xyz']))} / 收到 {len(frame['xyz'])}")
            if ranges:density.append(f'量程：{ranges[0]:.2f}–{ranges[3]:.2f} m；中位 {ranges[1]:.2f} m')
            for band in frame.get('sampling',[]):
                name=f"{band['low_m']:g}–{band['high_m']:g}m" if band['high_m'] is not None else '≥8m'
                spacing=f"{band['median_neighbor_cm']:.1f}cm" if band['median_neighbor_cm'] is not None else '—'
                density.append(f"  {name}: {band['points']}点 / 近邻间距 {spacing}")
        self.raw_stats.setText('完整单帧回波 / 近邻间距中位数\n'+'\n'.join(density))
        for row in self.scene.distances:
            label=self.direction_labels[row['index']];label.setText(row['name']+'\n'+(f"{row['distance_cm']:.1f} cm" if row['status']=='observed' else '未知'))
            label.setToolTip(('来源：'+row.get('sensor_id','')+' / '+row.get('link','')) if row['status']=='observed' else '该方向无新鲜、有效且有邻近支持的回波')
        fresh=sum(not x for x in stale.values());removed=sum(f['header'].get('removed_points',0) for f in frames.values());self.top.setText(('实时接收 '+str(fresh)+'/2 路' if fresh else '等待 / 数据过期')+f'  ·  开发显示  ·  实际删除 {removed} 点');self.top.setStyleSheet('font-size:18px;padding:10px;background:'+('#17473f' if fresh==2 else '#58452a'))
        if self.scene.mode==3:
            hidden=sum(f['header'].get('cable_overlay',{}).get('navigation_filtered_count',0) for sensor,f in frames.items() if not stale.get(sensor,True))
            self.top.setText(('实时接收 '+str(fresh)+'/2 路' if fresh else '等待导航同源滤除结果')+f' · 导航障碍层预览 · 隐藏自身/线材 {hidden} 点')
        if self.scene.saved_positions is not None:
            self.top.setText(('雷达实时 '+str(fresh)+'/2 路' if fresh else ('已断开 S100' if not self.receiver.enabled.is_set() else '等待雷达数据'))+' · 保存姿态（非实时） · 不作实时碰撞放行');self.top.setStyleSheet('font-size:18px;padding:10px;background:#58452a')
        elapsed=mono-self.last_cpu[0]
        if elapsed>=2.:
            self.cpu=100*(time.process_time()-self.last_cpu[1])/elapsed;self.last_cpu=(mono,time.process_time())
        metrics=info.get('load',{});cpu_text=(f'S100 整机 CPU：{metrics["cpu_all_percent"]:.1f}%\nS100 应用核：{metrics["cpu_app_percent"]:.1f}%' if metrics else '等待 S100 负载测量')
        offset=next(iter(frames.values()))['header']['clock_mapping']['remote_minus_local_s'] if frames else 0.
        if metrics and now-offset-metrics.get('updated_at',0)>10:cpu_text+='（旧测量）'
        reason=info.get('status',{}).get('reason','等待计算状态');self.info.setText('\n\n'.join(lines)+'\n\n'+cpu_text+f'\n本地查看器 CPU：单核 {self.cpu:.1f}%\n渲染提交：{self.scene.paint_ms:.1f} ms\n\n计算状态：{reason}'+ ('\n\n连接：'+error if error else ''))
        self.bottom.setText('GPU：'+self.scene.renderer+'  |  仅在新数据或视角变化时重绘  |  本地显示，不控制机器人')
        if elapsed>=2.:
            row=dict(monotonic=mono,elapsed_s=mono-self.started,local_cpu_one_core_percent=self.cpu,paint_submit_ms=self.scene.paint_ms,paint_count=self.scene.paint_count,fresh_sensors=fresh,renderer=self.scene.renderer,s100=metrics,error=error,pose_source='saved' if self.scene.saved_positions is not None else 'realtime',s100_connection_requested=self.receiver.enabled.is_set(),displayed_points=self.scene.count,full_cloud=self.scene.full_cloud,raw_clouds={k:dict(point_count=len(f['xyz']),source_valid=f['header'].get('valid_return_count'),range_m=f['header'].get('measured_range_m'),sampling=f.get('sampling')) for k,f in frames.items()},eight_direction=self.scene.distances,eight_compute_ms={k:f['header'].get('eight_direction',{}).get('compute_ms') for k,f in frames.items()})
            self.history.append(row)
            row['display_mode']='navigation' if self.scene.mode==3 else 'recognition'
            row['cable_overlays']={sensor:dict(frame['header'].get('cable_overlay',{}),stale=stale.get(sensor,True)) for sensor,frame in frames.items()}
            (ROOT/'results/local_viewer_metrics.json').write_text(json.dumps(self.history[-300:],indent=2)+'\n')
        if self.capture_path and not self.captured and mono-self.started>20 and fresh:
            self.capture();self.captured=True
    def capture(self):
        image=self.grab().toImage();p=QPainter(image);p.drawImage(self.scene.mapTo(self,QPoint(0,0)),self.scene.grabFramebuffer());p.end();image.save(str(self.capture_path))
    def closeEvent(self,event):
        self.timer.stop();self.receiver.stop_event.set();self.receiver.join(timeout=5.);event.accept()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--capture',type=Path);parser.add_argument('--test-exit-seconds',type=int,default=0);args=parser.parse_args()
    fmt=QSurfaceFormat();fmt.setVersion(2,1);fmt.setProfile(QSurfaceFormat.CompatibilityProfile);fmt.setSamples(0);fmt.setSwapInterval(1);QSurfaceFormat.setDefaultFormat(fmt)
    app=QApplication([]);lock=QLockFile('/run/user/'+str(os.getuid())+'/tianji_s100_cloud_viewer.lock')
    if not lock.tryLock(0):raise SystemExit('S100 三维查看器已运行')
    model=LocalRobotModel(default_urdf_path(),ROOT/'model_manifest.json',ROOT/'collision_geometry.json');config=json.loads((ROOT/'viewer_connection.json').read_text());w=Window(model,config,args.capture);w.show()
    signal.signal(signal.SIGTERM,lambda *_:w.close())
    signal.signal(signal.SIGINT,lambda *_:w.close())
    if args.test_exit_seconds:QTimer.singleShot(args.test_exit_seconds*1000,w.close)
    app.exec_()


if __name__=='__main__':main()
