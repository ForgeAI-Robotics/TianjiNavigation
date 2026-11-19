"""Desktop viewer for the S100 read-only live collision feed."""
import json
import shlex
import sys
import time
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QSurfaceFormat
from PyQt5.QtWidgets import (QApplication, QCheckBox, QHBoxLayout, QLabel,
                             QPushButton, QVBoxLayout, QWidget)

from onboard_client import REMOTE, connect
from paths import ROOT
from conflict_viewer import ConflictScene, LocalRobotModel, V2, default_urdf_path

STATES = {
    'OK': '当前帧通过',
    'LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY': '实时点云拦截',
    'LIDAR_STALE': '点云过期',
    'WAIT_LIDAR': '等待双雷达及关节反馈',
    'ODOMETRY_UNAVAILABLE': '里程计不可用或参考已过期',
}


class Feed(QThread):
    packet = pyqtSignal(dict)

    def run(self):
        try:
            pointer = json.loads((ROOT / 'runtime/latest_plan_office_1.json').read_text())
            reference = Path(pointer['remote_folder'])
            if reference.parent != Path(REMOTE) / 'results':
                raise ValueError('规划参考目录无效')
            script = Path(__file__).with_name('live_guard_feed.py').read_text()
            command = (
                'cd ' + shlex.quote(REMOTE) + ' && source /opt/ros/humble/setup.bash && '
                'export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp '
                'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CYCLONEDDS_URI="file://$PWD/config/input_cyclonedds.xml"; '
                'PYTHONPATH=modules:$PYTHONPATH /usr/bin/python3 -u -c '
                + shlex.quote(script) + ' ' + shlex.quote(str(reference))
            )
            with connect() as connection:
                stdin, stdout, stderr = connection.exec_command(command, timeout=10)
                try:
                    for line in stdout:
                        if self.isInterruptionRequested():
                            break
                        self.packet.emit(json.loads(line))
                    if not self.isInterruptionRequested():
                        error = stderr.read().decode().strip().splitlines()
                        self.packet.emit({'reason': error[-1] if error else '实时数据流已结束'})
                finally:
                    stdin.channel.shutdown_write()
                    stdout.channel.close()
        except Exception as exc:
            self.packet.emit({'reason': str(exc)})


class Window(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('实时雷达拦截点 · 办公室 1 号位 · 静止诊断')
        self.resize(1330, 850)
        self.model = LocalRobotModel(default_urdf_path(), V2/'model_manifest.json', V2/'collision_geometry.json')
        self.scene = ConflictScene(self.model)
        self.scene.show_boxes = self.scene.show_lines = self.scene.show_nearest = False
        layout = QVBoxLayout(self)
        self.status = QLabel('正在连接 S100…')
        self.status.setStyleSheet('font-size:18px;padding:8px')
        layout.addWidget(self.status)
        layout.addWidget(QLabel('红色为当前触发拦截的雷达回波，黄色为当前实测姿态的 URDF。静止模拟向右横移 1 cm/s。'))
        row = QHBoxLayout()
        layout.addLayout(row)
        for label, view in [('三维', None), ('俯视 XY', 'top'), ('侧视 XZ', 'side'), ('正视 YZ', 'front')]:
            button = QPushButton(label)
            button.clicked.connect(lambda _, v=view: self.scene.view(v))
            row.addWidget(button)
        for label, handler in [('整机视角', self.fit_robot), ('聚焦异常', self.fit_points)]:
            button = QPushButton(label)
            button.clicked.connect(handler)
            row.addWidget(button)
        model = QCheckBox('原 URDF 网格')
        model.setChecked(True)
        model.toggled.connect(self.toggle_model)
        row.addWidget(model)
        row.addStretch()
        layout.addWidget(self.scene, 1)
        layout.addWidget(QLabel('左键旋转 · 中键平移 · 滚轮缩放 ｜ 启动实车导航前请关闭此窗口'))
        self.setStyleSheet('QWidget{background:#101c26;color:#dae6ed}QPushButton{background:#263c4b;padding:6px}')
        self.fit_robot()
        self.last_update = time.monotonic()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_connection)
        self.timer.start(500)
        self.feed = Feed(self)
        self.feed.packet.connect(self.show_packet)
        self.feed.start()

    def fit_robot(self):
        self.scene.target = np.array([.12, 0., .65])
        self.scene.distance = 2.6
        self.scene.dirty = True

    def fit_points(self):
        points = self.scene.points
        if len(points):
            self.scene.target = (points.min(0) + points.max(0)) / 2
            self.scene.distance = 1.55
            self.scene.dirty = True

    def toggle_model(self, enabled):
        self.scene.enabled['model'] = enabled
        self.scene.dirty = True

    def show_packet(self, packet):
        self.last_update = time.monotonic()
        reason = packet['reason']
        text = STATES.get(reason, reason)
        groups = packet.get('groups', {})
        if 'age_s' in packet:
            text += f"  ｜  帧龄 {packet['age_s']:.2f} s  ｜  拦截点 {packet['count']}"
        if groups:
            text += '  ｜  ' + '、'.join(groups)
        if 'joints' in packet and packet.get('model_sha256') == self.model.sha256:
            self.scene.shapes = [shape for shape in self.model.shapes(packet['joints'])
                                 if shape['mode'] == 'measured_or_fixed']
            points = np.array([p for group in groups.values() for p in group]).reshape(-1, 3)
            self.scene.points = np.unique(points, axis=0)
        else:
            self.scene.shapes = []
            self.scene.points = np.empty((0, 3))
            if 'joints' in packet:
                text = 'S100 与本机 URDF 版本不一致'
        self.status.setText(text)
        self.status.setStyleSheet('font-size:18px;padding:8px;color:' + ('#69dba8' if reason == 'OK' else '#ff8b82'))
        self.scene.dirty = True

    def check_connection(self):
        if time.monotonic() - self.last_update > 2.5 and self.feed.isRunning():
            self.show_packet({'reason': '等待实时数据'})

    def closeEvent(self, event):
        self.feed.requestInterruption()
        self.feed.wait(11000)
        super().closeEvent(event)


if __name__ == '__main__':
    fmt = QSurfaceFormat()
    fmt.setVersion(2, 1)
    fmt.setProfile(QSurfaceFormat.CompatibilityProfile)
    fmt.setSamples(0)
    QSurfaceFormat.setDefaultFormat(fmt)
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
