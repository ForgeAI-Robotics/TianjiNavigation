"""Shared original navigation route image widget; both tabs use this class."""
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QLabel,QSizePolicy

class RouteImage(QLabel):
 def __init__(self):
  super().__init__('等待规划结果')
  self.original=QPixmap()
  self.setAlignment(Qt.AlignCenter)
  self.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored)
 def setPixmap(self,pixmap):
  self.original=pixmap
  self.fit()
 def clear(self):
  self.original=QPixmap()
  self.setText('等待规划结果')
 def fit(self):
  if not self.original.isNull():
   super().setPixmap(self.original.scaled(self.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
 def resizeEvent(self,event):
  super().resizeEvent(event)
  self.fit()

