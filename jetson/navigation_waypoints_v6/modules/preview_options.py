"""Persisted switches for read-only path previews; never used by live control."""
import json
from paths import ROOT
from common import atomic_json
FILE=ROOT/'config/preview_options.json'
DEFAULT=dict(ignore_head_camera=True,open_grippers=True,use_saved_arms=False)
def validate(value):
 value=dict(value);value.setdefault('use_saved_arms',False)
 if set(value)!=set(DEFAULT)or any(type(v)is not bool for v in value.values()):raise ValueError('INVALID_PREVIEW_SWITCHES')
 return dict(value)
def load():return validate(json.loads(FILE.read_text()))if FILE.exists()else dict(DEFAULT)
def save(value):value=validate(value);atomic_json(FILE,value);return value
def caption(value):
 value=validate(value)
 return '头部相机/ShapeMask：'+('忽略'if value['ignore_head_camera']else'要求实测通过')+'；夹爪：'+('按开启状态'if value['open_grippers']else'实测开合')+('；双臂：已保存姿态（仅预览，不代表当前）'if value['use_saved_arms']else'；双臂：实时反馈')
