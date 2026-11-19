"""Persisted debug frontend transport; navigation always computes on S100."""
import json
from paths import ROOT
from common import atomic_json
FILE=ROOT/'config/task_backend.json'
def load():
 value=json.loads(FILE.read_text())if FILE.exists()else dict(backend='legacy')
 if value.get('backend')not in ['legacy','api','bus']:raise ValueError('UNKNOWN_NAVIGATION_TASK_BACKEND')
 return value
def enabled():return load()['backend']in ('api','bus')
def save(value):
 if value not in ['legacy','api','bus']:raise ValueError('UNKNOWN_NAVIGATION_TASK_BACKEND')
 atomic_json(FILE,dict(backend=value));return value
