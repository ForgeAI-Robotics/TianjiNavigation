"""Explicit user-authorized test release, kept separate from raw sensor state."""
import json,time
from pathlib import Path
from common import ROOT,atomic_json,exclusive
from targets import scene_config
FILE=scene_config('clearance_override.json')
def settings():
 try:return json.loads(FILE.read_text())
 except (OSError,ValueError):return dict(enabled=False)
def effective(raw_camera_clear,raw_shapemask_clear,map_hash,model_hash,cfg=None):
 cfg=settings() if cfg is None else cfg
 active=bool(cfg.get('enabled') and cfg.get('map_sha256')==map_hash and cfg.get('model_sha256')==model_hash)
 camera=bool(raw_camera_clear or active and cfg.get('force_camera_clearance'))
 shapemask=bool(raw_shapemask_clear or active and cfg.get('force_shapemask_release'))
 return dict(clearance_clear=camera,shapemask_clear=shapemask,forced_release_active=active,
  release_state='FORCED_CLEAR_TEST' if active else 'MEASURED_CLEAR' if camera and shapemask else 'NOT_CLEAR',
  raw_clearance_clear=bool(raw_camera_clear),raw_shapemask_clear=bool(raw_shapemask_clear),
  source='USER_AUTHORIZED_TEST_OVERRIDE' if active else 'SENSOR',map_sha256=map_hash,model_sha256=model_hash,
  bypasses_localization=False,bypasses_arm_feedback=False,bypasses_lidar_obstacles=False,bypasses_link_watchdog=False)
def configure(enabled):
 with exclusive(ROOT/'runtime/clearance_override.lock'):
  from targets import get
  row=get(1);before=settings()
  cfg=dict(enabled=bool(enabled),force_camera_clearance=True,force_shapemask_release=True,map_sha256=row['map_voxel_sha256'],model_sha256=json.loads((ROOT/'config/self_filter_envelope.json').read_text())['model_sha256'],updated_wall_time=time.time(),reason='User requested assumed clearance and forced ShapeMask release for navigation integration tests')
  atomic_json(FILE,cfg)
  with (ROOT/'runtime/clearance_override_audit.jsonl').open('a') as f:f.write(json.dumps(dict(time=time.time(),before=before,after=cfg),ensure_ascii=False)+'\n')
  return cfg
