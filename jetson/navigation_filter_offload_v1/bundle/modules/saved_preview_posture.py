"""Historical measured posture is available only to explicit read-only preview."""
import json,math
from pathlib import Path
from preview_posture import ARMS

def load(root,model_sha256):
 root=Path(root);files=sorted((root/'results').glob('*/localization/localization.json'),key=lambda p:p.stat().st_mtime,reverse=True)
 for file in files:
  try:
   loc=json.loads(file.read_text());raw=loc['saved_pose'];q=raw['joint_positions_rad']
   if loc.get('historical_posture_only')or raw.get('historical_posture_only'):continue
   if raw['model_sha256']!=model_sha256:continue
   if any(k not in q or not math.isfinite(q[k])for k in ARMS):continue
   if not isinstance(raw.get('observation_time'),(int,float)):continue
   return dict(raw,source='SAVED_POSTURE_PREVIEW_ONLY',historical_posture_only=True,saved_posture_file=str(file),execution_permission=False)
  except (OSError,ValueError,KeyError,TypeError):continue
 raise ValueError('NO_SAVED_MEASURED_ARM_POSTURE_FOR_MODEL')
