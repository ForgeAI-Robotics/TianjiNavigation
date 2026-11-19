"""Version-pinned optional S100 controller activation before live navigation."""
import hashlib,json,os,subprocess,sys
from pathlib import Path
from common import ROOT
CONFIG=ROOT/'config/controller_extension.json'
def ensure():
 cfg=json.loads(CONFIG.read_text())if CONFIG.exists()else dict(enabled=False)
 if not cfg.get('enabled'):return dict(active=False,reason='ORIGINAL_CONTROLLER_SELECTED')
 binary=Path(cfg['binary']);manager=Path(cfg['manager'])
 if hashlib.sha256(binary.read_bytes()).hexdigest()!=cfg['binary_sha256']:raise ValueError('CONTROLLER_EXTENSION_BINARY_MISMATCH')
 env=dict(os.environ);prefix=Path('/home/sunrise/robot/bin/interface/install');packages=[p for p in prefix.iterdir()if p.is_dir()and(p/'share/ament_index').exists()]
 for key,paths in [('AMENT_PREFIX_PATH',[str(p)for p in packages]),('PYTHONPATH',[str(p/'local/lib/python3.10/dist-packages')for p in packages if(p/'local/lib/python3.10/dist-packages').exists()]),('LD_LIBRARY_PATH',[str(p/'lib')for p in packages if(p/'lib').exists()])]:env[key]=':'.join(paths+([env[key]]if env.get(key)else []))
 p=subprocess.run([sys.executable,str(manager)],env=env,capture_output=True,text=True,timeout=90)
 if p.returncode:raise RuntimeError('CONTROLLER_EXTENSION_ACTIVATION_FAILED: '+p.stderr[-800:])
 result=json.loads(p.stdout.strip().splitlines()[-1])
 if not result.get('active')or result['binary_sha256']!=cfg['binary_sha256']or result.get('executable')!=str(binary.resolve())or not healthy(result):raise RuntimeError('CONTROLLER_EXTENSION_NOT_VERIFIED')
 return result

def healthy(reference):
 if not reference or not reference.get('active'):return True
 try:
  p=Path('/proc')/str(reference['pid']);fields=(p/'stat').read_text().split()
  return fields[2]!='Z'and fields[21]==reference['start_ticks']and(p/'exe').stat().st_ino==reference['inode']and str((p/'exe').resolve())==reference['executable']
 except (OSError,ValueError,KeyError):return False
