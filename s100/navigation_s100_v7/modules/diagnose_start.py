from pathlib import Path
import json,math
import numpy as np
from paths import ROOT
f=Path(json.loads((ROOT/'runtime/latest_plan.json').read_text())['folder']);w=json.loads((f/'scene/immutable_world.json').read_text());p=np.array(w['occupied_points']);pad=.05+math.sqrt(3)*w['resolution_m']/2
for pose in [[.025,0,0],[-.025,0,0],[0,.025,0],[0,-.025,0],[0,0,.04],[0,0,-.04]]:
 a=pose[2];rot=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]]);contacts=[]
 for s in w['shapes']:
  T=np.array(s['matrix']);R=rot@T[:3,:3];t=rot@T[:3,3]+np.r_[pose[:2],0.];q=(p-t)@R;lo,hi=np.array(s['bounds']);gap=np.linalg.norm(np.maximum(np.maximum(lo-q,q-hi),0),axis=1);ids=np.flatnonzero(gap<=pad+.01)
  if len(ids):contacts.append(dict(link=s['link'],points=p[ids].tolist()[:5],count=len(ids),gap=float(gap[ids].min())))
 print(json.dumps(dict(pose=pose,contacts=contacts),ensure_ascii=False))
