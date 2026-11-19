"""Precompute immutable URDF constants; each measured FK uses its own pose cache.

No measured angles, missing-joint bounds, timestamps, or collision margins are
cached. The reference collision_geometry.volumes remains the independent reference implementation.
"""
import math
import numpy as np
from collision_geometry import origin,rotation,corners

class CompiledVolumes:
 def __init__(self,model):
  self.root=model['root'];self.joints={};self.shapes=[]
  def frozen(value):
   value=np.array(value,copy=True);value.setflags(write=False);return value
  for j in model['joints']:
   self.joints[j['child']]=(j.copy(),frozen(origin(j['xyz'],j['rpy'])))
  for shape in model['shapes']:
   bounds=frozen(shape['bounds']);vertices=frozen(corners(bounds));G=frozen(origin(shape['xyz'],shape['rpy']))
   self.shapes.append((shape['link'],shape['name'],G,bounds,vertices))
 def __call__(self,positions):
  cache={self.root:(np.eye(4),None,0.)}
  def pose(link):
   if link in cache:return cache[link]
   j,O=self.joints[link];parent,unknown,reach=pose(j['parent']);T=parent@O
   if unknown is not None:reach+=float(np.linalg.norm(j['xyz']))
   q=positions.get(j['name']);m=j.get('mimic')
   if m:q=None if m['joint'] not in positions else positions[m['joint']]*m['multiplier']+m['offset']
   if j['type']!='fixed':
    if q is None or not math.isfinite(q):
     if unknown is None:unknown=T[:3,3].copy();reach=0.
     if j['type']=='prismatic':reach+=max(abs(j['lower']),abs(j['upper']))
    elif j['type']=='prismatic':
     T[:3,3]+=T[:3,:3]@np.array(j['axis'])*q
     if unknown is not None:reach+=abs(q)*np.linalg.norm(j['axis'])
    else:T=T@rotation(j['axis'],q)
   cache[link]=(T,unknown,reach);return cache[link]
  result=[]
  for link,name,G,b,C in self.shapes:
   T,u,reach=pose(link)
   if u is not None:
    radius=reach+max(np.linalg.norm((G[:3,:3]@C.T).T+G[:3,3],axis=1));W=np.eye(4);W[:3,3]=u;lo=np.repeat(-radius,3);hi=-lo;mode='missing_joint_full_rotation_bound';vertices=corners([lo,hi])
   else:W=T@G;lo,hi=b;mode='measured_or_fixed';vertices=C
   pts=(W[:3,:3]@vertices.T).T+W[:3,3]
   result.append(dict(link=link,name=name,matrix=W.tolist(),bounds=[lo.tolist(),hi.tolist()],world_corners=pts.tolist(),mode=mode))
  return result
