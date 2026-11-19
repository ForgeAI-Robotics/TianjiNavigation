"""Conservative URDF collision volumes. Coordinates in chassis_base, metres."""
import math,itertools
import numpy as np

def origin(xyz=(0,0,0),rpy=(0,0,0)):
 r,p,y=rpy;cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
 T=np.eye(4);T[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]];T[:3,3]=xyz;return T

def rotation(axis,q):
 a=np.array(axis,dtype=float);a/=np.linalg.norm(a);x,y,z=a;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);T=np.eye(4);T[:3,:3]=np.eye(3)+math.sin(q)*K+(1-math.cos(q))*(K@K);return T

def corners(bounds):return np.array(list(itertools.product(*zip(*bounds))))

def volumes(model,positions):
 """Known joints produce OBBs; missing joints conservatively bound ALL rotations.
 No stale or zero pose is substituted. Mimic values follow their source joints.
 """
 joints={j['child']:j for j in model['joints']};cache={model['root']:(np.eye(4),None,0.)}
 def pose(link):
  if link in cache:return cache[link]
  j=joints[link];parent,unknown,reach=pose(j['parent']);O=origin(j['xyz'],j['rpy']);T=parent@O
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
 for shape in model['shapes']:
  T,u,reach=pose(shape['link']);G=origin(shape['xyz'],shape['rpy']);b=np.array(shape['bounds']);C=corners(b)
  if u is not None:
   radius=reach+max(np.linalg.norm((G[:3,:3]@C.T).T+G[:3,3],axis=1));W=np.eye(4);W[:3,3]=u;lo=np.repeat(-radius,3);hi=-lo;mode='missing_joint_full_rotation_bound'
  else:W=T@G;lo,hi=b;mode='measured_or_fixed'
  pts=(W[:3,:3]@corners([lo,hi]).T).T+W[:3,3]
  result.append(dict(link=shape['link'],name=shape['name'],matrix=W.tolist(),bounds=[lo.tolist(),hi.tolist()],world_corners=pts.tolist(),mode=mode))
 return result

def hull(points):
 pts=sorted(set((float(x),float(y)) for x,y in points))
 def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
 chains=[]
 for seq in (pts,pts[::-1]):
  half=[]
  for p in seq:
   while len(half)>=2 and cross(half[-2],half[-1],p)<=0:half.pop()
   half.append(p)
  chains.append(half[:-1])
 return chains[0]+chains[1]

def near_points(points,shapes,margin=.05):
 """OBB broad-phase candidates including possible self returns; never certifies free."""
 p=np.asarray(points);best=np.full(len(p),np.inf);which=np.full(len(p),-1)
 for i,s in enumerate(shapes):
  T=np.array(s['matrix']);q=(p-T[:3,3])@T[:3,:3];lo,hi=np.array(s['bounds']);dist=np.linalg.norm(np.maximum(np.maximum(lo-q,q-hi),0),axis=1);update=dist<best;best[update]=dist[update];which[update]=i
 mask=best<=margin
 return mask,best,which
