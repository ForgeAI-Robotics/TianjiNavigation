"""Small bounded tabletop/near-edge estimator for forward approach; no semantic free space."""
import numpy as np

def estimate_table(points):
 p=np.asarray(points,float);p=p[np.isfinite(p).all(1)&(p[:,0]>.02)&(p[:,0]<1.4)&(abs(p[:,1])<1.1)&(p[:,2]>.45)&(p[:,2]<.95)]
 if len(p)<400:return dict(valid=False,reason='INSUFFICIENT_TABLE_POINTS')
 rng=np.random.default_rng(826);h,e=np.histogram(p[:,2],np.arange(.45,.961,.01));i=int(np.argmax(h));z=(e[i]+e[i+1])/2;p0=p[abs(p[:,2]-z)<.025]
 if len(p0)>4000:p0=p0[rng.choice(len(p0),4000,replace=False)]
 X=np.c_[p0[:,:2],np.ones(len(p0))];best=None;best_n=0
 for _ in range(70):
  s=rng.choice(len(p0),3,replace=False)
  try:c=np.linalg.solve(X[s],p0[s,2])
  except np.linalg.LinAlgError:continue
  if np.linalg.norm(c[:2])>.1:continue
  n=np.sum(abs(p0[:,2]-X@c)<.007)
  if n>best_n:best=c;best_n=n
 if best is None or best_n<150:return dict(valid=False,reason='NO_SUPPORTED_HORIZONTAL_PLANE')
 for _ in range(3):
  keep=abs(p0[:,2]-X@best)<.008;best=np.linalg.lstsq(X[keep],p0[keep,2],rcond=None)[0]
 plane_support=int(keep.sum())
 top=p[abs(p[:,2]-np.c_[p[:,:2],np.ones(len(p))]@best)<.007]
 rows=[]
 for y in np.arange(-.95,.75,.025):
  q=top[(top[:,1]>=y)&(top[:,1]<y+.025)]
  q=q[q[:,0]<.4]
  if len(q)>=max(3,min(15,len(top)//2000)):rows.append([y+.0125,float(np.quantile(q[:,0],.02))])
 rows=np.array(rows)
 if len(rows)<12:return dict(valid=False,reason='EDGE_LATERAL_SUPPORT_MISSING')
 line=None;best_n=0
 for _ in range(100):
  a,b=rows[rng.choice(len(rows),2,replace=False)]
  if abs(a[0]-b[0])<.4:continue
  k=(a[1]-b[1])/(a[0]-b[0]);c=a[1]-k*a[0]
  if abs(k)>.1:continue
  n=np.sum(abs(rows[:,1]-k*rows[:,0]-c)<.01)
  if n>best_n:line=np.array([k,c]);best_n=n
 if line is None:return dict(valid=False,reason='NO_STRAIGHT_SUPPORTED_EDGE')
 for _ in range(3):
  keep=abs(rows[:,1]-line[0]*rows[:,0]-line[1])<.01
  line=np.linalg.lstsq(np.c_[rows[keep,0],np.ones(keep.sum())],rows[keep,1],rcond=None)[0]
 low,high=rows[keep,0].min(),rows[keep,0].max()
 if keep.sum()<12 or high-low<.6 or not low<-.1<.1<high:return dict(valid=False,reason='EDGE_DOES_NOT_BRACKET_ROBOT')
 return dict(valid=True,plane=best.tolist(),edge=line.tolist(),y_range=[float(low),float(high)],plane_points=plane_support,edge_strips=int(keep.sum()),edge_residual95_m=float(np.percentile(abs(rows[keep,1]-line[0]*rows[keep,0]-line[1]),95)),quality='ROUGH_TABLE_CONSTRAINT',underside_confirmed=False,scope='visible plane/edge only, not table legs or free-space proof')
