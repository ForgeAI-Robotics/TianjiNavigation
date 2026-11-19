"""Per-frame measured sampling statistics, without filling or accumulated scans."""
import numpy as np
from scipy.spatial import cKDTree
BANDS=((0.,1.),(1.,2.),(2.,4.),(4.,8.),(8.,None))
def statistics(points,origin):
    p=np.asarray(points);distance=np.linalg.norm(p-np.asarray(origin),axis=1)
    if len(p)>1:spacing=cKDTree(p).query(p,k=2,workers=1)[0][:,1]
    else:spacing=np.full(len(p),np.nan)
    result=[]
    for low,high in BANDS:
        ids=(distance>=low)&(distance<high if high is not None else True)
        selected=spacing[ids];selected=selected[np.isfinite(selected)]
        result.append(dict(low_m=low,high_m=high,points=int(ids.sum()),median_neighbor_cm=float(np.median(selected)*100) if len(selected) else None))
    return result

def preview_indices(points,labels,limit=8000):
    p=np.asarray(points);labels=np.asarray(labels);valid=(labels>0)&(labels<6)&(np.linalg.norm(p[:,:2],axis=1)<4.)&(p[:,2]>-.25)&(p[:,2]<3.)
    selected=[];room=limit
    for kinds in [(2,),(5,),(3,4),(1,)]:
        ids=np.flatnonzero(valid&np.isin(labels,kinds))
        if len(ids)>room:ids=ids[np.linspace(0,len(ids)-1,room,dtype=int)] if room else np.empty(0,int)
        selected.append(ids);room-=len(ids)
    return np.concatenate(selected)
