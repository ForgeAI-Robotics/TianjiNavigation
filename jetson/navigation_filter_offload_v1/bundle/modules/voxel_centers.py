"""Exact lexicographic voxel deduplication using bounded integer keys."""
import numpy as np

def downsample_centers(points,resolution):
 keys=np.floor(points/resolution).astype(np.int64)
 if not len(keys):return (keys+.5)*resolution
 low=keys.min(0);high=keys.max(0);spans=[int(b)-int(a)+1 for a,b in zip(low,high)]
 total=spans[0]*spans[1]*spans[2]
 if total>np.iinfo(np.int64).max:
  unique=np.unique(keys,axis=0)
 else:
  yz=spans[1]*spans[2];packed=(keys[:,0]-low[0])*yz+(keys[:,1]-low[1])*spans[2]+keys[:,2]-low[2];ids=np.unique(packed)
  unique=np.column_stack([ids//yz+low[0],ids//spans[2]%spans[1]+low[1],ids%spans[2]+low[2]])
 return (unique+.5)*resolution
