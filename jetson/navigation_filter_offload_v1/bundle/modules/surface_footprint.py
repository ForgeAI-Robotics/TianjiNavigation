"""Bound surface retention by observed support, without changing collision size."""
import numpy as np
from scipy.spatial import cKDTree,ConvexHull,QhullError

def plane_supported(surface,targets,normal,tolerance=.025,extension=.025,support_gap=.08):
 surface=np.asarray(surface,float);targets=np.asarray(targets,float)
 if not len(targets):return np.zeros(0,bool)
 # Degenerate/failed fits remain protected rather than becoming deletion.
 if len(surface)<3:return np.ones(len(targets),bool)
 origin=surface.mean(0);_,axes=np.linalg.eigh(np.cov(surface.T));basis=axes[:,1:]
 uv=(surface-origin)@basis;query=(targets-origin)@basis
 try:hull=ConvexHull(uv)
 except QhullError:return np.ones(len(targets),bool)
 signed=query@hull.equations[:,:2].T+hull.equations[:,2]
 in_footprint=(signed<=extension+1e-9).all(1)
 near=cKDTree(surface).query(targets)[0]<=support_gap+1e-9
 on_plane=np.abs((targets-origin)@normal)<=tolerance+1e-9
 inside=(signed<=1e-9).all(1)
 return on_plane&in_footprint&(inside|near)

def edge_supported(support,targets,tolerance=.025,extension=.025,support_gap=.08):
 support=np.asarray(support,float);targets=np.asarray(targets,float)
 if not len(targets):return np.zeros(0,bool)
 origin=support.mean(0);values,axes=np.linalg.eigh(np.cov(support[:,:2].T));direction=axes[:,-1]
 # A broad patch is not independent evidence for a thin edge. Planes handle it.
 if np.sqrt(max(values[0],0))>.025:return np.zeros(len(targets),bool)
 along=(support[:,:2]-origin[:2])@direction;delta=targets[:,:2]-origin[:2];t=delta@direction
 perpendicular=np.abs(delta[:,0]*direction[1]-delta[:,1]*direction[0])
 return ((t>=along.min()-extension)&(t<=along.max()+extension)&(perpendicular<=tolerance)
  &(targets[:,2]>=support[:,2].min()-tolerance)&(targets[:,2]<=support[:,2].max()+tolerance)
  &(((t>=along.min())&(t<=along.max()))|(cKDTree(support).query(targets)[0]<=support_gap)))
