"""Isolated FP64 CUDA OBB distances; one point upload, reduced outputs only."""
import ctypes,numpy as np
from pathlib import Path
class CudaDistances:
 def __init__(self):
  self.lib=ctypes.CDLL(str(Path(__file__).resolve().parents[1]/'cuda/libobb.so'));self.lib.obb_create.restype=ctypes.c_void_p;self.ctx=self.lib.obb_create()
  self.lib.obb_destroy.argtypes=[ctypes.c_void_p];self.lib.obb_error.argtypes=[ctypes.c_void_p];self.lib.obb_error.restype=ctypes.c_char_p
  a=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS');ids=np.ctypeslib.ndpointer(dtype=np.int32,flags='C_CONTIGUOUS')
  self.lib.obb_bind.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_int,a,a];self.lib.obb_bind.restype=ctypes.c_int
  self.lib.obb_query.argtypes=[ctypes.c_void_p,ctypes.c_int,ids,ctypes.c_double,a];self.lib.obb_query.restype=ctypes.c_int
 def check(self,code):
  if code:raise RuntimeError(self.lib.obb_error(self.ctx).decode() if code==-1 else 'INVALID_CUDA_INPUT')
 def close(self):
  if self.ctx:self.lib.obb_destroy(self.ctx);self.ctx=None
 def bind(self,points,shapes):
  self.points=np.ascontiguousarray(points,dtype=np.float64);self.ids={id(s):i for i,s in enumerate(shapes)};rows=[]
  for shape in shapes:
   T=np.asarray(shape['matrix']);bounds=np.asarray(shape['bounds']);center=T[:3,:3]@bounds.mean(0)+T[:3,3];radius=np.linalg.norm((bounds[1]-bounds[0])*.5)
   rows.append(np.r_[T.ravel(),bounds.ravel(),center,radius])
  packed=np.ascontiguousarray(rows,dtype=np.float64).reshape(-1,26)
  if not np.isfinite(self.points).all() or not np.isfinite(packed).all():raise ValueError('NONFINITE_CUDA_INPUT')
  self.check(self.lib.obb_bind(self.ctx,len(points),len(shapes),self.points,packed))
 def distances(self,points,shapes,limit=None):
  if len(points)!=len(self.points) or not np.array_equal(points,self.points):raise ValueError('FRAME_BINDING_MISMATCH')
  cols=np.ascontiguousarray([self.ids[id(s)] for s in shapes],dtype=np.int32)
  if not len(cols):return np.full(len(points),np.inf)
  out=np.empty(len(points),dtype=np.float64);self.check(self.lib.obb_query(self.ctx,len(cols),cols,-1. if limit is None else float(limit),out));return out
