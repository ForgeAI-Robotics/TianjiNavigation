import unittest
import numpy as np
from surface_footprint import plane_supported,edge_supported
class SurfaceFootprintTests(unittest.TestCase):
 def plane(self):
  x,y=np.meshgrid(np.linspace(.14,.40,16),np.linspace(-.2,.2,21))
  return np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)]
 def test_near_body_coplanar_point_cannot_inherit_distant_table_plane(self):
  p=np.array([[.06,0,.72],[.13,0,.72],[.20,0,.72],[.20,0,.80]])
  self.assertEqual(plane_supported(self.plane(),p,np.array([0,0,1])).tolist(),[False,True,True,False])
 def test_interpolation_across_body_occlusion_keeps_real_table(self):
  p=self.plane();support=p[abs(p[:,1])>.13]
  self.assertTrue(plane_supported(support,np.array([[.2,0,.72]]),np.array([0,0,1]))[0])
 def test_sparse_edge_retained_but_side_branch_and_extension_not_protected(self):
  y=np.r_[np.linspace(-.3,-.12,12),np.linspace(.12,.3,12)];line=np.c_[np.full(len(y),.08),y,np.full(len(y),.72)]
  p=np.array([[.08,0,.72],[.08,.2,.72],[.14,0,.72],[.08,0,.80],[.08,.4,.72]])
  self.assertEqual(edge_supported(line,p).tolist(),[True,True,False,False,False])
 def test_degenerate_plane_never_authorizes_deletion(self):
  self.assertTrue(plane_supported(np.array([[0,0,0],[1,0,0],[2,0,0]]),np.array([[0,1,1]]),np.array([0,0,1]))[0])
