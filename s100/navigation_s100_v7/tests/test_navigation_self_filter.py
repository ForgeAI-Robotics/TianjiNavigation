import unittest,sys,itertools
from pathlib import Path
from unittest.mock import patch
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'modules'))
from navigation_self_filter import evaluate

class SharedSelfFilterTests(unittest.TestCase):
 def setUp(self):
  self.sha='c'*64;lo=[-.03,-.03,.1];hi=[.03,.03,.92]
  self.shape=dict(link='assembly_lower_mount',name='pole',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))),mode='measured_or_fixed')
  self.cfg=dict(enabled=True,model_sha256=self.sha,radius_m=.04,max_anchor_translation_m=.02,max_anchor_rotation_deg=3,max_ignored_points_per_scan=250,surface_support_gap_m=.25,zones=[dict(link='assembly_lower_mount',collision_name='pole',reference_matrix=np.eye(4).tolist(),points_link=[[.08,0,.72]])])
 def call(self,p):
  with patch('cable_zones.settings',return_value=self.cfg):return evaluate(p,[self.shape],self.sha,envelope_cfg=dict(enabled=False),cable_cfg=dict(enabled=False))
 def test_raw_geometry_unchanged_and_masks_disjoint(self):
  p=np.array([[0,0,.72],[.08,0,.72],[.5,.5,.72]]);original=p.copy();result=self.call(p)
  np.testing.assert_array_equal(result['filtered'],[True,True,False]);np.testing.assert_array_equal(p,original)
  self.assertFalse(result['free_space_credit']);self.assertEqual(result['source'],'S100_V7_NAVIGATION_SELF_FILTER');self.assertFalse((result['protected']&result['filtered']).any())
 def test_table_passes_through_zone_and_is_kept(self):
  x,y=np.meshgrid(np.arange(-.16,.321,.02),np.arange(-.24,.241,.02));p=np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)];result=self.call(p);region=np.linalg.norm(p-[.08,0,.72],axis=1)<=.04
  self.assertTrue(region.any());self.assertFalse(result['filtered'][region].any());self.assertTrue(result['protected'][region].all())

if __name__=='__main__':unittest.main()
