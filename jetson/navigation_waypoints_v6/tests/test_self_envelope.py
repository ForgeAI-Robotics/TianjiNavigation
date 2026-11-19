import sys,json,copy,itertools,unittest
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'modules'))
from self_envelope import mask,settings

class SelfEnvelopeTests(unittest.TestCase):
 def setUp(self):
  self.cfg=settings();self.sha=self.cfg['model_sha256']
  lo=[-.34,-.32,0.];hi=[.34,.32,.39]
  self.shapes=[dict(link='YD_link',mode='measured_or_fixed',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))))]
 def test_voxel_extension_covers_previous_conflict_without_changing_collision_box(self):
  original=copy.deepcopy(self.shapes);p=np.array([[.34+.087,0,.2],[.34+.11,0,.2]])
  own,extra,report=mask(p,self.shapes,self.cfg,self.sha,.05)
  self.assertEqual(extra.tolist(),[True,False]);self.assertEqual(self.shapes,original);self.assertFalse(report['collision_geometry_modified'])
 def test_raw_extension_and_disabled_model_guard(self):
  p=np.array([[.39,0,.2],[.405,0,.2]])
  self.assertEqual(mask(p,self.shapes,self.cfg,self.sha)[0].tolist(),[True,False])
  self.assertFalse(mask(p,self.shapes,dict(self.cfg,enabled=False),self.sha)[0].any())
  self.assertFalse(mask(p,self.shapes,self.cfg,'changed')[0].any())
 def test_unlisted_or_unknown_shape_does_not_expand(self):
  self.shapes[0]['link']='arm';p=np.array([[.39,0,.2]])
  self.assertFalse(mask(p,self.shapes,self.cfg,self.sha)[0].any())
  self.shapes[0]['link']='YD_link';self.shapes[0]['mode']='unknown'
  self.assertFalse(mask(p,self.shapes,self.cfg,self.sha)[0].any())
if __name__=='__main__':unittest.main()
