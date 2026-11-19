import sys,unittest,json,itertools
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'modules'))
from pillar_cables import ignore_mask

class PillarIgnoreTests(unittest.TestCase):
 def setUp(self):
  self.cfg=json.loads((R/'config/temporary_pillar_cables.json').read_text());self.sha=self.cfg['model_sha256']
  lo=[-.03,-.03,.10];hi=[.03,.03,.92]
  self.shape=dict(link='assembly_lower_mount',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))),mode='measured_or_fixed')
 def call(self,p,cfg=None):return ignore_mask(np.array(p,float),[self.shape],cfg or self.cfg,self.sha)
 def test_vertical_thin_cable_in_confirmed_zone_is_ignored(self):
  p=np.c_[np.full(50,.075),np.full(50,.01),np.linspace(.45,.88,50)];m,r=self.call(p);self.assertTrue(m.all());self.assertFalse(r['free_space_credit'])
 def test_vertical_multi_strand_bundle_is_ignored(self):
  x,y,z=np.meshgrid([.065,.09,.115],[-.025,0,.025],np.arange(.45,.881,.015));p=np.c_[x.ravel(),y.ravel(),z.ravel()];m,r=self.call(p);self.assertTrue(m.all());self.assertTrue(r['components'][0]['vertical_bundle'])
 def test_floor_and_outside_zone_are_unchanged(self):
  m,r=self.call([[.4,0,.6],[.075,0,.08],[.075,0,1.05],[-.4,0,.8]]);self.assertFalse(m.any())
 def test_table_surface_entering_zone_is_kept(self):
  x,y=np.meshgrid(np.arange(-.30,.301,.025),np.arange(-.30,.301,.025));p=np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)];m,r=self.call(p);self.assertFalse(m.any());self.assertGreater(r['protected_broad_points'],0)
 def test_wall_surface_is_kept(self):
  y,z=np.meshgrid(np.arange(-.30,.301,.025),np.arange(.30,1.051,.025));p=np.c_[np.full(y.size,.08),y.ravel(),z.ravel()];m,r=self.call(p);self.assertFalse(m.any())
 def test_long_thin_table_edge_with_outside_support_is_kept(self):
  p=np.c_[np.full(65,.08),np.linspace(-.32,.32,65),np.full(65,.72)];m,r=self.call(p);self.assertFalse(m.any())
 def test_hand_sized_volume_is_kept(self):
  rng=np.random.default_rng(1);p=rng.uniform([.06,-.08,.55],[.16,.08,.69],(350,3));m,r=self.call(p);self.assertFalse(m.any())
 def test_disabled_or_changed_model_keeps_everything(self):
  p=[[.075,0,.6]];cfg=dict(self.cfg,enabled=False);self.assertFalse(self.call(p,cfg)[0].any());cfg=dict(self.cfg,model_sha256='0'*64);self.assertFalse(self.call(p,cfg)[0].any())

if __name__=='__main__':unittest.main()
