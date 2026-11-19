import sys,unittest,itertools,copy
from pathlib import Path
from unittest.mock import patch
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'modules'))
from cable_zones import zone_ignore,protected_surfaces
from self_envelope import mask
from pillar_cables import ignore_mask

class CableZoneTests(unittest.TestCase):
 def setUp(self):
  self.sha='c'*64;lo=[-.03,-.03,.1];hi=[.03,.03,.92]
  self.shape=dict(link='assembly_lower_mount',name='pole',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))),mode='measured_or_fixed')
  self.cfg=dict(enabled=True,model_sha256=self.sha,radius_m=.025,max_anchor_translation_m=.02,max_anchor_rotation_deg=3,max_ignored_points_per_scan=250,zones=[dict(link='assembly_lower_mount',collision_name='pole',reference_matrix=np.eye(4).tolist(),points_link=[[.08,0,.72]])])
 def call(self,p):return zone_ignore(np.asarray(p),[self.shape],self.sha,self.cfg)
 def test_confirmed_isolated_wire_deleted_outside_zone_kept(self):
  p=[[.08,0,.72],[.09,0,.72],[.13,0,.72],[.08,0,.76]]
  m,r=self.call(p);self.assertEqual(m.tolist(),[True,True,False,False]);self.assertFalse(r['free_space_credit'])
 def table(self,z=.72):
  x,y=np.meshgrid(np.arange(-.16,.321,.02),np.arange(-.24,.241,.02));return np.c_[x.ravel(),y.ravel(),np.full(x.size,z)]
 def test_table_plane_through_same_zone_retained(self):
  p=self.table();m,r=self.call(p);self.assertFalse(m.any());self.assertGreater(r['surface_protection']['protected_points'],0)
 def test_dense_repeated_table_context_is_fitted_not_blindly_budget_kept(self):
  p=np.repeat(self.table(),20,axis=0);m,r=self.call(p)
  self.assertFalse(m.any());self.assertEqual(r['surface_protection']['budget_keep_points'],0);self.assertGreater(r['surface_protection']['planes'],0)
 def test_table_priority_over_existing_self_envelope(self):
  p=self.table();ecfg=dict(enabled=True,model_sha256=self.sha,links=['assembly_lower_mount'],extra_surface_margin_m=.06)
  with patch('cable_zones.settings',return_value=self.cfg):m,extra,report=mask(p,[self.shape],ecfg,self.sha)
  inside=(np.linalg.norm(p-np.array([.08,0,.72]),axis=1)<.025)
  self.assertTrue(inside.any());self.assertFalse(m[inside].any())
 def test_priority_over_legacy_pillar_filter(self):
  p=self.table();legacy=dict(enabled=False)
  with patch('cable_zones.settings',return_value=self.cfg):m,r=ignore_mask(p,[self.shape],legacy,self.sha)
  self.assertFalse(m.any())
 def test_sparse_table_edge_through_zone_retained(self):
  p=np.c_[np.full(31,.08),np.linspace(-.3,.3,31),np.full(31,.72)]
  m,r=self.call(p);self.assertFalse(m.any());self.assertGreater(r['surface_protection']['edges'],0)
 def test_vertical_desk_face_retained(self):
  y,z=np.meshgrid(np.arange(-.24,.241,.02),np.arange(.52,.921,.02));p=np.c_[np.full(y.size,.08),y.ravel(),z.ravel()]
  self.assertFalse(self.call(p)[0].any())
 def test_separate_cable_removed_table_six_cm_above_retained(self):
  p=np.vstack([[[.08,0,.72]],self.table(.78)]);m,r=self.call(p);self.assertTrue(m[0]);self.assertFalse(m[1:].any())
 def test_posture_changed_or_unknown_model_keeps_points(self):
  s=copy.deepcopy(self.shape);s['matrix'][0][3]=.03
  self.assertFalse(zone_ignore(np.array([[.08,0,.72]]),[s],self.sha,self.cfg)[0].any())
  self.assertFalse(zone_ignore(np.array([[.08,0,.72]]),[self.shape],'wrong',self.cfg)[0].any())
 def test_voxelized_table_priority_kept(self):
  p=self.table(.725)
  m,r=protected_surfaces(p,[self.shape],self.sha,self.cfg,voxel_resolution_m=.05)
  self.assertTrue(m[np.linalg.norm(p-np.array([.08,0,.725]),axis=1)<.025].all())
 def test_voxel_centres_use_cell_extent_and_keep_table(self):
  from cable_zones import zone_ignore
  p=np.array([[.14,0,.72]])
  self.assertFalse(zone_ignore(p,[self.shape],self.sha,self.cfg)[0].any())
  self.assertTrue(zone_ignore(p,[self.shape],self.sha,self.cfg,voxel_resolution_m=.05)[0].all())
  self.assertFalse(zone_ignore(self.table(),[self.shape],self.sha,self.cfg,voxel_resolution_m=.05)[0].any())
 def test_preserved_table_still_blocks_live_collision_guard(self):
  from execution_safety import sweep_guard
  p=self.table();header=dict(pose=[0,0,0],source_age_s=0.)
  frames={sensor:(header,p,100.) for sensor in ['front','back']}
  envelope=dict(enabled=True,model_sha256=self.sha,links=['assembly_lower_mount'],extra_surface_margin_m=.06)
  with patch('cable_zones.settings',return_value=self.cfg),patch('execution_safety.envelope_settings',return_value=envelope),patch('execution_safety.cable_settings',return_value=dict(enabled=False)):
   ok,reason,age=sweep_guard(frames,{'front':np.eye(4),'back':np.eye(4)},[0,0,0],[self.shape],self.sha,None,[0,0,0],100.)
  self.assertFalse(ok);self.assertEqual(reason,'LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY')

if __name__=='__main__':unittest.main()
