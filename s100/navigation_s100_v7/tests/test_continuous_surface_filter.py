import sys,unittest,itertools
from pathlib import Path
import numpy as np
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import paths
from continuous_surface_filter import evaluate,SurfaceCache
from saved_map import base_transform

class SurfacePriorityTests(unittest.TestCase):
 def setUp(self):
  self.sha='c'*64;self.cfg=dict(enabled=True,model_sha256=self.sha,surface_band_m=.25,plane_refresh_s=1.)
  lo=[-.03,-.03,.1];hi=[.03,.03,.92]
  self.shape=dict(link='pole',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))),mode='measured_or_fixed')
 def table(self):
  x,y=np.meshgrid(np.arange(-.4,.501,.025),np.arange(-.35,.351,.025));return np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)]
 def call(self,p,cache=None,stamp=10.,pose=(0,0,0)):
  return evaluate(p,[self.shape],self.sha,self.cfg,cache,dict(observation_time=stamp,pose=pose,boot_id='boot',epoch=1))
 def test_table_kept_and_connected_hanging_wire_removed(self):
  table=self.table();wire=np.c_[np.full(16,.08),np.zeros(16),np.linspace(.74,1.04,16)]
  r=self.call(np.vstack([table,wire]))
  self.assertFalse(r['filtered'][:len(table)].any())
  self.assertTrue(r['filtered'][len(table):][wire[:,2]>.79].all())
 def test_sparse_near_returns_removed_outside_region_untouched(self):
  p=np.array([[.08,0,.72],[.25,0,.82],[.40,0,.72],[.08,0,.4]])
  np.testing.assert_array_equal(self.call(p)['filtered'],[True,True,False,False])
 def test_lower_mount_sparse_band_preserves_broad_low_surface(self):
  mount=dict(self.shape,link='assembly_lower_mount',bounds=[[-.03,-.03,.40],[.03,.03,.60]])
  cfg=dict(self.cfg,lower_mount_sparse_band_m=.10,lower_mount_height_m=[.50,.60])
  p=np.array([[.07,0,.56],[.07,0,.49],[.15,0,.56]])
  np.testing.assert_array_equal(evaluate(p,[mount],self.sha,cfg)['filtered'],[True,False,False])
  x,y=np.meshgrid(np.arange(.065,.301,.01),np.arange(-.12,.121,.01));surface=np.c_[x.ravel(),y.ravel(),np.full(x.size,.56)]
  self.assertFalse(evaluate(surface,[mount],self.sha,cfg)['filtered'].any())
 def test_wall_and_curved_object_kept(self):
  y,z=np.meshgrid(np.arange(-.35,.351,.025),np.arange(.61,1.31,.025));wall=np.c_[np.full(y.size,.08),y.ravel(),z.ravel()]
  self.assertFalse(self.call(wall)['filtered'].any())
  a,b=np.meshgrid(np.linspace(0,2*np.pi,48,endpoint=False),np.linspace(.05,np.pi-.05,25))
  sphere=np.c_[.22+.13*np.sin(b).ravel()*np.cos(a).ravel(),.13*np.sin(b).ravel()*np.sin(a).ravel(),.72+.13*np.cos(b).ravel()]
  self.assertFalse(self.call(sphere)['filtered'].any())
 def test_cache_requires_new_current_support_and_does_not_supply_points(self):
  cache=SurfaceCache();table=self.table();first=self.call(table,cache)
  self.assertTrue(first['cable_report']['near_body_cleanup']['plane_refit'])
  second=self.call(table,cache,10.25)
  self.assertFalse(second['cable_report']['near_body_cleanup']['plane_refit'])
  lone=np.array([[.08,0,.72]])
  self.assertTrue(self.call(lone,cache,10.5)['filtered'][0])
  self.assertEqual(cache.fitted_at,10.)
 def test_plane_convex_hull_does_not_fill_an_unobserved_hole_around_body(self):
  table=self.table();table=table[(abs(table[:,0])>.25)|(abs(table[:,1])>.25)]
  p=np.vstack([table,[.08,0,.72]])
  result=self.call(p)
  self.assertFalse(result['filtered'][:-1].any())
  self.assertTrue(result['filtered'][-1])
 def test_cached_plane_transforms_with_measured_odometry(self):
  cache=SurfaceCache();world=self.table();self.call(world,cache)
  pose=[.04,-.02,.08];T=base_transform(pose);body=(world-T[:3,3])@T[:3,:3]
  self.assertFalse(self.call(body,cache,10.25,pose)['filtered'].any())
 def test_horizontal_thin_external_table_edge_kept(self):
  p=np.c_[np.arange(.06,.46,.02),np.zeros(20),np.full(20,.72)]
  self.assertFalse(self.call(p)['filtered'].any())
 def test_shared_navigation_adapter_uses_complete_policy_without_legacy_fitting(self):
  from navigation_self_filter import evaluate as shared
  points=np.array([[.08,0,.72],[.4,0,.72]])
  with patch('continuous_surface_filter.settings',return_value=self.cfg),patch('navigation_self_filter.mask') as legacy:
   result=shared(points,[self.shape],self.sha,conservative_fast=True)
  legacy.assert_not_called();self.assertTrue(result['classification_complete'])
  np.testing.assert_array_equal(result['filtered'],[True,False])
 def test_thin_noisy_bundle_connected_to_desk_cannot_inherit_its_area(self):
  x,y=np.meshgrid(np.arange(.15,.46,.025),np.arange(.20,.51,.025));table=np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)]
  rng=np.random.default_rng(529);wire=np.c_[rng.uniform(.06,.10,160),rng.uniform(0,.21,160),rng.uniform(.70,.74,160)]
  result=self.call(np.vstack([table,wire]))
  self.assertFalse(result['filtered'][:len(table)].any())
  self.assertTrue(result['filtered'][len(table):][wire[:,1]<.10].all())
 def test_existing_low_chassis_envelope_filters_sparse_return_but_keeps_wall(self):
  chassis=dict(self.shape,link='YD_link',bounds=[[-.3,-.3,0],[.3,.3,.39]])
  envelope=dict(enabled=True,model_sha256=self.sha,extra_surface_margin_m=.06,links=['YD_link'])
  with patch('continuous_surface_filter.envelope_settings',return_value=envelope):
   result=evaluate(np.array([[-.345,0,.378]]),[self.shape,chassis],self.sha,self.cfg)
   self.assertTrue(result['filtered'][0])
   y,z=np.meshgrid(np.arange(-.25,.251,.025),np.arange(.05,.381,.025));wall=np.c_[np.full(y.size,-.345),y.ravel(),z.ravel()]
   result=evaluate(wall,[self.shape,chassis],self.sha,self.cfg)
   self.assertFalse(result['filtered'].any())
 def test_low_column_wire_does_not_inherit_chassis_wall_exception(self):
  chassis=dict(self.shape,link='YD_link',bounds=[[-.3,-.3,0],[.3,.3,.39]])
  envelope=dict(enabled=True,model_sha256=self.sha,extra_surface_margin_m=.06,links=['YD_link','pole'])
  x,y=np.meshgrid(np.arange(-.4,.501,.025),np.arange(-.35,.351,.025));table=np.c_[x.ravel(),y.ravel(),np.full(x.size,.50)]
  wire=np.c_[np.full(16,.08),np.zeros(16),np.linspace(.52,.82,16)]
  with patch('continuous_surface_filter.envelope_settings',return_value=envelope):result=evaluate(np.vstack([table,wire]),[self.shape,chassis],self.sha,self.cfg)
  self.assertFalse(result['filtered'][:len(table)].any())
  self.assertTrue(result['filtered'][len(table):][wire[:,2]>.57].all())
 def test_chassis_roof_patch_cannot_use_side_wall_exception(self):
  chassis=dict(self.shape,link='YD_link',bounds=[[-.3,-.3,0],[.3,.3,.39]])
  envelope=dict(enabled=True,model_sha256=self.sha,extra_surface_margin_m=.06,links=['YD_link','pole'])
  x,y=np.meshgrid(np.arange(-.12,.121,.015),np.arange(-.12,.121,.015))
  roof=np.c_[x.ravel(),y.ravel(),np.full(x.size,.43)]
  # A real broad external overhang still has current independent support.
  overhang=self.table();overhang[:,2]=.43
  with patch('continuous_surface_filter.envelope_settings',return_value=envelope):
   result=evaluate(roof,[self.shape,chassis],self.sha,self.cfg)
   self.assertTrue(result['filtered'].all())
   result=evaluate(overhang,[self.shape,chassis],self.sha,self.cfg)
   from pillar_cables import distances
   # Points physically inside the pole's own margin remain self returns.
   outside_pole=distances(overhang,[self.shape])>.025
   self.assertFalse(result['filtered'][outside_pole].any())
 def test_thin_vertical_cable_not_continuous_object(self):
  p=np.c_[np.full(30,.08),np.zeros(30),np.linspace(.62,1.05,30)]
  self.assertTrue(self.call(p)['filtered'].all())

class ConfirmedMountCableTests(SurfacePriorityTests):
 def setUp(self):
  super().setUp();self.shape['link']='assembly_lower_mount'
  self.cfg['confirmed_mount_cables']=dict(enabled=True,body_bounds_m=[[-.025,-.08,.40],[.10,.09,.83]])
 def test_dense_self_supported_cable_patch_removed(self):
  y,z=np.meshgrid(np.arange(-.06,.061,.012),np.arange(.70,.811,.012))
  patch=np.c_[np.full(y.size,.075),y.ravel(),z.ravel()]
  self.assertTrue(self.call(patch)['filtered'].all())
 def test_external_wall_through_confirmed_region_preserved(self):
  y,z=np.meshgrid(np.arange(-.30,.301,.012),np.arange(.60,.91,.012))
  wall=np.c_[np.full(y.size,.075),y.ravel(),z.ravel()]
  self.assertFalse(self.call(wall)['filtered'].any())
 def test_outside_region_classification_unchanged(self):
  p=self.table();p=p[(p[:,0]>.11)|(p[:,1]>.10)]
  old=dict(self.cfg);old.pop('confirmed_mount_cables')
  np.testing.assert_array_equal(self.call(p)['filtered'],evaluate(p,[self.shape],self.sha,old)['filtered'])

if __name__=='__main__':unittest.main()
