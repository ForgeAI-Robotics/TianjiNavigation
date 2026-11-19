import sys,unittest,itertools,types
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import paths
from near_body_cleanup import evaluate

class CleanupTests(unittest.TestCase):
 def setUp(self):
  self.sha='c'*64;lo=[-.03,-.03,.1];hi=[.03,.03,.92]
  self.shape=dict(link='assembly_lower_mount',name='pole',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))),mode='measured_or_fixed')
  self.cfg=dict(enabled=True,model_sha256=self.sha,surface_band_m=.20,height_m=[.6,1.5],connection_radius_m=.065)
  self.surface=patch('near_body_cleanup.surface_settings',return_value=dict(enabled=True,model_sha256=self.sha));self.surface.start();self.addCleanup(self.surface.stop)
 def test_sparse_wire_removed_without_motion_and_far_obstacle_retained(self):
  p=np.array([[.08,0,.72],[.09,0,.82],[.4,0,.72],[.08,0,.5]])
  mask,keep,report=evaluate(p,[self.shape],self.sha,self.cfg)
  self.assertEqual(mask.tolist(),[True,True,False,False]);self.assertFalse(keep.any());self.assertFalse(report['free_space_credit'])
 def test_table_plane_continuation_overrides_cleanup(self):
  x,y=np.meshgrid(np.arange(-.3,.5,.025),np.arange(-.35,.36,.025));p=np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)]
  mask,keep,report=evaluate(p,[self.shape],self.sha,self.cfg)
  near=(p[:,0]>.07)&(p[:,0]<.16)&(abs(p[:,1])<.1)
  self.assertTrue(near.any());self.assertFalse(mask[near].any());self.assertTrue(keep[near].all())
 def test_vertical_continuous_surface_retained(self):
  y,z=np.meshgrid(np.arange(-.3,.31,.025),np.arange(.61,1.3,.025));p=np.c_[np.full(y.size,.08),y.ravel(),z.ravel()]
  mask,keep,report=evaluate(p,[self.shape],self.sha,self.cfg)
  self.assertFalse(mask.any());self.assertTrue(keep.any())
 def test_thin_vertical_cable_is_not_a_broad_continuous_object(self):
  p=np.c_[np.full(30,.08),np.zeros(30),np.linspace(.62,1.05,30)]
  mask,_,_=evaluate(p,[self.shape],self.sha,self.cfg);self.assertTrue(mask.all())
 def test_near_robot_returns_cannot_provide_their_own_continuity_support(self):
  y,z=np.meshgrid(np.linspace(-.09,.09,8),np.linspace(.60,.91,12));p=np.c_[np.full(y.size,.08),y.ravel(),z.ravel()]
  mask,keep,_=evaluate(p,[self.shape],self.sha,self.cfg)
  self.assertFalse(keep.any());self.assertTrue(mask[p[:,2]>.6].all())
 def test_cable_branch_connected_to_table_does_not_inherit_whole_plane_protection(self):
  x,y=np.meshgrid(np.arange(-.3,.5,.025),np.arange(-.35,.36,.025));table=np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)]
  wire=np.c_[np.full(14,.08),np.zeros(14),np.linspace(.74,.99,14)];p=np.vstack([table,wire])
  mask,_,_=evaluate(p,[self.shape],self.sha,self.cfg)
  self.assertFalse(mask[:len(table)].any());self.assertTrue(mask[len(table):][wire[:,2]>.77].all())
 def test_large_table_component_does_not_blanket_protect_hanging_wire(self):
  x,y=np.meshgrid(np.arange(-1.,1.001,.02),np.arange(-.6,.601,.02));table=np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)]
  wire=np.c_[np.full(16,.08),np.zeros(16),np.linspace(.74,1.04,16)];points=np.vstack([table,wire])
  removed,_,_=evaluate(points,[self.shape],self.sha,self.cfg)
  self.assertFalse(removed[:len(table)].any())
  self.assertTrue(removed[len(table):][wire[:,2]>.78].all())
 def test_curved_solid_object_near_body_is_not_treated_as_wire(self):
  a,b=np.meshgrid(np.linspace(0,2*np.pi,48,endpoint=False),np.linspace(.05,np.pi-.05,25))
  points=np.c_[.22+.13*np.sin(b).ravel()*np.cos(a).ravel(),.13*np.sin(b).ravel()*np.sin(a).ravel(),.72+.13*np.cos(b).ravel()]
  removed,kept,_=evaluate(points,[self.shape],self.sha,self.cfg)
  self.assertFalse(removed.any());self.assertTrue(kept.any())
 def test_model_or_surface_config_mismatch_keeps_all(self):
  p=np.array([[.08,0,.72]])
  self.assertFalse(evaluate(p,[self.shape],'d'*64,self.cfg)[0].any())
  with patch('near_body_cleanup.surface_settings',return_value=dict(enabled=False)):
   self.assertFalse(evaluate(p,[self.shape],self.sha,self.cfg)[0].any())
 def test_small_chassis_roof_extension_is_limited_to_interior_top_projection(self):
  chassis=dict(self.shape,link='YD_link',bounds=[[-.4,-.3,0],[.4,.3,.39]])
  p=np.array([[0,.20,.4508],[0,.31,.4508],[0,.20,.47]])
  cfg=dict(self.cfg,chassis_roof_self_band_m=.065)
  removed,_,_=evaluate(p,[self.shape,chassis],self.sha,cfg)
  self.assertTrue(removed[0]);self.assertFalse(removed[1]);self.assertFalse(removed[2])
 def test_measured_roof_return_boundary_and_outside_obstacles(self):
  chassis=dict(self.shape,link='YD_link',bounds=[[-.33,-.32,0],[.35,.32,.39]])
  p=np.array([[-.158334875,-.209945903,.455744657],[-.158334875,-.34,.455744657],[0,.20,.471]])
  removed,_,_=evaluate(p,[self.shape,chassis],self.sha,dict(self.cfg,chassis_roof_self_band_m=.08))
  self.assertEqual(removed.tolist(),[True,False,False])
 def test_new_roof_margin_preserves_low_continuous_table(self):
  chassis=dict(self.shape,link='YD_link',bounds=[[-.33,-.32,0],[.35,.32,.39]])
  x,y=np.meshgrid(np.arange(-.8,.81,.04),np.arange(-.6,.61,.04));p=np.c_[x.ravel(),y.ravel(),np.full(x.size,.465)]
  removed,keep,_=evaluate(p,[self.shape,chassis],self.sha,dict(self.cfg,chassis_roof_self_band_m=.08))
  self.assertFalse(removed.any());self.assertTrue(keep.any())
 def test_continuous_table_over_chassis_roof_stays(self):
  chassis=dict(self.shape,link='YD_link',bounds=[[-.4,-.3,0],[.4,.3,.39]])
  x,y=np.meshgrid(np.arange(-.8,.81,.04),np.arange(-.6,.61,.04));p=np.c_[x.ravel(),y.ravel(),np.full(x.size,.4508)]
  removed,keep,_=evaluate(p,[self.shape,chassis],self.sha,dict(self.cfg,chassis_roof_self_band_m=.065))
  self.assertFalse(removed.any());self.assertTrue(keep.any())
 def test_lower_cleanup_is_limited_to_chassis_interior(self):
  T=np.eye(4);T[1,3]=.2;pole=dict(self.shape,matrix=T.tolist())
  chassis=dict(self.shape,link='YD_link',bounds=[[-.4,-.3,0],[.4,.3,.39]])
  p=np.array([[.08,.15,.49],[.08,.28,.49]])
  removed,_,_=evaluate(p,[pole,chassis],self.sha,dict(self.cfg,chassis_interior_cleanup_height_m=[.4,.6]))
  self.assertEqual(removed.tolist(),[True,False])
 def test_continuous_table_in_lower_cleanup_region_stays(self):
  chassis=dict(self.shape,link='YD_link',bounds=[[-.4,-.3,0],[.4,.3,.39]])
  x,y=np.meshgrid(np.arange(-.8,.81,.04),np.arange(-.6,.61,.04));p=np.c_[x.ravel(),y.ravel(),np.full(x.size,.49)]
  removed,keep,_=evaluate(p,[self.shape,chassis],self.sha,dict(self.cfg,chassis_interior_cleanup_height_m=[.4,.6]))
  self.assertFalse(removed.any());self.assertTrue(keep.any())
 def test_legacy_micro_execution_rejected_before_sensor_or_control_creation(self):
  import onboard_navigation as nav
  args=types.SimpleNamespace(cable_precheck=True,max_travel_m=0.)
  with patch.object(nav,'ControlTransport') as control,patch.object(nav,'SensorStream') as sensors:
   with self.assertRaisesRegex(ValueError,'MICRO_PROBE_DISABLED'):nav.run(args)
   control.assert_not_called();sensors.assert_not_called()

if __name__=='__main__':unittest.main()
