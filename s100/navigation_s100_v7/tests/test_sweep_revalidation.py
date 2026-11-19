import sys,unittest,itertools,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import paths
from execution_safety import translation_sweep_shapes,curved_sweep_shapes,sweep_reference_drift,position_check,sweep_guard
from environment import self_candidates

class SweepTests(unittest.TestCase):
 def shape(self,angle=0.,center=(0.,0.,0.)):
  c,s=math.cos(angle),math.sin(angle);T=np.eye(4);T[:2,:2]=[[c,-s],[s,c]];T[:3,3]=center
  return dict(link='arm',matrix=T.tolist(),bounds=[[-.1,-.2,-.3],[.1,.2,.3]],mode='measured_or_fixed')
 def test_continuous_translation_contains_every_original_collision(self):
  rng=np.random.default_rng(234)
  for _ in range(20):
   shapes=[self.shape(rng.uniform(-math.pi,math.pi),rng.uniform(-.4,.4,3)) for i in range(3)]
   v=[*rng.uniform(-.1,.1,2),0.];p=rng.uniform(-.7,.7,(2000,3));envelope=translation_sweep_shapes(shapes,v)
   covered=self_candidates(p,envelope,.05)
   for t in np.linspace(0,.5,31):
    moved=[]
    for shape in shapes:
     T=np.array(shape['matrix']);T[:3,3]+=np.array([*v[:2],0.])*t;moved.append(dict(shape,matrix=T.tolist()))
    self.assertFalse((self_candidates(p,moved,.05)&~covered).any())
 def test_curved_envelope_contains_legacy_and_exact_continuous_sweep(self):
  rng=np.random.default_rng(905)
  for _ in range(12):
   shapes=[self.shape(rng.uniform(-3,3),rng.uniform(-.6,.6,3))for i in range(3)]
   twist=[*rng.uniform(-.1,.1,2),rng.choice([-1,1])*rng.uniform(.01,.2)];vx,vy,w=twist
   p=rng.uniform(-1,1,(1800,3));covered=self_candidates(p,curved_sweep_shapes(shapes,twist),.05)
   for t in np.linspace(0,.5,41):
    c,s=math.cos(w*t),math.sin(w*t);R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    for move in [[vx*t,vy*t,0],[(vx*s+vy*(c-1))/w,(vx*(1-c)+vy*s)/w,0]]:
     group=[]
     for shape in shapes:
      T=np.array(shape['matrix']);T[:3,:3]=R@T[:3,:3];T[:3,3]=R@T[:3,3]+move;group.append(dict(shape,matrix=T.tolist()))
     self.assertFalse((self_candidates(p,group,.05)&~covered).any())
 def test_translation_and_joint_change_are_both_measured(self):
  a=self.shape();b=self.shape(center=(.001,0,0))
  drift=sweep_reference_drift([a],[b],[0,0,0],[.002,0,0],[.005,0,0])
  self.assertAlmostEqual(drift,.003)
  self.assertGreater(sweep_reference_drift([a],[b],[0,0,0],[.006,0,0],[.005,0,0]),.005)
 def test_rotation_of_distant_arm_is_not_treated_as_stationary(self):
  a=self.shape(center=(2.,0,0))
  self.assertGreater(sweep_reference_drift([a],[a],[0,0,0],[0,0,.003],[.005,0,0]),.005)
 def test_moving_base_requires_new_sweep_and_newly_approached_obstacle_stops(self):
  shape=self.shape(center=(0,0,1));frames={};prepared={};points=np.array([[.161,0,1.]])
  for sensor in ['front','back']:
   h=dict(sensor=sensor,sequence=1,pose=[0,0,0],source_age_s=.1)
   f=(h,points,99.9);frames[sensor]=f;prepared[sensor]=dict(frame=f,points_body=points,model_sha256='model',report={})
  self.assertGreater(sweep_reference_drift([shape],[shape],[0,0,0],[.01,0,0],[0,0,0]),.005)
  before=sweep_guard(frames,{},[0,0,0],[shape],'model',None,[0,0,0],100.,revalidation_reserve_m=.005,prepared=prepared)
  after=sweep_guard(frames,{},[.01,0,0],[shape],'model',None,[0,0,0],100.,revalidation_reserve_m=.005,prepared=prepared)
  self.assertTrue(before[0]);self.assertFalse(after[0]);self.assertEqual(after[1],'LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY')
 def test_changed_geometry_cannot_reuse_old_sweep(self):
  a=self.shape();b=dict(a,bounds=[[-.2,-.2,-.3],[.1,.2,.3]])
  self.assertTrue(math.isinf(sweep_reference_drift([a],[b],[0,0,0],[0,0,0],[0,0,0])))
  self.assertTrue(math.isinf(sweep_reference_drift([a],[],[0,0,0],[0,0,0],[0,0,0])))
 def test_newest_pose_still_requires_matching_source_and_epoch(self):
  loc=dict(source_boot_id='boot',odometry_epoch=0,calibration_sha256='cal')
  plan=dict(localization=loc,map_voxel_sha256='map')
  packet=dict(running=True,map_pose_valid=True,updated_wall_time=100.,odometry_age_s=.02,lidar_fix_age_s=.5,map_voxel_sha256='map',**loc)
  self.assertTrue(position_check(packet,plan,100.1)[0])
  for key,value in [('source_boot_id','new-boot'),('odometry_epoch',1),('calibration_sha256','other'),('map_voxel_sha256','other')]:
   result=position_check(dict(packet,**{key:value}),plan,100.1)
   self.assertFalse(result[0]);self.assertEqual(result[1],'LOCALIZATION_FRAME_OR_EPOCH_CHANGED')
 def test_newest_file_cannot_make_old_odometry_or_lidar_fresh(self):
  loc=dict(source_boot_id='boot',odometry_epoch=0,calibration_sha256='cal');plan=dict(localization=loc,map_voxel_sha256='map')
  p=dict(running=True,map_pose_valid=True,updated_wall_time=100.,odometry_age_s=.02,lidar_fix_age_s=.5,map_voxel_sha256='map',**loc)
  for packet,now in [(p,100.6),(p,99.9),(dict(p,odometry_age_s=.6),100.01),(dict(p,lidar_fix_age_s=3.1),100.01)]:self.assertFalse(position_check(packet,plan,now)[0])

if __name__=='__main__':unittest.main()
