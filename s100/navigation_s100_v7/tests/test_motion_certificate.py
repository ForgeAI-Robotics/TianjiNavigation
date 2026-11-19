import sys,unittest,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import paths
from motion_certificate import certify_motion,twist_pose
from execution_safety import curved_sweep_shapes,sweep_reference_drift,sweep_guard,REQUIRED_JOINTS
from environment import self_candidates
from saved_map import base_transform
from certification_recovery import classify,POSE_RECHECK
from guard_recovery import GuardRecovery
from navigation_admission import admission,REQUIRED

class MotionCertificateTests(unittest.TestCase):
 def shape(self):return dict(link='arm',matrix=np.eye(4).tolist(),bounds=[[-.2,-.15,.2],[.8,.15,.8]],mode='measured_or_fixed')
 def test_five_cm_per_second_advance_is_checked_motion_not_residual_error(self):
  s=self.shape();twist=[0,.05,0];measured=twist_pose(twist,.25)
  self.assertGreater(sweep_reference_drift([s],[s],[0,0,0],measured,twist),.005)
  c=certify_motion([s],[0,0,0],measured,twist,.001)
  self.assertTrue(c['proven']);self.assertAlmostEqual(c['phase_s'],.25);self.assertAlmostEqual(c['actual_base_translation_m'],.0125);self.assertLess(c['total_residual_bound_m'],.005)
 def test_unmodeled_sideways_or_excess_joint_motion_stays_rejected(self):
  s=[self.shape()]
  self.assertFalse(certify_motion(s,[0,0,0],[0,.012,0],[.05,0,0],0)['proven'])
  self.assertFalse(certify_motion(s,[0,0,0],[.01,0,0],[.05,0,0],.006)['proven'])
  self.assertFalse(certify_motion(s,[0,0,0],[.04,0,0],[.05,0,0],0)['proven'])
 def test_actual_standstill_does_not_get_assumed_command_motion(self):
  c=certify_motion([self.shape()],[0,0,0],[0,0,0],[.05,0,0],0)
  self.assertTrue(c['proven']);self.assertEqual(c['phase_s'],0.)
 def test_exact_curves_and_rotation_match_newer_measured_odometry(self):
  for twist in [[.03,.01,.15],[.03,-.01,-.12],[0,0,.2],[0,0,-.2]]:
   for phase in [.1,.25,.35]:
    c=certify_motion([self.shape()],[0,0,0],twist_pose(twist,phase),twist,.001)
    self.assertTrue(c['proven']);self.assertLess(c['base_residual_bound_m'],1e-6)
 def test_certified_future_vertices_remain_inside_extended_checked_envelope(self):
  rng=np.random.default_rng(42);shape=self.shape();twist=[.04,.01,.1];phase=.24;pred=base_transform(twist_pose(twist,phase));extra=base_transform([.0005,-.0003,.0004]);new=pred@extra;new_pose=[new[0,3],new[1,3],math.atan2(new[1,0],new[0,0])]
  c=certify_motion([shape],[0,0,0],new_pose,twist,.001);self.assertTrue(c['proven']);checked=curved_sweep_shapes([shape],twist,horizon=.85)
  corners=np.array([[x,y,z]for x in [-.2,.8]for y in [-.15,.15]for z in [.2,.8]])+np.array([0,0,.001])
  offsets=rng.normal(size=(8,3));offsets=offsets/np.linalg.norm(offsets,axis=1)[:,None]*.049
  for t in np.linspace(0,.5,61):
   T=new@base_transform(twist_pose(twist,t));points=(corners+offsets)@T[:3,:3].T+T[:3,3];self.assertTrue(self_candidates(points,checked,.055).all())
 def test_extended_guard_stops_at_newly_covered_front_obstacle(self):
  shape=dict(self.shape(),bounds=[[-.2,-.15,.2],[.2,.15,.8]]);p=np.array([[.291,0,.4]]);frames={};prepared={}
  for sensor in ['front','back']:
   h=dict(sensor=sensor,sequence=1,pose=[0,0,0],source_age_s=.1);frames[sensor]=(h,p,100.);prepared[sensor]=dict(frame=frames[sensor],points_body=p,model_sha256='model',report={})
  self.assertTrue(sweep_guard(frames,{},[0,0,0],[shape],'model',None,[.05,0,0],100.,prepared=prepared,revalidation_reserve_m=.005)[0])
  self.assertFalse(sweep_guard(frames,{},[0,0,0],[shape],'model',None,[.05,0,0],100.,prepared=prepared,revalidation_reserve_m=.005,motion_advance_s=.35)[0])
 def test_output_cannot_outlive_remaining_swept_pose_coverage(self):
  g=dict.fromkeys(REQUIRED,0.);g.update(state='CLEAR',path_checked=True,operator_estop_declared=True,map_sha256='a'*64,model_sha256='b'*64,swept_pose_lease_s=.2)
  self.assertTrue(admission(g,.19));self.assertFalse(admission(g,.201));self.assertFalse(admission(dict(g,swept_pose_lease_s=.4),0))

class CertificateRecoveryTests(unittest.TestCase):
 def inputs(self):
  q=dict.fromkeys(REQUIRED_JOINTS,0.);saved=dict(joint_positions_rad=q);j=dict(joint_positions_rad=q,model_sha256='model',observation_time=100.)
  loc=dict(source_boot_id='boot',odometry_epoch=0,calibration_sha256='cal');record=dict(localization=loc,map_voxel_sha256='map');position=dict(running=True,map_pose_valid=True,updated_wall_time=100.,odometry_age_s=.02,lidar_fix_age_s=.1,map_voxel_sha256='map',**loc)
  frames={s:(dict(sensor=s,boot_id='boot',epoch=0,calibration_sha256='cal',source_age_s=.1),None,10.)for s in ['front','back']}
  return saved,j,position,record,frames
 def test_only_fresh_inputs_qualify_for_geometry_wait(self):
  saved,j,p,record,frames=self.inputs();self.assertEqual(classify(POSE_RECHECK,saved,j,'model',p,record,frames,100.1,10.1),(True,POSE_RECHECK))
  self.assertFalse(GuardRecovery().waiting(POSE_RECHECK,None,10.))
  for joint,position,now_mono,expected in [(dict(j,observation_time=99.),p,10.1,'JOINT_FEEDBACK_STALE'),(j,dict(p,lidar_fix_age_s=4.),10.1,'LOCALIZATION_STALE'),(j,p,11.3,'LIDAR_STALE'),(j,dict(p,source_boot_id='other'),10.1,'LOCALIZATION_FRAME_OR_EPOCH_CHANGED')]:
   self.assertEqual(classify(POSE_RECHECK,saved,joint,'model',position,record,frames,100.1,now_mono),(False,expected))
 def test_collision_and_identity_are_not_reclassified_as_geometric_retry(self):
  saved,j,p,record,f=self.inputs()
  for reason in ['LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY','URDF_VERSION_MISMATCH','POSTURE_CHANGED_REPLAN_REQUIRED']:
   self.assertEqual(classify(reason,saved,j,'model',p,record,f,100.1,10.1),(False,reason))
  f['front'][0]['epoch']=1;self.assertEqual(classify(POSE_RECHECK,saved,j,'model',p,record,f,100.1,10.1),(False,'LIDAR_FRAME_CHANGED'))
if __name__=='__main__':unittest.main()
