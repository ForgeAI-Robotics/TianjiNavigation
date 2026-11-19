import unittest,sys,itertools
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import paths
from probe_cable_deferral import ProbeCableDeferral
from execution_safety import sweep_guard

class ProbeDeferralTests(unittest.TestCase):
 def setUp(self):
  self.sha='c'*64;lo=[-.03,-.03,.1];hi=[.03,.03,.92]
  self.shape=dict(link='assembly_lower_mount',name='pole',matrix=np.eye(4).tolist(),bounds=[lo,hi],world_corners=list(itertools.product(*zip(lo,hi))),mode='measured_or_fixed')
  self.cfg=dict(enabled=True,model_sha256=self.sha);self.p=np.array([[.079,0,.72]])
 def frozen(self):
  context=ProbeCableDeferral()
  with patch('cable_zones.settings',return_value=self.cfg):
   for side,seq in [('front',1),('back',2),('front',3),('back',4),('front',5),('back',6)]:context.observe(self.p,[self.shape],self.sha,[0,0,0],side,seq)
  context.freeze();return context
 def frames(self,p):return {side:(dict(pose=[0,0,0],source_age_s=0.,sequence=i),p,100.) for i,side in enumerate(['front','back'])}
 def test_initial_wire_approach_no_longer_interrupts_probe_but_normal_navigation_still_stops(self):
  context=self.frozen();T={s:np.eye(4) for s in ['front','back']}
  with patch('cable_zones.settings',return_value=self.cfg),patch('execution_safety.envelope_settings',return_value=dict(enabled=False)),patch('execution_safety.cable_settings',return_value=dict(enabled=False)):
   self.assertFalse(sweep_guard(self.frames(self.p),T,[0,0,0],[self.shape],self.sha,None,[.005,0,0],100.)[0])
   r=sweep_guard(self.frames(self.p),T,[0,0,0],[self.shape],self.sha,None,[.005,0,0],100.,probe_deferral=context)
   self.assertTrue(r[0]);self.assertEqual(r[1],'PROBE_INITIAL_WIRES_DEFERRED_UNTIL_5CM');self.assertFalse(context.summary()['permanent_filter'])
 def test_obstacle_outside_frozen_wire_region_still_stops(self):
  context=self.frozen();p=np.vstack([self.p,[[.079,0,.52]]]);T={s:np.eye(4) for s in ['front','back']}
  with patch('cable_zones.settings',return_value=self.cfg),patch('execution_safety.envelope_settings',return_value=dict(enabled=False)),patch('execution_safety.cable_settings',return_value=dict(enabled=False)):
   r=sweep_guard(self.frames(p),T,[0,0,0],[self.shape],self.sha,None,[.005,0,0],100.,probe_deferral=context)
   self.assertFalse(r[0]);self.assertEqual(len(context.seeds),1)
 def test_intermittent_wire_samples_inside_same_region_do_not_retrigger_stop(self):
  context=self.frozen();p=np.vstack([self.p,[[.079,0,.82]]]);T={s:np.eye(4) for s in ['front','back']}
  with patch('cable_zones.settings',return_value=self.cfg),patch('execution_safety.envelope_settings',return_value=dict(enabled=False)),patch('execution_safety.cable_settings',return_value=dict(enabled=False)):
   self.assertTrue(sweep_guard(self.frames(p),T,[0,0,0],[self.shape],self.sha,None,[.005,0,0],100.,probe_deferral=context)[0])
   self.assertEqual(len(context.seeds),1)
 def test_surface_protection_overrides_initial_wire_exemption(self):
  context=self.frozen();x,y=np.meshgrid(np.arange(-.16,.321,.02),np.arange(-.24,.241,.02));p=np.c_[x.ravel(),y.ravel(),np.full(x.size,.72)]
  with patch('cable_zones.settings',return_value=self.cfg):self.assertFalse(context.mask(p,[self.shape],self.sha,[0,0,0]).any())
 def test_both_motion_hypotheses_retained_only_inside_bounded_probe(self):
  context=self.frozen()
  with patch('cable_zones.settings',return_value=self.cfg):
   p=np.vstack([self.p,self.p-[.04,0,0]])
   self.assertTrue(context.mask(p,[self.shape],self.sha,[.04,0,0]).all())
   with self.assertRaisesRegex(ValueError,'OUTSIDE_BOUNDED_TEST'):context.mask(p,[self.shape],self.sha,[.07,0,0])
 def test_five_cm_scan_sequence_is_not_stopped_by_the_same_initial_wire(self):
  context=self.frozen();T={s:np.eye(4) for s in ['front','back']}
  with patch('cable_zones.settings',return_value=self.cfg),patch('execution_safety.envelope_settings',return_value=dict(enabled=False)),patch('execution_safety.cable_settings',return_value=dict(enabled=False)):
   for x in np.linspace(0,.052,27):
    frames={s:(dict(pose=[float(x),0,0],source_age_s=0.,sequence=i),self.p,100.) for i,s in enumerate(['front','back'])}
    self.assertTrue(sweep_guard(frames,T,[float(x),0,0],[self.shape],self.sha,None,[.005,0,0],100.,probe_deferral=context)[0])
 def test_blocked_goal_can_supply_intent_but_immediate_obstacle_still_blocks_prefix(self):
  from test_navigation import NativeTests
  from policy import inspection_intent_packet
  from cable_precheck import check_prefix
  import tempfile,json
  fixture=NativeTests();packet=fixture.packet([1.,0.,0.],[[1.,0.,.3]])
  self.assertIsNone(fixture.run_plan(packet)['selected']);intent=inspection_intent_packet(packet);selected=fixture.run_plan(intent)['selected'];self.assertIsNotNone(selected);self.assertEqual(packet['world']['occupied_points'],[[1.,0.,.3]])
  with tempfile.TemporaryDirectory() as directory:
   file=Path(directory)/'world.json';file.write_text(json.dumps(packet['world']));self.assertTrue(check_prefix(file,selected['path'])['clear'])
   packet['world']['occupied_points'].append([.25,0,.3]);file.write_text(json.dumps(packet['world']))
   with self.assertRaisesRegex(ValueError,'LOCAL_PROBE_COLLISION'):check_prefix(file,selected['path'])

if __name__=='__main__':unittest.main()
