import sys,unittest,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from navigation_control import limited_twist,UNOBSERVED_TRAVEL_BUDGET_M
from posture_lease import joint_shift_bound,revalidate
from execution_safety import REQUIRED_JOINTS,sweep_reference_drift

class StutterLatencyTests(unittest.TestCase):
 def test_speed_no_longer_oscillates_as_valid_scan_ages(self):
  for command in [[.08,0,.12],[0,.04,.1],[0,0,-.2]]:
   outputs=[limited_twist(command,age,1.2) for age in [0.,.2,.5,.9,1.2]]
   for output in outputs:self.assertEqual(output,outputs[0])
   v=outputs[0];self.assertLessEqual((math.hypot(*v[:2])+1.2*abs(v[2]))*1.45,UNOBSERVED_TRAVEL_BUDGET_M+1e-10)
   ratios=[v[i]/command[i] for i in range(3) if command[i]]
   self.assertLess(max(ratios)-min(ratios),1e-12)
 def test_base_and_joint_bound_share_original_reserve(self):
  q=dict.fromkeys(REQUIRED_JOINTS,0.);fresh=dict(q,Joint1_L=.002)
  shape=dict(link='arm',mode='measured_or_fixed',matrix=np.eye(4).tolist(),bounds=[[-.1,-.1,0],[.1,.1,1.]])
  weights=dict.fromkeys(q,1.)
  base=sweep_reference_drift([shape],[shape],[0,0,0],[.002,0,0],[.01,0,.02])
  self.assertAlmostEqual(base,.002);self.assertAlmostEqual(joint_shift_bound([shape],q,fresh,weights),.002)
  reference=dict(joint_positions_rad=q,shapes=[shape],reserve_m=.005-base,motion_weights_m=weights)
  def fk(joints):
   T=np.eye(4);T[0,3]=joints['Joint1_L'];return [dict(shape,matrix=T.tolist())]
  current=dict(joint_positions_rad=fresh,model_sha256='a'*64,observation_time=100.)
  self.assertIsNotNone(revalidate(reference,current,dict(joint_positions_rad=q),'a'*64,fk,100.01))
  current['joint_positions_rad']=dict(q,Joint1_L=.004)
  self.assertIsNone(revalidate(reference,current,dict(joint_positions_rad=q),'a'*64,fk,100.01))
 def test_unsupported_and_nonfinite_bounds_require_exact_fk(self):
  shape=dict(mode='measured_or_fixed')
  for weights in [None,{},dict(a=-1),dict(a=float('nan'))]:self.assertTrue(math.isinf(joint_shift_bound([shape],dict(a=0.),dict(a=.001),weights)))
  self.assertTrue(math.isinf(joint_shift_bound([dict(mode='proxy')],dict(a=0.),dict(a=.001),dict(a=1))))
if __name__=='__main__':unittest.main()
