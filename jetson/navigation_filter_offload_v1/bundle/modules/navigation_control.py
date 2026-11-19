"""Local control transport. Real ROS bridge is created ONLY after manual start+valid guards."""
import sys,subprocess,threading,time,math,collections,json
from common import ROOT
from wire import read_packet,write_packet
from navigation_admission import admission

_limits=json.loads((ROOT/'config/motion_policy.json').read_text()).get('navigation_test_limits',{})
TRANSLATION_CAP_MPS=float(_limits.get('translation_cap_mps',.03))
UNOBSERVED_TRAVEL_BUDGET_M=float(_limits.get('unobserved_travel_budget_m',.04))
LATERAL_CAP_MPS=float(_limits.get('lateral_cap_mps',.04))
ROTATING_TRANSLATION_CAP_MPS=float(_limits.get('rotating_translation_cap_mps',TRANSLATION_CAP_MPS))
ROTATING_LATERAL_CAP_MPS=float(_limits.get('rotating_lateral_cap_mps',LATERAL_CAP_MPS))
ROTATING_TRAVEL_BUDGET_M=float(_limits.get('rotating_unobserved_travel_budget_m',UNOBSERVED_TRAVEL_BUDGET_M))
if not .01<=TRANSLATION_CAP_MPS<=.10 or not .02<=UNOBSERVED_TRAVEL_BUDGET_M<=.145 or not .01<=LATERAL_CAP_MPS<=.10:raise ValueError('INVALID_NAVIGATION_SPEED_LIMITS')
if not .01<=ROTATING_TRANSLATION_CAP_MPS<=TRANSLATION_CAP_MPS or not .01<=ROTATING_LATERAL_CAP_MPS<=LATERAL_CAP_MPS or not .02<=ROTATING_TRAVEL_BUDGET_M<=UNOBSERVED_TRAVEL_BUDGET_M:raise ValueError('INVALID_ROTATING_SPEED_LIMITS')

def limited_twist(twist,cloud_age,radius):
 x,y,w=map(float,twist)
 if not all(math.isfinite(v) for v in [x,y,w,cloud_age,radius]) or cloud_age<0 or radius<0:raise ValueError('INVALID_TWIST_OR_OBSERVATION')
 # One scale preserves the planned curvature and translation direction.
 # Capping vx alone while keeping wz made the executed turn much tighter.
 # The adjustable translation profile applies to pure translation. Keep the previous profile
 # for ANY rotation so both pure and mixed angular commands remain unchanged.
 rotating=w!=0.
 linear_cap=ROTATING_TRANSLATION_CAP_MPS if rotating else TRANSLATION_CAP_MPS
 lateral_cap=ROTATING_LATERAL_CAP_MPS if rotating else LATERAL_CAP_MPS
 travel_budget=ROTATING_TRAVEL_BUDGET_M if rotating else UNOBSERVED_TRAVEL_BUDGET_M
 scale=1.
 for value,cap in [(x,.12 if x>=0 else .1),(y,lateral_cap),(w,.2)]:
  if abs(value)>cap:scale=min(scale,cap/abs(value))
 length=math.hypot(x,y)
 if length>linear_cap:scale=min(scale,linear_cap/length)
 x*=scale;y*=scale;w*=scale
 # Bound unobserved swept-surface travel; limits remain below configured caps.
 # Use the full admitted cloud lifetime for a stable, conservative cap.
 # Otherwise every new scan accelerates the wheels and its aging slows them.
 # Actual age still gates every output; this never extends the 1.2s deadline.
 travel=(math.hypot(x,y)+radius*abs(w))*(max(1.2,cloud_age)+.25)
 if travel>travel_budget:scale=travel_budget/travel;x*=scale;y*=scale;w*=scale
 return [x,y,w]
class ControlTransport:
 ros_load=False
 def __init__(self,real,posture_refresh=None,load_test_no_motion=False):
  if real and load_test_no_motion:raise ValueError('LOAD_TEST_CANNOT_ENABLE_MOTION')
  self.ros_load=bool(load_test_no_motion)
  self.real=bool(real);self.log=(ROOT/'runtime/navigation_control.log').open('ab');self.process=subprocess.Popen([sys.executable,str(ROOT/'modules/control_agent.py'),'--load-test-no-motion' if self.ros_load else '--enable-motion' if real else '--preview'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,bufsize=0);self.stop=threading.Event();self.lock=threading.Lock();self.status={};self.decision=None;self.sequence=0;self.sent=0;self.nonzero=0;self.error=None;self.gate_counts=collections.Counter()
  self.posture_refresh=posture_refresh;self.zero_epoch=0
  self.reader=threading.Thread(target=self.read,daemon=True);self.pump=threading.Thread(target=self.send_loop,daemon=True);self.reader.start();self.pump.start()
 def read(self):
  try:
   while not self.stop.is_set():
    h,p=read_packet(self.process.stdout)
    if p is not None or h.get('kind')!='control_status':raise ValueError('CONTROL_STATUS_REQUIRED')
    if self.ros_load and (h.get('mode')!='load_test_no_motion' or h.get('hardware_output_enabled') is not False):raise ValueError('LOAD_TEST_HARDWARE_ISOLATION_REQUIRED')
    if not self.real and not self.ros_load and (h.get('ros_publishers_created') or h.get('bridge_started')):raise ValueError('PREVIEW_CREATED_MOTION_BRIDGE')
    with self.lock:
     self.status=dict(h,received=time.monotonic());self.gate_counts['output_nonzero' if any(abs(v)>1e-8 for v in h.get('output',[0,0,0])) else 'output_zero']+=1;self.gate_counts[h.get('reason','UNKNOWN')]+=1
  except Exception as exc:
   if not self.stop.is_set():self.error=str(exc)
 def update(self,twist,guard,posture_reference=None):
  with self.lock:self.decision=(list(twist),dict(guard),time.monotonic(),posture_reference)
 def ready(self):
  with self.lock:status=dict(self.status)
  return bool(not self.error and 'challenge' in status and 0<=time.monotonic()-status['received']<=.2 and (not (self.real or self.ros_load) or status.get('bridge_ready') is True))
 def output_stats(self):
  with self.lock:return dict(self.gate_counts)
 def zero(self):
  with self.lock:self.decision=None;self.zero_epoch+=1
 def send_loop(self):
  try:
   while not self.stop.wait(.05):
    now=time.monotonic()
    with self.lock:status=dict(self.status);decision=self.decision;epoch=self.zero_epoch
    if 'challenge' not in status:continue
    guard=dict(state='BLOCKED');twist=[0.,0.,0.];age=0.
    if decision:
     candidate,original,stamp,reference=decision;age=now-stamp;guard=dict(original)
     refreshed=None
     if reference is not None and self.posture_refresh is not None:
      try:refreshed=self.posture_refresh(reference)
      except (ValueError,OSError):
       # One absent/stale measured snapshot invalidates this output only.
       # Keep sending zeros/challenges; the navigation supervisor applies its
       # normal recovery/stop policy. Never kill the sender on a data fault.
       with self.lock:self.gate_counts['posture_refresh_rejected']+=1
     # Posture validation can overlap a new supervisor decision or challenge.
     # Do not send an expired old snapshot as zero over a freshly admitted
     # command. Explicit zero epochs still take precedence immediately below.
     with self.lock:
      latest_status=dict(self.status);superseded=self.zero_epoch==epoch and self.decision is not decision
      if superseded:self.gate_counts['superseded_decision_rechecks']+=1
     if superseded:continue
     status=latest_status
     now=time.monotonic();age=now-stamp
     # All original deadlines keep aging. Only a newly measured, explicitly
     # bounded posture may replace its own age; no scene/pose stamp is renewed.
     for k in ('localization_age_s','pose_age_s','clearance_age_s','guard_age_s','cloud_age_s','operator_age_s'):
      if k in guard:guard[k]+=age
     posture_ok=reference is None or self.posture_refresh is None or refreshed is not None
     if refreshed is not None:
      guard['pose_age_s']=refreshed['age_s'];guard['posture_feedback_observation_time']=refreshed['observation_time'];guard['posture_output_revalidated']=True
      with self.lock:self.gate_counts['measured_posture_revalidated']+=1
     # The full guard enforces its original 350ms age plus every source TTL.
     # A separate 200ms cut caused zeros while that same guard was valid.
     admitted=admission(guard,0.)
     if posture_ok and admitted and now-status['received']<=.2 and (not (self.real or self.ros_load) or status.get('bridge_ready') is True):twist=candidate
     else:
      from guard_recovery import expired_deadlines
      causes=['deadline_'+k for k in expired_deadlines(guard)]
      if not posture_ok:causes.append('posture_geometry_rejected')
      if now-status['received']>.2:causes.append('control_status_stale')
      if not admitted and not causes:causes.append('admission_other_rejected')
      with self.lock:self.gate_counts.update(causes)
    with self.lock:
     if self.zero_epoch!=epoch:guard=dict(state='BLOCKED');twist=[0.,0.,0.]
    self.sequence+=1;write_packet(self.process.stdin,dict(kind='command',sequence=self.sequence,challenge=status['challenge'],command=twist,key_age_s=0.,mapping=dict(state='idle'),navigation=guard));self.sent+=1;self.nonzero+=int(any(abs(v)>1e-8 for v in twist))
  except Exception as exc:
   if not self.stop.is_set():self.error=str(exc)
 def close(self):
  self.zero();time.sleep(.12);self.stop.set();self.pump.join(timeout=1)
  try:self.process.stdin.close()
  except OSError:pass
  try:self.process.wait(timeout=5)
  except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=3)
  self.reader.join(timeout=1);self.process.stdout.close();self.log.close()
