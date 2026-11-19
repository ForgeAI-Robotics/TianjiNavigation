"""Read-only terminal-command stop reconciliation; the broker keeps the journal."""
import copy,json,os,subprocess,threading,time
from pathlib import Path
from navigation_task_contract import TERMINAL
from navigation_stop_evidence import valid_certificate

def owner_alive(job):
 owner=job.get('process')or{};pid=owner.get('pid')
 if not pid:return False
 try:
  ticks=Path('/proc',str(pid),'stat').read_text().rsplit(')',1)[1].split()[19]
  return ticks==owner.get('pid_start_ticks')
 except (OSError,ValueError,IndexError):return False

def eligible(job,active,busy,simulation):
 return (not simulation and job.get('state')in TERMINAL and bool(job.get('brain_command')) and
         not valid_certificate((job.get('result')or{}).get('stop_evidence')or{}) and not active and not busy and not owner_alive(job))

def collect(job):
 from client import motor_trace_environment
 env=motor_trace_environment();env.update(ROS_DOMAIN_ID='0',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',CYCLONEDDS_URI='file:///home/sunrise/tianji_table_approach_dev/navigation_s100_v7/config/input_cyclonedds.xml')
 script=Path(__file__).with_name('passive_stop_probe.py')
 proc=subprocess.run(['/bin/bash','-c','source /opt/ros/humble/setup.bash; exec /usr/bin/python3 "$1"','tianji-readonly-stop',str(script)],env=env,capture_output=True,text=True,timeout=6)
 if proc.returncode:raise ValueError('PASSIVE_STOP_PROBE_PROCESS_FAILED')
 return json.loads(proc.stdout.strip().splitlines()[-1])

class TerminalStopObserver:
 def __init__(self,manager,collector=collect):self.manager=manager;self.collector=collector;self.running=set();self.last={}
 def schedule(self,job):
  m=self.manager;ident=job['id'];now=time.monotonic()
  # Called under the manager condition; this cache owns no task/business state.
  if m.closed:return
  if ident in self.running or now-self.last.get(ident,-1e9)<5:return
  if not eligible(job,m.active,m.busy(),m.simulation):return
  self.running.add(ident);self.last[ident]=now
  threading.Thread(target=self._observe,args=(copy.deepcopy(job),),daemon=True,name='terminal-stop-readonly').start()
 def _observe(self,original):
  m=self.manager;ident=original['id']
  try:
   evidence=self.collector(original)
   if not valid_certificate(evidence):
    self._unconfirmed(original,evidence);return
   evidence.update(command_id=original['brain_command']['command_id'],task_id=original['brain_command']['task_id'],rechecked_after_terminal=True)
   with m.condition:
    current=m.jobs.get(ident)
    if m.closed or not current or current.get('revision')!=original.get('revision')or not eligible(current,m.active,m.busy(),m.simulation):return
    result=copy.deepcopy(current.get('result')or{});result['stop_evidence']=evidence
    m._change(current,current['state'],dict(result=result),event_type='physical_stop_reconfirmed',durable=True)
  except Exception as exc:self._unconfirmed(original,dict(reason='PASSIVE_STOP_RECHECK_ERROR',error=str(exc)))
  finally:
   with m.condition:self.running.discard(ident)
 def _unconfirmed(self,original,evidence):
  m=self.manager
  with m.condition:
   current=m.jobs.get(original['id'])
   if m.closed or not current or current.get('revision')!=original.get('revision'):return
   m._change(current,current['state'],dict(stop_recheck=dict(checked_at=time.time(),reason=evidence.get('reason'),error=evidence.get('error'))),event_type='physical_stop_recheck_unconfirmed',durable=True)
