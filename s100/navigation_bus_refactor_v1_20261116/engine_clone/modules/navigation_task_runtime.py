"""S100 task broker: owns local navigator lifecycle, never computes velocities.

HTTP and GUI are clients. All actual navigation stays in onboard_navigation;
its original posture, sensor, collision, command-lease and stop checks remain.
"""
import collections,copy,json,os,queue,secrets,signal,subprocess,threading,time,uuid
from pathlib import Path
from common import atomic_json,lock_held
from navigation_task_contract import ApiError,TERMINAL,Waypoints,normalize,fingerprint,terminal_result,status_summary

class TaskStore:
 def __init__(self,folder):
  self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True);self.queue=queue.Queue(256);self.errors=[];self.dropped=0
  self.worker=threading.Thread(target=self._write,daemon=True,name='task_api_journal');self.worker.start()
 def load(self):
  rows=[]
  for p in self.folder.glob('*/task.json'):
   try:rows.append(json.loads(p.read_text()))
   except (OSError,ValueError):continue
  return rows
 def save(self,record,event,wait=False):
  ack=threading.Event();result=[];item=(copy.deepcopy(record),copy.deepcopy(event),ack,result,wait)
  try:self.queue.put(item,timeout=2. if wait else 0.)
  except queue.Full:
   self.dropped+=1
   if wait:raise ApiError(503,'TASK_JOURNAL_BUSY')
   return
  if wait:
   if not ack.wait(5.):raise ApiError(503,'TASK_JOURNAL_TIMEOUT')
   if result:raise ApiError(503,'TASK_JOURNAL_ERROR',result[0])
 def _write(self):
  while True:
   item=self.queue.get()
   if item is None:return
   record,event,ack,result,durable=item
   try:
    folder=self.folder/record['id'];folder.mkdir(exist_ok=True)
    with(folder/'events.jsonl').open('a')as f:
     f.write(json.dumps(event,ensure_ascii=False,allow_nan=False)+'\n')
     if durable:f.flush();os.fsync(f.fileno())
    atomic_json(folder/'task.json',record)
    if durable:
     with(folder/'task.json').open('rb')as f:os.fsync(f.fileno())
     for directory in [folder,self.folder]:
      fd=os.open(directory,os.O_RDONLY|os.O_DIRECTORY)
      try:os.fsync(fd)
      finally:os.close(fd)
   except Exception as exc:result.append(str(exc));self.errors.append(str(exc))
   finally:ack.set()
 def events(self,task_id):
  try:return [json.loads(s)for s in(self.folder/task_id/'events.jsonl').read_text().splitlines()[-512:]]
  except (OSError,ValueError):return []
 def close(self):
  self.queue.put(None,timeout=5);self.worker.join(timeout=8)

class NavigatorRunner:
 def __init__(self,root,motion_enabled=False,operator_estop_declared=False):
  self.root=Path(root);self.motion_enabled=motion_enabled;self.estop=operator_estop_declared
 def command(self,job):
  req=job['request'];kind=job['kind'];env=dict(os.environ,TIANJI_NAV_SCENE=req['scene'],OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
  if kind=='plan':
   args=[str(self.root/'preview_plan.sh'),'--goal',str(req['goal'])]
   for key,flag in [('ignore_head_camera','--ignore-head-camera'),('open_grippers','--open-grippers'),('use_saved_arms','--use-saved-arms')]:args += [flag,str(int(req['options'][key]))]
  else:
   if kind=='navigate'and not(self.motion_enabled and self.estop):raise ApiError(409,'MOTION_DISABLED')
   args=[str(self.root/'execute.sh'),'--goal',str(req['goal']),'--session',job['id'],'--translation-cap-mps',str(req['translation_cap_mps'])]
   # The existing isolated simulator uses the same admission schema as the
   # CLI debug path. Its declaration applies ONLY to this no-motion test;
   # it neither enables hardware output nor changes server motion permission.
   args+=['--enable-motion','--operator-estop-ready']if kind=='navigate'else['--preview','--load-test-no-motion','--operator-estop-ready','--duration',str(req['duration_s'])]
  if job.get('brain_command'):
   file=self.root/'runtime/task_api/tasks'/job['id']/'coordinate_target.json'
   env['TIANJI_NAV_COORDINATE_TARGET_FILE']=str(file)
   env['TIANJI_NAV_COORDINATE_TARGET_SHA256']=__import__('hashlib').sha256(json.dumps(job['target'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
  return args,env
 def run(self,job,cancel,emit):
  args,env=self.command(job);folder=self.root/'runtime/task_api/tasks'/job['id'];folder.mkdir(parents=True,exist_ok=True)
  if job.get('brain_command'):
   (folder/'coordinate_target.json').write_text(json.dumps(job['target'],sort_keys=True,separators=(',',':')))
  process=subprocess.Popen(args,cwd=self.root,env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1,start_new_session=True,close_fds=True)
  def ticks():
   try:return Path('/proc',str(process.pid),'stat').read_text().rsplit(')',1)[1].split()[19]
   except OSError:return None
  try:emit('spawn',dict(pid=process.pid,pid_start_ticks=ticks(),session=job['id']))
  except Exception:
   process.stdin.close()
   try:os.killpg(process.pid,signal.SIGINT);process.wait(timeout=8)
   except (ProcessLookupError,subprocess.TimeoutExpired):
    if process.poll()is None:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
   process.stdout.close();process.stderr.close()
   raise
  finished=threading.Event();started=time.monotonic();timed_out=[False];tail=collections.deque(maxlen=40);final=None;preview=None
  def heartbeat():
   stop_since=None
   try:
    while not finished.is_set()and process.poll()is None:
     limit=job['request'].get('timeout_s',0.)
     if limit and time.monotonic()-started>limit:timed_out[0]=True;cancel.set()
     if cancel.is_set():
      if stop_since is None:
       stop_since=time.monotonic()
       if job['kind']!='plan':process.stdin.write(json.dumps(dict(kind='stop',session=job['id']))+'\n');process.stdin.flush()
       else:os.killpg(process.pid,signal.SIGINT)
       process.stdin.close() # EOF also stops the unchanged navigator lease.
      if time.monotonic()-stop_since>10.:os.killpg(process.pid,signal.SIGTERM)
      if time.monotonic()-stop_since>13.:os.killpg(process.pid,signal.SIGKILL)
     elif job['kind']!='plan':
      process.stdin.write(json.dumps(dict(kind='heartbeat',session=job['id']))+'\n');process.stdin.flush()
     finished.wait(.1)
   except (OSError,ValueError,BrokenPipeError):
    # A broken lease must terminate the owned process, not become an orphan.
    if process.poll()is None:
     try:os.killpg(process.pid,signal.SIGINT)
     except ProcessLookupError:pass
  def errors():
   for line in process.stderr:
    tail.append(line.rstrip()[:2000]);emit('log',dict(stream='stderr',message=line.rstrip()[:2000]))
  beat=threading.Thread(target=heartbeat,daemon=True);err=threading.Thread(target=errors,daemon=True);beat.start();err.start()
  try:
   for line in process.stdout:
    if line.startswith('STATUS '):
     try:
      raw=json.loads(line[7:])
      if raw.get('session')!=job['id']:raise ValueError('NAVIGATOR_SESSION_MISMATCH')
      summary=status_summary(raw);emit('navigator',summary)
      if raw.get('finished'):final=summary
     except (ValueError,TypeError)as exc:emit('log',dict(stream='protocol',message=str(exc)))
    elif line.startswith('PREVIEW_RESULT '):
     preview=json.loads(line[len('PREVIEW_RESULT '):]);emit('log',dict(stream='plan',message=line.rstrip()[:2000]))
    else:emit('log',dict(stream='stdout',message=line.rstrip()[:2000]))
   code=process.wait();err.join(timeout=2)
   artifacts={}
   if job['kind']=='plan'and preview:
    out=Path(preview['folder']).resolve();base=(self.root/'results').resolve()
    if out.parent!=base:raise ValueError('INVALID_PLAN_ARTIFACT_PATH')
    final=json.loads((out/'planning/plan.json').read_text())
    if final['target']['id']!=job['request']['goal']or final['target']['map_voxel_sha256']!=job['target']['map_voxel_sha256']:raise ValueError('PLAN_TARGET_MISMATCH')
    artifacts=dict(folder=str(out),plan_file=str(out/'planning/plan.json'))
   elif final and final.get('motor_trace_file'):
    out=Path(final['motor_trace_file']).parent
    if out.parent.resolve()==(self.root/'results').resolve():artifacts=dict(folder=str(out),plan_file=str(out/'planning/plan.json'),navigation_log=str(out/'navigation_execution_history.jsonl'),motor_trace=final['motor_trace_file'])
   state,result=terminal_result(job['kind'],code,final,cancel.is_set(),timed_out[0]);result.update(exit_code=code,stderr_tail=list(tail),artifacts=artifacts)
   if final and job.get('brain_command'):
    result.update(final_xyt=final.get('final_xyt'),stop_evidence=final.get('navigation_stop_evidence'),goal_error=final.get('goal_error'))
   if job['kind']=='test':result['isolated_test_operator_declaration']=True
   if job['kind']=='plan'and final:result.update(plan_state=final.get('state'),preview_assumptions=final.get('preview_posture_assumptions'),selected=None if not final.get('selected')else{k:final['selected'][k]for k in ['profile','length_m','geometry_recheck_passed']if k in final['selected']})
   return state,result
  finally:
   finished.set();beat.join(timeout=2)
   if process.poll()is None:
    try:process.stdin.close()
    except OSError:pass
    try:os.killpg(process.pid,signal.SIGINT);process.wait(timeout=8)
    except (ProcessLookupError,subprocess.TimeoutExpired):
     if process.poll()is None:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
   for f in [process.stdin,process.stdout,process.stderr]:
    try:f.close()
    except OSError:pass

class TaskManager:
 def __init__(self,root,motion_enabled=False,operator_estop_declared=False,default_scene='office',runner=None,busy=None):
  self.root=Path(root);self.scene=default_scene;self.motion_enabled=bool(motion_enabled and operator_estop_declared);self.registry=Waypoints(root);self.runner=runner or NavigatorRunner(root,self.motion_enabled,operator_estop_declared)
  self.store=TaskStore(self.root/'runtime/task_api/tasks');self.lock=threading.RLock();self.condition=threading.Condition(self.lock);self.jobs={};self.keys={};self.cancels={};self.threads={};self.events_by_job={};self.active=None;self.closed=False;self.last_progress={};self.commands={};self.simulation=bool(getattr(self.runner,'simulation',False))
  self.busy=busy or(lambda:lock_held(self.root/'runtime/navigation_execution.lock')or lock_held(self.root/'runtime/navigation.lock'))
  for job in self.store.load():
   self.jobs[job['id']]=job;self.events_by_job[job['id']]=collections.deque(self.store.events(job['id']),maxlen=512)
   if job.get('brain_command'):self.commands[job['brain_command']['command_id']]=job['id']
   if job.get('idempotency_key'):self.keys[job['idempotency_key']]=job['id']
   if job['state']not in TERMINAL:
    self._stop_orphan(job)
    with self.condition:self._change(job,'INTERRUPTED',dict(result=dict(code='API_SERVICE_RESTARTED',arrived=False)),durable=True)
 def _stop_orphan(self,job):
  owner=job.get('process')or{};pid=owner.get('pid')
  if not pid:return
  try:
   proc=Path('/proc',str(pid));args=proc.joinpath('cmdline').read_bytes().split(b'\0');ticks=proc.joinpath('stat').read_text().rsplit(')',1)[1].split()[19]
   if ticks!=owner.get('pid_start_ticks')or job['id'].encode()not in args or str(self.root/'modules/onboard_navigation.py').encode()not in args:return
   os.kill(pid,signal.SIGINT)
  except (OSError,ValueError,KeyError):pass
 def _change(self,job,state,extra=None,event_type='state',durable=False):
  job.update(extra or{});now=time.time()
  if state=='STARTING'and job.get('started_at')is None:job['started_at']=now
  if state in TERMINAL and job.get('completed_at')is None:job['completed_at']=now
  job.update(state=state,updated_at=now,revision=job.get('revision',0)+1)
  event=dict(sequence=job['revision'],time=job['updated_at'],type=event_type,state=state)
  if extra:event['data']=copy.deepcopy(extra)
  self.events_by_job[job['id']].append(event);self.store.save(job,event,wait=durable)
  if state in TERMINAL and self.active==job['id']:self.active=None
  self.condition.notify_all()
 def submit(self,kind,payload,key=None):
  request=normalize(kind,payload,self.scene)
  return self._accept(kind,request,key)
 def _accept(self,kind,request,key=None,target=None,brain_command=None):
  digest=fingerprint(kind,dict(request=request,brain_command=brain_command)) if brain_command else fingerprint(kind,request)
  if key is not None and(not isinstance(key,str)or not 1<=len(key)<=128):raise ApiError(422,'INVALID_IDEMPOTENCY_KEY')
  with self.condition:
   if self.closed:raise ApiError(503,'API_SHUTTING_DOWN')
   if key in self.keys:
    job=self.jobs[self.keys[key]]
    if job['fingerprint']!=digest:raise ApiError(409,'IDEMPOTENCY_KEY_CONFLICT')
    return copy.deepcopy(job),False
   if kind=='navigate'and not(self.motion_enabled or self.simulation and brain_command):raise ApiError(409,'MOTION_DISABLED','API server is running in read-only mode')
   if self.active or self.busy():raise ApiError(409,'NAVIGATION_BUSY')
   target=target if target is not None else self.registry.get(request['scene'],request['goal']);ident=uuid.uuid4().hex
   job=dict(id=ident,kind=kind,request=request,target=target,fingerprint=digest,idempotency_key=key,state='ACCEPTED',created_at=time.time(),revision=0,result=None,navigator_status=None)
   if brain_command:job['brain_command']=copy.deepcopy(brain_command)
   self.jobs[ident]=job;self.events_by_job[ident]=collections.deque(maxlen=512);self.cancels[ident]=threading.Event()
   if key:self.keys[key]=ident
   if brain_command:self.commands[brain_command['command_id']]=ident
   try:self._change(job,'ACCEPTED',durable=True)
   except Exception as exc:
    # No process starts after uncertain journal acknowledgment. Retain identity
    # in memory so a retry/query cannot create a second command.
    job.update(state='FAILED',completed_at=time.time(),result=dict(code='TASK_JOURNAL_UNCONFIRMED',arrived=False,message=str(exc)))
    raise
   self.active=ident;thread=threading.Thread(target=self._run,args=(ident,),daemon=True,name='navigation_task_'+ident[:8]);self.threads[ident]=thread;thread.start()
   return copy.deepcopy(job),True
 def submit_command(self,body):
  from brain_navigation_contract import normalize as normalize_command,coordinate_target,config
  body=normalize_command(body)
  with self.condition:
   old=self.commands.get(body['command_id'])
   if old:
    job=self.jobs[old]
    if job['brain_command']!=body:raise ApiError(409,'COMMAND_ID_CONFLICT')
    return copy.deepcopy(job),False
   cfg=config(self.root)
   if body['motion_mode']not in cfg['supported_motion_modes']:raise ApiError(422,'INVALID_MOTION_MODE','Mode not supported')
   if not(self.motion_enabled or self.simulation):raise ApiError(409,'MOTION_DISABLED')
   target=coordinate_target(self.root,self.scene,body,simulation=self.simulation)
   request=dict(goal=0,scene=self.scene,translation_cap_mps=cfg['translation_cap_mps'],timeout_s=cfg['timeout_s'],client_id='brain-coordinate')
   key='brain:'+__import__('hashlib').sha256(body['command_id'].encode()).hexdigest()
   return self._accept('navigate',request,key,target=target,brain_command=body)
 def command(self,command_id):
  with self.condition:
   ident=self.commands.get(command_id)
   if not ident:raise ApiError(404,'COMMAND_NOT_FOUND')
   return self.get(ident)
 def _run(self,ident):
  try:
   with self.condition:
    job=self.jobs[ident];cancel=self.cancels[ident]
    if cancel.is_set():self._change(job,'CANCELED',dict(result=dict(code='CANCELED_BEFORE_START',arrived=False)),durable=True);return
    if job.get('brain_command'):
     from brain_navigation_contract import validate_bound_target
     if not validate_bound_target(self.root,job['target']):raise ApiError(409,'TARGET_CHANGED_BEFORE_START')
    elif self.registry.get(job['request']['scene'],job['request']['goal'])!=job['target']:raise ApiError(409,'TARGET_CHANGED_BEFORE_START')
    self._change(job,'STARTING')
   state,result=self.runner.run(copy.deepcopy(job),cancel,lambda kind,data:self._emit(ident,kind,data))
   with self.condition:self._change(job,state,dict(result=result),durable=True)
  except Exception as exc:
   with self.condition:
    job=self.jobs[ident]
    try:self._change(job,'FAILED',dict(result=dict(code=getattr(exc,'code','TASK_RUNTIME_ERROR'),message=str(exc),arrived=False)),durable=True)
    except Exception as io:job.update(state='FAILED',result=dict(code='TASK_JOURNAL_ERROR',message=str(io),arrived=False))
  finally:
   with self.condition:
    if self.active==ident:self.active=None
    self.condition.notify_all()
 def _emit(self,ident,kind,data):
  with self.condition:
   job=self.jobs[ident]
   if kind=='spawn':self._change(job,'PLANNING'if job['kind']=='plan'else job['state'],dict(process=data),event_type='process',durable=True);return
   if kind=='log':
    messages=job.setdefault('messages',[]);messages.append(dict(time=time.time(),**data));del messages[:-30]
    self._change(job,job['state'],dict(last_message=data),event_type='log');return
   phase=data.get('phase','');state='RUNNING'if phase in ['FOLLOWING','FINE_ADJUSTING','FINE_OBSERVING','GOAL_SETTLING','PREVIEW_READY']else'PLANNING'if phase in ['PREPARING','LOCALIZING_AND_PLANNING','REPLANNING_STOPPED']else'WAITING'
   if job['state']=='CANCELING':state='CANCELING'
   old=job.get('navigator_status')or{};changed=old.get('phase')!=phase or old.get('reason')!=data.get('reason')
   job['navigator_status']=data
   if changed or time.monotonic()-self.last_progress.get(ident,0)>1. or data.get('finished'):
    self.last_progress[ident]=time.monotonic();self._change(job,state,dict(navigator_status=data),event_type='navigator')
 def get(self,ident):
  with self.condition:
   if ident not in self.jobs:raise ApiError(404,'TASK_NOT_FOUND')
   return copy.deepcopy(self.jobs[ident])
 def list(self,limit=30):
  with self.condition:return [copy.deepcopy(j)for j in sorted(self.jobs.values(),key=lambda j:j['created_at'],reverse=True)[:limit]]
 def cancel(self,ident):
  with self.condition:
   if ident not in self.jobs:raise ApiError(404,'TASK_NOT_FOUND')
   job=self.jobs[ident]
   if job['state']not in TERMINAL:
    self.cancels[ident].set();self._change(job,'CANCELING',event_type='cancel_requested')
   return copy.deepcopy(job)
 def events(self,ident,after=0,wait=0.):
  with self.condition:
   self.get(ident)
   def ready():return self.jobs[ident]['revision']>after or self.jobs[ident]['state']in TERMINAL
   if not ready()and wait:self.condition.wait_for(ready,timeout=wait)
   items=[copy.deepcopy(x)for x in self.events_by_job[ident]if x['sequence']>after]
   oldest=self.events_by_job[ident][0]['sequence']if self.events_by_job[ident]else self.jobs[ident]['revision']
   return dict(events=items,next_sequence=self.jobs[ident]['revision'],oldest_sequence=oldest,truncated=after<oldest-1,state=self.jobs[ident]['state'])
 def plan(self,ident):
  job=self.get(ident);file=(job.get('result')or{}).get('artifacts',{}).get('plan_file')
  if not file:raise ApiError(409,'PLAN_ARTIFACT_NOT_READY')
  p=Path(file).resolve()
  if (self.root/'results').resolve()not in p.parents or p.name!='plan.json':raise ApiError(409,'INVALID_PLAN_ARTIFACT')
  try:return json.loads(p.read_text())
  except (OSError,ValueError)as exc:raise ApiError(503,'PLAN_ARTIFACT_UNAVAILABLE',str(exc))
 def close(self):
  with self.condition:
   self.closed=True
   if self.active:self.cancels[self.active].set()
  for thread in list(self.threads.values()):thread.join(timeout=20)
  self.store.close()
