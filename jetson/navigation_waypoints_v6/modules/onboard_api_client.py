"""GUI debug adapter to the S100 task API over an existing SSH connection.

This client never owns navigation heartbeats or reconstructs collision checks.
"""
import http.client,json,os,signal,sys,threading,time
from pathlib import Path
from paths import ROOT
sys.path.insert(0,str(ROOT.parent/'navigation_s100_v7/sdk'))
from tianji_navigation import NavigationClient,TERMINAL

class Session:
 def __enter__(self):
  from onboard_client import connect,REMOTE
  from task_backend import load
  self.backend=load()['backend'];self.ssh=connect()
  try:
   if self.backend=='bus':
    i,o,e=self.ssh.exec_command('/home/sunrise/tianji_table_approach_dev/navigation_bus_refactor_v1_20261116/start_gui_services.sh',timeout=15)
    i.close();o.read();details=e.read().decode();code=o.channel.recv_exit_status();o.close();e.close()
    if code:raise RuntimeError('模块化GUI服务启动失败：'+details)
   with self.ssh.open_sftp()as ftp:
    token_path='/home/sunrise/tianji_table_approach_dev/navigation_bus_refactor_v1_20261116/runtime/gui_bus_token' if self.backend=='bus' else REMOTE+'/config/task_api_token'
    with ftp.open(token_path)as f:token=f.read().decode().strip()
   ssh=self.ssh;port=18769 if self.backend=='bus' else 8765
   class Connection(http.client.HTTPConnection):
    def connect(self):
     self.sock=ssh.get_transport().open_channel('direct-tcpip',('127.0.0.1',port),('127.0.0.1',0),timeout=8.);self.sock.settimeout(15.)
   factory=lambda:Connection('127.0.0.1',port,timeout=15.)
   if self.backend=='bus':
    from bus_gui_adapter import Client
    self.client=Client(token,factory)
   else:self.client=NavigationClient('http://127.0.0.1:8765',token,connection_factory=factory)
   return self
  except Exception:self.ssh.close();raise
 def __exit__(self,*_):self.ssh.close()

def plan_goals(goals):
 import targets,preview_options
 from common import atomic_json
 from onboard_client import fetch_latest
 options=preview_options.load();last=None
 with Session()as session:
  for goal in goals:
   job=session.client.plan(int(goal),scene=targets.scene_id(),options=options,client_id='gui-debug')
   print(('S100 模块化总线规划任务：'if session.backend=='bus'else'S100 API只读规划任务：')+job['id'],flush=True)
   done=session.client.wait(job['id'],on_update=lambda j:print('API_TASK '+j['id']+' '+j['state'],flush=True))
   atomic_json(ROOT/'runtime/last_api_task.json',done)
   if done['state']not in ['PLAN_READY','FAILED']:raise RuntimeError('API_PLAN_INTERRUPTED: '+done['state'])
   folder=(done.get('result')or{}).get('artifacts',{}).get('folder')
   if folder:last=fetch_latest(session.ssh,folder)
   if done['state']!='PLAN_READY':raise RuntimeError('API规划未通过：'+str(done['result']))
 return last

def execute(goal,real,cancellation=None,preview_duration=20.,execution_seconds=None,translation_cap_mps=None):
 import targets
 from common import atomic_json
 from onboard_client import fetch_latest,fetch_motor_trace,fetch_collision
 cancel=cancellation or threading.Event();previous={}
 def interrupted(*_):cancel.set()
 for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:previous[sig]=signal.signal(sig,interrupted)
 try:
  with Session()as session:
   opts=dict(client_id='gui-debug')
   if translation_cap_mps is not None:opts['translation_cap_mps']=translation_cap_mps
   if execution_seconds is not None:opts['timeout_s']=execution_seconds
   job=session.client.navigate(goal,scene=targets.scene_id(),**opts)if real else session.client.test(goal,scene=targets.scene_id(),duration_s=preview_duration,**opts)
   ident=job['id'];atomic_json(ROOT/'runtime/operator_session.json',dict(session=ident,task_id=ident,goal=goal,real=real,client_pid=os.getpid(),task_owner='S100_TASK_API',task_backend=session.backend,created_wall_time=time.time()))
   print(('S100 模块化总线任务：'if session.backend=='bus'else'S100 API任务：')+ident+'；执行与心跳由S100持有。',flush=True);seen=None;requested=False
   while True:
    if cancel.is_set()and not requested:session.client.cancel(ident);requested=True
    job=session.client.get(ident);raw=job.get('navigator_status')
    if job['revision']!=seen:
     print('API_TASK '+ident+' '+job['state'],flush=True);seen=job['revision']
     if raw:atomic_json(ROOT/'runtime/onboard_execution_status.json',raw);print('STATUS '+json.dumps(raw,ensure_ascii=False),flush=True)
    if job['state']in TERMINAL:break
    session.client.events(ident,after=seen or 0,wait=.5)
   raw=job.get('navigator_status')or dict(session=ident,mode='API_TASK',phase='STOP_API_TASK',reason=(job.get('result')or{}).get('code'),finished=True,active=False,updated_wall_time=time.time())
   atomic_json(ROOT/'runtime/onboard_execution_status.json',raw)
   result=job.get('result')or{};folder=result.get('artifacts',{}).get('folder')
   if folder:
    try:fetch_latest(session.ssh,folder)
    except Exception as exc:print('API路线展示更新失败：'+str(exc),flush=True)
   if raw.get('motor_trace_file'):
    try:fetch_motor_trace(session.ssh,raw)
    except Exception as exc:print('API执行日志下载失败：'+str(exc),flush=True)
   if raw.get('collision_report'):
    try:fetch_collision(session.ssh,raw);atomic_json(ROOT/'runtime/onboard_execution_status.json',raw)
    except Exception as exc:print('API碰撞快照下载失败：'+str(exc),flush=True)
   atomic_json(ROOT/'runtime/last_api_task.json',job)
   if job['state']not in ['SUCCEEDED','TEST_COMPLETE','CANCELED']:raise RuntimeError('S100 API任务结束：'+json.dumps(result,ensure_ascii=False))
   return job
 finally:
  for sig,handler in previous.items():signal.signal(sig,handler)

def stop():
 with Session()as session:return session.client.stop()
