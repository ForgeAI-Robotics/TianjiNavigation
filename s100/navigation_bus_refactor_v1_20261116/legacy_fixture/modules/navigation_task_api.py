"""Authenticated HTTP facade for the S100 navigation task broker."""
import argparse,json,os,secrets,signal,threading,time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit,parse_qs
from common import ROOT,exclusive
from navigation_task_runtime import TaskManager
from navigation_task_contract import ApiError

VERSION='1.1.0'
def token_file(path):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 if not path.exists():
  try:
   fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
   with os.fdopen(fd,'w')as f:f.write(secrets.token_urlsafe(32)+'\n')
  except FileExistsError:pass
 value=path.read_text().strip()
 if len(value)<24:raise ValueError('API_TOKEN_TOO_SHORT')
 os.chmod(path,0o600);return value

def make_server(manager,token,host='127.0.0.1',port=8765,readiness=None):
 def read_state():
  if readiness is not None:return readiness()
  from common import CONFIG
  from navigation_readiness import received_readiness
  import targets
  return received_readiness(json.loads(Path(CONFIG['joint_status']).read_text()),targets.get(1)['map_voxel_sha256'])
 from brain_navigation_api import BrainCompatibility
 compat=BrainCompatibility(manager,read_state)
 class Handler(BaseHTTPRequestHandler):
  server_version='TianjiNavigationAPI/'+VERSION
  protocol_version='HTTP/1.1'
  def setup(self):
   super().setup();self.connection.settimeout(15.)
  def log_message(self,*args):pass # Never log Authorization headers.
  def reply(self,status,data):
   blob=json.dumps(data,ensure_ascii=False,allow_nan=False).encode();self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Content-Length',str(len(blob)));self.send_header('Cache-Control','no-store');self.end_headers()
   try:self.wfile.write(blob)
   except (BrokenPipeError,ConnectionResetError):pass # Task life does not depend on its HTTP client.
  def body(self):
   try:size=int(self.headers.get('Content-Length','0'))
   except ValueError:raise ApiError(400,'INVALID_CONTENT_LENGTH')
   if not 0<=size<=32768:raise ApiError(413,'REQUEST_TOO_LARGE')
   if self.headers.get('Transfer-Encoding'):raise ApiError(400,'CHUNKED_BODY_UNSUPPORTED')
   if size==0:return {}
   try:return json.loads(self.rfile.read(size).decode('utf-8'),parse_constant=lambda x:(_ for _ in()).throw(ValueError(x)))
   except (UnicodeError,ValueError):raise ApiError(400,'INVALID_JSON')
  def route(self):
   parsed=urlsplit(self.path);path=parsed.path.rstrip('/');query=parse_qs(parsed.query)
   if self.command=='GET'and path=='/healthz':return 200,dict(status='ok',version=VERSION,motion_enabled=manager.motion_enabled)
   auth=self.headers.get('Authorization','')
   if not secrets.compare_digest(auth.encode('utf-8'),('Bearer '+token).encode('utf-8')):raise ApiError(401,'UNAUTHORIZED')
   self.compat_path=path in ['/health','/v1/status','/v1/world','/total_scene_graph_latest.json','/v1/navigation/goals','/v1/navigation/cancel']or path.startswith('/v1/commands/')
   if self.compat_path:
    self.compat_body=self.body()if self.command=='POST'else None
    result=compat.route(self.command,path,self.compat_body)
    if self.command=='POST'and path=='/v1/navigation/goals'and manager.simulation and self.compat_body.get('target_id')=='contract_post_lost':
     import socket
     self.connection.shutdown(socket.SHUT_RDWR);self.connection.close();self.close_connection=True
     return None
    return result
   if self.command=='GET'and path=='/v1/capabilities':return 200,dict(api_version=VERSION,compute_host='S100',default_scene=manager.scene,motion_enabled=manager.motion_enabled,task_ids=[1,2,3],states=['ACCEPTED','STARTING','PLANNING','WAITING','RUNNING','CANCELING','SUCCEEDED','FAILED','CANCELED','INTERRUPTED','PLAN_READY','TEST_COMPLETE'],navigation_owner='onboard_navigation',gui_role='debug client',velocity_control_endpoint=False,raw_sensor_endpoint=False)
   if self.command=='GET'and path=='/v1/readiness':
    try:return 200,dict(scene=manager.scene,motion_enabled=manager.motion_enabled,readiness=read_state())
    except (OSError,ValueError,KeyError)as exc:raise ApiError(503,'READINESS_UNAVAILABLE',str(exc))
   if self.command=='GET'and path=='/v1/waypoints':return 200,dict(scene=query.get('scene',[manager.scene])[0],waypoints=manager.registry.list(query.get('scene',[manager.scene])[0]))
   if self.command=='GET'and path=='/v1/navigation/tasks':return 200,dict(tasks=manager.list())
   if self.command=='POST'and path in ['/v1/navigation/tasks','/v1/navigation/plans','/v1/navigation/tests']:
    kind={'/v1/navigation/tasks':'navigate','/v1/navigation/plans':'plan','/v1/navigation/tests':'test'}[path]
    job,new=manager.submit(kind,self.body(),self.headers.get('Idempotency-Key'));return (202 if new else 200),job
   if self.command=='POST'and path=='/v1/navigation/stop':
    self.body()
    with manager.condition:ident=manager.active
    return 200,manager.cancel(ident)if ident else dict(state='IDLE',active_task=None)
   parts=path.split('/')
   if len(parts)in (5,6)and parts[1:4]==['v1','navigation','tasks']:
    ident=parts[4]
    if len(ident)!=32 or any(c not in'0123456789abcdef'for c in ident):raise ApiError(404,'TASK_NOT_FOUND')
    action=parts[5]if len(parts)==6 else''
    if self.command=='GET'and not action:return 200,manager.get(ident)
    if self.command=='POST'and action=='cancel':self.body();return 200,manager.cancel(ident)
    if self.command=='GET'and action=='plan':return 200,manager.plan(ident)
    if self.command=='GET'and action=='events':
     try:after=int(query.get('after',['0'])[0]);wait=float(query.get('wait',['0'])[0])
     except ValueError:raise ApiError(422,'INVALID_EVENT_CURSOR')
     if not after>=0 or not 0<=wait<=10:raise ApiError(422,'INVALID_EVENT_CURSOR')
     return 200,manager.events(ident,after,wait)
   if self.command=='GET'and path=='/v1/openapi.json':return 200,json.loads((manager.root/'config/task_api_openapi.json').read_text())
   raise ApiError(404,'ENDPOINT_NOT_FOUND')
  def dispatch(self):
   self.compat_path=False;self.compat_body=None
   try:
    result=self.route()
    if result is None:return
    status,data=result
   except ApiError as exc:
    if self.compat_path:status,data=compat.error(exc,urlsplit(self.path).path,self.compat_body)
    else:status=exc.status;data=dict(error=dict(code=exc.code,message=str(exc)))
   except Exception as exc:status=500;data=dict(error=dict(code='INTERNAL_ERROR',message=str(exc)))
   # Mutations and failures are durable API evidence, without body/auth secrets.
   if self.command=='POST'or status>=400:
    try:
     file=manager.root/'runtime/task_api/requests.jsonl'
     with server.audit_lock,file.open('a')as f:f.write(json.dumps(dict(time=time.time(),method=self.command,path=urlsplit(self.path).path,status=status,task_id=data.get('id'),error=data.get('error')),ensure_ascii=False)+'\n')
    except OSError:pass
   self.reply(status,data)
  do_GET=dispatch
  do_POST=dispatch
 class Server(ThreadingHTTPServer):daemon_threads=True;allow_reuse_address=True
 server=Server((host,port),Handler);server.audit_lock=threading.Lock();return server

def main():
 p=argparse.ArgumentParser();p.add_argument('--host');p.add_argument('--port',type=int);p.add_argument('--scene',choices=['office','outside']);p.add_argument('--contract-test',action='store_true');p.add_argument('--enable-motion',action='store_true');p.add_argument('--operator-estop-ready',action='store_true');args=p.parse_args()
 config=json.loads((ROOT/'config/task_api.json').read_text());scene=args.scene or config['default_scene']
 os.environ['TIANJI_NAV_SCENE']=scene
 if args.enable_motion and not args.operator_estop_ready:raise ValueError('EXISTING_OPERATOR_ESTOP_DECLARATION_REQUIRED')
 if args.enable_motion and (ROOT/'ISOLATED_CONTRACT_TEST_ROOT').exists():raise ValueError('ISOLATED_ROOT_CANNOT_ENABLE_PHYSICAL_MOTION')
 if args.contract_test and args.host not in (None,'127.0.0.1','localhost'):raise ValueError('CONTRACT_TEST_LOOPBACK_ONLY')
 if args.contract_test and (args.enable_motion or args.operator_estop_ready):raise ValueError('CONTRACT_TEST_CANNOT_ENABLE_MOTION')
 runner=None
 if args.contract_test:
  if not (ROOT/'ISOLATED_CONTRACT_TEST_ROOT').exists():raise ValueError('SEPARATE_CONTRACT_TEST_ROOT_REQUIRED')
  from brain_contract_fixture import ContractTestRunner
  runner=ContractTestRunner()
 lock=exclusive(ROOT/'runtime/navigation_task_api.lock');manager=TaskManager(ROOT,args.enable_motion,args.operator_estop_ready,scene,runner=runner)
 server=make_server(manager,token_file(ROOT/config['token_file']),args.host or config['host'],args.port if args.port is not None else config['port'],readiness=(lambda:dict(arm_fresh=False,head_fresh=False,receiver_status_age_s=None))if args.contract_test else None)
 def halt(*_):threading.Thread(target=server.shutdown,daemon=True).start()
 for sig in [signal.SIGTERM,signal.SIGINT,signal.SIGHUP]:signal.signal(sig,halt)
 print(json.dumps(dict(service='S100_NAVIGATION_TASK_API',version=VERSION,host=server.server_address[0],port=server.server_address[1],motion_enabled=manager.motion_enabled,motion_commands_sent=0)),flush=True)
 try:server.serve_forever(poll_interval=.2)
 finally:server.server_close();manager.close();lock.close()

if __name__=='__main__':main()
