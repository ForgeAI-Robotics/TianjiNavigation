"""Jetson command/display client only. All navigation decisions execute on S100."""
import argparse,json,shlex,sys,time
from pathlib import Path
import paramiko
from paths import ROOT,NAV
from common import atomic_json
REMOTE='/home/sunrise/tianji_table_approach_dev/navigation_s100_v7'
import targets
def remote_directory():
 return 'cd '+shlex.quote(REMOTE)+'\nexport TIANJI_NAV_SCENE='+shlex.quote(targets.scene_id())+'\n'

def connect():
 c=json.loads((NAV/'config/connection.json').read_text());s=paramiko.SSHClient();s.load_host_keys(str(NAV/'config/known_hosts'));s.set_missing_host_key_policy(paramiko.RejectPolicy());s.connect(c['host'],username=c['username'],password=c['password'],timeout=8);return s

def fetch_collision(connection,record):
 remote=Path(record['collision_report'])
 if not str(remote).startswith(REMOTE+'/results/') or '..' in remote.parts or remote.name!='live_collision.json' or remote.parent.name!='scene':raise ValueError('INVALID_COLLISION_REPORT_PATH')
 folder=ROOT/'results'/('s100_'+remote.parent.parent.name)/'scene';folder.mkdir(parents=True,exist_ok=True)
 ftp=connection.open_sftp()
 try:
  ftp.get(str(remote),str(folder/'live_collision.json'));data=json.loads((folder/'live_collision.json').read_text())
  if data.get('session')!=record['session']:raise ValueError('COLLISION_REPORT_SESSION_MISMATCH')
  ftp.get(str(remote.with_suffix('.npz')),str(folder/'live_collision.npz'))
 finally:ftp.close()
 record['collision_summary']={k:data.get(k) for k in ['hit_count','examples','source']}
 atomic_json(ROOT/'runtime/latest_collision.json',dict(folder=str(folder),session=record['session']))

def fetch_motor_trace(connection,record):
 remote=Path(record['motor_trace_file'])
 if not str(remote).startswith(REMOTE+'/results/') or '..' in remote.parts or remote.name!='motor_trace.jsonl':raise ValueError('INVALID_MOTOR_TRACE_PATH')
 folder=ROOT/'results'/('s100_'+remote.parent.name);folder.mkdir(parents=True,exist_ok=True);ftp=connection.open_sftp()
 try:
  for name in ['navigation_execution_history.jsonl','navigation_performance_summary.json','motor_trace.json','motor_trace.jsonl','sensor_relay_metrics.json','sensor_relay_metrics.jsonl']:
   try:ftp.get(str(remote.parent/name),str(folder/name))
   except FileNotFoundError:continue
 finally:ftp.close()
 print('执行日志：'+str(folder),flush=True)
 if (folder/'motor_trace.jsonl').exists():print('轮组目标/反馈记录：'+str(folder/'motor_trace.jsonl'),flush=True)

def fetch_latest(connection=None,remote_folder=None):
 s=connection or connect()
 try:
  ftp=s.open_sftp()
  if remote_folder is None:
   with ftp.open(REMOTE+'/runtime/latest_plan.json') as f:pointer=json.load(f)
   remote_folder=pointer['folder']
  if not remote_folder.startswith(REMOTE+'/results/') or '..'in Path(remote_folder).parts:raise ValueError('UNEXPECTED_REMOTE_RESULT')
  out=ROOT/'results'/('s100_'+Path(remote_folder).name);out.mkdir(exist_ok=True)
  for name in ['planning/plan.json','scene/frozen_input.json','scene/immutable_world.json','scene/display_clouds.npz','localization/localization.json']:
   dest=out/name;dest.parent.mkdir(parents=True,exist_ok=True);ftp.get(remote_folder+'/'+name,str(dest))
  try:ftp.get(remote_folder+'/scene/near_body_cleanup.json',str(out/'scene/near_body_cleanup.json'))
  except FileNotFoundError:pass
  ftp.close();result=json.loads((out/'planning/plan.json').read_text());expected=targets.get(1 if result['target']['id']==0 else result['target']['id'])
  if result['target']['map_voxel_sha256']!=expected['map_voxel_sha256']:raise ValueError('RESULT_BELONGS_TO_OTHER_SCENE')
  atomic_json(out/'execution_owner.json',dict(navigation_compute_host='S100',remote_folder=remote_folder,jetson_role='command/rendering and optional point-filter service; actual filter placement recorded per frame',motion_commands_sent=0))
  # Renderer only consumes S100's already-computed path; it does not plan.
  from render_plan import render
  render(out);pointer=dict(folder=str(out),result=str(out/'planning/plan.json'),compute_host='S100',remote_folder=remote_folder,scene_id=targets.scene_id());atomic_json(ROOT/'runtime/latest_plan.json',pointer);atomic_json(ROOT/'runtime'/f"latest_plan_{targets.scene_id()}_{result['target']['id']}.json",pointer);print('GUI_RESULT '+str(out),flush=True)
  return out
 finally:
  if connection is None:s.close()

def plan_goals(goals):
 from task_backend import enabled
 if enabled():
  from onboard_api_client import plan_goals as api_plan
  return api_plan(goals)
 from preview_options import load,caption
 options=load();s=connect();mode='--both'if goals==[1,2]else'--goal '+str(int(goals[0]));cmd=remote_directory()+'exec ./preview_plan.sh '+mode+' --ignore-head-camera '+str(int(options['ignore_head_camera']))+' --open-grippers '+str(int(options['open_grippers']))+' --use-saved-arms '+str(int(options['use_saved_arms']))
 _,stdout,stderr=s.exec_command('bash -c '+shlex.quote(cmd));print('S100只读规划；'+caption(options),flush=True);out=None
 try:
  for line in stdout:
   print(line,end='',flush=True)
   if line.startswith('PREVIEW_RESULT '):
    result=json.loads(line[len('PREVIEW_RESULT '):]);out=fetch_latest(s,result['folder'])
  err=stderr.read().decode();code=stdout.channel.recv_exit_status()
  if err:print(err,file=sys.stderr,flush=True)
  if code:raise RuntimeError('S100规划未完成：'+str(code))
  if out is None:raise RuntimeError('S100未返回规划结果')
  return out
 finally:stdout.channel.close();s.close()

def plan(goal,cable=False):
 if cable:print('5cm检测已取消；采用正常规划。',flush=True)
 return plan_goals([int(goal)])

def check():
 s=connect()
 script="""import sys,json,time
from pathlib import Path
sys.path.insert(0,'modules')
import paths
from onboard_state import readiness
from manual_floor import settings
d=readiness();d['manual_floor']=settings();d['checked_wall_time']=time.time()
p=Path('config/filter_compute.json')
if p.exists():
 cfg=json.loads(p.read_text());d['point_filter_compute']={k:cfg.get(k) for k in ['mode','backend','package_id']}

p=Path('config/near_body_cleanup.json')
d['near_body_cleanup']=json.loads(p.read_text()) if p.exists() else {'enabled':False}
pointer=Path('runtime/latest_plan.json')
if pointer.exists():
 folder=Path(json.loads(pointer.read_text())['folder']);report=folder/'scene/near_body_cleanup.json'
 if report.exists():d['cleanup_last_plan']=dict(folder=str(folder),**json.loads(report.read_text()))
print(json.dumps(d,ensure_ascii=False))
"""
 cmd=remote_directory()+'python3 -c '+shlex.quote(script)
 try:
  _,o,e=s.exec_command('bash -c '+shlex.quote(cmd));data=o.read().decode();err=e.read().decode();code=o.channel.recv_exit_status();o.channel.close()
  if code:raise RuntimeError(err)
  result=json.loads(data);result.update(navigation_compute_host='S100',jetson_role='GUI, measured-state publisher and point-filter service',execution_permission=False);atomic_json(ROOT/'runtime/onboard_readiness.json',result)
  if 'manual_floor' in result:atomic_json(ROOT/'runtime/manual_floor.json',result['manual_floor'])
  print(json.dumps(result,ensure_ascii=False,indent=2));return result
 finally:s.close()
def stop_execution():
 from task_backend import load as backend_settings
 if backend_settings()['backend']=='bus':
  s=connect()
  try:
   command='/usr/bin/python3 /home/sunrise/tianji_table_approach_dev/navigation_bus_refactor_v1_20261116/stop_gui_candidate.py'
   i,o,e=s.exec_command(command,timeout=10);i.close();print('模块化执行器SSH停车后备：'+o.read().decode(),flush=True);error=e.read().decode();o.close();e.close()
   if error:print(error,file=sys.stderr,flush=True)
  except Exception as exc:print('SSH停车后备失败：'+str(exc),flush=True)
  finally:s.close()
 try:
  from onboard_api_client import stop as api_stop
  print('API停车请求：'+json.dumps(api_stop(),ensure_ascii=False),flush=True)
 except Exception as exc:print('API停车状态：'+type(exc).__name__+'；继续发送原CLI停车请求。',flush=True)
 s=connect()
 try:
  cmd=remote_directory()+'python3 modules/stop_navigation.py';_,o,e=s.exec_command('bash -c '+shlex.quote(cmd));print(o.read().decode(),flush=True);o.channel.close()
 finally:s.close()

def head_feedback_reader(duration=3600):
 """Supply measured head angles without launching the arm GUI or enabling it."""
 import subprocess,os,ctypes,signal
 feedback=Path('/home/nvidia/tianji_upper_client/tianji_m6_teleop_delivery_20260902/logs/shapemask_head_state.json')
 def fresh():
  try:
   d=json.loads(feedback.read_text());return d.get('connected') and 0<=time.time()-d['received_at']<=.3
  except (OSError,ValueError,KeyError):return False
 if fresh():return None
 try:
  pid=int((ROOT/'runtime/navigation_head_state_service.lock').read_text().strip());args=Path('/proc',str(pid),'cmdline').read_bytes()
  independent=str(ROOT/'modules/navigation_head_state_service.py').encode()in args
 except (OSError,ValueError):independent=False
 if independent:
  for _ in range(50):
   if fresh():return None
   time.sleep(.1)
  raise RuntimeError('独立头部只读服务尚未取得新反馈，请检查串口占用；不启动另一份串口读取进程。')
 logfile=(ROOT/'runtime/readonly_head_feedback.log').open('ab')
 process=subprocess.Popen(['/home/nvidia/tianji_upper_client/tianji_m6_teleop_delivery_20260902/.venv/bin/python',str(ROOT/'modules/readonly_head_feedback.py'),'--duration',str(duration)],stdout=logfile,stderr=logfile,preexec_fn=lambda:ctypes.CDLL(None).prctl(1,signal.SIGTERM));logfile.close()
 for _ in range(40):
  if process.poll() is not None:raise RuntimeError('只读头部反馈未启动；串口可能已被GUI占用，详情见readonly_head_feedback.log')
  if fresh():print('头部姿态：持续只读角度查询，不发送运动指令。',flush=True);return process
  time.sleep(.1)
 process.terminate();process.wait(timeout=3);raise RuntimeError('头部实测反馈未就绪')

def execute(goal,real,micro=False,cable=False,continue_route=True,execution_seconds=None,translation_cap_mps=None):
 if micro:raise RuntimeError("微移检测已取消，旧方案保存在快照中。")
 if cable:print("5cm检测已取消；直接重新定位、规划并执行正常导航。",flush=True)
 cable=False;continue_route=False
 head_reader=head_feedback_reader()
 try:return execute_session(goal,real,micro,cable,continue_route,execution_seconds=execution_seconds,translation_cap_mps=translation_cap_mps)
 finally:
  if head_reader and head_reader.poll() is None:head_reader.terminate();head_reader.wait(timeout=3)

def cable_continuation_allowed(record):
 return bool(record and record.get('phase')=='CABLE_PRECHECK_COMPLETE' and record.get('measured_displacement_m',0)>=.05 and record.get('fixed_environment_features',0)==0 and (record.get('confirmed_cable_features',0)>0 or record.get('initial_candidate_count')==0))

def execute_session(goal,real,micro=False,cable=False,continue_route=True,cancellation=None,preview_duration=20.,execution_seconds=None,right_micro_preview=False,translation_cap_mps=None,load_test_no_motion=False):
 from task_backend import enabled
 if enabled():
  if micro or cable or right_micro_preview:raise ValueError('MICRO_PROBE_DISABLED_ON_TASK_API')
  if load_test_no_motion and real:raise ValueError('LOAD_TEST_NO_MOTION_ONLY')
  from onboard_api_client import execute as api_execute
  return api_execute(goal,real,cancellation,preview_duration,execution_seconds,translation_cap_mps)
 import threading,signal,secrets,ctypes,os
 if load_test_no_motion and (real or micro or cable or right_micro_preview):raise ValueError('LOAD_TEST_NO_MOTION_ONLY')
 if right_micro_preview and (real or micro or cable):raise ValueError("RIGHT_MICRO_PREVIEW_ONLY")
 if translation_cap_mps is not None and (micro or cable or right_micro_preview or not .005<=translation_cap_mps<=.10):
  raise ValueError('INVALID_TRANSLATION_SPEED_CAP')
 cancellation=threading.Event() if cancellation is None else cancellation
 if cancellation.is_set():return
 # The local display client dies with its GUI; it does not decide navigation.
 ctypes.CDLL(None).prctl(1,signal.SIGTERM)
 s=connect()
 if not real and not 5<=float(preview_duration)<=120:raise ValueError('PREVIEW_DURATION_MUST_BE_5_TO_120_SECONDS')
 session=secrets.token_hex(16);halt=threading.Event();mode='--enable-motion'+(' --duration 45' if micro or cable else '') if real else '--preview --duration '+('30' if micro or cable else str(float(preview_duration)))
 if execution_seconds is not None:
  if not real or micro or cable or not 5<=float(execution_seconds)<=180:raise ValueError('INVALID_SINGLE_TEST_DURATION')
  mode+=' --duration '+str(float(execution_seconds))
 if cable:mode+=' --cable-precheck'
 if translation_cap_mps is not None:mode+=f' --translation-cap-mps {translation_cap_mps:.6f}'
 if right_micro_preview:mode+=' --right-micro-preview'
 if load_test_no_motion:mode+=' --load-test-no-motion'
 cmd=remote_directory()+'exec '+('./micro_execute.sh' if micro else './execute.sh')+' --goal '+str(int(goal))+' --session '+session+' --operator-estop-ready '+mode
 if cancellation.is_set():s.close();return
 stdin,stdout,stderr=s.exec_command('bash -c '+shlex.quote(cmd));atomic_json(ROOT/'runtime/operator_session.json',dict(session=session,goal=goal,real=bool(real),bounded_micro=bool(micro or cable),maximum_travel_m=.06 if cable else .05 if micro else None,client_pid=os.getpid(),created_wall_time=time.time()))
 def heartbeat():
  try:
   while not halt.is_set() and not cancellation.is_set():stdin.write(json.dumps(dict(kind='heartbeat',session=session))+'\n');stdin.flush();halt.wait(.1)
  except (OSError,EOFError):halt.set()
 cancelled=[False]
 def interrupted(*_):
  cancelled[0]=True
  cancellation.set()
  halt.set()
  try:stdout.channel.shutdown_write()
  except OSError:pass
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
 worker=threading.Thread(target=heartbeat,daemon=True);worker.start();fetched=False;completed=None;print('S100执行会话：'+('先微移约5cm识别线缆，再复检剩余轨迹' if cable and real else '用户授权的轨迹微动，上限5cm' if real and micro else '实机，用户手动启动' if real else 'ROS完整链路负载测试，输出隔离，不驱动底盘' if load_test_no_motion else '零ROS输出预览验证'),flush=True)
 try:
  for line in stdout:
   print(line,end='',flush=True)
   if line.startswith('STATUS '):
    try:
     record=json.loads(line[7:]);atomic_json(ROOT/'runtime/onboard_execution_status.json',record)
     if cable and record.get('micro_maximum_m')!=.06:raise RuntimeError('S100未确认5cm检测与6cm保护上限')
     if record.get('finished'):
      if record.get('motor_trace_file'):
       try:fetch_motor_trace(s,record)
       except Exception as exc:print('轮组记录下载失败：'+str(exc),flush=True)
      if record.get('collision_report'):
       try:fetch_collision(s,record);atomic_json(ROOT/'runtime/onboard_execution_status.json',record)
       except Exception as exc:print('碰撞快照下载失败：'+str(exc),flush=True)
      completed=record
     if micro and record.get('micro_maximum_m')!=.05:raise RuntimeError('S100未确认5cm硬上限，停止操作会话')
     if not fetched and record.get('phase') in ['WAITING_GUARDS','FOLLOWING','PLAN_BLOCKED','ARRIVED']:
      try:fetch_latest(s);fetched=True
      except Exception as exc:print('路径图更新失败：'+str(exc),flush=True)
    except ValueError:pass
  err=stderr.read().decode();code=stdout.channel.recv_exit_status()
  atomic_json(ROOT/'runtime/last_execution_exit.json',dict(session=session,exit_code=code,stderr=err,finished_record_received=completed is not None,received_wall_time=time.time()))
  if err:print(err,file=sys.stderr,flush=True)
  if completed is None:
   previous={}
   try:previous=json.loads((ROOT/'runtime/onboard_execution_status.json').read_text())
   except (OSError,ValueError):pass
   if previous.get('session')==session:
    previous.update(active=False,previous_phase=previous.get('phase'),phase='STOP_EXECUTION_EXIT',reason='EXECUTION_PROCESS_EXITED_WITHOUT_FINAL_STATUS',exit_code=code,updated_wall_time=time.time())
    atomic_json(ROOT/'runtime/onboard_execution_status.json',previous)
  if code:raise RuntimeError('S100 execution failed '+str(code))
  if completed is None:raise RuntimeError('S100 execution ended without final status')
 finally:
  halt.set();worker.join(timeout=1)
  try:stdout.channel.shutdown_write();stdout.channel.close()
  except OSError:pass
  s.close()
 if cable and real and continue_route and not cancelled[0] and not cancellation.is_set() and cable_continuation_allowed(completed):
  print('线缆运动检测完成；S100重新定位、复检剩余路线后继续。',flush=True)
  execute_session(goal,True,cancellation=cancellation)

def manual_floor(action,bounds=None):
 s=connect();cmd=remote_directory()+'python3 modules/manual_floor.py '+shlex.quote(action)
 if bounds is not None:cmd+=' --bounds '+' '.join(shlex.quote(str(float(x))) for x in bounds)
 try:
  _,o,e=s.exec_command('bash -ce '+shlex.quote(cmd));out=o.read().decode();err=e.read().decode();code=o.channel.recv_exit_status();o.channel.close()
  if code:raise RuntimeError(err)
  cfg=json.loads(out);atomic_json(ROOT/'runtime/manual_floor.json',cfg);print(json.dumps(cfg,ensure_ascii=False),flush=True);return cfg
 finally:s.close()

def force_release(state):
 if state not in ['on','off']:raise ValueError('on/off required')
 s=connect();cmd=remote_directory()+'python3 modules/set_clearance_override.py '+state
 try:
  _,o,e=s.exec_command('bash -c '+shlex.quote(cmd));out=o.read().decode();err=e.read().decode();code=o.channel.recv_exit_status();o.channel.close()
  if code:raise RuntimeError(err)
  print(out,flush=True);return json.loads(out)
 finally:s.close()

def pipeline():
 s=connect();cmd=remote_directory()+'exec ./pipeline.sh --duration 15'
 try:
  _,o,e=s.exec_command('bash -c '+shlex.quote(cmd));out=o.read().decode();err=e.read().decode();code=o.channel.recv_exit_status();o.channel.close()
  if out:print(out,flush=True)
  if code:raise RuntimeError(err)
  result=json.loads(out);atomic_json(ROOT/'runtime/onboard_pipeline_report.json',result);return result
 finally:s.close()

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--scene',choices=list(targets.SCENES),default=targets.scene_id());p.add_argument('--plan-both',action='store_true');p.add_argument('--goal',type=int,choices=[1,2,3]);mode=p.add_mutually_exclusive_group();mode.add_argument('--execute',action='store_true');mode.add_argument('--execute-preview',action='store_true');mode.add_argument('--load-test-no-motion',action='store_true');mode.add_argument('--execute-micro',action='store_true');mode.add_argument('--preview-micro',action='store_true');mode.add_argument('--preview-right-micro',action='store_true');mode.add_argument('--execute-cable',action='store_true');mode.add_argument('--preview-cable',action='store_true');mode.add_argument('--plan-cable',action='store_true');mode.add_argument('--manual-floor',choices=['on','off','status']);mode.add_argument('--stop',action='store_true');mode.add_argument('--fetch-latest',action='store_true');mode.add_argument('--check',action='store_true');mode.add_argument('--pipeline',action='store_true');mode.add_argument('--force-release',choices=['on','off']);p.add_argument('--floor-bounds',nargs=4,type=float);p.add_argument('--assume-clearance',action='store_true');p.add_argument('--execution-seconds',type=float);p.add_argument('--translation-cap-mps',type=float);a=p.parse_args()
 targets.select_scene(a.scene)
 if a.plan_both:plan_goals([1,2])
 elif a.fetch_latest:fetch_latest()
 elif a.stop:stop_execution()
 elif a.manual_floor:manual_floor(a.manual_floor,a.floor_bounds)
 elif a.load_test_no_motion:
  if a.goal is None:raise ValueError('GOAL_REQUIRED')
  execute_session(a.goal,False,translation_cap_mps=a.translation_cap_mps,preview_duration=60.,load_test_no_motion=True)
 elif a.preview_right_micro:
  execute_session(1,False,right_micro_preview=True)
 elif a.execute or a.execute_preview or a.execute_micro or a.preview_micro or a.execute_cable or a.preview_cable:
  if not a.goal:p.error('execution requires --goal')
  execute(a.goal,a.execute or a.execute_micro or a.execute_cable,micro=a.execute_micro or a.preview_micro,cable=a.execute_cable or a.preview_cable,execution_seconds=a.execution_seconds,translation_cap_mps=a.translation_cap_mps)
 elif a.force_release:force_release(a.force_release)
 elif a.check:check()
 elif a.pipeline:pipeline()
 elif a.goal:plan(a.goal,cable=a.plan_cable)
 else:
  print('S100底盘计算；Jetson仅操作/显示。命令：1 / 2 / check / pipeline / quit')
  while True:
   try:command=input('S100导航> ').strip()
   except (EOFError,KeyboardInterrupt):break
   if command in ['q','quit','exit']:break
   try:
    if command in ['1','2']:plan(int(command))
    elif command=='check':check()
    elif command=='pipeline':pipeline()
    else:print('输入1、2、check、pipeline或quit')
   except Exception as exc:print(str(exc))
