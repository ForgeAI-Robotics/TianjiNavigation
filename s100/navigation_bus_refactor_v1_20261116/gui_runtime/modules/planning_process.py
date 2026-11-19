"""Read-only replanning in a subprocess; the caller retains navigation locks.

There is no ROS control publisher here. Cancellation kills the worker process
 group, including native search children, before any result may be adopted.
"""
import os,signal,subprocess,sys,time,uuid,json
from pathlib import Path

def wait_worker(command,log_path,stop,require_session,timeout_s=120.):
 log_path=Path(log_path);log_path.parent.mkdir(parents=True,exist_ok=True)
 with log_path.open('ab') as log:
  process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
  began=time.monotonic()
  try:
   while True:
    require_session(time.monotonic())
    if stop.is_set():raise ValueError('EXECUTION_STOP_REQUESTED')
    code=process.poll()
    if code is not None:
     if code:raise ValueError('PLANNING_WORKER_FAILED_'+str(code))
     require_session(time.monotonic());return
    if time.monotonic()-began>timeout_s:raise ValueError('PLANNING_WORKER_TIMEOUT')
    stop.wait(.02)
  finally:
   if process.poll() is None:
    try:os.killpg(process.pid,signal.SIGTERM)
    except ProcessLookupError:pass
    try:process.wait(timeout=2.)
    except subprocess.TimeoutExpired:
     try:os.killpg(process.pid,signal.SIGKILL)
     except ProcessLookupError:pass
     process.wait(timeout=2.)

def run(goal,stop,require_session,folder):
 from paths import ROOT
 folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);token=uuid.uuid4().hex
 reply=folder/('replan_'+token+'.json');log=folder/('replan_'+token+'.log')
 wait_worker([sys.executable,str(Path(__file__).with_name('planning_worker.py')),'--goal',str(goal),'--reply',str(reply)],log,stop,require_session)
 require_session(time.monotonic());return json.loads(reply.read_text())
