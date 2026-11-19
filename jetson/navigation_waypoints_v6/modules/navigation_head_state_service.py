"""Own read-only head feedback independently of GUI/Agent lifetime."""
import json,signal,subprocess,time,sys,threading
from pathlib import Path
import paths
ROOT=Path(__file__).resolve().parents[1]
FEEDBACK=Path('/home/nvidia/tianji_upper_client/tianji_m6_teleop_delivery_20260902/logs/shapemask_head_state.json')
PYTHON='/home/nvidia/tianji_upper_client/tianji_m6_teleop_delivery_20260902/.venv/bin/python'

def fresh():
 try:
  d=json.loads(FEEDBACK.read_text());return d.get('connected')and 0<=time.time()-d['received_at']<=.3
 except (OSError,ValueError,KeyError):return False

def main():
 from common import exclusive
 lock=exclusive(ROOT/'runtime/navigation_head_state_service.lock');stop=threading.Event();child=None;retry_at=0.
 for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,lambda *_:stop.set())
 try:
  while not stop.is_set():
   if child is not None and child.poll()is not None:child=None;retry_at=time.monotonic()+2.
   if child is None and time.monotonic()>=retry_at and not fresh():
    child=subprocess.Popen([PYTHON,str(ROOT/'modules/readonly_head_feedback.py'),'--duration','0'],close_fds=True)
   stop.wait(.5 if child is not None else .2)
 finally:
  if child is not None and child.poll()is None:
   child.terminate()
   try:child.wait(timeout=3)
   except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
  lock.close()

if __name__=='__main__':main()
