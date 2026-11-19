"""Bounded best-effort diagnostic IO, excluded from real-time control paths.

Items must be immutable snapshots after submit. Overflow discards oldest
queued diagnostics, never a sensor measurement or a control decision.
"""
import collections,threading,time

class AsyncDiagnostics:
 def __init__(self,write,capacity=64):
  if capacity<1:raise ValueError('INVALID_DIAGNOSTIC_CAPACITY')
  self.write=write;self.capacity=capacity;self.pending=collections.deque();self.condition=threading.Condition();self.closed=False;self.dropped=0;self.written=0;self.error=None;self.max_write_s=0.
  self.worker=threading.Thread(target=self.run,daemon=True,name='diagnostic_io');self.worker.start()
 def submit(self,item):
  with self.condition:
   if self.closed:raise RuntimeError('DIAGNOSTICS_CLOSED')
   if len(self.pending)>=self.capacity:self.pending.popleft();self.dropped+=1
   self.pending.append(item);self.condition.notify()
 def stats(self):
  with self.condition:return dict(pending=len(self.pending),dropped=self.dropped,written=self.written,error=self.error,max_write_s=self.max_write_s,worker_alive=self.worker.is_alive())
 def run(self):
  while True:
   with self.condition:
    while not self.pending and not self.closed:self.condition.wait()
    if not self.pending:return
    item=self.pending.popleft()
   began=time.monotonic()
   try:self.write(item)
   except Exception as exc:
    with self.condition:self.error=str(exc)
   else:
    with self.condition:self.written+=1
   finally:
    with self.condition:self.max_write_s=max(self.max_write_s,time.monotonic()-began)
 def close(self,timeout=4.):
  with self.condition:self.closed=True;self.condition.notify_all()
  self.worker.join(timeout=timeout)
  return not self.worker.is_alive()
