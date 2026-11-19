"""Lifecycle of a read-only native geometry process; no ROS/control imports."""
import json
from pathlib import Path
import select
import subprocess


class FrozenGuard:
    def __init__(self, executable, file, world, timeout=5.):
        file=Path(file);file.write_text(json.dumps(world,allow_nan=False))
        self.process=subprocess.Popen([str(executable),str(file)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
        self.sequence=0;self.timeout=timeout;self.last_reply=None
    def __enter__(self):return self
    def clear(self, poses):
        self.sequence+=1
        self.process.stdin.write(json.dumps(dict(id=self.sequence,poses=poses),allow_nan=False)+'\n');self.process.stdin.flush()
        if not select.select([self.process.stdout],[],[],self.timeout)[0]:raise TimeoutError('CORRIDOR_GEOMETRY_TIMEOUT')
        reply=json.loads(self.process.stdout.readline());self.last_reply=reply
        if reply.get('id')!=self.sequence:raise ValueError('CORRIDOR_GEOMETRY_ID_MISMATCH')
        return reply.get('clear')is True
    def __exit__(self,*_):
        self.process.stdin.close()
        try:self.process.wait(timeout=2.)
        except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=2.)
        finally:self.process.stdout.close();self.process.stderr.close()
