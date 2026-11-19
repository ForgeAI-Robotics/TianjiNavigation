"""GUI compatibility port; every navigation operation crosses the task bus."""
from pathlib import Path
import json
import sys
import time
from paths import ROOT

sys.path.insert(0,str(ROOT.parent/'navigation_bus_refactor_v1_20261116'))
from navbus.client import BusClient


class Client:
    def __init__(self, token, connection_factory):
        self.bus=BusClient('http://127.0.0.1:18769',token,connection_factory=connection_factory)

    def plan(self,goal,scene=None,**options):
        return self.bus.plan(goal,scene=scene or 'office',**options)

    def navigate(self,goal,scene=None,**options):
        return self.bus.navigate(goal,scene=scene or 'office',**options)

    def test(self,goal,scene=None,**options):
        return self.bus.call('navigation','test',dict(goal=goal,scene=scene or 'office',**options))

    def get(self,task_id):return self.bus.get(task_id)
    def cancel(self,task_id):return self.bus.cancel(task_id)
    def events(self,task_id,after=0,wait=0.):
        return self.bus.call('navigation','events',dict(task_id=task_id,after=after,wait=wait))

    def wait(self,task_id,timeout=None,on_update=None):
        began=time.monotonic();seen=None
        while True:
            job=self.get(task_id)
            if on_update and job['revision']!=seen:on_update(job)
            seen=job['revision']
            if job['state']in ('SUCCEEDED','FAILED','CANCELED','INTERRUPTED','PLAN_READY','TEST_COMPLETE'):return job
            if timeout is not None and time.monotonic()-began>=timeout:raise TimeoutError('Task remains queryable')
            self.events(task_id,seen,.5)

    def stop(self):
        file=ROOT/'runtime/operator_session.json'
        if not file.exists():return {'stopping':False,'reason':'NO_GUI_TASK'}
        session=json.loads(file.read_text())
        if session.get('task_backend')!='bus':return {'stopping':False,'reason':'NOT_BUS_OWNED'}
        return self.cancel(session['task_id'])
