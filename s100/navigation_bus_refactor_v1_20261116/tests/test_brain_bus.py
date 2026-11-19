"""Real task-store/protocol integration with a nonphysical injected executor."""
import copy
import hashlib
import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
import urllib.error

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'legacy_fixture/modules'))
from navigation_task_runtime import TaskManager
from navigation_task_api import make_server as broker_server
from navbus.adapters.legacy_http import LegacyGateway
from navbus.bus import Bus, Policy
from navbus.contracts import BusError, Call
from navbus.http_api import make_server
from navbus.modules.brain_navigation import BrainNavigationModule
from navbus.modules.navigation import NavigationModule


class Audit:
    def __init__(self):self.fail_intent=False;self.fail_result=False
    def record(self,row):
        if self.fail_intent and row['phase']=='intent' or self.fail_result and row['phase']=='completed':
            raise OSError('test audit fault')


class PureRunner:
    simulation=True
    motion_output_count=0
    def __init__(self):self.calls=0;self.counts={}
    def execution_count(self,ident):return self.counts.get(ident,0)
    def decorate_response(self,job,view):return view
    def run(self,job,cancel,emit):
        self.calls+=1;self.counts[job['id']]=self.counts.get(job['id'],0)+1
        emit('navigator',dict(phase='FOLLOWING',reason='NONPHYSICAL_CONTRACT_FIXTURE',session=job['id']))
        body=job.get('brain_command',{})
        cancel.wait(10 if body.get('target_id')=='contract_pending' else .02)
        if job['kind']=='plan':return 'PLAN_READY',dict(arrived=False,physical_execution=False)
        evidence=dict(confirmed=body.get('target_id')!='unconfirmed_stop',source='ISOLATED_CONTRACT_EXECUTOR',physical_execution=False)
        return ('CANCELED' if cancel.is_set() else 'SUCCEEDED'),dict(arrived=not cancel.is_set(),final_xyt=job['target']['pose'],goal_error=dict(distance_m=0.,heading_error_deg=0.),stop_evidence=evidence,physical_execution=False)


class BrainBusTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        (self.root/'ISOLATED_CONTRACT_TEST_ROOT').touch();(self.root/'config/scenes').mkdir(parents=True)
        sha='a'*64
        self.waypoints=self.root/'config/scenes/office_targets.json'
        self.waypoints.write_text(json.dumps(dict(targets={str(i):dict(id=i,pose=[i*.1,0.,0.],revision=1,map_folder=str(self.root/'fake-map'),map_voxel_sha256=sha,frame_id='session_base_start') for i in [1,2,3]})))
        (self.root/'config/brain_compat.json').write_text(json.dumps(dict(enabled=True,contract_version='fq/reception-lan/v1',scene='office',internal_frame_id='session_base_start',map_sha256=sha,T_internal_map_xyt=[0.,0.,0.],map_definition='Private synthetic fixture',supported_motion_modes=['forward_path'],unsupported_motion_modes=['lateral_path_aligned'],translation_cap_mps=.1,timeout_s=0.,updated_at='2026-11-16T00:00:00Z')))
        self.runner=PureRunner();self.manager=TaskManager(self.root,runner=self.runner,busy=lambda:False)
        self.broker=broker_server(self.manager,'upstream-test-token',port=0,readiness=lambda:dict(arm_fresh=True,head_fresh=True,receiver_status_age_s=0.,clearance_clear=True,shapemask_clear=True))
        self.gateway=LegacyGateway('http://127.0.0.1:'+str(self.broker.server_port),'upstream-test-token',access='isolated-fixture')
        self.audit=Audit();self.bus=Bus([BrainNavigationModule(self.gateway),NavigationModule(self.gateway)],self.audit,Policy(allow_motion=True,mode='nonphysical-contract-fixture'),time.time)
        self.server=make_server(self.bus,'downstream-test-token',port=0)
        self.threads=[]
        for s in [self.broker,self.server]:
            t=threading.Thread(target=s.serve_forever,kwargs=dict(poll_interval=.01));t.start();self.threads.append(t)
        self.before=self.waypoints.read_bytes()
    def tearDown(self):
        for s in [self.server,self.broker]:s.shutdown();s.server_close()
        self.manager.close()
        for t in self.threads:t.join()
        self.temp.cleanup()
    def body(self,cid='brain-1',target='normal'):
        return dict(contract_version='fq/reception-lan/v1',task_id='business-1',command_id=cid,target_id=target,route_phase='',leg_index=1,frame_id='map',goal_xyt=[.17,-.23,.07],motion_mode='forward_path',require_final_orientation=True)
    def request(self,method,path,body=None,token='downstream-test-token'):
        req=urllib.request.Request('http://127.0.0.1:'+str(self.server.server_port)+path,method=method,data=json.dumps(body).encode() if body is not None else None,headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(req,timeout=3) as r:return r.status,json.load(r)
        except urllib.error.HTTPError as e:return e.code,json.load(e)
    def terminal(self,cid):
        for _ in range(100):
            status,row=self.request('GET','/v1/commands/'+cid)
            if row.get('state') in ('succeeded','failed','cancelled'):return row
            time.sleep(.01)
        self.fail('fixture did not terminate')
    def test_metadata_auth_map_and_modes(self):
        self.assertEqual(self.request('GET','/health',token='bad')[0],401)
        for path in ['/health','/v1/status','/v1/world','/total_scene_graph_latest.json']:
            self.assertEqual(self.request('GET',path)[0],200)
        graph=self.request('GET','/total_scene_graph_latest.json')[1]
        self.assertEqual(graph['agent_navigation_contract']['supported_motion_modes'],['forward_path'])
        self.assertEqual(self.request('GET','/v1/world')[1]['map_sha256'],'a'*64)
    def test_identity_idempotency_and_shared_gui_ledger(self):
        b=self.body();code,job=self.request('POST','/v1/navigation/goals',b);self.assertEqual(code,202)
        other=self.request('POST','/v1/navigation/goals',b);self.assertEqual(other[0],200)
        self.assertEqual(job['internal_task_id'],other[1]['internal_task_id'])
        result=self.terminal(b['command_id']);self.assertTrue(result['result']['navigation_stopped']);self.assertTrue(result['result']['success'])
        original=self.bus.dispatch(Call.parse(dict(module='navigation',operation='get',arguments={'task_id':job['internal_task_id']}),'gui-check'))['result']
        self.assertEqual(original['brain_command']['command_id'],b['command_id']);self.assertEqual(original['target']['id'],0)
        self.assertEqual(original['target']['pose'],b['goal_xyt']);self.assertEqual(self.runner.calls,1)
        self.assertEqual(self.waypoints.read_bytes(),self.before)
    def test_conflicts_errors_and_unsupported_modes_preserved(self):
        b=self.body();self.request('POST','/v1/navigation/goals',b);b['goal_xyt'][0]+=.1
        code,error=self.request('POST','/v1/navigation/goals',b);self.assertEqual(code,409);self.assertEqual(error['error']['code'],'COMMAND_ID_CONFLICT');self.assertIn('details',error['error'])
        self.terminal('brain-1');b=self.body('brain-2');b['motion_mode']='lateral_path_aligned'
        self.assertEqual(self.request('POST','/v1/navigation/goals',b)[1]['error']['code'],'INVALID_MOTION_MODE')
        self.assertEqual(self.runner.calls,1)
        self.assertEqual(self.request('GET','/v1/commands/missing')[1]['error']['code'],'COMMAND_NOT_FOUND')
    def test_cancel_checks_business_identity_and_keeps_stop_uncertainty(self):
        b=self.body(target='contract_pending');self.request('POST','/v1/navigation/goals',b)
        self.assertEqual(self.request('POST','/v1/navigation/cancel',dict(task_id='foreign',command_id='brain-1'))[0],422)
        self.assertEqual(self.request('POST','/v1/navigation/cancel',dict(task_id='business-1',command_id='brain-1'))[0],202)
        self.assertEqual(self.terminal('brain-1')['state'],'cancelled')
        self.request('POST','/v1/navigation/goals',self.body('brain-2','unconfirmed_stop'))
        result=self.terminal('brain-2');self.assertEqual(result['state'],'failed');self.assertFalse(result['result']['navigation_stopped']);self.assertFalse(result['result']['success'])
    def test_repeated_cancel_cannot_stop_a_new_command(self):
        self.request('POST','/v1/navigation/goals',self.body('old'))
        self.terminal('old');self.request('POST','/v1/navigation/goals',self.body('current','contract_pending'))
        self.request('POST','/v1/navigation/cancel',dict(task_id='business-1',command_id='old'))
        active=self.request('GET','/v1/commands/current')[1];self.assertNotEqual(active['state'],'cancelled')
        self.request('POST','/v1/navigation/cancel',dict(task_id='business-1',command_id='current'));self.terminal('current')
    def test_upstream_post_loss_is_not_fabricated_as_definite_rejection(self):
        b=self.body(target='contract_post_lost')
        with self.assertRaises((http.client.RemoteDisconnected,urllib.error.URLError)):
            self.request('POST','/v1/navigation/goals',b)
        self.assertEqual(self.terminal('brain-1')['state'],'succeeded');self.assertEqual(self.runner.calls,1)
    def test_result_audit_failure_preserves_accepted_command_lookup(self):
        self.audit.fail_result=True
        with self.assertRaises((http.client.RemoteDisconnected,urllib.error.URLError)):
            self.request('POST','/v1/navigation/goals',self.body())
        self.audit.fail_result=False
        self.assertEqual(self.terminal('brain-1')['state'],'succeeded');self.assertEqual(self.runner.calls,1)
    def test_stop_is_dispatched_even_if_bus_audit_is_unavailable(self):
        self.request('POST','/v1/navigation/goals',self.body(target='contract_pending'));self.audit.fail_intent=True;self.audit.fail_result=True
        code,r=self.request('POST','/v1/navigation/cancel',dict(task_id='business-1',command_id='brain-1'));self.assertEqual(code,202);self.assertIn('bus_audit_warnings',r)
        self.audit.fail_intent=False;self.audit.fail_result=False;self.assertEqual(self.terminal('brain-1')['state'],'cancelled')
    def test_motion_disabled_and_shadow_cannot_submit(self):
        bus=Bus([BrainNavigationModule(self.gateway)],self.audit,Policy(),time.time)
        with self.assertRaises(BusError):bus.dispatch(Call.parse(dict(module='brain_navigation',operation='submit',arguments=self.body()),'deny'))
        shadow=LegacyGateway('http://127.0.0.1:'+str(self.broker.server_port),'upstream-test-token',access='read-only')
        with self.assertRaisesRegex(BusError,'cannot mutate'):shadow.brain_exchange('cancel',dict(task_id='business-1',command_id='brain-1'))
        self.assertEqual(self.runner.calls,0)
    def test_numbered_endpoint_is_still_compatible(self):
        code,r=self.request('POST','/v1/navigation/goals',dict(mode='plan',goal=2,scene='office'))
        self.assertEqual(code,200);self.assertEqual(r['result']['target']['id'],2)

if __name__=='__main__':unittest.main()
