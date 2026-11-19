"""Gateway transport checks using only the nonphysical broker fixture."""
import http.client
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.request
import urllib.error
import test_brain_bus as fixtures
from navbus.jetson_gateway import make_gateway, GatewaySettings, private_token


class LocalUpstream:
    def __init__(self,port):self.port=port;self.calls=0
    def open(self):
        self.calls+=1
        return http.client.HTTPConnection('127.0.0.1',self.port,timeout=3),'downstream-test-token'


class GatewayTests(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.BrainBusTests();self.fixture.setUp()
        self.upstream=LocalUpstream(self.fixture.server.server_port)
        self.gateway=make_gateway(self.upstream,'external-gateway-test-token',port=0)
        self.thread=threading.Thread(target=self.gateway.serve_forever,kwargs=dict(poll_interval=.01));self.thread.start()
    def tearDown(self):
        self.gateway.shutdown();self.gateway.server_close();self.thread.join();self.fixture.tearDown()
    def request(self,method,path,body=None,token='external-gateway-test-token'):
        req=urllib.request.Request('http://127.0.0.1:'+str(self.gateway.server_port)+path,method=method,data=json.dumps(body).encode() if body is not None else None,headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(req,timeout=3) as response:return response.status,json.load(response)
        except urllib.error.HTTPError as exc:return exc.code,json.load(exc)
    def terminal(self,cid):
        import time
        for _ in range(100):
            status,row=self.request('GET','/v1/commands/'+cid)
            if row.get('state') in ('succeeded','failed','cancelled'):return row
            time.sleep(.01)
        self.fail('nonphysical task did not terminate')
    def test_own_credential_required_and_not_used_as_upstream_credential(self):
        self.assertEqual(self.request('GET','/health',token='downstream-test-token')[0],401)
        self.assertEqual(self.upstream.calls,0)
        self.assertEqual(self.request('GET','/health')[0],200)
        self.assertEqual(self.upstream.calls,1)
    def test_coordinate_identity_and_status_pass_through_without_new_ledger(self):
        body=self.fixture.body();status,accepted=self.request('POST','/v1/navigation/goals',body)
        self.assertEqual(status,202);result=self.terminal(body['command_id'])
        self.assertEqual(result['goal_xyt'],body['goal_xyt']);self.assertEqual(result['internal_task_id'],accepted['internal_task_id'])
        self.assertEqual(result['state'],'succeeded');self.assertEqual(self.fixture.runner.calls,1)
        self.assertFalse(hasattr(self.upstream,'jobs'))
    def test_cancel_and_structured_errors_keep_original_identity(self):
        self.request('POST','/v1/navigation/goals',self.fixture.body(target='contract_pending'))
        status,error=self.request('POST','/v1/navigation/cancel',dict(task_id='other',command_id='brain-1'))
        self.assertEqual(status,422);self.assertIn('details',error['error'])
        self.assertEqual(self.request('POST','/v1/navigation/cancel',dict(task_id='business-1',command_id='brain-1'))[0],202)
        self.assertEqual(self.terminal('brain-1')['state'],'cancelled')
    def test_post_loss_is_not_replayed_or_changed_into_definite_http_failure(self):
        with self.assertRaises((http.client.RemoteDisconnected,urllib.error.URLError)):
            self.request('POST','/v1/navigation/goals',self.fixture.body(target='contract_post_lost'))
        self.assertEqual(self.terminal('brain-1')['state'],'succeeded');self.assertEqual(self.fixture.runner.calls,1)
    def test_numbered_goals_still_use_s100_task_store(self):
        status,row=self.request('POST','/v1/navigation/goals',dict(goal=3,scene='office',mode='plan'))
        self.assertEqual(status,200);ident=row['result']['id'];self.assertIn(ident,self.fixture.manager.jobs)
    def test_unknown_routes_and_ambiguous_framing_never_reach_upstream(self):
        self.assertEqual(self.request('GET','/not-a-navigation-route')[0],404)
        connection=http.client.HTTPConnection('127.0.0.1',self.gateway.server_port,timeout=3)
        connection.putrequest('POST','/v1/navigation/goals');connection.putheader('Authorization','Bearer external-gateway-test-token')
        connection.putheader('Content-Length','0');connection.putheader('Content-Length','1');connection.endheaders()
        response=connection.getresponse();self.assertEqual(response.status,400);response.read();connection.close()
        self.assertEqual(self.upstream.calls,0)
    def test_upstream_failure_before_send_cannot_look_like_healthy_navigation(self):
        def broken():raise OSError('offline fixture')
        self.upstream.open=broken
        code,row=self.request('GET','/health');self.assertEqual(code,503);self.assertEqual(row['error']['code'],'S100_UPSTREAM_UNAVAILABLE')
        code,row=self.request('POST','/v1/navigation/goals',self.fixture.body());self.assertEqual(code,503);self.assertFalse(row['error']['details']['upstream_attempted'])
        self.assertEqual(self.fixture.runner.calls,0)
    def test_other_scene_queries_and_creations_are_rejected_before_upstream(self):
        requests=[('/v1/navigation/goals',dict(goal=1,scene='outside',mode='plan')),
                  ('/v1/bus/call',dict(module='world',operation='waypoints',arguments=dict(scene='outside'))),
                  ('/v1/bus/call',dict(module='navigation',operation='navigate',arguments=dict(goal=1,scene='outside')))]
        for path,body in requests:
            status,data=self.request('POST',path,body);self.assertEqual(status,403)
            self.assertNotIn('outside',json.dumps(data))
        self.assertEqual(self.request('GET','/v1/world?scene=outside')[0],403)
        self.assertEqual(self.upstream.calls,0)
    def test_public_world_has_three_office_points_and_no_internal_modules(self):
        status,data=self.request('POST','/v1/bus/call',dict(module='world',operation='waypoints',arguments={}))
        self.assertEqual(status,200);self.assertEqual(data['result']['scene'],'office')
        self.assertEqual([v['id'] for v in data['result']['waypoints']],[1,2,3])
        self.assertEqual([v['id'] for v in self.request('GET','/total_scene_graph_latest.json')[1]['objects']],['office_waypoint_1','office_waypoint_2','office_waypoint_3'])
        names=[v['name'] for v in self.request('GET','/v1/modules')[1]['modules']]
        self.assertNotIn('localization',names);self.assertNotIn('perception',names)
    def test_history_and_cancel_cannot_expose_private_scene_task(self):
        rows=json.loads(self.fixture.waypoints.read_text())
        (self.fixture.root/'config/targets.json').write_text(json.dumps(rows))
        job,_=self.fixture.manager.submit('plan',dict(goal=1,scene='outside'),'private-scene-fixture')
        for operation in ['get','events','cancel']:
            status,data=self.request('POST','/v1/bus/call',dict(module='navigation',operation=operation,arguments=dict(task_id=job['id'])))
            self.assertEqual(status,403);self.assertNotIn('pose',json.dumps(data));self.assertNotIn('outside',json.dumps(data))

    def test_source_allowlist_is_checked_before_auth_or_upstream(self):
        server=make_gateway(self.upstream,'external-gateway-test-token',allowed_clients=('192.0.2.1',))
        thread=threading.Thread(target=server.serve_forever,kwargs=dict(poll_interval=.01));thread.start()
        try:
            req=urllib.request.Request('http://127.0.0.1:'+str(server.server_port)+'/health',headers={'Authorization':'Bearer external-gateway-test-token'})
            with self.assertRaises(urllib.error.HTTPError) as err:urllib.request.urlopen(req,timeout=3)
            self.assertEqual(err.exception.code,403);self.assertEqual(self.upstream.calls,0)
        finally:server.shutdown();server.server_close();thread.join()


class GatewayConfigTests(unittest.TestCase):
    def test_explicit_binding_and_pinned_upstream(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'config.json';base=dict(bind='127.0.0.1',port=18769,allowed_clients=['127.0.0.1'],token_file='token',connection_file='connection',known_hosts='hosts',remote_token_file='/private/token')
            for change in [{},{'bind':'0.0.0.0'},{'upstream_port':8765}]:
                p.write_text(json.dumps(dict(base,**change)))
                if change:
                    with self.assertRaises(ValueError):GatewaySettings.load(p)
                else:self.assertEqual(GatewaySettings.load(p).upstream_port,18769)
    def test_gateway_credential_is_independent_private_and_reused(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'token';one=private_token(p);self.assertEqual(one,private_token(p));self.assertEqual(p.stat().st_mode&0o777,0o600)
            p.chmod(0o644)
            with self.assertRaises(ValueError):private_token(p)

if __name__=='__main__':unittest.main()
