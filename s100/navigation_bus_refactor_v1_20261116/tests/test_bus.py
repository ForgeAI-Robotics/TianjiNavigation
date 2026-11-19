import ast
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from navbus.adapters.storage import DocumentReader
from navbus.bus import Bus, Policy
from navbus.client import BusClient
from navbus.contracts import BusError, Call
from navbus.http_api import make_server
from navbus.modules.navigation import NavigationModule


class Audit:
    def __init__(self): self.items = []
    def record(self, item): self.items.append(item)


class Tasks:
    def __init__(self): self.calls = []; self.task = 'a'*32
    def submit(self, kind, payload, key):
        self.calls.append((kind, payload, key))
        return dict(id=self.task, state='PLAN_READY', revision=1, simulation=True)
    def get(self, task_id): return dict(id=task_id, state='PLAN_READY', revision=1, simulation=True)
    def cancel(self, task_id): return dict(id=task_id, state='CANCELED')
    def events(self, task_id, after, wait): return dict(events=[], state='PLAN_READY')


class Tests(unittest.TestCase):
    def setUp(self):
        self.tasks, self.audit = Tasks(), Audit()
        self.bus = Bus([NavigationModule(self.tasks)], self.audit, Policy(), time.time)

    def call(self, operation, arguments, key='request-1'):
        return Call.parse(dict(module='navigation', operation=operation, arguments=arguments), key)

    def test_motion_denied_before_upstream_call(self):
        with self.assertRaisesRegex(BusError, 'does not admit'):
            self.bus.dispatch(self.call('navigate', dict(goal=1)))
        self.assertEqual(self.tasks.calls, [])

    def test_idempotency_identity_and_upstream_task_are_not_replaced(self):
        result = self.bus.dispatch(self.call('plan', dict(goal=3), 'same-request'))
        self.assertEqual(result['result']['id'], self.tasks.task)
        self.assertEqual(self.tasks.calls[0][2], 'same-request')
        self.assertFalse(hasattr(self.bus, 'jobs'))

    def test_nested_command_data_cannot_be_mutated(self):
        original = dict(goal=1, options=dict(open_grippers=True))
        call = self.call('plan', original)
        original['options']['open_grippers'] = False
        self.assertTrue(call.arguments['options']['open_grippers'])
        with self.assertRaises(TypeError): call.arguments['options']['open_grippers'] = False

    def test_invalid_goal_never_reaches_task_port(self):
        for goal in (0, 4, True, '1'):
            with self.assertRaises(BusError): self.bus.dispatch(self.call('plan', dict(goal=goal)))
        self.assertFalse(self.tasks.calls)

    def test_instances_do_not_share_registry_or_request_state(self):
        other = Bus([], Audit(), Policy(), time.time)
        self.bus.dispatch(self.call('plan', dict(goal=1)))
        with self.assertRaises(BusError): other.dispatch(self.call('plan', dict(goal=1)))

    def test_audit_failure_prevents_mutation(self):
        class Failed:
            def record(self, item): raise OSError('disk unavailable')
        bus = Bus([NavigationModule(self.tasks)], Failed(), Policy(), time.time)
        with self.assertRaisesRegex(BusError, 'not dispatched'):
            bus.dispatch(self.call('plan', dict(goal=1)))
        self.assertFalse(self.tasks.calls)

    def test_audit_failure_does_not_prevent_stop_dispatch(self):
        class Failed:
            def record(self, item): raise OSError('disk full')
        bus = Bus([NavigationModule(self.tasks)], Failed(), Policy(), time.time)
        result = bus.dispatch(self.call('cancel', dict(task_id='a'*32)))
        self.assertEqual(result['result']['state'], 'CANCELED')
        self.assertTrue(result['audit_warnings'])

    def test_motion_policy_is_generic_not_hard_coded_to_navigation(self):
        from navbus.contracts import Descriptor
        class Other:
            descriptor = Descriptor('other', ('move',), motion=('move',))
            def handle(self, call): raise AssertionError('must not run')
        bus = Bus([Other()], Audit(), Policy(), time.time)
        with self.assertRaisesRegex(BusError, 'does not admit'):
            bus.dispatch(Call.parse(dict(module='other', operation='move', arguments={}), 'other-key'))

    def test_stale_or_broken_primary_cannot_be_replaced_with_green_fallback(self):
        with tempfile.TemporaryDirectory() as folder:
            primary, fallback = Path(folder)/'ram.json', Path(folder)/'disk.json'
            primary.write_text('{broken');fallback.write_text('{"fresh":true}')
            with self.assertRaises(BusError): DocumentReader(primary, fallback).read()
            primary.write_text('{"observation_time":0,"status":"STALE"}')
            value = DocumentReader(primary, fallback).read()
            self.assertEqual(value['data']['observation_time'], 0)
            self.assertFalse(value['timestamps_rewritten'])

    def test_http_sdk_uses_only_bus_and_keeps_goal_identity(self):
        server = make_server(self.bus, 'test-token', port=0)
        thread = threading.Thread(target=server.serve_forever, daemon=True);thread.start()
        try:
            client = BusClient('http://127.0.0.1:'+str(server.server_address[1]), 'test-token')
            job = client.plan(2, request_id='http-idempotency')
            self.assertEqual(client.wait(job['id'])['id'], job['id'])
            self.assertEqual(self.tasks.calls[0][1]['goal'], 2)
            self.assertEqual(self.tasks.calls[0][2], 'http-idempotency')
            with self.assertRaisesRegex(RuntimeError, 'MOTION_DISABLED'): client.navigate(1)
            with self.assertRaisesRegex(RuntimeError, 'UNAUTHORIZED'):
                BusClient('http://127.0.0.1:'+str(server.server_address[1]), 'bad').plan(1)
        finally:
            server.shutdown();server.server_close();thread.join()

    def test_business_modules_cannot_import_each_other_or_io(self):
        root = Path(__file__).resolve().parents[1]/'navbus/modules'
        for file in root.glob('*.py'):
            tree = ast.parse(file.read_text())
            for node in tree.body:
                self.assertFalse(isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)), file.name)
            for node in ast.walk(tree):
                if isinstance(node, ast.Import): self.fail('Business module imported implementation: '+file.name)
                if isinstance(node, ast.ImportFrom):
                    self.assertEqual((node.level, node.module), (2, 'contracts'), file.name)


if __name__ == '__main__': unittest.main()
