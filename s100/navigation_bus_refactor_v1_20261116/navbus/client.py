"""Brain/GUI SDK: knows the bus contract, never imports any business module."""
import http.client
import json
import time
import uuid
from urllib.parse import urlsplit


class BusClient:
    def __init__(self, url: str, token: str, timeout: float = 15., connection_factory=None):
        parsed = urlsplit(url)
        if parsed.scheme != 'http' or not parsed.hostname:
            raise ValueError('HTTP bus URL required')
        self._host, self._port, self._prefix = parsed.hostname, parsed.port, parsed.path.rstrip('/')
        self._token, self._timeout = token, timeout
        self._connection_factory = connection_factory

    def call(self, module, operation, arguments=None, request_id=None):
        connection = self._connection_factory() if self._connection_factory else http.client.HTTPConnection(self._host, self._port, timeout=self._timeout)
        key = request_id or uuid.uuid4().hex
        body = dict(module=module, operation=operation, arguments=arguments or {})
        headers = {'Authorization': 'Bearer '+self._token, 'Content-Type': 'application/json',
                   'Idempotency-Key': key}
        try:
            connection.request('POST', self._prefix+'/v1/bus/call', json.dumps(body, allow_nan=False), headers)
            response = connection.getresponse()
            data = json.loads(response.read())
            if response.status >= 400:
                raise RuntimeError(data['error']['code']+': '+data['error']['message'])
            return data['result']
        finally:
            connection.close()

    def plan(self, goal, scene='office', request_id=None, **options):
        return self.call('navigation', 'plan', dict(goal=goal, scene=scene, **options), request_id)

    def navigate(self, goal, scene='office', request_id=None, **options):
        return self.call('navigation', 'navigate', dict(goal=goal, scene=scene, **options), request_id)

    def get(self, task_id):
        return self.call('navigation', 'get', dict(task_id=task_id))

    def cancel(self, task_id):
        return self.call('navigation', 'cancel', dict(task_id=task_id))

    def wait(self, task_id, timeout=30.):
        started = time.monotonic()
        while True:
            job = self.get(task_id)
            if job['state'] in ('SUCCEEDED', 'FAILED', 'CANCELED', 'INTERRUPTED', 'PLAN_READY', 'TEST_COMPLETE'):
                return job
            if time.monotonic()-started >= timeout:
                raise TimeoutError('Query the same upstream task ID; no implicit cancellation')
            self.call('navigation', 'events', dict(task_id=task_id, after=job['revision'], wait=.5))
