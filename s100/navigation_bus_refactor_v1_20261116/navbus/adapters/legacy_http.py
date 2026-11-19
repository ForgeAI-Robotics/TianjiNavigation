import http.client
import json
from urllib.parse import urlsplit, urlencode, quote
from ..contracts import BusError


class LegacyGateway:
    def __init__(self, url: str, token: str, access: str = 'read-only', timeout: float = 10.):
        parsed = urlsplit(url)
        if parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost'):
            raise ValueError('Legacy adapter is S100 loopback-only')
        if access not in ('read-only', 'isolated-fixture', 'gui-runtime'):
            raise ValueError('Unsupported legacy access policy')
        if access == 'gui-runtime' and parsed.port != 18768:
            raise ValueError('GUI adapter must use its dedicated broker')
        self._host, self._port, self._prefix = parsed.hostname, parsed.port, parsed.path.rstrip('/')
        self._token, self._access, self._timeout = token, access, timeout

    def request(self, method: str, path: str, body=None, key=None, preserve_response=False) -> dict:
        if method != 'GET' and self._access not in ('isolated-fixture', 'gui-runtime'):
            raise BusError('SHADOW_WRITE_DENIED', 'Read-only adapter cannot mutate production tasks', 409)
        connection = http.client.HTTPConnection(self._host, self._port, timeout=self._timeout)
        headers = {'Authorization': 'Bearer '+self._token, 'Content-Type': 'application/json'}
        if key:
            headers['Idempotency-Key'] = key
        try:
            payload = json.dumps(body, allow_nan=False).encode() if body is not None else None
            connection.request(method, self._prefix+path, payload, headers)
            response = connection.getresponse()
            data = json.loads(response.read())
            if response.status >= 400:
                error = data.get('error', {})
                raise BusError(error.get('code', 'LEGACY_ERROR'), error.get('message', 'Legacy request failed'), response.status, response=data if preserve_response else None)
            return dict(http_status=response.status, body=data) if preserve_response else data
        except (OSError, http.client.HTTPException, json.JSONDecodeError) as exc:
            if preserve_response and method == 'POST':
                raise BusError('UPSTREAM_RESULT_UNCONFIRMED',
                               'Query the original command identity; do not submit a new command', 503) from exc
            raise
        finally:
            connection.close()

    def submit(self, kind, payload, key):
        endpoint = {'plan': 'plans', 'navigate': 'tasks', 'test': 'tests'}[kind]
        return self.request('POST', '/v1/navigation/'+endpoint, payload, key)

    def get(self, task_id):
        return self.request('GET', '/v1/navigation/tasks/'+task_id)

    def cancel(self, task_id):
        return self.request('POST', '/v1/navigation/tasks/'+task_id+'/cancel', {})

    def events(self, task_id, after, wait):
        return self.request('GET', '/v1/navigation/tasks/'+task_id+'/events?'+urlencode(dict(after=after, wait=wait)))

    def readiness(self):
        return self.request('GET', '/v1/readiness')

    def waypoints(self, scene):
        return self.request('GET', '/v1/waypoints?'+urlencode(dict(scene=scene)))


    def brain_exchange(self, operation, arguments):
        routes = {'health': '/health', 'status': '/v1/status', 'world': '/v1/world',
                  'graph': '/total_scene_graph_latest.json'}
        if operation in routes:
            return self.request('GET', routes[operation], preserve_response=True)
        if operation == 'get':
            return self.request('GET', '/v1/commands/'+quote(arguments['command_id'], safe=''), preserve_response=True)
        route = {'submit': '/v1/navigation/goals', 'cancel': '/v1/navigation/cancel'}[operation]
        return self.request('POST', route, arguments, preserve_response=True)
