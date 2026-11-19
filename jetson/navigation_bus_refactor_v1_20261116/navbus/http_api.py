"""Transport only. All business operations go through the injected bus."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import secrets
import socket
import uuid
from urllib.parse import urlsplit
from .contracts import BusError, Call


def make_server(bus, token: str, host='127.0.0.1', port=18766):
    if host != '127.0.0.1':
        raise ValueError('Isolated bus must remain on loopback')

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def setup(self):
            super().setup()
            self.connection.settimeout(15.)

        def log_message(self, *_):
            pass

        def reply(self, code, body):
            payload = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def body(self):
            try:
                size = int(self.headers.get('Content-Length', '0'))
            except ValueError as exc:
                raise BusError('INVALID_LENGTH', 'Invalid body size', 400) from exc
            if not 0 < size <= 32768 or self.headers.get('Transfer-Encoding'):
                raise BusError('INVALID_LENGTH', 'Body must be 1..32768 bytes', 400)
            try:
                return json.loads(self.rfile.read(size))
            except (ValueError, UnicodeError) as exc:
                raise BusError('INVALID_JSON', 'Invalid JSON', 400) from exc

        def brain_reply(self, operation, arguments, key):
            self.brain_operation = operation
            self.brain_arguments = arguments
            call = Call.parse(dict(module='brain_navigation', operation=operation,
                                   arguments=arguments), key)
            reply = bus.dispatch(call)
            envelope = reply['result']
            payload = envelope['body']
            if reply['audit_warnings']:
                payload = dict(payload, bus_audit_warnings=reply['audit_warnings'])
            return self.reply(envelope['http_status'], payload)

        def uncertain_connection(self):
            # The existing brain client recovers transport loss by querying the
            # original command_id. A fabricated HTTP failure could conceal an
            # accepted command, so preserve that uncertainty across the bus.
            self.close_connection = True
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.connection.close()

        def dispatch(self):
            self.brain_operation = None
            self.brain_arguments = {}
            path = urlsplit(self.path).path.rstrip('/')
            try:
                if self.command == 'GET' and path == '/healthz':
                    return self.reply(200, bus.describe())
                auth = self.headers.get('Authorization', '').encode()
                if not secrets.compare_digest(auth, ('Bearer '+token).encode()):
                    raise BusError('UNAUTHORIZED', 'Authorization required', 401)
                if self.command == 'GET' and path == '/v1/modules':
                    return self.reply(200, bus.describe())
                key = self.headers.get('Idempotency-Key') or uuid.uuid4().hex
                brain_metadata = {'/health': 'health', '/v1/status': 'status',
                                  '/v1/world': 'world', '/total_scene_graph_latest.json': 'graph'}
                if self.command == 'GET' and path in brain_metadata:
                    return self.brain_reply(brain_metadata[path], {}, key)
                if self.command == 'GET' and path.startswith('/v1/commands/'):
                    from urllib.parse import unquote
                    return self.brain_reply('get', {'command_id': unquote(path[len('/v1/commands/'):])}, key)
                if self.command != 'POST':
                    raise BusError('NOT_FOUND', 'Unknown endpoint', 404)
                body = self.body()
                if path == '/v1/navigation/cancel':
                    return self.brain_reply('cancel', body, key)
                if path == '/v1/navigation/goals':
                    if not isinstance(body, dict):
                        raise BusError('INVALID_ARGUMENTS', 'Expected goal object')
                    if any(k in body for k in ('contract_version', 'goal_xyt', 'command_id')):
                        return self.brain_reply('submit', body, key)
                    operation = body.pop('mode', 'navigate')
                    if operation not in ('plan', 'navigate', 'test'):
                        raise BusError('INVALID_OPERATION', 'Unknown navigation mode')
                    body = dict(module='navigation', operation=operation, arguments=body)
                elif path != '/v1/bus/call':
                    raise BusError('NOT_FOUND', 'Unknown endpoint', 404)
                self.reply(200, bus.dispatch(Call.parse(body, key)))
            except BusError as exc:
                if self.brain_operation in ('submit', 'cancel') and exc.code in (
                        'RESULT_AUDIT_UNCONFIRMED', 'UPSTREAM_RESULT_UNCONFIRMED'):
                    return self.uncertain_connection()
                payload = exc.response
                if payload is None and self.brain_operation:
                    uncertain = exc.code == 'RESULT_AUDIT_UNCONFIRMED'
                    details = dict(original_code=exc.code, action_state_uncertain=uncertain,
                                   action_started=None if uncertain else False)
                    if isinstance(self.brain_arguments, dict):
                        details.update({k: self.brain_arguments[k] for k in ('task_id', 'command_id')
                                        if k in self.brain_arguments})
                    code = 'SAFETY_NOT_READY' if exc.code == 'MOTION_DISABLED' else exc.code
                    payload = dict(success=False, error=dict(code=code, message=str(exc),
                                                            retryable=False, details=details))
                self.reply(exc.status, payload if payload is not None else dict(error=dict(code=exc.code, message=str(exc))))
            except Exception:
                if self.brain_operation in ('submit', 'cancel'):
                    return self.uncertain_connection()
                self.reply(500, dict(error=dict(code='INTERNAL_ERROR', message='Inspect bus audit log')))

        do_GET = dispatch
        do_POST = dispatch

    class Server(ThreadingHTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    return Server((host, port), Handler)
