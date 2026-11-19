"""Jetson HTTP ingress only: no task ledger, executor, ROS or robot SDK.

Each request is sent at most once. Unknown POST outcomes remain transport
failures so the unchanged brain client queries the original command identity.
"""
import argparse
from dataclasses import dataclass
import hashlib
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import logging
import os
from pathlib import Path
import secrets
import signal
import socket
import threading
import time
from urllib.parse import urlsplit, parse_qs
from .public_scope import ScopeDenied, request_scope, public_response


@dataclass(frozen=True)
class GatewaySettings:
    bind: str
    port: int
    allowed_clients: tuple
    token_file: Path
    connection_file: Path
    known_hosts: Path
    remote_token_file: str
    upstream_port: int = 18769
    timeout_s: float = 12.
    maximum_body_bytes: int = 32768

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        data = json.loads(path.read_text())
        for name in ('token_file', 'connection_file', 'known_hosts'):
            value = Path(data[name])
            data[name] = value if value.is_absolute() else path.parent.parent/value
        data['allowed_clients'] = tuple(data['allowed_clients'])
        result = cls(**data)
        bind = ipaddress.ip_address(result.bind)
        if not (bind.is_private or bind.is_loopback) or bind.is_unspecified:
            raise ValueError('Gateway must bind one explicit local address')
        if type(result.port) is not int or not 1024 <= result.port <= 65535:
            raise ValueError('Invalid gateway port')
        if not result.allowed_clients:
            raise ValueError('Explicit brain source addresses required')
        for address in result.allowed_clients:
            ipaddress.ip_address(address)
        if result.upstream_port != 18769 or not 1 <= result.timeout_s <= 15:
            raise ValueError('Only the existing S100 bus and bounded timeout are allowed')
        if result.maximum_body_bytes != 32768:
            raise ValueError('Request limit must match the existing bus')
        return result


def private_token(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(secrets.token_urlsafe(32)+'\n')
    if path.is_symlink() or path.stat().st_mode & 0o077:
        raise ValueError('Gateway credential must be a private regular file')
    value = path.read_text().strip()
    if not 24 <= len(value) <= 4096 or any(not 33 <= ord(c) <= 126 for c in value):
        raise ValueError('Invalid gateway credential')
    return value


class SSHUpstream:
    """Pinned-host SSH transport; reconnect only BEFORE opening an HTTP request."""
    def __init__(self, config):
        self.config = config
        self._lock = threading.Lock()
        self._ssh = None
        self._token = None

    def open(self):
        import paramiko
        with self._lock:
            transport = self._ssh.get_transport() if self._ssh else None
            if transport is None or not transport.is_active():
                if self._ssh:
                    self._ssh.close()
                self._ssh = None
                values = json.loads(self.config.connection_file.read_text())
                ssh = paramiko.SSHClient()
                ssh.load_host_keys(str(self.config.known_hosts))
                ssh.set_missing_host_key_policy(paramiko.RejectPolicy())
                try:
                    ssh.connect(values['host'], username=values['username'], password=values['password'],
                                timeout=8, auth_timeout=8, banner_timeout=8,
                                look_for_keys=False, allow_agent=False)
                    with ssh.open_sftp() as sftp:
                        with sftp.open(self.config.remote_token_file) as stream:
                            token = stream.read(4097).decode('ascii').strip()
                    if not 24 <= len(token) <= 4096 or any(not 33 <= ord(c) <= 126 for c in token):
                        raise ValueError('Invalid upstream credential')
                    self._ssh, self._token = ssh, token
                except BaseException:
                    ssh.close()
                    raise
                transport = ssh.get_transport()
            config = self.config
            class Connection(http.client.HTTPConnection):
                def connect(self):
                    self.sock = transport.open_channel('direct-tcpip',
                        ('127.0.0.1', config.upstream_port), ('127.0.0.1', 0), timeout=8)
                    self.sock.settimeout(config.timeout_s)
            return Connection('127.0.0.1', config.upstream_port, timeout=config.timeout_s), self._token

    def close(self):
        with self._lock:
            if self._ssh:
                self._ssh.close()
            self._ssh, self._token = None, None


def make_gateway(upstream, token, *, bind='127.0.0.1', port=0,
                 allowed_clients=('127.0.0.1',), timeout_s=12., logger=None):
    lane = threading.BoundedSemaphore(12)
    stop_lane = threading.BoundedSemaphore(2)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        server_version = 'TianjiJetsonNavigationGateway/1'

        def setup(self):
            super().setup()
            self.connection.settimeout(timeout_s)

        def log_message(self, *_):
            pass  # Never log credentials or request bodies.

        def reply(self, status, body):
            self.gateway_status = status
            data = json.dumps(body, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def disconnect_uncertain(self):
            self.close_connection = True
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.connection.close()

        def dispatch(self):
            self.gateway_status = None
            started = time.monotonic()
            connection = None
            acquired = None
            attempted = False
            status = None
            parsed = urlsplit(self.path)
            path = parsed.path
            try:
                if self.client_address[0] not in allowed_clients:
                    self.close_connection = True
                    return self.reply(403, dict(error=dict(code='GATEWAY_SOURCE_DENIED')))
                if not secrets.compare_digest(self.headers.get('Authorization', '').encode(),
                                              ('Bearer '+token).encode()):
                    self.close_connection = True
                    return self.reply(401, dict(error=dict(code='UNAUTHORIZED')))
                get_paths = {'/healthz', '/health', '/v1/status', '/v1/world',
                             '/total_scene_graph_latest.json', '/v1/modules'}
                post_paths = {'/v1/navigation/goals', '/v1/navigation/cancel', '/v1/bus/call'}
                permitted = (self.command == 'GET' and (path in get_paths or path.startswith('/v1/commands/'))
                             or self.command == 'POST' and path in post_paths)
                if parsed.scheme or parsed.netloc or parsed.fragment or not permitted:
                    self.close_connection = True
                    return self.reply(404, dict(error=dict(code='GATEWAY_ENDPOINT_NOT_FOUND')))
                lengths = self.headers.get_all('Content-Length', [])
                if self.headers.get('Transfer-Encoding') or len(lengths) > 1:
                    self.close_connection = True
                    return self.reply(400, dict(error=dict(code='INVALID_REQUEST_FRAMING')))
                try:
                    size = int(lengths[0]) if lengths else 0
                except ValueError:
                    size = -1
                if not 0 <= size <= 32768 or self.command == 'GET' and size:
                    self.close_connection = True
                    return self.reply(413, dict(error=dict(code='INVALID_REQUEST_SIZE')))
                payload = self.rfile.read(size) if size else None
                if payload is not None and len(payload) != size:
                    self.close_connection = True
                    return self.reply(400, dict(error=dict(code='INCOMPLETE_REQUEST_BODY')))
                query = parse_qs(parsed.query, keep_blank_values=True)
                if any(value != 'office' for key in ('scene','scene_id') for value in query.get(key, [])):
                    raise ScopeDenied('PUBLIC_SCENE_UNAVAILABLE')
                lookup = request_scope(self.command, path, payload)
                # Reserve cancellation capacity even when ordinary polls fill up.
                selected = stop_lane if path == '/v1/navigation/cancel' else lane
                if not selected.acquire(blocking=False):
                    return self.reply(503, dict(error=dict(code='GATEWAY_BUSY',
                        details=dict(action_started=False, upstream_attempted=False))))
                acquired = selected
                connection, upstream_token = upstream.open()
                headers = {'Authorization': 'Bearer '+upstream_token,
                           'Content-Type': self.headers.get('Content-Type', 'application/json')}
                if self.headers.get('Idempotency-Key'):
                    headers['Idempotency-Key'] = self.headers['Idempotency-Key']
                if lookup is not None:
                    method, route, body = lookup
                    connection.request(method, route, body=json.dumps(body).encode() if body is not None else None,
                                       headers=headers)
                    checked = connection.getresponse()
                    checked_data = checked.read(4*1024*1024+1)
                    if len(checked_data)>4*1024*1024:
                        raise ValueError('Scope lookup budget exceeded')
                    public_response(json.loads(checked_data))
                    if checked.status >= 400:
                        return self.reply(checked.status, json.loads(checked_data))
                # Never replay this request, including on SSH disconnect.
                attempted = True
                connection.request(self.command, self.path, body=payload, headers=headers)
                response = connection.getresponse()
                data = response.read(4*1024*1024+1)
                if len(data) > 4*1024*1024:
                    raise ValueError('Upstream response exceeds task metadata budget')
                data = json.dumps(public_response(json.loads(data)), allow_nan=False).encode()
                status = response.status
                self.gateway_status = status
                self.send_response(status)
                self.send_header('Content-Type', response.getheader('Content-Type', 'application/json'))
                self.send_header('Content-Length', str(len(data)))
                self.send_header('X-Tianji-Gateway', 'Jetson')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)
            except ScopeDenied as exc:
                if self.command=='POST' and attempted:
                    return self.disconnect_uncertain()
                self.close_connection = True
                return self.reply(403, dict(error=dict(code=str(exc),
                    details=dict(action_started=False if not attempted else None,
                                 action_state_uncertain=self.command=='POST' and attempted))))
            except Exception:
                if self.command == 'POST' and attempted:
                    return self.disconnect_uncertain()
                self.close_connection = True
                try:
                    self.reply(503, dict(error=dict(code='S100_UPSTREAM_UNAVAILABLE',
                        details=dict(upstream_attempted=attempted, action_started=False))))
                except OSError:
                    pass
            finally:
                if connection:
                    connection.close()
                if acquired:
                    acquired.release()
                if logger:
                    # Hash unknown command paths; never log arbitrary URL text.
                    route = path if path in ('/healthz','/health','/v1/status','/v1/world',
                        '/total_scene_graph_latest.json','/v1/modules','/v1/navigation/goals',
                        '/v1/navigation/cancel','/v1/bus/call') else 'command-or-rejected:'+hashlib.sha256(path.encode()).hexdigest()[:12]
                    logger.info(json.dumps(dict(method=self.command,route=route,http_status=self.gateway_status,
                        upstream_attempted=attempted,result_uncertain=self.command=='POST' and attempted and self.gateway_status is None,elapsed_s=time.monotonic()-started)))

        do_GET = dispatch
        do_POST = dispatch

    class Server(ThreadingHTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    return Server((bind, port), Handler)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    config = GatewaySettings.load(args.config)
    token = private_token(config.token_file)
    upstream = SSHUpstream(config)
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    logger = logging.getLogger('navigation.jetson_gateway')
    server = make_gateway(upstream, token, bind=config.bind, port=config.port,
                          allowed_clients=config.allowed_clients, timeout_s=config.timeout_s, logger=logger)
    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, stop)
    logger.info(json.dumps(dict(service='JETSON_NAVIGATION_GATEWAY',bind=config.bind,port=config.port,
                               task_ledger='S100_ONLY',tasks_submitted=0)))
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        server.server_close()
        upstream.close()


if __name__ == '__main__':
    main()
