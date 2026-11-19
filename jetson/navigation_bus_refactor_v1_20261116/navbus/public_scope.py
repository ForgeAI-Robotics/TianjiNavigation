"""Public ingress scope; the private GUI bus retains its own scene support."""
import json
from urllib.parse import quote


class ScopeDenied(ValueError):
    pass


def office_only(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if key in ('scene', 'scene_id') and item not in (None, 'office'):
                raise ScopeDenied('PUBLIC_SCENE_UNAVAILABLE')
            office_only(item)
    elif isinstance(value, list):
        for item in value:
            office_only(item)


def request_scope(method, path, payload):
    """Return optional read-only lookup used before identity-based operations."""
    if method != 'POST':
        return None
    try:
        body = json.loads(payload or b'{}', parse_constant=lambda value: (_ for _ in ()).throw(ValueError()))
    except (ValueError, UnicodeError):
        raise ScopeDenied('INVALID_PUBLIC_REQUEST') from None
    if not isinstance(body, dict):
        raise ScopeDenied('INVALID_PUBLIC_REQUEST')
    office_only(body)
    if path == '/v1/navigation/cancel':
        command = body.get('command_id')
        return command_lookup(command)
    if path != '/v1/bus/call':
        return None
    module, operation, args = body.get('module'), body.get('operation'), body.get('arguments')
    permitted = {'navigation': ('plan','navigate','test','get','cancel','events'),
                 'world': ('waypoints',), 'readiness': ('get',),
                 'brain_navigation': ('health','status','world','graph','submit','get','cancel')}
    if module not in permitted or operation not in permitted[module] or not isinstance(args, dict):
        raise ScopeDenied('PUBLIC_OPERATION_UNAVAILABLE')
    if module == 'navigation' and operation in ('get','cancel','events'):
        return ('POST', '/v1/bus/call', dict(module='navigation',operation='get',arguments=dict(task_id=args.get('task_id'))))
    if module == 'brain_navigation' and operation in ('get','cancel'):
        return command_lookup(args.get('command_id'))
    return None


def command_lookup(command):
    if not isinstance(command, str) or not 1 <= len(command) <= 160 or any(
            not (c.isascii() and (c.isalnum() or c in '_.:-')) for c in command):
        raise ScopeDenied('INVALID_COMMAND_ID')
    return ('GET', '/v1/commands/'+quote(command, safe=''), None)


def public_response(data):
    office_only(data)
    if isinstance(data, dict) and isinstance(data.get('modules'), list):
        data = dict(data, modules=[item for item in data['modules']
                                  if item.get('name') in ('navigation','world','readiness','brain_navigation')])
    return data
