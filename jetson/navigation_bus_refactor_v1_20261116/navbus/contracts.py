"""Immutable messages shared by modules and transports. No environment reads."""
from dataclasses import dataclass
from collections.abc import Mapping
from types import MappingProxyType
from typing import Any, Protocol
import math


class BusError(Exception):
    def __init__(self, code: str, message: str, status: int = 422, response=None):
        super().__init__(message)
        self.code, self.status, self.response = code, status, response


def freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({str(k): freeze(v) for k, v in value.items()})
    if isinstance(value, (tuple, list)):
        return tuple(freeze(v) for v in value)
    if value is None or type(value) in (str, int, bool):
        return value
    if type(value) is float and math.isfinite(value):
        return value
    raise BusError('INVALID_MESSAGE', 'Messages must contain finite JSON values')


def plain(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    return value


@dataclass(frozen=True)
class Call:
    request_id: str
    module: str
    operation: str
    arguments: Mapping[str, Any]

    @classmethod
    def parse(cls, value: Mapping[str, Any], request_id: str) -> 'Call':
        if not isinstance(value, Mapping) or set(value) != {'module', 'operation', 'arguments'}:
            raise BusError('INVALID_CALL', 'Expected module, operation, arguments')
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
            raise BusError('INVALID_REQUEST_ID', 'Request ID must be 1..128 characters')
        if not all(isinstance(value[k], str) and value[k] for k in ('module', 'operation')):
            raise BusError('INVALID_CALL', 'Module and operation must be strings')
        if not isinstance(value['arguments'], Mapping):
            raise BusError('INVALID_ARGUMENTS', 'Arguments must be an object')
        return cls(request_id, value['module'], value['operation'], freeze(value['arguments']))


@dataclass(frozen=True)
class Descriptor:
    name: str
    operations: tuple[str, ...]
    contract_version: str = '1.0'
    mutations: tuple[str, ...] = ()
    motion: tuple[str, ...] = ()
    stops: tuple[str, ...] = ()


class Module(Protocol):
    @property
    def descriptor(self) -> Descriptor: ...
    def handle(self, call: Call) -> Any: ...


class TaskPort(Protocol):
    def submit(self, kind: str, payload: dict, key: str) -> dict: ...
    def get(self, task_id: str) -> dict: ...
    def cancel(self, task_id: str) -> dict: ...
    def events(self, task_id: str, after: int, wait: float) -> dict: ...


class BrainNavigationPort(Protocol):
    def brain_exchange(self, operation: str, arguments: dict) -> dict: ...


class ReadinessPort(Protocol):
    def readiness(self) -> dict: ...


class WorldPort(Protocol):
    def waypoints(self, scene: str) -> dict: ...


class DocumentPort(Protocol):
    def read(self) -> dict: ...


class AuditPort(Protocol):
    def record(self, item: Mapping[str, Any]) -> None: ...


def goal_request(arguments: Mapping, operation: str) -> dict:
    allowed = {'goal', 'scene', 'client_id'}
    allowed |= {'options'} if operation == 'plan' else {'translation_cap_mps', 'timeout_s'}
    if operation == 'test':
        allowed.add('duration_s')
    if set(arguments) - allowed:
        raise BusError('UNKNOWN_FIELDS', ','.join(sorted(set(arguments) - allowed)))
    if type(arguments.get('goal')) is not int or arguments['goal'] not in (1, 2, 3):
        raise BusError('INVALID_GOAL', 'Goal must be 1, 2 or 3')
    result = plain(arguments)
    result.setdefault('scene', 'office')
    if result['scene'] not in ('office', 'outside'):
        raise BusError('INVALID_SCENE', 'Unknown scene')
    if operation != 'plan':
        speed = result.get('translation_cap_mps', .1)
        if type(speed) not in (int, float) or not .005 <= speed <= .1:
            raise BusError('INVALID_SPEED', 'Translation cap must be .005.. .1 m/s')
    # Full legacy schema validation stays behind the compatibility port.
    return result
