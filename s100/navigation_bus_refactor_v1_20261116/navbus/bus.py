"""One composition-owned registry. No business-module imports or task ledger."""
from dataclasses import dataclass
from types import MappingProxyType
from collections.abc import Iterable
from typing import Callable
from .contracts import AuditPort, BusError, Call, Module, plain


@dataclass(frozen=True)
class Policy:
    allow_motion: bool = False
    mode: str = 'isolated-read-only'


class Bus:
    def __init__(self, modules: Iterable[Module], audit: AuditPort, policy: Policy,
                 clock: Callable[[], float]):
        entries = {}
        for module in modules:
            name = module.descriptor.name
            if name in entries:
                raise ValueError('Duplicate module: ' + name)
            descriptor = module.descriptor
            if not set(descriptor.mutations+descriptor.motion+descriptor.stops) <= set(descriptor.operations):
                raise ValueError('Invalid module capability declaration: ' + name)
            entries[name] = module
        self._modules = MappingProxyType(entries)
        self._audit, self._policy, self._clock = audit, policy, clock

    def describe(self) -> dict:
        return dict(mode=self._policy.mode, motion_enabled=self._policy.allow_motion,
                    modules=[dict(name=m.descriptor.name, operations=m.descriptor.operations,
                                  contract_version=m.descriptor.contract_version,
                                  mutations=m.descriptor.mutations, motion=m.descriptor.motion,
                                  stops=m.descriptor.stops)
                             for m in self._modules.values()])

    def dispatch(self, call: Call) -> dict:
        module = self._modules.get(call.module)
        if module is None or call.operation not in module.descriptor.operations:
            raise BusError('OPERATION_NOT_FOUND', 'Unknown module operation', 404)
        if call.operation in module.descriptor.motion and not self._policy.allow_motion:
            raise BusError('MOTION_DISABLED', 'Isolated bus does not admit physical navigation', 409)
        started = self._clock()
        identity = dict(request_id=call.request_id, module=call.module, operation=call.operation,
                        durable=call.operation in module.descriptor.mutations)
        warnings = []
        stopping = call.operation in module.descriptor.stops
        try:
            self._audit.record(dict(identity, phase='intent', time=started))
        except OSError as exc:
            if not stopping:
                raise BusError('AUDIT_UNAVAILABLE', 'Request was not dispatched', 503) from exc
            warnings.append('STOP_INTENT_AUDIT_UNAVAILABLE')
        try:
            result = module.handle(call)
        except Exception as exc:
            try:
                self._audit.record(dict(identity, phase='failed', time=self._clock(),
                                        code=getattr(exc, 'code', type(exc).__name__)))
            except OSError:
                pass
            raise
        try:
            self._audit.record(dict(identity, phase='completed', time=self._clock(),
                                    elapsed_s=self._clock()-started))
        except OSError as exc:
            if not stopping:
                raise BusError('RESULT_AUDIT_UNCONFIRMED', 'Query/retry with the same request ID', 503) from exc
            warnings.append('STOP_RESULT_AUDIT_UNAVAILABLE')
        return dict(identity, bus_mode=self._policy.mode, result=plain(result), audit_warnings=warnings)
