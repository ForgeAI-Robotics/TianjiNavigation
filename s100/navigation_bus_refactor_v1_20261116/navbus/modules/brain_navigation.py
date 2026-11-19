"""Brain contract facade over the same task owner used by numbered GUI goals."""
from ..contracts import BusError, Call, Descriptor, BrainNavigationPort, plain


class BrainNavigationModule:
    def __init__(self, source: BrainNavigationPort):
        self._source = source

    @property
    def descriptor(self) -> Descriptor:
        return Descriptor('brain_navigation', ('health', 'status', 'world', 'graph', 'submit', 'get', 'cancel'),
                          contract_version='fq/reception-lan/v1',
                          mutations=('submit', 'cancel'), motion=('submit',), stops=('cancel',))

    def handle(self, call: Call) -> dict:
        args = plain(call.arguments)
        if call.operation in ('health', 'status', 'world', 'graph') and args:
            raise BusError('UNKNOWN_FIELDS', 'Metadata takes no arguments')
        if call.operation == 'get':
            command = args.get('command_id')
            if (set(args) != {'command_id'} or not isinstance(command, str)
                    or not 1 <= len(command) <= 160
                    or any(not (c.isascii() and (c.isalnum() or c in '_.:-')) for c in command)):
                raise BusError('INVALID_COMMAND_ID', 'Expected original brain command identity')
        # Exact body validation, identity, persistence and stop evidence remain
        # owned by the upstream compatibility implementation and task manager.
        return self._source.brain_exchange(call.operation, args)
