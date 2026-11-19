from ..contracts import BusError, Call, Descriptor, ReadinessPort


class ReadinessModule:
    def __init__(self, source: ReadinessPort):
        self._source = source

    @property
    def descriptor(self) -> Descriptor:
        return Descriptor('readiness', ('get',))

    def handle(self, call: Call) -> dict:
        if call.arguments:
            raise BusError('UNKNOWN_FIELDS', 'Readiness takes no arguments')
        return self._source.readiness()
