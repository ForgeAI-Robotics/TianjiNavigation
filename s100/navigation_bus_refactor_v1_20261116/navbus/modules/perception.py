from ..contracts import BusError, Call, Descriptor, DocumentPort


class PerceptionModule:
    def __init__(self, source: DocumentPort):
        self._source = source

    @property
    def descriptor(self) -> Descriptor:
        return Descriptor('perception', ('status',))

    def handle(self, call: Call) -> dict:
        if call.arguments:
            raise BusError('UNKNOWN_FIELDS', 'Status takes no arguments')
        # Only status metadata enters the task bus. Point clouds stay on the
        # existing RAM/filter transport, outside this low-rate control API.
        return self._source.read()
