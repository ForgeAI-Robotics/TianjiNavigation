from ..contracts import BusError, Call, Descriptor, DocumentPort


class LocalizationModule:
    def __init__(self, source: DocumentPort):
        self._source = source

    @property
    def descriptor(self) -> Descriptor:
        return Descriptor('localization', ('snapshot',))

    def handle(self, call: Call) -> dict:
        if call.arguments:
            raise BusError('UNKNOWN_FIELDS', 'Snapshot takes no arguments')
        # Original pose/time/hash fields are returned unchanged; this module
        # does not manufacture a freshness or collision permission.
        return self._source.read()
