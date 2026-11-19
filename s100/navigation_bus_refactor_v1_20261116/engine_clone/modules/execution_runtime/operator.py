"""Session IO injected as ports; no robot or business-module imports."""
import json


class OperatorChannel:
    def __init__(self, lease, stop, lines, clock, thread_factory):
        self._lease, self._stop, self._lines = lease, stop, lines
        self._clock, self._thread_factory = clock, thread_factory
        self._threads = []

    def start(self):
        for target in (self.read, self.watchdog):
            thread = self._thread_factory(target=target, daemon=True)
            self._threads.append(thread)
            thread.start()

    def read(self):
        try:
            for line in self._lines:
                self._lease.receive(json.loads(line), self._clock())
                if self._stop.is_set():
                    return
            if not self._stop.is_set():
                self._lease.fail('OPERATOR_INPUT_CLOSED', self._clock())
        except (ValueError, OSError) as exc:
            self._lease.fail('OPERATOR_INPUT_ERROR:'+type(exc).__name__, self._clock())

    def watchdog(self):
        while not self._stop.wait(.05):
            self._lease.check(self._clock())

    def age(self):
        return self._lease.age(self._clock())
