"""Resource lifecycle only. Control closes before stop evidence/observers."""
import signal
import subprocess
from .contracts import ResourceBindings


class Resources:
    def __init__(self, bindings=None, remove_bus=None):
        self.bindings = bindings if bindings is not None else ResourceBindings()
        self.errors = []
        self._remove_bus = remove_bus

    def cleanup(self, name, action):
        try:
            action()
            return True
        except Exception as exc:
            self.errors.append(dict(resource=name, error=str(exc)))
            return False

    def stop_control(self):
        control = self.bindings.transport
        return self.cleanup('control_transport', control.close) if control else True

    @staticmethod
    def stop_child(child):
        if child.poll() is not None:
            return
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            child.terminate()
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=1)

    def stop_observers(self):
        b = self.bindings
        if b.frame_worker:
            self.cleanup('filter_workers', b.frame_worker.close)
        if b.stream:
            self.cleanup('sensor_stream', b.stream.close)
        if b.reader:
            self.cleanup('sensor_reader', lambda: b.reader.join(timeout=4))
        if b.tracker and not getattr(b.tracker, 'resident', False):
            self.cleanup('localization_tracker', lambda: self.stop_child(b.tracker))
        if b.resident_motor_trace[0]:
            self.cleanup('motor_trace', lambda: self.stop_child(b.resident_motor_trace[0]))
        if b.map_guard:
            self.cleanup('map_guard', b.map_guard.close)
        if b.observer_child:
            self.cleanup('raw_observer', lambda: self.stop_child(b.observer_child))
        if b.observer_log:
            self.cleanup('observer_log', b.observer_log.close)
        if b.sensor_bus and self._remove_bus:
            self.cleanup('sensor_bus', lambda: self._remove_bus(b.sensor_bus.folder))
