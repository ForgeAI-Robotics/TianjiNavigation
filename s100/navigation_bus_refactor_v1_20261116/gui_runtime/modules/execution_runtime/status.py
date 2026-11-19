"""Status ownership and sinks, separate from navigation/control decisions."""
import copy
import json
from .contracts import StatusIdentity, StatusInputs


class StatusPublisher:
    def __init__(self, root, identity: StatusIdentity, stopped, mono_clock, wall_clock,
                 atomic_write, queue_factory, console_write, output_failed,
                 joint_health, operator_summary, filter_timing, summarize):
        self._root, self._identity, self._stopped = root, identity, stopped
        self._mono, self._wall, self._atomic_write = mono_clock, wall_clock, atomic_write
        self._console_write, self._output_failed = console_write, output_failed
        self._joint_health, self._operator_summary = joint_health, operator_summary
        self._filter_timing, self._summarize = filter_timing, summarize
        self._last_phase, self._last_print = None, 0.
        self.history = []
        self.diagnostic_io = queue_factory(self._write)
        self.console_io = queue_factory(self._console, capacity=16)

    def _write(self, item):
        record, folder = item
        self._atomic_write(self._root/'runtime/navigation_execution_status.json', record)
        if folder:
            with (folder/'navigation_execution_history.jsonl').open('a') as stream:
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')

    def _console(self, record):
        try:
            self._console_write(record)
        except (BrokenPipeError, OSError):
            self._output_failed('OPERATOR_OUTPUT_CLOSED', self._mono())

    def emit(self, phase: str, inputs: StatusInputs, **extra):
        identity = self._identity
        control, worker = inputs.transport, inputs.frame_worker
        record = dict(
            active=not self._stopped() and not phase.startswith('STOP') and phase not in ('ARRIVED','TEST_COMPLETE','PLAN_BLOCKED'),
            pid=identity.pid, session=identity.session,
            mode='LOAD_TEST_NO_MOTION' if identity.load_test else 'REAL_MANUAL_START' if identity.enable_motion else 'PREVIEW_NO_ROS',
            translation_cap_mps=identity.requested_cap, phase=phase, compute_host='S100', updated_wall_time=self._wall(),
            micro_maximum_m=identity.micro_limit, operator_estop_declared=identity.operator_estop,
            hardware_stop_signal_verified=False,
            motion_commands_sent=control.sent if control and identity.enable_motion else 0,
            nonzero_packets=control.nonzero if control else 0,
            command_gate_stats=control.output_stats() if control else {},
            background_worker=worker.health() if worker else {},
            motor_trace_file=str(inputs.folder/'motor_trace.jsonl') if inputs.folder else None,
            sensor_relay=dict(inputs.relay_health), **extra)
        if worker:
            record['perception_timing'] = self._filter_timing(worker, self._mono())
        record['joint_feedback_health'] = self._joint_health()
        record['operator_session'] = self._operator_summary(self._mono())
        record['code_signature'] = identity.signature
        record['compute_cpu_affinity'] = list(identity.cpu_affinity)
        record['execution_profile'] = dict(inputs.execution_profile) if worker else None
        if inputs.route_summary is not None:
            record['route_recovery'] = dict(inputs.route_summary, active_plan_folder=str(inputs.active_plan_folder))
        record['controller_extension'] = dict(inputs.controller_extension)
        record['diagnostic_io'] = self.diagnostic_io.stats()
        # Queue/history own their observations; later caller mutation cannot
        # rewrite previously published status or stop evidence.
        record = copy.deepcopy(record)
        self.history.append(record)
        emit = bool(extra.get('finished') or phase != self._last_phase or self._mono()-self._last_print >= 1.)
        if extra.get('finished') and inputs.folder:
            self._atomic_write(inputs.folder/'navigation_performance_summary.json', self._summarize(self.history))
        self.diagnostic_io.submit((record, inputs.folder))
        if emit:
            self.console_io.submit(record)
            self._last_phase, self._last_print = phase, self._mono()
        return record
