"""Private decision stage; state and service dependencies are explicit."""
from dataclasses import dataclass
from typing import Any
from .contracts import ScopeFrame, Flow


class State(ScopeFrame):
    args: Any
    cable_precheck: Any
    certificate: Any
    collision_debug: Any
    copy: Any
    folder: Any
    follower: Any
    guard: Any
    last_collision_capture: Any
    last_micro_cloud: Any
    last_wait_reason: Any
    m: Any
    micro: Any
    new_pose: Any
    obstacle_wait: Any
    owned: Any
    pkt: Any
    plan: Any
    posture_reference: Any
    probe_deferral: Any
    progress: Any
    q: Any
    reason: Any
    record: Any
    recovery: Any
    refresh_output_posture: Any
    right_micro: Any
    route_recovery: Any
    sensor_recovery: Any
    started: Any
    status: Any
    stop: Any
    suggested: Any
    twist: Any
    wait_cycle: Any
    wait_reasons: Any
    warm: Any


@dataclass(frozen=True)
class Ports:
    BLOCKING_REASONS: Any
    CONTROL_CHECK_PERIOD_S: Any
    ControlTransport: Any
    POSE_RECHECK: Any
    classify_pose_wait: Any
    expired_deadlines: Any
    measured_joints: Any
    read_live_pose: Any
    time: Any
    waitable_data_fault: Any


def execute(frame, ports):
    from navigation_admission import admission
    admitted = frame.guard is not None and admission(frame.guard, 0.0)
    if frame.guard is not None and (not admitted):
        frame.reason = 'ADMISSION_DEADLINE_EXPIRED'
    pose_retry = False
    if frame.reason == ports.POSE_RECHECK and (not frame.micro):
        try:
            (pose_retry, frame.reason) = ports.classify_pose_wait(frame.reason, frame.record['saved_pose'], ports.measured_joints(), frame.m.sha256, ports.read_live_pose(), frame.record, frame.copy, ports.time.time(), ports.time.monotonic())
        except Exception as exc:
            frame.reason = str(exc)
    if frame.micro and frame.progress.was_admitted and (not frame.recovery.within_budget(ports.time.monotonic())):
        if frame.owned.transport:
            frame.owned.transport.zero()
        frame.progress.phase = 'STOP_GUARD_LOST'
        frame.progress.final_details = dict(reason='FRESH_DATA_RECOVERY_TIMEOUT', last_data_reason=frame.last_wait_reason, blocking_checks=dict(frame.wait_reasons))
        return Flow.BREAK
    required_ready = 3 if frame.micro or not frame.progress.was_admitted else 1
    if admitted:
        if frame.sensor_recovery.active:
            frame.recovery.resumed(ports.time.monotonic())
            if not frame.sensor_recovery.checked_clear(frame.copy, ports.time.monotonic()):
                if frame.owned.transport:
                    frame.owned.transport.zero()
                frame.progress.ready_count = 0
                frame.follower.pause('CONFIRMING_SENSOR_RECOVERY')
                frame.status('CONFIRMING_SENSOR_RECOVERY', reason='VERIFYING_CONTINUOUS_FRESH_DATA', limited_twist=[0.0, 0.0, 0.0], guard=frame.guard, sensor_recovery=frame.sensor_recovery.summary(ports.time.monotonic()))
                frame.stop.wait(max(0.0, ports.CONTROL_CHECK_PERIOD_S - (ports.time.monotonic() - frame.started)))
                return Flow.CONTINUE
        if frame.obstacle_wait.active:
            frame.recovery.resumed(ports.time.monotonic())
            if not frame.obstacle_wait.checked_clear(frame.copy, ports.time.monotonic()):
                if frame.owned.transport:
                    frame.owned.transport.zero()
                frame.progress.ready_count = 0
                frame.follower.pause('WAITING_FOR_OBSTACLE_CLEAR')
                frame.status('CONFIRMING_OBSTACLE_CLEAR', reason='VERIFYING_STABLE_CLEARANCE', limited_twist=[0.0, 0.0, 0.0], guard=frame.guard, obstacle_wait=frame.obstacle_wait.summary(ports.time.monotonic()))
                frame.stop.wait(max(0.0, ports.CONTROL_CHECK_PERIOD_S - (ports.time.monotonic() - frame.started)))
                return Flow.CONTINUE
        if frame.suggested['state'] == 'ARRIVED':
            if frame.owned.transport:
                frame.owned.transport.zero()
            confirmed = frame.follower.step(frame.new_pose, float(frame.pkt['observation_time']), ports.time.time(), frame.plan['target']['revision'], True, True, odometry_pose=frame.pkt['odometry_pose'], control_time=ports.time.monotonic())
            if confirmed['state'] != 'ARRIVED':
                frame.follower.pause('GOAL_MOVED_DURING_FINAL_CHECK')
                frame.progress.ready_count = 0
                frame.status('FINE_OBSERVING', reason='GOAL_MOVED_DURING_FINAL_CHECK', limited_twist=[0.0, 0.0, 0.0], goal_error=confirmed.get('goal_error'))
                frame.stop.wait(max(0.0, ports.CONTROL_CHECK_PERIOD_S - (ports.time.monotonic() - frame.started)))
                return Flow.CONTINUE
            frame.suggested = confirmed
            pending = frame.args.enable_motion and frame.plan['target'].get('source') == 'API_COORDINATE_GOAL'
            frame.progress.phase = 'GOAL_CONFIRMING_STOP' if pending else 'ARRIVED'
            frame.progress.final_details['arrival_verified_at'] = ports.time.time()
            frame.progress.final_details.update(reason='GOAL_CANDIDATE_PENDING_STOP' if pending else 'GOAL_TOLERANCE_REACHED', pre_parking_xyt=[float(v) for v in frame.new_pose], goal_error=frame.suggested.get('goal_error'), settled_s=frame.suggested.get('settled_s', 0.0), final_xyt=[float(v) for v in frame.new_pose], final_pose_observation_time=float(frame.pkt['observation_time']))
            frame.status(frame.progress.phase, cleanup_pending=True, limited_twist=[0.0, 0.0, 0.0], **frame.progress.final_details)
            return Flow.BREAK
        if not frame.micro:
            frame.progress.ready_count += 1
        else:
            cloud_key = tuple(((s, frame.copy[s][0]['sequence']) for s in sorted(frame.copy)))
            if cloud_key != frame.last_micro_cloud:
                frame.progress.ready_count += 1
                frame.last_micro_cloud = cloud_key
        if frame.args.enable_motion and frame.owned.transport is None and (frame.progress.ready_count >= required_ready):
            frame.owned.transport = ports.ControlTransport(True, posture_refresh=frame.refresh_output_posture)
        control_ready = frame.owned.transport is not None and frame.owned.transport.ready()
        if frame.right_micro and frame.progress.ready_count >= required_ready and control_ready:
            frame.progress.phase = 'TEST_COMPLETE'
            frame.progress.final_details = dict(reason='RIGHT_MICRO_GUARDS_PASSED', candidate_twist=frame.twist, checked_cloud_pairs=frame.progress.ready_count)
            return Flow.BREAK
        if frame.micro and frame.progress.ready_count >= required_ready and control_ready and (frame.micro.reference is None):
            if frame.probe_deferral and (not frame.probe_deferral.frozen):
                raise ValueError('PROBE_TEMPLATE_NOT_READY')
            frame.micro.arm(frame.pkt['odometry_pose'], ports.time.monotonic())
        if control_ready and frame.progress.ready_count >= required_ready:
            frame.owned.transport.update(frame.twist, frame.guard, posture_reference=frame.posture_reference)
            if not frame.micro:
                frame.follower.admitted(frame.twist, frame.pkt['odometry_pose'], ports.time.monotonic())
            if frame.micro and frame.micro.reference is not None:
                frame.micro.note_command(frame.twist, ports.time.monotonic())
            frame.progress.was_admitted = True
            frame.recovery.resumed(ports.time.monotonic())
        elif frame.owned.transport:
            frame.owned.transport.zero()
            if frame.micro:
                frame.micro.note_command([0.0, 0.0, 0.0], ports.time.monotonic())
            else:
                frame.follower.pause('CONTROL_NOT_READY')
        following_phase = 'FINE_ADJUSTMENT_PAUSED' if frame.suggested.get('mode') == 'fine_paused' else 'FINE_OBSERVING' if frame.suggested.get('mode') == 'fine_observe' else 'GOAL_SETTLING' if frame.suggested.get('mode') == 'goal_settling' else 'FINE_ADJUSTING' if str(frame.suggested.get('mode', '')).startswith('fine_') else 'FOLLOWING'
        frame.status(('CABLE_PROBE_MOVING' if frame.cable_precheck else following_phase) if frame.args.enable_motion and control_ready and (frame.progress.ready_count >= required_ready) else 'PREVIEW_READY' if not frame.args.enable_motion and control_ready and (frame.progress.ready_count >= required_ready) else 'WAIT_CONTROL_READY' if frame.progress.ready_count >= required_ready and (not control_ready) else 'ARMING_GUARDS', reason=frame.reason, requested_twist=frame.suggested['twist_preview'], limited_twist=frame.twist if control_ready and frame.progress.ready_count >= required_ready else [0.0, 0.0, 0.0], guard=frame.guard, tracking_mode=frame.suggested.get('mode'), goal_error=frame.suggested.get('goal_error'), settled_s=frame.suggested.get('settled_s', 0.0), settle_samples=frame.suggested.get('settle_samples', 0), control_note=frame.suggested.get('control_note'), fine_pulse_count=frame.suggested.get('fine_pulse_count', 0), fine_no_progress_pulses=frame.suggested.get('fine_no_progress_pulses', 0), loop_compute_s=ports.time.monotonic() - frame.started, probe_wire_deferral=frame.probe_deferral.summary() if frame.probe_deferral else None)
    else:
        frame.progress.ready_count = 0
        if frame.owned.transport:
            frame.owned.transport.zero()
        if frame.micro:
            frame.micro.note_command([0.0, 0.0, 0.0], ports.time.monotonic())
        else:
            frame.follower.pause('GUARD_NOT_ADMITTED')
        if frame.collision_debug and frame.reason == 'LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY' and (ports.time.monotonic() - frame.last_collision_capture >= 1.0):
            try:
                from collision_evidence import save as save_collision
                frame.progress.final_details['collision_report'] = save_collision(frame.folder / 'scene', frame.collision_debug, frame.copy, frame.q, frame.m.sha256, frame.args.session)
                frame.last_collision_capture = ports.time.monotonic()
            except Exception as evidence_error:
                frame.progress.final_details['collision_evidence_error'] = str(evidence_error)
        if frame.sensor_recovery.active:
            frame.sensor_recovery.interrupt_clear()
        frame.wait_reasons[frame.reason] += 1
        frame.last_wait_reason = frame.reason
        if not frame.micro and frame.reason in ('MAP_COLLISION', 'LOCALIZATION_RECHECK_REQUIRED'):
            frame.status('WAITING_FOR_ROUTE_CLEAR', reason=frame.route_recovery.reason, limited_twist=[0.0, 0.0, 0.0], blocking_checks=dict(frame.wait_reasons))
            frame.wait_cycle(frame.started, wake_for_scan=True)
            return Flow.CONTINUE
        if pose_retry:
            frame.recovery.resumed(ports.time.monotonic())
            frame.status('WAITING_FOR_POSE_CERTIFICATE', reason=frame.reason, limited_twist=[0.0, 0.0, 0.0], blocking_checks=dict(frame.wait_reasons), motion_certificate=frame.certificate)
            frame.stop.wait(max(0.0, ports.CONTROL_CHECK_PERIOD_S - (ports.time.monotonic() - frame.started)))
            return Flow.CONTINUE
        if not frame.micro and frame.reason in ports.BLOCKING_REASONS:
            frame.obstacle_wait.blocked(frame.reason, ports.time.monotonic(), frame.copy if frame.reason == 'LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY' else None)
            frame.follower.pause('WAITING_FOR_OBSTACLE_CLEAR')
            frame.recovery.resumed(ports.time.monotonic())
            frame.status('WAITING_FOR_OBSTACLE_CLEAR', reason=frame.reason, limited_twist=[0.0, 0.0, 0.0], obstacle_wait=frame.obstacle_wait.summary(ports.time.monotonic()), blocking_checks=dict(frame.wait_reasons), **{k: v for (k, v) in frame.progress.final_details.items() if k in ['collision_report', 'collision_evidence_error']})
            frame.stop.wait(max(0.0, ports.CONTROL_CHECK_PERIOD_S - (ports.time.monotonic() - frame.started)))
            return Flow.CONTINUE
        if frame.obstacle_wait.active:
            frame.obstacle_wait.interrupt_clear()
        if frame.progress.was_admitted or frame.obstacle_wait.active:
            diagnostics = dict(reason=frame.reason, blocking_checks=dict(frame.wait_reasons), guard=frame.guard, expired_deadlines=ports.expired_deadlines(frame.guard), loop_compute_s=ports.time.monotonic() - frame.started, **{k: v for (k, v) in frame.progress.final_details.items() if k in ['collision_report', 'collision_evidence_error']})
            bounded_wait = frame.recovery.waiting(frame.reason, frame.guard, ports.time.monotonic())
            if not frame.micro and ports.waitable_data_fault(frame.reason, frame.guard) and (frame.sensor_recovery.active or not bounded_wait):
                frame.sensor_recovery.blocked(frame.reason, ports.time.monotonic(), frame.copy if frame.has('copy') else None, guard=frame.guard)
                frame.status('WAITING_FOR_SENSOR_RECOVERY', limited_twist=[0.0, 0.0, 0.0], sensor_recovery=frame.sensor_recovery.summary(ports.time.monotonic()), **diagnostics)
                frame.wait_cycle(frame.started, wake_for_scan=True)
                return Flow.CONTINUE
            if bounded_wait:
                frame.status('PAUSED_FOR_FRESH_DATA', **diagnostics)
                frame.wait_cycle(frame.started, wake_for_scan=True)
                return Flow.CONTINUE
            frame.progress.phase = 'STOP_GUARD_LOST'
            frame.progress.final_details = diagnostics
            frame.status(frame.progress.phase, **frame.progress.final_details)
            return Flow.BREAK
        frame.status('WAITING_GUARDS', reason=frame.reason, limited_twist=[0.0, 0.0, 0.0], guard=frame.guard, loop_compute_s=ports.time.monotonic() - frame.started)
        if ports.time.monotonic() - frame.warm > 30.0:
            frame.progress.phase = 'STOP_GUARDS_NOT_READY'
            return Flow.BREAK
    frame.wait_cycle(frame.started, wake_for_scan=frame.reason == 'LIDAR_STALE' or bool(frame.guard and frame.guard['cloud_age_s'] > 0.85))
    return Flow.NEXT
