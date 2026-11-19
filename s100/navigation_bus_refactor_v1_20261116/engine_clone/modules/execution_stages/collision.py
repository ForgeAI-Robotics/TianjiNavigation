"""Private decision stage; state and service dependencies are explicit."""
from dataclasses import dataclass
from typing import Any
from .contracts import ScopeFrame, Flow


class State(ScopeFrame):
    _: Any
    args: Any
    c: Any
    certificate: Any
    cloud_age: Any
    cloud_checked_at: Any
    collision_debug: Any
    copy: Any
    floor: Any
    follower: Any
    geometric: Any
    guard: Any
    inv_start: Any
    limit_live_twist: Any
    m: Any
    matrices: Any
    micro: Any
    n: Any
    new_pose: Any
    normal: Any
    office_slow_translation: Any
    offset: Any
    operator_age: Any
    owned: Any
    pkt: Any
    posture_reference: Any
    posture_weights: Any
    prepared: Any
    probe_deferral: Any
    q: Any
    reason: Any
    record: Any
    refresh_output_posture: Any
    release: Any
    route_recovery: Any
    shapes: Any
    started: Any
    suggested: Any
    twist: Any


@dataclass(frozen=True)
class Ports:
    BASE_ADVANCE_S: Any
    FUTURE_HORIZON_S: Any
    base_transform: Any
    certify_motion: Any
    corner_shift: Any
    joint_shift_bound: Any
    math: Any
    measured_joints: Any
    measured_shapes: Any
    np: Any
    override_settings: Any
    position_check: Any
    posture_check: Any
    read_live_pose: Any
    state: Any
    sweep_guard: Any
    time: Any
    trajectory_deviation: Any


def execute(frame, ports):
    if frame.geometric:
        elapsed = ports.time.monotonic() - frame.started
        raw = ports.state().get('clearance_state', {})
        clear_age = 0.0 if frame.release['forced_release_active'] else max(0.0, float(raw.get('source_age_s', 999.0)))
        clear_checked_at = ports.time.monotonic()
        scene_refresh_rechecks = 0
        for refresh_attempt in range(2):
            fresh_joints = ports.measured_joints()
            (fresh_ok, fresh_reason, fresh_age) = ports.posture_check(frame.record['saved_pose'], fresh_joints, frame.m.sha256, ports.time.time())
            if not fresh_ok:
                raise ValueError(fresh_reason)
            newest = ports.read_live_pose()
            (valid, new_reason, new_age) = ports.position_check(newest, frame.record, ports.time.time())
            newest_checked_at = ports.time.monotonic()
            if not frame.micro:
                frame.route_recovery.observe(newest)
                if frame.route_recovery.pending or frame.route_recovery.checking_anchor:
                    raise ValueError('LOCALIZATION_RECHECK_REQUIRED')
            if not valid:
                raise ValueError(new_reason)
            joint_bound = ports.joint_shift_bound(frame.shapes, frame.q['joint_positions_rad'], fresh_joints['joint_positions_rad'], frame.posture_weights)
            frame.certificate = ports.certify_motion(frame.shapes, frame.pkt['odometry_pose'], newest['odometry_pose'], frame.twist, joint_bound, max_advance_s=ports.BASE_ADVANCE_S if not frame.micro else 0.0) if ports.math.isfinite(joint_bound) else None
            bounded_revalidation = frame.certificate is not None and frame.certificate['proven']
            if bounded_revalidation:
                base_drift = frame.certificate['base_residual_bound_m']
                fresh_shapes = frame.shapes
                reference_q = frame.q['joint_positions_rad']
                drift = frame.certificate['total_residual_bound_m']
                reference_reserve = 0.005 - base_drift
            else:
                fresh_shapes = ports.measured_shapes(frame.m, fresh_joints['joint_positions_rad'])
                reference_q = fresh_joints['joint_positions_rad']
                frame.certificate = ports.certify_motion(frame.shapes, frame.pkt['odometry_pose'], newest['odometry_pose'], frame.twist, ports.corner_shift(frame.shapes, fresh_shapes), max_advance_s=ports.BASE_ADVANCE_S if not frame.micro else 0.0)
                base_drift = frame.certificate['base_residual_bound_m']
                drift = frame.certificate['total_residual_bound_m']
                reference_reserve = max(0.0, 0.005 - drift)
            refresh_scene = False
            if drift <= 0.005 and refresh_attempt == 0:
                old_scene_age = max((ports.time.monotonic() - v[2] + float(v[0].get('source_age_s', 0.0)) for v in frame.copy.values()), default=999.0)
                if old_scene_age > 0.75:
                    latest_prepared = frame.owned.frame_worker.snapshot()
                    latest_copy = {sensor: data['frame'] for (sensor, data) in latest_prepared.items()}
                    latest_age = max((ports.time.monotonic() - v[2] + float(v[0].get('source_age_s', 0.0)) for v in latest_copy.values()), default=999.0)
                    if set(latest_copy) == {'front', 'back'} and latest_age + 0.1 < old_scene_age:
                        frame.prepared = latest_prepared
                        frame.copy = latest_copy
                        refresh_scene = True
                        scene_refresh_rechecks += 1
                        if bounded_revalidation:
                            fresh_shapes = frame.shapes
            if drift <= 0.005 and (not refresh_scene):
                break
            if refresh_attempt:
                raise ValueError('POSE_MOVED_DURING_COLLISION_CHECK')
            frame.shapes = fresh_shapes
            if not bounded_revalidation:
                frame.q = fresh_joints
            frame.pkt = newest
            T_live = ports.np.asarray(frame.pkt['T_map_base'])
            frame.normal = T_live[:3, :3].T @ frame.n
            frame.offset = frame.n @ T_live[:3, 3] - frame.c
            frame.floor = [-frame.normal[0] / frame.normal[2], -frame.normal[1] / frame.normal[2], -frame.offset / frame.normal[2]]
            frame.prepared = frame.owned.frame_worker.snapshot()
            frame.copy = {sensor: data['frame'] for (sensor, data) in frame.prepared.items()}
            current_cloud_age = max((ports.time.monotonic() - v[2] + float(v[0].get('source_age_s', 0.0)) for v in frame.copy.values()), default=999.0)
            frame.twist = frame.limit_live_twist(frame.suggested['twist_preview'], current_cloud_age)
            frame.cloud_checked_at = ports.time.monotonic()
            (frame.geometric, frame.reason, frame.cloud_age) = ports.sweep_guard(frame.copy, frame.matrices, frame.pkt['odometry_pose'], frame.shapes, frame.m.sha256, frame.floor, frame.twist, ports.time.monotonic(), probe_deferral=frame.probe_deferral, revalidation_reserve_m=0.005, diagnostics=frame.collision_debug, prepared=frame.prepared, motion_advance_s=ports.BASE_ADVANCE_S if not frame.micro else 0.0, directional_age_compensation=frame.office_slow_translation)
            if not frame.geometric:
                raise ValueError(frame.reason)
        new_T = ports.np.asarray(newest['T_map_base'])
        frame.new_pose = [new_T[0, 3], new_T[1, 3], ports.math.atan2(new_T[1, 0], new_T[0, 0])]
        if ports.trajectory_deviation(frame.new_pose, frame.follower.path) > 0.12:
            raise ValueError('PATH_DEVIATION_REPLAN_REQUIRED')
        relative = frame.inv_start @ ports.base_transform(frame.new_pose)
        local = [relative[0, 3], relative[1, 3], ports.math.atan2(relative[1, 0], relative[0, 0])]
        (clear, new_reason) = frame.owned.map_guard.check(local, frame.twist)
        if not clear:
            if not frame.micro:
                new_reason = frame.route_recovery.map_failure(new_reason, {s['link'] for s in frame.shapes})
            raise ValueError(new_reason)
        localization_age = new_age + ports.time.monotonic() - newest_checked_at
        position_revalidated = True
        frame.pkt = newest
        frame.posture_reference = dict(joint_positions_rad=reference_q, shapes=fresh_shapes, reserve_m=reference_reserve, motion_weights_m=frame.posture_weights)
        final_posture = frame.refresh_output_posture(frame.posture_reference)
        if final_posture is None:
            latest_joint_input = ports.measured_joints()
            (valid_joint_input, joint_input_reason, frame._) = ports.posture_check(frame.record['saved_pose'], latest_joint_input, frame.m.sha256, ports.time.time())
            if not valid_joint_input:
                raise ValueError(joint_input_reason)
            raise ValueError('POSE_MOVED_DURING_COLLISION_CHECK')
        measured_pose_age = final_posture['age_s']
        localization_age = new_age + ports.time.monotonic() - newest_checked_at
        if not frame.release['forced_release_active']:
            clear_age += ports.time.monotonic() - clear_checked_at
        frame.guard = dict(state='CLEAR', path_checked=True, operator_estop_declared=frame.args.operator_estop_ready, map_sha256=frame.record['map_voxel_sha256'], model_sha256=frame.m.sha256, localization_age_s=localization_age, pose_age_s=measured_pose_age, posture_revalidated=True, posture_feedback_observation_time=final_posture['observation_time'], position_revalidated=position_revalidated, full_sweep_rechecks=refresh_attempt, measured_sweep_drift_m=drift, revalidation_reserve_m=0.005, clearance_age_s=clear_age, guard_age_s=0.0, cloud_age_s=frame.cloud_age + ports.time.monotonic() - frame.cloud_checked_at, operator_age_s=frame.operator_age(), forced_release_active=frame.release['forced_release_active'], override_version=ports.override_settings().get('updated_wall_time'), assumed_clearance=frame.release['forced_release_active'], perception_mode='BACKGROUND_SCAN_FILTER_AND_COLLISION', background_filter_reports={sensor: {k: data['report'][k] for k in ['points', 'filtered', 'protected', 'filter_compute_s', 'filter_frame', 'scan_pose_collision_points']} for (sensor, data) in frame.prepared.items()})
        frame.guard.update(posture_revalidation_mode='KINEMATIC_BOUND' if bounded_revalidation else 'EXACT_FK', base_revalidation_drift_m=base_drift, scene_refresh_rechecks=scene_refresh_rechecks, motion_certificate=frame.certificate, swept_pose_lease_s=max(0.0, min(0.35, ports.FUTURE_HORIZON_S - localization_age)))
        if frame.probe_deferral:
            frame.guard.update(perception_mode='INITIAL_WIRES_DEFERRED_TEST', probe_limit_m=0.06, probe_deferred_points=frame.probe_deferral.last_deferred)
    return Flow.NEXT
