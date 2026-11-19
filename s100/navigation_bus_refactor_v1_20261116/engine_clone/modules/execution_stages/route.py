"""Private decision stage; state and service dependencies are explicit."""
from dataclasses import dataclass
from typing import Any
from .contracts import ScopeFrame, Flow


class State(ScopeFrame):
    PreparedFrames: Any
    _: Any
    active_plan_folder: Any
    args: Any
    cfg: Any
    folder: Any
    follower: Any
    inv_start: Any
    lease: Any
    m: Any
    micro: Any
    office_live_filter: Any
    owned: Any
    pkt: Any
    plan: Any
    pos: Any
    progress: Any
    radius: Any
    reason: Any
    record: Any
    refine_radius: Any
    route_recovery: Any
    sensor_recovery: Any
    started: Any
    status: Any
    stop: Any
    wait_cycle: Any


@dataclass(frozen=True)
class Ports:
    MapGuard: Any
    Path: Any
    PreviewFollower: Any
    ROOT: Any
    base_transform: Any
    exclusive: Any
    isolated_plan: Any
    json: Any
    measured_joints: Any
    measured_shapes: Any
    np: Any
    posture_check: Any
    read_live_pose: Any
    time: Any


def execute(frame, ports):
    if not frame.micro:
        frame.route_recovery.observe(frame.pkt)
        if frame.route_recovery.checking_anchor:
            if frame.reason == 'LOCALIZATION_FRAME_OR_EPOCH_CHANGED':
                raise ValueError(frame.reason)
            if frame.owned.transport:
                frame.owned.transport.zero()
            frame.follower.pause('LOCALIZATION_ANCHOR_CONFIRMING')
            frame.progress.ready_count = 0
            frame.status('WAITING_FOR_LOCALIZATION_STABILITY', reason='LOCALIZATION_ANCHOR_CONFIRMING', limited_twist=[0.0, 0.0, 0.0])
            frame.wait_cycle(frame.started, wake_for_scan=True)
            return Flow.CONTINUE
        if frame.route_recovery.pending:
            if frame.owned.transport:
                frame.owned.transport.zero()
            frame.follower.pause('STOPPED_FOR_ROUTE_RECOVERY')
            frame.progress.ready_count = 0
            if frame.reason == 'LOCALIZATION_FRAME_OR_EPOCH_CHANGED':
                raise ValueError(frame.reason)
            if frame.route_recovery.ready(frame.pkt, frame.pos, ports.time.monotonic()):
                from checked_route_reuse import candidate as reuse_candidate, checked as reuse_checked
                reusable = None
                try:
                    reusable = reuse_candidate(frame.route_recovery, frame.pkt, frame.record, frame.plan, frame.follower, ports.measured_joints(), ports.time.time())
                    if reusable is not None and reuse_checked(frame.active_plan_folder / 'scene/immutable_world.json', reusable):
                        frame.lease.require(ports.time.monotonic())
                        fresh = ports.read_live_pose()
                        same_anchor = ports.np.allclose(fresh.get('T_map_odom'), frame.pkt.get('T_map_odom'), atol=1e-07, rtol=0.0)
                        if same_anchor and reuse_candidate(frame.route_recovery, fresh, frame.record, frame.plan, frame.follower, ports.measured_joints(), ports.time.time()) is not None:
                            frame.route_recovery.accept_checked_route(fresh)
                            frame.sensor_recovery.blocked('LOCALIZATION_STALE', ports.time.monotonic())
                            frame.status('ROUTE_REVALIDATED_ARMING_GUARDS', reason='UNCHANGED_MAP_ROUTE_REVALIDATED', limited_twist=[0.0, 0.0, 0.0])
                            frame.wait_cycle(frame.started, wake_for_scan=True)
                            return Flow.CONTINUE
                except (ValueError, OSError, TypeError):
                    pass
                if frame.owned.frame_worker:
                    frame.owned.frame_worker.close()
                    frame.owned.frame_worker = None
                frame.status('REPLANNING_STOPPED', reason=frame.route_recovery.reason, limited_twist=[0.0, 0.0, 0.0])
                try:
                    previous_target = dict(frame.plan['target'])
                    with ports.exclusive(ports.ROOT / 'runtime/navigation.lock'):
                        renewed = ports.isolated_plan(frame.args.goal, frame.stop, frame.lease.require, frame.folder / 'replanning')
                    new_folder = ports.Path(ports.json.loads((ports.ROOT / 'runtime/latest_plan.json').read_text())['folder'])
                    new_record = ports.json.loads((new_folder / 'scene/frozen_input.json').read_text())
                    frame.lease.require(ports.time.monotonic())
                    if renewed['target'] != previous_target:
                        raise ValueError('TARGET_CHANGED_REPLAN_REQUIRED')
                    if new_record['model_sha256'] != frame.m.sha256 or new_record['map_voxel_sha256'] != frame.record['map_voxel_sha256']:
                        raise ValueError('REPLAN_IDENTITY_CHANGED')
                    for field in ['source_boot_id', 'odometry_epoch', 'calibration_sha256']:
                        if new_record['localization'][field] != frame.record['localization'][field]:
                            raise ValueError('REPLAN_SENSOR_IDENTITY_CHANGED')
                    (still_valid, posture_reason, frame._) = ports.posture_check(frame.record['saved_pose'], ports.measured_joints(), frame.m.sha256, ports.time.time())
                    if not still_valid:
                        raise ValueError(posture_reason)
                    if not renewed.get('selected'):
                        raise ValueError('NO_REVALIDATED_ROUTE')
                    replacement_guard = ports.MapGuard(new_folder / 'scene/immutable_world.json')
                    frame.owned.map_guard.close()
                    frame.owned.map_guard = replacement_guard
                    frame.plan = renewed
                    frame.record = new_record
                    frame.active_plan_folder = new_folder
                    frame.follower = ports.PreviewFollower(frame.plan, frame.cfg)
                    frame.inv_start = ports.np.linalg.inv(ports.base_transform(frame.plan['start_map_pose']))
                    frame.radius = max((ports.np.linalg.norm(ports.np.asarray(s['world_corners'])[:, :2], axis=1).max() for s in ports.measured_shapes(frame.m, frame.record['saved_pose']['joint_positions_rad'])))
                    frame.owned.frame_worker = frame.PreparedFrames(frame.m, frame.owned.sensor_bus.folder, conservative_fast=frame.office_live_filter, refine_near_hits=frame.office_live_filter, refine_radius_m=frame.refine_radius)
                    frame.route_recovery.adopt(frame.record['localization'])
                    frame.sensor_recovery.blocked('LOCALIZATION_STALE', ports.time.monotonic())
                    frame.status('REPLAN_READY_ARMING_GUARDS', reason='NEW_ROUTE_REQUIRES_FRESH_ADMISSION', limited_twist=[0.0, 0.0, 0.0])
                except Exception as replan_error:
                    frame.route_recovery.failed(replan_error, ports.time.monotonic())
                    frame.status('WAITING_FOR_ROUTE_CLEAR', reason=str(replan_error), limited_twist=[0.0, 0.0, 0.0])
            else:
                frame.status('WAITING_FOR_LOCALIZATION_RECHECK' if frame.pkt.get('status') == 'LOCALIZATION_RECHECK_REQUIRED' else 'WAITING_FOR_ROUTE_CLEAR', reason=frame.route_recovery.reason, limited_twist=[0.0, 0.0, 0.0])
            frame.wait_cycle(frame.started, wake_for_scan=True)
            return Flow.CONTINUE
    return Flow.NEXT
