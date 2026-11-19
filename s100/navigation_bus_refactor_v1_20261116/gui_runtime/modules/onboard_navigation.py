import sys as _bootstrap_sys
from pathlib import Path as _BootstrapPath
_bootstrap_sys.path.insert(0,'/home/sunrise/tianji_table_approach_dev/navigation_s100_v7/modules')
import paths as _production_paths
_bootstrap_sys.path.insert(0,str(_BootstrapPath(__file__).resolve().parent))
from execution_stages import route as route_stage, collision as collision_stage, admission as admission_stage
from execution_stages.contracts import Flow as StageFlow
'S100 navigator. Hardware bridge is lazy-started only by explicit manual launch.\n--preview validates the complete command chain without creating any ROS publisher.\n'
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import argparse, json, sys, time, math, threading, signal, subprocess, select, hashlib, resource, ctypes, collections
from pathlib import Path
import numpy as np
from paths import ROOT
from common import atomic_json, exclusive
from async_diagnostics import AsyncDiagnostics
from live_pose import read as read_live_pose
from controller_extension import ensure as ensure_controller_extension, healthy as controller_extension_healthy
from client import SensorStream
from onboard_state import measured_joints, readiness, model, state, feedback_health
from clearance_override import settings as override_settings
from navigation_control import ControlTransport
from follower import PreviewFollower
from execution_safety import posture_check, position_check, sweep_guard, trajectory_deviation, sweep_reference_drift
from extrinsics import transforms
from saved_map import base_transform
from chassis_proxy import replace_shapes, profile, measured_shapes
import targets
from sensor_bus import SensorBus
from guard_recovery import GuardRecovery, expired_deadlines, waitable_data_fault
from sensor_recovery import SensorRecovery
from posture_lease import revalidate as revalidate_posture, motion_weights, joint_shift_bound
from posture_lease import corner_shift
from motion_certificate import certify_motion, BASE_ADVANCE_S, FUTURE_HORIZON_S
from certification_recovery import classify as classify_pose_wait, POSE_RECHECK
from obstacle_wait import ObstacleWait, BLOCKING_REASONS
from execution_metrics import signature, summarize
from operator_session import OperatorSession
from planning_process import run as isolated_plan
from navigation_runtime_profile import RAW_SCAN_HZ, LOCALIZATION_MATCH_PERIOD_S, CONTROL_CHECK_PERIOD_S
import shutil
from execution_runtime.contracts import Progress, StatusIdentity, StatusInputs
from execution_runtime.resources import Resources
from execution_runtime.operator import OperatorChannel
from execution_runtime.status import StatusPublisher
from execution_runtime.motion import MotionLimits, NearSurfaceSpeed

class PlanningBlocked(Exception):
    pass

class MapGuard:

    def __init__(self, file):
        self.sequence = 0
        self.failed = False
        self.child = subprocess.Popen([str(ROOT / 'native/world_guard'), str(file)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)

    def check(self, local_pose, twist):
        if self.failed:
            return (False, 'MAP_GUARD_FAILED')
        self.sequence += 1
        (x, y, yaw) = local_pose
        (vx, vy, w) = twist
        poses = []
        for t in np.linspace(0, 0.5, 6):
            a = w * t
            (dx, dy) = (vx * t, vy * t) if abs(w) < 1e-08 else ((vx * math.sin(a) + vy * (math.cos(a) - 1)) / w, (vx * (1 - math.cos(a)) + vy * math.sin(a)) / w)
            poses.append([x + math.cos(yaw) * dx - math.sin(yaw) * dy, y + math.sin(yaw) * dx + math.cos(yaw) * dy, yaw + a])
        self.child.stdin.write(json.dumps(dict(id=self.sequence, poses=poses)) + '\n')
        self.child.stdin.flush()
        if not select.select([self.child.stdout], [], [], 0.4)[0]:
            self.failed = True
            return (False, 'MAP_GUARD_TIMEOUT')
        r = json.loads(self.child.stdout.readline())
        if r.get('id') != self.sequence:
            self.failed = True
            return (False, 'MAP_GUARD_RESPONSE_MISMATCH')
        return (True, 'OK') if r.get('clear') else (False, r.get('link') or r.get('error', 'MAP_COLLISION'))

    def close(self):
        self.child.stdin.close()
        try:
            self.child.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.child.terminate()
            self.child.wait(timeout=2)

def run(args):
    if getattr(args, 'enable_motion', False) and ((ROOT / 'ISOLATED_CONTRACT_TEST_ROOT').exists() or (Path(__file__).resolve().parents[1] / 'ISOLATED_CONTRACT_TEST_ROOT').exists()):
        raise ValueError('ISOLATED_ROOT_CANNOT_ENABLE_PHYSICAL_MOTION')
    motion_limits = MotionLimits.from_policy(json.loads((ROOT / 'config/motion_policy.json').read_text()))
    limited_twist = motion_limits.limit
    TRANSLATION_CAP_MPS = motion_limits.translation_cap_mps
    load_test = bool(getattr(args, 'load_test_no_motion', False))
    if load_test and args.enable_motion:
        raise ValueError('LOAD_TEST_CANNOT_ENABLE_MOTION')
    right_micro = bool(getattr(args, 'right_micro_preview', False))
    if right_micro and (args.enable_motion or args.max_travel_m or args.cable_precheck):
        raise ValueError('RIGHT_MICRO_PREVIEW_ONLY')
    micro_limit = 0.05 if right_micro else float(getattr(args, 'max_travel_m', 0.0))
    cable_precheck = bool(getattr(args, 'cable_precheck', False))
    requested_cap = getattr(args, 'translation_cap_mps', None)
    if requested_cap is not None and (micro_limit or cable_precheck or (not 0.005 <= requested_cap <= TRANSLATION_CAP_MPS)):
        raise ValueError('INVALID_TRANSLATION_SPEED_CAP')
    from near_body_cleanup import settings as cleanup_settings
    if (cable_precheck or (micro_limit and (not right_micro))) and (not cleanup_settings().get('micro_probe_enabled', False)):
        raise ValueError('MICRO_PROBE_DISABLED_SNAPSHOT_ONLY')
    if micro_limit and (not 0.02 <= micro_limit <= 0.05):
        raise ValueError('MICRO_TRAVEL_LIMIT_MUST_BE_2_TO_5_CM')
    if cable_precheck:
        micro_limit = 0.06
    resources = Resources(remove_bus=lambda folder: shutil.rmtree(folder, ignore_errors=True))
    owned = resources.bindings
    progress = Progress()
    micro = None
    last_micro_cloud = None
    wait_reasons = collections.Counter()
    last_wait_reason = None
    probe_deferral = None
    recovery = GuardRecovery()
    route_recovery = None
    active_plan_folder = None
    code_signature = signature(ROOT)
    execution_cpu_affinity = sorted(os.sched_getaffinity(0))
    obstacle_wait = ObstacleWait()
    sensor_recovery = SensorRecovery()
    controller_extension_state = {}
    execution_profile = None
    m = None
    lock = exclusive(ROOT / 'runtime/navigation_execution.lock')
    stop = threading.Event()
    frames = {}
    mutex = threading.Lock()
    error = []
    record = None
    folder = None
    began = time.monotonic()
    relay_health = {}
    last_collision_capture = 0.0
    lease = OperatorSession(args.session, stop, began)

    def halt(signum, *_):
        lease.fail('PROCESS_SIGNAL_' + signal.Signals(signum).name, time.monotonic())
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, halt)
    operator_channel = OperatorChannel(lease, stop, sys.stdin, time.monotonic, threading.Thread)
    operator_channel.start()

    def filter_timing(worker, now):
        from prepared_frames import latency_summary
        return latency_summary(worker.latest, now)
    identity = StatusIdentity(args.session, os.getpid(), args.enable_motion, load_test, requested_cap, micro_limit, args.operator_estop_ready, code_signature, tuple(execution_cpu_affinity))
    reporter = StatusPublisher(ROOT, identity, stop.is_set, time.monotonic, time.time, atomic_json, AsyncDiagnostics, lambda r: print('STATUS ' + json.dumps(r, ensure_ascii=False), flush=True), lease.fail, feedback_health, lease.summary, filter_timing, summarize)
    diagnostic_io = reporter.diagnostic_io
    console_io = reporter.console_io
    history = reporter.history

    def status(phase, **extra):
        inputs = StatusInputs(owned.transport, owned.frame_worker, folder, relay_health, execution_profile if owned.frame_worker else None, route_recovery.summary() if route_recovery else None, active_plan_folder, controller_extension_state)
        reporter.emit(phase, inputs, **extra)
    operator_age = operator_channel.age

    def refresh_output_posture(reference):
        return revalidate_posture(reference, measured_joints(), record['saved_pose'], m.sha256, lambda q: measured_shapes(m, q))
    log = (ROOT / 'runtime/navigation_tracking.log').open('w')
    try:
        if args.enable_motion and (not args.operator_estop_ready):
            raise ValueError('OPERATOR_ESTOP_DECLARATION_REQUIRED')
        if args.enable_motion:
            status('PREPARING_CONTROLLER_EXTENSION')
            controller_extension_state = ensure_controller_extension()
            if stop.is_set() or operator_age() > 0.5:
                raise ValueError('OPERATOR_LINK_LOST_BEFORE_PLANNING')
        status('LOCALIZING_AND_PLANNING')

        def start_frame_workers(geometry_model, profile_config):
            from prepared_frames import PreparedFrames
            live_filter = profile_config['live_filter']
            owned.frame_worker = PreparedFrames(
                geometry_model, owned.sensor_bus.folder,
                conservative_fast=live_filter, refine_near_hits=live_filter,
                refine_radius_m=profile_config['refine_radius_m'],
            )

        def start_perception(plan_folder, localization):
            nonlocal execution_profile, m
            if stop.is_set() or operator_age() > 0.5:
                raise ValueError('OPERATOR_LINK_LOST_BEFORE_START')

            def child_death():
                ctypes.CDLL(None).prctl(1, signal.SIGINT)
            owned.sensor_bus = SensorBus(Path('/dev/shm') / ('tianji_navigation_sensors_' + str(os.getpid()) + '_' + args.session))
            from resident_localization import capture_stream
            owned.stream = capture_stream()
            if owned.stream is None:
                owned.stream = SensorStream(localization=True, trace_path=plan_folder / 'motor_trace.jsonl', sensor_hz=RAW_SCAN_HZ)
            else:
                from client import motor_trace_environment
                trace_env = motor_trace_environment()
                resident_trace = subprocess.Popen([sys.executable, str(ROOT / 'modules/motor_trace_worker.py'), '--output', str(plan_folder / 'motor_trace.jsonl')], stdout=log, stderr=log, env=trace_env, preexec_fn=child_death)
                owned.resident_motor_trace[0] = resident_trace

            def receive():
                try:
                    while not stop.is_set():
                        (h, p) = owned.stream.receive()
                        received = time.monotonic()
                        owned.sensor_bus.publish(h, p, received)
                        if h.get('kind') == 'sensor_status':
                            relay_health.clear()
                            relay_health.update(h)
                        if h.get('kind') == 'frame' and p is not None:
                            if h['boot_id'] != localization['source_boot_id'] or h['epoch'] != localization['odometry_epoch'] or h['calibration_sha256'] != localization['calibration_sha256']:
                                raise ValueError('LIDAR_FRAME_CHANGED')
                            with mutex:
                                frames[h['sensor']] = (h, p, received)
                except Exception as exc:
                    if not stop.is_set():
                        error.append(str(exc))
            owned.reader = threading.Thread(target=receive, daemon=True)
            owned.reader.start()
            if not micro_limit and not cable_precheck:
                # The ordinary filter policy is independent of the selected path.
                # Warm read-only workers while the immutable planning scene is built.
                from navigation_runtime_profile import filter_profile
                execution_profile = filter_profile(
                    targets.scene_id(), args.goal, None, False, None,
                    requested_cap, TRANSLATION_CAP_MPS, BASE_ADVANCE_S,
                )
                m = model()
                start_frame_workers(m, execution_profile)
        from navigation import run as create_plan
        with exclusive(ROOT / 'runtime/navigation.lock'):
            if cable_precheck:
                create_plan(args.goal, True, cable_precheck=True, on_localized=start_perception)
            else:
                create_plan(args.goal, True, on_localized=start_perception, right_micro=right_micro, reviewed_map_cleanup=not micro_limit)
        folder = Path(json.loads((ROOT / 'runtime/latest_plan.json').read_text())['folder'])
        plan = json.loads((folder / 'planning/plan.json').read_text())
        record = json.loads((folder / 'scene/frozen_input.json').read_text())
        active_plan_folder = folder
        if stop.is_set() or operator_age() > 0.5:
            raise ValueError('OPERATOR_LINK_LOST_BEFORE_START')
        if not plan.get('selected'):
            while not micro_limit and (not cable_precheck) and (not stop.is_set()):
                status('WAITING_FOR_ROUTE_CLEAR', reason='INITIAL_ROUTE_BLOCKED', limited_twist=[0.0, 0.0, 0.0], planning_attempts=plan.get('attempts', []))
                retry_at = time.monotonic() + 5.0
                while not stop.is_set() and time.monotonic() < retry_at:
                    if operator_age() > 0.5:
                        raise ValueError('OPERATOR_LINK_LOST_WHILE_ROUTE_BLOCKED')
                    if not args.enable_motion and args.duration and (time.monotonic() - began >= args.duration):
                        progress.phase = 'TEST_COMPLETE'
                        progress.final_details.update(reason='STATIC_BLOCKED_ROUTE_HOLD_VALIDATED', planning_attempts=plan.get('attempts', []))
                        return 0
                    stop.wait(0.1)
                if stop.is_set():
                    progress.phase = 'STOP_REQUESTED'
                    return 0
                if not targets.unchanged(plan['target']):
                    raise ValueError('TARGET_CHANGED_REPLAN_REQUIRED')
                status('REPLANNING_STOPPED', reason='INITIAL_ROUTE_RETRY', limited_twist=[0.0, 0.0, 0.0])
                with exclusive(ROOT / 'runtime/navigation.lock'):
                    isolated_plan(args.goal, stop, lease.require, folder / 'replanning')
                active_plan_folder = Path(json.loads((ROOT / 'runtime/latest_plan.json').read_text())['folder'])
                plan = json.loads((active_plan_folder / 'planning/plan.json').read_text())
                record = json.loads((active_plan_folder / 'scene/frozen_input.json').read_text())
                if plan.get('selected'):
                    break
            if not plan.get('selected'):
                if stop.is_set():
                    progress.phase = 'STOP_REQUESTED'
                    return 0
                attempts = plan.get('attempts', [])
                reasons = [r.get('reason', 'UNKNOWN') for r in attempts]
                detail = '起步候选动作全部被碰撞/未知区域拒绝' if attempts and all((r.get('expanded_nodes', 99) <= 1 for r in attempts)) else '没有通过三维检查的路线'
                raise PlanningBlocked(detail + '；' + ','.join(reasons))
        if not targets.unchanged(plan['target']):
            raise ValueError('TARGET_CHANGED_REPLAN_REQUIRED')
        if m is None:
            m = model()
        elif record['model_sha256'] != m.sha256:
            raise ValueError('FILTER_MODEL_CHANGED_DURING_PLANNING')
        posture_weights = motion_weights(m.geometry)
        cfg = json.loads((ROOT / 'config/motion_policy.json').read_text())
        follower = PreviewFollower(plan, cfg)
        matrices = transforms()
        radius = max((np.linalg.norm(np.asarray(s['world_corners'])[:, :2], axis=1).max() for s in measured_shapes(m, record['saved_pose']['joint_positions_rad'])))
        guard_world = active_plan_folder / 'scene/candidate_world.json' if cable_precheck else active_plan_folder / 'scene/immutable_world.json'
        owned.map_guard = MapGuard(guard_world)
        map_file = Path(record['map_file']).parent / 'voxels.npz'
        last_hash = 0.0
        inv_start = np.linalg.inv(base_transform(plan['start_map_pose']))
        if micro_limit:
            from route_micro_probe import RouteMicroProbe
            if cable_precheck:
                from cable_precheck import check_prefix
                check_prefix(guard_world, plan['selected']['path'])
                from probe_cable_deferral import ProbeCableDeferral
                probe_deferral = ProbeCableDeferral()
            micro = RouteMicroProbe(plan['selected']['path'], micro_limit, target_m=0.052 if cable_precheck else None)

        def child_death():
            ctypes.CDLL(None).prctl(1, signal.SIGINT)
        tracker_started_wall = time.time()
        from resident_localization import ensure as ensure_resident
        owned.tracker = ensure_resident(active_plan_folder / 'localization/localization.json')
        from navigation_runtime_profile import filter_profile
        planned_profile = filter_profile(targets.scene_id(), args.goal, micro, cable_precheck, plan['selected']['profile'], requested_cap, TRANSLATION_CAP_MPS, BASE_ADVANCE_S)
        if owned.frame_worker and planned_profile != execution_profile:
            raise ValueError('FILTER_PROFILE_CHANGED_DURING_PLANNING')
        execution_profile = planned_profile
        if owned.frame_worker is None:
            start_frame_workers(m, execution_profile)
        office_live_filter = execution_profile['live_filter']
        office_slow_translation = execution_profile['directional_translation']
        translation_cap = execution_profile['translation_cap_mps']
        refine_radius = execution_profile['refine_radius_m']
        if office_slow_translation:
            sensor_recovery = SensorRecovery(clear_duration_s=0.3, minimum_pairs=2)

        near_surface_speed = NearSurfaceSpeed()

        def limit_live_twist(requested, cloud_age):
            (vx, vy, w) = limited_twist(requested, cloud_age, radius)
            speed = math.hypot(vx, vy)
            if abs(w) < 1e-08 and speed > translation_cap:
                scale = translation_cap / speed
                vx *= scale
                vy *= scale
            result = near_surface_speed.apply([vx, vy, w], prepared, shapes, cloud_age, time.monotonic())
            execution_profile['near_surface_speed'] = dict(near_surface_speed.report)
            return result

        def wait_cycle(started, wake_for_scan=False):
            deadline = started + CONTROL_CHECK_PERIOD_S
            if owned.frame_worker is None or not (office_slow_translation and wake_for_scan):
                stop.wait(max(0.0, deadline - time.monotonic()))
                return
            stop.wait(max(0.0, min(deadline, started + 0.1) - time.monotonic()))
            while not stop.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0 or owned.frame_worker.has_unread():
                    return
                stop.wait(min(0.025, remaining))
        warm = time.monotonic()
        if micro and (not right_micro):
            owned.observer_log = (folder / 'micro_motion_observer.log').open('w')
            observer_args = [sys.executable, str(ROOT / 'modules/observe_self_motion.py'), '--duration', '50', '--micro', '--sensor-bus', str(owned.sensor_bus.folder)]
            if cable_precheck:
                observer_args += ['--features', str(folder / 'probe_initial_wire_features.npz'), '--features-only', '--wait-features']
            owned.observer_child = subprocess.Popen(observer_args, stdout=owned.observer_log, stderr=owned.observer_log, preexec_fn=child_death)
        if not args.enable_motion:
            owned.transport = ControlTransport(False, posture_refresh=refresh_output_posture, load_test_no_motion=load_test)
        from route_recovery import RouteRecovery
        route_recovery = RouteRecovery(record['localization'], tolerance=cfg.get('localization_anchor_tolerance'))
        while not stop.is_set():
            started = time.monotonic()
            reason = 'WAIT_SENSORS'
            guard = None
            twist = [0.0, 0.0, 0.0]
            pose_ok = False
            geometric = False
            collision_debug = {}
            posture_reference = None
            if args.duration and time.monotonic() - warm >= args.duration:
                if right_micro:
                    progress.phase = 'STOP_GUARDS_NOT_READY'
                else:
                    progress.phase = 'STOP_TEST_DURATION' if args.enable_motion else 'TEST_COMPLETE'
                    progress.final_details['reason'] = 'ONE_SHOT_TEST_DURATION_REACHED' if args.enable_motion else 'PREVIEW_DURATION_REACHED'
                break
            if args.enable_motion and (not controller_extension_healthy(controller_extension_state)):
                progress.phase = 'STOP_CONTROLLER_EXTENSION'
                progress.final_details['reason'] = 'CONTROLLER_EXTENSION_PROCESS_LOST'
                break
            if operator_age() > 0.5:
                progress.phase = 'STOP_OPERATOR_LINK'
                break
            if error:
                progress.phase = 'STOP_SENSOR_LINK'
                progress.final_details = dict(reason='SENSOR_LINK_FAILED', sensor_errors=list(error))
                break
            if owned.frame_worker and owned.frame_worker.error:
                progress.phase = 'STOP_FILTER_WORKER'
                progress.final_details = dict(reason=owned.frame_worker.error)
                break
            if owned.observer_child and owned.observer_child.poll() is not None:
                progress.phase = 'STOP_RAW_OBSERVER'
                break
            if owned.tracker.poll() is not None:
                progress.phase = 'STOP_LOCALIZATION_PROCESS'
                break
            if owned.transport and owned.transport.error:
                progress.phase = 'STOP_CONTROL_LINK'
                progress.final_details = dict(reason='CONTROL_LINK_FAILED', control_error=owned.transport.error)
                break
            if not targets.unchanged(plan['target']):
                progress.phase = 'STOP_TARGET_CHANGED'
                break
            if time.monotonic() - last_hash > 1:
                if hashlib.sha256(map_file.read_bytes()).hexdigest() != record['map_voxel_sha256']:
                    progress.phase = 'STOP_MAP_CHANGED'
                    break
                from manual_floor import settings as current_floor, fingerprint
                if fingerprint(current_floor(record['map_voxel_sha256'])) != fingerprint(record.get('manual_floor', dict(enabled=False, revision=0))):
                    progress.phase = 'STOP_MANUAL_FLOOR_CHANGED'
                    break
                last_hash = time.monotonic()
            try:
                pkt = read_live_pose()
                now = time.time()
                (pos, reason, pos_age) = position_check(pkt, record, now)
                _stage_route_state = route_stage.State.capture(locals())
                try:
                    _stage_route_flow = route_stage.execute(_stage_route_state, route_stage.Ports(MapGuard=MapGuard, Path=Path, PreviewFollower=PreviewFollower, ROOT=ROOT, base_transform=base_transform, exclusive=exclusive, isolated_plan=isolated_plan, json=json, measured_joints=measured_joints, measured_shapes=measured_shapes, np=np, posture_check=posture_check, read_live_pose=read_live_pose, time=time))
                finally:
                    if _stage_route_state.has('_'):
                        _ = _stage_route_state._
                    if _stage_route_state.has('active_plan_folder'):
                        active_plan_folder = _stage_route_state.active_plan_folder
                    if _stage_route_state.has('follower'):
                        follower = _stage_route_state.follower
                    if _stage_route_state.has('inv_start'):
                        inv_start = _stage_route_state.inv_start
                    if _stage_route_state.has('plan'):
                        plan = _stage_route_state.plan
                    if _stage_route_state.has('radius'):
                        radius = _stage_route_state.radius
                    if _stage_route_state.has('record'):
                        record = _stage_route_state.record
                if _stage_route_flow is StageFlow.CONTINUE:
                    continue
                if _stage_route_flow is StageFlow.BREAK:
                    break
                if pkt.get('updated_wall_time', 0) < tracker_started_wall:
                    pos = False
                    reason = 'LOCALIZATION_INITIALIZING'
                if pos:
                    T = np.array(pkt['T_map_base'])
                    pose = [T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0])]
                    if trajectory_deviation(pose, follower.path) > 0.12:
                        raise ValueError('PATH_DEVIATION_REPLAN_REQUIRED')
                    q = measured_joints()
                    (pose_ok, reason, pose_age) = posture_check(record['saved_pose'], q, m.sha256, time.time())
                    release = readiness()
                    if pose_ok and release['clearance_clear'] and release['shapemask_clear']:
                        suggested = micro.step(pkt['odometry_pose'], time.monotonic()) if micro else follower.step(pose, float(pkt['observation_time']), time.time(), plan['target']['revision'], True, True, odometry_pose=pkt['odometry_pose'], control_time=time.monotonic())
                        if suggested['state'] == 'MICRO_STOP':
                            if owned.transport:
                                owned.transport.zero()
                            progress.final_details = dict(reason=suggested['reason'], traversed_m=micro.traversed, commanded_m=micro.commanded, planned_prefix_m=micro.target_travel)
                            progress.phase = 'MICRO_STOPPED'
                            if suggested['reason'] in ['DISTANCE_BUDGET_REACHED', 'PREFIX_REACHED']:
                                displacement = float(np.linalg.norm(np.asarray(pkt['odometry_pose'])[:2] - micro.reference[:2])) if micro.reference is not None else 0.0
                                progress.final_details['measured_displacement_m'] = displacement
                                if cable_precheck and (not args.enable_motion or displacement < 0.05):
                                    progress.phase = 'PREVIEW_CABLE_PROBE_VALIDATED' if not args.enable_motion else 'STOP_CABLE_BASELINE_INSUFFICIENT'
                                    break
                                status('MICRO_STOPPED_SAMPLING', **progress.final_details)
                                sample_until = time.monotonic() + 12.0
                                while time.monotonic() < sample_until and (not stop.wait(0.2)) and (operator_age() <= 0.5):
                                    if owned.observer_child.poll() is not None:
                                        break
                                progress.phase = 'MICRO_TEST_COMPLETE'
                                if cable_precheck:
                                    observer_status = json.loads((ROOT / 'runtime/motion_observer_status.json').read_text())
                                    if stop.is_set() or operator_age() > 0.5 or observer_status.get('parent_pid') != os.getpid() or (time.time() - observer_status.get('updated_at', 0) > 2.0):
                                        raise ValueError('CABLE_EVIDENCE_OR_OPERATOR_STALE')
                                    evidence = json.loads((Path(observer_status['folder']) / 'evidence.json').read_text())
                                    if evidence.get('parent_pid') != os.getpid() or not 0 <= time.time() - evidence.get('updated_at', 0) <= 2.0:
                                        raise ValueError('CABLE_EVIDENCE_SESSION_MISMATCH')
                                    expected = (record['localization']['source_boot_id'], record['localization']['odometry_epoch'], record['localization']['calibration_sha256'])
                                    if tuple(evidence.get('source_identity', [])) != expected:
                                        raise ValueError('CABLE_EVIDENCE_FRAME_CHANGED')
                                    owned.observer_child.send_signal(signal.SIGINT)
                                    owned.observer_child.wait(timeout=5)
                                    capture_result = json.loads((Path(observer_status['folder']) / 'result.json').read_text())
                                    if not capture_result.get('valid'):
                                        raise ValueError('INVALID_RAW_CABLE_CAPTURE: ' + str(capture_result.get('reason')))
                                    from replay_probe_evidence import replay
                                    evidence = replay(observer_status['folder'], evidence['reference_joints'], m.sha256, expected, displacement)
                                    if stop.is_set() or operator_age() > 0.5:
                                        raise ValueError('OPERATOR_STOPPED_DURING_CABLE_JUDGEMENT')
                                    from motion_cables import publish
                                    learned = publish(evidence, m.sha256)
                                    fixed = sum((f['status'] == 'ENVIRONMENT_FIXED' for f in evidence.get('features', [])))
                                    initial_count = len(evidence.get('features', []))
                                    progress.final_details.update(confirmed_cable_features=learned['confirmed_count'], fixed_environment_features=fixed, initial_candidate_count=initial_count, next_action='REPLAN_REMAINDER_WITH_CONFIRMED_SELF_FILTER')
                                    progress.phase = 'CABLE_PRECHECK_FOUND_STATIC_OBSTACLE' if fixed else 'CABLE_PRECHECK_COMPLETE' if learned['confirmed_count'] or initial_count == 0 else 'CABLE_EVIDENCE_UNRESOLVED'
                                    if progress.phase != 'CABLE_PRECHECK_COMPLETE':
                                        progress.final_details['next_action'] = 'HOLD_POSITION'
                            break
                        shapes = measured_shapes(m, q['joint_positions_rad'])
                        (a, b, c) = record['ground_plane']
                        n = np.array([-a, -b, 1.0])
                        normal = T[:3, :3].T @ n
                        offset = n @ T[:3, 3] - c
                        floor = [-normal[0] / normal[2], -normal[1] / normal[2], -offset / normal[2]]
                        prepared = owned.frame_worker.snapshot()
                        copy = {sensor: data['frame'] for (sensor, data) in prepared.items()}
                        cloud_age = max((time.monotonic() - v[2] + float(v[0].get('source_age_s', 0)) for v in copy.values()), default=999.0)
                        twist = limit_live_twist(suggested['twist_preview'], cloud_age)
                        if office_slow_translation:
                            owned.frame_worker.request_refinement(twist, pkt['odometry_pose'])
                        cloud_checked_at = time.monotonic()
                        if cable_precheck:
                            (geometric, reason, cloud_age) = sweep_guard(copy, matrices, pkt['odometry_pose'], shapes, m.sha256, floor, twist, time.monotonic(), probe_deferral=probe_deferral, revalidation_reserve_m=0.005, diagnostics=collision_debug, prepared=prepared)
                        else:
                            (geometric, reason, cloud_age) = sweep_guard(copy, matrices, pkt['odometry_pose'], shapes, m.sha256, floor, twist, time.monotonic(), revalidation_reserve_m=0.005, diagnostics=collision_debug, prepared=prepared, motion_advance_s=BASE_ADVANCE_S if not micro else 0.0, directional_age_compensation=office_slow_translation)
                        if probe_deferral and (not probe_deferral.frozen) and probe_deferral.ready_to_freeze():
                            probe_deferral.freeze()
                            atomic_json(folder / 'probe_initial_wire_deferral.json', dict(**probe_deferral.summary(), points_body_m=probe_deferral.seeds.tolist(), reference_odometry_matrix=probe_deferral.reference.tolist()))
                            temporary = folder / 'probe_initial_wire_features.tmp'
                            with temporary.open('wb') as f:
                                np.savez(f, points_body=probe_deferral.seeds)
                            os.replace(temporary, folder / 'probe_initial_wire_features.npz')
                        if geometric and micro and (not right_micro):
                            observer_status = json.loads((ROOT / 'runtime/motion_observer_status.json').read_text())
                            if observer_status.get('parent_pid') != os.getpid() or not observer_status.get('ready') or (not 0 <= time.time() - observer_status.get('updated_at', 0.0) <= 2.0):
                                geometric = False
                                reason = 'WAIT_RAW_OBSERVER'
                        _stage_collision_state = collision_stage.State.capture(locals())
                        try:
                            _stage_collision_flow = collision_stage.execute(_stage_collision_state, collision_stage.Ports(BASE_ADVANCE_S=BASE_ADVANCE_S, FUTURE_HORIZON_S=FUTURE_HORIZON_S, base_transform=base_transform, certify_motion=certify_motion, corner_shift=corner_shift, joint_shift_bound=joint_shift_bound, math=math, measured_joints=measured_joints, measured_shapes=measured_shapes, np=np, override_settings=override_settings, position_check=position_check, posture_check=posture_check, read_live_pose=read_live_pose, state=state, sweep_guard=sweep_guard, time=time, trajectory_deviation=trajectory_deviation))
                        finally:
                            if _stage_collision_state.has('_'):
                                _ = _stage_collision_state._
                            if _stage_collision_state.has('certificate'):
                                certificate = _stage_collision_state.certificate
                            if _stage_collision_state.has('cloud_age'):
                                cloud_age = _stage_collision_state.cloud_age
                            if _stage_collision_state.has('cloud_checked_at'):
                                cloud_checked_at = _stage_collision_state.cloud_checked_at
                            if _stage_collision_state.has('copy'):
                                copy = _stage_collision_state.copy
                            if _stage_collision_state.has('floor'):
                                floor = _stage_collision_state.floor
                            if _stage_collision_state.has('geometric'):
                                geometric = _stage_collision_state.geometric
                            if _stage_collision_state.has('guard'):
                                guard = _stage_collision_state.guard
                            if _stage_collision_state.has('new_pose'):
                                new_pose = _stage_collision_state.new_pose
                            if _stage_collision_state.has('normal'):
                                normal = _stage_collision_state.normal
                            if _stage_collision_state.has('offset'):
                                offset = _stage_collision_state.offset
                            if _stage_collision_state.has('pkt'):
                                pkt = _stage_collision_state.pkt
                            if _stage_collision_state.has('posture_reference'):
                                posture_reference = _stage_collision_state.posture_reference
                            if _stage_collision_state.has('prepared'):
                                prepared = _stage_collision_state.prepared
                            if _stage_collision_state.has('q'):
                                q = _stage_collision_state.q
                            if _stage_collision_state.has('reason'):
                                reason = _stage_collision_state.reason
                            if _stage_collision_state.has('shapes'):
                                shapes = _stage_collision_state.shapes
                            if _stage_collision_state.has('twist'):
                                twist = _stage_collision_state.twist
                        if _stage_collision_flow is StageFlow.CONTINUE:
                            continue
                        if _stage_collision_flow is StageFlow.BREAK:
                            break
                    elif pose_ok:
                        reason = 'CLEARANCE_OR_SHAPEMASK_NOT_CLEAR'
            except Exception as exc:
                reason = str(exc)
            _stage_admission_state = admission_stage.State.capture(locals())
            try:
                _stage_admission_flow = admission_stage.execute(_stage_admission_state, admission_stage.Ports(BLOCKING_REASONS=BLOCKING_REASONS, CONTROL_CHECK_PERIOD_S=CONTROL_CHECK_PERIOD_S, ControlTransport=ControlTransport, POSE_RECHECK=POSE_RECHECK, classify_pose_wait=classify_pose_wait, expired_deadlines=expired_deadlines, measured_joints=measured_joints, read_live_pose=read_live_pose, time=time, waitable_data_fault=waitable_data_fault))
            finally:
                if _stage_admission_state.has('last_collision_capture'):
                    last_collision_capture = _stage_admission_state.last_collision_capture
                if _stage_admission_state.has('last_micro_cloud'):
                    last_micro_cloud = _stage_admission_state.last_micro_cloud
                if _stage_admission_state.has('last_wait_reason'):
                    last_wait_reason = _stage_admission_state.last_wait_reason
                if _stage_admission_state.has('reason'):
                    reason = _stage_admission_state.reason
                if _stage_admission_state.has('suggested'):
                    suggested = _stage_admission_state.suggested
            if _stage_admission_flow is StageFlow.CONTINUE:
                continue
            if _stage_admission_flow is StageFlow.BREAK:
                break
        if progress.phase == 'PREPARING':
            progress.phase = 'STOPPED_BY_OPERATOR' if lease.reason == 'OPERATOR_STOP_COMMAND' else 'STOP_SESSION_LOST' if lease.reason else 'STOP_REQUESTED'
            if lease.reason:
                progress.final_details.update(reason=lease.reason)
    except PlanningBlocked as exc:
        progress.phase = 'PLAN_BLOCKED'
        progress.final_details = dict(reason=str(exc), error_code='NO_CHECKED_ROUTE')
        status(progress.phase, **progress.final_details)
    except Exception as exc:
        progress.phase = 'STOP_ERROR'
        progress.final_details = dict(error=str(exc))
        status(progress.phase, **progress.final_details)
    finally:
        if lease.reason and progress.phase in ('STOP_REQUESTED', 'STOPPED_BY_OPERATOR', 'STOP_SESSION_LOST'):
            progress.phase = 'STOPPED_BY_OPERATOR' if lease.reason == 'OPERATOR_STOP_COMMAND' else 'STOP_SESSION_LOST'
            progress.final_details['reason'] = lease.reason
        if probe_deferral:
            progress.final_details['probe_wire_deferral'] = probe_deferral.summary()
        if progress.phase == 'STOP_GUARDS_NOT_READY':
            progress.final_details.update(reason=last_wait_reason, error_code='GUARDS_NOT_READY', blocking_checks=dict(wait_reasons))
        stop.set()
        cleanup = resources.cleanup
        coordinate_task = bool(os.environ.get('TIANJI_NAV_COORDINATE_TARGET_FILE'))
        arrival_pending = progress.phase in ('ARRIVED', 'GOAL_CONFIRMING_STOP')
        arrival_pose = None
        if coordinate_task and arrival_pending:
            from navigation_stop_evidence import arrival_pose_before_parking
            arrival_pose = arrival_pose_before_parking(progress.final_details.get('pre_parking_xyt'), plan['target']['pose'], progress.final_details.get('final_pose_observation_time'), progress.final_details.get('arrival_verified_at'))
        stop_requested_at = time.time()
        control_stopped = resources.stop_control()
        if not control_stopped:
            progress.phase = 'STOP_CONTROL_SHUTDOWN'
            progress.final_details['reason'] = 'CONTROL_SHUTDOWN_FAILED'
        if coordinate_task and args.enable_motion and folder and (owned.transport or arrival_pending):
            from navigation_stop_evidence import wait_trace, arrival_verdict
            progress.final_details['navigation_stop_evidence'] = wait_trace(folder / 'motor_trace.jsonl', stop_requested_at)
            if arrival_pending:
                progress.final_details['final_pose_evidence'] = arrival_pose
                progress.final_details['arrival_pose_basis'] = 'BEFORE_PARKING'
                progress.final_details['post_parking_pose_evaluated'] = False
                if arrival_pose.get('valid'):
                    progress.final_details.update(final_xyt=arrival_pose['final_xyt'], goal_error=arrival_pose['goal_error'], final_pose_observation_time=arrival_pose['observation_time'])
                verdict = arrival_verdict(arrival_pose, progress.final_details['navigation_stop_evidence'], control_stopped)
                progress.phase = verdict
                progress.final_details['reason'] = 'GOAL_TOLERANCE_REACHED' if verdict == 'ARRIVED' else verdict
                if verdict != 'ARRIVED':progress.final_details['error_code'] = verdict
        if coordinate_task and 'navigation_stop_evidence' not in progress.final_details:
            progress.final_details['navigation_stop_evidence'] = dict(confirmed=False, reason='NO_FRESH_PHYSICAL_STOP_FEEDBACK')
        resources.stop_observers()
        if resources.errors:
            progress.final_details['cleanup_errors'] = resources.errors
        cleanup('tracking_log', log.close)
        status(progress.phase, finished=True, cleanup_pending=False, wall_s=time.monotonic() - began, **progress.final_details)
        console_io.close()
        drained = diagnostic_io.close()
        if not drained:
            print('DIAGNOSTIC_IO_WARNING final state=' + progress.phase + '; writer drain timed out; chassis output already stopped', flush=True)
        if folder:
            atomic_json(folder / 'navigation_execution_history.json', history)
            atomic_json(folder / 'navigation_performance_summary.json', summarize(history))
        lock.close()
    return 0
if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--goal', type=int, required=True, choices=[0, 1, 2, 3])
    p.add_argument('--session', required=True)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--enable-motion', action='store_true')
    mode.add_argument('--preview', action='store_true')
    p.add_argument('--operator-estop-ready', action='store_true')
    p.add_argument('--duration', type=float, default=0.0)
    p.add_argument('--max-travel-m', type=float, default=0.0)
    p.add_argument('--cable-precheck', action='store_true')
    p.add_argument('--right-micro-preview', action='store_true')
    p.add_argument('--translation-cap-mps', type=float)
    p.add_argument('--load-test-no-motion', action='store_true')
    a = p.parse_args()
    sys.exit(run(a))
