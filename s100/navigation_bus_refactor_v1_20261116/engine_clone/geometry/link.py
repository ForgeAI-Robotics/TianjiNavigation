"""Jetson/S100 advisory protocol. No actuator, SDK, ROS or network dependencies.

All times in evidence are UNIX seconds; receive watchdogs use local monotonic
time. A camera CLEAR is a current-pose estimate, never whole-body permission.
"""
import copy
import math
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / 'vendor'))
from table_approach import upper_forward_constraint, combined_limit

ARM = tuple(f'Joint{i}_{s}' for s in 'LR' for i in range(1, 8))
AUX = ('head_pitch_joint', 'left_robotiq_85_left_knuckle_joint',
       'right_robotiq_85_left_knuckle_joint')
JOINTS = ARM + AUX
POSE_TOLERANCE = math.radians(.5)
SCOPE = 'fixed measured pose; pure +X translation; observed table/camera geometry only'


def number(v):
    return type(v) in (int, float) and math.isfinite(v)


def vector(v, n):
    return isinstance(v, list) and len(v) == n and all(number(x) for x in v)


def fresh(now, stamp, ttl):
    return number(now) and number(stamp) and -.1 <= now - stamp <= ttl


class PoseHistory:
    """Independent per-joint histories; partial JointState messages never fill zeros."""
    def __init__(self):
        self.samples = {name: deque(maxlen=512) for name in JOINTS}

    def add(self, names, values, stamp):
        if (not number(stamp) or stamp <= 0 or len(names) != len(values)
                or len(set(names)) != len(names) or not all(number(v) for v in values)):
            return False
        selected = [(n, v) for n, v in zip(names, values) if n in self.samples]
        if any(self.samples[n] and stamp <= self.samples[n][-1][0] for n, _ in selected):
            return False
        for name, value in selected:
            self.samples[name].append((stamp, value))
        return True

    def envelope(self, start, now):
        result = {}
        if not number(start) or not fresh(now, start, 1.2):
            return result
        for name, history in self.samples.items():
            # Require a real sample on/before the oldest camera observation.
            previous = [p for p in history if p[0] <= start]
            if not previous:
                continue
            rows = [previous[-1]] + [p for p in history if start < p[0] <= now]
            stamps = [p[0] for p in rows]
            if (start - stamps[0] > .15 or now - stamps[-1] > .25
                    or any(b - a > .25 for a, b in zip(stamps, stamps[1:]))):
                continue
            values = [p[1] for p in rows]
            result[name] = dict(position=values[-1], stamp=stamps[-1],
                                minimum=min(values), maximum=max(values),
                                covered_from=stamps[0], max_sample_gap_s=max(
                                    [b-a for a, b in zip(stamps, stamps[1:])] or [0.]))
        return result

    def current(self, now):
        # Model animation does not depend on whether cameras/clearance run.
        return {name: dict(position=history[-1][1], stamp=history[-1][0])
                for name, history in self.samples.items()
                if history and fresh(now, history[-1][0], .25)}


class JetsonLink:
    def __init__(self, whole_sha, perception_sha, session_id):
        if not session_id:
            raise ValueError('session_id required')
        self.whole_sha, self.perception_sha = whole_sha, perception_sha
        self.session_id, self.sequence = session_id, 0
        self.history = PoseHistory()

    def packet(self, clearance, now):
        self.sequence += 1
        clearance = copy.deepcopy(clearance) if isinstance(clearance, dict) else {}
        times = [clearance.get('evaluation_time')]
        cameras = clearance.get('cameras')
        times += [p.get('observation_time') for p in (cameras if isinstance(cameras, list) else [])
                  if isinstance(p, dict)]
        start = min(times) if times and all(number(t) for t in times) else None
        return dict(schema_version=2, session_id=self.session_id,
                    sequence=self.sequence, generated_at=now,
                    whole_model_sha256=self.whole_sha,
                    perception_model_sha256=self.perception_sha,
                    joint_state=self.history.current(now),
                    motion_history=self.history.envelope(now-.3,now),
                    pose_interval_start=start, pose=self.history.envelope(start, now),
                    clearance=clearance, advisory_only=True)


def pose_check(packet, now):
    pose = packet.get('pose')
    d = packet.get('clearance', {})
    start = packet.get('pose_interval_start')
    times = [d.get('evaluation_time')] + [v.get('observation_time')
                for v in d.get('cameras', [])]
    if (not isinstance(pose, dict) or not all(number(t) for t in times)
            or not number(start) or start > min(times) or not fresh(now, start, 1.2)):
        return ['POSE_OBSERVATION_INTERVAL_INVALID']
    if set(pose) != set(JOINTS):
        return ['FULL_POSE_HISTORY_INCOMPLETE']
    for name in JOINTS:
        v = pose[name]
        if (not isinstance(v, dict) or not all(number(v.get(k)) for k in
                ('position', 'stamp', 'minimum', 'maximum', 'covered_from', 'max_sample_gap_s'))
                or not fresh(now, v['stamp'], .25)
                or not start-.15 <= v['covered_from'] <= start
                or not 0 <= v['max_sample_gap_s'] <= .25
                or v['stamp'] < v['covered_from']
                or not v['minimum'] <= v['position'] <= v['maximum']
                or v['maximum']-v['minimum'] > POSE_TOLERANCE):
            return ['POSE_MOVED_OR_HISTORY_EXPIRED:' + name]
    q = d.get('evaluation_joints_rad')
    if not vector(q, 14) or any(abs(pose[n]['position']-v) > POSE_TOLERANCE
                               for n, v in zip(ARM, q)):
        return ['ARM_POSE_DIFFERS_FROM_CAMERA_EVALUATION']
    return []


class S100Link:
    def __init__(self, whole_sha, perception_sha):
        self.whole_sha, self.perception_sha = whole_sha, perception_sha
        self.packet = None
        self.received = None
        self.session = None
        self.retired = set()
        self.sequence = -1
        self.reject_reason = 'NO_JETSON_PACKET'
        self.stop_latched = False
        self.odometry = deque(maxlen=256)

    def observe_odometry(self, odom):
        if (not isinstance(odom, dict) or not number(odom.get('stamp'))
                or not vector(odom.get('xy_yaw'), 3) or not vector(odom.get('velocity'), 3)
                or self.odometry and odom['stamp'] <= self.odometry[-1]['stamp']):
            return False
        self.odometry.append(copy.deepcopy(odom))
        return True

    def model_joint_state(self, now, monotonic_now):
        """Measured model updates remain available with cameras stopped."""
        packet = self.packet or {}
        state = packet.get('joint_state')
        if (self.received is None or not number(monotonic_now)
                or not 0 <= monotonic_now-self.received <= .3
                or not fresh(now, packet.get('generated_at'), .3)
                or packet.get('whole_model_sha256') != self.whole_sha
                or not isinstance(state, dict)):
            return {}
        return {name: copy.deepcopy(value) for name, value in state.items()
                if name in JOINTS and isinstance(value, dict)
                and number(value.get('position')) and fresh(now, value.get('stamp'), .25)}

    def camera_advance(self, start, now, current):
        previous = [p for p in self.odometry if p['stamp'] <= start]
        if not previous or start-previous[-1]['stamp'] > .15:
            raise ValueError('ODOMETRY_CAMERA_HISTORY_MISSING')
        rows = [previous[-1]] + [p for p in self.odometry if start < p['stamp'] <= now]
        if not rows or current['stamp'] != rows[-1]['stamp'] or any(
                b['stamp']-a['stamp'] > .2 for a, b in zip(rows, rows[1:])):
            raise ValueError('ODOMETRY_CAMERA_HISTORY_GAP')
        origin = rows[0]['xy_yaw']
        max_forward = 0.
        for row in rows:
            p, v = row['xy_yaw'], row['velocity']
            dx, dy = p[0]-origin[0], p[1]-origin[1]
            forward = math.cos(origin[2])*dx+math.sin(origin[2])*dy
            lateral = -math.sin(origin[2])*dx+math.cos(origin[2])*dy
            yaw = math.atan2(math.sin(p[2]-origin[2]), math.cos(p[2]-origin[2]))
            if (forward < -.002 or abs(lateral) > .005 or abs(yaw) > .01
                    or v[0] < -.001 or abs(v[1]) > .005 or abs(v[2]) > .02):
                raise ValueError('CAMERA_HISTORY_CONTAINS_UNSUPPORTED_MOTION')
            max_forward = max(max_forward, forward)
        return max_forward

    def receive(self, packet, monotonic_now):
        """Rejected/duplicate/out-of-order packets do not refresh the watchdog."""
        try:
            if not isinstance(packet, dict) or packet.get('schema_version') != 2:
                raise ValueError('UNSUPPORTED_LINK_SCHEMA')
            session, sequence = packet.get('session_id'), packet.get('sequence')
            if (not isinstance(session, str) or not 1 <= len(session) <= 128
                    or type(sequence) is not int or sequence < 1 or not number(monotonic_now)):
                raise ValueError('INVALID_SESSION_OR_SEQUENCE')
            if session in self.retired or session == self.session and sequence <= self.sequence:
                raise ValueError('OLD_OR_DUPLICATE_PACKET')
            # Validate structure before changing the accepted session.
            if not isinstance(packet.get('clearance'), dict) or not isinstance(packet.get('pose'), dict):
                raise ValueError('INVALID_LINK_PAYLOAD')
            if packet.get('advisory_only') is not True:
                raise ValueError('ADVISORY_CONTRACT_REQUIRED')
            if self.session is not None and session != self.session:
                self.retired.add(self.session)
            self.session, self.sequence = session, sequence
            self.packet, self.received = copy.deepcopy(packet), monotonic_now
            self.reject_reason = ''
            return True
        except (TypeError, ValueError) as exc:
            self.reject_reason = str(exc)
            return False

    def evaluate(self, now, monotonic_now, scene=None, odom=None):
        reasons = []
        upper = dict(valid=False, forward_remaining_m=0.)
        table = dict(valid=False, forward_remaining_m=0.)
        velocity = 0.
        packet = self.packet
        try:
            if packet is None:
                raise ValueError('NO_JETSON_PACKET')
            if not number(monotonic_now) or not 0 <= monotonic_now-self.received <= .3:
                reasons.append('JETSON_TRANSPORT_TIMEOUT')
            if not fresh(now, packet.get('generated_at'), .3):
                reasons.append('JETSON_CLOCK_SKEW_OR_PACKET_EXPIRED')
            if (packet.get('whole_model_sha256') != self.whole_sha
                    or packet.get('perception_model_sha256') != self.perception_sha
                    or packet['clearance'].get('model_sha256') != self.perception_sha):
                reasons.append('MODEL_CONTRACT_MISMATCH')
            reasons += pose_check(packet, now)
            data = packet['clearance']
            if data.get('stop_required') is True or data.get('state') == 2:
                self.stop_latched = True
            # Strict JSON numeric/normal validation before legacy geometry helper.
            cameras = data.get('cameras')
            if not isinstance(cameras, list) or len(cameras) != 3:
                raise ValueError('CAMERA_SET_INVALID')
            for camera in cameras:
                n = camera.get('plane_normal_chassis')
                if (not vector(n, 3) or abs(sum(x*x for x in n)-1.) > .02
                        or not all(number(camera.get(k)) for k in
                            ('raw_gap_m', 'conservative_gap_m', 'margin_m', 'plane_offset_chassis_m'))):
                    raise ValueError('CAMERA_GEOMETRY_INVALID')
            if not reasons:
                upper = upper_forward_constraint(data, now, monotonic_now-self.received,
                           True, True, [packet['pose'][n]['position'] for n in ARM])
                if not upper['valid']:
                    reasons += upper['reasons']
            if not reasons:
                table, velocity = self.table_constraint(scene, odom, packet, now)
                advance = self.camera_advance(packet['pose_interval_start'], now, odom)
                upper['camera_age_forward_travel_m'] = advance
                upper['forward_remaining_m'] = max(0., upper['forward_remaining_m']-advance)
        except (TypeError, ValueError, KeyError, IndexError, AttributeError, OverflowError) as exc:
            reasons.append(str(exc) or 'MALFORMED_LINK_INPUT')
        limit = combined_limit(table, upper, velocity_x=velocity)
        reasons = list(dict.fromkeys(reasons + limit['reasons']))
        if not reasons and not limit['forward_inhibit']:
            self.stop_latched = False
        inhibit = bool(reasons or limit['forward_inhibit'] or self.stop_latched)
        return dict(schema_version=2, evaluated_at=now, state=(
                    'BLOCKED' if self.stop_latched or table.get('table_collision_risk')
                    else 'UNKNOWN' if reasons else 'LIMIT_REACHED' if inhibit else 'FORWARD_ESTIMATE'),
                    forward_inhibit=inhibit, stop_request=self.stop_latched,
                    max_forward_speed_mps=0. if inhibit else limit['max_forward_speed_mps'],
                    forward_remaining_m=limit['forward_remaining_m'],
                    reasons=reasons, upper=upper, table=table,
                    jetson_session=self.session, acknowledged_sequence=self.sequence,
                    last_rejected_packet=self.reject_reason,
                    advisory_only=True, command_path_connected=False,
                    hardware_stop_connected=False, general_navigation_permission=False,
                    scope=SCOPE)

    def table_constraint(self, scene, odom, packet, now):
        if not isinstance(scene, dict) or scene.get('valid') is not True:
            raise ValueError('NO_VALID_LIDAR_TABLE_CONSTRAINT')
        if (not fresh(now, scene.get('observation_time'), .65)
                or not fresh(now, scene.get('generated_at'), .8)):
            raise ValueError('LIDAR_TABLE_CONSTRAINT_EXPIRED')
        if scene.get('whole_model_sha256') != self.whole_sha:
            raise ValueError('LIDAR_GEOMETRY_MODEL_MISMATCH')
        q = scene.get('joint_positions_rad')
        if not isinstance(q, dict) or set(q) != set(JOINTS) or any(
                not number(q[n]) or abs(q[n]-packet['pose'][n]['position']) > POSE_TOLERANCE
                for n in JOINTS):
            raise ValueError('LIDAR_GEOMETRY_POSE_MISMATCH')
        if not isinstance(odom, dict) or not fresh(now, odom.get('stamp'), .2):
            raise ValueError('ODOMETRY_EXPIRED_OR_MISSING')
        p, reference, v = odom.get('xy_yaw'), scene.get('base_xy_yaw'), odom.get('velocity')
        if not vector(p, 3) or not vector(reference, 3) or not vector(v, 3):
            raise ValueError('INVALID_ODOMETRY')
        yaw = math.atan2(math.sin(p[2]-reference[2]), math.cos(p[2]-reference[2]))
        dx, dy = p[0]-reference[0], p[1]-reference[1]
        forward = math.cos(reference[2])*dx + math.sin(reference[2])*dy
        lateral = -math.sin(reference[2])*dx + math.cos(reference[2])*dy
        if (abs(yaw) > .01 or abs(lateral) > .005 or forward < -.002
                or v[0] < -.001 or abs(v[1]) > .005 or abs(v[2]) > .02):
            raise ValueError('UNSUPPORTED_SIDE_REVERSE_OR_ROTATION')
        remaining = scene.get('forward_remaining_m')
        if not number(remaining) or remaining < 0:
            raise ValueError('INVALID_TABLE_DISTANCE')
        table = dict(valid=True, forward_remaining_m=max(0., remaining-max(0., forward)),
                     table_collision_risk=scene.get('table_collision_risk') is True,
                     blocking_link=scene.get('blocking_link'), scope=scene.get('scope', ''))
        if table['table_collision_risk']:
            table['forward_remaining_m'] = 0.
        return table, v[0]
