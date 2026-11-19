"""Source streamed to S100 for stationary live LiDAR inspection."""
import json
import os
import sys
import threading
import time
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from paths import ROOT
from chassis_proxy import measured_shapes
from client import SensorStream
from common import exclusive
from environment import self_candidates
from execution_safety import REQUIRED_JOINTS, curved_sweep_shapes, sweep_guard
from extrinsics import transforms
from motion_certificate import BASE_ADVANCE_S
from onboard_state import measured_joints, model
from prepared_frames import PreparedFrames
from saved_map import base_transform
from sensor_bus import SensorBus


def send(**packet):
    print(json.dumps(packet, allow_nan=False), flush=True)


def run(reference):
    scene = json.loads((reference / 'scene/frozen_input.json').read_text())
    loc = scene['localization']
    transform = np.array(loc['T_map_base'])
    a, b, c = scene['ground_plane']
    normal_map = np.array([-a, -b, 1.])
    stop = threading.Event()
    connection = threading.Thread(target=lambda: (os.read(0, 1), stop.set()), daemon=True)
    connection.start()
    m, matrices = model(), transforms()
    with exclusive(ROOT / 'runtime/navigation_execution.lock'), TemporaryDirectory(dir='/dev/shm') as temporary:
        bus = SensorBus(Path(temporary) / 'bus')
        stream = SensorStream(localization=True, sensor_hz=2.)
        latest = {}
        failure = []

        def relay():
            while not stop.is_set():
                try:
                    header, points = stream.receive()
                    bus.publish(header, points)
                    if header['kind'] == 'odometry':
                        latest['odom'] = header, time.monotonic()
                except ValueError:
                    continue
                except (OSError, EOFError) as exc:
                    if not stop.is_set():
                        failure.append(str(exc))
                    return

        reader = threading.Thread(target=relay, daemon=True)
        reader.start()
        worker = None
        try:
            worker = PreparedFrames(m, bus.folder, conservative_fast=True, refine_near_hits=True, refine_radius_m=.08)
            while not stop.wait(.30):
                if failure or worker.error:
                    raise RuntimeError(worker.error or failure[0])
                prepared = worker.snapshot()
                odometry = latest.get('odom')
                if len(prepared) != 2 or odometry is None:
                    send(reason='WAIT_LIDAR')
                    continue
                header, received = odometry
                now = time.monotonic()
                if (now - received + header.get('source_age_s', 0) > .5
                        or header['epoch'] != loc['odometry_epoch']
                        or header['boot_id'] != loc['source_boot_id']
                        or header['calibration_sha256'] != loc['calibration_sha256']):
                    send(reason='ODOMETRY_UNAVAILABLE')
                    continue
                try:
                    joints = measured_joints()['joint_positions_rad']
                    if set(REQUIRED_JOINTS) - joints.keys():
                        raise ValueError('FULL_JOINT_FEEDBACK_REQUIRED')
                    shapes = measured_shapes(m, joints)
                    pose = header['pose']
                    current = transform @ np.linalg.inv(base_transform(loc['wheel_pose'])) @ base_transform(pose)
                    normal = current[:3, :3].T @ normal_map
                    offset = normal_map @ current[:3, 3] - c
                    floor = [-normal[0] / normal[2], -normal[1] / normal[2], -offset / normal[2]]
                    frames = {key: data['frame'] for key, data in prepared.items()}
                    if any(tuple(f[0][key] for key in ('boot_id', 'epoch', 'calibration_sha256'))
                           != tuple(header[key] for key in ('boot_id', 'epoch', 'calibration_sha256'))
                           for f in frames.values()):
                        send(reason='FRAME_IDENTITY_CHANGED')
                        continue
                    debug = {}
                    twist = [0., -.01, 0.]
                    _, reason, age = sweep_guard(frames, matrices, pose, shapes, m.sha256, floor, twist,
                                                 time.monotonic(), prepared=prepared, diagnostics=debug,
                                                 revalidation_reserve_m=.005, motion_advance_s=BASE_ADVANCE_S,
                                                 directional_age_compensation=True)
                    hits = debug.get('points_body', np.empty((0, 3)))
                    groups = {}
                    if len(hits):
                        for shape in curved_sweep_shapes(shapes, twist, horizon=.5 + BASE_ADVANCE_S):
                            points = hits[self_candidates(hits, [shape], debug['margin_m'])]
                            if len(points):
                                groups.setdefault(shape['link'], []).extend(points.tolist())
                    send(reason=reason, age_s=age, count=len(hits), groups=groups, joints=joints, model_sha256=m.sha256,
                         margin_m=debug.get('margin_m'))
                except (ValueError, KeyError) as exc:
                    send(reason=str(exc))
        finally:
            stop.set()
            if worker:
                worker.close()
            stream.close()
            reader.join(timeout=2)


if __name__ == '__main__':
    run(Path(sys.argv[1]))
