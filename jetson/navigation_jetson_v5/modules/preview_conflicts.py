"""Reconstruct endpoint collision annotations from an S100 preview snapshot."""
import json
from itertools import product
from pathlib import Path

import numpy as np


def load_preview(folder, model_hash):
    folder = Path(folder)
    result = json.loads((folder / 'planning/plan.json').read_text())
    world = json.loads((folder / 'scene/immutable_world.json').read_text())
    if world['model_sha256'] != model_hash:
        raise ValueError('规划URDF版本与本机原模型不一致')
    frozen = json.loads((folder / 'scene/frozen_input.json').read_text())
    attempt = next((a for a in result['attempts']
                    if a['reason'] in ('START_COLLISION', 'GOAL_COLLISION')), {})
    at_goal = attempt.get('reason') == 'GOAL_COLLISION'
    x, y, yaw = frozen['goal'] if at_goal else (0, 0, 0)
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    origin = np.array([x, y, 0])
    cloud = (np.asarray(world['occupied_points'], dtype=float).reshape(-1, 3) - origin) @ rotation
    half_voxel = world['resolution_m'] / 2
    margin = .05
    diagonal = np.sqrt(3) * half_voxel
    axes = world.get('voxel_box_rotation')
    voxel = None if axes is None else rotation.T @ np.array(axes)
    boxes, pairs, groups = [], [], {}
    for index, shape in enumerate(world['shapes']):
        link = shape['link']
        matrix = np.array(shape['matrix'])
        r, t = matrix[:3, :3], matrix[:3, 3]
        lo, hi = np.array(shape['bounds'])
        corners = np.array(list(product(*zip(lo, hi)))) @ r.T + t
        boxes.append(dict(link=link, collision_name=str(index), T_body_collision=matrix.tolist(),
                          local_bounds_m=shape['bounds'], mode=shape['mode'], body_corners_m=corners.tolist()))
        if link != attempt.get('blocking_link'):
            continue
        local = (cloud - t) @ r
        nearest = np.clip(local, lo, hi)
        distances = np.linalg.norm(local - nearest, axis=1)
        blocked = distances <= margin + diagonal
        if voxel is not None:
            candidates = np.flatnonzero(blocked)
            delta = cloud[candidates] - (r @ ((lo + hi) / 2) + t)
            axes = [*r.T, *voxel.T, *(np.cross(a, b) for a in r.T for b in voxel.T)]
            for axis in axes:
                length = np.linalg.norm(axis)
                if length >= 1e-10:
                    limit = (hi - lo) / 2 @ np.abs(r.T @ axis)
                    limit += half_voxel * np.abs(voxel.T @ axis).sum() + margin * length + 1e-10
                    blocked[candidates[np.abs(delta @ axis) > limit]] = False
        for i in np.flatnonzero(blocked):
            point_id = f'P{i:05d}'
            row = dict(point_id=point_id, link=link, point_body_m=cloud[i].tolist(),
                       nearest_urdf_collision_body_m=(nearest[i] @ r.T + t).tolist(),
                       point_to_collision_distance_m=float(distances[i]))
            groups.setdefault(point_id, []).append(row)
            pairs.append(row)
    target = result['target']
    endpoint = '目标位姿' if at_goal else '起点位姿'
    status = attempt.get('reason', result['state'])
    label = f"{target['environment']} · {target['name']} · {endpoint if attempt else '规划结果'} · {status}"
    meta = dict(source_scene=str(folder), urdf_collision_poses=boxes, pairs=pairs,
                thresholds=dict(safety_margin_m=margin, voxel_half_diagonal_m=diagonal,
                                effective_collision_distance_m=margin + diagonal),
                preview_label=label)
    points = np.array([groups[k][0]['point_body_m'] for k in sorted(groups)]).reshape(-1, 3)
    return meta, points, groups, cloud
