"""Conservative fixed-arm planar swept-volume checks against occupied voxels.

OBB inflation covers voxel half diagonal and all between-sample motion. Reports
observed collision candidates, unknown coverage and current self/contact ambiguity.
"""
import math
import time
import numpy as np
from link import fresh, vector

try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None


def transformed(shape, delta):
    x,y,yaw = delta
    c,s = math.cos(yaw),math.sin(yaw)
    r = np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    matrix = np.asarray(shape['matrix'])
    return r@matrix[:3,:3], r@matrix[:3,3]+[x,y,0.]


def check_sweep(shapes, environment, delta, now, margin=.05, step=.01, max_steps=200, start_delta=None):
    started = time.perf_counter()
    if (not vector(delta,3) or not .01 <= margin <= .3 or not .002 <= step <= .05
            or not shapes or not fresh(now,environment.stamp,.65)):
        return dict(state='UNKNOWN', reasons=['INVALID_OR_EXPIRED_SWEEP_INPUT'], collision=False)
    start_delta = [0.,0.,0.] if start_delta is None else start_delta
    if not vector(start_delta,3):
        return dict(state='UNKNOWN', reasons=['INVALID_START_TRANSFORM'],collision=False)
    difference = [b-a for a,b in zip(start_delta,delta)]
    radius = max(float(np.linalg.norm(np.asarray(s['world_corners'])[:,:2],axis=1).max()) for s in shapes)
    travel = math.hypot(*difference[:2])+radius*abs(difference[2])
    count = max(1,int(math.ceil(travel/step)))
    if count > max_steps:
        return dict(state='UNKNOWN', reasons=['SWEEP_COMPUTATION_BUDGET_EXCEEDED'],collision=False)
    padding = margin+math.sqrt(3)*environment.resolution/2+travel/count/2
    points = environment.points
    tree = cKDTree(points) if cKDTree is not None and len(points) else None
    matrices=np.asarray([s['matrix'] for s in shapes])
    bounds=np.asarray([s['bounds'] for s in shapes]);lower=bounds[:,0];upper=bounds[:,1]
    local_centers=(lower+upper)/2
    centers0=np.einsum('sij,sj->si',matrices[:,:3,:3],local_centers)+matrices[:,:3,3]
    radii=np.linalg.norm((upper-lower)/2,axis=1)+padding
    nearest = float('inf'); nearest_link = None; collision = None
    for index in range(count+1):
        if time.perf_counter()-started > .35:
            return dict(state='UNKNOWN', reasons=['SWEEP_RUNTIME_BUDGET_EXCEEDED'],collision=False,
                        free_space_certified=False, compute_ms=(time.perf_counter()-started)*1000)
        fraction = index/count
        move = [a+d*fraction for a,d in zip(start_delta,difference)]
        cy,sy=math.cos(move[2]),math.sin(move[2])
        body_r=np.array([[cy,-sy,0.],[sy,cy,0.],[0.,0.,1.]])
        centers=centers0@body_r.T+[move[0],move[1],0.]
        rotations=np.einsum('ij,sjk->sik',body_r,matrices[:,:3,:3])
        translations=matrices[:,:3,3]@body_r.T+[move[0],move[1],0.]
        broad=(tree.query_ball_point(centers,radii) if tree is not None else
               [np.flatnonzero(np.linalg.norm(points-center,axis=1)<=radius)
                for center,radius in zip(centers,radii)])
        for shape_index,indices in enumerate(broad):
            if not len(indices):continue
            shape=shapes[shape_index]
            r,t=rotations[shape_index],translations[shape_index]
            lo,hi=lower[shape_index],upper[shape_index]
            candidate=points[indices]
            if not len(candidate):
                continue
            local = (candidate-t)@r
            gap = np.linalg.norm(np.maximum(np.maximum(lo-local,local-hi),0.),axis=1)
            closest = int(np.argmin(gap)); distance = float(gap[closest])
            if distance < nearest:
                nearest,nearest_link = distance,shape['link']
            if distance <= padding:
                collision = dict(link=shape['link'], shape=shape['name'], fraction=fraction,
                                 candidate_delta=move, voxel_center_m=candidate[closest].tolist(),
                                 point_to_obb_m=distance, effective_padding_m=padding,
                                 geometry_mode=shape['mode'])
                break
        if collision:
            break
    ambiguous = len(environment.ambiguous_points)>0
    return dict(state='COLLISION_RISK' if collision else 'NO_OBSERVED_COLLISION',
                collision=bool(collision), contact=collision,
                reasons=['CURRENT_SELF_OR_CONTACT_AMBIGUITY'] if ambiguous else [],
                swept_steps=count, sampled_max_displacement_m=travel/count,
                closest_checked_point_to_obb_m=None if not math.isfinite(nearest) else nearest,
                closest_checked_link=nearest_link, unknown_space_present=True,
                current_self_or_contact_ambiguous=ambiguous,
                free_space_certified=False, command_path_connected=False,
                compute_ms=(time.perf_counter()-started)*1000)
