"""Conservative targeted checks against actual measured beams.
Nearest angular neighbours only select candidate beams. Actual per-ray origins,
measured endpoints, stored same-view samples and exact distances decide votes.
Missing angular neighbours produce no vote; no synthetic rays are generated.
"""
import numpy as np
from scipy.spatial import cKDTree


def target_free_votes(lookup, targets, origins, endpoints, neighbours=12,
                      beam_radius=.02, endpoint_margin=.15, maximum_range=10.):
    targets=np.asarray(targets, dtype=int)
    origin=np.asarray(origins, float);endpoints=np.asarray(endpoints,float)
    result=np.zeros(len(lookup.centers),bool)
    if origin.shape==(3,):origin=np.broadcast_to(origin,endpoints.shape)
    if origin.shape!=endpoints.shape:raise ValueError('SENSOR_ORIGIN_SHAPE_MISMATCH')
    if not len(targets) or not len(endpoints):return result
    delta=endpoints-origin;length=np.linalg.norm(delta,axis=1)
    good=np.isfinite(delta).all(1)&(length>endpoint_margin)
    origin=origin[good];endpoints=endpoints[good];length=length[good];unit=delta[good]/length[:,None]
    if not len(unit):return result
    # Query with an approximate common origin, then validate exact per-ray geometry.
    center=origin.mean(0);direction=lookup.representatives[targets]-center
    distance=np.linalg.norm(direction,axis=1);valid=(distance>.04)&(distance<maximum_range+.2)
    targets=targets[valid];direction=direction[valid]/distance[valid,None]
    if not len(targets):return result
    _,ids=cKDTree(unit).query(direction,k=min(neighbours,len(unit)),workers=1)
    if ids.ndim==1:ids=ids[:,None]
    ray=unit[ids];ray_origin=origin[ids];stop=np.minimum(length[ids]-endpoint_margin,maximum_range)
    passed=np.zeros(len(targets),bool)
    for slot in range(8):
        reference=lookup.view_points[targets,slot]
        to_reference=reference[:,None,:]-ray_origin
        along=np.einsum('nki,nki->nk',to_reference,ray)
        perpendicular2=np.maximum(0.,np.einsum('nki,nki->nk',to_reference,to_reference)-along*along)
        same=lookup.view_valid[targets,slot,None]&(np.einsum('nki,ni->nk',ray,lookup.view_directions[targets,slot])>=np.cos(np.deg2rad(10.)))
        ok=same&(perpendicular2<=beam_radius**2)&(along>0)&(along<stop)
        if hasattr(lookup,'surface_normals'):
            normals=lookup.surface_normals[targets];planar=np.linalg.norm(normals,axis=1)>.5
            ok&=~planar[:,None]|(np.abs(np.einsum('nki,ni->nk',ray,normals))>=np.sin(np.deg2rad(20.)))
        passed|=ok.any(1)
    result[targets[passed]]=True
    return result
