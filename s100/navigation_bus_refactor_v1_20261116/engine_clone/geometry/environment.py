"""Bounded point-cloud environment perception in the robot model root frame.

Retains single-return obstacles. Self candidates are explicitly ambiguous, never
silently cleared. A sparse surface cloud provides no free-volume certificate.
"""
import math
import time
from dataclasses import dataclass
import numpy as np

from link import fresh, number, vector
from collision_geometry import near_points
from table_scene import estimate_table
try:
    from scipy.spatial import cKDTree
    from scipy.ndimage import label, find_objects
except ImportError:
    cKDTree = label = find_objects = None


def decode_cloud(msg):
    """PointCloud2 decoder supporting row padding, field offsets and endianness."""
    if not 0 < msg.width * msg.height <= 250000 or not 12 <= msg.point_step <= 256:
        raise ValueError('POINT_CLOUD_SIZE_INVALID')
    if msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.height*msg.row_step:
        raise ValueError('POINT_CLOUD_BUFFER_INVALID')
    fields = {f.name: f for f in msg.fields}
    if any(n not in fields or fields[n].datatype != 7 or fields[n].count != 1
           or not 0 <= fields[n].offset <= msg.point_step-4 for n in ('x','y','z')):
        raise ValueError('POINT_CLOUD_XYZ_FIELDS_INVALID')
    offsets=sorted(fields[n].offset for n in ('x','y','z'))
    if any(b-a<4 for a,b in zip(offsets,offsets[1:])):
        raise ValueError('POINT_CLOUD_XYZ_FIELDS_OVERLAP')
    dtype = np.dtype(dict(names=['x','y','z'], formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                         offsets=[fields[n].offset for n in ('x','y','z')], itemsize=msg.point_step))
    data = np.ndarray((msg.height,msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step,msg.point_step))
    return np.column_stack([data[n].reshape(-1) for n in ('x','y','z')]).astype(float)


def voxelize(points, resolution):
    keys = np.floor(points/resolution).astype(np.int64)
    unique, indices, counts = np.unique(keys, axis=0, return_index=True, return_counts=True)
    # Occupancy uses full voxel cells, not a centroid that could move a thin hit.
    return unique, (unique+.5)*resolution, counts


def ground_plane(points):
    low = points[(points[:,2] > -.15) & (points[:,2] < .10)]
    if len(low) < 100:
        return None
    h, edges = np.histogram(low[:,2], np.arange(-.15,.111,.01))
    center = edges[int(h.argmax())]+.005
    p = low[np.abs(low[:,2]-center) < .012]
    if len(p) < 100 or np.ptp(p[:,0]) < .5 or np.ptp(p[:,1]) < .5:
        return None
    X = np.c_[p[:,:2],np.ones(len(p))]
    c = np.linalg.lstsq(X,p[:,2],rcond=None)[0]
    error = np.abs(p[:,2]-X@c)
    if np.linalg.norm(c[:2]) > .05 or np.percentile(error,95) > .008:
        return None
    return c


def components(keys, resolution, limit=64):
    """6-connected voxel components for descriptions, never a removal filter."""
    if not len(keys):
        return []
    lower,upper=keys.min(0),keys.max(0)
    size=upper-lower+1
    if label is not None and int(np.prod(size))<=4000000:
        grid=np.zeros(tuple(size),dtype=bool)
        grid[tuple((keys-lower).T)]=True
        labels,count=label(grid)
        counts=np.bincount(labels.ravel());counts[0]=0
        selected=np.argsort(counts)[::-1][:limit]
        boxes=find_objects(labels)
        return [dict(voxels=int(counts[i]),minimum_m=((lower+[s.start for s in boxes[i-1]])*resolution).tolist(),
                     maximum_m=((lower+[s.stop for s in boxes[i-1]])*resolution).tolist(),
                     kind='UNCLASSIFIED_OCCUPIED_GEOMETRY') for i in selected if i and counts[i]]
    # Bounded fallback affects descriptions only; collision retains all cells.
    tail=keys[4096:];keys=keys[:4096]
    remaining = {tuple(k) for k in keys}
    result = []
    while remaining:
        start = remaining.pop(); stack = [start]; group = []
        while stack:
            k = stack.pop(); group.append(k)
            for axis in range(3):
                for d in (-1,1):
                    neighbor = list(k); neighbor[axis] += d; neighbor = tuple(neighbor)
                    if neighbor in remaining:
                        remaining.remove(neighbor); stack.append(neighbor)
        p = np.asarray(group)
        result.append(dict(voxels=len(group), minimum_m=(p.min(0)*resolution).tolist(),
                           maximum_m=((p.max(0)+1)*resolution).tolist(),
                           kind='UNCLASSIFIED_OCCUPIED_GEOMETRY'))
    result=sorted(result,key=lambda v:v['voxels'],reverse=True)[:limit]
    if len(tail):
        result.append(dict(voxels=len(tail),minimum_m=(tail.min(0)*resolution).tolist(),
                           maximum_m=((tail.max(0)+1)*resolution).tolist(),
                           kind='UNCLUSTERED_OCCUPIED_GEOMETRY'))
    return result


def self_candidates(points,shapes,margin,_tree=None):
    if cKDTree is None or not len(points):
        return near_points(points,shapes,margin)[0]
    if not shapes:return np.zeros(len(points),dtype=bool)
    tree=_tree if _tree is not None else cKDTree(points)
    mask=np.zeros(len(points),dtype=bool)
    matrices=np.asarray([s['matrix'] for s in shapes]);bounds=np.asarray([s['bounds'] for s in shapes])
    lower,upper=bounds[:,0],bounds[:,1]
    centers=np.einsum('sij,sj->si',matrices[:,:3,:3],(lower+upper)/2)+matrices[:,:3,3]
    radii=np.linalg.norm((upper-lower)/2,axis=1)+margin
    candidates=tree.query_ball_point(centers,radii)
    for i,rows in enumerate(candidates):
        matrix=matrices[i];lo,hi=lower[i],upper[i]
        indices=np.asarray(rows,dtype=int)
        if not len(indices):continue
        local=(points[indices]-matrix[:3,3])@matrix[:3,:3]
        distance=np.linalg.norm(np.maximum(np.maximum(lo-local,local-hi),0),axis=1)
        mask[indices[distance<=margin]]=True
    return mask


@dataclass
class Environment:
    stamp: float
    base_xy_yaw: list
    resolution: float
    points: np.ndarray
    ambiguous_points: np.ndarray
    summary: dict


class EnvironmentPerception:
    def __init__(self, resolution=.03, max_voxels=35000):
        if not .01 <= resolution <= .1 or not 1 <= max_voxels <= 100000:
            raise ValueError('INVALID_ENVIRONMENT_BUDGET')
        self.resolution, self.max_voxels = resolution, max_voxels

    def observe(self, points, stamp, now, frame_id, base_xy_yaw, shapes):
        started = time.perf_counter()
        if frame_id != 'tianji/chassis_base':
            raise ValueError('POINT_CLOUD_TO_MODEL_ALIGNMENT_REQUIRED')
        if not fresh(now,stamp,.65) or not vector(base_xy_yaw,3):
            raise ValueError('POINT_CLOUD_TIMESTAMP_OR_BASE_POSE_INVALID')
        p = np.asarray(points,dtype=float)
        if p.ndim != 2 or p.shape[1] != 3 or not 1 <= len(p) <= 250000:
            raise ValueError('POINT_CLOUD_ARRAY_INVALID')
        raw_count = len(p)
        p = p[np.isfinite(p).all(axis=1)]
        p = p[(np.abs(p[:,0]) <= 3.) & (np.abs(p[:,1]) <= 2.5)
              & (p[:,2] >= -.15) & (p[:,2] <= 2.3)]
        if len(p) < 10:
            raise ValueError('INSUFFICIENT_ENVIRONMENT_RETURNS')
        floor = ground_plane(p)
        floor_mask = (np.abs(p[:,2]-np.c_[p[:,:2],np.ones(len(p))]@floor)<.008
                      if floor is not None else np.zeros(len(p),dtype=bool))
        obstacle = p[~floor_mask]
        keys, centers, counts = voxelize(obstacle,self.resolution)
        if len(keys) > self.max_voxels:
            raise ValueError('ENVIRONMENT_VOXEL_BUDGET_EXCEEDED')
        # A full voxel close to a current OBB can be an external object or self.
        self_mask = self_candidates(centers,shapes,math.sqrt(3)*self.resolution/2)
        table = estimate_table(p)
        if table.get('valid'):
            a,b,c = table['plane']
            support = p[np.abs(p[:,2]-a*p[:,0]-b*p[:,1]-c)<.008]
            table['observed_x_range'] = [float(support[:,0].min()),float(support[:,0].max())]
        ext = centers[~self_mask]
        summary = dict(schema_version=1, frame_id=frame_id, observation_time=stamp,
                       base_xy_yaw=base_xy_yaw, raw_points=raw_count,
                       retained_points=len(p), voxel_resolution_m=self.resolution,
                       occupied_voxels=len(keys), external_voxels=len(ext),
                       single_return_voxels=int(np.sum(counts==1)),
                       self_or_contact_voxels=int(self_mask.sum()),
                       ground_plane= None if floor is None else floor.tolist(),
                       ground_return_points=int(floor_mask.sum()), table=table,
                       obstacles=components(keys[~self_mask],self.resolution),
                       missing_joint_bound_shapes=sum(s['mode']!='measured_or_fixed' for s in shapes),
                       free_space_certified=False, self_returns_disambiguated=False,
                       scope='observed occupied geometry; missing returns remain unknown')
        summary['compute_ms'] = (time.perf_counter()-started)*1000
        return Environment(stamp,list(base_xy_yaw),self.resolution,ext,centers[self_mask],summary)
