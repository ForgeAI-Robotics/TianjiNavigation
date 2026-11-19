"""LiDAR adaptation of the Jetson URDF/triangle ShapeMask geometry.

No RGB, Cutie, camera intrinsics, controller API or automatic point deletion.
Input sensor identity and point frame are mandatory to prevent double transforms.
"""
import ctypes
import hashlib
import json
import math
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as ET
import numpy as np
from link import fresh  # Initializes the isolated geometry dependency path.
from collision_geometry import origin
from environment import self_candidates

INVALID, ENVIRONMENT, SELF_CANDIDATE, PADDING, OCCLUDED, UNKNOWN_POSE = range(6)
LABEL_NAMES=('INVALID','ENVIRONMENT','SELF_CANDIDATE','PADDING','OCCLUDED_OR_MODEL_DISAGREEMENT','UNKNOWN_POSE')
LIBRARY=Path(__file__).resolve().parent/'native/lidar_mesh.so'


def rigid(matrix):
    m=np.asarray(matrix,dtype=float)
    if (m.shape!=(4,4) or not np.isfinite(m).all() or not np.allclose(m[3],[0,0,0,1])
            or not np.allclose(m[:3,:3].T@m[:3,:3],np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(m[:3,:3]),1.,atol=1e-6)):
        raise ValueError('RIGID_TRANSFORM_REQUIRED')
    return m


def rigid_batch(matrices):
    """Validate a whole FK snapshot once instead of two validations per link."""
    m=np.asarray(matrices,dtype=float)
    if m.size==0:return np.empty((0,4,4))
    if (m.ndim!=3 or m.shape[1:]!=(4,4) or not np.isfinite(m).all()
            or not np.allclose(m[:,3,:],[0,0,0,1])
            or not np.allclose(np.einsum('sji,sjk->sik',m[:,:3,:3],m[:,:3,:3]),np.eye(3),atol=1e-6)
            or not np.allclose(np.linalg.det(m[:,:3,:3]),1.,atol=1e-6)):
        raise ValueError('RIGID_TRANSFORM_REQUIRED')
    return m


def box_mesh(bounds):
    lo,hi=np.asarray(bounds,float)
    if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite([lo,hi]).all() or np.any(hi<=lo):
        raise ValueError('INVALID_BOX_BOUNDS')
    from collision_geometry import corners
    faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                    [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]],np.uint32)
    return corners([lo,hi]),faces


class TriangleBVH:
    def __init__(self,vertices,faces):
        v=np.ascontiguousarray(vertices,np.float32);f=np.asarray(faces)
        if (v.ndim!=2 or v.shape[1]!=3 or not np.isfinite(v).all() or f.ndim!=2
                or f.shape[1]!=3 or not len(f) or len(f)>2000000
                or not np.issubdtype(f.dtype,np.integer) or np.any(f<0) or np.any(f>=len(v))):
            raise ValueError('INVALID_TRIANGLE_MESH')
        if not LIBRARY.exists():raise RuntimeError('Build native/build.sh before using LiDAR ShapeMask')
        self.lib=ctypes.CDLL(str(LIBRARY));arr=np.ctypeslib.ndpointer
        self.lib.mesh_create.argtypes=[arr(np.float32,flags='C_CONTIGUOUS'),ctypes.c_uint32,
                                      arr(np.uint32,flags='C_CONTIGUOUS'),ctypes.c_uint32]
        self.lib.mesh_create.restype=ctypes.c_void_p
        self.lib.mesh_destroy.argtypes=[ctypes.c_void_p];self.lib.mesh_destroy.restype=None
        self.lib.mesh_rays.argtypes=[ctypes.c_void_p,arr(np.float64,flags='C_CONTIGUOUS'),
            arr(np.float64,flags='C_CONTIGUOUS'),arr(np.float64,flags='C_CONTIGUOUS'),
            ctypes.c_uint32,ctypes.c_double,arr(np.float64,flags='C_CONTIGUOUS')]
        self.lib.mesh_rays.restype=None
        self.handle=self.lib.mesh_create(v,len(v),np.ascontiguousarray(f,np.uint32),len(f))
        if not self.handle:raise ValueError('CANNOT_BUILD_TRIANGLE_BVH')

    def rays(self,ray_origin,directions,limits,minimum=.04):
        o=np.ascontiguousarray(ray_origin,np.float64);d=np.ascontiguousarray(directions,np.float64)
        limits=np.ascontiguousarray(limits,np.float64)
        if o.shape!=(3,) or d.ndim!=2 or d.shape[1]!=3 or limits.shape!=(len(d),):
            raise ValueError('INVALID_RAY_ARRAYS')
        if not np.isfinite(o).all() or not np.isfinite(d).all() or not np.isfinite(limits).all():
            raise ValueError('NONFINITE_RAY_INPUT')
        if not np.allclose(np.linalg.norm(d,axis=1),1.,atol=1e-6):raise ValueError('UNIT_RAY_DIRECTIONS_REQUIRED')
        if not self.handle:raise RuntimeError('BVH_CLOSED')
        out=np.empty(len(d));self.lib.mesh_rays(self.handle,o,d,limits,len(d),minimum,out)
        return out

    def close(self):
        if getattr(self,'handle',None):self.lib.mesh_destroy(self.handle);self.handle=None

    def plane_nearest(self,points,normal):
        p=np.ascontiguousarray(points,np.float64);n=np.ascontiguousarray(normal,np.float64)
        if (p.ndim!=2 or p.shape[1]!=3 or n.shape!=(3,) or not np.isfinite(p).all()
                or not np.isfinite(n).all() or not np.isclose(np.linalg.norm(n),1.,atol=1e-7)):
            raise ValueError('INVALID_PLANE_QUERY')
        if not self.handle:raise RuntimeError('BVH_CLOSED')
        function=self.lib.mesh_plane_query;arr=np.ctypeslib.ndpointer
        function.argtypes=[ctypes.c_void_p,arr(np.float64,flags='C_CONTIGUOUS'),arr(np.float64,flags='C_CONTIGUOUS'),
                           ctypes.c_uint32,arr(np.float64,flags='C_CONTIGUOUS'),arr(np.float64,flags='C_CONTIGUOUS')]
        function.restype=None
        distances=np.empty(len(p),np.float64);closest=np.empty_like(p)
        function(self.handle,p,n,len(p),distances,closest)
        return distances,closest

    def __del__(self):
        try:self.close()
        except (AttributeError,TypeError):pass


class LidarExtrinsics:
    def __init__(self,calibration,T_model_vendor,alignment_verified=False):
        self.T_model_vendor=rigid(T_model_vendor)
        self.alignment_verified=alignment_verified is True
        self.transforms={}
        for name,prefix in [('front','left'),('back','right')]:
            translation=np.asarray(calibration[prefix+'_to_base_T'],float)
            angles=np.asarray(calibration[prefix+'_to_base_R'],float)
            if translation.shape!=(3,) or angles.shape!=(3,) or not np.isfinite([translation,angles]).all():
                raise ValueError('INVALID_LIDAR_CALIBRATION')
            self.transforms[name]=self.T_model_vendor@origin(translation,np.deg2rad(angles))

    def transform(self,points,sensor_id,point_frame):
        if sensor_id not in self.transforms:raise ValueError('FRONT_OR_BACK_SOURCE_ID_REQUIRED')
        if point_frame=='sensor':matrix=self.transforms[sensor_id]
        elif point_frame=='vendor_base':matrix=self.T_model_vendor
        elif point_frame=='model_base':matrix=np.eye(4)
        else:raise ValueError('EXPLICIT_POINT_FRAME_REQUIRED')
        p=np.asarray(points,float)
        if p.ndim!=2 or p.shape[1]!=3 or len(p)>100000:raise ValueError('INVALID_LIDAR_POINTS')
        return p@matrix[:3,:3].T+matrix[:3,3],self.transforms[sensor_id][:3,3].copy()


def ray_box_entries(ray_origin,directions,bounds,matrix,minimum=.04):
    return _ray_box_entries(ray_origin,directions,bounds,rigid(matrix),minimum)


def _ray_box_entries(ray_origin,directions,bounds,matrix,minimum=.04):
    o=(np.asarray(ray_origin)-matrix[:3,3])@matrix[:3,:3]
    d=directions@matrix[:3,:3];lo,hi=np.asarray(bounds)
    lower=np.full(len(d),minimum);upper=np.full(len(d),np.inf)
    for k in range(3):
        parallel=np.abs(d[:,k])<1e-14
        denominator=np.where(parallel,1.,d[:,k])
        a=(lo[k]-o[k])/denominator;b=(hi[k]-o[k])/denominator
        near=np.where(parallel,-np.inf,np.minimum(a,b));far=np.where(parallel,np.inf,np.maximum(a,b))
        if o[k]<lo[k] or o[k]>hi[k]:far[parallel]=-np.inf
        lower=np.maximum(lower,near);upper=np.minimum(upper,far)
    return np.where(lower<=upper,lower,np.inf)


def classify_rays(points,sensor_origin,geometry,alignment_verified=False,self_tolerance=.012,padding=.025):
    """geometry entries contain (BVH or None, matrix, bounds, trusted, link)."""
    started=time.perf_counter()
    if not 0<self_tolerance<=padding<=.05:raise ValueError('INVALID_SELF_FILTER_TOLERANCE')
    p=np.asarray(points,float);sensor_origin=np.asarray(sensor_origin,float)
    if p.ndim!=2 or p.shape[1]!=3 or sensor_origin.shape!=(3,) or not np.isfinite(sensor_origin).all():
        raise ValueError('INVALID_LIDAR_GEOMETRY')
    labels=np.full(len(p),INVALID,np.uint8);expected=np.full(len(p),np.inf)
    ranges=np.linalg.norm(p-sensor_origin,axis=1)
    valid=np.isfinite(p).all(1)&(ranges>=.04)&(ranges<=30.)
    q=p[valid];distance=ranges[valid];directions=(q-sensor_origin)/distance[:,None]
    best=np.full(len(q),np.inf);unknown=np.full(len(q),np.inf);unknown_at_return=np.zeros(len(q),bool)
    link_ids=np.full(len(q),-1,int)
    matrices=rigid_batch([entry[1] for entry in geometry])
    for index,(mesh,_,bounds,trusted,link) in enumerate(geometry):
        matrix=matrices[index]
        entry=_ray_box_entries(sensor_origin,directions,bounds,matrix)
        eligible=(entry<=distance+padding)&(entry<=best+padding)
        if not trusted:
            unknown=np.minimum(unknown,entry)
            local=(q-matrix[:3,3])@matrix[:3,:3];lo,hi=np.asarray(bounds)
            gap=np.linalg.norm(np.maximum(np.maximum(lo-local,local-hi),0),axis=1)
            unknown_at_return|=gap<=padding
            continue
        if not eligible.any():continue
        local_origin=(sensor_origin-matrix[:3,3])@matrix[:3,:3]
        local_directions=directions[eligible]@matrix[:3,:3]
        hits=mesh.rays(local_origin,local_directions,distance[eligible]+padding)
        indices=np.flatnonzero(eligible);better=hits<best[indices]
        best[indices[better]]=hits[better];link_ids[indices[better]]=index
    output=np.full(len(q),ENVIRONMENT,np.uint8);hit=np.isfinite(best)
    delta=distance-best
    output[hit&(np.abs(delta)<=padding)]=PADDING
    output[hit&(np.abs(delta)<=self_tolerance)]=SELF_CANDIDATE
    output[hit&(delta>padding)]=OCCLUDED
    uncertain=unknown_at_return|((unknown<=distance+padding)&(unknown<=best+self_tolerance)&(output!=SELF_CANDIDATE))
    output[uncertain]=UNKNOWN_POSE
    labels[valid]=output;expected[valid]=best
    # Preview candidates do not authorize point removal under unverified alignment.
    removal=(labels==SELF_CANDIDATE)&bool(alignment_verified)
    return dict(labels=labels,expected_range_m=expected,measured_range_m=ranges,
                removal_mask=removal,keep_mask=~removal,
                summary=dict(points=len(p),labels={name:int(np.sum(labels==i)) for i,name in enumerate(LABEL_NAMES)},
                removed_points=int(removal.sum()),alignment_verified=bool(alignment_verified),
                self_tolerance_m=self_tolerance,padding_m=padding,
                free_space_inferred=False,compute_ms=(time.perf_counter()-started)*1000))


class LidarShapeMask:
    def __init__(self,model,extrinsics,simplify_chassis=False):
        self.model,self.extrinsics=model,extrinsics;self.meshes={};self.cache={};self.chassis_profile=None
        xml=ET.parse(model.urdf).getroot()
        for link in xml.findall('link'):
            for i,collision in enumerate(link.findall('collision')):
                name=collision.get('name',str(i));shape=collision.find('geometry')[0]
                if shape.tag=='box':
                    size=np.fromstring(shape.get('size'),sep=' ');key=('box',*size)
                    vertices,faces=box_mesh([-size/2,size/2])
                elif shape.tag=='mesh':
                    uri=shape.get('filename');prefix='package://tianji_robot_description/'
                    if not uri.startswith(prefix):raise ValueError('UNSUPPORTED_MESH_URI')
                    file=(model.urdf.parent.parent/uri[len(prefix):]).resolve(strict=True)
                    root=model.urdf.parents[3]
                    relative=str(file.relative_to(root))
                    raw=file.read_bytes()
                    if hashlib.sha256(raw).hexdigest()!=model.manifest['mesh_sha256'].get(relative):
                        raise ValueError('LOCAL_MESH_HASH_MISMATCH:'+relative)
                    count=struct.unpack_from('<I',raw,80)[0] if len(raw)>=84 else 0
                    if len(raw)!=84+50*count or not count:raise ValueError('BINARY_STL_REQUIRED')
                    scale=np.fromstring(shape.get('scale','1 1 1'),sep=' ')
                    if scale.shape!=(3,) or not np.isfinite(scale).all() or np.any(scale<=0):raise ValueError('INVALID_MESH_SCALE')
                    key=(str(file),*scale)
                    if key in self.cache:
                        self.meshes[(link.get('name'),name)]=self.cache[key];continue
                    records=np.frombuffer(raw,dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')]),offset=84,count=count)
                    vertices=records['vertices'].reshape(-1,3)*scale
                    faces=np.arange(count*3,dtype=np.uint32).reshape(-1,3)
                else:raise ValueError('UNSUPPORTED_LIDAR_COLLISION_GEOMETRY')
                if key not in self.cache:self.cache[key]=TriangleBVH(vertices,faces)
                self.meshes[(link.get('name'),name)]=self.cache[key]

        if simplify_chassis:
            from chassis_proxy import profile,PROXY_KEY
            self.chassis_profile=profile(model)
            vertices,faces=box_mesh(self.chassis_profile['bounds_m'])
            self.cache[PROXY_KEY]=TriangleBVH(vertices,faces);self.meshes[PROXY_KEY]=self.cache[PROXY_KEY]

    def classify(self,points,sensor_id,point_frame,positions):
        started=time.perf_counter()
        transformed,sensor=self.extrinsics.transform(points,sensor_id,point_frame)
        # Preserve Jetson policy: approximate finger linkage is not self-removal proof.
        trusted_positions={k:v for k,v in positions.items() if 'robotiq' not in k}
        shapes=self.model.shapes(trusted_positions)
        if self.chassis_profile:
            from chassis_proxy import replace_shapes
            shapes=replace_shapes(shapes,self.chassis_profile)
        geometry=[];known=[];unknown=[]
        for shape in shapes:
            mesh=self.meshes[(shape['link'],shape['name'])]
            trusted=shape['mode']=='measured_or_fixed'
            geometry.append((mesh,shape['matrix'],shape['bounds'],trusted,shape['link']))
            (known if trusted else unknown).append(shape)
        # Only returns near current measured geometry can be self. Avoid casting
        # tens of thousands of unrelated room returns against every mesh.
        # Unchecked points remain occupied returns, never free-space evidence.
        valid=np.isfinite(transformed).all(1)
        candidate=np.zeros(len(transformed),bool);uncertain=np.zeros(len(transformed),bool)
        if valid.any():
            from environment import cKDTree
            valid_points=transformed[valid]
            tree=cKDTree(valid_points) if cKDTree is not None else None
            if known:candidate[valid]=self_candidates(valid_points,known,.025,_tree=tree)
            if unknown:uncertain[valid]=self_candidates(valid_points,unknown,.025,_tree=tree)
        result=classify_rays(transformed[candidate],sensor,geometry,self.extrinsics.alignment_verified)
        labels=np.full(len(transformed),ENVIRONMENT,np.uint8)
        ranges=np.linalg.norm(transformed-sensor,axis=1)
        labels[~valid|(ranges<.04)|(ranges>30.)]=INVALID
        labels[uncertain&(labels!=INVALID)]=UNKNOWN_POSE
        labels[candidate]=result['labels']
        expected=np.full(len(transformed),np.inf);expected[candidate]=result['expected_range_m']
        removal=np.zeros(len(transformed),bool);removal[candidate]=result['removal_mask']
        if self.chassis_profile:
            lo,hi=np.array(self.chassis_profile['bounds_m'])
            inside=(transformed>=lo).all(1)&(transformed<=hi).all(1)&(labels!=INVALID)
            labels[inside]=SELF_CANDIDATE
            gap=np.linalg.norm(np.maximum(np.maximum(lo-transformed,transformed-hi),0),axis=1)
            # Approximate filled cuboid is a candidate volume, not deletion proof.
            removal[gap<=.025]=False
        result.update(labels=labels,expected_range_m=expected,measured_range_m=ranges,
                      removal_mask=removal,keep_mask=~removal)
        result['summary'].update(points=len(transformed),ray_checked_points=int(candidate.sum()),
            labels={name:int(np.sum(labels==i)) for i,name in enumerate(LABEL_NAMES)},
            removed_points=int(removal.sum()),compute_ms=(time.perf_counter()-started)*1000,
            occlusion_scope='self-filter candidates only; unchecked returns retained occupied')
        result['points_model']=transformed
        result['chassis_collision']=self.chassis_profile
        result['summary'].update(sensor_id=sensor_id,input_point_frame=point_frame,
            sensor_origin_model_m=sensor.tolist(),model_sha256=self.model.sha256,
            unknown_shape_count=sum(not s[3] for s in geometry),unique_meshes=len(self.cache),
            source='Jetson original triangle BVH + model FK; LiDAR radial ray adaptation; no Cutie')
        return result

    def close(self):
        for mesh in self.cache.values():mesh.close()
        self.cache.clear()


def positions_at_cloud(packet,model_sha,stamp,now):
    """Use only measured joints whose stationary history covers this cloud."""
    from link import number,POSE_TOLERANCE
    if (not isinstance(packet,dict) or packet.get('schema_version')!=2 or packet.get('advisory_only') is not True
            or packet.get('whole_model_sha256')!=model_sha
            or not fresh(now,packet.get('generated_at'),.3) or not fresh(now,stamp,.65)):
        raise ValueError('LIDAR_MODEL_OR_STATE_EXPIRED_OR_MISMATCH')
    history=packet.get('motion_history')
    if not isinstance(history,dict):raise ValueError('LIDAR_POSE_HISTORY_MISSING')
    uncertainty=0.
    if 'clock_mapping' in packet:
        clock=packet['clock_mapping']
        if (not isinstance(clock,dict) or clock.get('timestamp_domain')!='S100_SYSTEM_REALTIME'
                or not number(clock.get('uncertainty_s')) or not 0<=clock['uncertainty_s']<=.005
                or not number(clock.get('age_s')) or not 0<=clock['age_s']<=2.):
            raise ValueError('LIDAR_CLOCK_MAPPING_UNVERIFIED')
        uncertainty=clock['uncertainty_s']
    positions={}
    for name,sample in history.items():
        if (isinstance(sample,dict) and all(number(sample.get(k)) for k in
                ('position','covered_from','stamp','minimum','maximum','max_sample_gap_s'))
                and fresh(now,sample['stamp'],.25)
                and sample['covered_from']+uncertainty<=stamp<=sample['stamp']-uncertainty
                and sample['minimum']<=sample['position']<=sample['maximum']
                and sample['maximum']-sample['minimum']<=POSE_TOLERANCE
                and 0<=sample['max_sample_gap_s']<=.25):
            positions[name]=sample['position']
    return positions
