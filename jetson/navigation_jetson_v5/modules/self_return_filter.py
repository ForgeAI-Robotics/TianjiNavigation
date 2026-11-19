"""URDF surface + link-local temporal evidence for mapping self-return removal.
Never marks free space. Unknown/missing joints are not deletion evidence.
World cells with independent environment witnesses remain occupied.
"""
import ctypes,math,sys,time
from pathlib import Path
import numpy as np
from common import ROOT
LIB=ROOT.parent/'navigation_link_v2'
OFFSET=1<<20;MASK=(1<<21)-1

def voxel_codes(points,resolution=.05):
    key=np.floor(np.asarray(points,float)/resolution).astype(np.int64)
    if key.ndim!=2 or key.shape[1]!=3 or (abs(key)>=OFFSET).any():raise ValueError('VOXEL_CODE_RANGE')
    key+=OFFSET
    return (key[:,0]<<42)|(key[:,1]<<21)|key[:,2]

def triangle_distances(mesh,points):
    q=np.ascontiguousarray(points,np.float64)
    if q.ndim!=2 or q.shape[1]!=3 or not np.isfinite(q).all():raise ValueError('INVALID_MESH_POINTS')
    function=getattr(mesh,'_self_distance_query',None)
    if function is None:
        function=mesh.lib.mesh_query;arr=np.ctypeslib.ndpointer
        function.argtypes=[ctypes.c_void_p,arr(np.float64,flags='C_CONTIGUOUS'),ctypes.c_uint32,arr(np.float64,flags='C_CONTIGUOUS'),arr(np.float64,flags='C_CONTIGUOUS')];function.restype=None;mesh._self_distance_query=function
    distances=np.empty(len(q),float);closest=np.empty_like(q)
    if len(q):function(mesh.handle,q,len(q),distances,closest)
    return distances

class Evidence:
    def __init__(self,minimum_windows=25,minimum_span_s=45.,minimum_motion_m=.8,max_bindings=300000,max_voxels=250000,allowed_codes=None,resolution=.05):
        self.resolution=resolution;self.minimum_windows=minimum_windows;self.minimum_span_s=minimum_span_s;self.minimum_motion_m=minimum_motion_m;self.max_bindings=max_bindings;self.max_voxels=max_voxels
        self.rows={};self.environment={};self.self_hits={};self.bindings=0;self.capacity_exhausted=False;self.allowed_codes=allowed_codes
    def allowed(self,code):return self.allowed_codes is None or code in self.allowed_codes
    def environment_observed(self,codes,window):
        bit=1<<window
        for raw in np.unique(codes):
            code=int(raw)
            if not self.allowed(code):continue
            if code not in self.environment and len(self.environment)>=self.max_voxels:self.capacity_exhausted=True;return
            self.environment[code]=self.environment.get(code,0)|bit
    def candidate_observed(self,key,codes,window,position):
        if self.capacity_exhausted:return False
        bit=1<<window;position=np.asarray(position,float)
        row=self.rows.get(key)
        if row is None:
            if len(self.rows)>=50000:self.capacity_exhausted=True;return False
            row=dict(bits=0,first=window,last=window,low=position.copy(),high=position.copy(),ready=False,bindings={});self.rows[key]=row
        row['bits']|=bit;row['first']=min(row['first'],window);row['last']=max(row['last'],window);row['low']=np.minimum(row['low'],position);row['high']=np.maximum(row['high'],position)
        for raw in np.unique(codes):
            code=int(raw)
            if not self.allowed(code):continue
            if row['ready']:
                if code not in self.self_hits and len(self.self_hits)>=self.max_voxels:self.capacity_exhausted=True;return False
                self.self_hits[code]=self.self_hits.get(code,0)|bit
            else:
                if code not in row['bindings']:
                    if self.bindings>=self.max_bindings:self.capacity_exhausted=True;return False
                    self.bindings+=1
                row['bindings'][code]=row['bindings'].get(code,0)|bit
        if (not row['ready'] and row['bits'].bit_count()>=self.minimum_windows and row['last']-row['first']>=self.minimum_span_s and np.linalg.norm(row['high']-row['low'])>=self.minimum_motion_m):
            for code,bits in row['bindings'].items():self.self_hits[code]=self.self_hits.get(code,0)|bits
            self.bindings-=len(row['bindings']);row['bindings'].clear();row['ready']=True
        return row['ready']
    def removal_codes(self,protected=()):
        if self.capacity_exhausted:return set()
        protected=set(protected);result=set()
        for code in self.self_hits:
            if code not in protected and self.can_remove(code):result.add(code)
        return result
    def can_remove(self,code):
        if self.capacity_exhausted:return False
        self_count=self.self_hits.get(code,0).bit_count();env_count=self.environment.get(code,0).bit_count()
        height=((code&MASK)-OFFSET+.5)*self.resolution
        return self_count>=2 and env_count<2 and self_count/max(1,self_count+env_count)>=.8 and height>.12
    def summary(self):
        return dict(learned_link_surface_cells=sum(r['ready'] for r in self.rows.values()),candidate_link_surface_cells=len(self.rows),self_observed_world_cells=len(self.self_hits),environment_observed_world_cells=len(self.environment),pending_bindings=self.bindings,capacity_exhausted=self.capacity_exhausted)

class SurfaceGeometry:
    def __init__(self):
        sys.path.insert(0,str(LIB))
        from local_model import LocalRobotModel,default_urdf_path
        from lidar_shapemask import LidarShapeMask,LidarExtrinsics
        import json
        self.model=LocalRobotModel(default_urdf_path(),LIB/'model_manifest.json',LIB/'collision_geometry.json')
        self.meshes=LidarShapeMask(self.model,LidarExtrinsics(json.loads((ROOT/'config/lidar_calibration.json').read_text()),np.eye(4),alignment_verified=False))
        self.identities=sorted(self.meshes.meshes);self.ids={key:i for i,key in enumerate(self.identities)}
    def classify(self,points,packet,stamp,surface_margin=.04,environment_margin=.10):
        from scipy.spatial import cKDTree
        from environment import self_candidates
        p=np.asarray(points,float);positions={};pose_valid=False
        if isinstance(packet,dict) and packet.get('model_sha256')==self.model.sha256:
            try:
                age=abs(float(packet['observation_time'])-stamp)
                if math.isfinite(age) and age<=.3:
                    positions={k:v for k,v in packet['joint_positions_rad'].items() if 'robotiq' not in k};pose_valid=True
            except (KeyError,TypeError,ValueError):pass
        try:shapes=self.model.shapes(positions)
        except ValueError:shapes=self.model.shapes({});pose_valid=False
        # A possible unknown-pose volume blocks environment certification, but
        # can never become self-removal evidence.
        environment=~self_candidates(p,shapes,environment_margin)
        eligible=np.flatnonzero((p[:,2]>.12)&(np.linalg.norm(p[:,:2],axis=1)<2.2))
        best=np.full(len(p),np.inf);owner=np.full(len(p),-1,np.int32);local=np.zeros_like(p)
        if len(eligible):
            tree=cKDTree(p[eligible])
            for shape in shapes:
                if shape['mode']!='measured_or_fixed':continue
                T=np.array(shape['matrix']);lo,hi=np.array(shape['bounds']);center=T[:3,:3]@((lo+hi)/2)+T[:3,3];radius=np.linalg.norm(hi-lo)/2+surface_margin
                rows=np.array(tree.query_ball_point(center,radius),int)
                if not len(rows):continue
                ids=eligible[rows];q=(p[ids]-T[:3,3])@T[:3,:3];gap=np.linalg.norm(np.maximum(np.maximum(lo-q,q-hi),0),axis=1);use=gap<=surface_margin;ids=ids[use];q=np.ascontiguousarray(q[use],np.float64)
                if not len(ids):continue
                mesh=self.meshes.meshes[(shape['link'],shape['name'])];distances=triangle_distances(mesh,q)
                update=(distances<=surface_margin)&(distances<best[ids]);chosen=ids[update];best[chosen]=distances[update];owner[chosen]=self.ids[(shape['link'],shape['name'])];local[chosen]=q[update]
        return owner,local,environment,pose_valid
    def close(self):self.meshes.close()

class SelfReturnFilter:
    def __init__(self,allowed_codes=None,resolution=.05,surface_margin=.04,environment_margin=.10,local_resolution=.03,model_padding=0.):
        if not (all(math.isfinite(v) for v in (surface_margin,environment_margin,local_resolution,model_padding)) and 0 < surface_margin < environment_margin and local_resolution > 0 and model_padding>=0):raise ValueError('INVALID_SELF_GEOMETRY_MARGINS')
        self.surface_margin=surface_margin;self.environment_margin=environment_margin;self.local_resolution=local_resolution;self.model_padding=model_padding
        self.resolution=resolution;self.geometry=SurfaceGeometry();self.evidence=Evidence(allowed_codes=allowed_codes,resolution=resolution);self.first_stamp=None;self.frames=0;self.candidate_observations=0;self.removed_observations=0;self.last_compute_s=0.;self.max_compute_s=0.
    @classmethod
    def from_config(cls,config,**kwargs):
        return cls(surface_margin=float(config.get('surface_margin_m',.04)),environment_margin=float(config.get('environment_margin_m',.10)),local_resolution=float(config.get('local_resolution_m',.03)),model_padding=float(config.get('model_padding_m',0.)),**kwargs)
    def observe(self,points_base,T_map_base,packet,stamp):
        started=time.perf_counter();p=np.asarray(points_base,float);T=np.asarray(T_map_base,float)
        if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all() or T.shape!=(4,4) or not np.isfinite(T).all() or not np.isfinite(stamp):raise ValueError('INVALID_SELF_FILTER_FRAME')
        if not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(T[:3,:3]),1.,atol=1e-6):raise ValueError('INVALID_SELF_FILTER_RIGID_POSE')
        if self.first_stamp is None:self.first_stamp=stamp
        window=max(0,int(np.floor(stamp-self.first_stamp)));world=p@T[:3,:3].T+T[:3,3];codes=voxel_codes(world,self.resolution)
        owner,local,environment,valid=self.geometry.classify(p,packet,stamp,surface_margin=self.surface_margin+self.model_padding,environment_margin=self.environment_margin+self.model_padding);self.evidence.environment_observed(codes[environment],window)
        ids=np.flatnonzero(owner>=0);keep=np.ones(len(p),bool)
        if len(ids):
            keys=np.column_stack([owner[ids],np.floor(local[ids]/self.local_resolution).astype(int)]);unique,inverse=np.unique(keys,axis=0,return_inverse=True)
            for group,key in enumerate(unique):
                selected=ids[inverse==group];ready=self.evidence.candidate_observed(tuple(key),codes[selected],window,T[:2,3])
                if ready:keep[selected]=np.array([not self.evidence.can_remove(int(code)) for code in codes[selected]])
        self.frames+=1;self.candidate_observations+=len(ids)
        if self.evidence.capacity_exhausted:keep[:]=True
        self.removed_observations+=int((~keep).sum());self.last_compute_s=time.perf_counter()-started;self.max_compute_s=max(self.max_compute_s,self.last_compute_s)
        return keep
    def removable_codes(self,protected=()):return self.evidence.removal_codes(protected)
    def prune_recorder(self,recorder,protected=()):
        if not np.isclose(recorder.res,self.resolution):raise ValueError('SELF_FILTER_RESOLUTION_MISMATCH')
        removable=self.removable_codes(protected)
        if not removable:return 0
        keys=list(recorder.cells);centers=(np.array(keys,float)+.5)*recorder.res;codes=voxel_codes(centers,recorder.res);count=0
        for key,code in zip(keys,codes):
            if int(code) in removable:del recorder.cells[key];count+=1
        return count
    def summary(self):
        return dict(method='URDF_TRIANGLE_SURFACE_AND_LINK_LOCAL_TEMPORAL_SELF_EVIDENCE_V3',frames=self.frames,candidate_observations=self.candidate_observations,online_filtered_observations=self.removed_observations,last_compute_s=self.last_compute_s,max_compute_s=self.max_compute_s,model_sha256=self.geometry.model.sha256,surface_margin_m=self.surface_margin,model_padding_m=self.model_padding,effective_surface_band_m=self.surface_margin+self.model_padding,environment_witness_margin_m=self.environment_margin+self.model_padding,link_local_resolution_m=self.local_resolution,independent_window_s=1.,minimum_learning_windows=25,minimum_learning_span_s=45.,minimum_base_extent_m=.8,minimum_world_self_windows=2,environment_windows_preserve=2,no_trajectory_region_deletion=True,unknown_pose_never_self_evidence=True,alignment_independently_verified=False,**self.evidence.summary())
    def close(self):self.geometry.close()
