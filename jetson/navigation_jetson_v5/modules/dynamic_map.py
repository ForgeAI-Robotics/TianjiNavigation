"""Offline temporal hit / actual measured-ray consistency, not semantic people detection.
Inspired by visibility-based map cleaning; not a Removert/ERASOR implementation.
No ROS, network, motion commands or certified free-space output.
"""
import argparse,collections,hashlib,json,time
from datetime import datetime
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from common import ROOT,atomic_json,exclusive
from mapping_core import rotation,write_pcd
from saved_map import MapArchive

STATIC,UNCERTAIN,DYNAMIC=1,0,2

class VoxelLookup:
    def __init__(self,centers,resolution):
        self.res=float(resolution);self.centers=np.asarray(centers,float)
        keys=np.floor(self.centers/self.res).astype(np.int64);self.low=keys.min(0);self.shape=keys.max(0)-self.low+1;self.stride=np.array([self.shape[1]*self.shape[2],self.shape[2],1],np.int64)
        encoded=(keys-self.low)@self.stride;self.order=np.argsort(encoded);self.encoded=encoded[self.order];self.representatives=self.centers.copy()
    def lookup(self,points):
        keys=np.floor(np.asarray(points)/self.res).astype(np.int64)-self.low
        good=np.all((keys>=0)&(keys<self.shape),axis=1);result=np.full(len(keys),-1,np.int64)
        use=np.flatnonzero(good);encoded=keys[use]@self.stride;idx=np.searchsorted(self.encoded,encoded)
        found=idx<len(self.encoded);found[found]&=self.encoded[idx[found]]==encoded[found]
        result[use[found]]=self.order[idx[found]];return result

    def record_views(self,points,ids,origin):
        """Keep actual measurements, not a mean of incompatible object faces.
        Eight storage bins are an index only; a separate 10-degree test is
        required when a later beam tries to contradict any stored sample.
        """
        if not hasattr(self,'view_valid'):
            self.view_valid=np.zeros((len(self.centers),8),bool)
            self.view_points=np.zeros((len(self.centers),8,3),np.float32)
            self.view_directions=np.zeros_like(self.view_points)
        direction=np.asarray(points)-origin;length=np.linalg.norm(direction,axis=1)
        valid=length>.04;direction=direction[valid]/length[valid,None];p=np.asarray(points)[valid];ids=np.asarray(ids)[valid]
        bins=(np.floor((np.arctan2(direction[:,1],direction[:,0])+np.pi)/(np.pi/4)).astype(int))%8
        slots=ids*8+bins;unique,first=np.unique(slots,return_index=True);new=~self.view_valid.reshape(-1)[unique]
        unique=unique[new];first=first[new]
        self.view_valid.reshape(-1)[unique]=True
        self.view_points.reshape(-1,3)[unique]=p[first]
        self.view_directions.reshape(-1,3)[unique]=direction[first]

def free_ray_votes(lookup,origin,endpoints,max_range=10.,endpoint_margin=.15,beam_radius=.02,chunk=384):
    """Follow only real beams, never interpolate absent beams or clear occlusions.
    Voxel lookup supplies candidates. A near pass to the mean ORIGINAL hit
    location is required, avoiding grazing an empty corner of a coarse cell.
    """
    origin=np.asarray(origin,float);points=np.asarray(endpoints,float);mask=np.zeros(len(lookup.centers),bool)
    per_ray=origin.ndim==2
    if origin.shape not in [(3,),points.shape]:raise ValueError('SENSOR_ORIGIN_SHAPE_MISMATCH')
    step=lookup.res*.5
    for start in range(0,len(points),chunk):
        p=points[start:start+chunk];origins=origin[start:start+chunk] if per_ray else origin;delta=p-origins;ranges=np.linalg.norm(delta,axis=1);good=np.isfinite(ranges)&(ranges>endpoint_margin+step)
        delta=delta[good];ranges=ranges[good]
        if per_ray:origins=origins[good]
        if not len(ranges):continue
        unit=delta/ranges[:,None];stop=np.minimum(ranges-endpoint_margin,max_range)
        distance=np.arange(step,float(stop.max()),step)
        if not len(distance):continue
        samples=(origins[:,None,:] if per_ray else origins)+unit[:,None,:]*distance[None,:,None]
        allowed=distance[None,:]<stop[:,None];ray_index,sample_index=np.nonzero(allowed)
        sample_points=samples[ray_index,sample_index];ids=lookup.lookup(sample_points);found=ids>=0
        if not found.any():continue
        ids=ids[found];rays=ray_index[found];directions=unit[rays]
        if hasattr(lookup,'view_valid'):
            bins=(np.floor((np.arctan2(directions[:,1],directions[:,0])+np.pi)/(np.pi/4)).astype(int))%8
            keep=np.zeros(len(ids),bool)
            for shift in (-1,0,1):
                slot=(bins+shift)%8;reference=lookup.view_points[ids,slot]
                comparable=lookup.view_valid[ids,slot]&(np.einsum('ij,ij->i',directions,lookup.view_directions[ids,slot])>=np.cos(np.deg2rad(10.)))
                to_point=reference-(origins[rays] if per_ray else origins);along=np.einsum('ij,ij->i',to_point,directions)
                perpendicular2=np.maximum(0.,np.einsum('ij,ij->i',to_point,to_point)-along*along)
                keep|=comparable&(perpendicular2<=beam_radius**2)&(along>0)&(along<stop[rays])
        else:
            to_point=lookup.representatives[ids]-(origins[rays] if per_ray else origins)
            along=np.einsum('ij,ij->i',to_point,directions);perpendicular2=np.maximum(0.,np.einsum('ij,ij->i',to_point,to_point)-along*along)
            keep=(perpendicular2<=beam_radius**2)&(along>0)&(along<stop[rays])
        if hasattr(lookup,'surface_normals'):
            normals=lookup.surface_normals[ids];planar=np.linalg.norm(normals,axis=1)>.5
            keep&=~planar|(np.abs(np.einsum('ij,ij->i',normals,directions))>=np.sin(np.deg2rad(20.)))
        mask[ids[keep]]=True
    return mask

def classify(hits,free,hit_span,free_span,protected=None):
    fraction=free/np.maximum(1,hits+free)
    dynamic=(hits>0)&(free>=5)&(fraction>=.8)&(free_span>=3.)
    if protected is not None:dynamic&=~protected
    static=(hits>=3)&(hit_span>=1.5)&(fraction<=.2)&~dynamic
    labels=np.full(len(hits),UNCERTAIN,np.uint8);labels[static]=STATIC;labels[dynamic]=DYNAMIC;return labels

def persistent_structure(lookup,hits,span,minimum_hits=5,minimum_span=3.,neighbor_count=12,neighborhood_radius=.18):
    """Geometry protection, not object semantics or a certified shape model."""
    selected=(hits>=minimum_hits)&(span>=minimum_span)
    normal=np.zeros((len(hits),3),np.float32);linear=np.zeros(len(hits),bool)
    if selected.sum()<6:return normal,linear
    anchors=lookup.representatives[selected];tree=cKDTree(anchors)
    for start in range(0,len(hits),8192):
        points=lookup.representatives[start:start+8192];distance,index=tree.query(points,k=neighbor_count,distance_upper_bound=neighborhood_radius,workers=1);valid=np.isfinite(distance);count=valid.sum(1)
        neighbors=anchors[np.minimum(index,len(anchors)-1)];weights=valid[...,None]
        center=(neighbors*weights).sum(1)/np.maximum(1,count[:,None]);delta=(neighbors-center[:,None,:])*weights
        covariance=np.einsum('nki,nkj->nij',delta,delta)/np.maximum(1,count[:,None,None]);values,vectors=np.linalg.eigh(covariance)
        plane=(count>=6)&(values[:,1]>.000625)&(values[:,0]<.12*values[:,1])
        line=(count>=6)&(values[:,2]>.0025)&(values[:,1]<.1*values[:,2])
        normal[start:start+len(points)][plane]=vectors[plane,:,0]
        linear[start:start+len(points)]=line&selected[start:start+len(points)]
    return normal,linear

def nearby_surface_protection(lookup,cleared,world,origins,translation_error=.03,rotation_error_deg=.3):
    """Unknown within a range-dependent registration/voxel uncertainty band."""
    ids=np.flatnonzero(cleared)
    if not len(ids) or not len(world):return np.zeros(len(lookup.centers),bool)
    centers=lookup.representatives[ids];origins=np.asarray(origins)
    distance_to_sensor=np.min(np.linalg.norm(centers[:,None,:]-origins[None,:,:],axis=2),axis=1)
    radius=lookup.res*np.sqrt(3)/2+translation_error+distance_to_sensor*np.tan(np.deg2rad(rotation_error_deg))
    radius=np.minimum(radius,.15)
    distance,_=cKDTree(world).query(centers,distance_upper_bound=float(radius.max()),workers=1)
    protected=np.zeros(len(lookup.centers),bool);protected[ids[distance<=radius]]=True;return protected

def load_frame(path):
    with np.load(path,allow_pickle=False) as data:
        p=np.array(data['points_base'],float);pose=np.array(data['pose'],float);origin=np.array(data['sensor_origin_map'],float);stamp=float(data['observation_time'])
        matrix=np.array(data['pose_matrix'],float) if 'pose_matrix' in data else None
    world=p@rotation(pose[2]).T+[pose[0],pose[1],0.] if matrix is None else p@matrix[:3,:3].T+matrix[:3,3]
    return world,origin,stamp

def run(source,destination=None):
    lock=exclusive(ROOT/'runtime/dynamic_filter.lock')
    started=time.perf_counter();archive=MapArchive(source);meta=archive.meta;resolution=float(meta['resolution_m']);lookup=VoxelLookup(archive.points,resolution);n=len(archive.points)
    destination=Path(destination or ROOT/'maps'/('derived_dynamic_'+datetime.now().strftime('%Y%m%d_%H%M%S')));destination.mkdir(parents=True,exist_ok=False)
    records=[]
    for path in archive.keyframes:
        with np.load(path,allow_pickle=False) as a:records.append((float(a['observation_time']),path))
    records.sort();first=records[0][0];groups=collections.defaultdict(list)
    for stamp,path in records:groups[int(np.floor((stamp-first)+1e-6))].append(path)
    hits=np.zeros(n,np.uint16);free=np.zeros(n,np.uint16);hit_first=np.full(n,np.inf);hit_last=np.full(n,-np.inf);free_first=np.full(n,np.inf);free_last=np.full(n,-np.inf);sums=np.zeros((n,3));counts=np.zeros(n,np.int64)
    window_hits={};unmatched=0;total=0
    for bucket,paths in groups.items():
        occupied=np.zeros(n,bool)
        for path in paths:
            world,origin,stamp=load_frame(path);ids=lookup.lookup(world);good=ids>=0;unmatched+=int((~good).sum());total+=len(ids);ids=ids[good];world=world[good];occupied[ids]=True
            lookup.record_views(world,ids,origin[good] if origin.ndim==2 else origin)
            counts+=np.bincount(ids,minlength=n)
            for axis in range(3):sums[:,axis]+=np.bincount(ids,weights=world[:,axis],minlength=n)
        hits[occupied]+=1;hit_first[occupied]=np.minimum(hit_first[occupied],bucket);hit_last[occupied]=np.maximum(hit_last[occupied],bucket);window_hits[bucket]=occupied
    observed=counts>0;lookup.representatives[observed]=sums[observed]/counts[observed,None]
    span=np.maximum(0.,hit_last-hit_first);anchors=(hits>=10)&(span>=8.);rejected=[];timings=[]
    lookup.surface_normals,_=persistent_structure(lookup,hits,span)
    # Even a rarely observed thin leg must stay UNKNOWN rather than being
    # removed merely because most subsequent sparse beams missed it.
    _,thin_protection=persistent_structure(lookup,hits,span,minimum_hits=1,minimum_span=0.)
    _,wide_thin=persistent_structure(lookup,hits,span,minimum_hits=1,minimum_span=0.,neighbor_count=32,neighborhood_radius=.30)
    thin_protection|=wide_thin
    protected_votes=np.zeros(n,np.uint16)
    print('Hit pass complete:',len(groups),'independent time windows,',int(anchors.sum()),'persistent anchors',flush=True)
    for number,(bucket,paths) in enumerate(groups.items(),1):
        t=time.perf_counter();cleared=np.zeros(n,bool);observations=[];origins=[]
        for path in paths:
            world,origin,stamp=load_frame(path);observations.append(world);origins.append(origin.mean(0) if origin.ndim==2 else origin)
            budget=meta.get('negative_ray_budget_per_frame')
            if budget is not None and len(world)>budget:
                chosen=np.linspace(0,len(world)-1,int(budget),dtype=int);ray_points=world[chosen];ray_origins=origin[chosen] if origin.ndim==2 else origin
            else:ray_points=world;ray_origins=origin
            cleared|=free_ray_votes(lookup,ray_origins,ray_points)
        occupied=window_hits[bucket];cleared&=~occupied
        nearby=nearby_surface_protection(lookup,cleared,np.concatenate(observations),origins);protected_votes[nearby]+=1;cleared&=~nearby
        visible_anchor=(cleared|occupied)&anchors;contradiction=float(np.count_nonzero(cleared&anchors)/max(1,np.count_nonzero(visible_anchor)))
        if visible_anchor.sum()>=100 and contradiction>.15:
            rejected.append(dict(window=bucket,persistent_anchor_contradiction=contradiction));cleared[:]=False
        free[cleared]+=1;free_first[cleared]=np.minimum(free_first[cleared],bucket);free_last[cleared]=np.maximum(free_last[cleared],bucket);timings.append(time.perf_counter()-t)
        if number%5==0 or number==len(groups):print(f'Ray pass {number}/{len(groups)} · {time.perf_counter()-started:.1f}s',flush=True)
    free_span=np.maximum(0.,free_last-free_first);labels=classify(hits,free,span,free_span,thin_protection);keep=labels!=DYNAMIC;static=labels[keep]==STATIC
    points=archive.points;write_pcd(destination/'observed_all.pcd',points[keep]);write_pcd(destination/'candidate_static.pcd',points[labels==STATIC]);write_pcd(destination/'dynamic_candidates.pcd',points[labels==DYNAMIC]);write_pcd(destination/'uncertain.pcd',points[labels==UNCERTAIN])
    np.savez_compressed(destination/'voxels.npz',points=points[keep],candidate_static=static)
    np.savez_compressed(destination/'dynamic_evidence.npz',points=points,labels=labels,hits=hits,free=free,hit_span_s=span,free_span_s=free_span,nearby_surface_protected_votes=protected_votes,thin_structure_protected=thin_protection)
    (destination/'keyframes').symlink_to(archive.folder.resolve()/'keyframes',target_is_directory=True)
    report=dict(method='VIEW_AWARE_ACTUAL_BEAM_TEMPORAL_VOXEL_CONSISTENCY_V2',source=str(archive.folder),independent_window_s=1.,windows=len(groups),source_frames=len(records),original_voxels=n,retained_voxels=int(keep.sum()),dynamic_candidates=int(np.count_nonzero(labels==DYNAMIC)),static_confirmed=int(np.count_nonzero(labels==STATIC)),uncertain=int(np.count_nonzero(labels==UNCERTAIN)),rejected_visibility_windows=rejected,source_hit_lookup_unmatched_fraction=unmatched/max(1,total),max_cleanup_range_m=10.,beam_radius_m=.02,return_endpoint_exclusion_m=.15,free_fraction_threshold=.8,minimum_free_windows=5,minimum_free_span_s=3.,elapsed_s=time.perf_counter()-started,mean_window_processing_s=float(np.mean(timings)),same_view_max_angle_deg=10.,reference='ACTUAL_OBSERVED_POINT_PER_AZIMUTH_BIN_NOT_AVERAGED_FACES',assumed_translation_error_m=.03,assumed_rotation_error_deg=.3,near_surface_protected_vote_count=int(protected_votes.sum()),thin_structure_protected_voxels=int(thin_protection.sum()),planar_grazing_min_angle_deg=20.,per_point_deskew_available=bool(meta.get('raw_beam_reference')),negative_ray_budget_per_frame=meta.get('negative_ray_budget_per_frame'),ray_geometry=meta.get('ray_geometry','original keyframe measured endpoints with scan reference origin'),semantic_person_detection=False,navigation_usable=False,free_space_certified=False,source_invalid_reason=meta.get('invalid_reason'),source_map_hash=hashlib.sha256((archive.folder/'voxels.npz').read_bytes()).hexdigest(),human_labelled_accuracy_available=False)
    atomic_json(destination/'dynamic_report.json',report)
    result=dict(meta,cells=int(keep.sum()),candidate_static_cells=int(static.sum()),voxel_file_sha256=hashlib.sha256((destination/'voxels.npz').read_bytes()).hexdigest(),dynamic_removal=report,derived_from=str(archive.folder),dynamic_evidence='dynamic_evidence.npz',validation_only=True,static_map_verified=False,free_space_certified=False)
    atomic_json(destination/'map.json',result)
    atomic_json(ROOT/'runtime/latest_dynamic_map.json',dict(folder=str(destination),map=str(destination/'map.json')))
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True);return destination,report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path,nargs='?');parser.add_argument('--output',type=Path);args=parser.parse_args()
    if args.source is None:
        pointer=ROOT/'runtime/latest_map.json'
        args.source=Path(json.loads(pointer.read_text())['folder']) if pointer.exists() else ROOT/'maps/imported_s100_20261027_142818_INVALID'
    run(args.source,args.output)
