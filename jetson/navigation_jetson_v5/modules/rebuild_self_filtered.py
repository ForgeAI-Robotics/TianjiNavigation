"""Offline map correction from recorded frames. No ROS/network/control.
Self candidates are removed only from this derived, non-navigation map;
the native unverified-alignment policy and original raw data remain unchanged.
"""
import argparse,json,hashlib,sys,time
from pathlib import Path
import numpy as np
from common import ROOT,CONFIG,atomic_json,exclusive
from mapping_core import VoxelRecorder
LIB=ROOT.parent/'navigation_link_v2'

def candidate_removal(labels,joints,model_hash,stamp,unknown_label=5,self_label=2):
    if (not isinstance(joints,dict) or joints.get('model_sha256')!=model_hash
            or not isinstance(joints.get('joint_positions_rad'),dict)):
        return np.zeros(len(labels),bool),'NO_MATCHING_MEASURED_POSE'
    try:
        age=abs(float(joints['observation_time'])-stamp)
        if not np.isfinite(age) or age>.3:raise ValueError()
    except (KeyError,TypeError,ValueError):
        return np.zeros(len(labels),bool),'POSE_TIMESTAMP_MISSING_OR_STALE'
    return np.asarray(labels)==self_label,'MATCHING_RECORDED_POSE'

def run(source,destination):
    sys.path.insert(0,str(LIB))
    from local_model import LocalRobotModel,default_urdf_path
    from lidar_shapemask import LidarShapeMask,LidarExtrinsics,SELF_CANDIDATE,UNKNOWN_POSE
    lock=exclusive(ROOT/'runtime/offline_rebuild.lock')
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=False)
    model=LocalRobotModel(default_urdf_path(),LIB/'model_manifest.json',LIB/'collision_geometry.json')
    mask=LidarShapeMask(model,LidarExtrinsics(json.loads((ROOT/'config/lidar_calibration.json').read_text()),np.eye(4),alignment_verified=False))
    recorder=VoxelRecorder(destination,CONFIG['mapping']);recorder.validation_only=True
    meta=json.loads((source/'map.json').read_text());files=sorted((source/'keyframes').glob('*.npz'))
    rows=[];started=time.monotonic()
    try:
        for index,f in enumerate(files,1):
            with np.load(f,allow_pickle=False) as d:data={k:d[k] for k in d.files}
            p=data['points_base'];q=json.loads(str(data['joints_json']));stamp=float(data['observation_time']);sensor=str(data['sensor'])
            # Validate pose provenance before asking geometry to interpret it.
            _,reason=candidate_removal(np.zeros(len(p),np.uint8),q,model.sha256,stamp)
            if reason=='MATCHING_RECORDED_POSE':
                result=mask.classify(p,sensor,'model_base',q['joint_positions_rad'])
                removal,reason=candidate_removal(result['labels'],q,model.sha256,stamp)
                unknown=int((result['labels']==UNKNOWN_POSE).sum())
            else:removal=np.zeros(len(p),bool);unknown=len(p)
            recorder.ingest(p[~removal],stamp,data['pose'],sensor,data['sensor_origin_base'],q,int(data['odometry_epoch']))
            rows.append(dict(source_file=str(f.resolve()),original_points=len(p),self_candidate_points_removed=int(removal.sum()),unknown_points_preserved=unknown,pose_check=reason))
            if index%25==0 or index==len(files):print(f'Self filter {index}/{len(files)} | removed {sum(r["self_candidate_points_removed"] for r in rows)} | {time.monotonic()-started:.1f}s',flush=True)
    finally:mask.close()
    recorder.invalid_reason=recorder.invalid_reason or meta.get('invalid_reason') or 'OFFLINE_SELF_FILTER_REQUIRES_VALIDATION'
    result=recorder.save();report=dict(method='RECORDED_POSE_URDF_FIRST_SURFACE_CANDIDATES_BEFORE_MAP_ACCUMULATION',source=str(source.resolve()),model_sha256=model.sha256,calibration_sha256=hashlib.sha256((ROOT/'config/lidar_calibration.json').read_bytes()).hexdigest(),original_points=sum(r['original_points'] for r in rows),self_candidate_points_removed=sum(r['self_candidate_points_removed'] for r in rows),frames=len(rows),pose_unavailable_frames=sum(r['pose_check']!='MATCHING_RECORDED_POSE' for r in rows),self_tolerance_m=.012,alignment_independently_verified=False,visualization_only=True,navigation_usable=False,original_raw_frames_untouched=True,no_trajectory_corridor_deletion=True,elapsed_s=time.monotonic()-started,frame_report=rows)
    atomic_json(destination/'self_filter_report.json',report)
    result.update(derived_from=str(source.resolve()),self_filter_report='self_filter_report.json',navigation_usable=False,visualization_only=True)
    atomic_json(destination/'map.json',result)
    print(json.dumps({k:v for k,v in report.items() if k!='frame_report'},indent=2),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('source',type=Path);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();run(args.source,args.output)
