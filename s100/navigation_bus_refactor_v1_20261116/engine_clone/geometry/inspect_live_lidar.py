#!/usr/bin/env python3
"""Offline inspection of a passive LiDAR capture; never publishes filtered data."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree,ConvexHull
from lidar_shapemask import LidarExtrinsics,LidarShapeMask,LABEL_NAMES,SELF_CANDIDATE,ENVIRONMENT,PADDING,UNKNOWN_POSE
from local_model import LocalRobotModel,default_urdf_path

ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('capture',type=Path);args=parser.parse_args()
    meta=json.loads((args.capture/'capture.json').read_text());raw=np.load(args.capture/'pointclouds.npz')
    model=LocalRobotModel(default_urdf_path(),ROOT/'model_manifest.json',ROOT/'collision_geometry.json')
    mask=LidarShapeMask(model,LidarExtrinsics(json.loads((ROOT/'lidar_calibration.json').read_text()),np.eye(4),alignment_verified=False))
    history={}
    joint_file=args.capture/'joint_history.json'
    if joint_file.exists():
        for sample in json.loads(joint_file.read_text())['samples']:
            if 0<=sample['received_at']-sample['stamp']<=.3:
                arms={k:v for k,v in sample['positions'].items() if k.startswith('Joint')}
                if len(arms)==14:history[sample['stamp']]=arms
    for records in meta['streams'].values():
        for frame in records:
            state=frame.get('receiver_status',{})
            if not joint_file.exists() and state.get('arm_fresh') and state.get('model_hash_match'):
                stamp=state['received_at']-state['joint_header_age_s']
                history[stamp]={k:v for k,v in state['joint_positions_rad'].items() if k.startswith('Joint')}
    times=np.array(sorted(history));rows=[];latest={};positions={};merged=meta['streams']['merged']
    if not len(times):raise RuntimeError('No measured pose history in capture')
    try:
        for sensor in ('front','back'):
            for i,frame in enumerate(meta['streams'][sensor]):
                if frame['frame']!='lidar_'+sensor:raise ValueError('Unexpected raw LiDAR frame')
                j=int(np.searchsorted(times,frame['stamp']));positions={};association='UNKNOWN_POSE_NOT_BRACKETED'
                if 0<j<len(times) and times[j]-times[j-1]<=.4:
                    a,b=history[times[j-1]],history[times[j]]
                    for name in a.keys()&b.keys():
                        if abs(a[name]-b[name])<=np.deg2rad(.5):
                            fraction=(frame['stamp']-times[j-1])/(times[j]-times[j-1])
                            positions[name]=float(a[name]+fraction*(b[name]-a[name]))
                    association='OFFLINE_BRACKETED_STATIONARY_ARMS_ONLY'
                points=raw[sensor][raw[sensor+'_frame_index']==i]
                result=mask.classify(points,sensor,'sensor',positions);summary=result['summary']
                summary.update(frame=i,stamp=frame['stamp'],pose_association=association,measured_arm_joints=len(positions))
                # Check whether candidate returns are still present at the live merged input.
                if merged:
                    m=min(range(len(merged)),key=lambda k:abs(merged[k]['stamp']-frame['stamp']))
                    cloud=raw['merged'][raw['merged_frame_index']==m];cloud=cloud[np.isfinite(cloud).all(1)]
                    candidate=result['points_model'][result['labels']==SELF_CANDIDATE]
                    if len(cloud) and len(candidate):
                        distances=cKDTree(cloud).query(candidate)[0]
                        summary['candidate_match_in_merged']=dict(stamp_skew_s=abs(merged[m]['stamp']-frame['stamp']),
                            within_5mm=int((distances<=.005).sum()),candidates=len(candidate),median_distance_m=float(np.median(distances)))
                rows.append(summary);latest[sensor]=result
                np.savez_compressed(args.capture/(sensor+'_'+str(i)+'_classified.npz'),
                    points_model=result['points_model'],labels=result['labels'],expected_range_m=result['expected_range_m'],removal_mask=result['removal_mask'])
    finally:mask.close()
    sensors={}
    for sensor in ('front','back'):
        values=[r for r in rows if r['sensor_id']==sensor]
        sensors[sensor]=dict(frames=len(values),points_per_frame=[min(r['points'] for r in values),max(r['points'] for r in values)],
            self_candidate_per_frame=[min(r['labels']['SELF_CANDIDATE'] for r in values),max(r['labels']['SELF_CANDIDATE'] for r in values)],
            removed_points=sum(r['removed_points'] for r in values),labels_total={k:sum(r['labels'][k] for r in values) for k in LABEL_NAMES},
            frames_with_bracketed_arms=sum(r['measured_arm_joints']==14 for r in values))
    report=dict(mode='live captured data, offline preview; production navigation unchanged',sensors=sensors,
        alignment_verified=False,actual_point_deletion_enabled=False,development_output_in_production_graph=any('tianji_dev/navigation_v2' in x['topic'] for x in meta['pointcloud_topic_graph']),
        joint_max_motion_rad=max(np.ptp([h[n] for h in history.values() if n in h]) for n in next(iter(history.values()))),
        odometry_samples=len(meta['odometry']),motion_commands_sent=0,results=rows,
        limitations=['Arm poses interpolated only between nearby stationary recorded measurements; no extrapolation.',
                     'Head/finger/steering not trusted in this check; missing state remains bounded unknown.',
                     'Labels are model candidates, not independently verified self-return ground truth.',
                     'No production points deleted; merged presence comparison requires considering timestamp skew.'])
    (args.capture/'filter_check.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    for ax,dims in zip(axes,[(0,1),(0,2)]):
        for label,color,title in [(ENVIRONMENT,'0.65','environment'),(UNKNOWN_POSE,'royalblue','unknown pose'),(PADDING,'orange','padding'),(SELF_CANDIDATE,'red','self candidate')]:
            points=np.concatenate([r['points_model'][r['labels']==label] for r in latest.values()]);keep=np.isfinite(points).all(1)&(np.linalg.norm(points[:,:2],axis=1)<1.5)
            points=points[keep];ax.scatter(points[:,dims[0]],points[:,dims[1]],s=4 if label==SELF_CANDIDATE else 1,alpha=.7 if label==SELF_CANDIDATE else .25,c=color,label=title)
        for shape in model.shapes(positions):
            if shape['mode']!='measured_or_fixed':continue
            p=np.unique(np.asarray(shape['world_corners'])[:,dims],axis=0)
            if len(p)>2:
                try:
                    outline=p[ConvexHull(p).vertices];outline=np.vstack([outline,outline[0]]);ax.plot(*outline.T,c='black',lw=.5,alpha=.35)
                except Exception:pass
        ax.set_xlabel('X (m)');ax.set_ylabel(('Y' if dims[1]==1 else 'Z')+' (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
        ax.set_xlim(-.9,.9);ax.set_ylim((-1.,1.) if dims[1]==1 else (0.,1.6))
    axes[0].legend(markerscale=3,fontsize=8);fig.suptitle('Current LiDAR: model candidates only; actual removed points = 0; alignment unverified')
    fig.tight_layout();fig.savefig(args.capture/'self_candidate_views.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))

if __name__=='__main__':main()
