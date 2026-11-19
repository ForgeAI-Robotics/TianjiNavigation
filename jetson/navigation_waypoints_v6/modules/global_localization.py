"""Map-wide height-banded correlation + PCL GICP; passive acquisition only."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import json,time,math,subprocess,hashlib
import numpy as np
from scipy.fft import rfftn,irfftn
from scipy.ndimage import distance_transform_edt,maximum_filter
from scipy.spatial import cKDTree
from paths import ROOT,NAV,EVAL
from client import SensorStream
from common import atomic_json
from extrinsics import transforms
from saved_map import base_transform
from localization_tracker import voxel,usable,residuals

def correlation_candidates(source,target,max_candidates=12,res=.20,yaw_step=5):
 # Sparse distant stray returns do not enlarge the FFT indefinitely. The exact
 # searched support bounds are recorded; outside that mapped support is lost.
 limits=np.percentile(target[:,:2],[.1,99.9],axis=0)
 reference=target[((target[:,:2]>=limits[0])&(target[:,:2]<=limits[1])).all(1)]
 bound=float(max(np.abs(reference[:,:2]).max(),np.abs(source[:,:2]).max()))+float(np.abs(source[:,:2]).max())+2
 size=2**int(np.ceil(np.log2(max(64,2*bound/res))))
 if size>512:raise ValueError('GLOBAL_SEARCH_MAP_EXTENT_TOO_LARGE')
 mid=size//2;bands=[(.15,.55),(.55,.9),(.9,1.4),(1.4,2.08)]
 def indices(q):
  ids=np.floor(q[:,:2]/res+.5).astype(int)+mid;valid=((ids>=0)&(ids<size)).all(1);return ids[valid]
 references=[]
 for lo,hi in bands:
  grid=np.zeros((size,size),bool);ij=indices(reference[(reference[:,2]>=lo)&(reference[:,2]<hi)])
  if len(ij):grid[tuple(ij.T)]=True
  distance=distance_transform_edt(~grid)*res;likelihood=np.exp(-distance**2/(2*.16**2));likelihood[distance>.5]=0;references.append(rfftn(likelihood))
 axis=np.where(np.arange(size)<mid,np.arange(size),np.arange(size)-size)*res
 xx,yy=np.meshgrid(axis,axis,indexing='ij');allowed=(xx>=limits[0,0]-1)&(xx<=limits[1,0]+1)&(yy>=limits[0,1]-1)&(yy<=limits[1,1]+1)
 candidates=[];structure=voxel(source[source[:,2]>.15],.20)
 if len(structure)<150:raise ValueError('INSUFFICIENT_GLOBAL_LOCALIZATION_STRUCTURE')
 for degrees in range(0,360,yaw_step):
  a=math.radians(degrees);c,s=math.cos(a),math.sin(a);q=structure.copy();q[:,:2]=q[:,:2]@np.array([[c,-s],[s,c]]).T
  score=np.zeros((size,size));count=0
  for (lo,hi),reference_fft in zip(bands,references):
   grid=np.zeros((size,size));ij=indices(q[(q[:,2]>=lo)&(q[:,2]<hi)])
   if len(ij):grid[tuple(ij.T)]=1
   count+=grid.sum();score+=irfftn(reference_fft*np.conj(rfftn(grid)),s=grid.shape).real
  score/=max(1,count);score[~allowed]=-1
  ids=np.flatnonzero((score==maximum_filter(score,size=3,mode='wrap'))&allowed);ids=ids[np.argsort(score.ravel()[ids])[-8:]]
  for index in ids:
   i,j=np.unravel_index(index,score.shape);candidates.append(dict(pose=[float(axis[i]),float(axis[j]),a],score=float(score[i,j])))
 candidates.sort(key=lambda r:r['score'],reverse=True);selected=[]
 for row in candidates:
  if any(np.linalg.norm(np.array(row['pose'][:2])-p['pose'][:2])<.65 and abs(math.atan2(math.sin(row['pose'][2]-p['pose'][2]),math.cos(row['pose'][2]-p['pose'][2])))<math.radians(25) for p in selected):continue
  selected.append(row)
  if len(selected)>=max_candidates:break
 return selected,dict(search_support_xy_m=limits.tolist(),fft_cells=size,resolution_m=res,yaw_step_deg=yaw_step,scope='map-wide supported structure; no previous-pose or target-position prior')

def acquire(output):
 output.mkdir(parents=True,exist_ok=True);stream=SensorStream();stream.remote.stdout.channel.settimeout(3);rows=[];counts={'front':0,'back':0};start=time.monotonic()
 try:
  while min(counts.values())<3 and time.monotonic()-start<15:
   h,p=stream.receive()
   if h.get('kind')!='frame' or p is None or counts[h['sensor']]>=3:continue
   row=dict(header=h,file=str(output/f'scan_{len(rows):02d}_{h["sensor"]}.npz'));np.savez_compressed(row['file'],points_sensor=p);rows.append(row);counts[h['sensor']]+=1
 finally:stream.close()
 if min(counts.values())<3:raise ValueError('BOTH_LIDARS_REQUIRE_THREE_FRESH_SCANS')
 identities={(r['header']['boot_id'],r['header']['epoch'],r['header']['calibration_sha256']) for r in rows}
 if len(identities)!=1:raise ValueError('ODOMETRY_OR_CALIBRATION_CHANGED_DURING_CAPTURE')
 poses=np.array([r['header']['pose'] for r in rows]);a=np.unwrap(poses[:,2])
 if np.linalg.norm(np.ptp(poses[:,:2],axis=0))>.02 or np.ptp(a)>math.radians(1):raise ValueError('GLOBAL_LOCALIZATION_REQUIRES_STATIC_CAPTURE')
 atomic_json(output/'acquisition.json',dict(rows=rows,counts=counts,motion_commands_sent=0))
 return rows

def measured_joints():
 import sys
 client=NAV.parent.parent/'tianji_upper_client/tianji_m6_teleop_delivery_20260902'
 sys.path.insert(0,str(client/'src'))
 import tianji_state_ipc as ipc
 from shapemask_gripper_binding import GripperUrdfBinding
 from shapemask_head_control import load_settings,angle_delta
 from snapshot_builder import local_model
 model=local_model();sample=ipc.bridge_pose();status=ipc.alive_status()
 if not sample or not sample.get('pose_valid') or not status or status.get('whole_model_sha256')!=model.sha256:raise ValueError('FRESH_MEASURED_ARM_POSTURE_REQUIRED')
 q={f'Joint{i}_{side}':math.radians(v) for side,arm in zip(('L','R'),sample['joints_deg']) for i,v in enumerate(arm,1)}
 binding=GripperUrdfBinding(model.urdf)
 for side,data in status.get('gripper_raw_feedback',{}).items():
  if 0<=time.time()-data.get('received_at',0)<.4 and data.get('mapping_valid'):q[binding.joints[side][0]]=binding.angle(side,data)
 try:
  head=json.loads((client/'logs/shapemask_head_state.json').read_text())
  if head.get('connected') and 0<=time.time()-head['received_at']<.3:q['head_pitch_joint']=math.radians(angle_delta(head['mechanical_deg'],load_settings()))
 except (OSError,ValueError,KeyError):pass
 return dict(joint_positions_rad=q,model_sha256=model.sha256,observation_time=sample['received_at'],live=False,source='recorded_current_measured_feedback')

def localize(map_folder,output):
 output.mkdir(parents=True,exist_ok=True);began=time.monotonic();rows=acquire(output);matrices=transforms();latest=max(rows,key=lambda r:r['header']['observation_time'])['header'];ref=base_transform(latest['pose']);groups={'front':[],'back':[]}
 expected_hash=hashlib.sha256((NAV/'config/lidar_calibration.json').read_bytes()).hexdigest()
 if latest['calibration_sha256']!=expected_hash:raise ValueError('LIDAR_EXTRINSICS_MISMATCH')
 for row in rows:
  h=row['header'];s=h['sensor']
  with np.load(row['file']) as f:p=f['points_sensor']
  T=np.linalg.inv(ref)@base_transform(h['pose'])@matrices[s];groups[s].append(usable(p@T[:3,:3].T+T[:3,3]))
 training=voxel(np.concatenate([p for parts in groups.values() for p in parts[:2]]),.10)
 held={s:voxel(parts[2],.08) for s,parts in groups.items()}
 map_meta=json.loads((map_folder/'map.json').read_text());digest=hashlib.sha256((map_folder/'voxels.npz').read_bytes()).hexdigest()
 if digest!=map_meta['voxel_file_sha256']:raise ValueError('MAP_HASH_MISMATCH')
 with np.load(map_folder/'voxels.npz') as f:target=f['points']
 fine_target=np.ascontiguousarray(target[(target[:,2]>-.08)&(target[:,2]<2.08)],dtype='<f4')
 target=voxel(fine_target,.08)
 seeds,search=correlation_candidates(training,target);atomic_json(output/'global_seeds.json',dict(seeds=seeds,search=search))
 training.tofile(output/'source.xyz');target.tofile(output/'target.xyz')
 with (output/'seeds.txt').open('w') as f:
  for s in seeds:f.write(f'{s["pose"][0]} {s["pose"][1]} 0 {s["pose"][2]}\n')
 subprocess.run([str(EVAL/'native/gicp'),str(output/'source.xyz'),str(output/'target.xyz'),str(output/'seeds.txt'),str(output/'fits.txt'),'.45'],check=True,timeout=50,stdout=(output/'gicp.log').open('w'))
 tree=cKDTree(target);solutions=[]
 for row in np.atleast_2d(np.loadtxt(output/'fits.txt')):
  T=row[4:].reshape(4,4);m={s:residuals(p,T,tree) for s,p in held.items()};q=training[training[:,2]>.15];dist,_=tree.query(q@T[:3,:3].T+T[:3,3]);score=float(np.exp(-dist**2/(2*.10**2)).mean())
  plausible=bool(row[1]) and abs(T[2,3])<.15 and T[2,2]>math.cos(math.radians(3))
  passed=plausible and all(v['overlap']>.7 and v['rmse']<.08 and v['observable'] for v in m.values())
  solutions.append(dict(T_map_base=T.tolist(),score=score,valid=bool(passed),quality=m,seed=int(row[0])))
 solutions.sort(key=lambda s:(s['valid'],s['score']),reverse=True);best=solutions[0];T=np.array(best['T_map_base']);alternatives=[]
 for s in solutions[1:]:
  other=np.array(s['T_map_base']);dyaw=abs(math.atan2(math.sin(math.atan2(T[1,0],T[0,0])-math.atan2(other[1,0],other[0,0])),math.cos(math.atan2(T[1,0],T[0,0])-math.atan2(other[1,0],other[0,0]))))
  if np.linalg.norm(T[:2,3]-other[:2,3])>.4 or dyaw>math.radians(10):alternatives.append(s)
 gap=best['score']-max((s['score'] for s in alternatives),default=0.)
 accepted=best['valid'] and gap>.05
 coarse_T=T.copy();fine_quality=None
 if accepted:
  fine_target.tofile(output/'fine_target.xyz');(output/'fine_seed.txt').write_text(f'{T[0,3]} {T[1,3]} {T[2,3]} {math.atan2(T[1,0],T[0,0])}\n')
  with (output/'fine_gicp.log').open('w') as log:
   subprocess.run([str(EVAL/'native/gicp'),str(output/'source.xyz'),str(output/'fine_target.xyz'),str(output/'fine_seed.txt'),str(output/'fine_fit.txt'),'.20'],check=True,timeout=8,stdout=log)
  row=np.loadtxt(output/'fine_fit.txt');fine=row[4:].reshape(4,4);fine_tree=cKDTree(fine_target);fine_quality={s:residuals(p,fine,fine_tree) for s,p in held.items()}
  accepted=bool(row[1]) and np.linalg.norm(fine[:2,3]-T[:2,3])<.10 and abs(fine[2,3])<.15 and fine[2,2]>math.cos(math.radians(3)) and all(v['overlap']>.75 and v['rmse']<.065 and v['observable'] for v in fine_quality.values())
  if accepted:T=fine
 pose=dict(status='GLOBAL_LOCALIZED' if accepted else 'GLOBAL_LOCALIZATION_REJECTED',map_folder=str(map_folder),map_voxel_sha256=digest,frame_id=map_meta['frame_id'],T_map_base=T.tolist(),pose=[float(T[0,3]),float(T[1,3]),math.atan2(T[1,0],T[0,0])],score_gap=gap,quality=fine_quality or best['quality'],coarse_T_map_base=coarse_T.tolist(),fine_refinement=bool(fine_quality),solutions=solutions,search=search,observation_time=latest['observation_time'],wheel_pose=latest['pose'],source_boot_id=latest['boot_id'],odometry_epoch=latest['epoch'],calibration_sha256=expected_hash,compute_s=time.monotonic()-began,execution_permission=False,motion_commands_sent=0)
 atomic_json(output/'localization.json',pose)
 if not accepted:raise ValueError('GLOBAL_LOCALIZATION_AMBIGUOUS_OR_LOW_QUALITY: '+str(output/'localization.json'))
 pose['saved_pose']=measured_joints();atomic_json(output/'localization.json',pose)
 clouds=np.concatenate([p for parts in groups.values() for p in parts]);np.savez_compressed(output/'live_cloud.npz',points_base=clouds,points_map=clouds@T[:3,:3].T+T[:3,3])
 atomic_json(ROOT/'runtime/latest_localization.json',dict(folder=str(output),result=str(output/'localization.json')))
 return pose
