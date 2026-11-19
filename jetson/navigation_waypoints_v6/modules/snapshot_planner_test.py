"""Passive frozen-scene planner tests in a new local frame. No actuator API."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import json,time,math,hashlib,subprocess,argparse,shutil
from pathlib import Path
import numpy as np
from paths import ROOT,NAV
from common import atomic_json,exclusive
from global_localization import acquire,measured_joints
from saved_map import base_transform
from extrinsics import transforms
from prepare import prepare,reference_grid,downsample_centers
from mapping_core import write_pcd

CASES=[('forward_08','前方0.8米',[.8,0.,0.]),('forward_15','前方1.5米',[1.5,0.,0.]),('left_08','左侧0.8米',[0.,.8,0.]),('right_08','右侧0.8米',[0.,-.8,0.]),('back_08','后方0.8米',[-.8,0.,0.]),('back_02','后方0.2米小步',[-.2,0.,0.])]

def run():
 stamp=time.strftime('%Y%m%d_%H%M%S');out=ROOT/'results'/('new_environment_snapshot_'+stamp);capture=out/'capture';rows=acquire(capture);saved=measured_joints()
 h=max(rows,key=lambda r:r['header']['observation_time'])['header'];reference=base_transform(h['pose']);M=transforms();scans=[]
 expected=hashlib.sha256((NAV/'config/lidar_calibration.json').read_bytes()).hexdigest()
 if h['calibration_sha256']!=expected:raise ValueError('LIDAR_EXTRINSICS_MISMATCH')
 for row in rows:
  with np.load(row['file']) as data:p=np.asarray(data['points_sensor'])
  T=np.linalg.inv(reference)@base_transform(row['header']['pose'])@M[row['header']['sensor']];p=p@T[:3,:3].T+T[:3,3];p=p[np.isfinite(p).all(1)&(np.linalg.norm(p,axis=1)<12.)];scans.append(p)
 raw=np.concatenate(scans);cloud=downsample_centers(raw,.05);folder=out/'snapshot_map';folder.mkdir();np.savez_compressed(folder/'voxels.npz',points=cloud)
 digest=hashlib.sha256((folder/'voxels.npz').read_bytes()).hexdigest();frame='frozen_current_base_'+stamp
 atomic_json(folder/'map.json',dict(schema_version=1,voxel_file_sha256=digest,frame_id=frame,resolution_m=.05,frames=len(rows),cells=len(cloud),navigation_usable=False,coverage_complete=False,validation_only=True,source='stationary dual-lidar snapshot; not a globally localized map'))
 write_pcd(folder/'snapshot_raw.pcd',raw)
 loc=dict(status='LOCAL_SNAPSHOT_ONLY',map_folder=str(folder),map_voxel_sha256=digest,frame_id=frame,T_map_base=np.eye(4).tolist(),pose=[0.,0.,0.],wheel_pose=h['pose'],observation_time=h['observation_time'],saved_pose=saved,calibration_sha256=expected,source_boot_id=h['boot_id'],odometry_epoch=h['epoch'],execution_permission=False,scope='current frozen robot frame, no relation to original waypoints')
 atomic_json(capture/'localization.json',loc);cfg=json.loads((ROOT/'config/motion_policy.json').read_text());results=[]
 for key,name,goal in CASES:
  case=out/key;scene=case/'scene';case.mkdir();(case/'localization').symlink_to(capture,target_is_directory=True)
  target=dict(id=key,name=name,pose=goal,map_folder=str(folder),map_voxel_sha256=digest,frame_id=frame,ephemeral_local_test=True)
  recorded,world=prepare(loc,target,scene,assume_clearance=True,local_snapshot=True)
  grid,origin,res=reference_grid(recorded);packet=dict(world=world,goal=goal,motion_policy=cfg,grid=dict(origin=list(origin),width=grid.shape[1],height=grid.shape[0],resolution=res,data=grid.ravel().tolist()))
  planning=case/'planning';planning.mkdir();atomic_json(planning/'hybrid_input.json',packet);began=time.monotonic()
  subprocess.run([str(ROOT/'native/hybrid'),str(planning/'hybrid_input.json'),str(planning/'hybrid_result.json')],check=True,timeout=2*cfg['search_budget_s_per_stage']+15)
  native=json.loads((planning/'hybrid_result.json').read_text());selected=native['selected'];summary=dict(name=name,goal=goal,folder=str(case),found=bool(selected),attempts=[{k:v for k,v in a.items() if k not in ['path','primitives']} for a in native['attempts']],compute_s=time.monotonic()-began,selected=selected)
  atomic_json(planning/'plan.json',dict(selected=selected,attempts=summary['attempts'],scope='local frozen snapshot test only',execution_permission=False,motion_commands_sent=0))
  from render_plan import render
  render(case);results.append(summary);print(name+': '+('PATH_FOUND' if selected else str(summary['attempts'])),flush=True)
 report=dict(scope='new-environment stationary snapshot; NOT global localization or executable navigation',frame_id=frame,folder=str(out),lidar_frames=len(rows),raw_points=len(raw),snapshot_cells=len(cloud),old_waypoints_used=False,clearance='ASSUMED_CLEAR_BY_USER_FOR_PREVIEW',cases=results,execution_permission=False,motion_commands_sent=0)
 atomic_json(out/'report.json',report);atomic_json(ROOT/'runtime/latest_snapshot_test.json',dict(folder=str(out),report=str(out/'report.json')))
 render_overview(out,report)
 desktop=Path('/home/nvidia/Desktop/导航规划结果')/('新环境静态测试_'+stamp);desktop.mkdir()
 for file in [out/'report.json',out/'规划器测试总览.png'] :shutil.copy2(file,desktop/file.name)
 for key,_,_ in CASES:shutil.copy2(out/key/'route.png',desktop/(key+'.png'))
 print('RESULT_FOLDER '+str(out),flush=True);return report

def render_overview(out,report):
 import warnings
 warnings.filterwarnings('ignore',message='Unable to import Axes3D')
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib import font_manager
 plt.rcParams['font.family']=font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name();plt.rcParams['axes.unicode_minus']=False
 fig,axes=plt.subplots(2,3,figsize=(17,10),dpi=130)
 for ax,row in zip(axes.ravel(),report['cases']):
  case=Path(row['folder']);packet=json.loads((case/'planning/hybrid_input.json').read_text());g=packet['grid'];grid=np.asarray(g['data']).reshape(g['height'],g['width']);origin=g['origin'];res=g['resolution'];ax.imshow(np.where(grid<0,0,1),origin='lower',extent=[origin[0],origin[0]+g['width']*res,origin[1],origin[1]+g['height']*res],cmap=matplotlib.colors.ListedColormap(['#fff4d6','#e2eef6']),vmin=0,vmax=1,alpha=.8)
  w=json.loads((case/'scene/immutable_world.json').read_text());p=np.asarray(w['occupied_points'])
  with np.load(case/'scene/display_clouds.npz') as f:floor=f['floor_local']
  ax.scatter(floor[:,0],floor[:,1],s=.3,c='#d5e1e7');ax.scatter(p[:,0],p[:,1],s=.8,c='#758593');ax.scatter(0,0,c='green',s=45);ax.scatter(*row['goal'][:2],c='red',marker='*',s=95)
  if row['selected']:
   route=row['selected'];v=np.array(route['path']);ax.plot(v[:,0],v[:,1],c='#1257b0',lw=2);text=f'{route["profile"]}\n路径{route["length_m"]:.2f}m，横移{route["lateral_distance_m"]:.2f}m'
  else:
   reasons={a.get('reason','') for a in row['attempts']}
   text='起点或目标缺少地面观测' if 'START_OR_GOAL_OUTSIDE_OBSERVED_CANDIDATE_GRID' in reasons else '未找到路径：差速搜索耗尽 / 局部横移搜索超时' if 'SEARCH_BUDGET_EXCEEDED' in reasons else '未找到合规路径'
  ax.set_title(row['name']+'\n'+text,fontsize=10);ax.set_xlim(-2.5,3);ax.set_ylim(-2.5,2.5);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('X 前/后 (m)');ax.set_ylabel('Y 左/右 (m)')
 if len(report['cases'])<6:axes.ravel()[-1].axis('off');axes.ravel()[-1].text(0,.9,'当前新环境：冻结双雷达实测快照\n绿色：起点；红星：临时相对目标\n蓝线：规划器实际输出；灰点：占据\n浅蓝底：候选观测区；浅黄：未知\n\n原1/2号目标及旧地图未使用\n双臂净空仅按先前约定假设满足\n盲区与未观测区仍不代表可通行\n静止采集，无底盘输出',va='top',fontsize=12)
 fig.text(.5,.015,'浅蓝底：观测候选区；浅黄：未知；灰点：占据；蓝线：实际规划输出。冻结场景，仅预览，无底盘输出。',ha='center',fontsize=10)
 fig.suptitle('Hybrid A* 新环境静态测试｜不移动底盘｜当前姿态三维碰撞检查',fontsize=16);fig.tight_layout(rect=[0,.04,1,.94]);fig.savefig(out/'规划器测试总览.png');plt.close(fig)

if __name__=='__main__':
 with exclusive(ROOT/'runtime/navigation.lock'):run()
