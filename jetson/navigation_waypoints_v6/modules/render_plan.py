"""Standalone route artifact from actual planner output, never a drawn substitute."""
import json,math,csv
import numpy as np
from paths import ROOT
from saved_map import base_transform

def render(folder):
 import warnings
 warnings.filterwarnings("ignore",message="Unable to import Axes3D")
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib import font_manager
 for path in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc','/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc']:
  if __import__('pathlib').Path(path).exists():plt.rcParams['font.family']=font_manager.FontProperties(fname=path).get_name();break
 plt.rcParams['axes.unicode_minus']=False
 result=json.loads((folder/'planning/plan.json').read_text());scene=json.loads((folder/'scene/frozen_input.json').read_text());T=base_transform(scene['preview_start_map_pose'])
 with np.load(folder/'scene/display_clouds.npz') as f:p=f['occupied_local'];ground=f['floor_local'];manual=f['manual_floor_local'] if 'manual_floor_local' in f else np.empty((0,3))
 p=p@T[:3,:3].T+T[:3,3];ground=ground@T[:3,:3].T+T[:3,3]
 fig,ax=plt.subplots(figsize=(11,9),dpi=150);ax.set_facecolor('#eff4f7');ax.scatter(ground[::3,0],ground[::3,1],s=.4,c='#d9e4ea',label='历史地面观测')
 if len(manual):
  manual=manual@T[:3,:3].T+T[:3,3];ax.scatter(manual[:,0],manual[:,1],s=2,c='#35c5ca',alpha=.45,label='人工补地面（候选支持）')
 ax.scatter(p[:,0],p[:,1],s=.8,c='#6d7c89',alpha=.65,label='地图与当前雷达占据')
 start=scene['preview_start_map_pose'];goal=scene['target']['pose'];ax.scatter(*start[:2],s=90,c='#16a34a',zorder=5,label='当前定位');ax.scatter(*goal[:2],s=140,c='#df3535',marker='*',zorder=6,label=scene['target']['name'])
 ax.arrow(start[0],start[1],.5*math.cos(start[2]),.5*math.sin(start[2]),width=.02,color='#16a34a',zorder=5)
 world=json.loads((folder/'scene/immutable_world.json').read_text())
 from scipy.spatial import ConvexHull
 from itertools import product
 for shape in world['shapes']:
  m=np.array(shape['matrix']);lo,hi=np.array(shape['bounds']);v=np.array(list(product(*zip(lo,hi))));v=(v@m[:3,:3].T+m[:3,3])@T[:3,:3].T+T[:3,3]
  try:h=ConvexHull(v[:,:2]);a=v[h.vertices,:2];a=np.vstack([a,a[0]]);ax.plot(a[:,0],a[:,1],color='#d99a16',lw=.6,alpha=.65)
  except Exception:pass
 selected=result['selected'];sub='未找到合规路径'
 if selected:
  path=np.array(selected['path']);pts=np.c_[path[:,:2],np.zeros(len(path))]@T[:3,:3].T+T[:3,3];yaw=path[:,2]+start[2]
  ax.plot(pts[:,0],pts[:,1],color='#1565c0',lw=2.5,label='Hybrid A*实际输出')
  with (folder/'route_world.csv').open('w') as f:
   w=csv.writer(f);w.writerow(['map_x_m','map_y_m','map_yaw_rad'])
   w.writerows(np.column_stack([pts[:,:2],yaw]).tolist())
  sub=f"{selected['profile']} | 路长 {selected['length_m']:.2f}m | 横移累计 {selected['lateral_distance_m']:.2f}m"
 else:sub+='；'+str([a.get('reason') for a in result['attempts']])
 lower=np.asarray(scene['grid_search_lower_m']);upper=np.asarray(scene['grid_search_upper_m']);box=np.array([[lower[0],lower[1],0],[upper[0],lower[1],0],[upper[0],upper[1],0],[lower[0],upper[1],0]])@T[:3,:3].T+T[:3,3]
 ax.set_xlim(box[:,0].min()-.2,box[:,0].max()+.2);ax.set_ylim(box[:,1].min()-.2,box[:,1].max()+.2);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('地图 X / m');ax.set_ylabel('地图 Y / m');ax.legend(loc='upper right')
 phase='线缆待验证的候选路线；先微移5cm检测' if result.get('cable_precheck') else '双臂净空按用户假设满足'
 if result.get('preview_posture_assumptions'):
  from preview_options import caption
  phase='只读规划：'+caption(result.get('preview_options',dict(ignore_head_camera=True,open_grippers=True)))
 if result.get('frozen_frame_preview'):phase='历史冻结帧预览；不代表当前执行条件'
 if result.get('intent_prefix_only'):phase='仅供5cm检测的意图轨迹；剩余路线尚未验证'
 ax.set_title(f"导航至{scene['target']['name']} · {sub}\n{phase}；只读规划预览，无底盘运动",fontsize=12);fig.tight_layout();fig.savefig(folder/'route.png');plt.close(fig)
 return folder/'route.png'
