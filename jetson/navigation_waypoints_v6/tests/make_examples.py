import sys,json,subprocess,warnings
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'modules'));sys.path.insert(0,str(R/'tests'))
from test_navigation import NativeTests
warnings.filterwarnings('ignore',message='Unable to import Axes3D')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
plt.rcParams['font.family']=font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name();plt.rcParams['axes.unicode_minus']=False
out=R/'results/synthetic_examples';out.mkdir(exist_ok=True);test=NativeTests()
wall=[[2.,float(y),.3] for y in np.arange(-.55,.56,.04)];tight=[[x,float(y),.3] for x in [-.48,.48] for y in np.arange(-.1,.101,.02)]
cases=[('短距离：直接平移1米',test.packet([0.,1.,0.])),('长距离：差速方式绕障',test.packet([4.,0.,0.],wall)),('复杂局部：短横移后前行',test.packet([3.,0.,0.],tight,True)),('占据起点：拒绝规划',test.packet([2.,0.,0.],[[0.,0.,.3]]))]
fig,axes=plt.subplots(2,2,figsize=(13,9),dpi=150);records=[]
for i,((name,packet),ax) in enumerate(zip(cases,axes.ravel())):
 folder=out/str(i+1);folder.mkdir(exist_ok=True);(folder/'input.json').write_text(json.dumps(packet));subprocess.run([str(R/'native/hybrid'),str(folder/'input.json'),str(folder/'result.json')],check=True,stdout=subprocess.DEVNULL);result=json.loads((folder/'result.json').read_text());route=result['selected'];obstacles=np.array(packet['world']['occupied_points'])
 if len(obstacles):ax.scatter(obstacles[:,0],obstacles[:,1],c='#263238',s=18,label='障碍回波')
 ax.scatter(0,0,c='green',s=65,label='起点');g=packet['goal'];ax.scatter(g[0],g[1],c='red',marker='*',s=140,label='目标')
 summary='拒绝：'+result['attempts'][0]['reason']
 if route:
  p=np.array(route['path']);ax.plot(p[:,0],p[:,1],c='#1565c0',lw=2,label='实际Hybrid A*路径');summary=f"路径 {route['length_m']:.2f}m；横移累计 {route['lateral_distance_m']:.2f}m"
  # Color only actual side-going edges from the emitted trajectory.
  for a,b in zip(p[:-1],p[1:]):
   d=b[:2]-a[:2];bx=np.cos(a[2])*d[0]+np.sin(a[2])*d[1];by=-np.sin(a[2])*d[0]+np.cos(a[2])*d[1]
   if abs(by)>2*abs(bx) and np.linalg.norm(d)>1e-7:ax.plot([a[0],b[0]],[a[1],b[1]],c='#00a6b2',lw=3)
 ax.set_title(name+'\n'+summary);ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlim(-.8,max(1.,g[0]+.8));ax.set_ylim(-1.5,max(1.5,g[1]+.5));ax.set_xlabel('X / m');ax.set_ylabel('Y / m');records.append(dict(case=name,selected=route,attempts=[{k:v for k,v in a.items() if k not in ['path','primitives']} for a in result['attempts']]))
fig.suptitle('Hybrid A*策略验证 · 合成场景（非当前实景）\n蓝色：差速式主路径；青色：有限短横移；仅离线计算，无底盘运动',fontsize=14);fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(out/'hybrid_policy_examples.png');(out/'results.json').write_text(json.dumps(records,ensure_ascii=False,indent=2));print(out/'hybrid_policy_examples.png')
