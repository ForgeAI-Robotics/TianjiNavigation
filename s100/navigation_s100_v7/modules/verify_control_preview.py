"""Tests the actual local command protocol using --preview ONLY; no ROS publisher."""
import time,json,threading
from pathlib import Path
from common import ROOT
from navigation_control import ControlTransport
from onboard_navigation import MapGuard
from clearance_override import settings
transport=ControlTransport(False);deadline=time.monotonic()+5
while not transport.status and time.monotonic()<deadline:time.sleep(.02)
assert transport.status['mode']=='preview' and not transport.status['bridge_started'] and transport.status['ros_publishers_created']==0
cfg=settings();guard=dict(state='CLEAR',path_checked=True,operator_estop_declared=True,map_sha256=cfg['map_sha256'],model_sha256=cfg['model_sha256'],localization_age_s=.01,pose_age_s=.01,clearance_age_s=.01,guard_age_s=.01,cloud_age_s=.1,operator_age_s=.01,forced_release_active=True,override_version=cfg['updated_wall_time'],assumed_clearance=True)
nonzero=False
for _ in range(20):
 transport.update([.02,0.,0.],guard);time.sleep(.05)
 nonzero|=any(abs(v)>0 for v in transport.status.get('output',[0,0,0]))
assert nonzero,'preview did not exercise nonzero admission'
time.sleep(.55);assert transport.status['output']==[0.,0.,0.],transport.status
transport.close();assert transport.process.poll()==0
folder=Path(json.loads((ROOT/'runtime/latest_plan.json').read_text())['folder']);w=json.loads((folder/'scene/immutable_world.json').read_text());map_guard=MapGuard(folder/'scene/immutable_world.json');clear,_=map_guard.check([0,0,0],[0,0,0]);assert clear
point=next(p for p in w['occupied_points'] if .12<p[2]<.35);collision,_=map_guard.check([point[0],point[1],0],[0,0,0]);assert not collision;map_guard.close()
result=dict(control_protocol_exercised=True,preview_nonzero_admitted=True,expired_decision_zeroed=True,ros_publishers_created=0,motion_bridge_started=False,static_map_origin_clear=True,static_obstacle_collision_rejected=True,motion_commands_sent=0)
(ROOT/'runtime/control_preview_verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
