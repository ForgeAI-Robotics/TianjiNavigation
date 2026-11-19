"""Optional waypoint graph of checked local motions in one immutable frame.
No map edits, invented floor, long-slide exception, or required waypoint IDs.
"""
from dataclasses import dataclass
import heapq
import math
import time


@dataclass(frozen=True)
class Limits:
    maximum_nodes: int = 8
    maximum_heading_deg: float = 30.
    budget_s: float = 4.
    sample_m: float = .015


def known_path(path, grid):
    if not path:return False
    for row in path:
        if len(row)!=3 or any(not math.isfinite(v)for v in row):return False
        x=math.floor((row[0]-grid['origin'][0])/grid['resolution']);y=math.floor((row[1]-grid['origin'][1])/grid['resolution'])
        if not (0<=x<grid['width']and 0<=y<grid['height']and 0<=grid['data'][y*grid['width']+x]<100):return False
    return True


def dense(knots, step):
    result=[list(knots[0])]
    for a,b in zip(knots,knots[1:]):
        count=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/step))
        for i in range(1,count+1):result.append([a[k]+(b[k]-a[k])*i/count for k in range(3)])
    return result


def propose(packet, waypoints, clear, limits=Limits(), clock=time.monotonic):
    began=clock();cfg=packet['motion_policy'];goal=list(packet['goal']);distance=math.hypot(goal[0],goal[1]);cap=cfg.get('short_translation_goal_distance_m',1.5)
    diagnostic=dict(status='NO_CORRIDOR_PROPOSAL',candidate_edges=0,rejected_edges=0,physical_motion_commands_sent=0)
    if distance<=cap or not cfg.get('short_rectilinear_enabled',True):return None,diagnostic
    yaw=math.atan2(math.sin(goal[2]),math.cos(goal[2]));tolerance=math.radians(cfg.get('fine_goal',{}).get('heading_tolerance_deg',1.))
    if abs(math.degrees(yaw))>limits.maximum_heading_deg:return None,diagnostic
    grid=packet['grid']
    direct=dense([[0.,0.,0.],[goal[0],goal[1],0.]],limits.sample_m)
    if abs(yaw)>tolerance:direct.append(goal)
    if known_path(direct,grid)and clear(direct):
        diagnostic['status']='DIRECT_CORRIDOR_CLEAR_USE_STANDARD_PLANNER';return None,diagnostic
    nodes=[dict(point=[0.,0.],id='start'),dict(point=goal[:2],id='goal')]
    for row in waypoints:
        p=row['point']
        if math.hypot(*p)<.10 or math.hypot(p[0]-goal[0],p[1]-goal[1])<.10:continue
        nodes.append(dict(row))
        if len(nodes)>=limits.maximum_nodes:break
    if len(nodes)<3:return None,diagnostic
    best_candidate=None
    headings=[0.]+([yaw]if abs(yaw)>tolerance else[])
    for heading in headings:
        if clock()-began>limits.budget_s:break
        prefix=[[0.,0.,0.]]
        if heading:
            prefix.append([0.,0.,heading])
            if not clear(prefix):continue
        c,s=math.cos(heading),math.sin(heading);cache={}
        def edge(i,j):
            if(i,j)in cache:return cache[i,j]
            a,b=nodes[i]['point'],nodes[j]['point'];dx,dy=b[0]-a[0],b[1]-a[1]
            bx,by=c*dx+s*dy,-s*dx+c*dy
            if math.hypot(dx,dy)>cap or abs(bx)+abs(by)>cfg.get('short_rectilinear_max_length_m',2.):cache[i,j]=None;return None
            start=[a[0],a[1],heading];end=[b[0],b[1],heading]
            for side_first in (True,False):
                if clock()-began>limits.budget_s:break
                mid=[a[0]-s*by,a[1]+c*by,heading]if side_first else[a[0]+c*bx,a[1]+s*bx,heading]
                path=dense([start,mid,end],limits.sample_m);diagnostic['candidate_edges']+=1
                if known_path(path,grid)and clear(path):
                    cost=abs(bx)+cfg.get('lateral_distance_cost_multiplier',3.)*abs(by)+(cfg.get('lateral_activation_cost_m',.3)if abs(by)>.01 else 0.)
                    value=(cost,path);cache[i,j]=value;return value
                diagnostic['rejected_edges']+=1
            cache[i,j]=None;return None
        queue=[(0.,0)];costs={0:0.};parents={};settled=set()
        while queue and clock()-began<=limits.budget_s:
            cost,i=heapq.heappop(queue)
            if i in settled:continue
            settled.add(i)
            if i==1:break
            for j in range(len(nodes)):
                if j==i or j in settled:continue
                value=edge(i,j)
                if value is None:continue
                weight,path=value;new=cost+weight
                if new<costs.get(j,float('inf')):costs[j]=new;parents[j]=(i,path);heapq.heappush(queue,(new,j))
        if 1 not in settled:continue
        chain=[];current=1
        while current!=0:
            parent,path=parents[current];chain.append((current,path));current=parent
        chain.reverse();path=list(prefix)
        for _,part in chain:path.extend(part[1:])
        if abs(math.atan2(math.sin(yaw-heading),math.cos(yaw-heading)))>tolerance:path.append(goal)
        if not known_path(path,grid)or not clear(path):continue
        via=[nodes[i]['id']for i,_ in chain[:-1]]
        if not via:continue
        length=sum(math.hypot(b[0]-a[0],b[1]-a[1])for a,b in zip(path,path[1:]))
        lateral=sum(abs(-s*(b[0]-a[0])+c*(b[1]-a[1]))for a,b in zip(path,path[1:]))
        score=costs[1]+.08*abs(yaw)
        candidate=dict(found=True,profile='short_rectilinear_translation',candidate_family='waypoint_corridor_graph',
            via_waypoints=via,waypoint_snapshots=[nodes[i]for i,_ in chain[:-1]],path=path,primitives=[],length_m=length,
            lateral_distance_m=lateral,goal_error_m=0.,geometry_recheck_passed=True,independently_rechecked=True,
            selection_cost_m=score,reason='CHECKED_LOCAL_CORRIDORS_VIA_OPTIONAL_WAYPOINTS',whole_route_holonomic=False,
            maximum_local_edge_m=cap,intermediate_heading_required=False,heading_tolerance_deg=math.degrees(tolerance),
            remaining_heading_error_deg=abs(math.degrees(math.atan2(math.sin(yaw-path[-1][2]),math.cos(yaw-path[-1][2])))))
        if best_candidate is None or score<best_candidate['selection_cost_m']:best_candidate=candidate
    diagnostic.update(status='CHECKED_CORRIDOR_GRAPH_READY'if best_candidate else'CORRIDOR_GRAPH_DISCONNECTED',compute_ms=(clock()-began)*1000)
    if best_candidate:best_candidate['compute_ms']=diagnostic['compute_ms']
    return best_candidate,diagnostic
