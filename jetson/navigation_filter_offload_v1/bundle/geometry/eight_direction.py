"""Observed horizontal surface distances to the current arm/tool URDF.

45-degree object azimuth sectors about chassis_base: +X front, +Y left.
These are distances to measured returns, not certified translation distances.
"""
import time
import numpy as np
from scipy.spatial import cKDTree

NAMES=('前','前左','左','后左','后','后右','右','前右')


def arm_link(link):
    return (link in ('Base_L','Base_R','left_adapter_flange','right_adapter_flange','cam_left_wrist_link','cam_right_wrist_link')
            or link in {f'Link{i}_{s}' for i in range(1,8) for s in 'LR'}
            or link.startswith(('left_robotiq_85_','right_robotiq_85_')))


def sector_ids(points):
    return np.floor((np.arctan2(points[:,1],points[:,0])+np.pi/8)%(2*np.pi)/(np.pi/4)).astype(int)


def empty(reason):
    return dict(schema_version=1,scope='dual_arms_and_tools',kind='horizontal_surface_to_observed_return',
                sector_reference='chassis_base +X front +Y left; 45deg object azimuth',range_m=4.,
                directions=[dict(index=i,name=n,status='unknown',reason=reason) for i,n in enumerate(NAMES)])


class EightDirectionRanger:
    def __init__(self,model,meshes):self.model=model;self.meshes=meshes

    def assess(self,result,positions):
        started=time.perf_counter();report=empty('NO_SUPPORTED_RETURN_AT_ARM_HEIGHT')
        shapes=[s for s in self.model.shapes(positions) if arm_link(s['link'])]
        if not shapes or any(s['mode']!='measured_or_fixed' for s in shapes):
            return empty('ARM_OR_TOOL_POSE_INCOMPLETE')
        xyz=np.asarray(result['points_model']);labels=np.asarray(result['labels'])
        zmin=min(np.min(np.asarray(s['world_corners'])[:,2]) for s in shapes)
        zmax=max(np.max(np.asarray(s['world_corners'])[:,2]) for s in shapes)
        valid=np.isfinite(xyz).all(1)&(labels==1)&(np.linalg.norm(xyz[:,:2],axis=1)<=4.)&(xyz[:,2]>=zmin)&(xyz[:,2]<=zmax)
        source=np.flatnonzero(valid);p=xyz[source]
        report.update(height_band_m=[float(zmin),float(zmax)],model_shapes=len(shapes),environment_points_at_height=len(p))
        if not len(p):report['compute_ms']=(time.perf_counter()-started)*1000;return report
        # Do not use the downsampled display cloud. Reject isolated returns only;
        # every reported endpoint remains a real, unaveraged measured point.
        counts=cKDTree(p).query_ball_point(p,.08,return_length=True)
        supported=counts>=3
        report['isolated_points_ignored']=int((~supported).sum())
        source=source[supported];p=p[supported];support=counts[supported]
        if not len(p):report['compute_ms']=(time.perf_counter()-started)*1000;return report
        sectors=sector_ids(p);sector_best=np.full(8,np.inf)
        best=np.full(len(p),np.inf);nearest=np.full_like(p,np.nan);links=np.full(len(p),-1,int)
        for index,shape in enumerate(shapes):
            corners=np.asarray(shape['world_corners']);lo=corners.min(0);hi=corners.max(0)
            lower=np.linalg.norm(np.maximum(np.maximum(lo[:2]-p[:,:2],p[:,:2]-hi[:2]),0),axis=1)
            ids=np.flatnonzero((p[:,2]>=lo[2])&(p[:,2]<=hi[2])&(lower<=sector_best[sectors]))
            ids=ids[np.argsort(lower[ids])]
            matrix=np.asarray(shape['matrix']);rotation=matrix[:3,:3];translation=matrix[:3,3]
            for offset in range(0,len(ids),16):
                batch=ids[offset:offset+16];batch=batch[lower[batch]<=sector_best[sectors[batch]]]
                if not len(batch):continue
                local=np.ascontiguousarray((p[batch]-translation)@rotation)
                distance,q=self.meshes[(shape['link'],shape['name'])].plane_nearest(local,rotation[2,:])
                improve=distance<best[batch];selected=batch[improve]
                best[selected]=distance[improve];nearest[selected]=q[improve]@rotation.T+translation;links[selected]=index
                np.minimum.at(sector_best,sectors[batch],distance)
        for i,direction in enumerate(report['directions']):
            ids=np.flatnonzero((sectors==i)&np.isfinite(best))
            if not len(ids):continue
            j=ids[np.argmin(best[ids])]
            direction.update(status='observed',distance_cm=float(best[j]*100),
                robot_point=nearest[j].tolist(),object_point=p[j].tolist(),link=shapes[links[j]]['link'],
                support_points=int(support[j]),source_point_index=int(source[j]))
            direction.pop('reason',None)
        report['compute_ms']=(time.perf_counter()-started)*1000
        return report


def merge_directions(frames,stale):
    result=[dict(index=i,name=n,status='unknown') for i,n in enumerate(NAMES)]
    valid=[(sensor,frame) for sensor,frame in frames.items() if not stale.get(sensor,True) and frame['header'].get('pose_source','realtime')=='realtime']
    if not valid:return result
    newest=max(valid,key=lambda pair:pair[1]['header']['observation_time'])[1]['header']['positions_used']
    for sensor,frame in valid:
        h=frame['header'];q=h['positions_used']
        arm_names=[k for k in newest if k.startswith('Joint') or 'robotiq' in k]
        if any(n not in q or abs(q[n]-newest[n])>np.deg2rad(.5) for n in arm_names):continue
        for row in h.get('eight_direction',{}).get('directions',[]):
            i=row.get('index')
            if type(i) is not int or not 0<=i<8 or row.get('status')!='observed':continue
            try:
                a=np.array(row['robot_point'],float);b=np.array(row['object_point'],float);distance=float(row['distance_cm'])
                if (a.shape!=(3,) or b.shape!=(3,) or not np.isfinite([a,b]).all() or not np.isfinite(distance)
                        or distance<0 or abs(a[2]-b[2])>.001 or abs(np.linalg.norm(a[:2]-b[:2])*100-distance)>.1):continue
            except (ValueError,TypeError,KeyError):continue
            if result[i]['status']=='unknown' or distance<result[i]['distance_cm']:
                result[i]=dict(row,name=NAMES[i],sensor_id=sensor)
    return result
