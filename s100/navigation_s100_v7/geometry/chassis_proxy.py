"""Development-only chassis cuboid derived from verified original URDF bounds."""
import numpy as np
from functools import lru_cache
from collision_geometry import corners

CHASSIS_LINKS=frozenset(('YD_link','ZQ_Link','ZQL_Link','YQ_Link','YQL_Link','ZH_Link','ZHL_Link','YH_Link','YHL_Link','ZLD_Link','YLD_Link','IMU_Link','XJ_Link',*(f'C{i}_Link' for i in range(1,9))))
PROXY_KEY=('YD_link','development_chassis_cuboid')

@lru_cache(maxsize=4)
def profile(model):
    # Zero is used only to derive static geometry, never as a measured pose.
    # Steering parts then receive their full yaw swept envelope, not a sphere.
    q={j['name']:0. for j in model.geometry['joints'] if j['type']!='fixed'}
    shapes=[s for s in model.shapes(q) if s['link'] in CHASSIS_LINKS]
    if len({s['link'] for s in shapes})!=len(CHASSIS_LINKS):raise ValueError('CHASSIS_LINK_SET_MISMATCH')
    steering={j['child']:j for j in model.geometry['joints'] if j['name'] in ('ZQ','YQ','ZH','YH')}
    lo=np.full(3,np.inf);hi=-lo
    for s in shapes:
        c=np.array(s['world_corners']);a=c.min(0);b=c.max(0)
        key=s['link'] if s['link'] in steering else s['link'].replace('L_Link','_Link')
        if key in steering:
            joint=steering[key]
            if joint['parent']!='YD_link' or not np.allclose(np.abs(joint['axis']),[0,0,1]):raise ValueError('UNSUPPORTED_STEERING_GEOMETRY')
            center=np.array(joint['xyz']);radius=np.linalg.norm(c[:,:2]-center[:2],axis=1).max()
            a[:2]=center[:2]-radius;b[:2]=center[:2]+radius
        lo=np.minimum(lo,a);hi=np.maximum(hi,b)
    lo[np.abs(lo)<.001]=0.;hi[np.abs(hi)<.001]=0.
    lo=np.floor(lo*100)/100;hi=np.ceil(hi*100)/100
    return dict(schema_version=1,kind='urdf_chassis_cuboid',frame_id='chassis_base',bounds_m=[lo.tolist(),hi.tolist()],size_cm=((hi-lo)*100).tolist(),center_m=((hi+lo)/2).tolist(),source_urdf_sha256=model.sha256,replaced_links=sorted(CHASSIS_LINKS),steering_envelope='full_yaw_sweep_of_original_bounds',deletion_authorized=False)

def shape(p):
    lo,hi=np.array(p['bounds_m']);matrix=np.eye(4)
    return dict(link=PROXY_KEY[0],name=PROXY_KEY[1],matrix=matrix.tolist(),bounds=p['bounds_m'],world_corners=corners([lo,hi]).tolist(),mode='measured_or_fixed')

def replace_shapes(shapes,p):
    return [s for s in shapes if s['link'] not in CHASSIS_LINKS]+[shape(p)]

def measured_shapes(model,joints):
    # These wheel/steering fragments are discarded immediately and replaced
    # with the same full-yaw chassis envelope. Avoid enumerating their unknown
    # rotations before discarding them; never substitute missing arm values.
    temporary=dict(joints)
    for joint in model.geometry['joints']:
        if joint['child'] in CHASSIS_LINKS and joint['type']!='fixed':temporary.setdefault(joint['name'],0.)
    return replace_shapes(model.shapes(temporary),profile(model))
