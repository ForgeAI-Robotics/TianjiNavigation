"""Bounded display-only packet; never used for navigation or free-space inference."""
import json
import struct
import numpy as np

MAGIC=b'TJVIEW01'
DTYPE=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('label','<u4')])
PREVIEW_POINTS=8000
MAX_POINTS=100000
MAX_BYTES=12+64000+16*MAX_POINTS


def pack(result,metadata,max_points=PREVIEW_POINTS,full=False):
    xyz=np.asarray(result['points_model']);labels=np.asarray(result['labels'])
    valid=np.isfinite(xyz).all(1)&(labels>0)&(labels<6)&(np.linalg.norm(xyz[:,:2],axis=1)<4.)&(xyz[:,2]>-.25)&(xyz[:,2]<3.)
    ranges=np.asarray(result.get('measured_range_m',np.linalg.norm(xyz-np.asarray(metadata.get('sensor_origin_model_m',[0.,0.,0.])),axis=1)))
    raw_valid=np.isfinite(xyz).all(1)&np.isfinite(ranges)&(ranges>=.04)
    if full:
        ids=np.flatnonzero(raw_valid)
        if len(ids)>MAX_POINTS:raise ValueError('FULL_CLOUD_EXCEEDS_PROTOCOL_BOUND')
    selected=[]
    for kinds in ([] if full else [(2,),(5,),(3,4),(1,)]):
        ids=np.flatnonzero(valid&np.isin(labels,kinds));room=max_points-sum(len(x) for x in selected)
        if room<=0:break
        if len(ids)>room:ids=ids[np.linspace(0,len(ids)-1,room,dtype=int)]
        selected.append(ids)
    if not full:ids=np.concatenate(selected) if selected else np.empty(0,int)
    array=np.empty(len(ids),DTYPE)
    for i,name in enumerate('xyz'):array[name]=xyz[ids,i]
    array['label']=labels[ids]
    header=dict(metadata,schema_version=1,point_count=len(ids),display_only=True,
                original_counts={str(i):int((labels==i).sum()) for i in range(6)},
                cloud_mode='full_valid_returns' if full else 'preview',raw_point_count=len(xyz),
                valid_return_count=int(raw_valid.sum()),discarded_invalid_returns=int((~raw_valid).sum()),
                sensor_origin_model_m=metadata.get('sensor_origin_model_m',[0.,0.,0.]),
                measured_range_m=(np.percentile(ranges[raw_valid],[0,50,95,100]).tolist() if raw_valid.any() else []))
    overlay=result.get('cable_overlay')
    if overlay is not None:
        public={k:v for k,v in overlay.items() if not k.endswith('_mask')}
        for mask_key,index_key in [('ignored_mask','ignored_indices'),('protected_mask','protected_indices'),('navigation_filtered_mask','navigation_filtered_indices')]:
            flags=np.asarray(overlay.get(mask_key,np.zeros(len(xyz),bool)),bool)
            if flags.shape!=(len(xyz),):raise ValueError('CABLE_OVERLAY_MASK_SIZE_INVALID')
            indices=np.flatnonzero(flags[ids]);public[index_key]=indices[:2000].tolist()
            public[index_key+'_truncated']=bool(len(indices)>2000)
        header['cable_overlay']=public
    encoded=json.dumps(header,allow_nan=False,separators=(',',':')).encode()
    if len(encoded)>64000:raise ValueError('DISPLAY_HEADER_TOO_LARGE')
    return MAGIC+struct.pack('<I',len(encoded))+encoded+array.tobytes()


def unpack(payload,model_sha):
    if not 12<=len(payload)<=MAX_BYTES or payload[:8]!=MAGIC:raise ValueError('INVALID_DISPLAY_PACKET')
    size=struct.unpack_from('<I',payload,8)[0]
    if not 0<size<=64000 or 12+size>len(payload):raise ValueError('INVALID_DISPLAY_HEADER')
    h=json.loads(payload[12:12+size],parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
    count=h.get('point_count')
    if (h.get('schema_version')!=1 or h.get('display_only') is not True or
        h.get('model_sha256')!=model_sha or h.get('sensor_id') not in ('front','back') or
        type(count) is not int or not 0<=count<=MAX_POINTS or len(payload)-12-size!=count*DTYPE.itemsize):
        raise ValueError('DISPLAY_SCHEMA_OR_MODEL_MISMATCH')
    cloud=np.frombuffer(payload,dtype=DTYPE,offset=12+size,count=count).copy()
    xyz=np.column_stack([cloud[n] for n in 'xyz'])
    if not np.isfinite(xyz).all() or (cloud['label']>5).any():raise ValueError('INVALID_DISPLAY_POINTS')
    pos=h.get('positions_used',{})
    if not isinstance(pos,dict) or len(pos)>30 or not all(type(v) in (int,float) and np.isfinite(v) for v in pos.values()):
        raise ValueError('INVALID_DISPLAY_POSE')
    clock=h.get('clock_mapping',{})
    if (clock.get('timestamp_domain')!='S100_SYSTEM_REALTIME' or
        not isinstance(clock.get('remote_minus_local_s'),(int,float)) or
        not np.isfinite(clock['remote_minus_local_s']) or
        not isinstance(clock.get('uncertainty_s'),(int,float)) or not 0<=clock['uncertainty_s']<=.005):
        raise ValueError('INVALID_DISPLAY_CLOCK')
    if type(h.get('observation_time')) not in (int,float) or not np.isfinite(h['observation_time']):
        raise ValueError('INVALID_DISPLAY_STAMP')
    overlay=h.get('cable_overlay')
    if overlay is not None:
        if not isinstance(overlay,dict) or overlay.get('schema_version')!=1:raise ValueError('INVALID_CABLE_OVERLAY')
        for key in ['ignored_indices','protected_indices','navigation_filtered_indices']:
            indices=overlay.get(key,[])
            if not isinstance(indices,list) or len(indices)>2000 or any(type(i) is not int or not 0<=i<count for i in indices):raise ValueError('INVALID_CABLE_OVERLAY_INDEX')
        centers=np.asarray(overlay.get('centers_body_m',[]),float).reshape(-1,3)
        radius=overlay.get('radius_m',0.)
        if len(centers)>250 or not np.isfinite(centers).all() or type(radius) not in (int,float) or not 0<=radius<=.05:raise ValueError('INVALID_CABLE_OVERLAY_ZONES')
    return h,xyz,cloud['label'].astype(np.uint8)


def frame_age(header,jetson_now):
    return jetson_now-header['clock_mapping']['remote_minus_local_s']-header['observation_time']


def frame_fresh(header,jetson_now,arrival_age):
    return -.1<=frame_age(header,jetson_now)<=2. and 0<=arrival_age<=2.5
