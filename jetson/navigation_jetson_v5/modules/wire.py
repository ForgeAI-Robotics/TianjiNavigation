"""Bounded JSON + float32/zlib records. No pickle, no ROS, no actuator access."""
import json,struct,zlib
import numpy as np
MAX_POINTS=100000
MAX_HEADER=65536
MAX_PAYLOAD=MAX_POINTS*12+65536

def read_exact(stream,size):
    parts=[];remaining=size
    while remaining:
        data=stream.read(remaining)
        if not data:raise EOFError('connection ended')
        parts.append(data);remaining-=len(data)
    return b''.join(parts)

def write_packet(stream,header,points=None):
    header=dict(header,protocol=1);payload=b''
    if points is not None:
        points=np.asarray(points,dtype='<f4')
        if points.ndim!=2 or points.shape[1]!=3 or not 0<len(points)<=MAX_POINTS or not np.isfinite(points).all():raise ValueError('INVALID_POINTS')
        header['point_count']=len(points);payload=zlib.compress(points.tobytes(),1)
    data=json.dumps(header,allow_nan=False,separators=(',',':')).encode()
    if len(data)>MAX_HEADER or len(payload)>MAX_PAYLOAD:raise ValueError('PACKET_TOO_LARGE')
    stream.write(struct.pack('!II',len(data),len(payload))+data+payload);stream.flush()

def read_packet(stream):
    nh,npayload=struct.unpack('!II',read_exact(stream,8))
    if not 0<nh<=MAX_HEADER or npayload>MAX_PAYLOAD:raise ValueError('PACKET_TOO_LARGE')
    header=json.loads(read_exact(stream,nh))
    if not isinstance(header,dict) or header.get('protocol')!=1:raise ValueError('PROTOCOL_MISMATCH')
    payload=read_exact(stream,npayload);points=None
    if npayload:
        count=header.get('point_count')
        if type(count) is not int or not 0<count<=MAX_POINTS:raise ValueError('INVALID_POINT_COUNT')
        decoder=zlib.decompressobj();raw=decoder.decompress(payload,count*12+1)
        if len(raw)!=count*12 or not decoder.eof or decoder.unconsumed_tail or decoder.unused_data:raise ValueError('INVALID_COMPRESSED_POINTS')
        points=np.frombuffer(raw,dtype='<f4').reshape(count,3)
        if not np.isfinite(points).all():raise ValueError('NONFINITE_POINTS')
    return header,points
