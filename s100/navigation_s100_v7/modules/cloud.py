import numpy as np

def decode_cloud(msg):
    """PointCloud2 decoder supporting row padding, field offsets and endianness."""
    if not 0 < msg.width * msg.height <= 250000 or not 12 <= msg.point_step <= 256:
        raise ValueError('POINT_CLOUD_SIZE_INVALID')
    if msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.height*msg.row_step:
        raise ValueError('POINT_CLOUD_BUFFER_INVALID')
    fields = {f.name: f for f in msg.fields}
    if any(n not in fields or fields[n].datatype != 7 or fields[n].count != 1
           or not 0 <= fields[n].offset <= msg.point_step-4 for n in ('x','y','z')):
        raise ValueError('POINT_CLOUD_XYZ_FIELDS_INVALID')
    offsets=sorted(fields[n].offset for n in ('x','y','z'))
    if any(b-a<4 for a,b in zip(offsets,offsets[1:])):
        raise ValueError('POINT_CLOUD_XYZ_FIELDS_OVERLAP')
    dtype = np.dtype(dict(names=['x','y','z'], formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                         offsets=[fields[n].offset for n in ('x','y','z')], itemsize=msg.point_step))
    data = np.ndarray((msg.height,msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step,msg.point_step))
    return np.column_stack([data[n].reshape(-1) for n in ('x','y','z')]).astype(float)

