"""Read XYZ from PCD exports without changing coordinates or source files."""
import io
import numpy as np

def read_pcd(path):
    with open(path, 'rb') as handle:
        header = {}
        for _ in range(100):
            line = handle.readline().decode('ascii').strip()
            if not line or line.startswith('#'): continue
            key, *values = line.split(); header[key] = values
            if key == 'DATA': break
        fields = header['FIELDS']; sizes = list(map(int, header['SIZE']))
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        types = header['TYPE']; n = int(header['POINTS'][0]); mode = header['DATA'][0]
        if not all(name in fields for name in ('x', 'y', 'z')): raise ValueError('PCD没有XYZ字段')
        dtype = np.dtype([(name, '<'+{'F':'f','I':'i','U':'u'}[kind]+str(size), (count,))
                          for name, size, kind, count in zip(fields, sizes, types, counts)])
        if mode == 'binary':
            data = np.frombuffer(handle.read(n * dtype.itemsize), dtype=dtype, count=n)
            points = np.column_stack([data[name][:, 0] for name in ('x','y','z')])
        elif mode == 'ascii':
            data = np.loadtxt(io.BytesIO(handle.read()), ndmin=2)
            offsets = np.cumsum([0] + counts[:-1])
            points = data[:, [offsets[fields.index(name)] for name in ('x','y','z')]]
        else:
            raise ValueError('暂不支持此PCD编码：'+mode+'；请选择同会话的map.json或binary/ascii PCD')
    return np.ascontiguousarray(points[np.isfinite(points).all(axis=1)], dtype=np.float32)

class PcdArchive:
    def __init__(self, path):
        self.folder = path.parent; self.points = read_pcd(path)
        self.static = np.ones(len(self.points), dtype=bool)
        self.keyframes = []; self.trajectory = np.empty((0, 3))
        self.meta = dict(frames=0, cells=len(self.points), validation_only=True, source_file=str(path))
