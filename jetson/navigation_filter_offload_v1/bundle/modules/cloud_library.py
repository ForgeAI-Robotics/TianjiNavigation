"""Local cloud catalogue. References originals; never moves or edits source data."""
import json, os, tempfile, fcntl
from contextlib import contextmanager
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LIBRARY = ROOT / 'cloud_library'

def _scan_library_unlocked(root=ROOT, library=LIBRARY):
    root, library = Path(root), Path(library)
    library.mkdir(parents=True, exist_ok=True)
    maps = library / 'maps'
    if not maps.exists():
        maps.symlink_to(root / 'maps', target_is_directory=True)
    entries, errors = [], []
    for path in (root / 'maps').glob('*/map.json'):
        try:
            meta = json.loads(path.read_text())
            if not (path.parent / 'voxels.npz').is_file():
                continue
            kind = ('失效记录' if meta.get('invalid_reason') else
                    '裁剪视图' if meta.get('view_only_crop') else
                    '人工整理·仅查看' if meta.get('user_confirmed_trail_cleanup') else
                    '验证副本' if meta.get('validation_only') else '记录/派生地图')
            entries.append(dict(path=str(path.parent.resolve()), name=path.parent.name,
                                kind=kind, frames=meta.get('frames', 0),
                                cells=meta.get('cells', 0), modified=path.stat().st_mtime,
                                format='map', label=f'{path.parent.name} · {kind} · {meta.get("frames", 0)}帧'))
        except (OSError, ValueError, TypeError) as exc:
            errors.append(dict(path=str(path), error=str(exc)))
    # Standalone exports from earlier evaluation pipelines, not thousands of raw scans.
    exports = library / 'exports'
    exports.mkdir(exist_ok=True)
    for pattern in ('*_eval/data/*.pcd', 'dufomap_eval/data/*/official_default.pcd'):
        for path in root.parent.glob(pattern):
            name = '__'.join(path.relative_to(root.parent).parts)
            link = exports / name
            if not link.exists() and not link.is_symlink():
                link.symlink_to(path.resolve())
    for path in exports.glob('*.pcd'):
        if path.is_file():
            entries.append(dict(path=str(path.resolve()), name=path.name, kind='独立PCD·原坐标系',
                                modified=path.stat().st_mtime, format='pcd', label=path.name+' · 独立PCD'))
    entries.sort(key=lambda row: row['modified'], reverse=True)
    result = dict(schema_version=1, entries=entries, errors=errors,
                  source_policy='Originals retained; map saves appear automatically; manual edits are visualization only.')
    fd, temp = tempfile.mkstemp(prefix='.index-', dir=library)
    try:
        with os.fdopen(fd, 'w') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
        os.replace(temp, library / 'index.json')
    finally:
        if os.path.exists(temp): os.unlink(temp)
    return result

@contextmanager
def catalogue_lock(library):
    library=Path(library);library.mkdir(parents=True,exist_ok=True)
    with (library/'.catalogue.lock').open('a') as handle:
        fcntl.flock(handle,fcntl.LOCK_EX)
        try:yield
        finally:fcntl.flock(handle,fcntl.LOCK_UN)

def scan_library(root=ROOT,library=LIBRARY):
    with catalogue_lock(library):return _scan_library_unlocked(root,library)

def register_exports(paths,root=ROOT,library=LIBRARY):
    # Keep the complete previous catalogue; only inspect this plan's exports.
    # The explicit full refresh still rescans maps and removes missing files.
    library=Path(library)
    with catalogue_lock(library):
        try:
            result=json.loads((library/'index.json').read_text())
            if result.get('schema_version')!=1 or not isinstance(result.get('entries'),list):raise ValueError('INVALID_CATALOGUE')
            entries={(row['format'],row['name']):row for row in result['entries']}
        except (OSError,ValueError,KeyError,TypeError):return _scan_library_unlocked(root,library)
        for name in paths:
            path=Path(name)
            if not path.is_file():continue
            entries[('pcd',path.name)]=dict(path=str(path.resolve()),name=path.name,kind='独立PCD·原坐标系',modified=path.stat().st_mtime,format='pcd',label=path.name+' · 独立PCD')
        result['entries']=sorted(entries.values(),key=lambda row:row['modified'],reverse=True)
        fd,temp=tempfile.mkstemp(prefix='.index-',dir=library)
        try:
            with os.fdopen(fd,'w')as handle:json.dump(result,handle,ensure_ascii=False,indent=2)
            os.replace(temp,library/'index.json')
        finally:
            if os.path.exists(temp):os.unlink(temp)
        return result

def preferred_map():
    for name in ('latest_manual_map.json', 'latest_dynamic_map.json', 'latest_map.json', 'latest_validation_map.json'):
        try:
            folder = Path(json.loads((ROOT / 'runtime' / name).read_text())['folder'])
            if (folder / 'map.json').is_file(): return folder
        except (OSError, ValueError, KeyError):
            pass
    return next(iter((ROOT / 'maps').glob('*/map.json')), ROOT / 'maps').parent

if __name__ == '__main__':
    result = scan_library()
    print(f'{LIBRARY}: {len(result["entries"])} clouds, {len(result["errors"])} unreadable metadata')
