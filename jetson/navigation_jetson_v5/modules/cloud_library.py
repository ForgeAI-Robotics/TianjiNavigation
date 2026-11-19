"""Local cloud catalogue. References originals; never moves or edits source data."""
import json, os, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LIBRARY = ROOT / 'cloud_library'

def scan_library(root=ROOT, library=LIBRARY):
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
    archived=[]
    catalog=library/'active_catalog.json'
    if catalog.exists():
        config=json.loads(catalog.read_text());active=[];allowed=config['maps']
        for entry in entries:
            key=Path(entry['path']).name
            if entry['format']=='map' and key in allowed:
                entry=dict(entry,display_name=allowed[key],label=allowed[key]+' · '+str(entry.get('frames',0))+'帧')
                active.append(entry)
            else:archived.append(entry)
        order=list(allowed);entries=sorted(active,key=lambda entry:order.index(Path(entry['path']).name))
        history=library/'history';history.mkdir(exist_ok=True)
        temp=history/'index.json.tmp';temp.write_text(json.dumps(dict(schema_version=1,entries=archived,policy='Historical references retained at original paths; no source deletion or relocation.'),ensure_ascii=False,indent=2)+'\n');os.replace(temp,history/'index.json')
    result = dict(schema_version=2, entries=entries, archived_entries=archived, errors=errors,
                  source_policy='Originals retained; map saves appear automatically; manual edits are visualization only.')
    fd, temp = tempfile.mkstemp(prefix='.index-', dir=library)
    try:
        with os.fdopen(fd, 'w') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
        os.replace(temp, library / 'index.json')
    finally:
        if os.path.exists(temp): os.unlink(temp)
    return result

def preferred_map():
    try:
        config=json.loads((LIBRARY/'active_catalog.json').read_text());folder=ROOT/'maps'/config['default_map']
        if (folder/'map.json').is_file():return folder
    except (OSError,ValueError,KeyError):pass
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
