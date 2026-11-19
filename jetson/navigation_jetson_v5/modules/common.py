"""Shared paths and local-only persistence. Importing this starts no services."""
import fcntl,json,os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
CONFIG=json.loads((ROOT/'config/config.json').read_text())

def atomic_json(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False,indent=2));os.replace(temp,path)

def exclusive(path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    handle=path.open('a+')
    try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:handle.close();raise RuntimeError('模块已运行：'+str(path))
    handle.seek(0);handle.truncate();handle.write(str(os.getpid()));handle.flush();return handle

def lock_held(path):
    try:
        with Path(path).open('r') as handle:
            try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB);return False
            except BlockingIOError:return True
    except FileNotFoundError:return False
