"""Read-only Tianji gateway monitor. No command transport is implemented."""
import argparse
import json
from pathlib import Path
import time
import urllib.request

CONFIG=Path(__file__).resolve().parents[1]/'config/brain_monitor.json'

def read_config(path=CONFIG):
    config=json.loads(Path(path).read_text())
    if config['url']!='http://192.168.31.144:15001':raise ValueError('ONLY_TIANJI_MONITOR_ALLOWED')
    token_path=Path(config['token_file'])
    if token_path.is_symlink() or token_path.stat().st_mode&0o077:raise ValueError('PRIVATE_MONITOR_TOKEN_REQUIRED')
    token=token_path.read_text().strip()
    if len(token)<24:raise ValueError('INVALID_MONITOR_TOKEN')
    return config['url'],token

def sample(path=CONFIG):
    url,token=read_config(path)
    def get(route):
        request=urllib.request.Request(url+route,headers={'Authorization':'Bearer '+token},method='GET')
        with urllib.request.urlopen(request,timeout=3) as response:return json.load(response)
    return dict(monitor=get('/api/monitor'),logs=get('/api/logs')['rows'],received_at=time.time())

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--once',action='store_true');args=parser.parse_args()
    while True:
        try:print(json.dumps(sample(),ensure_ascii=False),flush=True)
        except Exception as exc:print(json.dumps({'online':False,'error':str(exc)},ensure_ascii=False),flush=True)
        if args.once:return
        time.sleep(3)

if __name__=='__main__':main()
