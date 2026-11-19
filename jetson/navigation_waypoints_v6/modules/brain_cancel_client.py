"""Explicit cancel of the observed Tianji task. No navigation submission API."""
import json,time,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def request(path,token,body=None):
 if path not in ('/api/status','/api/stop'):raise ValueError('CANCEL_CLIENT_ROUTE_NOT_ALLOWED')
 r=urllib.request.Request('http://192.168.31.144:15001'+path,method='GET'if body is None else'POST',data=None if body is None else json.dumps(body).encode(),headers={'Authorization':'Bearer '+token,'Content-Type':'application/json','X-Project':'tianji'})
 with urllib.request.urlopen(r,timeout=8)as f:return json.load(f)
def cancel(expected=None):
 path=ROOT/'config/brain_cancel_token'
 if path.is_symlink()or path.stat().st_mode&0o077:raise ValueError('PRIVATE_CANCEL_CREDENTIAL_REQUIRED')
 token=path.read_text().strip()
 if len(token)<24:raise ValueError('INVALID_CANCEL_CREDENTIAL')
 state=request('/api/status',token)
 if state.get('project')!='tianji':raise ValueError('WRONG_GATEWAY_PROJECT')
 ident=state.get('task_id')
 if expected and ident!=expected:raise ValueError('任务已切换；未取消新任务，请刷新后重试')
 if not ident:return dict(project='tianji',accepted=False,reason='NO_CURRENT_TASK',status=state)
 result=request('/api/stop',token,dict(project='tianji',task_id=ident))
 if result.get('status',{}).get('task_id')!=ident:raise ValueError('取消回执任务身份不符；停止仍未确认')
 return result
