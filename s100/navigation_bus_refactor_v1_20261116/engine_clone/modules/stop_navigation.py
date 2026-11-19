"""Stop only this isolated navigator, never vendor navigation or other processes."""
import os,signal,json
from pathlib import Path
from common import ROOT
try:
 d=json.loads((ROOT/'runtime/navigation_execution_status.json').read_text());pid=int(d['pid']);args=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
 if str(ROOT/'modules/onboard_navigation.py').encode() not in args:raise ValueError('NO_MATCHING_NAVIGATOR')
 os.kill(pid,signal.SIGINT);print('STOP_REQUESTED')
except (OSError,ValueError,KeyError):print('NAVIGATOR_NOT_RUNNING')
