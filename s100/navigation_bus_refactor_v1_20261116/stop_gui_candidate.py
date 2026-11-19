"""SSH stop fallback restricted to a task owned by the GUI candidate broker."""
import json
import os
from pathlib import Path
import signal


def stop(candidate, production, proc=Path('/proc'), kill=None):
    kill=kill or os.kill
    try:
        record=json.loads((production/'runtime/navigation_execution_status.json').read_text())
        session=record['session'];pid=int(record['pid'])
        if not isinstance(session,str)or len(session)!=32 or any(c not in '0123456789abcdef'for c in session):raise ValueError('INVALID_SESSION')
        job=json.loads((candidate/'runtime/task_api/tasks'/session/'task.json').read_text())
        owner=job.get('process')or{}
        if job.get('id')!=session or owner.get('pid')!=pid:raise ValueError('NOT_CANDIDATE_OWNED')
        args=(proc/str(pid)/'cmdline').read_bytes().split(b'\0')
        if str(candidate/'modules/onboard_navigation.py').encode()not in args or session.encode()not in args:raise ValueError('PROCESS_MISMATCH')
        ticks=(proc/str(pid)/'stat').read_text().rsplit(')',1)[1].split()[19]
        if ticks!=owner.get('pid_start_ticks'):raise ValueError('PID_REUSED')
        kill(pid,signal.SIGINT)
        return dict(stopped=True,pid=pid,session=session,method='OWNED_CANDIDATE_SIGINT')
    except (OSError,ValueError,KeyError,IndexError,TypeError)as exc:
        return dict(stopped=False,reason='NO_MATCHING_GUI_CANDIDATE',detail=str(exc))


if __name__=='__main__':
    project=Path(__file__).resolve().parent
    print(json.dumps(stop(project/'gui_runtime',project.parent/'navigation_s100_v7')))
