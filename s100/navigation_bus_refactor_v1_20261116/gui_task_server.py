"""Dedicated, manually started broker for GUI testing of the modular engine."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import threading


def main():
    p=argparse.ArgumentParser();p.add_argument('--enable-motion',action='store_true');p.add_argument('--scene',choices=('office','outside'),default='office');a=p.parse_args()
    # Set legacy scene-scoped readers once, before any adapter is imported.
    os.environ['TIANJI_NAV_SCENE']=a.scene
    project=Path(__file__).resolve().parent;root=project/'gui_runtime';prod=project.parent/'navigation_s100_v7'
    deployment=json.loads((root/'deployment.json').read_text())
    if not deployment['sealed_fixture_marker_preserved'] or not (project/'engine_clone/ISOLATED_CONTRACT_TEST_ROOT').exists():
        raise ValueError('Sealed fixture must be preserved')
    sys.path.insert(0,str(prod/'modules'))
    from common import exclusive,lock_held
    from navigation_task_runtime import TaskManager
    from common import CONFIG
    from navigation_readiness import received_readiness
    from navigation_task_api import make_server,token_file
    lock=exclusive(root/'runtime/task_server.lock')
    manager=TaskManager(root,default_scene=a.scene,motion_enabled=a.enable_motion,operator_estop_declared=a.enable_motion,
        busy=lambda:lock_held(prod/'runtime/navigation_execution.lock')or lock_held(prod/'runtime/navigation.lock'))
    def scene_readiness():
        # Use the broker's scene registry rather than the legacy default map.
        reference=manager.registry.get(manager.scene,1)
        raw=json.loads(Path(CONFIG['joint_status']).read_text())
        return received_readiness(raw,reference['map_voxel_sha256'])
    server=make_server(manager,token_file(root/'config/gui_task_api_token'),host='127.0.0.1',port=18768,readiness=scene_readiness)
    def stop(*_):threading.Thread(target=server.shutdown,daemon=True).start()
    for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,stop)
    print(json.dumps(dict(service='MODULAR_GUI_TASK_BROKER',port=18768,motion_enabled=manager.motion_enabled,task_started=False)),flush=True)
    try:server.serve_forever(poll_interval=.2)
    finally:server.server_close();manager.close();lock.close()


if __name__=='__main__':main()
