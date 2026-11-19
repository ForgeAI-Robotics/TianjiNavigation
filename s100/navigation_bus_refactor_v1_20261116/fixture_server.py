"""Inject a nonphysical runner into a private COPY of the existing task broker.

The sys.path change is intentionally confined to this legacy compatibility
process. No navbus business module imports any legacy implementation.
"""
import argparse
import json
from pathlib import Path
import secrets
import signal
import sys
import threading
import time


class FixtureRunner:
    simulation = True

    def run(self, job, cancel, emit):
        emit('navigator', dict(phase='PREPARING', reason='ISOLATED_BUS_FIXTURE',
                              session=job['id'], updated_wall_time=time.time()))
        canceled = cancel.wait(.5 if job['kind'] == 'test' else .05)
        if canceled:
            return 'CANCELED', dict(code='FIXTURE_CANCELED', arrived=False,
                                    physical_execution=False, motion_output_count=0)
        state = 'PLAN_READY' if job['kind'] == 'plan' else 'TEST_COMPLETE'
        return state, dict(code='FIXTURE_RESPONSE_ONLY', arrived=False,
                           simulation=True, geometry_checked=False,
                           physical_execution=False, motion_output_count=0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    if not (root/'ISOLATED_CONTRACT_TEST_ROOT').is_file():
        raise ValueError('Separate isolated root is required')
    sys.path.insert(0, str(root/'modules'))
    from navigation_task_runtime import TaskManager
    from navigation_task_api import make_server
    token_path = root/'config/fixture_token'
    if not token_path.exists():
        token_path.write_text(secrets.token_urlsafe(32)+'\n')
        token_path.chmod(0o600)
    manager = TaskManager(root, runner=FixtureRunner(), busy=lambda:False)
    server = make_server(manager, token_path.read_text().strip(), port=18765,
                         readiness=lambda:dict(simulation=True, arm_fresh=False,
                                               motion_ready=False, motion_output_count=0))
    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, stop)
    print(json.dumps(dict(service='isolated-legacy-fixture', port=18765,
                          physical_execution=False, motion_output_count=0)), flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        server.server_close()
        manager.close()


if __name__ == '__main__':
    main()
