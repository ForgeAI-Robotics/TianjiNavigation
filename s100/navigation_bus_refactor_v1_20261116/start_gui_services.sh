#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
test -f gui_runtime/deployment.json
start_if_needed() {
  local service_name="$1"
  shift
  if ! systemctl --user is-active --quiet "$service_name.service"; then
    systemctl --user reset-failed "$service_name.service" >/dev/null 2>&1 || true
    systemd-run --user --unit="$service_name" --collect --property="WorkingDirectory=$PWD" "$@"
  fi
}
start_if_needed tianji-navigation-modular-gui-broker /usr/bin/python3 "$PWD/gui_task_server.py" --enable-motion
/usr/bin/python3 - <<'PY'
import time
import urllib.request
for attempt in range(30):
    try:
        with urllib.request.urlopen('http://127.0.0.1:18768/healthz',timeout=1.)as response:
            assert response.status==200
        break
    except OSError:time.sleep(.1)
else:raise RuntimeError('GUI candidate broker did not become ready')
PY
start_if_needed tianji-navigation-modular-gui-bus /usr/bin/python3 -m navbus.app --config config/gui_s100.json
/usr/bin/python3 - <<'PY'
import time
import urllib.request
for attempt in range(30):
    try:
        with urllib.request.urlopen('http://127.0.0.1:18769/healthz',timeout=1.)as response:
            assert response.status==200
        break
    except OSError:time.sleep(.1)
else:raise RuntimeError('GUI bus did not become ready')
PY
