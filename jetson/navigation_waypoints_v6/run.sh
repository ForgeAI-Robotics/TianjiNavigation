#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ "${1:-}" == "--local-dev" ]]; then
  shift
  exec /usr/bin/python3 "$PROJECT_ROOT/modules/navigation.py" "$@"
fi
exec /usr/bin/python3 "$PROJECT_ROOT/modules/onboard_client.py" "$@"
