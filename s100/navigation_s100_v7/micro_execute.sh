#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
exec "$TASK_ROOT/execute.sh" "$@" --max-travel-m 0.05
