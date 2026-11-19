#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
exec python3 -m navbus.app --config "${1:-config/fixture.json}"
