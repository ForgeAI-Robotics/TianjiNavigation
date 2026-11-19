#!/usr/bin/env python3
"""Provide deployment-only members for the captured CUDA filter release."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('private_bundle', type=Path,
                        help='existing Jetson navigation_filter_offload_v1/bundle directory')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    lock = json.loads((root / 'config/examples/cuda-bundle.lock.json').read_text())
    dest = root / 'jetson/navigation_filter_offload_v1'
    missing = []
    for name, digest in lock['files'].items():
        rel = Path(name)
        if rel.is_absolute() or '..' in rel.parts:
            raise SystemExit(f'Invalid member: {name}')
        target = dest / 'bundle' / rel
        if target.is_file() and sha256(target) == digest:
            continue
        if target.exists():
            raise SystemExit(f'Packaged member differs from release: {name}')
        source = args.private_bundle / rel
        if not source.is_file() or sha256(source) != digest:
            raise SystemExit(f'Private member missing or differs from release: {name}')
        missing.append((source, target))
    if not args.check:
        for source, target in missing:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        (dest / 'bundle_manifest.json').write_text(json.dumps(lock, indent=2) + '\n')
    print(f'CUDA release verified: {len(lock["files"])} members; private members: {len(missing)}')


if __name__ == '__main__':
    main()
