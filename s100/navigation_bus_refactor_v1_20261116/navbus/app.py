"""Composition root: the only place allowed to know module implementations."""
import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import secrets
import signal
import threading
import time
from urllib.parse import urlsplit
from .adapters.legacy_http import LegacyGateway
from .adapters.storage import DocumentReader, JsonJournal
from .bus import Bus, Policy
from .http_api import make_server
from .modules.navigation import NavigationModule
from .modules.brain_navigation import BrainNavigationModule
from .modules.world import WorldModule
from .modules.readiness import ReadinessModule
from .modules.localization import LocalizationModule
from .modules.perception import PerceptionModule


@dataclass(frozen=True)
class Config:
    mode: str
    port: int
    upstream_url: str
    upstream_token_file: Path
    token_file: Path
    journal: Path
    pose_primary: Path
    pose_fallback: Path
    perception_status: Path
    allow_motion: bool = False

    @classmethod
    def load(cls, path: Path):
        value = json.loads(path.read_text())
        base = path.parent.parent.resolve()
        for key in ('upstream_token_file', 'token_file', 'journal', 'pose_primary', 'pose_fallback', 'perception_status'):
            candidate = Path(value[key])
            value[key] = candidate if candidate.is_absolute() else base/candidate
        config = cls(**value)
        if type(config.allow_motion) is not bool:
            raise ValueError('Motion permission must be boolean')
        if config.mode not in ('fixture', 'shadow', 'gui'):
            raise ValueError('Unknown bus mode')
        parsed = urlsplit(config.upstream_url)
        if config.mode == 'fixture' and parsed.port != 18765:
            raise ValueError('Fixture mutations may only target isolated port 18765')
        if config.mode == 'gui' and (config.port != 18769 or parsed.port != 18768):
            raise ValueError('GUI runtime must use its dedicated broker and bus ports')
        if config.mode != 'gui' and config.allow_motion:
            raise ValueError('Isolated fixtures and shadow cannot enable motion')
        if config.port not in (18766, 18767, 18769):
            raise ValueError('Do not replace a production service port')
        return config


def compose(config: Config):
    gateway = LegacyGateway(config.upstream_url, config.upstream_token_file.read_text().strip(),
                            access={'fixture':'isolated-fixture','shadow':'read-only','gui':'gui-runtime'}[config.mode])
    modules = [NavigationModule(gateway), BrainNavigationModule(gateway), WorldModule(gateway), ReadinessModule(gateway),
               LocalizationModule(DocumentReader(config.pose_primary, config.pose_fallback)),
               PerceptionModule(DocumentReader(config.perception_status))]
    return Bus(modules, JsonJournal(config.journal),
               Policy(allow_motion=config.mode=='gui' and config.allow_motion, mode='modular-gui' if config.mode=='gui' else 'isolated-'+config.mode), time.time)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    config = Config.load(args.config)
    bus = compose(config)
    config.token_file.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if not config.token_file.exists():
        fd = os.open(config.token_file, os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(secrets.token_urlsafe(32)+'\n')
    config.token_file.chmod(0o600)
    token = config.token_file.read_text().strip()
    if len(token) < 24:
        raise ValueError('Invalid bus token')
    server = make_server(bus, token, port=config.port)
    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop)
    print(json.dumps(dict(bus_mode=config.mode, port=config.port, motion_enabled=config.mode=='gui' and config.allow_motion)), flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
