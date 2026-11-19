import json
import os
from pathlib import Path
import threading
from ..contracts import BusError, plain


class JsonJournal:
    def __init__(self, path: Path):
        self._path = path
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def record(self, item):
        line = json.dumps(plain(item), allow_nan=False)+'\n'
        with self._lock, self._path.open('a') as stream:
            stream.write(line)
            stream.flush()
            if item.get('durable'):
                os.fsync(stream.fileno())


class DocumentReader:
    def __init__(self, primary: Path, fallback: Path | None = None):
        self._primary, self._fallback = primary, fallback

    def read(self) -> dict:
        # An invalid/stale existing primary NEVER falls back to older data.
        path = self._primary if self._primary.exists() or self._fallback is None else self._fallback
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError) as exc:
            raise BusError('OBSERVATION_UNAVAILABLE', 'Status document unavailable', 503) from exc
        return dict(source=str(path), data=data, timestamps_rewritten=False)
