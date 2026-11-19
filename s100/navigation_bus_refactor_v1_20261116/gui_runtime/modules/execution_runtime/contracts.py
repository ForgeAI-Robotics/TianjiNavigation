from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Protocol


class Closeable(Protocol):
    def close(self) -> Any: ...


class ControlLifecycle(Closeable, Protocol):
    sent: int
    nonzero: int
    def output_stats(self) -> Mapping: ...


class PerceptionLifecycle(Closeable, Protocol):
    latest: Mapping
    def health(self) -> Mapping: ...


class ReaderLifecycle(Protocol):
    def join(self, timeout: float) -> Any: ...


class ChildLifecycle(Protocol):
    def poll(self) -> int | None: ...
    def send_signal(self, signum: int) -> Any: ...
    def wait(self, timeout: float) -> int: ...
    def terminate(self) -> Any: ...
    def kill(self) -> Any: ...


class BusFolder(Protocol):
    folder: Path


@dataclass
class Progress:
    phase: str = 'PREPARING'
    final_details: dict = field(default_factory=dict)
    ready_count: int = 0
    was_admitted: bool = False
    sent_nonzero: int = 0


@dataclass
class ResourceBindings:
    transport: ControlLifecycle | None = None
    frame_worker: PerceptionLifecycle | None = None
    stream: Closeable | None = None
    reader: ReaderLifecycle | None = None
    tracker: ChildLifecycle | None = None
    map_guard: Closeable | None = None
    sensor_bus: BusFolder | None = None
    observer_child: ChildLifecycle | None = None
    observer_log: Closeable | None = None
    resident_motor_trace: list[ChildLifecycle | None] = field(default_factory=lambda: [None])


@dataclass(frozen=True)
class StatusIdentity:
    session: str
    pid: int
    enable_motion: bool
    load_test: bool
    requested_cap: float | None
    micro_limit: float
    operator_estop: bool
    signature: Mapping
    cpu_affinity: tuple


@dataclass(frozen=True)
class StatusInputs:
    transport: ControlLifecycle | None
    frame_worker: PerceptionLifecycle | None
    folder: Path | None
    relay_health: Mapping
    execution_profile: Mapping | None
    route_summary: Mapping | None
    active_plan_folder: Path | None
    controller_extension: Mapping
