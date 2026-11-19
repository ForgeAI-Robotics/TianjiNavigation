"""S100 bridge gate; rechecked for every output, including watchdog repeats."""
import json,os,time,math
from pathlib import Path
from common import ROOT,CONFIG,lock_held
from gate import CommandGate

def legacy_mapping_active():
    root=Path(CONFIG['legacy_root'])
    if lock_held(root/'runtime/mapping.lock'):return True
    try:return bool(json.loads((root/'runtime/mapping_status.json').read_text()).get('active'))
    except FileNotFoundError:return False
    except (OSError,ValueError):return True

def bounded_admitted_twist(requested,allowed):
    """Scale the CURRENT certified direction uniformly under local caps.

    Independent component clipping could manufacture a different curvature
    when a new guard arrived between ROS receipt and watchdog publication.
    A local zero or sign/mode disagreement remains zero until refreshed.
    """
    if not all(math.isfinite(float(v)) for v in (*requested,*allowed)):return (0.,0.,0.)
    scale=1.
    for value,limit in zip(requested,allowed):
        if abs(limit)<1e-12:continue
        if value*limit<=0:return (0.,0.,0.)
        scale=min(scale,abs(value/limit))
    return tuple(float(v*scale) for v in allowed)

class MappingGuard:
    def __init__(self,gate_file=None):
        self.gate_file=Path(gate_file) if gate_file is not None else ROOT/'runtime/control_gate.json'
    def constrain(self,x,y,w):
        try:
            state=json.loads(self.gate_file.read_text())
            if not Path('/proc/'+str(state['pid'])).exists():return 0.,0.,0.
            gate=CommandGate(CONFIG);gate.packet=state['packet'];gate.received=state['received_monotonic']
            allowed=gate.output(time.monotonic(),legacy_mapping_active())
            # A locally requested zero (sensor timeout, shutdown) stays zero.
            # An old direction cannot replace a newer admitted command.
            return bounded_admitted_twist((x,y,w),allowed)
        except (OSError,ValueError,TypeError,KeyError):return 0.,0.,0.
