"""S100 bridge gate; rechecked for every output, including watchdog repeats."""
import json,os,time
from pathlib import Path
from common import ROOT,CONFIG,lock_held
from gate import CommandGate

def legacy_mapping_active():
    root=Path(CONFIG['legacy_root'])
    if lock_held(root/'runtime/mapping.lock'):return True
    try:return bool(json.loads((root/'runtime/mapping_status.json').read_text()).get('active'))
    except FileNotFoundError:return False
    except (OSError,ValueError):return True

class MappingGuard:
    def constrain(self,x,y,w):
        try:
            state=json.loads((ROOT/'runtime/control_gate.json').read_text())
            if not Path('/proc/'+str(state['pid'])).exists():return 0.,0.,0.
            gate=CommandGate(CONFIG);gate.packet=state['packet'];gate.received=state['received_monotonic']
            allowed=gate.output(time.monotonic(),legacy_mapping_active())
            # A locally requested zero (sensor timeout, shutdown) stays zero.
            # An old direction cannot replace a newer admitted command.
            return tuple((min(abs(v),abs(a))*(1 if v>0 else -1)) if v*a>0 else 0. for v,a in zip((x,y,w),allowed))
        except (OSError,ValueError,TypeError,KeyError):return 0.,0.,0.
