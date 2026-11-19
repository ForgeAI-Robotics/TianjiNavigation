"""S100 RAM pose transport; original measurement timestamps are authoritative."""
import json
from pathlib import Path
from common import ROOT,atomic_json
FAST_POSE=Path('/dev/shm/tianji_navigation_v7_pose.json')

def publish(packet):atomic_json(FAST_POSE,packet)

def read():
 # An existing stale/invalid RAM packet must remain stale/invalid. Never fall
 # back to a disk mirror that happens to contain a greener historical status.
 path=FAST_POSE if FAST_POSE.exists() else ROOT/'runtime/current_pose.json'
 return json.loads(path.read_text())
