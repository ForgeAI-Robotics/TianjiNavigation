"""Read-only artifact download; uses the FIRST tab's unchanged renderer."""
import argparse
from pathlib import Path
import targets
from onboard_client import fetch_latest
p=argparse.ArgumentParser();p.add_argument('--folder',required=True);a=p.parse_args()
targets.select_scene('office')
fetch_latest(remote_folder=a.folder)
