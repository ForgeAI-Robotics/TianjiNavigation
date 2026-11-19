"""Isolated preview workspace. Importing does not start ROS or robot interfaces."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
NAV=ROOT.parent/'navigation_jetson_v5'
STACK=ROOT.parent/'navigation_stack_v3'
VIEWER=ROOT.parent/'navigation_link_v2'
EVAL=ROOT.parent/'lidar_localization_eval'
for path in (NAV/'modules',STACK,VIEWER):
 if str(path) not in sys.path:sys.path.append(str(path))
