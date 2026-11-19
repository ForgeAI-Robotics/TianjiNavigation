"""Load the actual S100 v7 self-filter for display; never load a controller."""
import importlib,sys
from pathlib import Path

def load():
 directory=Path(__file__).resolve().parent.parent/'navigation_s100_v7/modules'
 sys.path.insert(0,str(directory))
 try:return importlib.import_module('navigation_self_filter').evaluate
 finally:
  if sys.path[0]==str(directory):sys.path.pop(0)
