from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
NAV=ROOT
STACK=ROOT
VIEWER=ROOT/'geometry'
EVAL=ROOT/'localization_backend'
for path in (ROOT/'modules',VIEWER,VIEWER/'vendor'):
 if str(path) not in sys.path:sys.path.append(str(path))
