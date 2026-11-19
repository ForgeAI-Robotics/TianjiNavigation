"""Isolated read-only navigation plan worker. No execution/control interface."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
if __name__=='__main__':
 import argparse
 from pathlib import Path
 from navigation import run
 from common import atomic_json
 p=argparse.ArgumentParser();p.add_argument('--goal',type=int,required=True,choices=[0,1,2,3]);p.add_argument('--reply',type=Path,required=True);a=p.parse_args()
 result=run(a.goal,True,reviewed_map_cleanup=True);atomic_json(a.reply,result)
