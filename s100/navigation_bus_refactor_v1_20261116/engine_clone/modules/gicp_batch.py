"""Bounded file-only parallel GICP; preserve every candidate and its seed index."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import numpy as np

def run(binary,source,target,seeds,output,distance,timeout=50.,workers=2):
 seeds=Path(seeds);output=Path(output)
 lines=[s for s in seeds.read_text().splitlines() if s.strip()]
 if not lines:raise ValueError('GICP_SEEDS_EMPTY')
 count=min(max(1,int(workers)),2,len(lines))
 def batch(worker):
  ids=list(range(worker,len(lines),count));seed=output.with_name(output.stem+f'_worker{worker}_seeds.txt');result=output.with_name(output.stem+f'_worker{worker}.txt')
  seed.write_text(''.join(lines[i]+'\n' for i in ids))
  with result.with_suffix('.log').open('w') as log:
   subprocess.run([str(binary),str(source),str(target),str(seed),str(result),str(distance)],check=True,timeout=timeout,stdout=log)
  rows=np.atleast_2d(np.loadtxt(result))
  if rows.shape!=(len(ids),20) or not np.isfinite(rows).all():raise ValueError('INVALID_GICP_BATCH_RESULT')
  if not np.array_equal(rows[:,0],np.arange(len(ids))):raise ValueError('GICP_SEED_INDEX_MISMATCH')
  rows[:,0]=ids
  return rows
 with ThreadPoolExecutor(max_workers=count) as pool:rows=np.concatenate(list(pool.map(batch,range(count))))
 rows=rows[np.argsort(rows[:,0])];np.savetxt(output,rows,fmt='%.10g')
 return rows
