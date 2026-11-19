#!/usr/bin/env python3
"""Replay the saved static S100 cloud capture; never start ROS or control APIs."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse
import json
from pathlib import Path
import numpy as np
from environment import EnvironmentPerception
from sweep import check_sweep
from environment_assessment import table_relationship
from local_model import LocalRobotModel,default_urdf_path
from collision_geometry import hull

ROOT=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,default=Path('/home/nvidia/tianji_robot_state_bridge/reports/column_table_edge_20261019'))
    parser.add_argument('--output',type=Path,default=ROOT/'results/environment_replay')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    model=LocalRobotModel(default_urdf_path(),ROOT/'model_manifest.json',ROOT/'collision_geometry.json')
    cloud=np.load(args.capture/'pointclouds.npz')
    metadata=json.loads((args.capture/'capture.json').read_text())
    receiver=json.loads((args.capture/'receiver_status.json').read_text())
    positions=receiver['joint_positions_rad']
    shapes=model.shapes(positions)
    rows=[];last=None
    for i,frame in enumerate(metadata['streams']['merged']):
        points=cloud['merged'][cloud['merged_frame_index']==i]
        # Offline geometry inspection only: original vendor base_link alignment
        # is an unverified identity. This is prominently retained in the report.
        environment=EnvironmentPerception().observe(points,frame['stamp'],frame['stamp'],
                              'tianji/chassis_base',[0.,0.,0.],shapes)
        environment.summary['alignment_assumption']='UNVERIFIED_VENDOR_BASE_LINK_TO_MODEL_IDENTITY'
        environment.summary['joint_positions_rad']=positions
        table=environment.summary['table']
        distance=model.table_constraint(positions,table,frame['stamp'],frame['stamp'],[0.,0.,0.]) if table.get('valid') else {}
        requests={name:check_sweep(shapes,environment,delta,frame['stamp']) for name,delta in
                  [('forward_5cm',[.05,0.,0.]),('forward_15cm',[.15,0.,0.]),
                   ('left_10cm',[0.,.1,0.]),('rotate_10deg',[0.,0.,float(np.deg2rad(10))]) ]}
        rows.append(dict(frame_index=i,environment=environment.summary,table_constraint=distance,
                         table_relationship=table_relationship(shapes,table),candidates=requests))
        last=(points,environment,shapes)
    valid=[v['table_constraint'] for v in rows if v['table_constraint'].get('blocking')]
    gaps=[v['blocking']['surface_gap_m'] for v in valid]
    report=dict(input='existing 2026-10-19 static real cloud capture; no new acquisition',
                frames=len(rows),table_frames=len(valid),
                table_surface_gap_m=dict(minimum=min(gaps),maximum=max(gaps),median=float(np.median(gaps))) if gaps else None,
                measured_reference_m=.14,
                limitations=['Saved vendor-to-model alignment is not physically verified.',
                             'Saved full head/gripper state is missing: conservative full-range bounds retained.',
                             'Point-cloud absence does not certify free space; no motion authorization.',
                             'OBB candidate collision can be conservative; self/contact ambiguity is unresolved.'],
                model_sha256=model.sha256,results=rows)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    points,environment,shapes=last
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for ax,indices,title in [(axes[0],[0,1],'Top view'),(axes[1],[0,2],'Side view')]:
        p=points[np.isfinite(points).all(1)][::8]
        ax.scatter(p[:,indices[0]],p[:,indices[1]],s=1,c='gray',alpha=.35,label='Recorded LiDAR')
        for shape in shapes:
            polygon=np.asarray(hull(np.asarray(shape['world_corners'])[:,indices]))
            if len(polygon)>2:
                polygon=np.vstack([polygon,polygon[0]])
                ax.plot(polygon[:,0],polygon[:,1],color='orange',lw=.5)
        ax.set(title=title,xlabel='X (m)',ylabel='Y (m)' if indices[1]==1 else 'Z (m)',xlim=(-.6,1.2))
        ax.set_ylim((-.9,.9) if indices[1]==1 else (-.15,1.7));ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.suptitle('Recorded geometry inspection: model alignment unverified; not motion permission')
    fig.tight_layout();fig.savefig(args.output/'environment_and_model.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))


if __name__=='__main__':main()
