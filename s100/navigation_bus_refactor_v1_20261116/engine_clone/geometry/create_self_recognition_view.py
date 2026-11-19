#!/usr/bin/env python3
"""Offline standalone self-recognition view from exact ShapeMask pose snapshots."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import argparse,json,struct,base64,zlib,hashlib,xml.etree.ElementTree as ET
import numpy as np
from local_model import LocalRobotModel,default_urdf_path
from lidar_shapemask import box_mesh
ROOT=Path(__file__).resolve().parent


def mesh_edges(model,positions):
    collisions={(link.get('name'),c.get('name',str(i))):c
        for link in ET.parse(model.urdf).getroot().findall('link')
        for i,c in enumerate(link.findall('collision'))}
    lines=[];unknown=[];known=0
    # Match the self-filter's policy: approximate finger angles are not trusted.
    for shape in model.shapes({k:v for k,v in positions.items() if 'robotiq' not in k}):
        if shape['mode']!='measured_or_fixed':
            unknown.append(shape);continue
        c=collisions[(shape['link'],shape['name'])];g=c.find('geometry');mesh=g.find('mesh');box=g.find('box')
        if mesh is not None:
            uri=mesh.get('filename');prefix='package://tianji_robot_description/'
            assert uri.startswith(prefix)
            file=model.urdf.parent.parent/uri[len(prefix):];raw=file.read_bytes();count=struct.unpack_from('<I',raw,80)[0]
            relative=str(file.relative_to(model.urdf.parents[3]))
            if hashlib.sha256(raw).hexdigest()!=model.manifest['mesh_sha256'][relative]:
                raise ValueError('Display mesh differs from the ShapeMask model: '+relative)
            assert len(raw)==84+50*count
            rec=np.frombuffer(raw,dtype=[('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')],offset=84,count=count)
            vertices=np.array(rec['v'],float).reshape(-1,3)*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            vertices,inverse=np.unique(np.round(vertices,7),axis=0,return_inverse=True);faces=inverse.reshape(-1,3)
        elif box is not None:vertices,faces=box_mesh(shape['bounds'])
        else:raise ValueError('Unsupported collision geometry')
        matrix=np.array(shape['matrix']);vertices=vertices@matrix[:3,:3].T+matrix[:3,3]
        tri=vertices[faces];normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
        edges=np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]);edges.sort(axis=1)
        ids=np.tile(np.arange(len(faces)),3);order=np.lexsort((edges[:,1],edges[:,0]));edges=edges[order];ids=ids[order]
        unique,indices,counts=np.unique(edges,axis=0,return_index=True,return_counts=True)
        first=normals[ids[indices]];second=normals[ids[indices+counts-1]]
        feature=(np.sum(first*second,axis=1)<.866)|(counts==1)
        # Store actual geometric edges and normals; silhouettes recompute with view.
        packed=np.column_stack([vertices[unique[:,0]],vertices[unique[:,1]],first,second,feature.astype(int)])
        # Coplanar internal triangulation edges cannot form a visible silhouette.
        # Keep source coordinates; omit redundant display lines only (no mesh edit).
        packed=packed[feature|(np.linalg.norm(first-second,axis=1)>1e-4)]
        lines.append(packed);known+=1
    return np.concatenate(lines),unknown,known


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);args=parser.parse_args();p=args.directory
    model=LocalRobotModel(default_urdf_path(),ROOT/'model_manifest.json',ROOT/'collision_geometry.json')
    meta={s:json.loads((p/'snapshots'/f'{s}.json').read_text()) for s in ('front','back')}
    for d in meta.values():assert d['model_sha256']==model.sha256
    diff=max(abs(meta['front']['positions_used'][n]-meta['back']['positions_used'][n]) for n in meta['front']['positions_used'] if n.startswith('Joint'))
    if diff>np.deg2rad(.5):raise ValueError('Separate poses must not be merged in one static model view')
    edges,unknown,known=mesh_edges(model,meta['front']['positions_used'])
    points=[];counts={};clouds=[]
    for sensor in ('front','back'):
        raw=np.load(p/'snapshots'/f'{sensor}.npz');xyz=raw['points_model'];labels=raw['labels'];counts[sensor]={str(i):int((labels==i).sum()) for i in range(6)}
        valid=(labels!=0)&np.isfinite(xyz).all(1)&(np.linalg.norm(xyz[:,:2],axis=1)<2.5)&(xyz[:,2]>-.15)&(xyz[:,2]<2.2)
        clouds.append((xyz[valid],labels[valid]))
        for label in [1,2,3,4,5]:
            rows=xyz[valid&(labels==label)];limit=6500 if label==1 else 2500
            if len(rows)>limit:rows=rows[np.linspace(0,len(rows)-1,limit,dtype=int)]
            points.extend(np.column_stack([rows,np.full(len(rows),label)]).round(5).tolist())
    stamp=max(d['observation_time'] for d in meta.values())
    from datetime import datetime
    capture=datetime.fromtimestamp(stamp).strftime('%Y-%m-%d %H:%M:%S')
    data=dict(points=points,edges_deflate_b64=base64.b64encode(zlib.compress(np.asarray(edges,dtype='<f4').tobytes(),6)).decode(),
              counts=counts,capture=capture,model_sha256=model.sha256,
              known_shapes=known,unknown_shapes=len(unknown),front_back_skew_ms=abs(meta['front']['observation_time']-meta['back']['observation_time'])*1000,
              pose_difference_rad=diff,clock_uncertainty_ms=max(d['clock_mapping']['uncertainty_s'] for d in meta.values())*1000)
    (p/'visual_data.json').write_text(json.dumps(data,separators=(',',':')))
    html=(ROOT/'self_recognition_template.html').read_text().replace('__DATA__',json.dumps(data,separators=(',',':')))
    (p/'self_recognition.html').write_text(html)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(1,3,figsize=(16,6),facecolor='#101820')
    colors={1:'#647d8b',2:'#20e5cf',3:'#f4a340',4:'#a582ca',5:'#4a9dff'}
    names={1:'环境点',2:'自体候选',3:'容差区',4:'模型不一致',5:'姿态未知'}
    xyz=np.concatenate([x for x,l in clouds]);labels=np.concatenate([l for x,l in clouds])
    for ax,dims,title,limits,view in zip(axes,[(0,1),(0,2),(1,2)],['俯视 XY','侧视 XZ','正视 YZ'],[(-1.1,1.1,-1.05,1.05),(-1.1,1.1,-.05,1.6),(-1.1,1.1,-.05,1.6)],[(0,0,1),(0,1,0),(1,0,0)]):
        ax.set_facecolor('#101820')
        selected=(edges[:,12]>.5)|(np.sum(edges[:,6:9]*view,axis=1)*np.sum(edges[:,9:12]*view,axis=1)<-1e-10)
        seg=edges[selected,:6].reshape(-1,2,3)[:,:,dims]
        ax.add_collection(LineCollection(seg,colors='#f5d56b',linewidths=.55,alpha=.62,zorder=2))
        for label in [1,4,3,5,2]:
            q=xyz[labels==label];ax.scatter(q[:,dims[0]],q[:,dims[1]],s=9 if label==2 else 2,c=colors[label],alpha=.95 if label==2 else .55,label=names[label],zorder=3 if label==2 else 1)
        ax.set_xlim(limits[:2]);ax.set_ylim(limits[2:]);ax.set_aspect('equal');ax.set_title(title,color='white',fontsize=13);ax.set_xlabel('XYZ'[dims[0]]+' (m)',color='white');ax.set_ylabel('XYZ'[dims[1]]+' (m)',color='white');ax.tick_params(colors='#b4c6cf');ax.grid(alpha=.13,color='white')
        for spine in ax.spines.values():spine.set_edgecolor('#405361')
    fig.suptitle('S100 自体识别快照 ｜ 黄色：实际 URDF 碰撞网格轮廓　青色：自体候选　蓝色：未知',color='white',fontsize=16)
    total_self=sum(c['2'] for c in counts.values());total_unknown=sum(c['5'] for c in counts.values())
    fig.text(.5,.065,f'采集 {capture}　自体候选 {total_self} 点　未知 {total_unknown} 点　实际删除 0 点（外参尚未独立验证）',ha='center',color='#e0c891',fontsize=11)
    fig.text(.5,.025,'轮廓由该帧计算实际使用的关节位姿和原始网格生成；此图为快照，不是实时安全状态。',ha='center',color='#b4c6cf',fontsize=10)
    fig.tight_layout(rect=(0,.10,1,.93));fig.savefig(p/'self_recognition.png',dpi=160,facecolor=fig.get_facecolor());plt.close(fig)
    print(json.dumps({k:v for k,v in data.items() if k not in ('points','edges_deflate_b64')},indent=2))


if __name__=='__main__':main()
