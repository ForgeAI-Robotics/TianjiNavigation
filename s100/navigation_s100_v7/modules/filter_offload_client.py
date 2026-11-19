"""S100 point-filter RPC. Original frame and local monotonic arrival are retained."""
import io,json,socket,time,uuid
from pathlib import Path
import numpy as np
from filter_transport import receive,send,request_digest

class FilterClient:
    def __init__(self,config,model_sha,configuration_stamp):
        self.config=config;self.model_sha=model_sha;self.stamp=configuration_stamp
        self.socket=None;self.stream=None
        try:
            self.socket=socket.create_connection((config['host'],config['port']),timeout=3)
            self.socket.settimeout(20)
            self.stream=self.socket.makefile('rwb',buffering=0)
            token=Path(config['token_file']).read_text().strip()
            send(self.stream,dict(kind='FILTER_HELLO_V1',token=token,model_sha256=model_sha,
                configuration_stamp=configuration_stamp,package_id=config['package_id'],backend=config['backend']))
            reply,payload=receive(self.stream)
            if reply.get('kind')!='FILTER_READY_V1' or reply.get('package_id')!=config['package_id'] or reply.get('model_sha256')!=model_sha or reply.get('configuration_stamp')!=configuration_stamp or reply.get('backend')!=config['backend'] or payload:
                raise ValueError('FILTER_HANDSHAKE_REJECTED')
            self.socket.settimeout(float(config.get('timeout_s',1.2)))
        except BaseException:
            self.close();raise

    def process(self,header,raw,*,conservative_fast,refine_near_hits,refine_radius_m,refine_request=None):
        points=np.asarray(raw,dtype='<f4',order='C')
        if points.ndim!=2 or points.shape[1]!=3 or len(points)>100000 or not np.isfinite(points).all():raise ValueError('FILTER_INVALID_POINTS')
        body=dict(kind='FILTER_FRAME_V1',id=uuid.uuid4().hex,frame=header,point_count=len(points),
            package_id=self.config['package_id'],model_sha256=self.model_sha,configuration_stamp=self.stamp,
            options=dict(conservative_fast=bool(conservative_fast),refine_near_hits=bool(refine_near_hits),refine_radius_m=refine_radius_m))
        # Continuous surface mode is complete. No cross-host monotonic request
        # is used; the legacy sparse mode retains all-direction refinement.
        payload=points.tobytes();digest=request_digest(body,payload)
        began=time.monotonic();cpu=time.process_time()
        sent=send(self.stream,dict(body,request_sha256=digest),payload)
        reply,payload=receive(self.stream)
        elapsed=time.monotonic()-began
        if reply.get('error'):raise ValueError('JETSON_FILTER_ERROR:'+str(reply['error']))
        if (reply.get('id')!=body['id'] or reply.get('request_sha256')!=digest or
            reply.get('package_id')!=self.config['package_id'] or reply.get('model_sha256')!=self.model_sha or
            reply.get('configuration_stamp')!=self.stamp or reply.get('observation_time')!=header['observation_time'] or
            reply.get('joint_observation_time')!=header['joints']['observation_time'] or reply.get('backend')!=self.config['backend']):
            raise ValueError('FILTER_RESPONSE_IDENTITY_MISMATCH')
        with np.load(io.BytesIO(payload),allow_pickle=False) as z:
            kept=np.array(z['points_body'])
        if kept.ndim!=2 or kept.shape[1]!=3 or len(kept)>len(points) or not np.isfinite(kept).all():raise ValueError('FILTER_INVALID_RESPONSE_POINTS')
        report=reply['report'];remote_compute=report['filter_compute_s'];remote_cpu=report['filter_cpu_s']
        report.update(filter_compute_s=elapsed,filter_cpu_s=time.process_time()-cpu,
            compute_host='Jetson',filter_backend=self.config['backend'],remote_filter_compute_s=remote_compute,
            remote_filter_cpu_s=remote_cpu,round_trip_s=elapsed,request_bytes=sent,response_bytes=len(payload),
            source_timestamps_preserved=True,package_id=self.config['package_id'])
        return dict(points_body=kept,report=report,model_sha256=self.model_sha)

    def close(self):
        if self.stream is not None:self.stream.close();self.stream=None
        if self.socket is not None:self.socket.close();self.socket=None
