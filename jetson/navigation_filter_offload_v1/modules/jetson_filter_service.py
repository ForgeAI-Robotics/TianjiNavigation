"""Jetson filter-only service; one process/cache/CUDA context per sensor stream."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import argparse,hashlib,hmac,io,json,signal,socket,subprocess,sys,threading,time
from pathlib import Path
from filter_transport import receive,send,request_digest
ROOT=Path(__file__).resolve().parents[1]

def worker():
    import importlib.abc
    class NoControl(importlib.abc.MetaPathFinder):
        def find_spec(self,fullname,path=None,target=None):
            if fullname.split('.')[0] in {'rclpy','rospy','control_agent','control_bridge','navigation_control','controller_extension','onboard_navigation'}:
                raise ImportError('FILTER_SERVICE_HAS_NO_CONTROL_INTERFACE')
    sys.meta_path.insert(0,NoControl())
    config=json.loads((ROOT/'config/service.json').read_text());manifest=json.loads((ROOT/'bundle_manifest.json').read_text())
    for name,digest in manifest['files'].items():
        if hashlib.sha256((ROOT/'bundle'/name).read_bytes()).hexdigest()!=digest:raise ValueError('FILTER_BUNDLE_INTEGRITY_FAILURE')
    h,payload=receive(sys.stdin.buffer)
    if (h.get('kind')!='FILTER_HELLO_V1' or payload or not hmac.compare_digest(str(h.get('token','')),Path(config['token_file']).read_text().strip()) or
        h.get('package_id')!=manifest['package_id'] or h.get('backend') not in ('cpu','cuda_obb')):
        raise ValueError('FILTER_AUTH_OR_PACKAGE_REJECTED')
    sys.path.insert(0,str(ROOT/'bundle/modules'))
    import numpy as np
    import paths
    from local_model import LocalRobotModel
    from prepared_frames import prepare,configuration_stamp
    from extrinsics import transforms
    from continuous_surface_filter import SurfaceCache
    from chassis_proxy import measured_shapes
    import continuous_surface_filter as surface
    model=LocalRobotModel(ROOT/'bundle/robot.urdf',ROOT/'bundle/geometry/model_manifest.json',ROOT/'bundle/geometry/collision_geometry.json')
    stamp=configuration_stamp();backend=h['backend'];engine=None
    if h.get('model_sha256')!=model.sha256 or h.get('configuration_stamp')!=stamp:raise ValueError('FILTER_MODEL_OR_CONFIG_MISMATCH')
    if backend=='cuda_obb':
        from cuda_distance import CudaDistances
        engine=CudaDistances();original=surface.evaluate
        def accelerated(p,shapes,sha,cfg,cache=None,header=None):
            engine.bind(p,[s for s in shapes if s['mode']=='measured_or_fixed'])
            return original(p,shapes,sha,cfg,cache=cache,header=header)
        surface.evaluate=accelerated;surface.distances=engine.distances
    cache=SurfaceCache();matrices=transforms();sensor=None
    identity=dict(package_id=manifest['package_id'],model_sha256=model.sha256,configuration_stamp=stamp,backend=backend)
    send(sys.stdout.buffer,dict(kind='FILTER_READY_V1',**identity))
    try:
        while True:
            request,payload=receive(sys.stdin.buffer)
            try:
                body={k:v for k,v in request.items() if k!='request_sha256'}
                digest=request_digest(body,payload)
                count=request.get('point_count')
                if request.get('kind')!='FILTER_FRAME_V1' or request.get('request_sha256')!=digest or any(request.get(k)!=identity[k] for k in ('package_id','model_sha256','configuration_stamp')):raise ValueError('FILTER_FRAME_IDENTITY_MISMATCH')
                if type(count)is not int or not 0<=count<=100000 or len(payload)!=count*12:raise ValueError('FILTER_POINT_BUDGET')
                frame=request['frame']
                if frame.get('kind')!='frame' or frame.get('sensor') not in ('front','back'):raise ValueError('FILTER_FRAME_INVALID')
                if sensor is None:sensor=frame['sensor']
                if frame['sensor']!=sensor:raise ValueError('FILTER_SENSOR_CHANGED')
                raw=np.frombuffer(payload,dtype='<f4').reshape(count,3)
                if not np.isfinite(raw).all():raise ValueError('FILTER_POINTS_NONFINITE')
                options=request['options'];data=prepare(model,matrices,(frame,raw,time.monotonic()),surface_cache=cache,**options)
                buf=io.BytesIO();np.savez(buf,points_body=data['points_body'])
                send(sys.stdout.buffer,dict(id=request['id'],request_sha256=digest,report=data['report'],
                    observation_time=frame['observation_time'],joint_observation_time=frame['joints']['observation_time'],**identity),buf.getvalue())
            except Exception as exc:
                send(sys.stdout.buffer,dict(id=request.get('id'),error=type(exc).__name__+': '+str(exc)))
    except EOFError:pass
    finally:
        if engine:engine.close()

def serve():
    config=json.loads((ROOT/'config/service.json').read_text());stop=threading.Event();slots=threading.BoundedSemaphore(4);children=set();connections=set();lock=threading.Lock()
    server=socket.socket();server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);server.bind((config['bind'],config['port']));server.listen(4);server.settimeout(.5)
    def halt(*_):stop.set()
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,halt)
    def relay(conn):
        child=None
        try:
            conn.settimeout(30)
            log_path=ROOT/'runtime/workers.log'
            if log_path.exists() and log_path.stat().st_size>2_000_000:
                os.replace(log_path,log_path.with_suffix('.log.1'))
            log=(ROOT/'runtime/workers.log').open('ab')
            child=subprocess.Popen([sys.executable,str(Path(__file__)),'--worker'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,bufsize=0)
            with lock:children.add(child);connections.add(conn)
            def incoming():
                try:
                    while not stop.is_set():
                        chunk=conn.recv(65536)
                        if not chunk:break
                        data=memoryview(chunk)
                        while data:
                            n=child.stdin.write(data)
                            if not n:raise EOFError()
                            data=data[n:]
                except (OSError,EOFError):pass
                finally:
                    try:child.stdin.close()
                    except OSError:pass
            thread=threading.Thread(target=incoming,daemon=True);thread.start()
            while not stop.is_set():
                chunk=os.read(child.stdout.fileno(),65536)
                if not chunk:break
                conn.sendall(chunk)
        except OSError:pass
        finally:
            try:conn.shutdown(socket.SHUT_RDWR)
            except OSError:pass
            conn.close()
            if child:
                if child.poll()is None:child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.kill();child.wait()
                child.stdout.close();log.close()
                with lock:children.discard(child);connections.discard(conn)
            slots.release()
    try:
        while not stop.is_set():
            try:conn,addr=server.accept()
            except socket.timeout:continue
            if addr[0] not in config['allowed_clients'] or not slots.acquire(blocking=False):conn.close();continue
            threading.Thread(target=relay,args=(conn,),daemon=True).start()
    finally:
        server.close()
        with lock:
            for conn in list(connections):
                try:conn.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                conn.close()
            for child in list(children):
                if child.poll()is None:child.terminate()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--worker',action='store_true');args=parser.parse_args()
    if args.worker:worker()
    else:serve()
