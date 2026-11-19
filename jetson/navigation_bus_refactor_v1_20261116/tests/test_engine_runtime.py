"""Pure lifecycle/status tests. No robot, ROS, CAN or vendor imports."""
import ast
import copy
import io
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'engine_clone/modules'))
from execution_runtime.contracts import Progress,ResourceBindings,StatusIdentity,StatusInputs
from execution_runtime.operator import OperatorChannel
from execution_runtime.resources import Resources
from execution_runtime.status import StatusPublisher
from execution_runtime.motion import MotionLimits, NearSurfaceSpeed


class ImmediateQueue:
    def __init__(self,write,capacity=64):self.write=write
    def submit(self,item):self.write(item)
    def stats(self):return dict(pending=0,dropped=0,written=0,error=None)
    def close(self):return True


class RuntimeTests(unittest.TestCase):
    def test_components_have_no_module_state_or_implementation_peer_imports(self):
        folder=Path(__file__).resolve().parents[1]/'engine_clone/modules/execution_runtime'
        for path in folder.glob('*.py'):
            tree=ast.parse(path.read_text())
            for node in tree.body:
                self.assertFalse(isinstance(node,(ast.Assign,ast.AnnAssign,ast.AugAssign)),path.name)
            for node in ast.walk(tree):
                if isinstance(node,ast.ImportFrom)and node.level:
                    self.assertEqual((node.level,node.module),(1,'contracts'),path.name)
                if isinstance(node,ast.Import):
                    self.assertFalse(any(x.name.split('.')[0]in('rclpy','rospy','onboard_navigation','control_agent','control_bridge')for x in node.names))

    def test_progress_and_resource_instances_do_not_share_mutable_state(self):
        first,second=Progress(),Progress();first.final_details['error']='one'
        self.assertEqual(second.final_details,{})
        first,second=ResourceBindings(),ResourceBindings();first.resident_motor_trace[0]='one'
        self.assertEqual(second.resident_motor_trace,[None])

    def test_control_closes_before_evidence_and_observers(self):
        events=[]
        control=Mock();control.close.side_effect=lambda:events.append('control')
        stream=Mock();stream.close.side_effect=lambda:events.append('sensor')
        resource=Resources(ResourceBindings(transport=control,stream=stream))
        self.assertTrue(resource.stop_control())
        events.append('stop_evidence')
        resource.stop_observers()
        self.assertEqual(events,['control','stop_evidence','sensor'])

    def test_shutdown_failure_is_reported_without_skipping_other_cleanup(self):
        control=Mock();control.close.side_effect=OSError('shutdown failed')
        stream=Mock();resource=Resources(ResourceBindings(transport=control,stream=stream))
        self.assertFalse(resource.stop_control());resource.stop_observers()
        stream.close.assert_called_once();self.assertEqual(resource.errors[0]['resource'],'control_transport')

    def test_resident_localization_is_not_killed_as_a_task_child(self):
        tracker=Mock(resident=True)
        Resources(ResourceBindings(tracker=tracker)).stop_observers()
        tracker.send_signal.assert_not_called()

    def test_operator_eof_and_bad_json_preserve_failure_reasons(self):
        for text,expected in [('', 'OPERATOR_INPUT_CLOSED'),('bad\n','OPERATOR_INPUT_ERROR:JSONDecodeError')]:
            lease=Mock();stop=threading.Event()
            channel=OperatorChannel(lease,stop,io.StringIO(text),lambda:10.,Mock())
            channel.read();self.assertEqual(lease.fail.call_args.args,(expected,10.))

    def test_status_is_owned_snapshot_and_keeps_measurement_time(self):
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);(root/'runtime').mkdir();rows=[]
            identity=StatusIdentity('session',1,False,False,.1,0.,False,{'engine':'hash'},(0,1))
            publisher=StatusPublisher(root,identity,lambda:False,lambda:10.,lambda:20.,
                lambda path,data:None,ImmediateQueue,rows.append,Mock(),lambda:{},lambda now:{},
                lambda worker,now:{'observation_time':5.},lambda rows:{})
            worker=Mock();worker.health.return_value={}
            inputs=StatusInputs(None,worker,None,{}, {},None,None,{})
            guard={'observation_time':3.}
            record=publisher.emit('FOLLOWING',inputs,guard=guard)
            guard['observation_time']=999.
            self.assertEqual(record['guard']['observation_time'],3.)
            self.assertEqual(record['perception_timing']['observation_time'],5.)
            self.assertEqual(record['motion_commands_sent'],0)


class ReplayTests(unittest.TestCase):
    def source_root(self):
        return Path(__file__).resolve().parents[1]

    def replay(self,path,case,*,expect_early_filter=False):
        import collections
        class NoThread:
            def __init__(self,*args,**kwargs):pass
            def start(self):pass
            def join(self,timeout=None):pass
        class Lease:
            reason=None
            def __init__(self,*args):pass
            def age(self,now):return 0.
            def summary(self,now):return {'fixture':True}
            def fail(self,reason,now):self.reason=reason
        class Handle:
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def close(self):pass
        class Forbidden:
            def __init__(self,*args,**kwargs):raise AssertionError('HARDWARE_FACTORY_CALLED')
        class PlanningBlocked(Exception):pass
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);(root/'runtime').mkdir();(root/'ISOLATED_CONTRACT_TEST_ROOT').touch()
            (root/'config').mkdir();(root/'config/motion_policy.json').write_text('{}')
            folder=root/'results/replay';(folder/'planning').mkdir(parents=True);(folder/'scene').mkdir()
            frame_workers_started = []
            def planning(*args,**kwargs):
                if case=='blocked_exception':raise PlanningBlocked('NO_CHECKED_ROUTE')
                if case=='unexpected_error':raise ValueError('FROZEN_MAP_CHANGED')
                (root/'runtime/latest_plan.json').write_text(json.dumps({'folder':str(folder)}))
                selected={'path':[],'profile':'fixture'}if case=='selected_startup'else None
                (folder/'planning/plan.json').write_text(json.dumps({'selected':selected,'attempts':[],'target':{'id':1},'start_map_pose':[0.,0.,0.]}))
                record={'saved_pose':{'joint_positions_rad':{}},'localization':{},'map_file':str(folder/'voxels.npz'),'model_sha256':'fixture'}if selected else{}
                (folder/'scene/frozen_input.json').write_text(json.dumps(record))
                if selected:
                    kwargs['on_localized'](folder,record['localization'])
                    if expect_early_filter:
                        self.assertEqual(frame_workers_started, [True])
            cleanup=types.ModuleType('near_body_cleanup');cleanup.settings=lambda:{'micro_probe_enabled':False}
            navigation=types.ModuleType('navigation');navigation.run=planning
            output=[]
            fake_signal=types.SimpleNamespace(SIGINT=signal.SIGINT,SIGTERM=signal.SIGTERM,SIGHUP=signal.SIGHUP,
                signal=lambda *args:None,Signals=signal.Signals)
            namespace=dict(ROOT=root,os=os,json=json,sys=types.SimpleNamespace(stdin=io.StringIO()),
                time=time,signal=fake_signal,threading=types.SimpleNamespace(Thread=NoThread,Event=threading.Event,Lock=threading.Lock),
                collections=collections,Path=Path,GuardRecovery=Mock,ObstacleWait=Mock,SensorRecovery=Mock,
                signature=lambda root:{'fixture':'hash'},exclusive=lambda path:Handle(),
                OperatorSession=Lease,AsyncDiagnostics=ImmediateQueue,
                atomic_json=lambda path,value:None,feedback_health=lambda:{'fixture':True},
                summarize=lambda rows:{'rows':len(rows)},SensorStream=Forbidden,SensorBus=Forbidden,
                ControlTransport=Forbidden,PlanningBlocked=PlanningBlocked,
                Progress=Progress,Resources=Resources,OperatorChannel=OperatorChannel,
                StatusIdentity=StatusIdentity,StatusInputs=StatusInputs,StatusPublisher=StatusPublisher,
                MotionLimits=MotionLimits,NearSurfaceSpeed=NearSurfaceSpeed,
                shutil=__import__('shutil'))
            injected={'near_body_cleanup':cleanup,'navigation':navigation}
            if case=='selected_startup':
                import numpy as np
                class PassiveStream:
                    def __init__(self,*a,**kw):pass
                    def close(self):pass
                class PassiveBus:
                    def __init__(self,*a):self.folder=root/'fake_sensor_bus'
                class PreviewControl:
                    sent=0;nonzero=0
                    def __init__(self,real,**kw):
                        if real:raise AssertionError('REAL_CONTROL_FORBIDDEN')
                    def close(self):pass
                    def output_stats(self):return{}
                class PassiveFrames:
                    latest={}
                    def __init__(self,*a,**kw):frame_workers_started.append(True)
                    def health(self):return{}
                    def close(self):pass
                resident=types.ModuleType('resident_localization');resident.capture_stream=lambda:None
                resident.ensure=lambda *a:types.SimpleNamespace(resident=True)
                prepared=types.ModuleType('prepared_frames');prepared.PreparedFrames=PassiveFrames;prepared.latency_summary=lambda *a:{}
                runtime_profile=types.ModuleType('navigation_runtime_profile')
                def profile(*args):
                    if args[2]is not None:raise AssertionError('NORMAL_NAVIGATION_MICRO_MUST_BE_NONE')
                    return dict(live_filter=False,directional_translation=False,translation_cap_mps=.1,refine_radius_m=.2)
                runtime_profile.filter_profile=profile
                route=types.ModuleType('route_recovery');route.RouteRecovery=lambda *a:types.SimpleNamespace(summary=lambda:{})
                injected.update(resident_localization=resident,prepared_frames=prepared,navigation_runtime_profile=runtime_profile,route_recovery=route)
                namespace.update(np=np,model=lambda:types.SimpleNamespace(geometry=None,sha256='fixture'),
                    motion_weights=lambda geometry:{},PreviewFollower=lambda *a:types.SimpleNamespace(path=[]),
                    transforms=lambda:{},measured_shapes=lambda *a:[{'world_corners':[[.1,.1,.1]]}],
                    MapGuard=lambda *a:types.SimpleNamespace(close=lambda:None),base_transform=lambda *a:np.eye(4),
                    SensorStream=PassiveStream,SensorBus=PassiveBus,ControlTransport=PreviewControl,
                    targets=types.SimpleNamespace(unchanged=lambda *a:True,scene_id=lambda:'office'),
                    TRANSLATION_CAP_MPS=.1,BASE_ADVANCE_S=.1,RAW_SCAN_HZ=10.,CONTROL_CHECK_PERIOD_S=.5)
            function=copy.deepcopy(next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='run'))
            exec(compile(ast.Module(body=[function],type_ignores=[]),str(path),'exec'),namespace)
            args=types.SimpleNamespace(enable_motion=False,load_test_no_motion=False,right_micro_preview=False,
                max_travel_m=0.,cable_precheck=False,translation_cap_mps=None,session='replay',operator_estop_ready=False,
                goal=1,duration=1e-9)
            def collect(*args,**kwargs):
                if args and isinstance(args[0],str)and args[0].startswith('STATUS '):
                    output.append(json.loads(args[0][7:]))
            namespace['print']=collect
            with patch.dict(sys.modules,injected):
                code=namespace['run'](args)
            def normalized(value):
                if isinstance(value,dict):return {k:normalized(v)for k,v in value.items()if k not in('updated_wall_time','wall_s')}
                if isinstance(value,list):return [normalized(v)for v in value]
                if isinstance(value,str):return value.replace(str(root),'<ROOT>')
                return value
            return code,[normalized(row)for row in output]

    def test_original_and_extracted_engine_preserve_terminal_statuses(self):
        original=self.source_root().parent/'navigation_s100_v7/modules/onboard_navigation.py'
        # On S100 the current baseline is next to the isolated project too.
        extracted=self.source_root()/'engine_clone/modules/onboard_navigation.py'
        for case,phase in [('route_blocked','TEST_COMPLETE'),('blocked_exception','PLAN_BLOCKED'),('unexpected_error','STOP_ERROR')]:
            before=self.replay(original,case);after=self.replay(extracted,case)
            self.assertEqual(after,before,case)
            self.assertEqual(after[1][-1]['phase'],phase)
            self.assertTrue(after[1][-1]['finished'])
            self.assertEqual(after[1][-1]['motion_commands_sent'],0)

    def test_refactor_preserves_optional_session_initializers(self):
        root=self.source_root()
        def defaults(path):
            run=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='run')
            found=set()
            for statement in run.body:
                if isinstance(statement,ast.Try):break
                if isinstance(statement,ast.Assign)and isinstance(statement.value,ast.Constant)and statement.value.value is None:
                    found.update(target.id for target in statement.targets if isinstance(target,ast.Name))
            return found
        original=defaults(root.parent/'navigation_s100_v7/modules/onboard_navigation.py')
        current=defaults(root/'engine_clone/modules/onboard_navigation.py')
        bindings=ResourceBindings()
        migrated={name for name,value in vars(bindings).items()if value is None}
        # StatusPublisher owns its last_phase; resource defaults are instance-owned.
        self.assertEqual(original-current-migrated-{'last_phase'},set())

    def test_selected_route_initializes_normal_mode_and_reaches_ready_preview(self):
        root=self.source_root()
        before=self.replay(root.parent/'navigation_s100_v7/modules/onboard_navigation.py','selected_startup')
        after=self.replay(root/'engine_clone/modules/onboard_navigation.py','selected_startup',expect_early_filter=True)
        self.assertEqual(before[1][-1]['phase'],'TEST_COMPLETE',before)
        self.assertEqual(after,before)
        self.assertEqual(after[1][-1]['motion_commands_sent'],0)
        self.assertEqual(after[1][-1]['execution_profile']['translation_cap_mps'],.1)

    def test_collision_and_motion_decision_loop_ast_is_unchanged(self):
        root=self.source_root()
        before=ast.parse((root.parent/'navigation_s100_v7/modules/onboard_navigation.py').read_text())
        after=ast.parse((root/'engine_clone/modules/onboard_navigation.py').read_text())
        from stage_extraction import expand_stages
        after=expand_stages(after,root/'engine_clone/modules/execution_stages')
        class Normalize(ast.NodeTransformer):
            def visit_Attribute(self,node):
                node=self.generic_visit(node)
                if isinstance(node.value,ast.Name)and node.value.id in('owned','progress'):
                    return ast.copy_location(ast.Name(id=node.attr,ctx=node.ctx),node)
                return node
        def loop(tree):
            run=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='run')
            return max((n for n in ast.walk(run)if isinstance(n,ast.While)),key=lambda n:n.end_lineno-n.lineno)
        self.assertEqual(ast.dump(loop(before)),ast.dump(Normalize().visit(loop(after))))

    def test_isolated_motion_refused_before_factories(self):
        path=self.source_root()/'engine_clone/modules/onboard_navigation.py'
        node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='run')
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);(root/'ISOLATED_CONTRACT_TEST_ROOT').touch();namespace={'ROOT':root}
            exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
            with self.assertRaisesRegex(ValueError,'CANNOT_ENABLE_PHYSICAL_MOTION'):
                namespace['run'](types.SimpleNamespace(enable_motion=True))

    def test_instance_motion_policy_matches_original_limiter(self):
        import math,random
        root=self.source_root();source=root.parent/'navigation_s100_v7'
        policy=json.loads((source/'config/motion_policy.json').read_text());limits=MotionLimits.from_policy(policy)
        mapping=dict(TRANSLATION_CAP_MPS=limits.translation_cap_mps,
            LATERAL_CAP_MPS=limits.lateral_cap_mps,UNOBSERVED_TRAVEL_BUDGET_M=limits.unobserved_travel_budget_m,
            ROTATING_TRANSLATION_CAP_MPS=limits.rotating_translation_cap_mps,
            ROTATING_LATERAL_CAP_MPS=limits.rotating_lateral_cap_mps,
            ROTATING_TRAVEL_BUDGET_M=limits.rotating_travel_budget_m,math=math)
        tree=ast.parse((source/'modules/navigation_control.py').read_text())
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='limited_twist')
        exec(compile(ast.Module(body=[function],type_ignores=[]),'original-limiter','exec'),mapping)
        rng=random.Random(42)
        for _ in range(200):
            twist=[rng.uniform(-.3,.3),rng.uniform(-.3,.3),rng.choice([0.,rng.uniform(-.4,.4)])]
            for age in (0.,.3,1.1,2.):
                for radius in (0.,.5,2.):
                    self.assertEqual(limits.limit(twist,age,radius),mapping['limited_twist'](twist,age,radius))

    def test_isolation_guard_does_not_trust_a_cached_common_root(self):
        path=self.source_root()/'engine_clone/modules/onboard_navigation.py'
        node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='run')
        with tempfile.TemporaryDirectory()as unrelated:
            namespace={'ROOT':Path(unrelated),'Path':Path,'__file__':str(path)}
            exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
            with self.assertRaisesRegex(ValueError,'CANNOT_ENABLE_PHYSICAL_MOTION'):
                namespace['run'](types.SimpleNamespace(enable_motion=True))

    def test_direct_control_constructor_is_sealed_before_process_creation(self):
        path=self.source_root()/'engine_clone/modules/navigation_control.py'
        tree=ast.parse(path.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='ControlTransport')
        node=copy.deepcopy(next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='__init__'))
        node.name='construct'
        namespace={'__file__':str(path)}
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
        with self.assertRaisesRegex(ValueError,'CANNOT_ENABLE_PHYSICAL_MOTION'):
            namespace['construct'](types.SimpleNamespace(),True)


if __name__=='__main__':unittest.main()
