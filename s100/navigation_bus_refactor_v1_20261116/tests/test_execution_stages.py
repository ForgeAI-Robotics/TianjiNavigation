"""Stage tests with deterministic sensor/clock/control ports; no hardware imports."""
import ast
from dataclasses import fields, FrozenInstanceError
import collections
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'engine_clone/modules'))
from execution_stages import route, collision, admission
from execution_stages.contracts import Flow
from execution_runtime.contracts import Progress, ResourceBindings


def ports(module, **values):
    return module.Ports(**{f.name:values.get(f.name,Mock(name=f.name)) for f in fields(module.Ports)})


class StageTests(unittest.TestCase):
    def test_contract_has_no_shared_state_and_rejects_undeclared_values(self):
        first=route.State.capture({'reason':'one'});second=route.State.capture({'reason':'two'})
        first.reason='changed';self.assertEqual(second.reason,'two')
        with self.assertRaises(AttributeError):first.unregistered=0
        with self.assertRaises(UnboundLocalError):_ = first.pkt
        with self.assertRaises(TypeError):route.State({'unregistered':0})
        dependency=ports(route)
        with self.assertRaises(FrozenInstanceError):dependency.time=None

    def test_stage_modules_import_no_sibling_implementation(self):
        directory=Path(route.__file__).parent
        for file in directory.glob('*.py'):
            for node in ast.walk(ast.parse(file.read_text())):
                if isinstance(node,ast.ImportFrom) and node.level:
                    self.assertEqual((node.level,node.module),(1,'contracts'))
                if isinstance(node,(ast.Global,ast.Nonlocal)):
                    self.fail('Stage must not write global or closure state')

    def route_state(self, **overrides):
        recovery=Mock(checking_anchor=False,pending=False,reason='ANCHOR_CHANGED')
        recovery.ready.return_value=False
        values=dict(micro=None,route_recovery=recovery,pkt={},reason='OK',pos=True,
            owned=ResourceBindings(transport=Mock(),frame_worker=Mock()),follower=Mock(),
            progress=Progress(ready_count=5),status=Mock(),wait_cycle=Mock(),started=100.,
            active_plan_folder=Path('/fixture'),record={},plan={'target':{'id':3}},
            lease=Mock(),args=types.SimpleNamespace(goal=3),stop=Mock(),folder=Path('/fixture'),
            sensor_recovery=Mock())
        values.update(overrides);return route.State.capture(values)

    def test_route_idle_does_not_touch_control(self):
        frame=self.route_state();self.assertIs(route.execute(frame,ports(route)),Flow.NEXT)
        frame.owned.transport.zero.assert_not_called()

    def test_anchor_confirmation_stops_before_waiting(self):
        events=[];frame=self.route_state()
        frame.route_recovery.checking_anchor=True
        frame.owned.transport.zero.side_effect=lambda:events.append('zero')
        frame.follower.pause.side_effect=lambda reason:events.append('pause')
        frame.status.side_effect=lambda *a,**kw:events.append('status')
        frame.wait_cycle.side_effect=lambda *a,**kw:events.append('wait')
        self.assertIs(route.execute(frame,ports(route)),Flow.CONTINUE)
        self.assertEqual(events,['zero','pause','status','wait'])
        self.assertEqual(frame.progress.ready_count,0)

    def test_sensor_identity_change_is_not_recovered_by_ignoring_it(self):
        frame=self.route_state(reason='LOCALIZATION_FRAME_OR_EPOCH_CHANGED')
        frame.route_recovery.checking_anchor=True
        with self.assertRaisesRegex(ValueError,'FRAME_OR_EPOCH_CHANGED'):route.execute(frame,ports(route))

    def test_pending_route_retains_goal_while_not_ready(self):
        frame=self.route_state();frame.route_recovery.pending=True
        self.assertIs(route.execute(frame,ports(route)),Flow.CONTINUE)
        self.assertEqual(frame.plan['target']['id'],3)
        self.assertEqual(frame.status.call_args.args[0],'WAITING_FOR_ROUTE_CLEAR')

    def test_failed_replan_records_exception_and_keeps_stopped(self):
        frame=self.route_state();frame.route_recovery.pending=True;frame.route_recovery.ready.return_value=True
        reuse=types.ModuleType('checked_route_reuse');reuse.candidate=lambda *a:None;reuse.checked=lambda *a:False
        class Lock:
            def __enter__(self):return self
            def __exit__(self,*a):pass
        dependency=ports(route,time=types.SimpleNamespace(time=lambda:1000.,monotonic=lambda:100.),
            exclusive=lambda p:Lock(),ROOT=Path('/fixture'),isolated_plan=Mock(side_effect=ValueError('NO_ROUTE')))
        with patch.dict(sys.modules,{'checked_route_reuse':reuse}):
            self.assertIs(route.execute(frame,dependency),Flow.CONTINUE)
        self.assertIsNone(frame.owned.frame_worker)
        self.assertEqual(str(frame.route_recovery.failed.call_args.args[0]),'NO_ROUTE')
        self.assertEqual(frame.status.call_args.kwargs['reason'],'NO_ROUTE')
        frame.owned.transport.zero.assert_called_once()

    def collision_fixture(self):
        identity=np.eye(4);pose={'odometry_pose':[0.,0.,0.],'T_map_base':identity}
        scan=({},None,100.)
        report=dict(points=10,filtered=2,protected=8,filter_compute_s=.01,filter_frame='fixture',scan_pose_collision_points=0)
        prepared={s:dict(frame=scan,report=dict(report)) for s in ('front','back')}
        worker=Mock();worker.snapshot.return_value=prepared
        values=dict(geometric=True,started=100.,release={'forced_release_active':True},
            record={'saved_pose':{},'map_voxel_sha256':'map'},m=types.SimpleNamespace(sha256='model'),
            micro=None,route_recovery=Mock(pending=False,checking_anchor=False),
            shapes=[],q={'joint_positions_rad':[0.]},posture_weights={},pkt=pose,twist=[.05,0.,0.],
            copy={s:scan for s in prepared},prepared=prepared,owned=ResourceBindings(frame_worker=worker,map_guard=Mock()),
            follower=types.SimpleNamespace(path=[]),inv_start=identity,
            refresh_output_posture=lambda ref:{'age_s':.01,'observation_time':999.},
            args=types.SimpleNamespace(operator_estop_ready=True),cloud_age=.1,cloud_checked_at=100.,
            operator_age=lambda:0.,probe_deferral=None,guard=None,posture_reference=None)
        values['owned'].map_guard.check.return_value=(True,'OK')
        dependency=ports(collision,time=types.SimpleNamespace(time=lambda:1000.,monotonic=lambda:100.),
            math=__import__('math'),np=np,state=lambda:{},measured_joints=lambda:{'joint_positions_rad':[0.]},
            posture_check=lambda *a:(True,'OK',0.),read_live_pose=lambda:pose,
            position_check=lambda *a:(True,'OK',.1),joint_shift_bound=lambda *a:0.,
            certify_motion=lambda *a,**kw:dict(proven=True,base_residual_bound_m=0.,total_residual_bound_m=.001),
            BASE_ADVANCE_S=.1,FUTURE_HORIZON_S=.5,trajectory_deviation=lambda *a:0.,
            base_transform=lambda p:identity,override_settings=lambda:{})
        return collision.State.capture(values),dependency

    def test_collision_revalidation_keeps_time_reserve_and_evidence(self):
        frame,dependency=self.collision_fixture()
        self.assertIs(collision.execute(frame,dependency),Flow.NEXT)
        self.assertEqual(frame.guard['revalidation_reserve_m'],.005)
        self.assertAlmostEqual(frame.guard['cloud_age_s'],.1)
        self.assertEqual(frame.guard['posture_feedback_observation_time'],999.)
        self.assertEqual(frame.guard['motion_certificate']['total_residual_bound_m'],.001)
        frame.owned.map_guard.check.assert_called_once()

    def test_collision_revalidation_rejects_stale_joints(self):
        frame,dependency=self.collision_fixture()
        dependency=ports(collision,**{f.name:getattr(dependency,f.name) for f in fields(dependency) if f.name!='posture_check'},posture_check=lambda *a:(False,'ARMS_NOT_FRESH',99.))
        with self.assertRaisesRegex(ValueError,'ARMS_NOT_FRESH'):collision.execute(frame,dependency)
        self.assertIsNone(frame.guard)
        frame.owned.map_guard.check.assert_not_called()

    def test_collision_revalidation_checks_map_and_new_anchor(self):
        frame,dependency=self.collision_fixture();frame.owned.map_guard.check.return_value=(False,'MAP_COLLISION')
        frame.route_recovery.map_failure.return_value='MAP_COLLISION'
        with self.assertRaisesRegex(ValueError,'MAP_COLLISION'):collision.execute(frame,dependency)
        frame,dependency=self.collision_fixture();frame.route_recovery.pending=True
        with self.assertRaisesRegex(ValueError,'LOCALIZATION_RECHECK_REQUIRED'):collision.execute(frame,dependency)

    def test_unadmitted_wait_keeps_missing_cloud_missing(self):
        frame=admission.State.capture(dict(guard=None,collision_debug={},reason='WAIT_SENSORS',micro=None,progress=Progress(),
            probe_deferral=None,last_wait_reason=None,wait_reasons=collections.Counter(),follower=Mock(),
            owned=ResourceBindings(),sensor_recovery=Mock(active=False),obstacle_wait=Mock(active=False),status=Mock(),started=100.,warm=100.,wait_cycle=Mock()))
        dependency=ports(admission,time=types.SimpleNamespace(time=lambda:1000.,monotonic=lambda:100.),BLOCKING_REASONS=(),POSE_RECHECK='POSE_RECHECK')
        fake=types.ModuleType('navigation_admission');fake.admission=lambda *a:False
        with patch.dict(sys.modules,{'navigation_admission':fake}):
            self.assertIs(admission.execute(frame,dependency),Flow.NEXT)
        self.assertFalse(frame.has('copy'))
        self.assertEqual(frame.status.call_args.args[0],'WAITING_GUARDS')
        self.assertEqual(frame.wait_reasons['WAIT_SENSORS'],1)


class LoopIntegrationTests(unittest.TestCase):
    collision_fixture = StageTests.collision_fixture
    # Inherit deterministic fixture helpers, but do not duplicate their tests.
    def replay_loop(self, staged, case):
        import copy
        import json
        import math
        import hashlib
        import refactor_engine
        root=Path(__file__).resolve().parents[1]
        source=(root.parent/'navigation_s100_v7/modules/onboard_navigation.py').read_text()
        if staged:
            source=(root/'engine_clone/modules/onboard_navigation.py').read_text()
        else:
            with patch('stage_extraction.extract',side_effect=lambda text,target:(text,[])):
                source=refactor_engine.rewrite(source,root/'engine_clone/modules/execution_stages')
        run=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef)and n.name=='run')
        loop=copy.deepcopy(max((n for n in ast.walk(run)if isinstance(n,ast.While)),key=lambda n:n.end_lineno-n.lineno))
        frame,dependency=self.collision_fixture();values=dict(frame._values)
        stopped=[False];stop=Mock();stop.is_set.side_effect=lambda:stopped[0]
        stop.wait.side_effect=lambda *a:stopped.__setitem__(0,True)
        values['pkt'].update(observation_time=1000.,updated_wall_time=1000.)
        owned=values['owned'];owned.tracker=Mock();owned.tracker.poll.return_value=None
        owned.frame_worker.error=None;owned.transport=Mock(error=None);owned.transport.ready.return_value=True
        events=[];follower=Mock(path=[])
        follower.step.return_value=dict(state='ARRIVED'if case=='arrived'else'FOLLOWING',twist_preview=[.05,0.,0.])
        owned.transport.update.side_effect=lambda twist,*a,**kw:events.append(('update',list(twist)))
        owned.transport.zero.side_effect=lambda:events.append(('zero',))
        values.update(args=types.SimpleNamespace(enable_motion=False,duration=0.,operator_estop_ready=True),
            stop=stop,operator_age=lambda:0.,error=[],last_hash=100.,warm=100.,right_micro=False,cable_precheck=False,
            plan={'target':{'revision':1}},record=dict(values['record'],ground_plane=[0.,0.,0.]),
            progress=Progress(ready_count=3,was_admitted=True),follower=follower,
            sensor_recovery=Mock(active=False),obstacle_wait=Mock(active=False),
            recovery=Mock(),last_wait_reason=None,last_micro_cloud=None,last_collision_capture=0.,
            wait_reasons=collections.Counter(),tracker_started_wall=999.,matrices={},office_slow_translation=False,
            limit_live_twist=lambda twist,age:twist,
            status=lambda phase,**kw:events.append(('status',phase,kw.get('reason'))),
            wait_cycle=lambda *a,**kw:stopped.__setitem__(0,True))
        values['recovery'].waiting.return_value=False
        if case=='anchor':values['route_recovery'].checking_anchor=True
        namespace={f.name:getattr(dependency,f.name)for f in fields(dependency)}
        namespace.update(json=json,math=math,np=np,hashlib=hashlib,targets=types.SimpleNamespace(unchanged=lambda target:True),
            read_live_pose=lambda:values['pkt'],measured_joints=lambda:values['q'],
            measured_shapes=lambda *a:[],readiness=lambda:dict(clearance_clear=True,shapemask_clear=True,forced_release_active=True),
            sweep_guard=lambda *a,**kw:(case!='obstacle','OK'if case!='obstacle'else'LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY',.1),
            route_stage=route,collision_stage=collision,admission_stage=admission,StageFlow=Flow,
            CONTROL_CHECK_PERIOD_S=.5,POSE_RECHECK='POSE_RECHECK',BLOCKING_REASONS=('LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY',),
            classify_pose_wait=Mock(side_effect=AssertionError('UNEXPECTED_POSE_RECHECK')),waitable_data_fault=lambda *a:False,expired_deadlines=lambda guard:[],ControlTransport=Mock(side_effect=AssertionError('HARDWARE_FACTORY_CALLED')),
            MapGuard=Mock(side_effect=AssertionError('HARDWARE_FACTORY_CALLED')),Path=Path,
            ROOT=Path('/fixture'),PreviewFollower=Mock(),exclusive=Mock(),isolated_plan=Mock())
        if case=='stale_joints':namespace['posture_check']=lambda *a:(False,'ARMS_NOT_FRESH',99.)
        function=ast.FunctionDef(name='cycle',args=ast.arguments(posonlyargs=[],args=[ast.arg(arg=k)for k in values],kwonlyargs=[],kw_defaults=[],defaults=[]),body=[loop,ast.Return(ast.Name('progress',ast.Load()))],decorator_list=[])
        exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),'fixture-cycle','exec'),namespace)
        fake=types.ModuleType('navigation_admission');fake.admission=lambda guard,age:guard['state']=='CLEAR'
        with patch.dict(sys.modules,{'navigation_admission':fake}):
            progress=namespace['cycle'](**values)
        return events,progress.phase,progress.ready_count,progress.was_admitted

    def test_actual_stage_calls_match_reference_control_loop(self):
        for case in ('clear','arrived','anchor','stale_joints','obstacle'):
            with self.subTest(case=case):
                before=self.replay_loop(False,case);after=self.replay_loop(True,case)
                self.assertEqual(after,before)
                self.assertTrue(after[0],case)
                if case=='clear':self.assertIn(('update',[.05,0.,0.]),after[0])
                if case=='arrived':self.assertEqual(after[1],'ARRIVED')


if __name__=='__main__':unittest.main()
