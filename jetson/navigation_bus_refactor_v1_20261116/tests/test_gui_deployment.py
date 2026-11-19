"""Opt-in GUI deployment checks; never imports ROS or launches an executor."""
import ast
import json
from pathlib import Path
import tempfile
import unittest
from navbus.app import Config
from navbus.adapters.legacy_http import LegacyGateway


class GuiDeploymentTests(unittest.TestCase):
    def config(self, folder, **updates):
        value=dict(mode='gui',port=18769,upstream_url='http://127.0.0.1:18768',upstream_token_file='runtime/token',token_file='runtime/bus_token',journal='runtime/journal',pose_primary='runtime/pose',pose_fallback='runtime/fallback',perception_status='runtime/status',allow_motion=True)
        value.update(updates);p=Path(folder)/'config.json';p.write_text(json.dumps(value));return p

    def test_gui_ports_are_explicit_and_isolated_modes_cannot_enable_motion(self):
        with tempfile.TemporaryDirectory()as folder:
            config=Config.load(self.config(folder));self.assertTrue(config.allow_motion)
            for change in [dict(port=8765),dict(upstream_url='http://127.0.0.1:8765'),dict(mode='fixture',port=18766,upstream_url='http://127.0.0.1:18765'),dict(mode='shadow',port=18767,upstream_url='http://127.0.0.1:8765'),dict(allow_motion='true')]:
                with self.subTest(change=change),self.assertRaises(ValueError):Config.load(self.config(folder,**change))

    def test_gui_adapter_does_not_write_original_broker(self):
        with self.assertRaises(ValueError):LegacyGateway('http://127.0.0.1:8765','not-a-real-token',access='gui-runtime')

    def test_deployment_keeps_fixture_sealed_and_original_source_intact(self):
        from deploy_gui_runtime import prepare
        with tempfile.TemporaryDirectory()as folder:
            project=Path(folder)/'candidate';source=project/'engine_clone/modules';source.mkdir(parents=True)
            (project/'engine_clone/ISOLATED_CONTRACT_TEST_ROOT').touch()
            for name in ('execution_runtime','execution_stages'):(source/name).mkdir()
            original='def run(args):\n return 0\n'
            (source/'onboard_navigation.py').write_text(original)
            (source/'execution_metrics.py').write_text('def signature(root):\n result={}\n return result\n')
            prod=Path(folder)/'navigation_s100_v7';(prod/'config').mkdir(parents=True);(prod/'results').mkdir()
            (prod/'config/targets.json').write_text('{}')
            (prod/'config/brain_compat.json').write_text('{}')
            deployed=prepare(project)
            self.assertTrue((project/'engine_clone/ISOLATED_CONTRACT_TEST_ROOT').exists())
            self.assertEqual((source/'onboard_navigation.py').read_text(),original)
            self.assertEqual((deployed/'config/targets.json').resolve(),prod/'config/targets.json')
            self.assertEqual((deployed/'config/brain_compat.json').resolve(),prod/'config/brain_compat.json')
            self.assertFalse((deployed/'config/brain_navigation_compat.json').exists())
            entry=(deployed/'modules/onboard_navigation.py').read_text()
            ast.parse(entry);self.assertTrue(entry.endswith(original))
            self.assertIn(str(prod/'modules'),entry)
            self.assertTrue((deployed/'results').is_symlink())


    def test_broker_scene_is_selected_before_legacy_readers_are_created(self):
        import os,sys,types
        from unittest.mock import patch
        import gui_task_server
        with tempfile.TemporaryDirectory() as folder:
            project=Path(folder)/'candidate';(project/'gui_runtime').mkdir(parents=True)
            (project/'engine_clone').mkdir();(project/'engine_clone/ISOLATED_CONTRACT_TEST_ROOT').touch()
            (project/'gui_runtime/deployment.json').write_text(json.dumps(dict(sealed_fixture_marker_preserved=True)))
            raw=Path(folder)/'received.json';raw.write_text('{}');seen={}
            class Manager:
                def __init__(self,root,default_scene,**kw):
                    self.scene=default_scene;self.motion_enabled=False
                    seen['reader_scene_at_construction']=os.environ['TIANJI_NAV_SCENE']
                    self.registry=types.SimpleNamespace(get=lambda scene,n:dict(map_voxel_sha256=scene+'-map'))
                def close(self):pass
            def server(manager,token,**kwargs):
                seen['readiness']=kwargs['readiness']()
                return types.SimpleNamespace(serve_forever=lambda **kw:None,server_close=lambda:None)
            common=types.ModuleType('common');common.CONFIG={'joint_status':str(raw)};common.exclusive=lambda p:types.SimpleNamespace(close=lambda:None);common.lock_held=lambda p:False
            runtime=types.ModuleType('navigation_task_runtime');runtime.TaskManager=Manager
            api=types.ModuleType('navigation_task_api');api.make_server=server;api.token_file=lambda p:'test-token'
            readiness=types.ModuleType('navigation_readiness');readiness.received_readiness=lambda raw,sha:dict(map_sha256=sha,override_scene=os.environ['TIANJI_NAV_SCENE'])
            with patch.dict(sys.modules,dict(common=common,navigation_task_runtime=runtime,navigation_task_api=api,navigation_readiness=readiness)),patch.dict(os.environ,{'TIANJI_NAV_SCENE':'outside'}),patch.object(gui_task_server,'__file__',str(project/'gui_task_server.py')),patch.object(sys,'argv',['gui_task_server.py']),patch.object(sys,'path',list(sys.path)),patch.object(gui_task_server.signal,'signal'),patch('builtins.print'):
                gui_task_server.main()
            self.assertEqual(seen['reader_scene_at_construction'],'office')
            self.assertEqual(seen['readiness'],dict(map_sha256='office-map',override_scene='office'))


class StopFallbackTests(unittest.TestCase):
    def fixture(self, folder):
        root=Path(folder);candidate=root/'candidate';prod=root/'production';proc=root/'proc';ident='a'*32;pid=42
        (candidate/'runtime/task_api/tasks'/ident).mkdir(parents=True);(prod/'runtime').mkdir(parents=True);(proc/str(pid)).mkdir(parents=True)
        (prod/'runtime/navigation_execution_status.json').write_text(json.dumps(dict(session=ident,pid=pid)))
        (candidate/'runtime/task_api/tasks'/ident/'task.json').write_text(json.dumps(dict(id=ident,process=dict(pid=pid,pid_start_ticks='123'))))
        (proc/str(pid)/'cmdline').write_bytes(b'/usr/bin/python3\0'+str(candidate/'modules/onboard_navigation.py').encode()+b'\0--session\0'+ident.encode()+b'\0')
        (proc/str(pid)/'stat').write_text('42 (python3) '+' '.join(['0']*19+['123']))
        return candidate,prod,proc

    def test_matching_owned_session_receives_only_sigint(self):
        from stop_gui_candidate import stop
        import signal
        with tempfile.TemporaryDirectory()as folder:
            candidate,prod,proc=self.fixture(folder);signals=[]
            self.assertTrue(stop(candidate,prod,proc,lambda *a:signals.append(a))['stopped'])
            self.assertEqual(signals,[(42,signal.SIGINT)])

    def test_other_process_and_reused_pid_are_not_signaled(self):
        from stop_gui_candidate import stop
        for invalid in ('other_process','reused_pid'):
            with tempfile.TemporaryDirectory()as folder:
                candidate,prod,proc=self.fixture(folder);signals=[]
                if invalid=='other_process':(proc/'42/cmdline').write_bytes(b'/vendor/navigation\0')
                else:(proc/'42/stat').write_text('42 (python3) '+' '.join(['0']*19+['999']))
                self.assertFalse(stop(candidate,prod,proc,lambda *a:signals.append(a))['stopped']);self.assertFalse(signals)


if __name__=='__main__':unittest.main()
