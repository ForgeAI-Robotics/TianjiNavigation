"""A failed GUI plan must replace the stale last-task record, without real IO."""
import ast
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


class PlanFailureLogTests(unittest.TestCase):
    def test_failed_plan_is_saved_before_reporting_failure(self):
        project=Path(__file__).resolve().parents[1];file=project.parent/'navigation_waypoints_v6/modules/onboard_api_client.py'
        # The GUI frontend lives on Jetson; S100 retains only this test file.
        if not file.exists():self.skipTest('Jetson frontend only')
        node=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='plan_goals')
        failed={'id':'b'*32,'state':'FAILED','result':{'code':'NO_CHECKED_PLAN'}}
        class Session:
            backend='bus'
            def __enter__(self):
                self.client=types.SimpleNamespace(plan=lambda *a,**kw:{'id':failed['id']},wait=lambda *a,**kw:failed)
                return self
            def __exit__(self,*a):pass
        targets=types.ModuleType('targets');targets.scene_id=lambda:'office'
        preview=types.ModuleType('preview_options');preview.load=lambda:{}
        onboard=types.ModuleType('onboard_client');onboard.fetch_latest=lambda *a:self.fail('Failed plan has no artifact')
        common=types.ModuleType('common');common.atomic_json=lambda p,d:p.write_text(json.dumps(d))
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);(root/'runtime').mkdir();file=root/'runtime/last_api_task.json';file.write_text(json.dumps({'state':'SUCCEEDED','kind':'navigate'}))
            namespace={'ROOT':root,'Session':Session,'print':lambda *a,**kw:None}
            exec(compile(ast.Module(body=[node],type_ignores=[]),'gui-plan-fixture','exec'),namespace)
            with patch.dict(sys.modules,{'targets':targets,'preview_options':preview,'onboard_client':onboard,'common':common}):
                with self.assertRaisesRegex(RuntimeError,'API规划未通过'):namespace['plan_goals']([1])
            self.assertEqual(json.loads(file.read_text()),failed)


if __name__=='__main__':unittest.main()
