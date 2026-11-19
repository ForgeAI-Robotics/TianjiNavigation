"""Monitor must remain GET-only, even while downstream reports blocked."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
import brain_monitor_client as client

class Response:
    def __init__(self,data):self.data=json.dumps(data).encode()
    def __enter__(self):return self
    def __exit__(self,*a):pass
    def read(self,*a):return self.data

class Tests(unittest.TestCase):
    def test_read_only_sampling_preserves_blocked_status(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);token=root/'token';token.write_text('fixture-monitor-token-xxxxxxxxxxxx');token.chmod(0o600)
            config=root/'config.json';config.write_text(json.dumps(dict(url='http://192.168.31.144:15001',token_file=str(token))))
            seen=[]
            def get(request,timeout):
                seen.append((request.get_method(),request.full_url,request.data,timeout))
                return Response({'rows':[]} if request.full_url.endswith('/api/logs') else {'link':{'online':True,'navigation_status':{'motion_ready':False}}})
            with patch.object(client.urllib.request,'urlopen',get):packet=client.sample(config)
            self.assertFalse(packet['monitor']['link']['navigation_status']['motion_ready'])
            self.assertEqual([row[0] for row in seen],['GET','GET']);self.assertTrue(all(row[2] is None for row in seen))
            self.assertEqual(len(seen),2)
    def test_foreign_gateway_and_public_token_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);token=root/'token';token.write_text('fixture-monitor-token-xxxxxxxxxxxx');token.chmod(0o644)
            config=root/'config.json';config.write_text(json.dumps(dict(url='http://192.168.31.144:15001',token_file=str(token))))
            with self.assertRaisesRegex(ValueError,'PRIVATE'):client.read_config(config)
            config.write_text(json.dumps(dict(url='http://192.168.31.144:5000',token_file=str(token))))
            with self.assertRaisesRegex(ValueError,'ONLY_TIANJI'):client.read_config(config)

if __name__=='__main__':unittest.main()
