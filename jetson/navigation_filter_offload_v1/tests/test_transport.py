import io,json,struct,sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from filter_transport import send,receive,request_digest,MAX_HEADER
from filter_offload_client import FilterClient

class TransportTests(unittest.TestCase):
    def test_packet_roundtrip(self):
        stream=io.BytesIO();send(stream,{'x':1},b'points');stream.seek(0)
        self.assertEqual(receive(stream),({'x':1},b'points'))
    def test_truncated_packet_rejected(self):
        with self.assertRaises(EOFError):receive(io.BytesIO(b'123'))
    def test_oversized_header_rejected_before_allocation(self):
        with self.assertRaises(ValueError):receive(io.BytesIO(struct.pack('!II',MAX_HEADER+1,0)))
    def test_request_identity_binds_points_and_original_time(self):
        a=request_digest({'observation_time':1},b'a')
        self.assertNotEqual(a,request_digest({'observation_time':2},b'a'))
        self.assertNotEqual(a,request_digest({'observation_time':1},b'b'))
    def client(self):
        c=FilterClient.__new__(FilterClient);c.config={'package_id':'bundle','backend':'cuda_obb'}
        c.model_sha='model';c.stamp='config';c.stream=io.BytesIO();c.socket=None
        return c
    def test_old_or_foreign_reply_is_rejected(self):
        c=self.client();h={'observation_time':1,'joints':{'observation_time':1}}
        with patch('filter_offload_client.receive',return_value=({'id':'foreign'},b'')):
            with self.assertRaisesRegex(ValueError,'IDENTITY'):c.process(h,np.zeros((1,3)),conservative_fast=True,refine_near_hits=True,refine_radius_m=.16)
    def test_network_timeout_is_not_a_valid_filter_result(self):
        c=self.client();h={'observation_time':1,'joints':{'observation_time':1}}
        with patch('filter_offload_client.receive',side_effect=TimeoutError()):
            with self.assertRaises(TimeoutError):c.process(h,np.zeros((1,3)),conservative_fast=True,refine_near_hits=True,refine_radius_m=.16)

if __name__=='__main__':unittest.main()
