import unittest,tempfile,sys,time,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from sensor_bus import SensorBus,SnapshotStream

class SensorBusTests(unittest.TestCase):
 def test_filter_worker_reads_only_its_own_sensor(self):
  from unittest.mock import patch
  with tempfile.TemporaryDirectory()as d:
   b=SensorBus(Path(d)/'bus');s=SnapshotStream(b.folder,names=('front',))
   b.publish(dict(kind='odometry'),None)
   for sensor in ['back','front']:b.publish(dict(kind='frame',sensor=sensor,observation_time=123.,source_age_s=.1),np.ones((1,3)))
   original=np.load;opened=[]
   def tracked(file,*args,**kwargs):opened.append(Path(file.name).name);return original(file,*args,**kwargs)
   with patch('sensor_bus.np.load',side_effect=tracked):header,_=s.receive()
   self.assertEqual(opened,['front.npz']);self.assertEqual(header['observation_time'],123.);self.assertGreaterEqual(header['source_age_s'],.1)
   for names in [(),('bad',),('front','front')]:
    with self.assertRaises(ValueError):SnapshotStream(b.folder,names=names)
 def test_two_consumers_receive_identical_original_raw_data_and_stamp(self):
  with tempfile.TemporaryDirectory() as d:
   bus=SensorBus(Path(d)/'bus');one=SnapshotStream(bus.folder);two=SnapshotStream(bus.folder);p=np.array([[.12,.24,.98]],dtype=np.float32);h=dict(kind='frame',sensor='back',sequence=7,observation_time=123.5,source_age_s=.12)
   bus.publish(h,p,time.monotonic()-.03)
   for stream in [one,two]:
    got,q=stream.receive();self.assertEqual(got['observation_time'],123.5);self.assertEqual(got['sequence'],7);self.assertGreaterEqual(got['source_age_s'],.15);np.testing.assert_array_equal(q,p)
   self.assertEqual(h['source_age_s'],.12)
 def test_latest_frames_replace_old_data_and_odometry_does_not_starve_clouds(self):
  with tempfile.TemporaryDirectory() as d:
   b=SensorBus(Path(d)/'bus');s=SnapshotStream(b.folder)
   for sequence in [1,2]:b.publish(dict(kind='frame',sensor='front',sequence=sequence),np.array([[sequence,0,0]]))
   b.publish(dict(kind='odometry',observation_time=12),None)
   h,p=s.receive();self.assertEqual(h['kind'],'odometry');self.assertIsNone(p)
   b.publish(dict(kind='odometry',observation_time=13),None)
   h,p=s.receive();self.assertEqual(h['kind'],'frame');self.assertEqual(h['sequence'],2)
 def test_source_diagnostics_are_retained_without_becoming_sensor_data(self):
  with tempfile.TemporaryDirectory()as d:
   b=SensorBus(Path(d)/'bus');h=dict(kind='sensor_status',sensors={'front':{'counts':{'received':10}}})
   b.publish(h,None);self.assertEqual(json.loads((b.folder/'relay_status.json').read_text()),h)
   stream=SnapshotStream(b.folder);stream.settimeout(.01)
   with self.assertRaises(TimeoutError):stream.receive()
 def test_stale_snapshot_is_not_retimestamped_or_reused(self):
  with tempfile.TemporaryDirectory() as d:
   b=SensorBus(Path(d)/'bus');s=SnapshotStream(b.folder);s.settimeout(.01)
   b.publish(dict(kind='frame',sensor='front',observation_time=1),np.ones((1,3)),time.monotonic()-2.)
   with self.assertRaises(TimeoutError):s.receive()

if __name__=='__main__':unittest.main()
