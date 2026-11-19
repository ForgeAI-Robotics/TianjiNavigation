import sys,unittest,io,json,struct,collections
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'modules'))
from continuity import OdometryHistory
from relay_scheduling import FrameSchedule
from wire import write_packet,read_packet
from client import SensorStream

class RelayDeliveryTests(unittest.TestCase):
 def test_late_odometry_does_not_discard_frame_or_spend_quota(self):
  history=OdometryHistory();history.add(100.,[0,0,0]);s=FrameSchedule(3.)
  s.offer('front','front-message',100.08,10.);s.offer('back','back-message',100.,10.)
  job=s.take(10.,100.1,history);self.assertEqual(job[0],'back');s.sent('back',100.,10.,{})
  self.assertIn('front',s.pending);self.assertNotIn('front',s.last_sent)
  history.add(100.08,[0,0,0]);job=s.take(10.01,100.11,history);self.assertEqual(job[0],'front');self.assertEqual(job[4][0],100.08)
 def test_latest_unpaired_scan_cannot_erase_an_older_strictly_matched_scan(self):
  h=OdometryHistory();h.add(100.,[0,0,0]);s=FrameSchedule(3.)
  s.offer('front','matched-scan',100.,10.);s.offer('front','ahead-of-odometry',100.08,10.08)
  job=s.take(10.1,100.1,h);self.assertEqual(job[1],'matched-scan');self.assertEqual(job[4][0],100.)
  self.assertEqual(len(s.pending['front']),1);self.assertEqual(s.pending['front'][0][1],100.08)
 def test_capture_queue_is_bounded_and_never_replays_old_forwarded_scans(self):
  h=OdometryHistory();s=FrameSchedule(3.)
  for i in range(20):s.offer('front',str(i),100.+i*.01,10.+i*.01)
  self.assertEqual(len(s.pending['front']),5);h.add(100.19,[0,0,0]);job=s.take(10.2,100.2,h);self.assertEqual(job[3],100.19)
  s.sent('front',job[3],10.2,{});s.offer('front','old-reordered',100.,10.21)
  self.assertEqual(s.counts['front']['older_than_last_forwarded'],1)
 def test_round_robin_stops_front_priority_under_a_busy_loop(self):
  h=OdometryHistory();h.add(100.,[0,0,0]);s=FrameSchedule(3.)
  for sensor in ['front','back']:s.offer(sensor,sensor,100.,10.)
  self.assertEqual(s.take(10.,100.,h)[0],'front');s.sent('front',100.,10.,{})
  s.offer('front','new-front',100.,10.4)
  self.assertEqual(s.take(10.4,100.4,h)[0],'back')
 def test_expired_frame_rejected_without_rate_penalty_for_next_valid_frame(self):
  h=OdometryHistory();h.add(100.,[0,0,0]);s=FrameSchedule(3.);s.offer('front','old',99.,10.)
  self.assertIsNone(s.take(10.,100.,h));s.offer('front','new',100.,10.01)
  self.assertEqual(s.take(10.01,100.01,h)[1],'new')
 def test_unmatched_odometry_never_interpolates_or_relaxes_tolerance(self):
  h=OdometryHistory();h.add(100.,[0,0,0]);s=FrameSchedule(3.);s.offer('front','missing',100.08,10.)
  self.assertIsNone(s.take(10.,100.1,h));self.assertNotIn('front',s.last_sent)
 def test_continuous_pairing_under_delayed_odometry_with_simulated_motion(self):
  import math
  h=OdometryHistory();s=FrameSchedule(3.);events=[]
  for i in range(12000):
   stamp=i*.005;events.append((stamp+.22+.025*math.sin(stamp), 'odom', stamp))
  for i in range(600):
   for sensor,phase in [('front',0.),('back',.04)]:
    stamp=i*.1+phase;events.append((stamp+.1,sensor,stamp))
  events.sort();index=0;outputs={'front':[],'back':[]}
  for tick in range(6000):
   now=tick*.01
   while index<len(events)and events[index][0]<=now:
    arrival,kind,stamp=events[index];index+=1
    if kind=='odom':h.add(100.+stamp,[.03*stamp,0,.02*stamp])
    else:s.offer(kind,stamp,100.+stamp,10.+arrival)
   job=s.take(10.+now,100.+now,h)
   if job:
    sensor,_,_,stamp,matched=job;self.assertLessEqual(abs(stamp-matched[0]),.05+1e-10)
    outputs[sensor].append(now);s.sent(sensor,stamp,10.+now,{})
  for values in outputs.values():
   self.assertGreater(len(values),160);self.assertLess(max(b-a for a,b in zip(values,values[1:])),.5)
 def test_both_payload_codecs_preserve_exact_points_and_original_stamp(self):
  points=np.random.default_rng(3).normal(size=(200,3)).astype('f4')
  for encoding in ['zlib','raw_f32_le']:
   stream=io.BytesIO();write_packet(stream,dict(observation_time=123.5),points,point_encoding=encoding);stream.seek(0);h,q=read_packet(stream)
   self.assertEqual(h['observation_time'],123.5);np.testing.assert_array_equal(q,points)
 def test_invalid_raw_length_and_unknown_codec_rejected(self):
  for encoding,payload in [('raw_f32_le',b'bad'),('unsupported',b'x'*12)]:
   h=json.dumps(dict(protocol=1,point_count=1,point_encoding=encoding)).encode();stream=io.BytesIO(struct.pack('!II',len(h),len(payload))+h+payload)
   with self.assertRaises(ValueError):read_packet(stream)
 def test_local_pipe_age_is_added_instead_of_restamping_observation(self):
  stream=io.BytesIO();write_packet(stream,dict(kind='frame',challenge='token',source_emitted_monotonic=9.9,source_age_s=.2,observation_time=123.))
  stream.seek(0);s=SensorStream.__new__(SensorStream);s.remote=type('Remote',(),dict(stdout=stream))();s.tokens=collections.deque([('token',9.8)])
  with patch('client.time.monotonic',return_value=10.):h,_=s.receive()
  self.assertAlmostEqual(h['source_age_s'],.3);self.assertAlmostEqual(h['transport_delay_s'],.1);self.assertEqual(h['observation_time'],123.)
if __name__=='__main__':unittest.main()
