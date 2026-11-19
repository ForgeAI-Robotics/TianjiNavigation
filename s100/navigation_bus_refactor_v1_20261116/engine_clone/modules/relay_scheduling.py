"""Fair bounded capture/odometry pairing. No ROS, IO or actuator access."""
import collections,math

class FrameSchedule:
 def __init__(self,hz):
  if not math.isfinite(hz) or not 1<=hz<=5:raise ValueError('INVALID_CAPTURE_RATE')
  self.period=1./hz;self.pending={};self.last_sent={};self.next=0
  self.counts={s:collections.Counter()for s in ['front','back']};self.latest={};self.errors={};self.last_forwarded={};self.gap_max={s:dict(callback_s=0.,source_stamp_s=0.,forwarded_s=0.)for s in self.counts};self.timing_max={s:{}for s in self.counts}
 def offer(self,sensor,message,stamp,arrival):
  self.counts[sensor]['received']+=1
  if sensor in self.latest:
   stamp0,arrival0=self.latest[sensor];self.gap_max[sensor]['callback_s']=max(self.gap_max[sensor]['callback_s'],arrival-arrival0);self.gap_max[sensor]['source_stamp_s']=max(self.gap_max[sensor]['source_stamp_s'],stamp-stamp0)
  self.latest[sensor]=(stamp,arrival)
  if sensor in self.last_forwarded and stamp<=self.last_forwarded[sensor]['source_stamp']:
   self.counts[sensor]['older_than_last_forwarded']+=1;return
  queue=self.pending.setdefault(sensor,collections.deque(maxlen=5))
  if len(queue)==queue.maxlen:self.counts[sensor]['bounded_queue_superseded']+=1
  queue.append((message,stamp,arrival))
 def take(self,now,wall,history):
  names=['front','back']
  for offset in range(2):
   index=(self.next+offset)%2;sensor=names[index]
   if sensor not in self.pending or now-self.last_sent.get(sensor,-1e9)<self.period:continue
   queue=self.pending[sensor];valid=[]
   for row in queue:
    msg,stamp,arrival=row;age=wall-stamp
    if now-arrival>.5 or not -.05<=age<=.7:
     reason='arrival_expired'if now-arrival>.5 else 'source_stamp_expired_or_future';self.counts[sensor][reason]+=1;self.errors[sensor]=reason
    else:valid.append(row)
   self.pending[sensor]=collections.deque(valid,maxlen=5)
   # Use the NEWEST strictly matched scan. Keeping a few recent unmatched
   # scans prevents faster cloud callbacks from continually erasing the scan
   # whose measured odometry is just about to arrive. Never pool point data.
   for row in sorted(valid,key=lambda x:x[1],reverse=True):
    msg,stamp,arrival=row;matched=history.nearest(stamp)
    if matched is None:continue
    self.pending[sensor]=collections.deque((v for v in valid if v[1]>stamp),maxlen=5);self.next=(index+1)%2
    return sensor,msg,arrival,stamp,matched
   if valid:self.counts[sensor]['odometry_pair_wait_checks']+=1;self.errors[sensor]='WAIT_ODOMETRY_PAIR'
  return None
 def sent(self,sensor,stamp,now,timing):
  if sensor in self.last_sent:self.gap_max[sensor]['forwarded_s']=max(self.gap_max[sensor]['forwarded_s'],now-self.last_sent[sensor])
  for k,v in timing.items():
   if k.endswith('_s')and isinstance(v,(int,float)):self.timing_max[sensor][k]=max(self.timing_max[sensor].get(k,0.),v)
  self.last_sent[sensor]=now;self.counts[sensor]['forwarded']+=1;self.errors[sensor]=None;self.last_forwarded[sensor]=dict(source_stamp=stamp,**timing)
 def failed(self,sensor,reason):self.counts[sensor]['decode_or_send_failures']+=1;self.errors[sensor]=reason
 def snapshot(self,now,wall):
  return {s:dict(counts=dict(self.counts[s]),pending_frames=len(self.pending.get(s,[])),gap_max_s=dict(self.gap_max[s]),timing_max_s=dict(self.timing_max[s]),last_error=self.errors.get(s),latest_source_age_s=None if s not in self.latest else wall-self.latest[s][0],latest_callback_age_s=None if s not in self.latest else now-self.latest[s][1],last_forwarded=self.last_forwarded.get(s))for s in self.counts}
