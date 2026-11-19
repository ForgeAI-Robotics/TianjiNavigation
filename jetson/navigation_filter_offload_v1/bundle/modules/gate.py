"""Pure command admission. All deadlines use the receiving host's clock."""
import math,secrets
from collections import deque

class CommandGate:
    def __init__(self,config):
        self.config=config;self.challenges=deque(maxlen=30);self.sequence=-1;self.received=None
        self.packet=None;self.reason='WAITING_FOR_AUTHENTICATED_SESSION';self.session=secrets.token_hex(16)

    def challenge(self,now):
        token=secrets.token_hex(12);self.challenges.append((token,now));return token

    def accept(self,packet,now):
        try:
            sequence=packet['sequence'];command=packet['command'];age=float(packet['key_age_s'])
            issued=next(t for token,t in self.challenges if token==packet['challenge'])
            if type(sequence) is not int or sequence<=self.sequence or not 0<=now-issued<=self.config['protocol']['challenge_ttl_s']:raise ValueError('STALE_OR_REPLAYED_COMMAND')
            if len(command)!=3 or not all(math.isfinite(float(x)) for x in command) or not math.isfinite(age) or age<0:raise ValueError('INVALID_COMMAND')
            if age>.25:command=[0.,0.,0.]
            self.sequence=sequence;self.received=now;self.packet=dict(packet,command=[float(x) for x in command],key_age_s=age);return True
        except (KeyError,TypeError,ValueError,StopIteration,OverflowError):
            self.reason='REJECTED_OR_EXPIRED_PACKET';return False

    def output(self,now,legacy_mapping=False):
        if self.packet is None or not 0<=now-self.received<=self.config['protocol']['command_ttl_s']:
            self.reason='LINK_TIMEOUT_LOCAL_STOP';return [0.,0.,0.]
        if now-self.received+self.packet['key_age_s']>.25:
            self.reason='KEY_RELEASE_LOCAL_STOP';return [0.,0.,0.]
        if legacy_mapping:self.reason='S100_LEGACY_MAPPING_ACTIVE';return [0.,0.,0.]
        if 'navigation' in self.packet:
            from navigation_admission import admission
            if not admission(self.packet['navigation'],now-self.received):
                self.reason='NAVIGATION_GUARD_OR_LEASE_EXPIRED_STOP';return [0.,0.,0.]
        limits=self.config['teleop'];mapping=self.packet.get('mapping',{})
        if not isinstance(mapping,dict):self.reason='INVALID_MAPPING_STATE';return [0.,0.,0.]
        if mapping.get('state')=='idle':linear=math.hypot(limits['max_x'],limits['max_y']);angular=limits['max_wz'];self.reason='NORMAL_TELEOP'
        elif mapping.get('state')=='healthy':
            try:
                cfg=self.config['mapping_teleop'];since=now-self.received
                if mapping.get('backend')=='fast_lio2':
                    for key in ('estimator_age_s','imu_age_s','adapter_age_s'):
                        age=mapping[key]+since
                        if not math.isfinite(age) or not 0<=age<=.5:raise ValueError()
                if mapping.get('invalid') or mapping.get('paused') or not 0<=mapping['heartbeat_age_s']+since<=cfg['heartbeat_timeout_s']:raise ValueError()
                intervals=[]
                for sensor in ('front','back'):
                    s=mapping['sensors'][sensor];age=s['age_s']+since;interval=s['interval_s'];duration=s['processing_s']
                    if not all(math.isfinite(v) for v in [age,interval,duration]) or not 0<=age<=cfg['capture_timeout_s'] or not 0<interval<=cfg['capture_timeout_s'] or not 0<=duration<=cfg['max_processing_s']:raise ValueError()
                    intervals.extend([age,interval])
                scale=1/max(1.,*intervals);linear=cfg['max_translation_mps']*scale;angular=cfg['max_rotation_radps']*scale;self.reason='MAPPING_SPEED_LIMIT'
            except (KeyError,TypeError,ValueError):self.reason='MAPPING_UNHEALTHY_STOP';return [0.,0.,0.]
        else:self.reason='MAPPING_UNKNOWN_OR_FAILED_STOP';return [0.,0.,0.]
        x,y,w=self.packet['command'];x=max(-limits['max_x'],min(limits['max_x'],x));y=max(-limits['max_y'],min(limits['max_y'],y));n=math.hypot(x,y)
        if n>linear:x*=linear/n;y*=linear/n
        return [x,y,max(-angular,min(angular,w))]

def snapshot_mapping(root,now,lock_held):
    import json
    try:
        path=root/'runtime/mapping_status.json';data=json.loads(path.read_text()) if path.exists() else {}
        running=lock_held(root/'runtime/mapping.lock')
        if not running:return dict(state='failed' if data.get('active') else 'idle')
        if (root/'runtime/mapping.lock').read_text().strip()!=str(data.get('pid')):return dict(state='failed')
        if not data.get('active') or data.get('invalid') or data.get('paused') or not data.get('input_ready') or data.get('readonly_test_inhibit'):return dict(state='failed')
        extra={}
        if data.get('backend')=='fast_lio2':
            for output,source in [('estimator_age_s','estimator_completed_monotonic'),('imu_age_s','imu_completed_monotonic'),('adapter_age_s','adapter_completed_monotonic')]:
                age=now-data[source]
                if not math.isfinite(age) or not 0<=age<=.5:return dict(state='failed')
                extra[output]=age
            extra['backend']='fast_lio2'
        return dict(**extra,state='healthy',heartbeat_age_s=now-data['heartbeat_monotonic'],sensors={name:dict(age_s=now-value['completed_monotonic'],interval_s=value['interval_s'],processing_s=value['processing_s']) for name,value in data.get('sensors',{}).items()})
    except (OSError,ValueError,TypeError,KeyError):return dict(state='failed')
