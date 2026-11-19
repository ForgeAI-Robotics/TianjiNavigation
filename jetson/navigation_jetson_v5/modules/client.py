"""Pinned SSH transports. No discovery outside the dedicated wired peer."""
import collections,json,shlex,threading,time
import paramiko
from common import ROOT,CONFIG
from wire import read_packet,write_packet

class Remote:
    def __init__(self,module,args=()):
        c=json.loads((ROOT/'config/connection.json').read_text())
        if c['host']!='192.168.1.10':raise RuntimeError('Expected the dedicated S100 wired address')
        self.ssh=paramiko.SSHClient();self.ssh.load_host_keys(str(ROOT/'config/known_hosts'));self.ssh.set_missing_host_key_policy(paramiko.RejectPolicy())
        self.ssh.connect(c['host'],username=c['username'],password=c['password'],timeout=8,auth_timeout=8,banner_timeout=8)
        self.ssh.get_transport().set_keepalive(1)
        command='source '+shlex.quote(CONFIG['remote_root']+'/env_s100.sh')+'\nexec /usr/bin/python3 '+shlex.quote(CONFIG['remote_root']+'/modules/'+module)+''.join(' '+shlex.quote(x) for x in args)
        self.stdin,self.stdout,self.stderr=self.ssh.exec_command('bash -c '+shlex.quote(command))
        self.error_tail=collections.deque(maxlen=20);self.send_lock=threading.Lock();self.closed=False
        def errors():
            try:
                for line in self.stderr:self.error_tail.append(line.rstrip())
            except (OSError,EOFError):pass
        threading.Thread(target=errors,daemon=True).start()
    def send(self,packet):
        with self.send_lock:write_packet(self.stdin,packet)
    def receive(self):return read_packet(self.stdout)
    def close(self):
        if self.closed:return
        self.closed=True
        for stream in (self.stdin,self.stdout,self.stderr):
            try:stream.close()
            except (OSError,EOFError):pass
        self.ssh.close()

class SensorStream:
    def __init__(self,localization=False):
        self.remote=Remote('sensor_relay.py',['--localization'] if localization else []);self.tokens=collections.deque(maxlen=40);self.stopped=threading.Event()
        self.thread=threading.Thread(target=self.challenge_loop,daemon=True);self.thread.start()
    def challenge_loop(self):
        import secrets
        try:
            while not self.stopped.is_set():
                token=secrets.token_hex(12);self.tokens.append((token,time.monotonic()));self.remote.send(dict(kind='sensor_challenge',challenge=token));self.stopped.wait(.1)
        except (OSError,EOFError):self.stopped.set()
    def receive(self):
        header,points=self.remote.receive();now=time.monotonic()
        issued=next((t for token,t in list(self.tokens) if token==header.get('challenge')),None)
        if issued is None or not 0<=now-issued<=CONFIG['protocol']['sensor_challenge_ttl_s']:raise ValueError('QUEUED_OR_STALE_SENSOR_PACKET')
        return header,points
    def close(self):
        self.stopped.set();self.remote.close();self.thread.join(timeout=1)
