"""Pinned SSH raw-CDR transport; bounded and age-checked on both hosts."""
import collections,json,secrets,struct,threading,time
from client import Remote
from wire import read_exact
class RawStream:
    def __init__(self):
        self.remote=Remote('fast_raw_relay.py');self.tokens=collections.deque(maxlen=30);self.stop=threading.Event()
        self.thread=threading.Thread(target=self.challenge,daemon=True);self.thread.start()
    def challenge(self):
        try:
            while not self.stop.is_set():
                token=secrets.token_hex(12);self.tokens.append((token,time.monotonic()));self.remote.send(dict(kind='raw_challenge',challenge=token));self.stop.wait(.1)
        except (OSError,EOFError):self.stop.set()
    def receive(self):
        nh,nb=struct.unpack('!II',read_exact(self.remote.stdout,8))
        if not 0<nh<=65536 or nb>4000000:raise ValueError('RAW_RECORD_TOO_LARGE')
        h=json.loads(read_exact(self.remote.stdout,nh));data=read_exact(self.remote.stdout,nb)
        issued=next((t for token,t in list(self.tokens) if token==h.get('challenge')),None)
        if issued is None or not 0<=time.monotonic()-issued<=.5:raise ValueError('STALE_RAW_TRANSPORT')
        return h,data
    def close(self):
        self.stop.set();self.remote.ssh.close();self.remote.close();self.thread.join(timeout=1)
