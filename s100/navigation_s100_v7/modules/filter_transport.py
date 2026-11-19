"""Bounded binary point-filter transport. Contains no motion interface."""
import hashlib,json,struct
MAX_HEADER=65536
MAX_PAYLOAD=4_000_000

def exact(stream,n):
    data=bytearray()
    while len(data)<n:
        part=stream.read(n-len(data))
        if not part:raise EOFError('FILTER_CONNECTION_CLOSED')
        data.extend(part)
    return bytes(data)

def receive(stream):
    h,n=struct.unpack('!II',exact(stream,8))
    if not 0<h<=MAX_HEADER or not 0<=n<=MAX_PAYLOAD:raise ValueError('FILTER_PACKET_BUDGET')
    return json.loads(exact(stream,h)),exact(stream,n)

def send(stream,header,payload=b''):
    h=json.dumps(header,allow_nan=False,separators=(',',':')).encode()
    if not 0<len(h)<=MAX_HEADER or len(payload)>MAX_PAYLOAD:raise ValueError('FILTER_PACKET_BUDGET')
    packet=memoryview(struct.pack('!II',len(h),len(payload))+h+payload)
    total=len(packet)
    while packet:
        n=stream.write(packet)
        if not n:raise EOFError('FILTER_WRITE_CLOSED')
        packet=packet[n:]
    stream.flush()
    return total

def request_digest(header,payload):
    return hashlib.sha256(json.dumps(header,allow_nan=False,sort_keys=True,separators=(',',':')).encode()+payload).hexdigest()
