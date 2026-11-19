#!/usr/bin/env python3
"""Read-only four-timestamp clock mapping. Does not set either system clock."""
import argparse
from collections import deque
import json
import math
import secrets
import socket
import time

PORT = 17480


class ClockMap:
    def __init__(self):
        self.rows = deque(maxlen=32)
        self.anchor = None
        self.generation = 0

    def clear(self):
        self.rows.clear()
        self.generation += 1

    def add(self, t0, t1, t2, t3, mono0, mono3):
        values = (t0, t1, t2, t3, mono0, mono3)
        if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
            return False
        elapsed = mono3-mono0
        if elapsed < 0 or abs((t3-t0)-elapsed) > .002:
            self.clear()
            return False
        if self.anchor is not None:
            wall, mono = self.anchor
            if abs((t3-wall)-(mono3-mono)) > .002:
                self.clear()
        self.anchor = (t3, mono3)
        delay = (t3-t0)-(t2-t1)
        if t2 < t1 or not 0 <= delay <= .010:
            return False
        offset = ((t1-t0)+(t2-t3))/2  # remote minus local
        recent = [r for r in self.rows if mono3-r['mono'] < 2.]
        if recent and abs(offset-min(recent,key=lambda r:r['delay'])['offset']) > .005:
            self.clear()
        self.rows.append(dict(mono=mono3, wall=t3, offset=offset, delay=delay))
        return True

    def estimate(self, now=None, wall=None):
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        if self.anchor is not None and abs((wall-self.anchor[0])-(now-self.anchor[1])) > .002:
            self.clear()
        rows = [r for r in self.rows if 0 <= now-r['mono'] <= 2.]
        if len(rows) < 3:
            raise ValueError('CLOCK_MAP_NOT_READY_OR_EXPIRED')
        rows=rows[-5:]
        # Each exchange bounds the offset without assuming symmetric delays.
        # Intersect these intervals; a slow/asymmetric packet cannot widen a
        # more precise exchange. Allow 1ms/s relative clock slew between probes.
        best = min(rows, key=lambda r:r['delay'])
        lower=max(r['offset']-r['delay']/2-.0005-.001*(now-r['mono']) for r in rows)
        upper=min(r['offset']+r['delay']/2+.0005+.001*(now-r['mono']) for r in rows)
        if lower>upper:
            self.clear()
            raise ValueError('CLOCK_MAP_INCONSISTENT')
        uncertainty = (upper-lower)/2
        if uncertainty > .005:
            raise ValueError('CLOCK_MAP_UNCERTAINTY_TOO_LARGE')
        return dict(remote_minus_local_s=(lower+upper)/2, uncertainty_s=uncertainty,
                    round_trip_s=best['delay'], age_s=now-best['mono'],
                    samples=len(rows), generation=self.generation,
                    timestamp_domain='S100_SYSTEM_REALTIME', method='wired four timestamps')


def probe(sock, mapping, host='192.168.1.100'):
    nonce = secrets.token_hex(12)
    t0, m0 = time.time(), time.monotonic()
    sock.sendto(json.dumps({'nonce':nonce}).encode(), (host, PORT))
    try:
        raw, peer = sock.recvfrom(1024)
        t3, m3 = time.time(), time.monotonic()
        reply = json.loads(raw)
        if peer != (host, PORT) or reply.get('nonce') != nonce:
            return False
        return mapping.add(t0,reply['received_at'],reply['sent_at'],t3,m0,m3)
    except (OSError,ValueError,KeyError,TypeError):
        return False


def serve():
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(('192.168.1.100',PORT))
    while True:
        raw, peer = sock.recvfrom(1024)
        received = time.time()
        if peer[0] != '192.168.1.10':
            continue
        try:
            request=json.loads(raw)
            nonce=request['nonce']
            if not isinstance(nonce,str) or len(nonce)!=24:
                continue
            reply=dict(nonce=nonce,received_at=received,sent_at=time.time())
            sock.sendto(json.dumps(reply).encode(),peer)
        except (ValueError,KeyError,TypeError):
            continue


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role',choices=['serve','probe'])
    args=parser.parse_args()
    if args.role=='serve':
        serve()
    else:
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.settimeout(.02)
        mapping=ClockMap()
        for _ in range(12):
            probe(sock,mapping);time.sleep(.1)
        print(json.dumps(mapping.estimate(),indent=2))
