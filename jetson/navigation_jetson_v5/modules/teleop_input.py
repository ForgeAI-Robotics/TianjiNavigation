"""Terminal input policy. No networking, ROS, or motion side effects."""
class KeyboardCommand:
    def __init__(self, config):
        self.config = config
        self.scale = 1.
        self.command = [0., 0., 0.]
        self.key_at = -1e9
        self.last_loop = None
        self.escape = None

    def clear(self):
        self.command = [0., 0., 0.]
        self.key_at = -1e9

    def update(self, data, now, link_ready):
        # Input buffered across a stalled loop is old, even when read just now.
        stalled = self.last_loop is not None and now - self.last_loop > .25
        self.last_loop = now
        if b'\x03' in data or not link_ready or stalled:
            self.clear()
            self.escape = None
            return b'\x03' in data
        keys = []
        for byte in data:
            if byte == 27:
                self.escape = 'start'
                continue
            if self.escape == 'start':
                self.escape = 'sequence' if byte in (ord('['), ord('O')) else None
                continue
            if self.escape == 'sequence':
                if 0x40 <= byte <= 0x7e:
                    self.escape = None
                continue
            keys.append(chr(byte).lower())
        # Stop always wins over movement queued in the same read.
        if any(k in ' x' for k in keys):
            self.clear()
            return False
        for key in keys:
            if key in '+=-':
                self.scale = max(.5, min(2., self.scale + (.25 if key in '+=' else -.25)))
                self.clear()
            elif key in 'wasdqe':
                x = self.config['initial_x'] * self.scale
                y = self.config['initial_y'] * self.scale
                w = self.config['initial_wz'] * self.scale
                self.command = {'w':[x,0.,0.], 's':[-x,0.,0.], 'a':[0.,0.,w],
                                'd':[0.,0.,-w], 'q':[0.,y,0.], 'e':[0.,-y,0.]}[key]
                self.key_at = now
            else:
                self.clear()
        return False

    def output(self, now):
        age = max(0., now - self.key_at)
        return (list(self.command) if age <= .2 else [0.,0.,0.]), min(age,9999.)
