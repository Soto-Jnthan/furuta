import numpy as np
from scipy import signal

class VelocityFilter_CtD:
    def __init__(self, x_len, dt, num=(50, 0), den=(1, 50), x_init=None):
        derivative_filter = signal.cont2discrete((num, den), dt)
        self.b = derivative_filter[0].ravel().astype(np.float32)
        self.a = derivative_filter[1].astype(np.float32)
        if x_init is None:
            self.z = np.zeros((max(len(self.a), len(self.b)) - 1, x_len), dtype=np.float32)
        else:
            self.set_initial_state(x_init)

    def set_initial_state(self, x_init):
        assert isinstance(x_init, np.ndarray)
        zi = signal.lfilter_zi(self.b, self.a)
        self.z = np.outer(zi, x_init)

    def __call__(self, x):
        xd, self.z = signal.lfilter(self.b, self.a, x[None, :], 0, self.z)
        return xd.ravel()
    
class MAFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer = []

    def reset(self):
        self.buffer = []

    def __call__(self, s):
        self.buffer.append(s)
        if len(self.buffer) > self.window_size:
            del self.buffer[0]
        return np.mean(self.buffer)    

class MMFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer = []

    def reset(self):
        self.buffer = []

    def __call__(self, s):
        self.buffer.append(s)
        if len(self.buffer) > self.window_size:
            del self.buffer[0]
        return np.median(self.buffer)

class EMAFilter:
    def __init__(self, ema_init=0, alpha=0.5):
        self.EMA = ema_init
        self.alpha = alpha

    def reset(self, ema_init=0):
        self.EMA = ema_init

    def __call__(self, s):
        self.EMA = self.alpha * s + (1 - self.alpha) * self.EMA    
        return self.EMA