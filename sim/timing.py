"""CAN transport, burst latency and a non-blocking async worker.

Latency per message is U(can_min, can_max). Burst *episodes* arrive as a Poisson
process (burst_rate_hz); inside one, every message takes ~can_burst. Messages
can therefore arrive out of order; receivers keep the highest sequence number
(stale overtaken messages are discarded, as a real firmware would).
"""
import heapq


class LatencyModel:
    def __init__(self, tc, rng):
        self.tc, self.rng = tc, rng
        self.burst_until = -1.0
        self.next_burst = self._draw_next(0.0)

    def _draw_next(self, t):
        if self.tc.burst_rate_hz <= 0:
            return float("inf")
        return t + self.rng.exponential(1.0 / self.tc.burst_rate_hz)

    def in_burst(self, t):
        while t >= self.next_burst:
            lo, hi = self.tc.burst_len_s
            self.burst_until = max(self.burst_until, self.next_burst + self.rng.uniform(lo, hi))
            self.next_burst = self._draw_next(self.next_burst)
        return t < self.burst_until

    def sample(self, t):
        """Returns latency in s, or None if the message is lost."""
        tc = self.tc
        for t0, t1 in tc.blackout:
            if t0 <= t < t1:
                return None
        if tc.drop_prob > 0 and self.rng.random() < tc.drop_prob:
            return None
        if self.in_burst(t):
            return tc.can_burst * self.rng.uniform(0.9, 1.0)
        return self.rng.uniform(tc.can_min, tc.can_max)


class Channel:
    """One-direction CAN channel: send(t, seq, payload); poll(t) -> newest delivered."""

    def __init__(self, latency, enabled=True):
        self.lat, self.enabled = latency, enabled
        self.q = []
        self.latest = None     # (seq, t_sent, t_arrive, payload)

    def send(self, t, seq, payload):
        lat = self.lat.sample(t) if self.enabled else 0.0
        if lat is None:
            return
        heapq.heappush(self.q, (t + lat, seq, t, payload))

    def poll(self, t):
        while self.q and self.q[0][0] <= t + 1e-12:
            t_arr, seq, t_sent, payload = heapq.heappop(self.q)
            if self.latest is None or seq > self.latest[0]:
                self.latest = (seq, t_sent, t_arr, payload)
        return self.latest


class AsyncWorker:
    """Models an inference job that runs off the real-time thread.

    submit(t, fn, *args) starts a job if none is running; the result becomes
    visible at t + latency. The RT loop only ever calls poll(t) - it never waits.
    """

    def __init__(self, rng, latency_range):
        self.rng, self.lo_hi = rng, latency_range
        self.pending = None      # (t_ready, t_obs, result)
        self.result = None       # (t_obs, t_ready, value)

    def busy(self, t):
        return self.pending is not None and t < self.pending[0]

    def submit(self, t, fn, *args):
        if self.busy(t):
            return False
        self.poll(t)
        lat = self.rng.uniform(*self.lo_hi)
        self.pending = (t + lat, t, fn(*args))
        return True

    def poll(self, t):
        if self.pending is not None and t >= self.pending[0]:
            t_ready, t_obs, val = self.pending
            self.result = (t_obs, t_ready, val)
            self.pending = None
        return self.result
