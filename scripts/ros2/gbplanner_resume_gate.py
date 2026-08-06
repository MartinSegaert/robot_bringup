"""State machine for safely accepting GBPlanner commands after a mode switch."""


class GbplannerResumeGate:
    """Wait for a post-transition path and a short controller settling time."""

    def __init__(self, settle_time_s):
        if settle_time_s < 0.0:
            raise ValueError('settle_time_s must be non-negative')
        self._settle_time_ns = int(settle_time_s * 1e9)
        self.cancel()

    @property
    def waiting(self):
        return self._transition_ns is not None

    def start(self, transition_ns):
        self._transition_ns = int(transition_ns)
        self._path_received_ns = None

    def cancel(self):
        self._transition_ns = None
        self._path_received_ns = None

    def observe_path(self, path_stamp_ns, received_ns):
        """Latch the first path generated strictly after the mode transition."""
        if not self.waiting or self._path_received_ns is not None:
            return False
        if int(path_stamp_ns) <= self._transition_ns:
            return False
        self._path_received_ns = int(received_ns)
        return True

    def command_allowed(self, now_ns):
        if not self.waiting:
            return True
        if self._path_received_ns is None:
            return False
        if int(now_ns) - self._path_received_ns < self._settle_time_ns:
            return False
        self.cancel()
        return True
