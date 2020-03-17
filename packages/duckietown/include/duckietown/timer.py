import time
import numpy as np
from collections import namedtuple, defaultdict
from contextlib import contextmanager

TimeRange = namedtuple('TimeRange', 'start end')
SinglePhaseStatistics = namedtuple('TimeRange', 'num_events frequency avg_duration')

class PhaseTimer:

    def __init__(self):
        self.phases = defaultdict(SinglePhaseTimer)
        self.time_phase = self._time_phase_notrecording
        self.is_recording = False

    def reset(self):
        self.__init__()

    def stop_recording(self):
        self.time_phase = self._time_phase_notrecording
        self.reset()
        self.is_recording = False

    def start_recording(self):
        self.time_phase = self._time_phase_recording
        self.is_recording = True

    @contextmanager
    def _time_phase_recording(self, phase_name):
        time_start = time.time()
        yield
        time_end = time.time()
        self.phases[phase_name].add_measurement(start=time_start, end=time_end)

    @contextmanager
    def _time_phase_notrecording(self, phase_name):
        yield

    def get_statistics(self):
        return {phase_name: phase_timer.get_statistics() for phase_name, phase_timer in self.phases.iteritems()}

class SinglePhaseTimer:

    # Constants
    MAX_KEEP_SEC = 3  #: Max time for keeping past measurements

    def __init__(self):

        # each event is a TimeRange tuple:
        self.events = list()

    def reset(self):
        self.events = list()

    def add_measurement(self, start, end):
        self.prune_old_observations()
        self.events.append(TimeRange(start, end))

    def get_statistics(self):
        self.prune_old_observations()

        num_events = len(self.events)

        # compute the frequency over all the events (within MAX_KEEP_SEC of now)
        if num_events > 1:
            first_time = self.events[0].start
            last_time = self.events[-1].start
            frequency = float(num_events-1) / (last_time-first_time)

        else:
            frequency = 0

        if num_events > 0:
            avg_duration = np.average([event.end - event.start for event in self.events])
        else:
            avg_duration = 0

        return SinglePhaseStatistics(num_events, frequency, avg_duration)

    def prune_old_observations(self):

        # only if the list is not empty:
        if self.events:
            last_to_remove = -1

            # iterate until the first event that is within MAX_KEEP_SEC of now,
            # as the list is ordered the rest are bound to be within MAX_KEEP_SEC of now
            for event_idx, event in enumerate(self.events):
                if event.end < time.time()-SinglePhaseTimer.MAX_KEEP_SEC:
                    last_to_remove = event_idx
                else:
                    break

            # remove the old events
            self.events = self.events[last_to_remove+1:]

