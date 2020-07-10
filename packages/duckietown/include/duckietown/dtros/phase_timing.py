import time
import inspect
from collections import namedtuple, defaultdict
from contextlib import contextmanager

from .constants import PHASE_TIMER_MAX_KEEP_SEC

TimeRange = namedtuple('TimeRange', 'start end')
SinglePhaseStatistics = namedtuple('TimeRange', 'num_events frequency avg_duration filename lines line_nums')

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

        # get and save the lines and the source code in this timing segment if not done already
        if not self.phases[phase_name].context_set:
            try:
                start_frame = inspect.currentframe()
                start_called_from_frame_line = inspect.getouterframes(start_frame)[2][2]
                called_from_file = inspect.getfile(inspect.getouterframes(start_frame)[2][0])
            except Exception as e:
                print("Error in extracting the source code of the timing context (start): %s" % str(e))

        # actual timing code (yield hands back to the code in the context)
        time_start = time.time()
        yield
        time_end = time.time()
        self.phases[phase_name].add_measurement(start=time_start, end=time_end)

        # get and save the lines and the source code in this timing segment if not done already
        if not self.phases[phase_name].context_set:
            try:
                end_frame = inspect.currentframe()
                end_called_from_frame_line = inspect.getouterframes(end_frame)[2][2]
                code_lines, start_idx = inspect.getsourcelines(inspect.getouterframes(end_frame)[2][0])
                code_lines = code_lines[start_called_from_frame_line-start_idx:end_called_from_frame_line-start_idx+1]
                self.phases[phase_name].add_context(filename=called_from_file,
                                                    lines=code_lines,
                                                    line_nums=(start_called_from_frame_line,
                                                               end_called_from_frame_line))
            except Exception as e:
                print("Error in extracting the source code of the timing context (end): %s" % str(e))

    @contextmanager
    def _time_phase_notrecording(self, phase_name):
        yield

    def get_statistics(self):
        return {phase_name: phase_timer.get_statistics()
                for phase_name, phase_timer
                in self.phases.iteritems()
                if phase_timer.context_set}

class SinglePhaseTimer:

    # Constants
    def __init__(self):

        # each event is a TimeRange tuple:
        self.events = list()

        # store the information about where the timing context was called from
        self.context_filename = ""
        self.context_lines = []
        self.context_line_nums = (None, None)
        self.context_set = False

    def reset(self):
        self.events = list()

    def add_measurement(self, start, end):
        self.prune_old_observations()
        self.events.append(TimeRange(start, end))

    def add_context(self, filename="", lines=[], line_nums=(None,None)):
        self.context_filename = filename
        self.context_lines = lines
        self.context_line_nums = line_nums
        self.context_set = True

    def get_statistics(self):
        self.prune_old_observations()

        num_events = len(self.events)

        # compute the frequency over all the events (within PHASE_TIMER_MAX_KEEP_SEC of now)
        if num_events > 1:
            first_time = self.events[0].start
            last_time = self.events[-1].start
            frequency = float(num_events-1) / (last_time-first_time)

        else:
            frequency = 0

        if num_events > 0:
            durations = [event.end - event.start for event in self.events]
            avg_duration = float(np.sum(durations)) / len(durations)
        else:
            avg_duration = 0

        return SinglePhaseStatistics(num_events, frequency, avg_duration,
                                     self.context_filename, self.context_lines, self.context_line_nums)

    def prune_old_observations(self):

        # only if the list is not empty:
        if self.events:
            last_to_remove = -1

            # iterate until the first event that is within PHASE_TIMER_MAX_KEEP_SEC of now,
            # as the list is ordered the rest are bound to be within PHASE_TIMER_MAX_KEEP_SEC of now
            for event_idx, event in enumerate(self.events):
                if event.end < time.time()-PHASE_TIMER_MAX_KEEP_SEC:
                    last_to_remove = event_idx
                else:
                    break

            # remove the old events
            self.events = self.events[last_to_remove+1:]

