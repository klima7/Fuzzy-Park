from time import time
from control import Stage


class WaitSomeTime(Stage):

    def __init__(self, duration):
        super().__init__()
        self.duration = duration
        self.start_time = None

    def started(self, tank, distances):
        self.start_time = time()

    def control(self, tank, distances):
        tank.stop()
        return time() - self.start_time > self.duration
