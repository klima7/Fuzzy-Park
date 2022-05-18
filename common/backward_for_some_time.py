from control import Stage
from time import time


class BackwardForSomeTime(Stage):

    def __init__(self, duration):
        super().__init__()
        self.duration = duration
        self.start_time = None

    def started(self, tank, distances):
        self.start_time = time()

    def control(self, tank, distances):
        tank.forward(-5)
        return time() - self.start_time > self.duration
