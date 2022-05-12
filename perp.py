from time import time
from control import Controller, Stage


class ForwardStage(Stage):

    def __init__(self):
        self.start = None
        super().__init__()

    def started(self, tank, distances):
        self.start = time()

    def control(self, tank, distances):
        tank.forward(5)
        return time() - self.start > 5


class LeftStage(Stage):

    def __init__(self):
        self.start = None
        super().__init__()

    def started(self, tank, distances):
        self.start = time()

    def control(self, tank, distances):
        tank.turn_left(5)
        return time() - self.start > 5


class PerpParkController(Controller):

    def __init__(self, tank):
        stages = [
            ForwardStage(),
            LeftStage(),
            ForwardStage(),
        ]
        super().__init__(tank, stages)

