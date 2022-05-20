from time import time

from matplotlib import pyplot as plt

from .model import FuzzyModel
from utils.control import Controller, Stage


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


class PerpParkController(Controller):

    def __init__(self, tank):
        stages = [
            WaitSomeTime(0.1),
            ForwardToFindLeftSpace(),
            BackwardBeforeTurn(),
            TurnLeftToPark(),
            ForwardToFinish()
        ]
        super().__init__(tank, stages)


class ForwardToFindLeftSpace(Stage):

    _model = FuzzyModel(
        max_vel=10,
        break_vel=5,
        stop_dist=1.50,
        break_dist=2,
        sharpness=0.2,
    )

    def control(self, tank, distances):
        velocity = self._model.get_velocity(distances.nw)
        tank.forward(velocity)
        return velocity == 0


class BackwardBeforeTurn(Stage):

    _model = FuzzyModel(
        max_vel=4,
        break_vel=2,
        stop_dist=4.23,
        break_dist=4.6,
        sharpness=0.1,
    )

    def control(self, tank, distances):
        velocity = self._model.get_velocity(6 - distances.ne)
        tank.backward(velocity)
        return velocity == 0


class TurnLeftToPark(Stage):

    _model = FuzzyModel(
        max_vel=5,
        break_vel=2,
        stop_dist=1.55,
        break_dist=1.8,
        sharpness=0.1,
    )

    def control(self, tank, distances):
        distance = min(distances.ne, distances.nw, distances.wn)
        velocity = self._model.get_velocity(distance)
        tank.turn_left(velocity)
        return velocity == 0


class ForwardToFinish(Stage):

    _model = FuzzyModel(
        max_vel=10,
        break_vel=5,
        stop_dist=1.50,
        break_dist=2.2,
        sharpness=0.3
    )

    def control(self, tank, distances):
        velocity = self._model.get_velocity(distances.nw)
        tank.forward(4)
        stop = max(distances.wn, distances.en) > 3
        return stop
