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
        break_vel=3,
        stop_dist=4.23,
        break_dist=4.6,
        sharpness=0.1
    )

    def control(self, tank, distances):
        velocity = self._model.get_velocity(6 - distances.ne)
        tank.backward(velocity)
        return velocity == 0


class TurnLeftToPark(Stage):

    _model = FuzzyModel(
        max_vel=7,
        break_vel=2,
        stop_dist=1.6,
        break_dist=2.5,
        sharpness=0.4
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

    def started(self, tank, distances):
        tank.restart_plot()
        self.start = time()
        self.tmp1 = []
        self.tmp2 = []
        self.tmp3 = []
        self.tmp4 = []

    def control(self, tank, distances):
        velocity = self._model.get_velocity(distances.nw)
        tank.forward(4)
        self.tmp1.append(max(distances.nw, distances.ne))
        self.tmp2.append(max(distances.wn, distances.en))
        self.tmp3.append(max(distances.sw, distances.se))
        self.tmp4.append(max(distances.es, distances.ws))
        stop = max(distances.wn, distances.en) > 3
        # if stop:
        #     plt.plot(self.tmp1, label='1')
        #     plt.plot(self.tmp2, label='2')
        #     plt.plot(self.tmp3, label='3')
        #     plt.plot(self.tmp4, label='4')
        #     plt.legend()
        #     plt.show()
        return stop
