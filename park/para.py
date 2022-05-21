from time import time

from matplotlib import pyplot as plt

from .model import FuzzyModel
from utils.control import Controller, Stage


class ParaParkController(Controller):

    def __init__(self, tank):
        stages = [
            Tmp1(),
            Tmp2(),
            ForwardToFindSpace(),
            FirstTurn(),
            SecondTurn(),
            Stop(),
        ]
        super().__init__(tank, stages)


class Tmp1(Stage):

    def control(self, tank, distances):
        tank.turn_left_circle(4)
        return time() - self.start > 3


class Tmp2(Stage):

    def control(self, tank, distances):
        tank.turn_right_circle(4)
        return time() - self.start > 3


class ForwardToFindSpace(Stage):

    _model = FuzzyModel(
        max_vel=10,
        break_vel=3,
        stop_dist=3.2,
        break_dist=4,
        sharpness=0.2,
        # plot_sets=True,
        # plot_history=True
    )

    def control(self, tank, distances):
        distance = 6 - (0 if distances.se2 == 6 else distances.se2)
        velocity = self._model.get_velocity(distance)
        tank.forward(velocity)
        stop = velocity == 0
        # if stop:
        #     self.plot_history()
        return stop


class FirstTurn(Stage):

    def control(self, tank, distances):
        tank.turn_left_circle(-5)
        return time() - self.start > 5


class SecondTurn(Stage):

    def control(self, tank, distances):
        tank.turn_right_circle(-5)
        return distances.se2 - distances.sw2 < 0.001


class Stop(Stage):

    def control(self, tank, distances):
        tank.stop()
        return True


