from time import time

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from skfuzzy.control import Antecedent, Consequent, Rule

from utils.control import Controller, Stage
from .model import FuzzyModel


class ParaParkController(Controller):

    def __init__(self, tank):
        stages = [
            ForwardToFindSpace(),
            DriveCloserFirstTurn(),
            DriveCloserSecondTurn(),
            ParkFirstTurn(),
            ParkSecondTurn(),
            # Stop(),
        ]
        super().__init__(tank, stages)


class ForwardToFindSpace(Stage):

    _model = FuzzyModel(
        max_vel=10,
        break_vel=3,
        stop_dist=3.9,
        break_dist=4.4,
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


class DriveCloserFirstTurn(Stage):

    _model = FuzzyModel(
        max_vel=5,
        break_vel=1,
        stop_dist=0.7,
        break_dist=1,
        sharpness=0.1,
        # plot_sets=True,
        # plot_history=True
    )

    def control(self, tank, distances):
        distance = distances.ne2
        velocity = self._model.get_velocity(distance)
        tank.turn_left_circle(velocity)
        stop = velocity == 0
        return stop


class DriveCloserSecondTurn(Stage):

    diff = Antecedent(np.linspace(0, 2, 10000), 'diff')
    vel = Consequent(np.linspace(-1, 6, 200), 'vel')

    adj = 0.1

    diff['l'] = fuzz.trapmf(diff.universe, [0, 0, 0.001, 0.01])
    diff['m'] = fuzz.trapmf(diff.universe, [0.001, 0.01, 0.09+adj, 0.1+adj])
    diff['h'] = fuzz.trapmf(diff.universe, [0.09+adj, 0.1+adj, 2, 2])

    vel['z'] = fuzz.trimf(vel.universe, [-1, 0, 1])
    vel['m'] = fuzz.trimf(vel.universe, [1, 2, 3])
    vel['h'] = fuzz.trimf(vel.universe, [4, 5, 6])

    rules = [
        Rule(diff['l'], vel['z']),
        Rule(diff['m'], vel['m']),
        Rule(diff['h'], vel['h']),
    ]

    ctrl_system = ctrl.ControlSystem(rules)
    simulation = ctrl.ControlSystemSimulation(ctrl_system)

    @classmethod
    def get_velocity(cls, distances):
        diff = abs(distances.es2 - distances.en2)
        cls.simulation.input['diff'] = diff
        cls.simulation.compute()
        velocity = cls.simulation.output['vel']
        if abs(velocity) < 0.1:
            return 0
        return velocity

    def control(self, tank, distances):
        velocity = self.get_velocity(distances)
        tank.turn_right_circle(velocity)
        stop = velocity == 0
        # if stop:
        #     self.plot_history()
        return stop


class ParkFirstTurn(Stage):

    def control(self, tank, distances):
        tank.turn_left_circle(-5)
        return time() - self.start > 5


class ParkSecondTurn(Stage):

    def control(self, tank, distances):
        tank.turn_right_circle(-5)
        return time() - self.start > 5
        # Może dobry pomysł
        # return distances.se2 - distances.sw2 < 0.001


class Stop(Stage):

    def control(self, tank, distances):
        tank.stop()
        return True


