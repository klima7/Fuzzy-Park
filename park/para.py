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
            LastAdjustment(),
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

    _model = FuzzyModel(
        max_vel=5,
        break_vel=3,
        stop_dist=6-3.8,
        break_dist=6-2.75,
        sharpness=0.2,
        # plot_sets=True,
        # plot_history=True
    )

    def control(self, tank, distances):
        velocity = -self._model.get_velocity(6 - distances.ws2)
        tank.turn_left_circle(velocity)
        # stop = distances.ws2 > 3.6
        # if stop:
        #     self.plot_history()
        return velocity == 0


class ParkSecondTurn(Stage):

    _model = FuzzyModel(
        max_vel=5,
        break_vel=1,
        stop_dist=1,
        break_dist=1.4,
        sharpness=0.2,
        # plot_sets=True,
        # plot_history=True
    )

    def control(self, tank, distances):
        distance = min(distances.se2, distances.sw2)
        velocity = -self._model.get_velocity(distance)
        tank.turn_right_circle(velocity)
        return velocity == 0


class LastAdjustment(Stage):

    diff = Antecedent(np.linspace(-2, 2, 200), 'diff')
    vel = Consequent(np.linspace(-4, 4, 200), 'vel')

    best = -0.5
    slope = 0.4
    speed = 1.5

    diff['l'] = fuzz.trapmf(diff.universe, [-2, -2, best - slope, best])
    diff['m'] = fuzz.trimf(diff.universe, [best - slope, best, best + slope])
    diff['h'] = fuzz.trapmf(diff.universe, [best, best + slope, 2, 2])

    vel['zero'] = fuzz.trimf(vel.universe, [-1, 0, 1])
    vel['forward'] = fuzz.trimf(vel.universe, [speed-1, speed, speed+1])
    vel['backward'] = fuzz.trimf(vel.universe, [-speed-1, -speed, -speed+1])

    rules = [
        Rule(diff['h'], vel['forward']),
        Rule(diff['l'], vel['backward']),
        Rule(diff['m'], vel['zero']),
    ]

    ctrl_system = ctrl.ControlSystem(rules)
    simulation = ctrl.ControlSystemSimulation(ctrl_system)

    @classmethod
    def get_velocity(cls, distances):
        front = min(distances.nw2, distances.ne2)
        back = min(distances.sw2, distances.se2)
        diff = front - back
        cls.simulation.input['diff'] = diff
        cls.simulation.compute()
        velocity = cls.simulation.output['vel']
        if abs(velocity) < 0.1:
            return 0
        return velocity

    def control(self, tank, distances):
        velocity = self.get_velocity(distances)
        tank.forward(velocity)
        return velocity == 0
