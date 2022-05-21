from time import time

import numpy as np
import skfuzzy as fuzz
from matplotlib import pyplot as plt
from skfuzzy import control as ctrl
from skfuzzy.control import Antecedent, Consequent, Rule

from utils.control import Controller, Stage
from .model import FuzzyModel


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
        break_vel=3,
        stop_dist=1.55,
        break_dist=2.05,
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

    dist_min = Antecedent(np.linspace(0, 6, 100), 'dist_min')
    dist_wn = Antecedent(np.linspace(0, 6, 100), 'dist_wn')
    vel = Consequent(np.linspace(-1, 7, 100), 'vel')

    dist_min['l'] = fuzz.trapmf(dist_min.universe, [0, 0, 1.55, 160])
    dist_min['h'] = fuzz.trapmf(dist_min.universe, [1.55, 1.60, 6, 6])

    dist_wn['l'] = fuzz.trapmf(dist_wn.universe, [0, 0, 2, 2.05])
    dist_wn['h'] = fuzz.trapmf(dist_wn.universe, [2, 2.05, 6, 6])

    vel['z'] = fuzz.trimf(vel.universe, [-1, 0, 1])
    vel['l'] = fuzz.trimf(vel.universe, [0, 1, 2])
    vel['h'] = fuzz.trimf(vel.universe, [5, 6, 7])

    rules = [
        Rule(dist_min['h'] & dist_wn['l'], vel['h']),
        Rule(dist_min['h'] & dist_wn['h'], vel['l']),
        Rule(dist_min['l'], vel['z']),
    ]

    ctrl_system = ctrl.ControlSystem(rules)
    simulation = ctrl.ControlSystemSimulation(ctrl_system)

    tmp = []

    @classmethod
    def get_velocity(cls, distances):
        cls.simulation.input['dist_min'] = min(distances.ne, distances.nw, distances.wn)
        cls.simulation.input['dist_wn'] = distances.wn
        cls.tmp.append(min(distances.ne, distances.nw, distances.wn))
        cls.simulation.compute()
        velocity = cls.simulation.output['vel']
        if abs(velocity) < 0.5:
            return 0
        return velocity

    def control(self, tank, distances):
        velocity = self.get_velocity(distances)
        tank.turn_left(velocity)
        # if velocity == 0:
        #     plt.plot(self.tmp)
        #     plt.show()
        #     self.plot_history()
        return velocity == 0


class ForwardToFinish2(Stage):

    _model = FuzzyModel(
        max_vel=6,
        break_vel=6,
        stop_dist=3,
        break_dist=4,
        sharpness=0.3,
    )

    def control(self, tank, distances):
        print(f'nw: {distances.nw2:.2f} ne: {distances.ne2:.2f}')
        distance = min(distances.nw2, distances.ne2)
        velocity = self._model.get_velocity(6 - distance)
        tank.forward(velocity)
        return velocity == 0


class ForwardToFinish(Stage):

    dist_f = Antecedent(np.linspace(0, 6, 100), 'dist_f')
    dist_b = Antecedent(np.linspace(0, 7, 100), 'dist_b')
    vel = Consequent(np.linspace(-1, 7, 100), 'vel')

    dist_f['l'] = fuzz.trapmf(dist_f.universe, [0, 0, 2.9, 3])
    dist_f['h'] = fuzz.trapmf(dist_f.universe, [2.9, 3, 6, 6])

    dist_b['l'] = fuzz.trapmf(dist_b.universe, [0, 0, 1.9, 2.1])
    dist_b['m'] = fuzz.trapmf(dist_b.universe, [1.9, 2.1, 5.4, 5.5])
    dist_b['h'] = fuzz.trapmf(dist_b.universe, [5.4, 5.5, 7, 7])

    vel['z'] = fuzz.trimf(vel.universe, [-1, 0, 1])
    vel['l'] = fuzz.trimf(vel.universe, [2, 3, 4])
    vel['h'] = fuzz.trimf(vel.universe, [5, 6, 7])

    rules = [
        Rule(dist_f['l'] & dist_b['m'], vel['h']),
        Rule(dist_f['l'] & (dist_b['l'] | dist_b['h']), vel['l']),
        Rule(dist_f['h'], vel['z']),
    ]

    ctrl_system = ctrl.ControlSystem(rules)
    simulation = ctrl.ControlSystemSimulation(ctrl_system)

    tmp = []

    @classmethod
    def get_velocity(cls, distances):
        cls.simulation.input['dist_f'] = min(distances.nw2, distances.ne2)
        cls.simulation.input['dist_b'] = max(distances.ws2, distances.es2)
        cls.tmp.append(min(distances.ne, distances.nw, distances.wn))
        cls.simulation.compute()
        velocity = cls.simulation.output['vel']
        if abs(velocity) < 0.5:
            return 0
        return velocity

    def control(self, tank, distances):
        velocity = self.get_velocity(distances)
        tank.forward(velocity)
        # if velocity == 0:
        #     plt.plot(self.tmp)
        #     plt.show()
        #     self.plot_history()
        return velocity == 0
