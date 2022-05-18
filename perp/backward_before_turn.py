from time import time

import matplotlib.pyplot as plt
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from skfuzzy.control import Antecedent, Consequent, Rule

from control import Stage
from tank import Distances


class BackwardBeforeTurn(Stage):

    STOP_DISTANCE = 1.79
    STEEPENESS = 0.05

    dist_nw = Antecedent(np.linspace(1, 4, 100), 'dist_nw')
    vel = Consequent(np.linspace(-6, 2, 100), 'vel')

    dist_nw['l'] = fuzz.trapmf(dist_nw.universe, [1, 1, 1.5+STEEPENESS, STOP_DISTANCE-STEEPENESS])
    dist_nw['h'] = fuzz.trapmf(dist_nw.universe, [1.5+STEEPENESS, STOP_DISTANCE-STEEPENESS, 4, 4])

    vel['l'] = fuzz.trimf(vel.universe, [-1, 0, 1])
    vel['h'] = fuzz.trimf(vel.universe, [-6, -5, -4])

    rules = [
        Rule(dist_nw['l'], vel['h']),
        Rule(dist_nw['h'], vel['l']),
    ]

    ctrl_system = ctrl.ControlSystem(rules)
    simulation = ctrl.ControlSystemSimulation(ctrl_system)

    def __init__(self):
        super().__init__()
        self.start = None

    @classmethod
    def get_velocity(cls, distances: Distances):
        cls.simulation.input['dist_nw'] = distances.nw
        cls.simulation.compute()
        velocity = cls.simulation.output['vel']
        if abs(velocity) < 0.3:
            return 0
        return velocity

    def started(self, tank, distances):
        self.start = time()

    def control(self, tank, distances):
        velocity = self.get_velocity(distances)
        tank.forward(velocity)
        return velocity == 0
