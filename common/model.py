import matplotlib.pyplot as plt
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from skfuzzy.control import Antecedent, Consequent, Rule


class FuzzyModel:

    MAX_DIST = 6

    def __init__(self, max_vel, stop_dist, break_dist, break_vel, sharpness):
        self._max_vel = max_vel
        self._stop_dist = stop_dist
        self._break_dist = break_dist
        self._break_vel = break_vel
        self._sharpness = sharpness

        self._model = self._construct_model()

    def get_velocity(self, distance):
        if distance < self._stop_dist:
            return 0

        self._model.input['dist'] = distance
        self._model.compute()
        velocity = self._model.output['vel']

        if velocity < 0.2:
            return 0

        return velocity

    def _construct_model(self):
        l_span = self._break_vel
        h_span = self._max_vel - self._break_vel

        dist = Antecedent(np.linspace(0, self.MAX_DIST, 600), 'dist')
        vel = Consequent(np.linspace(-l_span, self._max_vel + h_span, 600), 'vel')

        s = self._sharpness
        dist['h'] = fuzz.trapmf(dist.universe, [self._break_dist - s, self._break_dist + s, self.MAX_DIST, self.MAX_DIST])
        dist['m'] = fuzz.trapmf(dist.universe, [self._stop_dist - s, self._stop_dist + s, self._break_dist - s, self._break_dist + s])
        dist['l'] = fuzz.trapmf(dist.universe, [0, 0, self._stop_dist - s, self._stop_dist + s])

        vel['l'] = fuzz.trimf(vel.universe, [-l_span, 0, l_span])
        vel['m'] = fuzz.trimf(vel.universe, [0, self._break_vel, self._max_vel])
        vel['h'] = fuzz.trimf(vel.universe, [self._max_vel - h_span, self._max_vel, self._max_vel + h_span])

        dist.view()
        vel.view()
        plt.show()

        rules = [
            Rule(dist['l'], vel['l']),
            Rule(dist['m'], vel['m']),
            Rule(dist['h'], vel['h']),
        ]

        ctrl_system = ctrl.ControlSystem(rules)
        model = ctrl.ControlSystemSimulation(ctrl_system)

        return model
