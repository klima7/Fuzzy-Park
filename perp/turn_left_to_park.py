from time import time

import matplotlib.pyplot as plt
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from skfuzzy.control import Antecedent, Consequent, Rule

from control import Stage
from tank import Distances


class TurnLeftToPark(Stage):

    def __init__(self):
        super().__init__()
        self.start = None
        self.tmp = []

    @classmethod
    def get_velocity(cls, distances: Distances):
        minimum_dist = min(distances.ne, distances.nw, distances.wn)

        velocity = 2
        if velocity < 0.2:
            return 0
        return velocity

    def started(self, tank, distances):
        self.start = time()

    def control(self, tank, distances):
        velocity = self.get_velocity(distances)
        print(f'{distances.es:.3f} {velocity:.3f}')
        tank.turn_left(2)
        # return velocity == 0
        minimum = min(distances.ne, distances.nw, distances.wn)
        self.tmp.append(minimum)
        stop = minimum < 1.60
        if stop:
            plt.plot(self.tmp)
        return stop
