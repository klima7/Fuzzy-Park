from time import time

import matplotlib.pyplot as plt
import numpy as np

from utils.tank import Tank


class Controller:

    def __init__(self, tank, stages):
        self._tank = tank
        self._stages = [WaitSomeTime(0.1)] + stages

    def control(self):
        # immediate return if finished
        if not self._stages:
            return True

        distances = self._tank.read_distances()
        current_stage = self._stages[0]

        # start stage
        if not current_stage.was_started:
            current_stage.started(self._tank, distances)
            current_stage.was_started = True

        # perform current stage
        stage_finished = current_stage.control(self._tank, distances)
        current_stage.dist_history.append(distances)

        # remove stage if finished
        if stage_finished:
            self._stages.pop(0)

        # finally, stop tank
        if not self._stages:
            self._tank.stop()

        return len(self._stages) == 0


class Stage:

    def __init__(self):
        self.was_started = False
        self.dist_history = []
        self.start = None

    def started(self, tank, distances):
        self.start = time()

    def control(self, tank, distances):
        raise NotImplementedError('control method not implemented')
    
    def plot_history(self):
        fig, axs = plt.subplots(4, 2)
        x = np.arange(len(self.dist_history)) / 10

        nw = [d.nw2 for d in self.dist_history]
        ne = [d.ne2 for d in self.dist_history]
        wn = [d.wn2 for d in self.dist_history]
        en = [d.en2 for d in self.dist_history]
        sw = [d.sw2 for d in self.dist_history]
        se = [d.se2 for d in self.dist_history]
        ws = [d.ws2 for d in self.dist_history]
        es = [d.es2 for d in self.dist_history]

        axs[0, 0].plot(x, nw, '-', label='nw')
        axs[0, 1].plot(x, ne, '-', label='ne')
        axs[1, 0].plot(x, wn, '-', label='wn')
        axs[1, 1].plot(x, en, '-', label='en')
        axs[2, 0].plot(x, sw, '--', label='sw')
        axs[2, 1].plot(x, se, '--', label='se')
        axs[3, 0].plot(x, ws, '--', label='ws')
        axs[3, 1].plot(x, es, '--', label='es')

        for i in range(4):
            for j in range(2):
                axs[i][j].legend()
                axs[i][j].grid()

        plt.show()


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


def run_with_controller(controller):
    tank = Tank()
    controller = controller(tank)

    finished = controller.control()
    while not finished:
        finished = controller.control()

