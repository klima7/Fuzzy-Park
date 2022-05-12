from tank import Tank


class Controller:

    def __init__(self, tank, stages):
        self._tank = tank
        self._stages = stages

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

    def started(self, tank, distances):
        pass

    def control(self, tank, distances):
        raise NotImplementedError('control method not implemented')


def run_simulation_with_controller(controller):
    tank = Tank()
    controller = controller(tank)

    finished = controller.control()
    while not finished:
        finished = controller.control()

    tank.plot_distances()
