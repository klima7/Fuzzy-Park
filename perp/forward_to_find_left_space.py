from common.model import FuzzyModel
from control import Stage


class ForwardToFindLeftSpace(Stage):

    _model = FuzzyModel(
        max_vel=10,
        break_vel=6,
        stop_dist=1.5,
        break_dist=2.2,
        sharpness=0.2
    )

    def control(self, tank, distances):
        velocity = self._model.get_velocity(distances.nw)
        tank.forward(velocity)
        return velocity == 0
