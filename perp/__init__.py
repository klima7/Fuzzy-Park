from control import Controller
from common import WaitSomeTime, BackwardForSomeTime, ForwardForSomeTime
from .forward_to_find_left_space import ForwardToFindLeftSpace
from .turn_left_to_park import TurnLeftToPark
from .backward_before_turn import BackwardBeforeTurn


class PerpParkController(Controller):

    def __init__(self, tank):
        stages = [
            WaitSomeTime(0.1),
            ForwardToFindLeftSpace(),
            BackwardBeforeTurn(),
            TurnLeftToPark(),
            # ForwardForSomeTime(3)
        ]
        super().__init__(tank, stages)
