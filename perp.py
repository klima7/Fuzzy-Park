# from common import WaitSomeTime
# from control import Controller
#
#
# class PerpParkController(Controller):
#
#     def __init__(self, tank):
#         stages = [
#             WaitSomeTime(0.1),
#             ForwardToFindLeftSpace(),
#             BackwardBeforeTurn(),
#             TurnLeftToPark(),
#             # ForwardForSomeTime(3)
#         ]
#         super().__init__(tank, stages)
