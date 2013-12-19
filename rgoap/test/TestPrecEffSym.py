'''
RGOAPTestPreconditionEffectSymmetry

Created on Jul 2, 2013
@author: felix
'''

from rgoap import Condition, Precondition, Effect, Action, Goal
from rgoap import MemoryCondition
from rgoap import Runner



class SymmetricAction(Action):

    def __init__(self):
        Action.__init__(self,
                        [Precondition(Condition.get('robot.bumpered'), True)],
                        [Effect(Condition.get('robot.bumpered'), False)])

    def run(self, next_worldstate):
        print '%s: resetting bumper..' % self.__class__



if __name__ == "__main__":

    runner = Runner()

    Condition.add(MemoryCondition(runner.memory, 'robot.bumpered', True))

    runner.actions.add(SymmetricAction())


    Condition.initialize_worldstate(runner.worldstate)
    print 'worldstate now is: ', runner.worldstate


    goal = Goal([Precondition(Condition.get('robot.bumpered'), False)])

    start_node = runner.update_and_plan(goal, introspection=True)

    print 'start_node: ', start_node


    if start_node is None:
        print 'No plan found! Check your ROS graph!'
    else:
        runner.execute(start_node)
