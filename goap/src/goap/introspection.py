'''
Created on Aug 7, 2013

@author: felix
'''
import rospy
import pickle

from smach_msgs.msg import SmachContainerStatus, SmachContainerStructure
from smach_ros.introspection import STATUS_TOPIC, STRUCTURE_TOPIC


class Introspector(object):
    """Gives insight to a planner's plan and GOAP net by publishing prepared
    information to a smach_viewer.
    """
    def __init__(self):
        self._pathprefix = '/GOAP_PLAN'
        self._pathprefix_net = '/GOAP_NET'
        self._publisher_structure = rospy.Publisher(self._pathprefix + STRUCTURE_TOPIC, SmachContainerStructure, latch=True)
        self._publisher_status = rospy.Publisher(self._pathprefix + STATUS_TOPIC, SmachContainerStatus, latch=True)
        self._publisher_structure_net = rospy.Publisher(self._pathprefix_net + STRUCTURE_TOPIC, SmachContainerStructure, latch=True)
        self._publisher_status_net = rospy.Publisher(self._pathprefix_net + STATUS_TOPIC, SmachContainerStatus, latch=True)


    def publish_net(self, start_node, goal_node):
        """Publishes a GOAP planning net"""
        def _add_nodes_recursively(node, structure):
            """node: beginning with the goal node"""
            structure.children.append(self._nodeid(node))

            if len(node.possible_prev_nodes) > 0: # goal or inner node
                for prev_node in node.possible_prev_nodes:
                    structure.internal_outcomes.append(str(prev_node.action._effects))
                    structure.outcomes_from.append(self._nodeid(prev_node))
                    structure.outcomes_to.append(self._nodeid(node))
                    _add_nodes_recursively(prev_node, structure)
            else: # start or dead-end node
                pass

#            if len(node.parent_nodes_path_list) == 0: # goal node
#                structure.internal_outcomes.append('succeeded')
#                structure.outcomes_from.append(self._nodeid(node))
#                structure.outcomes_to.append('None')
#            else:
#                structure.internal_outcomes.append('aborted')
#                structure.outcomes_from.append(self._nodeid(node))
#                structure.outcomes_to.append('None')

        structure = SmachContainerStructure()
        structure.header.stamp = rospy.Time.now()
        structure.path = self._pathprefix_net
#        structure.container_outcomes = ['succeeded', 'aborted']
        _add_nodes_recursively(goal_node, structure)
        self._publisher_structure_net.publish(structure)
        print 'published net has ~%s nodes' % len(structure.children)

        status = SmachContainerStatus()
        status.header.stamp = rospy.Time.now()
        status.path = self._pathprefix_net
        status.initial_states = [self._nodeid(start_node) if start_node is not None else 'No plan found']
        #status.active_states = ['None']
        status.active_states = [self._nodeid(goal_node)]
        goal_states_dict = goal_node.worldstate.get_state_name_dict()
        goal_states_dict['WORLDSTATE'] = 'GOAL'
        status.local_data = pickle.dumps(goal_states_dict, 2)
        status.info = 'goal state'
        self._publisher_status_net.publish(status)

        rospy.sleep(5)

    def publish(self, start_node):
        """Publishes a planned GOAP plan"""
        def _add_nodes_recursively(node, structure):
            """node: beginning with the start node"""
            structure.children.append(self._nodeid(node))

            if len(node.parent_nodes_path_list) > 0:
                next_node = node.parent_nodes_path_list[-1]

                structure.internal_outcomes.append(str(node.action._effects))
                structure.outcomes_from.append(self._nodeid(node))
                structure.outcomes_to.append(self._nodeid(next_node))
#
#                structure.internal_outcomes.append('aborted')
#                structure.outcomes_from.append(self._nodeid(node))
#                structure.outcomes_to.append('None')

                _add_nodes_recursively(next_node, structure)
#            else: # goal node
#                structure.internal_outcomes.append('succeeded')
#                structure.outcomes_from.append(self._nodeid(node))
#                structure.outcomes_to.append('None')

        structure = SmachContainerStructure()
        structure.header.stamp = rospy.Time.now()
        structure.path = self._pathprefix
#        structure.container_outcomes = ['succeeded', 'aborted']
        _add_nodes_recursively(start_node, structure)
        self._publisher_structure.publish(structure)
        print 'published plan has ~%s nodes' % len(structure.children)

        status = SmachContainerStatus()
        status.header.stamp = rospy.Time.now()
        status.path = self._pathprefix
        status.initial_states = [self._nodeid(start_node)]
        status.active_states = ['None']
        start_states_dict = start_node.worldstate.get_state_name_dict()
        start_states_dict['WORLDSTATE'] = 'START'
        status.local_data = pickle.dumps(start_states_dict, 2)
        status.info = 'initial state'
        self._publisher_status.publish(status)

#        for node in start_node.parent_nodes_path_list:
#            print node
#            self.publish_update(node)

    def publish_update(self, node):
        nodeid = self._nodeid(node)
        status = SmachContainerStatus()
        status.header.stamp = rospy.Time.now()
        status.path = self._pathprefix
        status.initial_states = []
        status.active_states = [nodeid]
        start_states_dict = node.worldstate.get_state_name_dict()
        start_states_dict['WORLDSTATE'] = str(node)
        status.local_data = pickle.dumps(start_states_dict, 2)
        status.info = 'node state'
        self._publisher_status.publish(status)


    def _nodeid(self, node):
        if node.action is None:
            node_action_name = 'GOAL'
            node_action_cost = '0'
        else:
            node_action_name = str(node.action) #.__class__.__name__
            node_action_cost = node.action.cost()
        return node_action_name + ' %X' % id(node) + ' n' + str(node_action_cost) + ' t' + str(node.cost())
