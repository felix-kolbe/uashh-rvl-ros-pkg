'''
Created on Aug 7, 2013

@author: felix
'''
import rospy
import pickle

from smach_msgs.msg import SmachContainerStatus, SmachContainerStructure

class GOAPIntrospection(object):
    '''
    classdocs
    '''

    def __init__(self):
        self._publisher_structure = rospy.Publisher('GOAP/smach/container_structure', SmachContainerStructure, latch=True)
        self._publisher_status = rospy.Publisher('GOAP/smach/container_status', SmachContainerStatus, latch=True)
        self._pathprefix = '/GOAP_ROOT'


    def _add_nodes_recursively(self, node, structure):

        structure.children.append(self._nodeid(node))

        if len(node.parent_nodes_path_list) > 0:
            next_node = node.parent_nodes_path_list[-1]

            structure.internal_outcomes.append(str(node.action._effects))
            structure.outcomes_from.append(self._nodeid(node))
            structure.outcomes_to.append(self._nodeid(next_node))
#
#            structure.internal_outcomes.append('aborted')
#            structure.outcomes_from.append(self._nodeid(node))
#            structure.outcomes_to.append('None')

            self._add_nodes_recursively(next_node, structure)
#        else: # goal node
#            structure.internal_outcomes.append('succeeded')
#            structure.outcomes_from.append(self._nodeid(node))
#            structure.outcomes_to.append('None')


    def publish(self, start_node):
        structure = SmachContainerStructure()
        structure.header.stamp = rospy.Time.now()
        structure.path = self._pathprefix
##        structure.container_outcomes = ['succeeded', 'aborted']
#        structure.internal_outcomes.append('start')
#        structure.outcomes_from.append('None')
#        structure.outcomes_to.append(self._nodeid(start_node))
        self._add_nodes_recursively(start_node, structure)
        self._publisher_structure.publish(structure)

        status = SmachContainerStatus()
        status.header.stamp = rospy.Time.now()
        status.path = self._pathprefix
        status.initial_states = [self._nodeid(start_node)]
        status.active_states = ['None']
#        status.local_data = pickle.dumps(self._worldstate_to_userdata(start_node.worldstate)._data, 2)
        status.local_data = pickle.dumps(start_node.worldstate.get_state_name_dict(), 2)
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
#        status.local_data = pickle.dumps(self._worldstate_to_userdata(start_node.worldstate)._data, 2)
        status.local_data = pickle.dumps(node.worldstate.get_state_name_dict(), 2)
        status.info = 'node state'
        self._publisher_status.publish(status)


    def _nodeid(self, node):
        if node.action is None:
            node_action_name = 'GOAL'
        else:
            node_action_name = str(node.action) #.__class__.__name__
        return node_action_name + ' %X' % id(node)
