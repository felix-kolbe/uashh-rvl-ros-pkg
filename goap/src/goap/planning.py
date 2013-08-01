'''
Created on Jul 5, 2013

Regressive A* planner with Node, Planner and PlanExecutor

@author: felix
'''

from collections import deque

from goap import ActionBag, WorldState



class Node(object):

    def __init__(self, worldstate, action, parent_nodes_path_list, parent_actions_path_list):
        """
        worldstate: states at this node
        action: action that led (regressively) to this node (and that should be run when executing this path forwards)
        parent_nodes_path_list: nodes that led (from the goal) to this node
        parent_actions_path_list: actions that led (from the goal) to this node
        """
        self.worldstate = worldstate
        self.action = action
        self.parent_nodes_path_list = parent_nodes_path_list
        self.parent_actions_path_list = parent_actions_path_list

    def __repr__(self):
        return '<Node %X cost=%s action=%s worldstate=%s>' % \
            (id(self), self.cost(), self.action, self.worldstate)

    def cost(self):
        return len(self.parent_nodes_path_list) + \
                self.action.cost() if self.action is not None else 0

    # regressive planning
    def get_child_nodes_for_valid_actions(self, actionbag, start_worldstate):
        nodes = []
        for action in actionbag:
            nodes_path_list = self.parent_nodes_path_list[:]
            nodes_path_list.append(self)
            actions_path_list = self.parent_actions_path_list[:]
            actions_path_list.append(action)
            worldstatecopy = WorldState(self.worldstate)
            action.apply_preconditions(worldstatecopy, start_worldstate)
            node = Node(worldstatecopy, action, nodes_path_list, actions_path_list)
            nodes.append(node)
        return nodes



class Planner(object):

    def __init__(self, actionbag, worldstate, goal):
        self._actionbag = actionbag
        self._start_worldstate = worldstate
        self._goal = goal

    def plan(self):
        """Plan ...
        Return the node that matches the given start worldstate and 
        is the start node for a plan reaching the given goal.
        """
        print 'start_worldstate: ', self._start_worldstate

        print 'goal: ', self._goal

        goal_worldstate = WorldState()

        self._goal.apply_preconditions(goal_worldstate)
        print 'goal_worldstate: ', goal_worldstate

        goal_node = Node(goal_worldstate, None, [], [])
        print 'goal_node: ', goal_node

#         worldstate = self._start_worldstate

        child_nodes = deque([goal_node])

#         while not self._goal.is_valid(worldstate):
        count = 0
        while len(child_nodes) != 0:
            count += 1
            if count >= 20:
                break

            print "=Doing another planning loop="
            print 'nodes: ', child_nodes

            current_node = child_nodes.popleft()
            print 'popping this: ', current_node
#             print 'nodes: ', child_nodes, len(child_nodes)

            print 'current_node.worldstate: ', current_node.worldstate

            if self._start_worldstate.matches(current_node.worldstate):
                print "Found plan!"
                print 'plan nodes: ', current_node.parent_nodes_path_list
                print 'plan actions: ', current_node.parent_actions_path_list
                return current_node

            #current_node.check_and_add_actions(self._actionbag)
            print "Current node: ", current_node

            new_child_nodes = current_node.get_child_nodes_for_valid_actions(
                    self._actionbag.generate_matching_actions(self._start_worldstate, current_node.worldstate),
                    self._start_worldstate)
            print 'new child nodes: ', new_child_nodes

            # add new nodes and sort. this is stable, so old nodes stay
            # more left in the deque than new nodes with same weight
            child_nodes.extend(new_child_nodes)
            child_nodes = deque(sorted(child_nodes, key=Node.cost))

        print 'No plan found.'
        return None


class PlanExecutor(object):

    def __init__(self):
        pass

    def execute(self, start_node):
        assert len(start_node.parent_nodes_path_list) == len(start_node.parent_actions_path_list)

        print 'list lengths: ', len(start_node.parent_nodes_path_list), len(start_node.parent_actions_path_list)

        if len(start_node.parent_nodes_path_list) == 0:
            print 'Sole node left must be goal node, stopping executor'
            return

        current_worldstate = start_node.worldstate
        action = start_node.action # or: parent_actions_path_list[-1]
        next_node = start_node.parent_nodes_path_list[-1]
        next_worldstate = next_node.worldstate

        if action.is_valid(current_worldstate):
            if action.check_freeform_context():
                print 'PlanExecutor now executing: ', action
                action.run(next_worldstate)
#                print 'Memory is now: ', action._memory   # only for mem actions
                self.execute(next_node)
            else:
                print 'Action\'s freeform context isn\'t valid! Aborting executor'
                print 'Action: ', action
        else:
            print 'Action isn\'t valid to workspace! Aborting executor'
            print 'Action: ', action
            print 'worldstate: ', current_worldstate

