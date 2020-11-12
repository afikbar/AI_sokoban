import random
import math
from copy import deepcopy
from utils import hashabledict, vector_add, memoize, PriorityQueue, is_in

import sys

ids = ["1111111111", "111111111"]

WALL, CELL = 99, 10
TARGET = [20, 25, 27]  # target, +box, +player
ICE = [30, 35, 37]  # ice, +box, +player
BOX = [15, 25, 35]  # box, +target, +ice
PLAYER = [17, 27, 37]  # player, +target, +ice

DIRECTIONS = {'R': (0, 1), 'D': (1, 0), 'L': (0, -1), 'U': (- 1, 0)}  # order matters


def man_dist(start: tuple, end: tuple) -> int:
    return abs(start[0] - end[0]) + abs(start[1] - end[1])


class State(object):

    def __init__(self, grid):
        self._grid = {(x, y): ele for x, row in enumerate(grid) for y, ele in enumerate(row)}
        self._player = next((cord for cord, ele in self._grid.items() if ele in PLAYER), None)
        self._box = [cord for cord, ele in self._grid.items() if ele in BOX]
        self._targets = [cord for cord, ele in self._grid.items() if ele in TARGET]
        self._box_left = len([box for box in self._box if box not in self._targets])
        self._target_left = len([cord for cord in self._targets if cord not in self._box])

    # region Properties
    @property
    def grid(self):
        return self._grid

    @grid.setter
    def grid(self, dict):
        self._grid = dict

    @property
    def player(self):
        return self._player

    @player.setter
    def player(self, cord):
        self._player = cord

    @property
    def box(self):
        return self._box

    @box.setter
    def box(self, cord):
        self._box = cord

    @property
    def targets(self):
        return self._targets

    @targets.setter
    def targets(self, new_targets):
        self._targets = new_targets

    @property
    def box_left(self):
        return self._box_left

    @box_left.setter
    def box_left(self, cnt):
        self._box_left = cnt

    @property
    def target_left(self):
        return self._target_left

    @target_left.setter
    def target_left(self, cnt):
        self._target_left = cnt

    # endregion

    # region hash
    def __eq__(self, other):
        return isinstance(other, State) and self._grid.__eq__(other.grid)

    def __lt__(self, other):
        return isinstance(other, State) and (self._target_left.__lt__(other.target_left))

    def __hash__(self):
        return hash(hashabledict(self._grid))

    # endregion

    def print(self, depth=0):
        # temp = sys.stdout
        with open("temp.csv", 'a') as f:
            f.write("\n    {}:\n".format(depth))
            row = 0
            for cord, ele in self._grid.items():
                if row != cord[0]:
                    f.write("\n")
                    row = row + 1
                f.write("{}, ".format(ele))
            f.write("\n")

    def is_corner(self, pos):  # todo: improve to any kind of deadlock (boxes around etc)
        # todo: add inspection for borders that box will get stuck there
        steps = [(0, 1), (1, 0), (0, -1), (- 1, 0)]  # order is: R,D,L,U
        neighbor_cells = [vector_add(pos, step) for step in steps]
        is_last_cell_wall = self._grid[neighbor_cells[3]] == WALL
        for cell in neighbor_cells:
            is_curr_cell_wall = self._grid[cell] == WALL
            if is_last_cell_wall and is_curr_cell_wall:
                return True
            is_last_cell_wall = is_curr_cell_wall
        return False


def grid_wrap(grid):
    walled = (
            ((99,) * (len(grid[0]) + 2),) +
            tuple(((99,) + tuple(x for x in row) + (99,)) for row in grid) +
            ((99,) * (len(grid[0]) + 2),)

    )
    return walled


class Problem(object):
    """The abstract class for a formal problem.  You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal.  Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        raise NotImplementedError

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        if isinstance(self.goal, list):
            return is_in(state, self.goal)
        else:
            return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value.  Hill-climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError


class SokobanController(Problem):
    def __init__(self, input_map):
        # pass
        # TODO: fill in
        # Timeout: 60 seconds
        self._size = (len(input_map), len(input_map[0]))
        self._current_state = State(grid_wrap(input_map))
        self._expected_state = self._current_state
        self._last_action = None
        self._visited_states = {self._current_state}  # SET
        self._discovered_ice = set()  # SET
        self._ice_suspected = set(
            [cord for cord, ele in self._current_state.grid.items() if
             ele in [CELL] and not self._current_state.is_corner(cord)])
        self._exploring = len(self._ice_suspected) > 0
        self._explored_pos = set()
        self._solution = None
        self._visited_cnt = {cord: 0 for cord in self._current_state.grid.keys()}

    def get_next_action(self, observed_map):
        # TODO: fill in
        # pass
        # Should output one of the following: "U", "D", "L", "R"
        # Timeout: 5 seconds
        new_state = State(grid_wrap(observed_map))
        self._exploring = len(self._ice_suspected) > 0

        for ice_pos in self._discovered_ice:
            new_state.grid[ice_pos] += 20  # to ice
        # new_state.print("Exploring: {} - Last action: {}".format(self._exploring, self._last_action)) #debug
        if self._exploring:
            return self.explore_map(new_state)
        else:
            if self._solution is None:
                Problem.__init__(self, new_state)
                self._solution = best_first_graph_search(self, self.h).path()[1:]

            if new_state.player != self._expected_state.player:  # we encountered ice, save it.
                player_pos = self._expected_state.player
                while player_pos != new_state.player:
                    self._discovered_ice.add(player_pos)
                    new_state.grid[player_pos] += 20  # add ice
                    player_pos = vector_add(player_pos, DIRECTIONS[self._last_action])
                Problem.__init__(self, new_state)
                self._solution = best_first_graph_search(self, self.h).path()[1:]

        self._visited_states.add(new_state)
        self._expected_state = self._solution[0].state
        action = self._solution[0].action
        # print(action, end=', ')
        self._last_action = action
        self._solution = self._solution[1:]
        return action

    def explore_map(self, state):

        aim_cords = lambda action: vector_add(state.player, DIRECTIONS[action])
        player_pos = self._expected_state.player
        while player_pos != state.player:
            self._discovered_ice.add(player_pos)
            self._explored_pos.add(player_pos)
            self._visited_cnt[player_pos] += 1
            self._ice_suspected.discard(player_pos)
            state.grid[player_pos] += 20  # add ice
            player_pos = vector_add(player_pos, DIRECTIONS[self._last_action])

        self._explored_pos.add(state.player)
        self._ice_suspected.discard(player_pos)
        self._visited_cnt[player_pos] += 1

        possible_states = {action: self.result(state, action) for action in self.actions(state)}

        best_action = min(possible_states,
                          key=lambda action:
                          (possible_states[action].player in self._explored_pos) +
                          # to the best of my knowledge, this action wont take me to corner
                          state.is_corner(possible_states[action].player) +
                          (aim_cords(action) in state.targets) * 3 +
                          state.is_corner(aim_cords(action)) * 2 +
                          self._visited_cnt[possible_states[action].player])

        self._expected_state = possible_states[best_action]
        self._last_action = best_action
        # print(best_action, end=', ') #debug
        return best_action

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a tuple, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        p_cords = state.player
        grid = state.grid
        if state.target_left > state.box_left:  # if unsolvable (more targets than boxes)
            return None
        for direction, step in DIRECTIONS.items():
            aim_cords = vector_add(p_cords, step)
            # res_cords = p_cords if state.grid[aim_cords] == WALL else aim_cords
            if grid[aim_cords] == WALL:
                continue
            # if exploring a target/box dont yield
            if self._exploring and (
                    # aim_cords in state.targets or
                    grid[aim_cords] in BOX):
                # state.is_corner(aim_cords)):
                continue
            if grid[aim_cords] in BOX:  # if tries to move a box and not during exploring
                seq_cell = vector_add(aim_cords, step)
                if grid[seq_cell] in BOX + [WALL]:  # Ignore actions that has no effect
                    continue
                # else, box can be moved -> check if not pushing to corner (unless it's target):
                if seq_cell not in state.targets and (  # check if have box to spare
                        state.is_corner(seq_cell) and state.target_left == state.box_left):
                    continue

            yield direction

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        rslt = deepcopy(state)
        grid = rslt.grid
        step = DIRECTIONS[action]

        aim_cords = vector_add(rslt.player, step)
        grid[rslt.player] -= 7  # returns cell to its default value without player

        if grid[aim_cords] in BOX:  # from action no blocking box is allowed (a move that won't change anything)
            box_cords = aim_cords

            if box_cords in rslt.targets:  # if we moved a box from target pos
                rslt.box_left += 1
                rslt.target_left += 1

            grid[box_cords] -= 5  # returns cell to its value without box
            rslt.box.remove(box_cords)

            seq_cell = vector_add(box_cords, step)  # the sequential cell in same direction

            while grid[seq_cell] == ICE[0]:
                box_cords = seq_cell
                seq_cell = vector_add(box_cords, step)

            if grid[seq_cell] in [CELL, TARGET[0]]:
                box_cords = seq_cell

            if box_cords in rslt.targets:
                rslt.box_left -= 1
                rslt.target_left -= 1

            grid[box_cords] += 5
            rslt.box.append(box_cords)
        else:
            seq_cell = aim_cords
            while grid[seq_cell] == ICE[0]:
                aim_cords = seq_cell
                seq_cell = vector_add(aim_cords, step)
            if grid[seq_cell] in [CELL, TARGET[0]]:  # if player can go out of ice
                aim_cords = seq_cell
        # todo: Understand if box can init on ice(without blocking), and thus jam (or move with player?)
        rslt.player = aim_cords
        # update grid with player pos
        grid[rslt.player] += 7

        return rslt

    def goal_test(self, state):
        """ Given a state, checks if this is the goal state.
         Returns True if it is, False otherwise."""
        # return all([(box in state.targets) for box in state.box])
        return state.target_left == 0

    def h(self, node):
        """ This is the heuristic. It gets a node (not a state,
        state can be accessed via node.state)
        and returns a goal distance estimate"""
        state = node.state
        grid = state.grid

        if state.target_left == 0:
            return 0
        if state.box_left == 0:
            return sys.maxsize

        # sum of the distances of each target to its nearest box (using box once).
        sum_min_md_target_box = 0
        curr_boxes = state.box[:]
        for target in state.targets:
            target_md_box = [(box, man_dist(target, box)) for box in curr_boxes]
            closest_box = min(target_md_box, key=lambda box: box[1])
            curr_boxes.remove(closest_box[0])  # removes "used" box
            sum_min_md_target_box += closest_box[1]

        # todo: box is assigned to a goal so that the total sum of distances is minimized.

        # nearest box not in target to player:
        # box_not_in_target_md = [man_dist(state.player, box) for box in state.box if grid[box] not in TARGET]
        # player_box_md = min(box_not_in_target_md)
        # todo: agent wont put box in place because it will result in increase of min dist
        visited_factor = 2 if state in self._visited_states else 1

        # return state.box_left
        # return state.target_left
        # sum of man_dist from targets to nearest box with factor of more box than targets
        return sum_min_md_target_box * (
                state.target_left / state.box_left)  # + player_box_md + state.box_left * len(grid)


class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state.  Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node.  Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next = problem.result(self.state, action)
        return Node(next, self, action,
                    problem.path_cost(self.path_cost, self.state,
                                      action, next))

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)


def best_first_graph_search(problem, f):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = memoize(f, 'f')
    node = Node(problem.initial)
    if problem.goal_test(node.state):
        return node
    frontier = PriorityQueue(min, f)
    frontier.append(node)
    explored = set()
    while frontier:
        node = frontier.pop()
        # node.state.print("SEARCHING")  # printing
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
    return None
