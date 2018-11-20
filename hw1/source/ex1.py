import search
import random
import math
from copy import deepcopy
from utils import hashabledict, vector_add
import sys

ids = ["111111111", "111111111"]

WALL, CELL = 99, 10
TARGET = [20, 25, 27]  # target, +box, +player
ICE = [30, 35, 37]  # ice, +box, +player
BOX = [15, 25, 35]  # box, +target, +ice
PLAYER = [17, 27, 37]  # player, +target, +ice

DIRECTIONS = {'R': (0, 1), 'D': (1, 0), 'L': (0, -1), 'U': (- 1, 0)}  # order matters


class State(object):

    def __init__(self, grid):
        self._grid = {(x, y): ele for x, row in enumerate(grid) for y, ele in enumerate(row)}
        self._player = next((cord for cord, ele in self._grid.items() if ele in PLAYER), None)
        self._box = [cord for cord, ele in self._grid.items() if ele in BOX]
        self._targets = [cord for cord, ele in self._grid.items() if ele in TARGET]
        self._box_left = len([box for box in self._box if box not in self._targets])

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

    # endregion

    # region hash
    def __eq__(self, other):
        return isinstance(other, State) and self._grid.__eq__(other.grid)

    def __lt__(self, other):
        return isinstance(other, State) and (self._box_left.__lt__(other._box_left))

    def __hash__(self):
        return hash(hashabledict(self._grid))

    # endregion

    def print(self, depth=0):
        temp = sys.stdout
        sys.stdout = open("temp.csv", 'a')
        print(" State {}:\n".format(depth))
        row = 0
        for cord, ele in self._grid.items():
            if row != cord[0]:
                print("\n")
                row = row + 1
            print(ele, ", ", end='')
        print("\n")
        sys.stdout.close()
        sys.stdout = temp

    def is_corner(self, pos):  # todo: improve to any kind of deadlock (boxes etc)

        steps = [(0, 1), (1, 0), (0, -1), (- 1, 0)]  # order is: R,D,L,U
        neighbor_cells = [vector_add(pos, step) for step in steps]
        is_last_cell_wall = self._grid[neighbor_cells[3]] == WALL
        for cell in neighbor_cells:
            is_curr_cell_wall = self._grid[cell] == WALL
            if is_last_cell_wall and is_curr_cell_wall:
                return True;
            is_last_cell_wall = is_curr_cell_wall
        return False


class SokobanProblem(search.Problem):
    """This class implements a sokoban problem"""

    def __init__(self, initial):
        """Don't forget to implement the goal test
        You should change the initial to your own representation"""
        self._mapSize = (len(initial), len(initial[0]))  # N x M
        search.Problem.__init__(self, State(initial))

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a tuple, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        # todo: remove unsolvable states (box to corner)
        p_cords = state.player
        grid = state.grid
        for direction, step in DIRECTIONS.items():
            aim_cords = vector_add(p_cords, step)
            # res_cords = p_cords if state.grid[aim_cords] == WALL else aim_cords
            if grid[aim_cords] == WALL:
                continue
            if grid[aim_cords] in BOX:  # if tries to move a box
                seq_cell = vector_add(aim_cords, step)
                if grid[seq_cell] in BOX + [WALL]:  # Ignore actions that has no effect
                    continue
                # else, box can be moved -> check if not pushing to corner (unless it's target):
                if seq_cell not in state.targets and state.is_corner(seq_cell):
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

            grid[box_cords] += 5
            rslt.box.append(box_cords)
        else:
            seq_cell = aim_cords
            while grid[seq_cell] == ICE[0]:
                aim_cords = seq_cell
                seq_cell = vector_add(aim_cords, step)
        # todo: Understand if box can init on ice(without blocking), and thus jam (or move with player?)
        rslt.player = aim_cords
        # update grid with player pos
        grid[rslt.player] += 7

        return rslt

    def goal_test(self, state):
        """ Given a state, checks if this is the goal state.
         Returns True if it is, False otherwise."""
        return all([(box in state.targets) for box in state.box])

    def h(self, node):
        """ This is the heuristic. It gets a node (not a state,
        state can be accessed via node.state)
        and returns a goal distance estimate"""
        return node.state.box_left

    """Feel free to add your own functions"""


def create_sokoban_problem(game):
    return SokobanProblem(game)
