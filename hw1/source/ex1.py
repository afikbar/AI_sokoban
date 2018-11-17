import search
import random
import math
from utils import hashabledict, vector_add

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

    def __eq__(self, other):
        return isinstance(other, State) and self._grid.__eq__(other.grid)

    def __lt__(self, other):
        return isinstance(other, State) and (self._box_left.__lt__(other._box_left))

    def __hash__(self):
        return hash(hashabledict(self._grid))  # consider adding hash for ghosts cnt\dist?

ids = ["111111111", "111111111"]


class SokobanProblem(search.Problem):
    """This class implements a sokoban problem"""
    def __init__(self, initial):
        """Don't forget to implement the goal test
        You should change the initial to your own representation"""
        search.Problem.__init__(self, initial)
        
    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a tuple, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""

    def goal_test(self, state):
        """ Given a state, checks if this is the goal state.
         Returns True if it is, False otherwise."""

    def h(self, node):
        """ This is the heuristic. It gets a node (not a state,
        state can be accessed via node.state)
        and returns a goal distance estimate"""

    """Feel free to add your own functions"""


def create_sokoban_problem(game):
    return SokobanProblem(game)

