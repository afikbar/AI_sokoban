import ex2
import time
import inputs

ALLOWED_ACTIONS = ("U", "D", "L", "R")
CODES = {"wall": 99, "nothing": 10, "crate": 15, "player": 17, "goal": 20, "goal+crate": 25, "goal+player": 27,
         "ice": 30, "ice+crate": 35, "ice+player": 37}
TRAVERSABLE = {CODES["nothing"], CODES["goal"], CODES["ice"]}
INVERSE_CODES = dict([(j, i) for i, j in CODES.items()])
ACTION_TIMEOUT = 5
CONSTRUCTOR_TIMEOUT = 60


class Checker:
    def __init__(self, true_map):
        self.initial_state = wrap_with_walls(true_map)
        self.current_state = state_to_dict(self.initial_state)
        self.turn_limit = (5 + len(true_map) + len(true_map[0])) ** 2
        print(f"Maximal amount of turns is {self.turn_limit}!")

    def true_state_to_controller_input(self):
        state_to_return = []
        for i in range(len(self.initial_state)):
            row = []
            for j in range(len(self.initial_state[0])):
                if "ice" in INVERSE_CODES[self.current_state[(i, j)]]:
                    row.append(self.current_state[(i, j)] - (CODES["ice"] - CODES["nothing"]))
                else:
                    row.append(self.current_state[(i, j)])
            state_to_return.append(row)
        state_to_return = [row[1:-1] for row in state_to_return[1:-1]]
        return state_to_return

    def change_state_after_action(self, action):
        current_tile = [(i, j) for i, j in self.current_state.keys() if self.current_state[(i, j)] % 10 == 7][0]
        next_tile = None
        third_tile = None
        if action == "U":
            next_tile = (current_tile[0] - 1 , current_tile[1])
            third_tile = (next_tile[0] - 1, next_tile[1])
        elif action == "D":
            next_tile = (current_tile[0] + 1, current_tile[1])
            third_tile = (next_tile[0] + 1, next_tile[1])
        elif action == "R":
            next_tile = (current_tile[0], current_tile[1] + 1)
            third_tile = (next_tile[0], next_tile[1] + 1)
        elif action == "L":
            next_tile = (current_tile[0], current_tile[1] - 1)
            third_tile = (next_tile[0], next_tile[1] - 1)

        assert next_tile and third_tile

        if self.current_state[next_tile] == 99:
            return

        if "crate" in INVERSE_CODES[self.current_state[next_tile]]:
            if self.current_state[third_tile] not in TRAVERSABLE:
                return
            self.current_state[current_tile] -= 7
            self.current_state[next_tile] += 2

            while True:
                next_tile = third_tile
                if action == "U":
                    third_tile = (next_tile[0] - 1, next_tile[1])
                elif action == "D":
                    third_tile = (next_tile[0] + 1, next_tile[1])
                elif action == "R":
                    third_tile = (next_tile[0], next_tile[1] + 1)
                elif action == "L":
                    third_tile = (next_tile[0], next_tile[1] + 1)
                if self.current_state[next_tile] != CODES["ice"] or self.current_state[third_tile] not in TRAVERSABLE:
                    self.current_state[next_tile] += 5
                    break
        else:
            while (self.current_state[next_tile] == CODES["ice"]) and self.current_state[third_tile] in TRAVERSABLE:
                next_tile = third_tile
                if action == "U":
                    third_tile = (next_tile[0] - 1, next_tile[1])
                elif action == "D":
                    third_tile = (next_tile[0] + 1, next_tile[1])
                elif action == "R":
                    third_tile = (next_tile[0], next_tile[1] + 1)
                elif action == "L":
                    third_tile = (next_tile[0], next_tile[1] + 1)
            self.current_state[current_tile] -= 7
            self.current_state[next_tile] += 7

    def check_controller(self):
        constructor_start = time.time()
        controller = ex2.SokobanController(self.true_state_to_controller_input())
        constructor_finish = time.time()
        if constructor_finish - constructor_start > CONSTRUCTOR_TIMEOUT:
            return f"Timeout on constructor! Took {constructor_finish - constructor_start} seconds, should take no more than {CONSTRUCTOR_TIMEOUT}"

        counter = 0
        while not self.at_goal():
            state_to_controller = self.true_state_to_controller_input()
            start = time.time()
            action = controller.get_next_action(state_to_controller)
            finish = time.time()
            if finish-start > ACTION_TIMEOUT:
                return f"Timeout on action! Took {finish - start} seconds, should take no more than {ACTION_TIMEOUT}"
            if not is_action_legal(action):
                return f"Action {action} is illegal!"
            counter += 1
            if counter > self.turn_limit:
                return "Turn limit exceeded!"
            self.change_state_after_action(action)
        return f"Goal achieved in {counter} steps!"

    def at_goal(self):
        map_objects = list(self.current_state.values())
        if (20 not in map_objects) and (27 not in map_objects):
            return True
        return False


# utility functions
# _______________________________________________


def state_to_dict(state):
    dict_to_return = {}
    for row_index, row in enumerate(state):
        for column_index, cell in enumerate(row):
            dict_to_return[(row_index, column_index)] = cell
    return dict_to_return


def is_action_legal(action):
    if action in ALLOWED_ACTIONS:
        return True
    return False


def wrap_with_walls(map):
    x_dimension = len(map[0]) + 2
    new_map = []
    new_map.append([99] * x_dimension)
    for row in map:
        new_map.append([99] + row + [99])
    new_map.append([99] * x_dimension)
    return new_map


if __name__ == '__main__':
    print(ex2.ids)
    for number, input in enumerate(inputs.inputs):
        my_checker = Checker(input)
        print(f"Output on input number {number + 1}: {my_checker.check_controller()}")