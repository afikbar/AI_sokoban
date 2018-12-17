import ex1
import search
import time


def timeout_exec(func, args=(), kwargs={}, timeout_duration=10, default=None):
    """This function will spawn a thread and run the given function
    using the args, kwargs and return the given default value if the
    timeout_duration is exceeded.
    """
    import threading

    class InterruptableThread(threading.Thread):

        def __init__(self):
            threading.Thread.__init__(self)
            self.result = default

        def run(self):
            # remove try if you want program to abort at error
            # try:
            self.result = func(*args, **kwargs)
            # except Exception as e:
            #     self.result = (-3, -3, e)

    it = InterruptableThread()
    it.start()
    it.join(timeout_duration)
    if it.isAlive():
        return default
    else:
        return it.result


def check_problem(p, search_method, timeout):
    """ Constructs a problem using ex1.create_poisonserver_problem,
    and solves it using the given search_method with the given timeout.
    Returns a tuple of (solution length, solution time, solution)"""

    """ (-2, -2, None) means there was a timeout
    (-3, -3, ERR) means there was some error ERR during search """

    t1 = time.time()
    s = timeout_exec(search_method, args=[p], timeout_duration=timeout)
    t2 = time.time()

    if isinstance(s, search.Node):
        solve = s
        # for node in s.path():
        #     node.state.print(node.depth)
        solution = list(map(lambda n: n.action, solve.path()))[1:]
        return (len(solution), t2 - t1, solution)
    elif s is None:
        return (-2, t2 - t1, None)
    else:
        return s


def solve_problems(problems):
    solved = 0
    for num, problem in enumerate(problems):
        try:
            p = ex1.create_sokoban_problem(problem)
        except Exception as e:
            print("Error creating problem: ", e)
            return None
        timeout = 60 * 2
        print("Input #", num + 1, "out of", len(problems))
        result = check_problem(p, (lambda p: search.best_first_graph_search(p, p.h)), timeout)
        print("GBFS ".rjust(6), result)
        if result[2] != None:
            if result[0] != -3:
                solved = solved + 1
        # result = check_problem(p, search.astar_search, timeout)
        # print("A*   ", result)
        # result = check_problem(p, search.breadth_first_search, timeout)
        # print("BFS ", result)
    print("GBFS Solved ", solved)


def main():
    print(ex1.ids)
    """Here goes the input you want to check"""

    problems = [
        (
            (99, 99, 20, 99, 99),
            (99, 99, 15, 99, 99),
            (20, 15, 17, 15, 20),
            (99, 99, 15, 99, 99),
            (99, 99, 20, 99, 99),

        ),

        (
            (99, 20, 30, 10, 10),
            (17, 15, 30, 10, 10),
            (99, 99, 10, 10, 99),
        ),

        (
            (20, 20, 20, 20, 99, 99, 99),
            (10, 10, 15, 99, 99, 10, 10),
            (10, 15, 15, 10, 10, 10, 10),
            (17, 15, 10, 99, 10, 10, 10),
            (99, 10, 10, 99, 99, 99, 99),
        ),
        # Original inputs end

        # https://www.sokobanonline.com/play/web-archive/alberto-garcia/1-3/3472_1-3-1

        (
            (20, 10, 10, 10, 20),
            (10, 15, 15, 99, 10),
            (10, 15, 10, 99, 17),
            (99, 15, 10, 99, 10),
            (20, 10, 10, 20, 10)

        ),

        # https://www.sokobanonline.com/play/web-archive/alberto-garcia/1-3/3482_1-3-11
        (
            (10, 10, 10, 10, 10, 99),
            (10, 10, 10, 99, 10, 99),
            (99, 15, 99, 99, 10, 99),
            (10, 10, 10, 99, 10, 99),
            (10, 15, 10, 99, 10, 10),
            (10, 99, 10, 20, 10, 17),
            (10, 10, 10, 20, 10, 99),
            (10, 99, 10, 99, 10, 99),
            (99, 10, 10, 10, 10, 99)

        ),

        # https://www.sokobanonline.com/play/web-archive/alberto-garcia/1-3/3501_1-3-30
        (
            (10, 10, 99, 99, 99),
            (10, 10, 99, 10, 99),
            (10, 20, 25, 20, 99),
            (99, 15, 99, 99, 99),
            (10, 15, 10, 10, 99),
            (10, 10, 10, 10, 17)
        ),

        # https://www.sokobanonline.com/play/web-archive/alberto-garcia/1-3/3514_1-3-43

        (
            (99, 99, 10, 10, 10),
            (99, 10, 20, 15, 10),
            (10, 15, 99, 99, 10),
            (10, 10, 10, 20, 10),
            (10, 15, 99, 99, 10),
            (10, 10, 20, 10, 17)

        ),

        # https://www.sokobanonline.com/play/web-archive/alberto-garcia/1-3/3485_1-3-14
        (
            (99, 99, 99, 10, 10, 10, 99, 99, 99),
            (99, 99, 10, 10, 20, 10, 99, 99, 99),
            (99, 10, 10, 20, 10, 99, 99, 10, 10),
            (10, 10, 20, 10, 99, 99, 10, 15, 10),
            (10, 20, 10, 99, 99, 10, 15, 10, 10),
            (10, 10, 99, 99, 10, 15, 10, 10, 99),
            (17, 10, 10, 10, 15, 10, 10, 99, 99),
            (99, 10, 10, 10, 10, 10, 99, 99, 99)
        ),

        # https://www.sokobanonline.com/play/lessons/6707_lesson-4-2
        (
            (99, 30, 30, 20),
            (99, 30, 37, 30),
            (30, 35, 30, 99),
            (30, 30, 30, 99)
        ),

        # https://www.sokobanonline.com/play/lessons/6708_lesson-4-3
        (
            (99, 10, 10, 15, 10),
            (10, 30, 10, 20, 10),
            (10, 30, 10, 99, 99),
            (99, 15, 10, 99, 99),
            (99, 17, 10, 99, 99)
        )

    ]
    my_problems = [
        (
            (99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99),
            (99, 10, 17, 10, 30, 30, 30, 30, 30, 30, 30, 30, 99, 20, 99),
            (99, 10, 10, 10, 10, 99, 99, 99, 10, 10, 10, 30, 30, 10, 99),
            (99, 10, 10, 10, 10, 99, 10, 10, 10, 10, 10, 15, 10, 30, 99),
            (99, 10, 10, 10, 10, 99, 99, 99, 10, 10, 10, 10, 10, 10, 99),
            (99, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 99),
            (99, 10, 10, 99, 10, 99, 99, 15, 10, 99, 99, 99, 10, 10, 99),
            (99, 10, 10, 99, 10, 10, 10, 10, 10, 10, 99, 99, 10, 10, 99),
            (99, 99, 15, 99, 10, 99, 99, 99, 10, 10, 99, 99, 10, 10, 99),
            (99, 10, 10, 99, 10, 99, 20, 10, 10, 10, 99, 10, 10, 10, 99),
            (99, 10, 10, 99, 10, 99, 10, 99, 10, 10, 99, 10, 10, 10, 99),
            (99, 10, 30, 99, 99, 99, 99, 99, 99, 99, 99, 99, 10, 10, 99),
            (99, 10, 10, 99, 10, 15, 30, 30, 30, 30, 20, 99, 10, 10, 99),
            (99, 10, 20, 99, 10, 10, 10, 99, 10, 10, 10, 10, 10, 10, 99),
            (99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99),
        ),
        (
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 99, 10, 20, 15, 17, 99, 99),
            (99, 99, 10, 25, 15, 15, 10, 99),
            (99, 10, 20, 10, 10, 10, 10, 99),
            (99, 10, 99, 10, 99, 10, 10, 99),
            (99, 10, 10, 20, 99, 99, 99, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
        ),
        (
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 99, 99, 99, 10, 17, 99, 99),
            (99, 99, 99, 99, 10, 10, 10, 99),
            (99, 20, 10, 99, 15, 15, 10, 99),
            (99, 10, 10, 10, 10, 10, 99, 99),
            (99, 20, 10, 10, 15, 99, 99, 99),
            (99, 99, 20, 10, 10, 99, 99, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
        ),
        (
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 99, 99, 10, 10, 20, 99, 99),
            (99, 10, 15, 10, 99, 10, 99, 99),
            (99, 10, 25, 15, 10, 10, 99, 99),
            (99, 10, 20, 99, 17, 10, 99, 99),
            (99, 10, 10, 10, 10, 99, 99, 99),
            (99, 10, 10, 10, 99, 99, 99, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
        ),
        (
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 10, 20, 10, 10, 99, 99, 99),
            (99, 10, 20, 99, 10, 99, 99, 99),
            (99, 10, 17, 15, 15, 10, 10, 99),
            (99, 10, 15, 20, 10, 10, 10, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
        ),
        (
            (10, 99, 99, 99, 99, 99, 99, 99, 99),
            (10, 99, 99, 10, 10, 99, 99, 99, 99),
            (10, 99, 99, 10, 25, 10, 20, 10, 99),
            (10, 99, 10, 15, 10, 10, 99, 10, 99),
            (10, 99, 10, 99, 10, 15, 10, 10, 99),
            (10, 99, 17, 10, 20, 99, 99, 99, 99),
            (10, 99, 99, 99, 99, 99, 99, 99, 99),
        )

    ]
    unsolvables = [
        # unsolvable:
        (
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 10, 20, 20, 99, 99, 99, 99),
            (99, 10, 15, 10, 10, 10, 10, 99),
            (99, 10, 10, 99, 15, 99, 10, 99),
            (99, 10, 17, 10, 20, 15, 10, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
        ),
        (
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 99, 99, 10, 20, 10, 10, 99),
            (99, 10, 15, 17, 99, 20, 10, 99),
            (99, 10, 10, 15, 99, 10, 99, 99),
            (99, 10, 10, 25, 10, 10, 99, 99),
            (99, 99, 10, 10, 99, 10, 99, 99),
            (99, 99, 99, 10, 10, 10, 99, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
        ),
        (
            (10, 99, 99, 99, 99, 99, 99, 99),
            (10, 99, 17, 25, 10, 10, 99, 99),
            (10, 99, 15, 15, 15, 10, 10, 99),
            (10, 99, 20, 10, 10, 99, 10, 99),
            (10, 99, 10, 10, 10, 20, 20, 99),
            (10, 99, 99, 99, 99, 99, 99, 99),
        ),
        (
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 10, 10, 10, 10, 10, 10, 99),
            (99, 10, 99, 15, 10, 10, 10, 99),
            (99, 10, 15, 10, 17, 99, 20, 99),
            (99, 99, 15, 99, 20, 10, 10, 99),
            (99, 99, 10, 10, 10, 10, 20, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),
            (99, 99, 99, 99, 99, 99, 99, 99),

        )
        # end of unsolvable
    ]
    solve_problems(problems + my_problems + unsolvables)


if __name__ == '__main__':
    main()
