from SearchSolution import SearchSolution
from collections import deque

class SearchNode:

        def __init__(self, state, parent=None):
            self.parent = parent
            self.state = state
        def __str__(self):
            return str(self.state)

def bfs_search(mazeworldProblem):
    start_node = SearchNode(mazeworldProblem.start_state)
    queue = [start_node]
    set = {mazeworldProblem.start_state}

    while len(queue) > 0:
        current_node = queue.pop(0)
        if mazeworldProblem.goal_test(current_node.state):
            solution = SearchSolution(mazeworldProblem, "BFS")
            solution.path = backcheck(current_node)
            return solution
        for sucessor in mazeworldProblem.get_successors(current_node.state, mazeworldProblem.maze):
            if sucessor not in set:
                queue.append(SearchNode(sucessor, current_node))
                set.add(sucessor)

def backcheck(node):
    correct_path = [node.state]
    while node.parent != None:
        node = node.parent
        correct_path.insert(0, node.state)
    return correct_path

