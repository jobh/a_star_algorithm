

import unittest
import a_star
from a_star.tools import GridWithWeights, manhattan_distance


class MazeTests(unittest.TestCase):

    def test_maze_1(self):
        maze = GridWithWeights(4,4)
        walls = [(1,1),(2,2)]
        maze.walls = walls
        weights = {(1,0):20,(3,0) : 2}
        maze.weights = weights
        my_solution = [(3,0),(3,1),(3,2),(3,3),(2,3),(1,3),(0,3),(0,2)]

        end = (3,0)
        start = (0,2)

        # Call the A* algorithm and get the frontier
        frontier = a_star.a_star_search(graph = maze, start=start, end=end, heuristic=manhattan_distance)
        solution = list(frontier.backtrack(end))
        self.assertTrue( solution == my_solution  )


    def test_weights_instead_of_walls(self):
        maze = GridWithWeights(4,4)
        walls = []
        maze.walls = walls
        weights = { (1,1):300, (1,2): 300, (1,3):300}
        maze.weights = weights

        start = (0,3)
        end = (3,3)

        my_solution = [(3, 3), (2, 3), (2, 2), (2, 1),
                       (2, 0), (1, 0), (0, 0), (0, 1),
                       (0, 2), (0, 3)]
        # Call the A* algorithm and get the frontier
        frontier = a_star.a_star_search(graph = maze, start=start, end=end, heuristic=manhattan_distance)

        solution = list(frontier.backtrack(end))

        self.assertTrue( solution == my_solution  )

if __name__ == "__main__":
    unittest.main()
