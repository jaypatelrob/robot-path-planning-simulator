"""Tests for search correctness, input validation, and scenario reproducibility."""

import unittest

from robot_path_planner import Grid, find_path, make_grid
from robot_path_planner.planner import manhattan


class GridTests(unittest.TestCase):
    def test_edges_and_walls(self):
        grid = Grid(4, 4, frozenset({(1, 0), (0, 1)}))
        self.assertEqual(grid.neighbors((0, 0)), ())
        self.assertFalse(grid.traversable((-1, 0)))

    def test_invalid_dimensions(self):
        with self.assertRaises(ValueError):
            Grid(1, 4, frozenset())

    def test_out_of_bounds_wall(self):
        with self.assertRaises(ValueError):
            Grid(4, 4, frozenset({(4, 0)}))


class SearchTests(unittest.TestCase):
    def test_open_grid_shortest_paths(self):
        grid = Grid(8, 6, frozenset())
        for algorithm in ("astar", "dijkstra"):
            result = find_path(grid, (0, 0), (7, 5), algorithm)
            self.assertTrue(result.found)
            self.assertEqual(result.cost, 12)
            self.assertEqual(len(result.path), 13)
            self.assertEqual(result.path[0], (0, 0))
            self.assertEqual(result.path[-1], (7, 5))

    def test_same_start_goal(self):
        grid = Grid(4, 4, frozenset())
        result = find_path(grid, (2, 2), (2, 2))
        self.assertEqual(result.path, ((2, 2),))
        self.assertEqual(result.cost, 0)
        self.assertEqual(result.expanded, 1)

    def test_no_route(self):
        grid = Grid(5, 5, frozenset({(2, y) for y in range(5)}))
        for algorithm in ("astar", "dijkstra"):
            result = find_path(grid, (0, 0), (4, 4), algorithm)
            self.assertFalse(result.found)
            self.assertEqual(result.path, ())
            self.assertIsNone(result.cost)

    def test_blocked_endpoint_rejected(self):
        grid = Grid(4, 4, frozenset({(0, 0)}))
        with self.assertRaises(ValueError):
            find_path(grid, (0, 0), (3, 3))
        with self.assertRaises(ValueError):
            find_path(grid, (1, 1), (10, 10))

    def test_unknown_algorithm_rejected(self):
        with self.assertRaises(ValueError):
            find_path(Grid(3, 3, frozenset()), (0, 0), (2, 2), "greedy")

    def test_obstacles_and_optimality(self):
        for scenario in ("Maze challenge", "Warehouse shelves", "Random obstacles", "Clear floor", "Blocked passage"):
            for seed in range(10):
                grid = make_grid(25, 18, scenario, 0.31, seed)
                a = find_path(grid, (0, 0), (24, 17), "astar")
                d = find_path(grid, (0, 0), (24, 17), "dijkstra")
                self.assertEqual(a.cost, d.cost)
                self.assertEqual(a.found, d.found)
                for result in (a, d):
                    if result.found:
                        self.assertEqual(result.cost, len(result.path) - 1)
                        self.assertTrue(all(grid.traversable(node) for node in result.path))
                        self.assertTrue(all(manhattan(u, v) == 1 for u, v in zip(result.path, result.path[1:])))

    def test_no_diagonal_shortcut(self):
        grid = Grid(2, 2, frozenset({(1, 0), (0, 1)}))
        self.assertFalse(find_path(grid, (0, 0), (1, 1)).found)


class ScenarioTests(unittest.TestCase):
    def test_seed_reproducible(self):
        g1 = make_grid(25, 18, "Random obstacles", 0.20, 77)
        g2 = make_grid(25, 18, "Random obstacles", 0.20, 77)
        self.assertEqual(g1.walls, g2.walls)
        self.assertNotEqual(g1.walls, make_grid(25, 18, "Random obstacles", 0.20, 78).walls)

    def test_endpoints_clear(self):
        grid = make_grid(10, 10, "Blocked passage", start=(5, 5), goal=(5, 6))
        self.assertTrue(grid.traversable((5, 5)))
        self.assertTrue(grid.traversable((5, 6)))

    def test_invalid_scenarios(self):
        with self.assertRaises(ValueError):
            make_grid(scenario="Supermarket")
        with self.assertRaises(ValueError):
            make_grid(density=0.8)

    def test_default_scenarios_find_path(self):
        for scenario in ("Maze challenge", "Warehouse shelves", "Random obstacles", "Clear floor"):
            grid = make_grid(scenario=scenario)
            self.assertTrue(find_path(grid, (0, 0), (24, 17)).found)
        blocked = make_grid(scenario="Blocked passage")
        self.assertFalse(find_path(blocked, (0, 0), (24, 17)).found)


if __name__ == "__main__":
    unittest.main()
