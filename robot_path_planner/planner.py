"""Original educational implementations of A* and Dijkstra on a 4-connected grid.

A cell (x, y) has unit cost to move north, south, east, or west. The
Manhattan-distance heuristic is admissible and consistent for these moves.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
from itertools import count
from math import inf

Cell = tuple[int, int]


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    walls: frozenset[Cell]

    def __post_init__(self) -> None:
        if self.width < 2 or self.height < 2:
            raise ValueError("Grid dimensions must be at least 2 by 2")
        if any(not self.in_bounds(cell) for cell in self.walls):
            raise ValueError("A wall lies outside the grid")

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def traversable(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and cell not in self.walls

    def neighbors(self, cell: Cell) -> tuple[Cell, ...]:
        x, y = cell
        candidates = ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1))
        return tuple(next_cell for next_cell in candidates if self.traversable(next_cell))


@dataclass(frozen=True)
class SearchResult:
    algorithm: str
    path: tuple[Cell, ...]
    visited: tuple[Cell, ...]
    cost: int | None

    @property
    def found(self) -> bool:
        return self.cost is not None

    @property
    def expanded(self) -> int:
        return len(self.visited)


def manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _reconstruct(parents: dict[Cell, Cell], goal: Cell) -> tuple[Cell, ...]:
    route = [goal]
    current = goal
    while current in parents:
        current = parents[current]
        route.append(current)
    route.reverse()
    return tuple(route)


def find_path(grid: Grid, start: Cell, goal: Cell, algorithm: str = "astar") -> SearchResult:
    """Find an optimal route or return an empty path when no route exists.

    Raises ValueError for invalid endpoints or unknown algorithms.
    A* uses Manhattan distance; Dijkstra uses a zero heuristic.
    """
    if algorithm not in {"astar", "dijkstra"}:
        raise ValueError("algorithm must be 'astar' or 'dijkstra'")
    if not grid.traversable(start) or not grid.traversable(goal):
        raise ValueError("Start and goal must be traversable cells inside the grid")

    heuristic = manhattan if algorithm == "astar" else (lambda _a, _b: 0)
    queue: list[tuple[int, int, int, Cell]] = []
    counter = count()
    heapq.heappush(queue, (heuristic(start, goal), heuristic(start, goal), next(counter), start))
    distance: dict[Cell, int] = {start: 0}
    parents: dict[Cell, Cell] = {}
    explored: set[Cell] = set()
    visited: list[Cell] = []

    while queue:
        _, _, _, current = heapq.heappop(queue)
        if current in explored:
            continue
        explored.add(current)
        visited.append(current)
        if current == goal:
            return SearchResult(algorithm, _reconstruct(parents, goal), tuple(visited), distance[goal])

        for neighbor in grid.neighbors(current):
            candidate_cost = distance[current] + 1
            if candidate_cost < distance.get(neighbor, inf):
                distance[neighbor] = candidate_cost
                parents[neighbor] = current
                priority = candidate_cost + heuristic(neighbor, goal)
                heapq.heappush(queue, (priority, heuristic(neighbor, goal), next(counter), neighbor))

    return SearchResult(algorithm, (), tuple(visited), None)
