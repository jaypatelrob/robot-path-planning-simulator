"""Reproducible obstacle maps for the interactive demo."""

from __future__ import annotations
import random
from .planner import Cell, Grid

SCENARIOS = ("Maze challenge", "Warehouse shelves", "Random obstacles", "Clear floor", "Blocked passage")


def make_grid(
    width: int = 25,
    height: int = 18,
    scenario: str = "Maze challenge",
    density: float = 0.12,
    seed: int = 7,
    start: Cell = (0, 0),
    goal: Cell | None = None,
) -> Grid:
    """Create a grid with endpoints left clear.

    Warehouse and random modes preserve a top-row/right-column route from
    (0, 0) to (width-1, height-1), but user-chosen endpoints need not have a route.
    Blocked passage intentionally divides the grid to demonstrate failure.
    """
    if width < 2 or height < 2:
        raise ValueError("Grid dimensions must be at least 2 by 2")
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {scenario}")
    if not 0 <= density <= 0.5:
        raise ValueError("density must be between 0 and 0.5")
    if goal is None:
        goal = (width - 1, height - 1)
    for cell in (start, goal):
        if not (0 <= cell[0] < width and 0 <= cell[1] < height):
            raise ValueError("Start and goal must be inside the grid")

    rng = random.Random(seed)
    walls: set[Cell] = set()
    protected = {(x, 0) for x in range(width)} | {(width - 1, y) for y in range(height)}

    if scenario == "Maze challenge":
        for y in range(3, height - 1, 3):
            opening = 1 if (y // 3) % 2 == 1 else width - 2
            for x in range(width):
                if x != opening:
                    walls.add((x, y))
    elif scenario == "Warehouse shelves":
        for x in range(3, width - 1, 5):
            for y in range(2, height - 1):
                if y not in (height - 4 if (x // 5) % 2 == 0 else 3,):
                    walls.add((x, y))
    elif scenario == "Random obstacles":
        for x in range(width):
            for y in range(height):
                if (x, y) not in protected and rng.random() < density:
                    walls.add((x, y))
    elif scenario == "Blocked passage":
        for y in range(height):
            walls.add((width // 2, y))

    walls.discard(start)
    walls.discard(goal)
    return Grid(width, height, frozenset(walls))
