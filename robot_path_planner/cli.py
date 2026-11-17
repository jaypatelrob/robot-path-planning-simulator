"""Run a deterministic path planning example from a terminal."""

import argparse
from .planner import find_path
from .scenarios import SCENARIOS, make_grid


def main() -> None:
    parser = argparse.ArgumentParser(description="Robot path planning with A* or Dijkstra")
    parser.add_argument("--algorithm", choices=("astar", "dijkstra"), default="astar")
    parser.add_argument("--scenario", choices=SCENARIOS, default="Maze challenge")
    parser.add_argument("--width", type=int, default=25)
    parser.add_argument("--height", type=int, default=18)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--density", type=float, default=0.12)
    args = parser.parse_args()
    start, goal = (0, 0), (args.width - 1, args.height - 1)
    grid = make_grid(args.width, args.height, args.scenario, args.density, args.seed, start, goal)
    result = find_path(grid, start, goal, args.algorithm)
    route = set(result.path)
    print(f"{args.algorithm.upper()} | {args.scenario} | {grid.width} x {grid.height}")
    for y in range(grid.height):
        line = ""
        for x in range(grid.width):
            cell = (x, y)
            char = "S" if cell == start else "G" if cell == goal else "#" if cell in grid.walls else "*" if cell in route else "."
            line += char
        print(line)
    print(f"Found: {result.found} | Path cost: {result.cost} | Expanded nodes: {result.expanded}")


if __name__ == "__main__":
    main()
