# A* and Dijkstra: how they work

## Problem statement
A robot begins at **S** on a 2D grid and needs to reach **G** without stepping on obstacles. Each north, south, east, or west move costs **1**. The algorithms return a minimum-cost route when one exists. This is a grid-search demonstration, not a physics/sensing simulation.

## A* heuristic
**priority(n) = g(n) + h(n)**. Here g(n) is the shortest known distance from the start, and h(n) is estimated distance remaining. This project uses Manhattan distance: |x - goal_x| + |y - goal_y|. For four-direction unit-cost moves, that heuristic is admissible and consistent, so A* finds an optimal route.

## Dijkstra
**priority(n) = g(n)**. Dijkstra does not estimate how close a cell is to the goal, so it often explores more cells. It must agree with A* about optimal cost, although equally short routes may differ.

## Pseudocode
```text
push start into priority queue
distance[start] = 0
while priority queue is not empty:
    pop lowest-priority cell
    if already expanded: continue
    mark expanded
    if goal reached: reconstruct path using parent pointers
    for each non-obstacle neighbor:
        new_cost = distance[current] + 1
        if new_cost improves distance[neighbor]:
            update distance and parent
            push neighbor with priority = new_cost + heuristic
return no route
```

## Code structure
- `robot_path_planner/planner.py`: grid graph, A* and Dijkstra, visited order and path reconstruction
- `robot_path_planner/scenarios.py`: warehouse, maze, random, blocked, and clear maps
- `robot_path_planner/plotting.py`: renders pathfinding with Matplotlib
- `robot_path_planner/cli.py`: command-line example
- `app.py`: Streamlit exploration and algorithm comparison
- `tests/test_planner.py`: validation, optimality, no-route and reproducibility tests

## Limitations
The robot is a grid point. There are no wheel dynamics, sensor measurements, moving obstacles, turning radius or continuous-space collision geometry.

## Attribution
Algorithms were learned from standard shortest-path concepts and [PythonRobotics](https://atsushisakai.github.io/PythonRobotics/) by Atsushi Sakai and contributors. This code is an independently written, AI-assisted educational implementation and does not copy PythonRobotics source.
