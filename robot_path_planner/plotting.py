"""Matplotlib visualizations for grid search; suitable for local and Streamlit use."""

from __future__ import annotations
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
import numpy as np
from .planner import Cell, Grid, SearchResult

COLORS = ["#F8FAFC", "#273449", "#D1E9FB", "#3B82F6", "#0FA968", "#F97316"]


def plot_search(
    grid: Grid,
    start: Cell,
    goal: Cell,
    result: SearchResult,
    progress: int | None = None,
    title: str | None = None,
):
    """Render obstacle grid, expanded nodes, completed route, start and goal.

    progress shows only that many expanded cells; the final route is drawn only
    after goal has been expanded. Returns a Matplotlib figure for caller to close.
    """
    if progress is None:
        progress = len(result.visited)
    progress = min(max(int(progress), 0), len(result.visited))
    codes = np.zeros((grid.height, grid.width), dtype=int)
    for x, y in grid.walls:
        codes[y, x] = 1
    for x, y in result.visited[:progress]:
        codes[y, x] = 2
    search_complete = progress == len(result.visited)
    if search_complete:
        for x, y in result.path:
            codes[y, x] = 3
    codes[start[1], start[0]] = 4
    codes[goal[1], goal[0]] = 5

    fig, ax = plt.subplots(figsize=(9.5, 6.3))
    fig.patch.set_facecolor("#FFFFFF")
    cmap = ListedColormap(COLORS)
    norm = BoundaryNorm(np.arange(-0.5, 6.5, 1), cmap.N)
    ax.imshow(codes, cmap=cmap, norm=norm, interpolation="nearest", origin="upper")
    ax.set_xticks(np.arange(-0.5, grid.width, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.height, 1), minor=True)
    ax.grid(which="minor", color="#E2E8F0", linewidth=0.55)
    ax.tick_params(which="minor", bottom=False, left=False)
    ax.set_xticks(np.arange(0, grid.width, max(1, grid.width // 8)))
    ax.set_yticks(np.arange(0, grid.height, max(1, grid.height // 8)))
    ax.tick_params(labelsize=9, colors="#64748B")
    ax.set_title(title or f"{result.algorithm.upper()} search", fontsize=15, weight="bold", pad=15, color="#0F172A")
    ax.text(start[0], start[1], "S", ha="center", va="center", color="white", weight="bold", fontsize=10)
    ax.text(goal[0], goal[1], "G", ha="center", va="center", color="white", weight="bold", fontsize=10)
    ax.set_xlabel("X / columns", fontsize=10, color="#475569")
    ax.set_ylabel("Y / rows", fontsize=10, color="#475569")
    fig.tight_layout()
    return fig
