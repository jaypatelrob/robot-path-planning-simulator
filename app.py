"""Streamlit UI for the robot path-planning portfolio demo.

Run: streamlit run app.py
"""

from __future__ import annotations
import csv
from io import StringIO
import time

import streamlit as st
import matplotlib.pyplot as plt

from robot_path_planner import find_path, make_grid
from robot_path_planner.plotting import COLORS, plot_search
from robot_path_planner.scenarios import SCENARIOS

st.set_page_config(page_title="Robot Route Lab | A* Path Planning", page_icon="🤖", layout="wide")
st.title("🤖 Robot Route Lab")
st.caption("Explore shortest-path planning on a 2D obstacle grid. Compare A* search against Dijkstra's algorithm.")

with st.sidebar:
    st.header("Map controls")
    scenario = st.selectbox("Environment", SCENARIOS)
    width = st.slider("Grid width", 10, 40, 25)
    height = st.slider("Grid height", 10, 32, 18)
    seed = st.number_input("Random seed", min_value=0, max_value=1_000_000, value=7, step=1)
    density = st.slider("Random obstacle density", 0, 45, 15, 5) / 100
    if scenario != "Random obstacles":
        st.caption("Obstacle density applies only to the Random obstacles environment.")
    st.divider()
    st.subheader("Robot coordinates")
    sx = st.number_input("Start X", 0, width - 1, 0)
    sy = st.number_input("Start Y", 0, height - 1, 0)
    gx = st.number_input("Goal X", 0, width - 1, width - 1)
    gy = st.number_input("Goal Y", 0, height - 1, height - 1)
    st.caption("X increases rightward; Y increases downward. Moves are up/down/left/right.")

start, goal = (int(sx), int(sy)), (int(gx), int(gy))
grid = make_grid(width, height, scenario, density, int(seed), start, goal)

with st.spinner("Computing shortest routes..."):
    start_time = time.perf_counter()
    a = find_path(grid, start, goal, "astar")
    a_ms = (time.perf_counter() - start_time) * 1000
    start_time = time.perf_counter()
    d = find_path(grid, start, goal, "dijkstra")
    d_ms = (time.perf_counter() - start_time) * 1000

if a.found != d.found or a.cost != d.cost:
    st.error("Internal consistency check failed: algorithms disagree. Please report the map settings.")
    st.stop()

m1, m2, m3, m4 = st.columns(4)
m1.metric("Route found", "Yes" if a.found else "No")
m2.metric("Shortest path", f"{a.cost} steps" if a.found else "No route")
m3.metric("A* nodes expanded", a.expanded)
m4.metric("Dijkstra nodes expanded", d.expanded)

if not a.found:
    st.warning("No traversable route connects these endpoints. Try changing the environment or start/goal.")

tab_explore, tab_compare, tab_learn = st.tabs(["🔎 Explore search", "📊 Compare algorithms", "📖 How it works"])
with tab_explore:
    left, right = st.columns([3, 1], gap="large")
    with right:
        algorithm = st.radio("Algorithm", ["A*", "Dijkstra"], horizontal=False)
        result = a if algorithm == "A*" else d
        progress = st.slider("Search progress (expanded cells)", 0, result.expanded, result.expanded, key=f"progress_{algorithm}")
        st.markdown("**Map key**")
        for label, color in zip(["Open cell", "Obstacle", "Explored", "Shortest route", "Start (S)", "Goal (G)"], COLORS):
            st.markdown(f'<span style="display:inline-block;width:12px;height:12px;background:{color};border:1px solid #cbd5e1;margin-right:8px"></span>{label}', unsafe_allow_html=True)
        st.caption("Drag the progress slider backward to see the exploration sequence. The complete route appears only at the final step.")
    with left:
        fig = plot_search(grid, start, goal, result, progress, f"{algorithm} | {scenario}")
        st.pyplot(fig, use_container_width=True)
        plt.close(fig)
    if a.found:
        output = StringIO()
        writer = csv.writer(output)
        writer.writerow(["step", "x", "y"])
        for i, (x, y) in enumerate(result.path):
            writer.writerow([i, x, y])
        st.download_button("Download route coordinates (.csv)", output.getvalue(), file_name=f"{result.algorithm}_route.csv", mime="text/csv")

with tab_compare:
    st.markdown("Both algorithms use the **same map**, four-direction movement, and unit movement cost. Their optimal path lengths must match.")
    st.dataframe(
        {
            "Algorithm": ["A*", "Dijkstra"],
            "Route cost (steps)": [a.cost, d.cost],
            "Expanded nodes": [a.expanded, d.expanded],
            "Elapsed time (ms)": [round(a_ms, 3), round(d_ms, 3)],
        },
        hide_index=True,
        use_container_width=True,
    )
    st.caption("Execution times are illustrative, measured once on this server; they are not controlled performance benchmarks.")
    c1, c2 = st.columns(2)
    with c1:
        fig = plot_search(grid, start, goal, a, title="A* — heuristic-guided")
        st.pyplot(fig, use_container_width=True)
        plt.close(fig)
    with c2:
        fig = plot_search(grid, start, goal, d, title="Dijkstra — uninformed")
        st.pyplot(fig, use_container_width=True)
        plt.close(fig)
    if a.found:
        st.success(f"Both algorithms found an optimal {a.cost}-step route. A* expanded {a.expanded} nodes; Dijkstra expanded {d.expanded}.")

with tab_learn:
    st.subheader("How the algorithm works")
    st.markdown("""
    1. **Represent the world** as a 2D grid of open cells and obstacles.
    2. **Explore neighbors** in four directions, each move costing one step.
    3. **Track the cheapest known distance** to each cell and remember its parent.
    4. **Choose the next candidate** from a priority queue; A* uses f(n) = g(n) + h(n), while Dijkstra uses f(n) = g(n).
    5. **Reconstruct the route** by following parent links from the goal to the start.

    **A* heuristic:** Manhattan distance, abs(x - goal_x) + abs(y - goal_y). It never overestimates the remaining cost for four-direction unit-cost movement, so the returned route is optimal when a route exists.

    **Limitations:** This is a grid-world simulation—not a physically accurate robot model. It does not account for turning radius, dynamics, moving obstacles, sensors, or collision geometry beyond occupied cells.
    """)
    st.markdown("Source code, unit tests, usage instructions, and learning reference are included in the project's GitHub repository.")

st.divider()
st.caption("Independent educational implementation. Inspired by path-planning concepts documented in Atsushi Sakai et al.'s PythonRobotics project; no code copied. AI-assisted development is disclosed in the README.")
