#!/usr/bin/env python3
import argparse
import numpy as np
from scipy.spatial import cKDTree

# ==============================
# Load PCD ASCII
# ==============================

import plotly.graph_objects as go

def visualize_multi_traversability(pcd_points, robot_radius, n_robots=3, n_dyn_obstacles=5, max_steps=200, step_size=0.1):
    """
    Visualize multi-robot traversability with dynamic obstacles.
    """
    # Bounds
    x_l, x_h = pcd_points[:,0].min(), pcd_points[:,0].max()
    y_l, y_h = pcd_points[:,1].min(), pcd_points[:,1].max()

    # Static obstacles
    static_scatter = go.Scatter3d(
        x=pcd_points[:,0], y=pcd_points[:,1], z=pcd_points[:,2],
        mode='markers', marker=dict(size=2, color='gray', opacity=0.3),
        name='Static obstacles'
    )

    # Random initial positions
    robots_pos = np.column_stack([
        np.random.uniform(x_l, x_h, n_robots),
        np.random.uniform(y_l, y_h, n_robots),
        np.random.uniform(1.0, 3.0, n_robots)
    ])
    robots_dirs = np.random.uniform(0, 2*np.pi, n_robots)
    robots_dirs = np.column_stack([np.cos(robots_dirs), np.sin(robots_dirs), np.zeros(n_robots)])

    dyn_pos = np.column_stack([
        np.random.uniform(x_l, x_h, n_dyn_obstacles),
        np.random.uniform(y_l, y_h, n_dyn_obstacles),
        np.random.uniform(1.0, 3.0, n_dyn_obstacles)
    ])
    dyn_dirs = np.column_stack([np.cos(np.random.uniform(0, 2*np.pi, n_dyn_obstacles)),
                                np.sin(np.random.uniform(0, 2*np.pi, n_dyn_obstacles)),
                                np.zeros(n_dyn_obstacles)])

    # Store trajectories
    robot_traj = [ [robots_pos[i].copy()] for i in range(n_robots) ]
    dyn_traj = [ [dyn_pos[i].copy()] for i in range(n_dyn_obstacles) ]

    active = np.ones(n_robots, dtype=bool)

    # Simulate steps
    for _ in range(max_steps):
        if not np.any(active):
            break
        robots_pos[active] += robots_dirs[active] * step_size
        dyn_pos += dyn_dirs * step_size

        # Collision & bounds
        for i in range(n_robots):
            if not active[i]:
                continue
            x, y = robots_pos[i, 0], robots_pos[i, 1]
            if not (x_l <= x <= x_h and y_l <= y <= y_h):
                active[i] = False
                continue
            if np.any(np.linalg.norm(robots_pos[i] - dyn_pos, axis=1) < 2*robot_radius):
                active[i] = False
                continue
            for j in range(n_robots):
                if i != j and np.linalg.norm(robots_pos[i] - robots_pos[j]) < 2*robot_radius:
                    active[i] = False
                    break
        # Save positions
        for i in range(n_robots):
            if active[i]:
                robot_traj[i].append(robots_pos[i].copy())
        for i in range(n_dyn_obstacles):
            dyn_traj[i].append(dyn_pos[i].copy())

    # Plotly traces
    fig = go.Figure()
    fig.add_trace(static_scatter)

    # Robots
    for i in range(n_robots):
        traj = np.array(robot_traj[i])
        fig.add_trace(go.Scatter3d(
            x=traj[:,0], y=traj[:,1], z=traj[:,2],
            mode='lines+markers',
            marker=dict(size=robot_radius*50, color='blue'),
            line=dict(color='blue', width=4),
            name=f'Robot {i+1}'
        ))

    # Dynamic obstacles
    for i in range(n_dyn_obstacles):
        traj = np.array(dyn_traj[i])
        fig.add_trace(go.Scatter3d(
            x=traj[:,0], y=traj[:,1], z=traj[:,2],
            mode='lines+markers',
            marker=dict(size=robot_radius*50, color='red'),
            line=dict(color='red', width=2),
            name=f'Dyn Obs {i+1}'
        ))

    fig.update_layout(
        scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Z (m)',
            aspectmode='data'
        ),
        margin=dict(r=0, l=0, b=0, t=0)
    )
    fig.show()
def load_pcd_ascii(filename):
    points = []
    with open(filename) as f:
        data = False
        for line in f:
            if line.startswith("DATA"):
                data = True
                continue
            if data:
                vals = line.split()
                if len(vals) >= 3:
                    points.append([float(vals[0]),
                                   float(vals[1]),
                                   float(vals[2])])
    return np.asarray(points)

# ==============================
# Multi-robot traversability
# ==============================

def compute_multi_traversability(pcd_points, robot_radius, n_robots=3, 
                                 n_dyn_obstacles=5, N=5500, step_size=0.1, max_dist=50.0):
    x_l, x_h = pcd_points[:,0].min(), pcd_points[:,0].max()
    y_l, y_h = pcd_points[:,1].min(), pcd_points[:,1].max()

    tree = cKDTree(pcd_points)
    total_lengths = np.zeros(n_robots)

    for _ in range(N):
        # Random start positions for robots and dynamic obstacles
        robots_pos = np.column_stack([
            np.random.uniform(x_l, x_h, n_robots),
            np.random.uniform(y_l, y_h, n_robots),
            np.random.uniform(1.0, 3.0, n_robots)
        ])
        robots_dirs = np.random.uniform(0, 2*np.pi, n_robots)
        robots_dirs = np.column_stack([np.cos(robots_dirs), np.sin(robots_dirs), np.zeros(n_robots)])

        dyn_pos = np.column_stack([
            np.random.uniform(x_l, x_h, n_dyn_obstacles),
            np.random.uniform(y_l, y_h, n_dyn_obstacles),
            np.random.uniform(1.0, 3.0, n_dyn_obstacles)
        ])
        dyn_dirs = np.random.uniform(0, 2*np.pi, n_dyn_obstacles)
        dyn_dirs = np.column_stack([np.cos(dyn_dirs), np.sin(dyn_dirs), np.zeros(n_dyn_obstacles)])

        travelled = np.zeros(n_robots)
        active = np.ones(n_robots, dtype=bool)

        while np.any(active) and np.max(travelled) < max_dist:
            robots_pos[active] += robots_dirs[active] * step_size
            travelled[active] += step_size
            dyn_pos += dyn_dirs * step_size

            # Collision with static obstacles
            for i in np.where(active)[0]:
                if tree.query_ball_point(robots_pos[i], r=robot_radius):
                    active[i] = False

            # Collision with other robots
            for i in np.where(active)[0]:
                for j in range(n_robots):
                    if i != j and np.linalg.norm(robots_pos[i] - robots_pos[j]) < 2*robot_radius:
                        active[i] = False
                        break

            # Collision with dynamic obstacles
            for i in np.where(active)[0]:
                if np.any(np.linalg.norm(robots_pos[i] - dyn_pos, axis=1) < 2*robot_radius):
                    active[i] = False

            # Out-of-bounds check
            for i in np.where(active)[0]:
                x, y = robots_pos[i, 0], robots_pos[i, 1]
                if not (x_l <= x <= x_h and y_l <= y <= y_h):
                    active[i] = False

        total_lengths += travelled

    avg_lengths = total_lengths / N
    return avg_lengths

# ==============================
# Main
# ==============================
def main(pcd_file, robot_radius, n_robots, n_dyn_obstacles):
    points = load_pcd_ascii(pcd_file)
    print(f"Loaded {len(points)} points from {pcd_file}")

    # visualize_multi_traversability(points, robot_radius, n_robots, n_dyn_obstacles)
    avg_lengths = compute_multi_traversability(points, robot_radius, n_robots, n_dyn_obstacles)
    for i, length in enumerate(avg_lengths):
        print(f"Robot {i+1} average traversable path: {length:.2f} m")

# ==============================
# Entry point
# ==============================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_file", help="Path to PCD file")
    parser.add_argument("robot_radius", type=float, help="Robot radius in meters")
    parser.add_argument("n_robots", type=int, help="Number of robots")
    parser.add_argument("n_dyn_obstacles", type=int, help="Number of dynamic obstacles")
    args = parser.parse_args()

    main(args.pcd_file, args.robot_radius, args.n_robots, args.n_dyn_obstacles)
