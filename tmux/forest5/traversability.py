#!/usr/bin/env python3
import argparse
import numpy as np
from scipy.spatial import cKDTree

# -------------------------------
# Load PCD ASCII
# -------------------------------
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

# -------------------------------
# Traversability
# -------------------------------
def compute_traversability(pcd_points, R, N=1000, step_size=0.1, max_dist=50.0):
    x_l, x_h = pcd_points[:,0].min(), pcd_points[:,0].max()
    y_l, y_h = pcd_points[:,1].min(), pcd_points[:,1].max()
    z_l, z_h = pcd_points[:,2].min(), pcd_points[:,2].max()

    tree = cKDTree(pcd_points)
    total_length = 0.0

    for _ in range(N):
        start = np.array([
            np.random.uniform(x_l, x_h),
            np.random.uniform(y_l, y_h),
            np.random.uniform(1.0, 3.0)
        ])
        theta = np.random.uniform(0, 2*np.pi)
        direction = np.array([np.cos(theta), np.sin(theta), 0.0])

        pos = start.copy()
        travelled = 0.0

        while travelled < max_dist:
            pos += direction * step_size
            travelled += step_size

            # Check out-of-bounds
            if not (x_l <= pos[0] <= x_h and y_l <= pos[1] <= y_h):
                break

            # Check collision
            if tree.query_ball_point(pos, r=R):
                break

        total_length += travelled

    avg_length = total_length / N
    return avg_length

# -------------------------------
# Main
# -------------------------------
def main(pcd_file, robot_radius):
    points = load_pcd_ascii(pcd_file)
    print(f"Loaded {len(points)} points from {pcd_file}")

    trav_length = compute_traversability(points, robot_radius)
    print(f"Traversability (avg path length before collision): {trav_length:.2f} m")

# -------------------------------
# Entry point
# -------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_file", help="Path to PCD file")
    parser.add_argument("robot_radius", type=float, help="Robot radius in meters")
    args = parser.parse_args()

    main(args.pcd_file, args.robot_radius)
