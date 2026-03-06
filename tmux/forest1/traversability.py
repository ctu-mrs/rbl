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
def sample_start_on_surface(tree, points, x_l, x_h, y_l, y_h, R,
                            min_clearance=2.0, max_clearance=2.5):
    """
    Sample a random (x,y) and place robot at a random height above the
    nearest surface point.
    """

    while True:
        x = np.random.uniform(x_l, x_h)
        y = np.random.uniform(y_l, y_h)

        # find nearest surface point
        dist, idx = tree.query([x, y, 0], k=1)
        ground_z = points[idx][2]

        # random altitude above surface
        clearance = np.random.uniform(min_clearance, max_clearance)
        z = ground_z + clearance

        start = np.array([x, y, z])

        # reject if still colliding
        if not tree.query_ball_point(start, r=R):
            return start
# import open3d as o3d

# def visualize_paths(paths):
#     geometries = []

#     for path in paths:
#         if len(path) < 2:
#             continue

#         # create line indices
#         lines = [[i, i+1] for i in range(len(path)-1)]

#         line_set = o3d.geometry.LineSet()
#         line_set.points = o3d.utility.Vector3dVector(path)
#         line_set.lines = o3d.utility.Vector2iVector(lines)

#         # color: red
#         colors = [[1, 0, 0] for _ in lines]
#         line_set.colors = o3d.utility.Vector3dVector(colors)

#         geometries.append(line_set)

#     o3d.visualization.draw_geometries(geometries)
import open3d as o3d
import numpy as np

def visualize_paths(paths):
    geometries = []

    for path in paths:
        if len(path) < 2:
            continue

        path = np.asarray(path)

        # -------------------
        # 1) draw trajectory
        # -------------------
        lines = [[i, i+1] for i in range(len(path)-1)]

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(path)
        line_set.lines  = o3d.utility.Vector2iVector(lines)

        # red path
        colors = [[1, 0, 0] for _ in lines]
        line_set.colors = o3d.utility.Vector3dVector(colors)

        geometries.append(line_set)

        # -------------------
        # 2) START POINT (blue sphere)
        # -------------------
        start = path[0]

        start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
        start_sphere.compute_vertex_normals()
        start_sphere.paint_uniform_color([0, 0, 1])  # blue
        start_sphere.translate(start)

        # geometries.append(start_sphere)

        # -------------------
        # 3) COLLISION POINT (black sphere)
        # -------------------
        end = path[-1]

        end_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.10)
        end_sphere.compute_vertex_normals()
        end_sphere.paint_uniform_color([0, 0, 0])  # black
        end_sphere.translate(end)

        geometries.append(end_sphere)

    # better camera defaults
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1280, height=900)
    for g in geometries:
        vis.add_geometry(g)

    opt = vis.get_render_option()
    opt.background_color = np.asarray([1, 1, 1])  # white background
    opt.line_width = 2.0

    vis.run()
    vis.destroy_window()
# -------------------------------
# Traversability
# -------------------------------
def compute_traversability(pcd_points, R, N=50000, step_size=0.2,
                           max_dist=500.0, return_paths=False):

    x_l, x_h = pcd_points[:,0].min(), pcd_points[:,0].max()
    y_l, y_h = pcd_points[:,1].min(), pcd_points[:,1].max()

    tree = cKDTree(pcd_points)

    total_length = 0.0
    all_paths = []
    valid_samples = 0

    while valid_samples < N:

        # -------------------------
        # 1) Sample valid start
        # -------------------------
        # start = np.array([
        #     np.random.uniform(x_l, x_h),
        #     np.random.uniform(y_l, y_h),
        #     2.0   # FIXED robot height
        # ])
        start = sample_start_on_surface(tree, pcd_points,
                                x_l, x_h, y_l, y_h, R)

        # If spawn already collides → reject
        if tree.query_ball_point(start, r=R):
            continue

        theta = np.random.uniform(0, 2*np.pi)
        direction = np.array([np.cos(theta), np.sin(theta), 0.0])

        pos = start.copy()
        travelled = 0.0
        path = [pos.copy()]

        collided = False
        out_of_bounds = False

        # -------------------------
        # 2) Rollout
        # -------------------------
        while travelled < max_dist:
            pos += direction * step_size
            travelled += step_size

            # Boundary check
            if not (x_l <= pos[0] <= x_h and y_l <= pos[1] <= y_h):
                out_of_bounds = True
                break

            # Collision check
            if tree.query_ball_point(pos, r=R):
                collided = True
                break

            path.append(pos.copy())

        # -------------------------
        # 3) Accept only collisions
        # -------------------------
        if collided and not out_of_bounds:
            total_length += travelled
            valid_samples += 1

            if return_paths:
                all_paths.append(np.array(path))

    avg_length = total_length / N

    if return_paths:
        return avg_length, all_paths
    return avg_length
# def compute_traversability(pcd_points, R, N=5000, step_size=0.1, max_dist=50.0, return_paths=False):
#     x_l, x_h = pcd_points[:,0].min(), pcd_points[:,0].max()
#     y_l, y_h = pcd_points[:,1].min(), pcd_points[:,1].max()

#     tree = cKDTree(pcd_points)
#     total_length = 0.0

#     all_paths = []

#     for _ in range(N):
#         start = np.array([
#             np.random.uniform(x_l, x_h),
#             np.random.uniform(y_l, y_h),
#             np.random.uniform(1.0, 3.0)
#         ])

#         theta = np.random.uniform(0, 2*np.pi)
#         direction = np.array([np.cos(theta), np.sin(theta), 0.0])

#         pos = start.copy()
#         travelled = 0.0

#         path = [pos.copy()]

#         while travelled < max_dist:
#             pos += direction * step_size
#             travelled += step_size

#             # Out of bounds
#             if not (x_l <= pos[0] <= x_h and y_l <= pos[1] <= y_h):
#                 break

#             # Collision
#             if tree.query_ball_point(pos, r=R):
#                 break

#             path.append(pos.copy())

#         total_length += travelled

#         if return_paths:
#             all_paths.append(np.array(path))

#     avg_length = total_length / N

#     if return_paths:
#         return avg_length, all_paths
#     return avg_length
# def compute_traversability(pcd_points, R, N=5000, step_size=0.1, max_dist=50.0):
#     x_l, x_h = pcd_points[:,0].min(), pcd_points[:,0].max()
#     y_l, y_h = pcd_points[:,1].min(), pcd_points[:,1].max()
#     z_l, z_h = pcd_points[:,2].min(), pcd_points[:,2].max()

#     tree = cKDTree(pcd_points)
#     total_length = 0.0

#     for _ in range(N):
#         start = np.array([
#             np.random.uniform(x_l, x_h),
#             np.random.uniform(y_l, y_h),
#             np.random.uniform(1.0, 3.0)
#         ])
#         theta = np.random.uniform(0, 2*np.pi)
#         direction = np.array([np.cos(theta), np.sin(theta), 0.0])

#         pos = start.copy()
#         travelled = 0.0

#         while travelled < max_dist:
#             pos += direction * step_size
#             travelled += step_size

#             # Check out-of-bounds
#             if not (x_l <= pos[0] <= x_h and y_l <= pos[1] <= y_h):
#                 break

#             # Check collision
#             if tree.query_ball_point(pos, r=R):
#                 break

#         total_length += travelled

#     avg_length = total_length / N
#     return avg_length

# -------------------------------
# Main
# -------------------------------
def main(pcd_file, robot_radius):
    points = load_pcd_ascii(pcd_file)
    print(f"Loaded {len(points)} points from {pcd_file}")

    trav_length, paths = compute_traversability(
        points,
        robot_radius,
        N=10000,                 # IMPORTANT: keep small for visualization
        return_paths=True
    )

    print(f"Traversability (avg path length before collision): {trav_length:.2f} m")

    visualize_paths(paths)

# -------------------------------
# Entry point
# -------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_file", help="Path to PCD file")
    parser.add_argument("robot_radius", type=float, help="Robot radius in meters")
    args = parser.parse_args()

    main(args.pcd_file, args.robot_radius)
