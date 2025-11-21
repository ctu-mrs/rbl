
import re
import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# -----------------------------
# Decode raw.txt PointCloud2
# -----------------------------
FILENAME = "map_ego.txt"
with open(FILENAME, "r") as f:
    text = f.read()

width = int(re.search(r"width:\s*(\d+)", text).group(1))
point_step = int(re.search(r"point_step:\s*(\d+)", text).group(1))
data_str = re.search(r"data:\s*\[([0-9,\s]+)\]", text, re.DOTALL).group(1)
data_bytes = np.array([int(x) for x in data_str.split(",")], dtype=np.uint8)

points = []
for i in range(width):
    offset = i * point_step
    x = np.frombuffer(data_bytes[offset+0:offset+4], dtype=np.float32)[0]
    y = np.frombuffer(data_bytes[offset+4:offset+8], dtype=np.float32)[0]
    z = np.frombuffer(data_bytes[offset+8:offset+12], dtype=np.float32)[0]
    points.append([x, y, z])
points = np.array(points)

# Open3D PointCloud for map
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd = pcd.voxel_down_sample(voxel_size=0.05)
pcd.paint_uniform_color([0.1, 0.6, 0.9])  # light blue

# -----------------------------
# Load trajectory CSV and analyze
# -----------------------------
CSV_FILE = "ego_odom.csv"
DIST_THRESHOLD = 0.20
SPEED_THRESHOLD = 0.01
WINDOW_LENGTH = 11
POLYORDER = 2

df = pd.read_csv(CSV_FILE)
x = df["field.pose.position.x"].values
y = df["field.pose.position.y"].values
z = df["field.pose.position.z"].values
t = df["%time"].values * 1e-9
t = t - t[0]
speed = df["field.scale.x"].values

# Moving segment
idx_start = np.where(speed > SPEED_THRESHOLD)[0][0]
goal = np.array([x[-1], y[-1], z[-1]])
dist = np.sqrt((x - goal[0])**2 + (y - goal[1])**2 + (z - goal[2])**2)
idx_end_candidates = np.where(dist <= DIST_THRESHOLD)[0]
idx_end = idx_end_candidates[0] if len(idx_end_candidates) > 0 else len(t)-1

x_m = x[idx_start:idx_end+1]
y_m = y[idx_start:idx_end+1]
z_m = z[idx_start:idx_end+1]
t_m = t[idx_start:idx_end+1]
speed_m = speed[idx_start:idx_end+1]

# Remove duplicates
_, unique_idx = np.unique(t_m, return_index=True)
x_m = x_m[unique_idx]
y_m = y_m[unique_idx]
z_m = z_m[unique_idx]
t_m = t_m[unique_idx]
speed_m = speed_m[unique_idx]

# Acceleration
dt = np.diff(t_m)
dv = np.diff(speed_m)
acceleration_m = np.concatenate(([0], np.divide(dv, dt, out=np.zeros_like(dv), where=dt!=0)))
if len(acceleration_m) >= WINDOW_LENGTH:
    acceleration_filtered = savgol_filter(acceleration_m, WINDOW_LENGTH, POLYORDER)
else:
    acceleration_filtered = acceleration_m

# Path length
diffs = np.sqrt(np.diff(x_m)**2 + np.diff(y_m)**2 + np.diff(z_m)**2)
path_length = np.sum(diffs)
time_to_reach = t_m[-1] - t_m[0]

# -----------------------------
# Create Open3D trajectory objects
# -----------------------------
trajectory_points = np.vstack([x_m, y_m, z_m]).T
trajectory_pcd = o3d.geometry.PointCloud()
trajectory_pcd.points = o3d.utility.Vector3dVector(trajectory_points)
trajectory_pcd.paint_uniform_color([1, 0, 0])  # red

lines = [[i, i+1] for i in range(len(trajectory_points)-1)]
colors = [[1, 0, 0] for _ in lines]
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(trajectory_points)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector(colors)

# Start and goal points
start_pcd = o3d.geometry.PointCloud()
start_pcd.points = o3d.utility.Vector3dVector([trajectory_points[0]])
start_pcd.paint_uniform_color([0, 1, 0])  # green

goal_threshold_pcd = o3d.geometry.PointCloud()
goal_threshold_pcd.points = o3d.utility.Vector3dVector([trajectory_points[-1]])
goal_threshold_pcd.paint_uniform_color([1, 0, 0])  # red

# Coordinate frame
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

# Visualize map + trajectory
o3d.visualization.draw_geometries([pcd, trajectory_pcd, line_set, start_pcd, goal_threshold_pcd, axes],
                                  window_name="Map + Trajectory",
                                  width=1280,
                                  height=720,
                                  point_show_normal=False)

# -----------------------------
# Matplotlib plots
# -----------------------------
fig = plt.figure(figsize=(12, 8))

# 3D trajectory
ax1 = fig.add_subplot(311, projection="3d")
ax1.plot(x_m, y_m, z_m, color='blue', label="Trajectory")
ax1.scatter(x_m[0], y_m[0], z_m[0], color='green', s=50, label="Start")
ax1.scatter(x_m[-1], y_m[-1], z_m[-1], color='red', s=50, label="Goal threshold")
ax1.set_title("3D Trajectory (moving segment)")
ax1.set_xlabel("X [m]")
ax1.set_ylabel("Y [m]")
ax1.set_zlabel("Z [m]")
ax1.legend()

# Equal aspect
max_range = np.array([x_m.max()-x_m.min(),
                      y_m.max()-y_m.min(),
                      z_m.max()-z_m.min()]).max()
mid_x = (x_m.max()+x_m.min())/2
mid_y = (y_m.max()+y_m.min())/2
mid_z = (z_m.max()+z_m.min())/2
ax1.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
ax1.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
ax1.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

# Speed vs time
ax2 = fig.add_subplot(312)
ax2.plot(t_m, speed_m, color='green')
ax2.set_ylabel("Speed [m/s]")
ax2.set_title("Speed vs Time")
ax2.grid(True)

# Acceleration vs time (filtered)
ax3 = fig.add_subplot(313)
ax3.plot(t_m, acceleration_filtered, color='red', label="Filtered")
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Acceleration [m/s²]")
ax3.set_title("Acceleration vs Time")
ax3.grid(True)
ax3.legend()

plt.tight_layout()
plt.show()

# -----------------------------
# Print analytics
# -----------------------------
print("=== Moving Segment Trajectory Analysis ===")
print(f"Path length: {path_length:.4f} m")
print(f"Time to reach goal threshold: {time_to_reach:.4f} s")
print(f"Average speed: {np.mean(speed_m):.4f} m/s")
print(f"Max speed:     {np.max(speed_m):.4f} m/s")
print(f"Max acceleration: {np.max(acceleration_filtered):.4f} m/s²")

# Convert Open3D point cloud to numpy array
map_points = np.asarray(pcd.points)

# Build KD-tree for fast nearest neighbor search
pcd_tree = o3d.geometry.KDTreeFlann(pcd)

min_distances = []

for pt in trajectory_points:
    [_, idx, dists] = pcd_tree.search_knn_vector_3d(pt, 1)  # find nearest neighbor
    min_distances.append(np.sqrt(dists[0]))  # distance is squared in dists

min_distance_to_obstacle = np.min(min_distances)
print(f"Minimum distance to obstacles during mission: {min_distance_to_obstacle:.4f} m")
