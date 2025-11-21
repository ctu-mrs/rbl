
import re
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

FILENAME = "raw.txt"

with open(FILENAME, "r") as f:
    text = f.read()

# Extract width and point_step from header
width_match = re.search(r"width:\s*(\d+)", text)
point_step_match = re.search(r"point_step:\s*(\d+)", text)

if not width_match or not point_step_match:
    raise ValueError("Cannot find width or point_step in raw.txt")

width = int(width_match.group(1))
point_step = int(point_step_match.group(1))

# Extract data array: numbers inside data: [ ... ]
data_match = re.search(r"data:\s*\[([0-9,\s]+)\]", text, re.DOTALL)
if not data_match:
    raise ValueError("Cannot find data field in raw.txt")

data_str = data_match.group(1)
# Convert string to numpy array of uint8
data_bytes = np.array([int(x) for x in data_str.split(",")], dtype=np.uint8)

# Each point uses point_step bytes
num_points = width
points = []

for i in range(num_points):
    offset = i * point_step
    # x,y,z are float32 at offsets 0,4,8
    x = np.frombuffer(data_bytes[offset+0:offset+4], dtype=np.float32)[0]
    y = np.frombuffer(data_bytes[offset+4:offset+8], dtype=np.float32)[0]
    z = np.frombuffer(data_bytes[offset+8:offset+12], dtype=np.float32)[0]
    points.append([x, y, z])

points = np.array(points)
print(f"Decoded {points.shape[0]} points")

# Example: points is Nx3 array from your previous decoding
# points = ...


# # Plot
# fig = plt.figure(figsize=(10,7))
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(points[:,0], points[:,1], points[:,2], s=1)
# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# ax.set_zlabel("Z")
# ax.set_title("PointCloud2 from raw.txt")

# Create Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Optional: voxel downsample to match resolution 0.05 m
resolution = 0.05
pcd = pcd.voxel_down_sample(voxel_size=resolution)

# Optionally colorize points
pcd.paint_uniform_color([0.1, 0.6, 0.9])  # light blue

# Visualize
o3d.visualization.draw_geometries([pcd],
                                  window_name="PointCloud2",
                                  width=1280,
                                  height=720,
                                  point_show_normal=False)
