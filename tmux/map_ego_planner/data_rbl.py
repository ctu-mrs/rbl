
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from mpl_toolkits.mplot3d import Axes3D

# -----------------------------
# Parameters
# -----------------------------
CSV_FILE = "2025-11-21-11-01-38.bag.csv"
DIST_THRESHOLD = 0.20    # meters to goal
SPEED_THRESHOLD = 0.01   # minimum speed to start plotting
WINDOW_LENGTH = 11       # Savitzky-Golay filter (must be odd)
POLYORDER = 2
# -----------------------------

# Load CSV
df = pd.read_csv(CSV_FILE)

# Extract positions and time
x = df["field.pose.pose.position.x"].values
y = df["field.pose.pose.position.y"].values
z = df["field.pose.pose.position.z"].values
t = df["%time"].values * 1e-9
t = t - t[0]

# Extract twist velocities
vx = df["field.twist.twist.linear.x"].values
vy = df["field.twist.twist.linear.y"].values
vz = df["field.twist.twist.linear.z"].values

# Compute speed from twist
speed = np.sqrt(vx**2 + vy**2 + vz**2)

# -----------------------------
# Find moving segment
# -----------------------------
idx_start = np.where(speed > SPEED_THRESHOLD)[0][0]

goal = np.array([x[-1], y[-1], z[-1]])
dist = np.sqrt((x - goal[0])**2 + (y - goal[1])**2 + (z - goal[2])**2)
idx_end_candidates = np.where(dist <= DIST_THRESHOLD)[0]
idx_end = idx_end_candidates[0] if len(idx_end_candidates) > 0 else len(t)-1

# Slice moving segment
x_m = x[idx_start:idx_end+1]
y_m = y[idx_start:idx_end+1]
z_m = z[idx_start:idx_end+1]
t_m = t[idx_start:idx_end+1]
speed_m = speed[idx_start:idx_end+1]

# -----------------------------
# Remove duplicate timestamps
# -----------------------------
_, unique_idx = np.unique(t_m, return_index=True)
t_m = t_m[unique_idx]
speed_m = speed_m[unique_idx]
x_m = x_m[unique_idx]
y_m = y_m[unique_idx]
z_m = z_m[unique_idx]

# -----------------------------
# Compute acceleration safely
# -----------------------------
dt = np.diff(t_m)
dv = np.diff(speed_m)
# Avoid division by zero
acceleration_m = np.concatenate(([0], np.divide(dv, dt, out=np.zeros_like(dv), where=dt!=0)))

# Apply Savitzky-Golay filter
if len(acceleration_m) >= WINDOW_LENGTH:
    acceleration_filtered = savgol_filter(acceleration_m, WINDOW_LENGTH, POLYORDER)
else:
    acceleration_filtered = acceleration_m

# -----------------------------
# Compute path length
# -----------------------------
diffs = np.sqrt(np.diff(x_m)**2 + np.diff(y_m)**2 + np.diff(z_m)**2)
path_length = np.sum(diffs)
time_to_reach = t_m[-1] - t_m[0]

# -----------------------------
# Plotting
# -----------------------------
fig = plt.figure(figsize=(12, 8))

# 3D trajectory
ax1 = fig.add_subplot(311, projection="3d")
ax1.plot(x_m, y_m, z_m, label="Trajectory", color='blue')
ax1.scatter(x_m[0], y_m[0], z_m[0], color='green', s=50, label="Start")
ax1.scatter(x_m[-1], y_m[-1], z_m[-1], color='red', s=50, label="Goal threshold")
ax1.set_title("3D Trajectory (axis equal, moving segment)")
ax1.set_xlabel("X [m]")
ax1.set_ylabel("Y [m]")
ax1.set_zlabel("Z [m]")
ax1.legend()

# Equal aspect ratio
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
ax3.set_title("Acceleration vs Time (filtered)")
ax3.grid(True)
ax3.legend()

plt.tight_layout()
# plt.show()

# -----------------------------
# Print results
# -----------------------------
print("=== Moving Segment Trajectory Analysis ===")
print(f"Path length: {path_length:.4f} m")
print(f"Time to reach goal threshold: {time_to_reach:.4f} s")
print(f"Average speed: {np.mean(speed_m):.4f} m/s")
print(f"Max speed:     {np.max(speed_m):.4f} m/s")
print(f"Max acceleration: {np.max(acceleration_filtered):.4f} m/s²")
