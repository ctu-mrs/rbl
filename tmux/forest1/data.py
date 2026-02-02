import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import csv
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--sim", type=int, required=True, help="Simulation number")
args = parser.parse_args()

print("sim =", args.sim)

# -----------------------------
# Parameters
# -----------------------------
CSV_FILE = "rbl_odom_" + str(args.sim) + ".bag.csv"
OUTPUT_FILE = "./uav_metrics_summary" + str(args.sim) + ".csv"
DIST_THRESHOLD = 0.2   # Goal threshold [m] — only used to determine arrival, not to truncate
SPEED_THRESHOLD = 0.1  # Minimum speed to start moving [m/s]
WINDOW_LENGTH = 11
POLYORDER = 2

# -----------------------------
# Load CSV and identify UAVs
# -----------------------------
df = pd.read_csv(CSV_FILE)

if 'uav' not in df.columns:
    df['uav'] = df['field.child_frame_id'].str.extract(r'(uav\d+)')

uav_groups = {}   # will store both full and moving-segment data
metrics_list = {}

# -----------------------------
# Process each UAV
# -----------------------------
for uav, group in df.groupby('uav'):
    # Full trajectory
    x_full = group["field.pose.pose.position.x"].values
    y_full = group["field.pose.pose.position.y"].values
    z_full = group["field.pose.pose.position.z"].values
    t_full = group["%time"].values * 1e-9
    if len(t_full) == 0:
        continue
    t_full = t_full - t_full[0]  # normalize

    vx_full = group["field.twist.twist.linear.x"].values
    vy_full = group["field.twist.twist.linear.y"].values
    vz_full = group["field.twist.twist.linear.z"].values
    speed_full = np.sqrt(vx_full**2 + vy_full**2 + vz_full**2)

    # Determine moving segment indices (start moving to goal arrival)
    moving_idx_candidates = np.where(speed_full > SPEED_THRESHOLD)[0]
    idx_start = moving_idx_candidates[0] if len(moving_idx_candidates) > 0 else 0

    goal = np.array([x_full[-1], y_full[-1], z_full[-1]])
    dist_to_goal_full = np.sqrt((x_full - goal[0])**2 + (y_full - goal[1])**2 + (z_full - goal[2])**2)
    idx_end_candidates = np.where(dist_to_goal_full <= DIST_THRESHOLD)[0]
    idx_end = idx_end_candidates[0] if len(idx_end_candidates) > 0 else len(t_full)-1

    if idx_end < idx_start:
        idx_start = idx_end  # edge case

    # Moving segment (for metrics)
    x_m = x_full[idx_start:idx_end+1]
    y_m = y_full[idx_start:idx_end+1]
    z_m = z_full[idx_start:idx_end+1]
    t_m = t_full[idx_start:idx_end+1]
    speed_m = speed_full[idx_start:idx_end+1]

    # Remove duplicate timestamps
    if len(t_m) > 0:
        _, unique_idx = np.unique(t_m, return_index=True)
        x_m = x_m[unique_idx]
        y_m = y_m[unique_idx]
        z_m = z_m[unique_idx]
        t_m = t_m[unique_idx]
        speed_m = speed_m[unique_idx]

    # -----------------------------
    # Acceleration (moving segment)
    # -----------------------------
    if len(t_m) >= 2:
        dt = np.diff(t_m)
        dv = np.diff(speed_m)
        acceleration = np.concatenate(([0], np.divide(dv, dt, out=np.zeros_like(dv), where=dt!=0)))
    else:
        acceleration = np.array([0.0])

    if len(acceleration) >= WINDOW_LENGTH:
        acceleration_filtered = savgol_filter(acceleration, WINDOW_LENGTH, POLYORDER)
    else:
        acceleration_filtered = acceleration

    # -----------------------------
    # Path length and time (moving segment)
    # -----------------------------
    if len(x_m) >= 2:
        diffs = np.sqrt(np.diff(x_m)**2 + np.diff(y_m)**2 + np.diff(z_m)**2)
        path_length = float(np.sum(diffs))
        time_to_reach = float(t_m[-1] - t_m[0])
        avg_speed = float(np.mean(speed_m)) if len(speed_m) > 0 else 0.0
        max_speed = float(np.max(speed_m)) if len(speed_m) > 0 else 0.0
        max_accel = float(np.max(acceleration_filtered)) if len(acceleration_filtered) > 0 else 0.0
    else:
        path_length = 0.0
        time_to_reach = 0.0
        avg_speed = 0.0
        max_speed = float(np.max(speed_m)) if len(speed_m) > 0 else 0.0
        max_accel = float(np.max(acceleration_filtered)) if len(acceleration_filtered) > 0 else 0.0

    # Store both full and moving data
    uav_groups[uav] = {
        "t_full": t_full,
        "x_full": x_full,
        "y_full": y_full,
        "z_full": z_full,
        "speed_full": speed_full,
        "t": t_m,
        "x": x_m,
        "y": y_m,
        "z": z_m,
        "speed": speed_m,
        "acceleration": acceleration_filtered
    }

    # Store metrics (moving segment)
    metrics_list[uav] = {
        "PathLength_m": path_length,
        "TimeToReach_s": time_to_reach,
        "AvgSpeed_mps": avg_speed,
        "MaxSpeed_mps": max_speed,
        "MaxAcceleration_mps2": max_accel
    }

# -----------------------------
# Compute inter-UAV distances (FULL trajectories)
# -----------------------------
all_times = np.unique(np.concatenate([v["t_full"] for v in uav_groups.values()]))
uav_list = list(uav_groups.keys())
num_uav = len(uav_list)
num_t = len(all_times)

positions = np.zeros((num_t, num_uav, 3))
for ui, uav in enumerate(uav_list):
    data = uav_groups[uav]
    t = data["t_full"]
    x = data["x_full"]
    y = data["y_full"]
    z = data["z_full"]

    # Ensure monotonic timestamps
    if np.any(np.diff(t) < 0):
        order = np.argsort(t)
        t = t[order]
        x = x[order]
        y = y[order]
        z = z[order]

    positions[:, ui, 0] = np.interp(all_times, t, x)
    positions[:, ui, 1] = np.interp(all_times, t, y)
    positions[:, ui, 2] = np.interp(all_times, t, z)

# Pairwise distances
dist_matrix = np.linalg.norm(positions[:, :, None, :] - positions[:, None, :, :], axis=-1)

# Set self-distances to infinity
dist_matrix[:, np.arange(num_uav), np.arange(num_uav)] = np.inf

# Min distance per UAV over time
min_dist_per_uav = dist_matrix.min(axis=2)
inter_uav_distances = {uav_list[ui]: min_dist_per_uav[:, ui] for ui in range(num_uav)}

# Store min distance in metrics
for ui, uav in enumerate(uav_list):
    min_val = float(np.nanmin(min_dist_per_uav[:, ui])) if min_dist_per_uav.size > 0 else float('nan')
    if np.isinf(min_val):
        min_val = float('nan')
    metrics_list[uav]["MinDistanceToOtherUAV_m"] = min_val

# -----------------------------
# Save metrics to CSV
# -----------------------------
with open(OUTPUT_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "UAV",
        "PathLength_m",
        "TimeToReach_s",
        "AvgSpeed_mps",
        "MaxSpeed_mps",
        "MaxAcceleration_mps2",
        "MinDistanceToOtherUAV_m"
    ])
    for uav, m in metrics_list.items():
        writer.writerow([
            uav,
            m["PathLength_m"],
            m["TimeToReach_s"],
            m["AvgSpeed_mps"],
            m["MaxSpeed_mps"],
            m["MaxAcceleration_mps2"],
            m["MinDistanceToOtherUAV_m"]
        ])

print(f"Metrics successfully saved to '{OUTPUT_FILE}'")

# -----------------------------
# Plot results
# -----------------------------
plt.figure(figsize=(15,10))

# 3D Trajectory
ax1 = plt.subplot(3,1,1, projection='3d')
for uav, data in uav_groups.items():
    ax1.plot(data["x_full"], data["y_full"], data["z_full"], label=uav)
ax1.set_xlabel("X [m]")
ax1.set_ylabel("Y [m]")
ax1.set_zlabel("Z [m]")
ax1.set_title("UAV Trajectories (full)")

x_limits = ax1.get_xlim3d()
y_limits = ax1.get_ylim3d()
z_limits = ax1.get_zlim3d()
max_range = np.array([x_limits[1]-x_limits[0], y_limits[1]-y_limits[0], z_limits[1]-z_limits[0]]).max() / 2.0
mid_x = np.mean(x_limits)
mid_y = np.mean(y_limits)
mid_z = np.mean(z_limits)
ax1.set_xlim3d(mid_x - max_range, mid_x + max_range)
ax1.set_ylim3d(mid_y - max_range, mid_y + max_range)
ax1.set_zlim3d(mid_z - max_range, mid_z + max_range)

# Speed & Acceleration (moving segment)
ax2 = plt.subplot(3,1,2)
for uav, data in uav_groups.items():
    if len(data["t"]) > 0:
        ax2.plot(data["t"], data["speed"], label=f"{uav} speed")
        ax2.plot(data["t"], data["acceleration"], '--', label=f"{uav} accel")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Speed [m/s] / Acceleration [m/s²]")
ax2.set_title("Speed & Acceleration over Time (moving segment)")
ax2.grid(True)

# Inter-UAV distance
ax3 = plt.subplot(3,1,3)
for uav, dist_list in inter_uav_distances.items():
    ax3.plot(all_times, dist_list, label=uav)
ax3.axhline(y=0.4, linestyle='--', label='Threshold 1 m')
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Distance to Closest UAV [m]")
ax3.set_title("Inter-UAV Distance over Time (full trajectories)")
ax3.grid(True)

plt.tight_layout()

import numpy as np

# -----------------------------
# Compute inter-UAV distances (FULL trajectories)
# -----------------------------

# Collect global time vector
all_times = np.unique(np.concatenate([v["t_full"] for v in uav_groups.values()]))

uav_list = list(uav_groups.keys())
num_uav = len(uav_list)
num_t = len(all_times)

# -----------------------------
# Interpolate UAV positions to common timeline
# -----------------------------
positions = np.zeros((num_t, num_uav, 3))

for ui, uav in enumerate(uav_list):
    data = uav_groups[uav]
    t = np.asarray(data["t_full"])
    x = np.asarray(data["x_full"])
    y = np.asarray(data["y_full"])
    z = np.asarray(data["z_full"])

    # Ensure monotonic timestamps
    if np.any(np.diff(t) < 0):
        order = np.argsort(t)
        t = t[order]
        x = x[order]
        y = y[order]
        z = z[order]

    positions[:, ui, 0] = np.interp(all_times, t, x)
    positions[:, ui, 1] = np.interp(all_times, t, y)
    positions[:, ui, 2] = np.interp(all_times, t, z)

# -----------------------------
# Compute pairwise distance matrix
# dist_matrix[t, i, j] = distance between UAV i and j at time t
# -----------------------------
dist_matrix = np.linalg.norm(
    positions[:, :, None, :] - positions[:, None, :, :],
    axis=-1
)

# Set self-distances to infinity
dist_matrix[:, np.arange(num_uav), np.arange(num_uav)] = np.inf

# -----------------------------
# Store FULL pairwise distance time series
# -----------------------------
inter_uav_distances_all = {}

for i in range(num_uav):
    for j in range(i + 1, num_uav):
        uav_i = uav_list[i]
        uav_j = uav_list[j]
        inter_uav_distances_all[(uav_i, uav_j)] = dist_matrix[:, i, j]

# -----------------------------
# Minimum distance per UAV over time
# -----------------------------
min_dist_per_uav_time = dist_matrix.min(axis=2)

inter_uav_distances = {
    uav_list[i]: min_dist_per_uav_time[:, i]
    for i in range(num_uav)
}

# -----------------------------
# Store minimum distance per UAV
# -----------------------------
for i, uav in enumerate(uav_list):
    min_val = np.nanmin(min_dist_per_uav_time[:, i]) if num_t > 0 else np.nan
    if np.isinf(min_val):
        min_val = np.nan
    metrics_list[uav]["MinDistanceToOtherUAV_m"] = float(min_val)

# -----------------------------
# Store minimum distance per UAV pair
# -----------------------------
min_dist_per_pair = {}

for i in range(num_uav):
    for j in range(i + 1, num_uav):
        uav_i = uav_list[i]
        uav_j = uav_list[j]
        min_dist_per_pair[(uav_i, uav_j)] = float(
            np.nanmin(dist_matrix[:, i, j])
        )

# -----------------------------
# OPTIONAL: Store per-UAV distances to all other UAVs
# -----------------------------
for i, uav in enumerate(uav_list):
    metrics_list[uav]["DistancesToOtherUAVs_m"] = {}
    for j, other_uav in enumerate(uav_list):
        if i == j:
            continue
        metrics_list[uav]["DistancesToOtherUAVs_m"][other_uav] = dist_matrix[:, i, j]


plt.figure(figsize=(10, 6))

for (uav_i, uav_j), dist_ts in inter_uav_distances_all.items():
    plt.plot(all_times, dist_ts, label=f"{uav_i}–{uav_j}")

plt.xlabel("Time")
plt.ylabel("Distance [m]")
plt.title("Inter-UAV Distances Over Time")
plt.grid(True)
plt.tight_layout()
# plt.show()

